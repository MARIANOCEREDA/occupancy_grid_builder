#include "yolo_onnx_inference/postprocess.hpp"

#include <algorithm>
#include <stdexcept>

namespace yolo_onnx_inference
{
namespace postprocess
{

cv::Mat get_boxes_from_tensor(std::vector<Ort::Value>& inference_output_tensor)
{
  std::vector<int64_t> output_shape =
      inference_output_tensor[0].GetTensorTypeAndShapeInfo().GetShape();

  if (output_shape.size() != 3)
  {
    throw std::runtime_error("Unexpected YOLO output tensor shape (expected 3 dims)");
  }

  float* output_data = inference_output_tensor[0].GetTensorMutableData<float>();
  return cv::Mat(
      static_cast<int>(output_shape[1]), static_cast<int>(output_shape[2]), CV_32F, output_data);
}

cv::Mat get_masks_from_tensor(std::vector<Ort::Value>& inference_output_tensor,
                              int& proto_height,
                              int& proto_width)
{
  if (inference_output_tensor.size() < 2)
  {
    throw std::runtime_error("Expected at least 2 output tensors for YOLO segmentation");
  }

  std::vector<int64_t> mask_shape =
      inference_output_tensor[1].GetTensorTypeAndShapeInfo().GetShape();

  if (mask_shape.size() != 4)
  {
    throw std::runtime_error("Unexpected YOLO output tensor shape (expected 4 dims)");
  }

  // Extract correct proto dimensions: [batch, channels, height, width]
  proto_height = static_cast<int>(mask_shape[2]);
  proto_width = static_cast<int>(mask_shape[3]);

  float* output_data = inference_output_tensor[1].GetTensorMutableData<float>();
  return cv::Mat(static_cast<int>(mask_shape[1]),
                 static_cast<int>(mask_shape[2] * mask_shape[3]),
                 CV_32F,
                 output_data);
}

std::vector<BoundingBox> parse_boxes(const cv::Mat& boxes,
                                     const cv::Size& original_image_size,
                                     int num_classes)
{
  // Match both detection (4+C, 5+C) and segmentation (4+C+32, 5+C+32) attribute widths.
  auto matches_attrs = [&](int n) {
    return n == (num_classes + 4) || n == (num_classes + 5) ||
           n == (num_classes + 4 + 32) || n == (num_classes + 5 + 32);
  };
  const bool rows_match_attrs = matches_attrs(boxes.rows);
  const bool cols_match_attrs = matches_attrs(boxes.cols);

  bool boxes_are_rows = true;
  if (cols_match_attrs)
  {
    boxes_are_rows = true;
  }
  else if (rows_match_attrs)
  {
    boxes_are_rows = false;
  }
  else
  {
    boxes_are_rows = boxes.rows >= boxes.cols;
  }

  const int boxes_count = boxes_are_rows ? boxes.rows : boxes.cols;
  const int attrs_count = boxes_are_rows ? boxes.cols : boxes.rows;
  const bool has_objectness = (attrs_count == (num_classes + 5));
  const int class_start_col = has_objectness ? 5 : 4;

  auto at = [&boxes, boxes_are_rows](int box_idx, int attr_idx) -> float
  {
    if (boxes_are_rows)
    {
      return boxes.at<float>(box_idx, attr_idx);
    }
    return boxes.at<float>(attr_idx, box_idx);
  };

  int num_mask_coeffs = 32;
  int data_width = num_classes + num_mask_coeffs + 4;

  std::vector<BoundingBox> decoded_boxes;
  decoded_boxes.reserve(boxes_count);

  for (int i = 0; i < boxes_count; ++i)
  {
    const float center_x = at(i, 0);
    const float center_y = at(i, 1);
    const float width = at(i, 2);
    const float height = at(i, 3);

    if (width <= 0.0f || height <= 0.0f)
    {
      continue;
    }

    float max_class_confidence = -1.0f;
    int class_id = 0;
    // Only scan the class score slots — stop before mask coefficients.
    const int class_end = class_start_col + num_classes;
    for (int c = class_start_col; c < class_end; ++c)
    {
      const float score = at(i, c);
      if (score > max_class_confidence)
      {
        max_class_confidence = score;
        class_id = c - class_start_col;
      }
    }

    if (max_class_confidence < 0.0f)
    {
      continue;
    }

    const float objectness = has_objectness ? at(i, 4) : 1.0f;
    const float confidence = max_class_confidence * objectness;

    const float left = center_x - (width * 0.5f);
    const float top = center_y - (height * 0.5f);
    const float right = center_x + (width * 0.5f);
    const float bottom = center_y + (height * 0.5f);

    const int x1 = std::clamp(static_cast<int>(left), 0, original_image_size.width - 1);
    const int y1 = std::clamp(static_cast<int>(top), 0, original_image_size.height - 1);
    const int x2 = std::clamp(static_cast<int>(right), 0, original_image_size.width - 1);
    const int y2 = std::clamp(static_cast<int>(bottom), 0, original_image_size.height - 1);

    const int box_width = x2 - x1;
    const int box_height = y2 - y1;
    if (box_width <= 0 || box_height <= 0)
    {
      continue;
    }

    const int mask_coeff_start = class_start_col + num_classes;
    std::vector<float> mask_coefficients;
    mask_coefficients.reserve(num_mask_coeffs);

    for (int mask_idx = 0; mask_idx < num_mask_coeffs; ++mask_idx)
    {
      mask_coefficients.push_back(at(i, mask_coeff_start + mask_idx));
    }

    BoundingBox decoded_box(static_cast<float>(x1),
                            static_cast<float>(y1),
                            static_cast<float>(box_width),
                            static_cast<float>(box_height),
                            confidence,
                            class_id);
    decoded_box.bbox = cv::Rect(x1, y1, box_width, box_height);
    decoded_box.mask_coefficients = mask_coefficients;
    decoded_boxes.emplace_back(decoded_box);
  }

  return decoded_boxes;
}

cv::Mat process_yolo_masks(const cv::Mat& raw_proto_masks,
                           const std::vector<BoundingBox>& boxes,
                           const cv::Size& original_image_size,
                           int proto_height,
                           int proto_width)
{
  const int num_masks = static_cast<int>(boxes.size());
  if (num_masks == 0)
  {
    return cv::Mat();
  }

  // num_proto_channels is the number of rows in the proto mask cv::Mat
  const int num_proto_channels = raw_proto_masks.rows;

  cv::Mat masks(num_masks, proto_height * proto_width, CV_32F);

  for (int i = 0; i < num_masks; ++i)
  {
    const auto& box = boxes[i];
    const std::vector<float>& mask_coeffs = box.mask_coefficients;

    cv::Mat mask_flat = cv::Mat::zeros(proto_height * proto_width, 1, CV_32F);
    for (int j = 0; j < num_proto_channels; ++j)
    {
      cv::Mat proto_channel(proto_height,
                            proto_width,
                            CV_32F,
                            raw_proto_masks.data + j * proto_height * proto_width * sizeof(float));
      mask_flat += mask_coeffs[j] * proto_channel.reshape(1, proto_height * proto_width);
    }

    masks.row(i) = mask_flat.t();
  }

  return masks;
}

void process_box_masks(std::vector<BoundingBox>& boxes,
                       const cv::Mat& proto_masks,
                       int proto_height,
                       int proto_width,
                       const cv::Size& model_input_size)
{
  if (boxes.empty() || proto_masks.empty())
  {
    return;
  }

  for (size_t i = 0; i < boxes.size(); ++i)
  {
    auto& box = boxes[i];
    
    // Get the mask row index (stored during parse or NMS tracking)
    if (i >= static_cast<size_t>(proto_masks.rows))
    {
      continue;
    }

    // Extract and reshape mask from flat row to 2D
    cv::Mat mask_flat = proto_masks.row(i);
    cv::Mat mask_proto = mask_flat.reshape(1, proto_height);
    
    // Apply sigmoid activation
    cv::Mat mask_sigmoid;
    cv::exp(-mask_proto, mask_sigmoid);
    mask_sigmoid = 1.0 / (1.0 + mask_sigmoid);
    
    // Resize from proto size (e.g., 160x160) to model input size (e.g., 640x640)
    cv::Mat mask_resized;
    cv::resize(mask_sigmoid, mask_resized, model_input_size, 0, 0, cv::INTER_LINEAR);
    
    // Crop mask to bounding box region (in model input space)
    int x1 = std::max(0, static_cast<int>(box.x));
    int y1 = std::max(0, static_cast<int>(box.y));
    int x2 = std::min(model_input_size.width - 1, static_cast<int>(box.x + box.width));
    int y2 = std::min(model_input_size.height - 1, static_cast<int>(box.y + box.height));
    
    int crop_width = std::max(1, x2 - x1);
    int crop_height = std::max(1, y2 - y1);
    
    // Ensure box dimensions are at least 1 pixel when casting
    int box_width_int = std::max(1, static_cast<int>(std::round(box.width)));
    int box_height_int = std::max(1, static_cast<int>(std::round(box.height)));
    
    if (crop_width > 0 && crop_height > 0 && box_width_int > 0 && box_height_int > 0)
    {
      cv::Rect crop_region(x1, y1, crop_width, crop_height);
      cv::Mat mask_cropped = mask_resized(crop_region);
      
      // Resize to bounding box size
      cv::Mat mask_box;
      cv::resize(mask_cropped, mask_box, cv::Size(box_width_int, box_height_int), 
                 0, 0, cv::INTER_LINEAR);
      
      // Apply threshold to get binary mask
      cv::threshold(mask_box, box.mask, 0.5, 1.0, cv::THRESH_BINARY);
    }
  }
}

std::vector<BoundingBox> filter_boxes_by_confidence(const std::vector<BoundingBox>& boxes,
                                                    float confidence_threshold)
{
  std::vector<BoundingBox> filtered_boxes;
  filtered_boxes.reserve(boxes.size());
  for (const auto& box : boxes)
  {
    if (box.confidence >= confidence_threshold)
    {
      filtered_boxes.emplace_back(box);
    }
  }
  return filtered_boxes;
}

std::vector<BoundingBox> non_max_suppression(const std::vector<BoundingBox>& boxes,
                                             float iou_threshold)
{
  std::vector<cv::Rect> output_boxes;
  output_boxes.reserve(boxes.size());
  std::vector<float> confidences;
  confidences.reserve(boxes.size());
  for (const auto& box : boxes)
  {
    output_boxes.emplace_back(box.bbox);
    confidences.emplace_back(box.confidence);
  }

  std::vector<int> filtered_idxs;
  cv::dnn::NMSBoxes(output_boxes, confidences, 0.0f, iou_threshold, filtered_idxs);

  std::vector<BoundingBox> filtered_boxes;
  filtered_boxes.reserve(filtered_idxs.size());
  for (const int idx : filtered_idxs)
  {
    filtered_boxes.emplace_back(boxes[idx]);
  }
  return filtered_boxes;
}

std::vector<int> non_max_suppression_with_indices(const std::vector<BoundingBox>& boxes,
                                                   float iou_threshold)
{
  std::vector<cv::Rect> output_boxes;
  output_boxes.reserve(boxes.size());
  std::vector<float> confidences;
  confidences.reserve(boxes.size());
  for (const auto& box : boxes)
  {
    output_boxes.emplace_back(box.bbox);
    confidences.emplace_back(box.confidence);
  }

  std::vector<int> filtered_idxs;
  cv::dnn::NMSBoxes(output_boxes, confidences, 0.0f, iou_threshold, filtered_idxs);
  return filtered_idxs;
}

}  // namespace postprocess
}  // namespace yolo_onnx_inference
