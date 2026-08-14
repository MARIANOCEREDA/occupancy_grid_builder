#include "yolo_onnx_inference/yolo_onnx_inference.hpp"

#include "yolo_onnx_inference/postprocess.hpp"
#include "yolo_onnx_inference/preprocess.hpp"

namespace yolo_onnx_inference
{

YoloONNXInference::YoloONNXInference(const std::string& model_path, DeviceType device_type)
  : model_path_(model_path),
    device_type_(device_type),
    env_(ORT_LOGGING_LEVEL_WARNING, "YOLOOnnxDetector")
{
  session_ = load_model();
  initialize_model_io();
  this->output_image_ = cv::Mat(input_size.height, input_size.width, CV_32FC3);
  this->output_image_blob_ = cv::Mat(input_size.batch, input_size.channels, CV_32FC1);
};

YoloONNXInference::~YoloONNXInference()
{
  if (cuda_provider_)
  {
    Ort::GetApi().ReleaseCUDAProviderOptions(cuda_provider_);
  }
}

Ort::SessionOptions YoloONNXInference::create_session_options()
{
  Ort::SessionOptions session_options;
  session_options.SetIntraOpNumThreads(1);
  session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
  if (device_type_ == DeviceType::GPU)
  {
    Ort::GetApi().CreateCUDAProviderOptions(&cuda_provider_);
    /**
     * NOTE:
     * 1. ORT (ONNX Runtime) knows which matrix operations should be performed, but it does not know
     * how to perform them on the GPU.
     * 2. The CUDA provider options tell ORT how to perform these operations on the GPU.
     * 3. For instance, the CUDA execution provider know how to map matrix operations to CUDA
     * kernels, manage GPU memory, and optimize data transfer between CPU and GPU.
     * Check the tutorial: https://youtu.be/Wp5PaRpudlk?t=181
     */
    session_options.AppendExecutionProvider_CUDA_V2(*cuda_provider_);
  }

  return session_options;
}

Ort::Session YoloONNXInference::load_model()
{
  this->device_type_ = device_type_;
  session_options_ = create_session_options();
  return Ort::Session(env_, model_path_.c_str(), session_options_);
}

void YoloONNXInference::initialize_model_io()
{
  Ort::AllocatorWithDefaultOptions allocator;
  input_name_ = session_.GetInputNameAllocated(0, allocator).get();
  input_names_[0] = input_name_.c_str();

  for (int i = 0; i < N_OUTPUTS_SEG; ++i)
  {
    output_name_storage_[i] = session_.GetOutputNameAllocated(i, allocator).get();
    output_names_[i] = output_name_storage_[i].c_str();
  }
}

void YoloONNXInference::preprocess(const cv::Mat& input_image)
{
  if (!preprocess::letterbox_image(
          input_image, output_image_, cv::Size(input_size.width, input_size.height), 114))
  {
    return;
  }

  if (!this->output_image_.empty())
  {
    preprocess::convert_bgr_to_rgb(output_image_);
  }

  if (!preprocess::from_image_to_blob(
          output_image_, output_image_blob_, cv::Size(input_size.width, input_size.height)))
  {
    return;
  }

  blob_to_onnx_tensor(output_image_blob_);
}

std::vector<BoundingBox> YoloONNXInference::infer_segmentation(const cv::Mat& input_image)
{
  if (input_image.empty() || input_image.type() != CV_8UC3)
  {
    throw std::invalid_argument("Expected a non-empty BGR8 image for inference");
  }

  last_input_image_size_ = input_image.size();
  preprocess(input_image);

  std::vector<Ort::Value> output = session_.Run(Ort::RunOptions{nullptr},
                                                input_names_.data(),
                                                &input_tensor_,
                                                1,
                                                output_names_.data(),
                                                N_OUTPUTS_SEG);

  auto final_boxes = postprocess(output);

  return final_boxes;
}

void YoloONNXInference::blob_to_onnx_tensor(const cv::Mat& blob)
{
  // Convert the OpenCV blob to an ONNX Runtime tensor
  std::vector<int64_t> input_shape = {
      input_size.batch, input_size.channels, input_size.height, input_size.width};
  this->input_tensor_ = Ort::Value::CreateTensor<float>(
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault),
      reinterpret_cast<float*>(blob.data),
      blob.total(),
      input_shape.data(),
      input_shape.size());
}

void YoloONNXInference::remove_letterbox_offset(std::vector<BoundingBox>& boxes) const
{
  const float scale_x =
      static_cast<float>(input_size.width) / static_cast<float>(last_input_image_size_.width);
  const float scale_y =
      static_cast<float>(input_size.height) / static_cast<float>(last_input_image_size_.height);
  const float scale = std::min(scale_x, scale_y);

  const float pad_x =
      (static_cast<float>(input_size.width) - last_input_image_size_.width * scale) * 0.5f;
  const float pad_y =
      (static_cast<float>(input_size.height) - last_input_image_size_.height * scale) * 0.5f;

  const float max_x = static_cast<float>(last_input_image_size_.width - 1);
  const float max_y = static_cast<float>(last_input_image_size_.height - 1);

  for (auto& box : boxes)
  {
    // Store old box dimensions for mask resizing
    const float old_width = box.width;
    const float old_height = box.height;

    const float x1 = std::clamp((box.x - pad_x) / scale, 0.0f, max_x);
    const float y1 = std::clamp((box.y - pad_y) / scale, 0.0f, max_y);
    const float x2 = std::clamp((box.x + box.width - pad_x) / scale, 0.0f, max_x);
    const float y2 = std::clamp((box.y + box.height - pad_y) / scale, 0.0f, max_y);

    box.x = x1;
    box.y = y1;
    box.width = std::max(0.0f, x2 - x1);
    box.height = std::max(0.0f, y2 - y1);
    box.bbox = cv::Rect(static_cast<int>(box.x),
                        static_cast<int>(box.y),
                        static_cast<int>(box.width),
                        static_cast<int>(box.height));

    // Resize mask to match new bounding box dimensions in original image space
    if (!box.mask.empty())
    {
      // Ensure at least 1 pixel dimensions to avoid resize errors
      int new_width = std::max(1, static_cast<int>(std::round(box.width)));
      int new_height = std::max(1, static_cast<int>(std::round(box.height)));

      if (new_width > 0 && new_height > 0)
      {
        cv::Mat mask_resized;
        cv::resize(box.mask, mask_resized, cv::Size(new_width, new_height), 0, 0, cv::INTER_LINEAR);
        box.mask = mask_resized;
      }
      else
      {
        box.mask = cv::Mat();
      }
    }
  }
}

std::vector<BoundingBox> YoloONNXInference::postprocess(std::vector<Ort::Value>& output)
{
  auto raw_boxes = postprocess::get_boxes_from_tensor(output);

  int proto_height = 0;
  int proto_width = 0;
  auto raw_masks = postprocess::get_masks_from_tensor(output, proto_height, proto_width);

  auto bounding_boxes =
      postprocess::parse_boxes(raw_boxes, cv::Size(input_size.width, input_size.height), 80);

  auto proto_masks = postprocess::process_yolo_masks(raw_masks,
                                                      bounding_boxes,
                                                      cv::Size(input_size.width, input_size.height),
                                                      proto_height,
                                                      proto_width);

  // Filter by confidence while preserving the original index into proto_masks / bounding_boxes.
  std::vector<int> conf_indices;
  std::vector<BoundingBox> filtered_boxes;
  for (int i = 0; i < static_cast<int>(bounding_boxes.size()); ++i)
  {
    if (bounding_boxes[i].confidence >= confidence_threshold_)
    {
      conf_indices.push_back(i);
      filtered_boxes.push_back(bounding_boxes[i]);
    }
  }

  // NMS returns indices into filtered_boxes.
  auto nms_indices = postprocess::non_max_suppression_with_indices(filtered_boxes, 0.5f);

  // Build result boxes and copy the correct proto_masks row for each survivor.
  std::vector<BoundingBox> result_boxes;
  result_boxes.reserve(nms_indices.size());
  cv::Mat final_masks(static_cast<int>(nms_indices.size()), proto_masks.cols, CV_32F);
  for (size_t i = 0; i < nms_indices.size(); ++i)
  {
    result_boxes.emplace_back(filtered_boxes[nms_indices[i]]);
    // Map: nms_indices[i] → filtered_boxes index → conf_indices[...] → bounding_boxes index
    const int original_idx = conf_indices[nms_indices[i]];
    proto_masks.row(original_idx).copyTo(final_masks.row(static_cast<int>(i)));
  }

  // // Process masks for each box (crop, resize, threshold) - in model input space
  postprocess::process_box_masks(result_boxes, final_masks, proto_height, proto_width,
                                 cv::Size(input_size.width, input_size.height));

  // Remove letterbox offset from boxes AND resize masks to original image space
  remove_letterbox_offset(result_boxes);

  return result_boxes;
}

}  // namespace yolo_onnx_inference