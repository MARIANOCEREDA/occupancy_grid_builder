#include "yolo_onnx_inference/yolo_onnx_inference.hpp"

int main(int argc, char** argv)
{
  std::string model_path = "/home/dev/ws/src/segmentation_inference_onnx/models/yolo11n-seg.onnx";
  yolo_onnx_inference::DeviceType device_type = yolo_onnx_inference::DeviceType::CPU;

  yolo_onnx_inference::YoloONNXInference detector(model_path, device_type);

  cv::Mat input_image =
      cv::imread("/home/dev/ws/src/segmentation_inference_onnx/images/test_image.jpeg");
  if (input_image.empty())
  {
    std::cerr << "Failed to read input image." << std::endl;
    return -1;
  }

  std::vector<yolo_onnx_inference::BoundingBox> detections =
      detector.infer_segmentation(input_image);
  std::cout << "Detected " << detections.size() << " objects." << std::endl;

  for (const auto& box : detections)
  {
    std::cout << "Class ID: " << box.class_id << ", Confidence: " << box.confidence
              << ", BBox: [" << box.x << ", " << box.y << ", " << box.width << ", "
              << box.height << "]" << std::endl;
  }

  // Build a colour overlay: one fixed colour per class_id, blended onto the image.
  static const std::vector<cv::Vec3b> class_colors = {
    {128,   0,   0}, {  0, 128,   0}, {  0,   0, 128}, {128, 128,   0},
    {  0, 128, 128}, {128,   0, 128}, {255,   0,   0}, {  0, 255,   0},
    {  0,   0, 255}, {255, 255,   0}, {  0, 255, 255}, {255,   0, 255},
    {255, 128,   0}, {  0, 128, 255}, {128, 255,   0}, {255,   0, 128},
  };

  cv::Mat color_layer = cv::Mat::zeros(input_image.size(), CV_8UC3);
  for (const auto& box : detections)
  {
    if (box.mask.empty()) continue;

    const cv::Vec3b color = class_colors[box.class_id % class_colors.size()];
    int x = std::max(0, static_cast<int>(box.x));
    int y = std::max(0, static_cast<int>(box.y));
    int w = std::min(static_cast<int>(box.width),  color_layer.cols - x);
    int h = std::min(static_cast<int>(box.height), color_layer.rows - y);
    if (w <= 0 || h <= 0) continue;

    cv::Mat src = box.mask;
    cv::Mat src_resized;
    if (src.rows != h || src.cols != w)
      cv::resize(src, src_resized, cv::Size(w, h), 0, 0, cv::INTER_NEAREST);
    else
      src_resized = src;

    cv::Mat roi = color_layer(cv::Rect(x, y, w, h));
    for (int r = 0; r < h; ++r)
      for (int c = 0; c < w; ++c)
        if (src_resized.at<float>(r, c) > 0.5f)
          roi.at<cv::Vec3b>(r, c) = color;
  }

  cv::Mat result;
  cv::addWeighted(input_image, 0.55, color_layer, 0.45, 0, result);

  // Draw bounding boxes on top of the blend.
  for (const auto& box : detections)
    cv::rectangle(result, box.bbox, cv::Scalar(0, 255, 0), 2);

  cv::imshow("Detections", result);
  cv::waitKey(0);

  return 0;
}