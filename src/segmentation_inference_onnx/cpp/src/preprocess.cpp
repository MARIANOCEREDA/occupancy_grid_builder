#include "yolo_onnx_inference/preprocess.hpp"

#include <algorithm>
#include <iostream>

namespace yolo_onnx_inference
{
namespace preprocess
{

namespace
{

bool ValidateImageSize(const cv::Mat& image)
{
  if (image.empty())
  {
    std::cerr << "Input image is empty." << std::endl;
    return false;
  }

  if (!image.cols || !image.rows)
  {
    std::cerr << "Input image has invalid dimensions." << std::endl;
    return false;
  }

  return true;
}

}  // namespace

bool letterbox_image(const cv::Mat& input_image,
                    cv::Mat& output_image,
                    const cv::Size& target_size,
                    int fill_value)
{
  if (!ValidateImageSize(input_image))
  {
    return false;
  }

  const float normalized_fill = static_cast<float>(fill_value) / 255.0f;
  output_image.setTo(cv::Scalar(normalized_fill, normalized_fill, normalized_fill));

  if (input_image.size() == target_size)
  {
    input_image.convertTo(output_image, CV_32FC3, 1.0 / 255.0);
    return true;
  }

  const float relative_width = static_cast<float>(target_size.width) / input_image.cols;
  const float relative_height = static_cast<float>(target_size.height) / input_image.rows;
  const float scale = std::min(relative_width, relative_height);

  const cv::Size new_image_size(static_cast<int>(input_image.cols * scale),
                                static_cast<int>(input_image.rows * scale));

  const int padding_x = (target_size.width - new_image_size.width) / 2;
  const int padding_y = (target_size.height - new_image_size.height) / 2;

  cv::Mat resized_image;
  cv::resize(input_image, resized_image, new_image_size);

  cv::Mat output_roi =
      output_image(cv::Rect(padding_x, padding_y, new_image_size.width, new_image_size.height));
  resized_image.convertTo(output_roi, CV_32FC3, 1.0 / 255.0);
  return true;
}

void convert_bgr_to_rgb(cv::Mat& image)
{
  cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
}

bool from_image_to_blob(const cv::Mat& input_image,
                     cv::Mat& output_image_blob,
                     const cv::Size& target_size)
{
  if (input_image.empty())
  {
    std::cerr << "Input image is empty. Cannot convert to blob." << std::endl;
    return false;
  }

  output_image_blob =
      cv::dnn::blobFromImage(input_image, 1.0, target_size, cv::Scalar(), true, false, CV_32F);
  return true;
}

}  // namespace preprocess
}  // namespace yolo_onnx_inference
