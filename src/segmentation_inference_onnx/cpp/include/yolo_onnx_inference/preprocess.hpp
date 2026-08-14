#pragma once

#include <opencv2/opencv.hpp>

namespace yolo_onnx_inference
{
namespace preprocess
{

/**
 * @brief Resize and pad an image while preserving aspect ratio.
 * @param input_image Source image.
 * @param output_image Destination image buffer (preallocated target size).
 * @param target_size Desired output size.
 * @param fill_value Padding value used for empty areas.
 * @return True on success, false when input image is invalid.
 */
bool letterbox_image(const cv::Mat& input_image,
                    cv::Mat& output_image,
                    const cv::Size& target_size,
                    int fill_value = 114);

/**
 * @brief Convert an image from BGR to RGB in place.
 * @param image Image buffer to convert.
 */
void convert_bgr_to_rgb(cv::Mat& image);

/**
 * @brief Convert preprocessed image to model input blob layout (NCHW).
 * @param input_image Source image in HWC format.
 * @param output_image_blob Destination blob matrix.
 * @param target_size Model input dimensions.
 * @return True on success, false if source image is empty.
 */
bool from_image_to_blob(const cv::Mat& input_image,
                     cv::Mat& output_image_blob,
                     const cv::Size& target_size);

}  // namespace preprocess
}  // namespace yolo_onnx_inference
