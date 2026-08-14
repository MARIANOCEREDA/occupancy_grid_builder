#pragma once

#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>
#include <vector>

#include "yolo_onnx_inference/yolo_onnx_inference.hpp"

namespace yolo_onnx_inference
{
namespace postprocess
{

/**
 * @brief Wrap raw ONNX output tensor memory into an OpenCV matrix view.
 * @param inference_output_tensor Raw output tensor(s) from ONNX Runtime.
 * @return Matrix view over tensor memory without copying.
 */
cv::Mat get_boxes_from_tensor(std::vector<Ort::Value>& inference_output_tensor);

/**
 * @brief Wrap raw ONNX output tensor memory into an OpenCV matrix view.
 * @param inference_output_tensor Raw output tensor(s) from ONNX Runtime.
 * @param proto_height Output parameter for proto mask height.
 * @param proto_width Output parameter for proto mask width.
 * @return Matrix view over tensor memory without copying.
 */
cv::Mat get_masks_from_tensor(std::vector<Ort::Value>& inference_output_tensor,
                              int& proto_height,
                              int& proto_width);

cv::Mat process_yolo_masks(const cv::Mat& masks,
                           const std::vector<BoundingBox>& boxes,
                           const cv::Size& original_image_size,
                           int proto_height,
                           int proto_width);

/**
 * @brief Process individual masks for each box after NMS.
 * @param proto_masks Raw proto mask coefficients (num_masks x flattened_size).
 * @param boxes Bounding boxes (in model input space).
 * @param proto_height Height of proto mask.
 * @param proto_width Width of proto mask.
 * @param model_input_size Size of model input (e.g., 640x640).
 */
void process_box_masks(std::vector<BoundingBox>& boxes,
                       const cv::Mat& proto_masks,
                       int proto_height,
                       int proto_width,
                       const cv::Size& model_input_size);

/**
 * @brief Parse raw model boxes into image-space detections.
 * @param boxes Output matrix from GetBoxesFromTensor.
 * @param original_image_size Image size used for coordinate clamping.
 * @param num_classes Number of classes expected in model output.
 * @return Parsed detection candidates.
 */
std::vector<BoundingBox> parse_boxes(const cv::Mat& boxes,
                                     const cv::Size& original_image_size,
                                     int num_classes = 80);

/**
 * @brief Keep only detections whose confidence exceeds threshold.
 * @param boxes Candidate detections.
 * @param confidence_threshold Minimum confidence to keep.
 * @return Confidence-filtered detections.
 */
std::vector<BoundingBox> filter_boxes_by_confidence(const std::vector<BoundingBox>& boxes,
                                                    float confidence_threshold = 0.6f);

/**
 * @brief Run non-maximum suppression on candidate detections.
 * @param boxes Candidate detections.
 * @param iou_threshold IoU threshold for suppression.
 * @return Final detections after NMS.
 */
std::vector<BoundingBox> non_max_suppression(const std::vector<BoundingBox>& boxes,
                                             float iou_threshold = 0.5f);

/**
 * @brief Run non-maximum suppression and return surviving indices.
 * @param boxes Candidate detections.
 * @param iou_threshold IoU threshold for suppression.
 * @return Indices of boxes that survived NMS.
 */
std::vector<int> non_max_suppression_with_indices(const std::vector<BoundingBox>& boxes,
                                                   float iou_threshold = 0.5f);

}  // namespace postprocess
}  // namespace yolo_onnx_inference
