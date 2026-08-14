#pragma once

#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace yolo_onnx_inference
{

enum class DeviceType
{
  /// Run inference on CPU.
  CPU,
  /// Run inference on GPU through CUDA provider.
  GPU
};

static constexpr struct InputTensorSize
{
  /// Batch dimension.
  int batch;
  /// Channel dimension.
  int channels;
  /// Input width in pixels.
  int width;
  /// Input height in pixels.
  int height;
  /// Total number of scalar values in one input tensor.
  float size = batch * channels * width * height;
} input_size = {1, 3, 640, 640};

/**
 * @brief Represents one detected object bounding box.
 */
struct BoundingBox
{
  /// Top-left x coordinate in image space.
  float x;
  /// Top-left y coordinate in image space.
  float y;
  /// Bounding box width in pixels.
  float width;
  /// Bounding box height in pixels.
  float height;
  /// Detection confidence score in [0, 1].
  float confidence;
  /// Predicted class index.
  int class_id;
  /// OpenCV rectangle representation of the same box.
  cv::Rect bbox;
  // Mask coefficients for instance segmentation.
  std::vector<float> mask_coefficients;
  // Segmentation mask for instance segmentation.
  cv::Mat mask; // <-- NEW

  /**
   * @brief Construct an empty bounding box.
   */
  BoundingBox() : x(0), y(0), width(0), height(0), confidence(0), class_id(0) {}

  /**
   * @brief Construct a bounding box from scalar values.
   * @param x Top-left x coordinate.
   * @param y Top-left y coordinate.
   * @param width Box width in pixels.
   * @param height Box height in pixels.
   * @param confidence Detection confidence score.
   * @param class_id Predicted class index.
   */
  BoundingBox(float x, float y, float width, float height, float confidence, int class_id)
    : x(x), y(y), width(width), height(height), confidence(confidence), class_id(class_id)
  {
  }
};

class YoloONNXInference
{
 public:
  YoloONNXInference(const std::string& model_path, DeviceType device_type = DeviceType::CPU);
  ~YoloONNXInference();

  std::vector<BoundingBox> infer_segmentation(const cv::Mat& input_image);

  /**
   * @brief Create ONNX Runtime session options based on current configuration.
   * @return Configured session options.
   */
  Ort::SessionOptions create_session_options();

  /**
   * @brief Load model session on the configured device.
   * @return Ready-to-run ONNX Runtime session.
   */
  Ort::Session load_model();

  /**
   * @brief Initialize model input and output metadata.
   */
  void initialize_model_io();

  /**
   * @brief Get current preprocessed image buffer.
   * @return Preprocessed image matrix.
   */
  cv::Mat get_image() const
  {
    return output_image_;
  }

  void preprocess(const cv::Mat& input_image);
  std::vector<BoundingBox> postprocess(std::vector<Ort::Value>& output);
  void blob_to_onnx_tensor(const cv::Mat& blob);
    /**
   * @brief Map detections from letterbox/model space back to original image space.
   * @param boxes Detections in model-input coordinates to be updated in-place.
   */
  void remove_letterbox_offset(std::vector<BoundingBox>& boxes) const;

 private:
  /// Device used to run inference.
  DeviceType device_type_ = DeviceType::CPU;
  /// Path to model file.
  std::string model_path_;

  /// ONNX Runtime logging environment.
  Ort::Env env_{nullptr};
  /// Session options used to construct the runtime session.
  Ort::SessionOptions session_options_{nullptr};
  /// Active ONNX Runtime session.
  Ort::Session session_{nullptr};
  /// CUDA provider configuration handle, set only for GPU mode.
  OrtCUDAProviderOptionsV2* cuda_provider_ = nullptr;

  /// Model input tensor.
  Ort::Value input_tensor_{nullptr};
  /// Model output tensor.
  Ort::Value output_tensor_{nullptr};

  static constexpr int N_OUTPUTS_SEG = 2;
  static constexpr float confidence_threshold_ = 0.4f;

  /// Input node name.
  std::string input_name_;
  /// Output node names, kept alive for the lifetime of the session calls.
  std::array<std::string, N_OUTPUTS_SEG> output_name_storage_{};
  /// Input node names passed to ONNX Runtime.
  std::array<const char*, 1> input_names_{};
  /// Output node names passed to ONNX Runtime.
  std::array<const char*, N_OUTPUTS_SEG> output_names_{};

  cv::Size last_input_image_size_{input_size.width, input_size.height};
  cv::Mat output_image_;
  cv::Mat output_image_blob_;
};
}  // namespace yolo_onnx_inference