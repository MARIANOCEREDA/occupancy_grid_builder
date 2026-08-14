#include "segmentation_inference_onnx/inference_node.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace yolo_onnx_inference_ros
{
InferenceNode::InferenceNode(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("inference_node", options)
{
  this->declare_parameter<std::string>("model_path");
  this->declare_parameter<std::string>("device_type");
}

CallbackReturn InferenceNode::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Configuring InferenceNode...");
  model_path_ = this->get_parameter("model_path").as_string();
  device_type_str_ = this->get_parameter("device_type").as_string();

  if (model_path_.empty() || device_type_str_.empty())
  {
    RCLCPP_ERROR(get_logger(), "Both 'model_path' and 'device_type' parameters must be set.");
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn InferenceNode::on_activate(const rclcpp_lifecycle::State&)
{
  if (!device_type_map.count(device_type_str_))
  {
    RCLCPP_ERROR(get_logger(), "Invalid device type: %s", device_type_str_.c_str());
    return CallbackReturn::FAILURE;
  }

  device_type_ = device_type_map.at(device_type_str_);

  try
  {
    detector_ = std::make_shared<yolo_onnx_inference::YoloONNXInference>(model_path_, device_type_);
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(get_logger(), "Unable to load model '%s': %s", model_path_.c_str(), error.what());
    return CallbackReturn::FAILURE;
  }

  if (!detection_pub_)
  {
    detection_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>("detections", 10);
  }
  detection_pub_->on_activate();

  if (!mask_pub_)
  {
    mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/segmentation/mask", 10);
  }
  mask_pub_->on_activate();

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_rect", 10, std::bind(&InferenceNode::ImageCallback, this, std::placeholders::_1));

    inference_thread_ = std::jthread(
      [this](std::stop_token stop_token) { RunInferenceAsync(stop_token); });

  RCLCPP_INFO(get_logger(), "Activated InferenceNode...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn InferenceNode::on_deactivate(const rclcpp_lifecycle::State&)
{
  inference_thread_.request_stop();
  input_image_cv_.notify_all();

  if (inference_thread_.joinable())
  {
    inference_thread_.join();
  }

  image_sub_.reset();
  if (detection_pub_)
  {
    detection_pub_->on_deactivate();
  }
  if (mask_pub_)
  {
    mask_pub_->on_deactivate();
  }

  RCLCPP_INFO(get_logger(), "Deactivated InferenceNode...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn InferenceNode::on_cleanup(const rclcpp_lifecycle::State&)
{
  inference_thread_.request_stop();
  input_image_cv_.notify_all();

  if (inference_thread_.joinable())
  {
    inference_thread_.join();
  }

  image_sub_.reset();
  detection_pub_.reset();
  mask_pub_.reset();
  detector_.reset();
  RCLCPP_INFO(get_logger(), "InferenceNode cleaned up.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn InferenceNode::on_shutdown(const rclcpp_lifecycle::State&)
{
  inference_thread_.request_stop();
  input_image_cv_.notify_all();

  if (inference_thread_.joinable())
  {
    inference_thread_.join();
  }

  RCLCPP_INFO(get_logger(), "Shutting down InferenceNode...");
  return CallbackReturn::SUCCESS;
}

void InferenceNode::ToVisionMsgsDetections(
    const std::vector<yolo_onnx_inference::BoundingBox>& detections,
    vision_msgs::msg::Detection2DArray& detection_msg)
{
  detection_msg.detections.clear();
  for (const auto& detection : detections)
  {
    vision_msgs::msg::Detection2D detection2d;
    vision_msgs::msg::BoundingBox2D bbox;
    vision_msgs::msg::Pose2D pose;
    pose.position.x = detection.x + detection.width / 2.0f;
    pose.position.y = detection.y + detection.height / 2.0f;
    pose.theta = 0.0f;  // Assuming no rotation information is available
    bbox.center = pose;
    bbox.size_x = detection.width;
    bbox.size_y = detection.height;

    detection2d.bbox = bbox;

    vision_msgs::msg::ObjectHypothesisWithPose result;
    result.hypothesis.class_id = std::to_string(detection.class_id);
    result.hypothesis.score = detection.confidence;

    detection2d.results.push_back(result);
    detection_msg.detections.push_back(detection2d);
  }
}

cv::Mat InferenceNode::CreateBinaryMask(
    const cv::Size& image_size,
    const std::vector<yolo_onnx_inference::BoundingBox>& detections)
{
  cv::Mat output(image_size, CV_8UC1, cv::Scalar(0));

  for (const auto& detection : detections)
  {
    if (detection.mask.empty())
    {
      continue;
    }

    int x = std::max(0, static_cast<int>(detection.x));
    int y = std::max(0, static_cast<int>(detection.y));
    int w = std::min(static_cast<int>(detection.width),  output.cols - x);
    int h = std::min(static_cast<int>(detection.height), output.rows - y);

    if (w <= 0 || h <= 0)
    {
      continue;
    }

    cv::Mat mask_resized;
    if (detection.mask.rows != h || detection.mask.cols != w)
    {
      cv::resize(detection.mask, mask_resized, cv::Size(w, h), 0, 0, cv::INTER_NEAREST);
    }
    else
    {
      mask_resized = detection.mask;
    }

    cv::Mat roi = output(cv::Rect(x, y, w, h));
    for (int r = 0; r < h; ++r)
    {
      for (int c = 0; c < w; ++c)
      {
        if (mask_resized.at<float>(r, c) > 0.5f)
        {
          roi.at<uint8_t>(r, c) = 255;
        }
      }
    }
  }

  return output;
}

void InferenceNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(input_image_mutex_);
    latest_image_ = std::move(msg);
    new_image_available_ = true;
  }
  input_image_cv_.notify_one();
}

void InferenceNode::RunInferenceAsync(std::stop_token stop_token)
{
  while (rclcpp::ok() && !stop_token.stop_requested())
  {
    sensor_msgs::msg::Image::SharedPtr input_image_msg{nullptr};
    {
      std::unique_lock<std::mutex> lock(input_image_mutex_);
      input_image_cv_.wait(
          lock,
          [this, &stop_token]
          { return new_image_available_.load() || !rclcpp::ok() || stop_token.stop_requested(); });

      if (stop_token.stop_requested())
      {
        break;
      }

      if (!new_image_available_ || !latest_image_)
      {
        continue;
      }

      input_image_msg = std::move(latest_image_);
      new_image_available_ = false;
    }

    cv::Mat input_inference_image;
    try
    {
      input_inference_image =
          cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (const cv_bridge::Exception& error)
    {
      RCLCPP_ERROR(get_logger(), "Unable to convert input image to BGR8: %s", error.what());
      continue;
    }

    std::vector<yolo_onnx_inference::BoundingBox> detections;
    try
    {
      detections = detector_->infer_segmentation(input_inference_image);
    }
    catch (const std::exception& error)
    {
      RCLCPP_ERROR(get_logger(), "Inference failed: %s", error.what());
      continue;
    }

    vision_msgs::msg::Detection2DArray detection_msg;
    detection_msg.header.stamp = input_image_msg->header.stamp;
    detection_msg.header.frame_id = input_image_msg->header.frame_id;

    this->ToVisionMsgsDetections(detections, detection_msg);
    if (detection_pub_ && detection_pub_->is_activated())
    {
      detection_pub_->publish(detection_msg);
    }

    // Publish class-coloured segmentation mask (black background)
    if (mask_pub_ && mask_pub_->is_activated())
    {
      cv::Mat binary_mask = CreateBinaryMask(input_inference_image.size(), detections);

      auto mask_msg = cv_bridge::CvImage(
        input_image_msg->header,
        sensor_msgs::image_encodings::MONO8,
        binary_mask
      ).toImageMsg();

      mask_pub_->publish(*mask_msg);
    }
  }
}

}  // namespace yolo_onnx_inference_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(yolo_onnx_inference_ros::InferenceNode)
