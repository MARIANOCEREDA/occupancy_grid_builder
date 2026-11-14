#include "image_processing/image_undistort.hpp"

namespace image_undistort
{
static constexpr char const* const kNodeName = "image_undistort_node";
ImageUndistort::ImageUndistort(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode(kNodeName, options)
{
  RCLCPP_INFO(get_logger(), "ImageUndistort node created");
}

CallbackReturn ImageUndistort::on_configure(const rclcpp_lifecycle::State&)
{
  this->declare_parameter("input_image_topic", "image");
  this->declare_parameter("camera_info_topic", "camera_info");
  this->declare_parameter("output_image_topic", "image_rect");
  this->declare_parameter("display_latency", false);

  image_topic_ = this->get_parameter("input_image_topic").as_string();
  camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
  output_image_topic_ = this->get_parameter("output_image_topic").as_string();
  display_latency_ = this->get_parameter("display_latency").as_bool();

  RCLCPP_INFO(get_logger(), "Configured %s node", kNodeName);

  return CallbackReturn::SUCCESS;
}

CallbackReturn ImageUndistort::on_activate(const rclcpp_lifecycle::State&)
{
  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      1,
      std::bind(&ImageUndistort::image_callback, this, std::placeholders::_1),
      sub_options);

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      1,
      std::bind(&ImageUndistort::camera_info_callback, this, std::placeholders::_1),
      sub_options);

  undistorted_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_image_topic_, 1);
  RCLCPP_INFO(get_logger(), "Activated %s node", kNodeName);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImageUndistort::on_deactivate(const rclcpp_lifecycle::State&)
{
  image_sub_.reset();
  camera_info_sub_.reset();
  undistorted_image_pub_.reset();
  RCLCPP_INFO(get_logger(), "Deactivated %s node", kNodeName);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImageUndistort::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s node", kNodeName);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImageUndistort::on_shutdown(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s node", kNodeName);
  return CallbackReturn::SUCCESS;
}

void ImageUndistort::image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg)
{
  if (!camera_info_received_)
  {
    RCLCPP_WARN(get_logger(), "Camera info not received yet. Skipping image undistortion.");
    return;
  }

  const auto t_start = std::chrono::steady_clock::now();
  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, image_msg->encoding);

  if (distortion_model_ != DistortionModels::FISHEYE)
  {
    RCLCPP_ERROR(get_logger(), "Only fisheye distortion model is supported.");
    return;
  }

  // 1. Build the map once, since intrinsics are static.
  if (map1_.empty() || map2_.empty())
  {
    cv::fisheye::initUndistortRectifyMap(camera_matrix_,      // K  
                                         dist_coeffs_,        // D
                                         cv::Matx33d::eye(),  // I
                                         camera_matrix_,      // P
                                         cv_ptr->image.size(),
                                         CV_32FC1,
                                         map1_,
                                         map2_);
  }

  // 2. Undistort using the calculated map.
  cv::remap(cv_ptr->image, undistorted_image_, map1_, map2_, cv::INTER_LINEAR);

  auto undistorted_image_msg = std::make_unique<sensor_msgs::msg::Image>();

  undistorted_image_msg->header = image_msg->header;
  undistorted_image_msg->height = undistorted_image_.rows;
  undistorted_image_msg->width = undistorted_image_.cols;
  undistorted_image_msg->encoding = image_msg->encoding;
  undistorted_image_msg->is_bigendian = 0;
  undistorted_image_msg->step = undistorted_image_.cols * undistorted_image_.elemSize();

  undistorted_image_msg->data.resize(undistorted_image_.total() * undistorted_image_.elemSize());
  undistorted_image_msg->data.assign(undistorted_image_.datastart, undistorted_image_.dataend);

  undistorted_image_pub_->publish(std::move(undistorted_image_msg));
  const auto t_published = std::chrono::steady_clock::now();

  if (display_latency_)
  {
    // Processing time of image_callback only (steady clock): excludes upstream/transport latency.
    total_ms_sum_ += std::chrono::duration<double, std::milli>(t_published - t_start).count();
    ++timing_frames_;
    if (timing_frames_ % 10 == 0)
    {
      RCLCPP_INFO(get_logger(),
                  "Average processing time over %zu frames: total=%.2f ms",
                  timing_frames_,
                  total_ms_sum_ / static_cast<double>(timing_frames_));
      total_ms_sum_ = 0.0;
      timing_frames_ = 0;
    }
  }
}

void ImageUndistort::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg)
{
  auto n_coeffs = camera_info_msg->d.size();
  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  dist_coeffs_ = cv::Mat(n_coeffs, 1, CV_64F);

  for (size_t row = 0; row < 3; ++row)
  {
    for (size_t col = 0; col < 3; ++col)
    {
      camera_matrix_.at<double>(row, col) = camera_info_msg->k[row * 3 + col];
    }
  }

  for (size_t i = 0; i < n_coeffs; ++i)
  {
    dist_coeffs_.at<double>(i) = camera_info_msg->d[i];
  }

  camera_info_received_ = true;
  camera_info_sub_.reset();
}

}  // namespace image_undistort

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_undistort::ImageUndistort);
