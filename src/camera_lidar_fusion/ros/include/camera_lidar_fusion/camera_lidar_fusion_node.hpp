#pragma once

#include <atomic>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace camera_lidar_fusion
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CameraLidarFusionNode : public rclcpp_lifecycle::LifecycleNode
{
 public:
  explicit CameraLidarFusionNode(const rclcpp::NodeOptions& options);

 private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

  void segmentation_mask_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void label_point_cloud();

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr labeled_pcl_pub_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr segmentation_mask_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string camera_frame_name_;
  std::string lidar_frame_name_;

  cv::Mat camera_intrinsics_;

  std::atomic<bool> camera_info_received_{false};
  std::atomic<bool> segmentation_mask_received_{false};
  std::atomic<bool> lidar_data_received_{false};

  std::mutex data_mutex_;

  sensor_msgs::msg::Image latest_segmentation_mask_;
  sensor_msgs::msg::PointCloud2 latest_lidar_data_;
  sensor_msgs::msg::PointCloud2 output_labeled_cloud_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr labeled_cloud_;
};
}  // namespace camera_lidar_fusion