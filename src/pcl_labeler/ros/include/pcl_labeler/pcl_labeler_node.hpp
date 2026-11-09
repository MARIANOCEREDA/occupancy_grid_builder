#pragma once

#include <atomic>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
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

namespace pcl_labeler
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using CloudSubscriber =
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2, rclcpp_lifecycle::LifecycleNode>;
using MaskSubscriber =
    message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode>;
using SyncPolicy = message_filters::sync_policies::
    ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image>;
using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

class PCLLabelerNode : public rclcpp_lifecycle::LifecycleNode
{
 public:
  explicit PCLLabelerNode(const rclcpp::NodeOptions& options);

 private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void label_point_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input_cloud,
                         const sensor_msgs::msg::Image::ConstSharedPtr& segmentation_mask);

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr labeled_pcl_pub_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  std::shared_ptr<MaskSubscriber> segmentation_mask_sub_;
  std::shared_ptr<CloudSubscriber> lidar_sub_;
  std::shared_ptr<Synchronizer> sync_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string camera_frame_name_;
  std::string lidar_frame_name_;

  cv::Mat camera_intrinsics_;

  std::atomic<bool> camera_info_received_{false};

  std::mutex data_mutex_;

  sensor_msgs::msg::PointCloud2 output_labeled_cloud_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr labeled_cloud_;
};
}  // namespace pcl_labeler