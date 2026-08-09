#pragma once

#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <occupancy_grid_builder/occupancy_grid_builder.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace occupancy_grid_builder
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class OccupancyGridBuilderNode : public rclcpp_lifecycle::LifecycleNode
{

 public:
  explicit OccupancyGridBuilderNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~OccupancyGridBuilderNode() override = default;

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

 private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  nav_msgs::msg::OccupancyGrid convertToOccupancyGrid(const std::vector<std::vector<int>>& grid,
                                                      const std_msgs::msg::Header& header);

  // Publishers and subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Occupancy grid builder
  std::unique_ptr<OccupancyGridBuilder> grid_builder_;

  // Parameters
  std::string pointcloud_topic_;
  std::string grid_topic_;
  std::string map_frame_;
  std::string sensor_frame_;
  double resolution_;
  int width_;
  int height_;
  double origin_x_;
  double origin_y_;
};

}  // namespace occupancy_grid_builder
