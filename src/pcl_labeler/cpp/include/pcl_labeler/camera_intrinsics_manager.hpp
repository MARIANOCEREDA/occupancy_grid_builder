#pragma once

#include <eigen3/Eigen/Core>
#include <sensor_msgs/msg/camera_info.hpp>

namespace pcl_labeler
{
class CameraInfoManager
{
 public:
  CameraInfoManager();
  ~CameraInfoManager() = default;

  void set_message(const sensor_msgs::msg::CameraInfo& camera_info_msg);

  Eigen::Matrix3d get_intrinsic_matrix() const;
  Eigen::Matrix4d get_projection_matrix() const;
  Eigen::VectorXd get_distortion_coefficients() const;

 private:
  void process_intrinsics_matrix();
  void process_projection_matrix();
  void process_distortion_coefficients();

  sensor_msgs::msg::CameraInfo camera_info_msg_;
  Eigen::Matrix3d intrinsics_matrix_;
  Eigen::Matrix4d projection_matrix_;
  Eigen::VectorXd distortion_coefficients_;
};
}  // namespace pcl_labeler