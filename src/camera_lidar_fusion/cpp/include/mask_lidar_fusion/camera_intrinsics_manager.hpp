#pragma once

#include <eigen3/Eigen/Core>
#include <sensor_msgs/msg/camera_info.hpp>

namespace mask_lidar_fusion
{
class CameraInfoManager
{
 public:
  CameraInfoManager();
  ~CameraInfoManager() = default;

  void SetMessage(const sensor_msgs::msg::CameraInfo& camera_info_msg);

  Eigen::Matrix3d GetIntrinsicMatrix() const;
  Eigen::Matrix4d GetProjectionMatrix() const;
  Eigen::VectorXd GetDistortionCoefficients() const;

 private:
  void ProcessIntrinsicsMatrix();
  void ProcessProjectionMatrix();
  void ProcessDistortionCoefficients();

  sensor_msgs::msg::CameraInfo camera_info_msg_;
  Eigen::Matrix3d intrinsics_matrix_;
  Eigen::Matrix4d projection_matrix_;
  Eigen::VectorXd distortion_coefficients_;
};
}  // namespace mask_lidar_fusion