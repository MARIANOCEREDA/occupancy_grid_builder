#include "mask_lidar_fusion/camera_intrinsics_manager.hpp"

namespace mask_lidar_fusion
{
CameraInfoManager::CameraInfoManager()
{
  intrinsics_matrix_ = Eigen::Matrix3d::Identity();
  projection_matrix_ = Eigen::Matrix4d::Identity();
}

void CameraInfoManager::SetMessage(const sensor_msgs::msg::CameraInfo& camera_info_msg)
{
  camera_info_msg_ = camera_info_msg;
  ProcessIntrinsicsMatrix();
};

void CameraInfoManager::ProcessIntrinsicsMatrix()
{
  if (camera_info_msg_.k.size() != 9)
  {
    return;
  }
  for (size_t i = 0; i < 3; i++)
  {
    for (size_t j = 0; j < 3; j++)
    {
      intrinsics_matrix_(i, j) = camera_info_msg_.k[i * 3 + j];
    }
  }
}

void CameraInfoManager::ProcessProjectionMatrix()
{
  if (camera_info_msg_.p.size() != 12)
  {
    return;
  }
  for (size_t i = 0; i < 3; i++)
  {
    for (size_t j = 0; j < 4; j++)
    {
      projection_matrix_(i, j) = camera_info_msg_.p[i * 4 + j];
    }
  }
}

void CameraInfoManager::ProcessDistortionCoefficients()
{
  distortion_coefficients_ = Eigen::VectorXd::Zero(camera_info_msg_.d.size());
  for (size_t i = 0; i < camera_info_msg_.d.size(); i++)
  {
    distortion_coefficients_(i) = camera_info_msg_.d[i];
  }
}

Eigen::Matrix3d CameraInfoManager::GetIntrinsicMatrix() const
{
  return intrinsics_matrix_;
}

Eigen::Matrix4d CameraInfoManager::GetProjectionMatrix() const
{
  return projection_matrix_;
}

Eigen::VectorXd CameraInfoManager::GetDistortionCoefficients() const
{
  return distortion_coefficients_;
}
}  // namespace mask_lidar_fusion