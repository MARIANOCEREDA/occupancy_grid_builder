#include "pcl_labeler/camera_intrinsics_manager.hpp"

namespace pcl_labeler
{
CameraInfoManager::CameraInfoManager()
{
  intrinsics_matrix_ = Eigen::Matrix3d::Identity();
  projection_matrix_ = Eigen::Matrix4d::Identity();
}

void CameraInfoManager::set_message(const sensor_msgs::msg::CameraInfo& camera_info_msg)
{
  camera_info_msg_ = camera_info_msg;
  process_intrinsics_matrix();
};

void CameraInfoManager::process_intrinsics_matrix()
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

void CameraInfoManager::process_projection_matrix()
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

void CameraInfoManager::process_distortion_coefficients()
{
  distortion_coefficients_ = Eigen::VectorXd::Zero(camera_info_msg_.d.size());
  for (size_t i = 0; i < camera_info_msg_.d.size(); i++)
  {
    distortion_coefficients_(i) = camera_info_msg_.d[i];
  }
}

Eigen::Matrix3d CameraInfoManager::get_intrinsic_matrix() const
{
  return intrinsics_matrix_;
}

Eigen::Matrix4d CameraInfoManager::get_projection_matrix() const
{
  return projection_matrix_;
}

Eigen::VectorXd CameraInfoManager::get_distortion_coefficients() const
{
  return distortion_coefficients_;
}
}  // namespace pcl_labeler