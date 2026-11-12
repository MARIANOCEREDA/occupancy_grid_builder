#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl_labeler
{
class PCLLabeler
{
 public:
  PCLLabeler();
  void set_extrinsics(const Eigen::Matrix4f& extrinsics);
  void set_intrinsics(const Eigen::Matrix3f& intrinsics);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr fuse(const cv::Mat& image,
                                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

 private:
  cv::Mat erode_mask(const cv::Mat& mask);
  Eigen::Vector4f project_point_to_camera(const Eigen::Vector4f& point_lidar);
  bool is_point_in_front_of_camera(const Eigen::Vector4f& point_cam);
  bool is_point_valid(const float& x, const float& y, const float& z);
  bool is_point_in_image(const Eigen::Vector4f& point_cam, int img_width, int img_height);
  Eigen::Vector3f project_point_to_image(const Eigen::Vector4f& point_cam);
  cv::Vec3b sample_color_at_pixel(const cv::Mat& image, int u, int v);

  Eigen::Matrix4f extrinsics_;
  Eigen::Matrix3f intrinsics_;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> fuse_cloud_;

  constexpr static float kMinDepthThreshold = 0.0000001f;
};

}  // namespace pcl_labeler