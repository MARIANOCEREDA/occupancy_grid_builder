#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mask_lidar_fusion
{

struct Image
{
  int width;
  int height;
  std::vector<uint8_t> data;
};

class MaskLidarFusion
{
 public:
  MaskLidarFusion();
  void SetExtrinsics(const Eigen::Matrix4f& extrinsics);
  void SetIntrinsics(const Eigen::Matrix3f& intrinsics);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr fuse(const Image& image,
                                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

 private:
  Eigen::Vector4f ProjectPointToCamera(const Eigen::Vector4f& point_lidar);
  bool IsPointInFrontOfCamera(const Eigen::Vector4f& point_cam);
  bool IsPointValid(const Eigen::Vector4f& point_lidar);
  bool IsPointInImage(const Eigen::Vector4f& point_cam, int img_width, int img_height);
  Eigen::Vector3f ProjectPointToImage(const Eigen::Vector4f& point_cam);
  cv::Vec3b SampleColorAtPixel(const Image& image, int u, int v);

  Eigen::Matrix4f extrinsics_;
  Eigen::Matrix3f intrinsics_;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> fuse_cloud_;

  constexpr static float kMinDepthThreshold = 0.0000001f;
};

}  // namespace mask_lidar_fusion