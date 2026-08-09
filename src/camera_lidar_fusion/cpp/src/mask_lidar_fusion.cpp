#include "mask_lidar_fusion/mask_lidar_fusion.hpp"

namespace mask_lidar_fusion
{

MaskLidarFusion::MaskLidarFusion()
{
  extrinsics_ = Eigen::Matrix4f::Identity();
  intrinsics_ = Eigen::Matrix3f::Identity();

  fuse_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
}

void MaskLidarFusion::SetExtrinsics(const Eigen::Matrix4f& extrinsics)
{
  extrinsics_ = extrinsics;
}

void MaskLidarFusion::SetIntrinsics(const Eigen::Matrix3f& intrinsics)
{
  intrinsics_ = intrinsics;
}

Eigen::Vector4f MaskLidarFusion::ProjectPointToCamera(const Eigen::Vector4f& point_lidar)
{
  return extrinsics_ * point_lidar;
}

bool MaskLidarFusion::IsPointInFrontOfCamera(const Eigen::Vector4f& point_cam)
{
  return point_cam.z() > kMinDepthThreshold;
}

Eigen::Vector3f MaskLidarFusion::ProjectPointToImage(const Eigen::Vector4f& point_cam)
{
  const double inv_z = 1.0 / static_cast<double>(point_cam.z());
  const double u =
      intrinsics_(0, 0) * static_cast<double>(point_cam.x()) * inv_z + intrinsics_(0, 2);
  const double v =
      intrinsics_(1, 1) * static_cast<double>(point_cam.y()) * inv_z + intrinsics_(1, 2);
  return Eigen::Vector3f(static_cast<float>(u), static_cast<float>(v), point_cam.z());
}

bool MaskLidarFusion::IsPointInImage(const Eigen::Vector4f& point_cam,
                                     int img_width,
                                     int img_height)
{
  if (!IsPointInFrontOfCamera(point_cam))
  {
    return false;
  }
  const Eigen::Vector3f uv = ProjectPointToImage(point_cam);
  return uv.x() >= 0.0f && uv.y() >= 0.0f && uv.x() < static_cast<float>(img_width) &&
         uv.y() < static_cast<float>(img_height);
}

bool MaskLidarFusion::IsPointValid(const Eigen::Vector4f& point_lidar)
{
  return std::isfinite(point_lidar.x()) && std::isfinite(point_lidar.y()) &&
         std::isfinite(point_lidar.z());
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MaskLidarFusion::fuse(
    const Image& image, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
  // 1. Clear the overlay cloud
  fuse_cloud_->clear();

  // 2. Loop through each point in the input cloud
  for (const auto& point : cloud->points)
  {
    if (!IsPointValid(Eigen::Vector4f(point.x, point.y, point.z, 1.0f)))
    {
      continue;
    }

    // 3. Project the point to camera coordinates
    const Eigen::Vector4f point_lidar(point.x, point.y, point.z, 1.0f);
    const Eigen::Vector4f point_cam = ProjectPointToCamera(point_lidar);

    // 4. Check if the point is in front of the camera and within the image bounds
    if (!IsPointInImage(point_cam, image.width, image.height))
    {
      continue;
    }

    // 5. Sample the color from the image
    const Eigen::Vector3f uv = ProjectPointToImage(point_cam);
    const int u = static_cast<int>(uv.x());
    const int v = static_cast<int>(uv.y());
    const cv::Vec3b color = SampleColorAtPixel(image, u, v);

    // 6. Create a colored point and add it to the overlay cloud
    pcl::PointXYZRGB colored_point;
    colored_point.x = point.x;
    colored_point.y = point.y;
    colored_point.z = point.z;
    colored_point.b = color[0];  // OpenCV stores channels as BGR
    colored_point.g = color[1];
    colored_point.r = color[2];
    fuse_cloud_->push_back(colored_point);
  }

  return fuse_cloud_;
}

cv::Vec3b MaskLidarFusion::SampleColorAtPixel(const Image& image, int u, int v)
{
  const int idx = (v * image.width + u) * 3;
  return cv::Vec3b(image.data[idx], image.data[idx + 1], image.data[idx + 2]);
}

}  // namespace mask_lidar_fusion