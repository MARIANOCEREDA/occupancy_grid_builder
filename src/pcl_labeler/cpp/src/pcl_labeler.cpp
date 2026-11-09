#include "pcl_labeler/pcl_labeler.hpp"

namespace pcl_labeler
{

PCLLabeler::PCLLabeler()
{
  extrinsics_ = Eigen::Matrix4f::Identity();
  intrinsics_ = Eigen::Matrix3f::Identity();

  fuse_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
}

void PCLLabeler::set_extrinsics(const Eigen::Matrix4f& extrinsics)
{
  extrinsics_ = extrinsics;
}

void PCLLabeler::set_intrinsics(const Eigen::Matrix3f& intrinsics)
{
  intrinsics_ = intrinsics;
}

Eigen::Vector4f PCLLabeler::project_point_to_camera(const Eigen::Vector4f& point_lidar)
{
  return extrinsics_ * point_lidar;
}

bool PCLLabeler::is_point_in_front_of_camera(const Eigen::Vector4f& point_cam)
{
  return point_cam.z() > kMinDepthThreshold;
}

Eigen::Vector3f PCLLabeler::project_point_to_image(const Eigen::Vector4f& point_cam)
{
  const double inv_z = 1.0 / static_cast<double>(point_cam.z());
  const double u =
      intrinsics_(0, 0) * static_cast<double>(point_cam.x()) * inv_z + intrinsics_(0, 2);
  const double v =
      intrinsics_(1, 1) * static_cast<double>(point_cam.y()) * inv_z + intrinsics_(1, 2);
  return Eigen::Vector3f(static_cast<float>(u), static_cast<float>(v), point_cam.z());
}

bool PCLLabeler::is_point_in_image(const Eigen::Vector4f& point_cam,
                                   int img_width,
                                   int img_height)
{
  if (!is_point_in_front_of_camera(point_cam))
  {
    return false;
  }
  const Eigen::Vector3f uv = project_point_to_image(point_cam);
  return uv.x() >= 0.0f && uv.y() >= 0.0f && uv.x() < static_cast<float>(img_width) &&
         uv.y() < static_cast<float>(img_height);
}

bool PCLLabeler::is_point_valid(const Eigen::Vector4f& point_lidar)
{
  return std::isfinite(point_lidar.x()) && std::isfinite(point_lidar.y()) &&
         std::isfinite(point_lidar.z());
}

Image PCLLabeler::erode_mask(const Image& mask)
{
  cv::Mat mask_mat(mask.height, mask.width, CV_8UC1,
           const_cast<uint8_t*>(mask.data.data()));
  cv::Mat eroded_mat;
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::erode(mask_mat, eroded_mat, kernel, cv::Point(-1, -1), 1);
  Image eroded_mask = mask;
  std::memcpy(eroded_mask.data.data(), eroded_mat.data, mask.width * mask.height);
  return eroded_mask;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLLabeler::fuse(
    const Image& image, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
  // 1. Clear the overlay cloud
  fuse_cloud_->clear();

  // 2. Erode the mask to avoid edge effects
  Image image_eroded = erode_mask(image);

  // 3. Loop through each point in the input cloud
  for (const auto& point : cloud->points)
  {
    if (!is_point_valid(Eigen::Vector4f(point.x, point.y, point.z, 1.0f)))
    {
      continue;
    }

    // 4. Project the point to camera coordinates
    const Eigen::Vector4f point_lidar(point.x, point.y, point.z, 1.0f);
    const Eigen::Vector4f point_cam = project_point_to_camera(point_lidar);

    // 5. Check if the point is in front of the camera and within the image bounds
    if (!is_point_in_image(point_cam, image_eroded.width, image_eroded.height))
    {
      continue;
    }

    // 6. Sample the color from the image
    const Eigen::Vector3f uv = project_point_to_image(point_cam);
    const int u = static_cast<int>(uv.x());
    const int v = static_cast<int>(uv.y());
    const cv::Vec3b color = sample_color_at_pixel(image_eroded, u, v);

    // 7. Create a colored point and add it to the overlay cloud
    pcl::PointXYZRGB colored_point;
    colored_point.x = point.x;
    colored_point.y = point.y;
    colored_point.z = point.z;
    colored_point.b = color[0];  // OpenCV stores channels as BGR
    colored_point.g = color[1];
    colored_point.r = color[2];
    fuse_cloud_->push_back(colored_point);
  }

  // 8. Clustering and outlier removal based on statistical analysis

  return fuse_cloud_;
}

cv::Vec3b PCLLabeler::sample_color_at_pixel(const Image& image, int u, int v)
{
  const int idx = (v * image.width + u) * 3;
  return cv::Vec3b(image.data[idx], image.data[idx + 1], image.data[idx + 2]);
}

}  // namespace pcl_labeler