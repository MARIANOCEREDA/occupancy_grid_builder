#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace occupancy_grid_builder
{
class OccupancyGridBuilder
{
 public:
  OccupancyGridBuilder(double resolution = 0.1,
                       int width = 1000,
                       int height = 1000,
                       double origin_x = -50.0,
                       double origin_y = -50.0)
    : resolution_(resolution),
      width_(width),
      height_(height),
      origin_x_(origin_x),
      origin_y_(origin_y)
  {
    occupancy_grid_.resize(height_, std::vector<int>(width_, 0));
  }
  ~OccupancyGridBuilder();

  std::vector<std::vector<int>>& buildGridFromPCL(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                  Eigen::Matrix4f pcl_origin_to_map_transform);
  Eigen::Vector4f pointToMap(const Eigen::Vector4f& point_sensor_frame,
                             const Eigen::Matrix4f& sensor_to_map_transform);
  Eigen::Vector2i mapToGrid(const Eigen::Vector4f& point_in_map_frame);

 private:
  std::vector<std::vector<int>> occupancy_grid_;
  double resolution_ = 0.1;  // meters/cell — 0.1m is a common default
  int width_ = 1000;         // cells → 100m
  int height_ = 1000;        // cells → 100m
  double origin_x_ = -50.0;  // meters, grid origin offset from map frame origin
  double origin_y_ = -50.0;
};
}  // namespace occupancy_grid_builder