#include <occupancy_grid_builder/occupancy_grid_builder.hpp>

namespace occupancy_grid_builder
{
OccupancyGridBuilder::~OccupancyGridBuilder() = default;

std::vector<std::vector<int>>& OccupancyGridBuilder::buildGridFromPCL(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Matrix4f pcl_origin_to_map_transform)
{
    for (const auto& point : cloud->points)
    {
        Eigen::Vector4f point_sensor_frame(point.x, point.y, point.z, 1.0f);
        Eigen::Vector4f point_in_map_frame = pointToMap(point_sensor_frame, pcl_origin_to_map_transform);
        Eigen::Vector2i grid_coords = mapToGrid(point_in_map_frame);

        if (grid_coords.x() >= 0 && grid_coords.x() < width_ &&
            grid_coords.y() >= 0 && grid_coords.y() < height_)
        {
            if (point.r > 0 || point.g > 0 || point.b > 0)  // Check if the point is colored
            {
                occupancy_grid_[grid_coords.y()][grid_coords.x()] = 1;  // Mark as occupied
            }
            else
            {
                occupancy_grid_[grid_coords.y()][grid_coords.x()] = 0;  // Mark as free
            }
        }
    }
    return occupancy_grid_;
}

Eigen::Vector4f OccupancyGridBuilder::pointToMap(const Eigen::Vector4f& point_sensor_frame,
                                                  const Eigen::Matrix4f& sensor_to_map_transform)
{
    return sensor_to_map_transform * point_sensor_frame;
}

Eigen::Vector2i OccupancyGridBuilder::mapToGrid(const Eigen::Vector4f& point_in_map_frame)
{
    int grid_x = static_cast<int>((point_in_map_frame.x() - origin_x_) / resolution_);
    int grid_y = static_cast<int>((point_in_map_frame.y() - origin_y_) / resolution_);
    return Eigen::Vector2i(grid_x, grid_y);
}

}