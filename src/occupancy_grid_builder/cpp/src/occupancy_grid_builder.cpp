#include <occupancy_grid_builder/occupancy_grid_builder.hpp>

namespace occupancy_grid_builder
{
OccupancyGridBuilder::~OccupancyGridBuilder() = default;

std::vector<std::vector<int>>& OccupancyGridBuilder::buildGridFromPCL(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const Eigen::Matrix4f& pcl_origin_to_map_transform)
{
  // Compute sensor origin in map frame (sensor origin is at 0,0,0 in sensor frame)
  Eigen::Vector4f sensor_origin_sensor_frame(0.0f, 0.0f, 0.0f, 1.0f);
  Eigen::Vector4f sensor_origin_in_map_frame = 
      pointToMap(sensor_origin_sensor_frame, pcl_origin_to_map_transform);
  
  for (const auto& point : cloud->points)
  {
    Eigen::Vector4f point_sensor_frame(point.x, point.y, point.z, 1.0f);
    Eigen::Vector4f point_in_map_frame =
        pointToMap(point_sensor_frame, pcl_origin_to_map_transform);
    Eigen::Vector2i grid_coords = mapToGrid(point_in_map_frame);

    // Raycasting with Bresenham's line algorithm to mark free space between the sensor origin and
    // the point
    auto cell_hit = mapToGrid(point_in_map_frame);
    auto cell_origin = mapToGrid(sensor_origin_in_map_frame); 

    auto ray_cells = bresenhamLine(cell_origin.x(), cell_origin.y(), cell_hit.x(), cell_hit.y());

    // Mark all cells along the ray as free (except the last one which is the hit point)
    // Only update if cell was unknown (-1) or already free (0), don't override occupied cells
    for (size_t i = 0; i < ray_cells.size() - 1; ++i)
    {
      int x = ray_cells[i].x();
      int y = ray_cells[i].y();
      if (x >= 0 && x < width_ && y >= 0 && y < height_)
      {
        if (occupancy_grid_[y][x] != 100)  // Don't override occupied cells
        {
          occupancy_grid_[y][x] = 0;  // Mark as free (ROS2 nav2 standard)
        }
      }
    }

    // Mark the hit point as occupied or free based on color
    if (grid_coords.x() >= 0 && grid_coords.x() < width_ && grid_coords.y() >= 0 &&
        grid_coords.y() < height_)
    {
      if (point.r > 0 || point.g > 0 || point.b > 0)  // Check if the point is colored
      {
        occupancy_grid_[grid_coords.y()][grid_coords.x()] = 100;  // Mark as occupied (ROS2 nav2 standard)
      }
      else
      {
        if (occupancy_grid_[grid_coords.y()][grid_coords.x()] != 100)  // Don't override occupied
        {
          occupancy_grid_[grid_coords.y()][grid_coords.x()] = 0;  // Mark as free
        }
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

std::vector<Eigen::Vector2i> OccupancyGridBuilder::bresenhamLine(int x0, int y0, int x1, int y1)
{
  std::vector<Eigen::Vector2i> line;
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (true)
  {
    line.emplace_back(x0, y0);

    if (x0 == x1 && y0 == y1)
      break;

    int e2 = 2 * err;
    if (e2 > -dy)
    {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx)
    {
      err += dx;
      y0 += sy;
    }
  }

  return line;
}

}  // namespace occupancy_grid_builder