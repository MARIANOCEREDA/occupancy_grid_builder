#include <Eigen/Geometry>
#include <lifecycle_msgs/msg/state.hpp>
#include <occupancy_grid_builder/occupancy_grid_builder_node.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace occupancy_grid_builder
{

OccupancyGridBuilderNode::OccupancyGridBuilderNode(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("occupancy_grid_builder_node", options)
{
  RCLCPP_INFO(this->get_logger(), "OccupancyGridBuilderNode created");

  pcl_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
}

CallbackReturn OccupancyGridBuilderNode::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(this->get_logger(), "Configuring OccupancyGridBuilderNode");

  // Declare and get parameters
  this->declare_parameter<std::string>("pointcloud_topic", "/points_labeled");
  this->declare_parameter<std::string>("grid_topic", "/map");
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("sensor_frame", "lidar_frame");
  this->declare_parameter<double>("resolution", 0.1);
  this->declare_parameter<int>("width", 1000);
  this->declare_parameter<int>("height", 1000);
  this->declare_parameter<double>("origin_x", -50.0);
  this->declare_parameter<double>("origin_y", -50.0);

  this->get_parameter("pointcloud_topic", pointcloud_topic_);
  this->get_parameter("grid_topic", grid_topic_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("sensor_frame", sensor_frame_);
  this->get_parameter("resolution", resolution_);
  this->get_parameter("width", width_);
  this->get_parameter("height", height_);
  this->get_parameter("origin_x", origin_x_);
  this->get_parameter("origin_y", origin_y_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn OccupancyGridBuilderNode::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(this->get_logger(), "Activating OccupancyGridBuilderNode");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  grid_builder_ =
      std::make_unique<OccupancyGridBuilder>(resolution_, width_, height_, origin_x_, origin_y_);

  // Use TRANSIENT_LOCAL durability for map topic to be compatible with nav2 costmap
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic_, qos);

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_,
      rclcpp::QoS(10),
      std::bind(&OccupancyGridBuilderNode::pointCloudCallback, this, std::placeholders::_1));

  processing_thread_ = std::jthread(&OccupancyGridBuilderNode::gridBuilderThread, this);
  grid_pub_->on_activate();
  RCLCPP_INFO(this->get_logger(), "Activated OccupancyGridBuilderNode");
  return CallbackReturn::SUCCESS;
}

CallbackReturn OccupancyGridBuilderNode::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating OccupancyGridBuilderNode");
  pcl_cv_.notify_all();
  processing_thread_.request_stop();
  if (processing_thread_.joinable())
  {
    processing_thread_.join();
  }
  grid_pub_->on_deactivate();
  RCLCPP_INFO(this->get_logger(), "Deactivated OccupancyGridBuilderNode");
  return CallbackReturn::SUCCESS;
}

CallbackReturn OccupancyGridBuilderNode::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up OccupancyGridBuilderNode");
  pointcloud_sub_.reset();
  grid_pub_.reset();
  grid_builder_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  pcl_cv_.notify_all();
  processing_thread_.request_stop();
  if (processing_thread_.joinable())
  {
    processing_thread_.join();
  }
  RCLCPP_INFO(this->get_logger(), "Cleaned up OccupancyGridBuilderNode");
  return CallbackReturn::SUCCESS;
}

CallbackReturn OccupancyGridBuilderNode::on_shutdown(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down OccupancyGridBuilderNode");
  pointcloud_sub_.reset();
  grid_pub_.reset();
  grid_builder_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  RCLCPP_INFO(this->get_logger(), "Shut down OccupancyGridBuilderNode");
  return CallbackReturn::SUCCESS;
}

void OccupancyGridBuilderNode::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(pcl_mutex_);
    latest_pointcloud_msg_ = std::move(msg);
    new_pointcloud_available_ = true;
  }
  pcl_cv_.notify_one();
}

nav_msgs::msg::OccupancyGrid OccupancyGridBuilderNode::convertToOccupancyGrid(
    const std::vector<std::vector<int>>& grid, const std_msgs::msg::Header& header)
{
  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header.stamp = header.stamp;
  grid_msg.header.frame_id = map_frame_;
  grid_msg.info.resolution = resolution_;
  grid_msg.info.width = width_;
  grid_msg.info.height = height_;
  grid_msg.info.origin.position.x = origin_x_;
  grid_msg.info.origin.position.y = origin_y_;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;

  // Convert grid to OccupancyGrid data
  // OccupancyGrid: -1 = unknown, 0 = free, 100 = occupied
  grid_msg.data.resize(width_ * height_);

  for (int y = 0; y < height_; ++y)
  {
    for (int x = 0; x < width_; ++x)
    {
      int index = y * width_ + x;
      if (grid[y][x] == 0)
      {
        grid_msg.data[index] = 0;  // Free
      }
      else if (grid[y][x] > 0)
      {
        grid_msg.data[index] = 100;  // Occupied
      }
      else
      {
        grid_msg.data[index] = -1;  // Unknown
      }
    }
  }

  return grid_msg;
}

void OccupancyGridBuilderNode::gridBuilderThread(std::stop_token stop_token)
{
  while (rclcpp::ok() && !stop_token.stop_requested())
  {
    sensor_msgs::msg::PointCloud2::SharedPtr msg{nullptr};
    {
      std::unique_lock<std::mutex> lock(pcl_mutex_);
      pcl_cv_.wait(lock,
                   [this, &stop_token]
                   {
                     return new_pointcloud_available_.load() || !rclcpp::ok() ||
                            stop_token.stop_requested();
                   });

      if (stop_token.stop_requested())
      {
        break;
      }

      if (!new_pointcloud_available_ || !latest_pointcloud_msg_)
      {
        continue;
      }
      msg = std::move(latest_pointcloud_msg_);
      new_pointcloud_available_ = false;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
      transform_stamped = tf_buffer_->lookupTransform(
          map_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(),
                           *this->get_clock(),
                           1000,
                           "Could not transform %s to %s: %s",
                           msg->header.frame_id.c_str(),
                           map_frame_.c_str(),
                           ex.what());
      continue;
    }
    Eigen::Isometry3d transform_eigen = tf2::transformToEigen(transform_stamped.transform);
    transform_matrix_ = transform_eigen.matrix().cast<float>();

    pcl_cloud_->clear();
    pcl::fromROSMsg(*msg, *pcl_cloud_);

    const auto& grid = grid_builder_->buildGridFromPCL(pcl_cloud_, transform_matrix_);

    auto grid_msg = convertToOccupancyGrid(grid, msg->header);
    auto end_time = std::chrono::high_resolution_clock::now();

    grid_pub_->publish(grid_msg);
  }
}

}  // namespace occupancy_grid_builder

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<occupancy_grid_builder::OccupancyGridBuilderNode>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

// Register the component with class_loader for composable nodes
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(occupancy_grid_builder::OccupancyGridBuilderNode)
