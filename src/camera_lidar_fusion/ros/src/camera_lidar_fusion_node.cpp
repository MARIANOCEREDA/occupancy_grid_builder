#include "camera_lidar_fusion/camera_lidar_fusion_node.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "mask_lidar_fusion/mask_lidar_fusion.hpp"

namespace camera_lidar_fusion
{

CameraLidarFusionNode::CameraLidarFusionNode(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("camera_lidar_fusion_node", options)
{
  RCLCPP_INFO(get_logger(), "Camera-LiDAR Fusion Node has been created.");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
  pcl_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  labeled_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
}

CallbackReturn CameraLidarFusionNode::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Configuring Camera-LiDAR Fusion Node...");

  this->declare_parameter<std::string>("camera_frame_name", "camera_frame");
  this->declare_parameter<std::string>("lidar_frame_name", "lidar_frame");
  this->declare_parameter<std::string>("point_cloud_topic", "rslidar_points");
  this->declare_parameter<std::string>("labeled_point_cloud_topic", "points_labeled");
  this->declare_parameter<std::string>("segmentation_mask_topic", "/segmentation/mask");
  this->declare_parameter<std::string>("camera_info_topic", "camera_info");

  camera_frame_name_ = this->get_parameter("camera_frame_name").as_string();
  lidar_frame_name_ = this->get_parameter("lidar_frame_name").as_string();

  std::string point_cloud_topic = this->get_parameter("point_cloud_topic").as_string();
  std::string segmentation_mask_topic = this->get_parameter("segmentation_mask_topic").as_string();
  std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
  std::string labeled_point_cloud_topic =
      this->get_parameter("labeled_point_cloud_topic").as_string();

  // Create publishers
  labeled_pcl_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(labeled_point_cloud_topic, 10);

  rclcpp::SubscriptionOptions camera_sub_options;
  camera_sub_options.callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create subscriptions
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic,
      10,
      std::bind(&CameraLidarFusionNode::camera_info_callback, this, std::placeholders::_1),
      camera_sub_options);
  segmentation_mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      segmentation_mask_topic,
      10,
      std::bind(&CameraLidarFusionNode::segmentation_mask_callback, this, std::placeholders::_1),
      camera_sub_options);
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic,
      10,
      std::bind(&CameraLidarFusionNode::lidar_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Camera-LiDAR Fusion Node configured.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CameraLidarFusionNode::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Activating Camera-LiDAR Fusion Node...");
  labeled_pcl_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "Camera-LiDAR Fusion Node activated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CameraLidarFusionNode::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Deactivating Camera-LiDAR Fusion Node...");
  labeled_pcl_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Camera-LiDAR Fusion Node deactivated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CameraLidarFusionNode::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Cleaning up Camera-LiDAR Fusion Node...");
  labeled_pcl_pub_.reset();
  camera_info_sub_.reset();
  segmentation_mask_sub_.reset();
  lidar_sub_.reset();
  RCLCPP_INFO(get_logger(), "Camera-LiDAR Fusion Node cleaned up.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CameraLidarFusionNode::on_shutdown(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Shutting down Camera-LiDAR Fusion Node...");
  return CallbackReturn::SUCCESS;
}

void CameraLidarFusionNode::segmentation_mask_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_segmentation_mask_ = *msg;
    segmentation_mask_received_ = true;
  }
}

void CameraLidarFusionNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  camera_intrinsics_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      camera_intrinsics_.at<double>(i, j) = msg->k[i * 3 + j];
    }
  }
  camera_info_received_ = true;
  camera_info_sub_.reset();
  RCLCPP_INFO(get_logger(), "Received camera info.");
}

void CameraLidarFusionNode::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_lidar_data_ = *msg;
    lidar_data_received_ = true;
  }

  // Process the point cloud if all data is available
  label_point_cloud();
}

void CameraLidarFusionNode::label_point_cloud()
{
  sensor_msgs::msg::PointCloud2 input_cloud;
  sensor_msgs::msg::Image segmentation_mask;
  cv::Mat camera_intrinsics;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!camera_info_received_ || !segmentation_mask_received_ || !lidar_data_received_)
    {
      RCLCPP_DEBUG(get_logger(), "Not all data received yet. Skipping labeling.");
      return;
    }
    input_cloud = latest_lidar_data_;
    segmentation_mask = latest_segmentation_mask_;
    camera_intrinsics = camera_intrinsics_.clone();
  }

  try
  {
    // 1. Get camera-to-lidar transform
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped =
        tf_buffer_->lookupTransform(camera_frame_name_, lidar_frame_name_, tf2::TimePointZero);

    Eigen::Isometry3d transform_eigen = tf2::transformToEigen(transform_stamped.transform);
    Eigen::Matrix4f extrinsics = transform_eigen.matrix().cast<float>();

    // 2. Convert segmentation mask to OpenCV
    cv_bridge::CvImagePtr cv_mask = cv_bridge::toCvCopy(segmentation_mask);
    cv::Mat mask_single_channel = cv_mask->image;

    // Convert single-channel mask to 3-channel BGR image
    // Labels are encoded in the blue channel (or all channels for grayscale)
    cv::Mat mask_bgr;
    if (mask_single_channel.channels() == 1)
    {
      cv::cvtColor(mask_single_channel, mask_bgr, cv::COLOR_GRAY2BGR);
    }
    else
    {
      mask_bgr = mask_single_channel;
    }

    // 3. Convert ROS PointCloud2 to PCL
    pcl_cloud_->clear();
    pcl::fromROSMsg(input_cloud, *pcl_cloud_);

    // 4. Setup mask_lidar_fusion and call fuse()
    mask_lidar_fusion::MaskLidarFusion fusion;

    // Convert camera intrinsics to Eigen
    Eigen::Matrix3f intrinsics_eigen;
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        intrinsics_eigen(i, j) = static_cast<float>(camera_intrinsics.at<double>(i, j));
      }
    }

    fusion.SetExtrinsics(extrinsics);
    fusion.SetIntrinsics(intrinsics_eigen);

    // Prepare image structure
    mask_lidar_fusion::Image mask_image;
    mask_image.width = mask_bgr.cols;
    mask_image.height = mask_bgr.rows;
    mask_image.data.assign(mask_bgr.data, mask_bgr.data + mask_bgr.total() * mask_bgr.channels());

    // Fuse mask with point cloud
    labeled_cloud_->clear();
    labeled_cloud_ = fusion.fuse(mask_image, pcl_cloud_);

    // 5. Convert back to ROS PointCloud2
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*labeled_cloud_, output_msg);
    output_msg.header = input_cloud.header;

    // 6. Publish labeled point cloud
    labeled_pcl_pub_->publish(output_msg);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(get_logger(), "Could not get transform: %s", ex.what());
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(), "Error in label_point_cloud: %s", ex.what());
  }
}
}  // namespace camera_lidar_fusion

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<camera_lidar_fusion::CameraLidarFusionNode>(rclcpp::NodeOptions());
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(camera_lidar_fusion::CameraLidarFusionNode)