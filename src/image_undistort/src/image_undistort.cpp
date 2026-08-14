#include "image_undistort/image_undistort.hpp"

namespace image_undistort
{
    static constexpr char const *const kNodeName = "image_undistort_node";
    ImageUndistort::ImageUndistort(const rclcpp::NodeOptions &options)
        : rclcpp_lifecycle::LifecycleNode(kNodeName, options)
    {
        RCLCPP_INFO(get_logger(), "ImageUndistort node created");
    }

    CallbackReturn ImageUndistort::on_configure(
        const rclcpp_lifecycle::State &)
    {
        this->declare_parameter("input_image_topic", "image");
        this->declare_parameter("camera_info_topic", "camera_info");
        this->declare_parameter("output_image_topic", "image_rect");
        this->declare_parameter("resize_output", false);

        image_topic_ = this->get_parameter("input_image_topic").as_string();
        camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
        output_image_topic_ = this->get_parameter("output_image_topic").as_string();
        resize_output_ = this->get_parameter("resize_output").as_bool();

        RCLCPP_INFO(get_logger(), "Configured %s node", kNodeName);
        
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ImageUndistort::on_activate(
        const rclcpp_lifecycle::State &)
    {
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_,
            1,
            std::bind(&ImageUndistort::image_callback, this, std::placeholders::_1),
            sub_options
        );

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_,
            1,
            std::bind(&ImageUndistort::camera_info_callback, this, std::placeholders::_1),
            sub_options
        );

        undistorted_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            output_image_topic_,
            1
        );
        RCLCPP_INFO(get_logger(), "Activated %s node", kNodeName);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ImageUndistort::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        image_sub_.reset();
        camera_info_sub_.reset();
        undistorted_image_pub_.reset();
        RCLCPP_INFO(get_logger(), "Deactivated %s node", kNodeName);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ImageUndistort::on_cleanup(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up %s node", kNodeName);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ImageUndistort::on_shutdown(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Shutting down %s node", kNodeName);
        return CallbackReturn::SUCCESS;
    }

    void ImageUndistort::image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg)
    {
        if (!camera_info_received_)
        {
            RCLCPP_WARN(get_logger(), "Camera info not received yet. Skipping image undistortion.");
            return;
        }
        
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, image_msg->encoding);

        if (distortion_model_ == DistortionModels::FISHEYE)
        {
            cv::fisheye::undistortImage(
                cv_ptr->image, undistorted_image_, camera_matrix_, dist_coeffs_,
                camera_matrix_, cv_ptr->image.size()
            );
        }
        else if (distortion_model_ == DistortionModels::PINHOLE)
        {
            cv::undistort(cv_ptr->image, undistorted_image_, camera_matrix_, dist_coeffs_);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Unsupported distortion model.");
            return;
        }

        if (resize_output_)
        {
            cv::resize(undistorted_image_, undistorted_image_, cv::Size(), 0.5, 0.5);
        }

        sensor_msgs::msg::Image::SharedPtr undistorted_image_msg = cv_bridge::CvImage(
            image_msg->header, image_msg->encoding, undistorted_image_).toImageMsg();
        undistorted_image_pub_->publish(*undistorted_image_msg);
    }

    void ImageUndistort::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg)
    {
        auto n_coeffs = camera_info_msg->d.size();
        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        dist_coeffs_ = cv::Mat(n_coeffs, 1, CV_64F);

        for (size_t row = 0; row < 3; ++row)
        {
            for (size_t col = 0; col < 3; ++col)
            {
                camera_matrix_.at<double>(row, col) = camera_info_msg->k[row * 3 + col];
            }
        }

        for (size_t i = 0; i < n_coeffs; ++i)
        {
            dist_coeffs_.at<double>(i) = camera_info_msg->d[i];
        }

        camera_info_received_ = true;
    }

} // namespace image_undistort

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<image_undistort::ImageUndistort>(rclcpp::NodeOptions());
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_undistort::ImageUndistort);