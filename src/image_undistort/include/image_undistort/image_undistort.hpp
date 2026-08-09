#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

namespace image_undistort
{

    enum class DistortionModels
    {
        FISHEYE,
        PINHOLE
    };

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class ImageUndistort : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        explicit ImageUndistort(const rclcpp::NodeOptions &options);

    private:
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

        void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg);
        void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg);

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr undistorted_image_pub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;
        cv::Mat input_distorded_image_;
        cv::Mat undistorted_image_;

        DistortionModels distortion_model_ = DistortionModels::FISHEYE;

        std::atomic<bool> camera_info_received_{false};

        std::string image_topic_;
        std::string camera_info_topic_;
        std::string output_image_topic_;
    };
} // namespace image_undistort