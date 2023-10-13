//
// Created by bzeren on 05.10.2023.
//

#pragma once

#include "color_point_cloud/data_type/Camera.hpp"
#include "color_point_cloud/data_type/PointCloudType.hpp"
#include "color_point_cloud/utils/TransformeProvider.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>

namespace color_point_cloud {
    class ColorPointCloud : public rclcpp::Node {
    public:
        explicit ColorPointCloud(const rclcpp::NodeOptions &options);

    private:
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

        double timeout_sec_;

        std::string point_cloud_topic_;

        std::string point_cloud_frame_id_;

        ImageType image_type_;

        std::string image_topic_last_name_;

        std::string camera_info_topic_last_name_;

        std::vector<std::string> camera_topics_;

        std::map<std::string, CameraTypePtr> camera_type_stdmap_;

        std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subscribers_;
        std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subscribers_;

        void timer_callback();

        void point_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

        TransformProviderConstPtr transform_provider_ptr_;
    };
} // namespace color_point_cloud
