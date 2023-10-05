//
// Created by bzeren on 05.10.2023.
//

#pragma once

#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace color_point_cloud {
    class CameraType {
    public:
        CameraType(std::string image_topic, std::string camera_info_topic) :
                image_topic_(std::move(image_topic)), camera_info_topic_(std::move(camera_info_topic)) {

        }

        void set_image(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
            image_ = msg;
        }

        void set_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
            camera_info_ = msg;
        }

        sensor_msgs::msg::Image::ConstSharedPtr get_image() {
            return image_;
        }

        sensor_msgs::msg::CameraInfo::ConstSharedPtr get_camera_info() {
            return camera_info_;
        }

    private:
        std::string image_topic_;
        std::string camera_info_topic_;

        sensor_msgs::msg::Image::ConstSharedPtr image_;
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
    };

    typedef std::shared_ptr<CameraType> CameraTypePtr;
    typedef std::shared_ptr<const CameraType> CameraTypeConstPtr;

} // namespace color_point_cloud

