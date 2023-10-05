//
// Created by bzeren on 05.10.2023.
//

#include "color_point_cloud/ColorPointCloud.hpp"

namespace color_point_cloud {
    ColorPointCloud::ColorPointCloud(const rclcpp::NodeOptions &options) : Node("color_point_cloud_node", options), transform_provider_ptr_(std::make_shared<TransformProvider>(this->get_clock())) {
        RCLCPP_INFO(this->get_logger(), "ColorPointCloud node started");

        // Declare and get parameters
        {
            // timeout_sec
            this->declare_parameter<double>("timeout_sec", 0.1);
            timeout_sec_ = this->get_parameter("timeout_sec").as_double();

            // camera_topics
            this->declare_parameter<std::vector<std::string>>("camera_topics", std::vector<std::string>());
            camera_topics_ = this->get_parameter("camera_topics").as_string_array();

            for (auto &camera_topic: camera_topics_) {
                RCLCPP_INFO(this->get_logger(), "camera_topic: %s", camera_topic.c_str());
            }
        }

        // Create camera object, create subscriber to image and camera_info
        {
            for (const auto &camera_topic: camera_topics_) {
                std::string image_topic = camera_topic + "/image";
                std::string camera_info_topic = camera_topic + "/camera_info";

                CameraTypePtr camera_type_ptr = std::make_shared<CameraType>(image_topic, camera_info_topic);
                camera_type_stdmap_[camera_topic] = camera_type_ptr;

                this->image_subscribers_.push_back(this->create_subscription<sensor_msgs::msg::Image>(
                        image_topic, 1,
                        [this, image_topic, camera_topic](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
                            // RCLCPP_INFO(this->get_logger(), "Received image on topic %s", camera_topic.c_str());
                            camera_type_stdmap_[camera_topic]->set_image(msg);
                        }));

                this->camera_info_subscribers_.push_back(this->create_subscription<sensor_msgs::msg::CameraInfo>(
                        camera_info_topic, 1, [this, camera_info_topic, camera_topic](
                                const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
                            // RCLCPP_INFO(this->get_logger(), "Received camera_info on topic %s", camera_topic.c_str());
                            camera_type_stdmap_[camera_topic]->set_camera_info(msg);
                        }));
            }
        }

        // Set timer
        {
            const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(timeout_sec_));
            timer_ = rclcpp::create_timer(this, this->get_clock(), period_ns,
                                          std::bind(&ColorPointCloud::timer_callback, this));
        }

    }

    void ColorPointCloud::timer_callback() {
        for (const auto &pair: camera_type_stdmap_) {
            if (pair.second->get_image() == nullptr || pair.second->get_camera_info() == nullptr) {
                RCLCPP_INFO(this->get_logger(), "Image or camera info is null for topic: %s", pair.first.c_str());
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            if (!pair.second->is_info_initialized()) {
                RCLCPP_INFO(this->get_logger(), "Camera info is setting: %s", pair.first.c_str());
                pair.second->set_camera_utils(pair.second->get_camera_info());
            }

            if (!pair.second->is_transform_initialized() && pair.second->is_info_initialized()) {
                geometry_msgs::msg::TransformStamped transform = (*transform_provider_ptr_)("CAM_F/camera_link", "lidar");
                pair.second->set_lidar_to_camera_matrix(transform);
            }

            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(pair.second->get_image(), pair.second->get_image()->encoding);
                cv::imshow(pair.first, cv_ptr->image);
                cv::waitKey(1);
            } catch (cv_bridge::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
        }
    }
} // namespace color_point_cloud