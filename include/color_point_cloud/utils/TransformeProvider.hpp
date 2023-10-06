//
// Created by bzeren on 05.10.2023.
//

#pragma once

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/transform_stamped.h>

namespace color_point_cloud {
    class TransformProvider {
    public:
        explicit TransformProvider(rclcpp::Clock::SharedPtr clock) {
            std::cout << "TransformProvider constructor" << std::endl;

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock);
            tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        }

        std::optional<geometry_msgs::msg::TransformStamped> operator()(const std::string &target_frame,
                                                        const std::string &source_frame) const {
            std::optional<geometry_msgs::msg::TransformStamped> transform_stamped;
            try {
                transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            } catch (tf2::TransformException &ex) {
//                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", ex.what());
                return std::nullopt;
            }
            return transform_stamped;
        }

    private:
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    };

    typedef std::shared_ptr<TransformProvider> TransformProviderPtr;
    typedef std::shared_ptr<const TransformProvider> TransformProviderConstPtr;

} // namespace color_point_cloud