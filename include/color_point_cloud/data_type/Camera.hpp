//
// Created by bzeren on 05.10.2023.
//

#pragma once

#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>

#include <Eigen/Dense>

namespace color_point_cloud {
    class CameraType {
    public:
        CameraType(std::string image_topic, std::string camera_info_topic) :
                image_topic_(std::move(image_topic)), camera_info_topic_(std::move(camera_info_topic)),
                is_info_initialized_(false), is_transform_initialized_(false) {

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

        bool is_info_initialized() {
            return is_info_initialized_;
        }

        bool is_transform_initialized() {
            return is_transform_initialized_;
        }

        void set_camera_utils(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
            camera_frame_id_ = msg->header.frame_id;
            distortion_model_ = msg->distortion_model;

            image_width_ = msg->width;
            image_height_ = msg->height;

            camera_matrix_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(msg->k.data());
            rectification_matrix_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(msg->r.data());
            projection_matrix_ = Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(msg->p.data());

            distortion_matrix_ = Eigen::Map<const Eigen::Matrix<double, 1, 5, Eigen::RowMajor>>(msg->d.data());

            is_info_initialized_ = true;
        }

        void set_lidar_to_camera_matrix(geometry_msgs::msg::TransformStamped &msg) {
            lidar_to_camera_matrix_ = Eigen::Matrix<double, 4, 4>::Identity(4, 4);

            Eigen::Quaterniond q(msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z);
            lidar_to_camera_matrix_.block<3, 3>(0, 0) = q.toRotationMatrix();
            lidar_to_camera_matrix_(0, 3) = msg.transform.translation.x;
            lidar_to_camera_matrix_(1, 3) = msg.transform.translation.y;
            lidar_to_camera_matrix_(2, 3) = msg.transform.translation.z;
        }

        std::string get_image_topic() {
            return image_topic_;
        }

        std::string get_camera_info_topic() {
            return camera_info_topic_;
        }

        std::string get_camera_frame_id() {
            return camera_frame_id_;
        }

        std::string get_distortion_model() {
            return distortion_model_;
        }

        double get_image_width() {
            return image_width_;
        }

        double get_image_height() {
            return image_height_;
        }

        Eigen::Matrix<double, 3, 3> get_camera_matrix() {
            return camera_matrix_;
        }

        Eigen::Matrix<double, 3, 3> get_rectification_matrix() {
            return rectification_matrix_;
        }

        Eigen::Matrix<double, 3, 4> get_projection_matrix() {
            return projection_matrix_;
        }

        Eigen::Matrix<double, 1, 5> get_distortion_matrix() {
            return distortion_matrix_;
        }

        Eigen::Matrix4d get_lidar_to_camera_matrix() {
            return lidar_to_camera_matrix_;
        }

    private:
        std::string image_topic_;
        std::string camera_info_topic_;

        sensor_msgs::msg::Image::ConstSharedPtr image_;
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;

        bool is_info_initialized_;
        bool is_transform_initialized_;

        std::string camera_frame_id_;
        std::string distortion_model_;

        double image_width_;
        double image_height_;

        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix_;
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rectification_matrix_;
        Eigen::Matrix<double, 3, 4, Eigen::RowMajor> projection_matrix_;
        Eigen::Matrix<double, 1, 5, Eigen::RowMajor> distortion_matrix_;

        Eigen::Matrix4d lidar_to_camera_matrix_;
    };

    typedef std::shared_ptr<CameraType> CameraTypePtr;
    typedef std::shared_ptr<const CameraType> CameraTypeConstPtr;

} // namespace color_point_cloud

