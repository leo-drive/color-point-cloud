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

#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Dense>

namespace color_point_cloud {
    class CameraType {
    public:
        CameraType(std::string image_topic, std::string camera_info_topic) :
                image_topic_(std::move(image_topic)), camera_info_topic_(std::move(camera_info_topic)),
                is_info_initialized_(false), is_transform_initialized_(false), image_width_(0), image_height_(0) {

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

        bool is_info_initialized() const {
            return is_info_initialized_;
        }

        bool is_transform_initialized() const {
            return is_transform_initialized_;
        }

        void set_camera_utils(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
            camera_frame_id_ = msg->header.frame_id;
            distortion_model_ = msg->distortion_model;

            image_width_ = msg->width;
            image_height_ = msg->height;

            camera_matrix_(0,0) = msg->k[0];
            camera_matrix_(0,2) = msg->k[2];
            camera_matrix_(1,1) = msg->k[4];
            camera_matrix_(1,2) = msg->k[5];
            camera_matrix_(2,2) = 1;

            rectification_matrix_(0,0) = msg->r[0];
            rectification_matrix_(0,1) = msg->r[1];
            rectification_matrix_(0,2) = msg->r[2];
            rectification_matrix_(1,0) = msg->r[3];
            rectification_matrix_(1,1) = msg->r[4];
            rectification_matrix_(1,2) = msg->r[5];
            rectification_matrix_(2,0) = msg->r[6];
            rectification_matrix_(2,1) = msg->r[7];
            rectification_matrix_(2,2) = msg->r[8];

            projection_matrix_(0,0) = msg->p[0];
            projection_matrix_(0,2) = msg->p[2];
            projection_matrix_(1,1) = msg->p[5];
            projection_matrix_(1,2) = msg->p[6];
            projection_matrix_(2,2) = 1;

            distortion_matrix_(0,0) = msg->d[0];
            distortion_matrix_(0,1) = msg->d[1];
            distortion_matrix_(0,2) = msg->d[2];
            distortion_matrix_(0,3) = msg->d[3];
            distortion_matrix_(0,4) = msg->d[4];

            is_info_initialized_ = true;
        }

        void set_lidar_to_camera_matrix(geometry_msgs::msg::TransformStamped &msg) {
            Eigen::Affine3d affine_lidar_to_camera_matrix = tf2::transformToEigen(msg);

            lidar_to_camera_matrix_ = Eigen::Matrix4d::Identity();
            lidar_to_camera_matrix_ = affine_lidar_to_camera_matrix.matrix();

            is_transform_initialized_ = true;
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

        double get_image_width() const {
            return image_width_;
        }

        double get_image_height() const {
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

        Eigen::Matrix<double, 3, 3> camera_matrix_;
        Eigen::Matrix<double, 3, 3> rectification_matrix_;
        Eigen::Matrix<double, 3, 4> projection_matrix_;
        Eigen::Matrix<double, 1, 5> distortion_matrix_;

        Eigen::Matrix4d lidar_to_camera_matrix_;
    };

    typedef std::shared_ptr<CameraType> CameraTypePtr;
    typedef std::shared_ptr<const CameraType> CameraTypeConstPtr;

} // namespace color_point_cloud

