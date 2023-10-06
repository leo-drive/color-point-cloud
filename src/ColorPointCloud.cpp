//
// Created by bzeren on 05.10.2023.
//

#include "color_point_cloud/ColorPointCloud.hpp"

namespace color_point_cloud {
    ColorPointCloud::ColorPointCloud(const rclcpp::NodeOptions &options) : Node("color_point_cloud_node", options),
                                                                           transform_provider_ptr_(
                                                                                   std::make_shared<TransformProvider>(
                                                                                           this->get_clock())) {
        RCLCPP_INFO(this->get_logger(), "ColorPointCloud node started");

        // Declare and get parameters
        {
            // timeout_sec
            this->declare_parameter<double>("timeout_sec", 0.1);
            timeout_sec_ = this->get_parameter("timeout_sec").as_double();

            this->declare_parameter<std::string>("point_cloud_topic", "/points_raw");
            point_cloud_topic_ = this->get_parameter("point_cloud_topic").as_string();

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
                        image_topic, rclcpp::SensorDataQoS(),
                        [this, image_topic, camera_topic](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
                            // RCLCPP_INFO(this->get_logger(), "Received image on topic %s", camera_topic.c_str());
                            camera_type_stdmap_[camera_topic]->set_image(msg);
                        }));

                this->camera_info_subscribers_.push_back(this->create_subscription<sensor_msgs::msg::CameraInfo>(
                        camera_info_topic, rclcpp::SensorDataQoS(), [this, camera_info_topic, camera_topic](
                                const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
                            // RCLCPP_INFO(this->get_logger(), "Received camera_info on topic %s", camera_topic.c_str());
                            camera_type_stdmap_[camera_topic]->set_camera_info(msg);
                        }));
            }
        }

        {
            point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    point_cloud_topic_, rclcpp::SensorDataQoS(),
                    std::bind(&ColorPointCloud::point_cloud_callback, this, std::placeholders::_1));

            point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "/points_color", rclcpp::SensorDataQoS());
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
//                RCLCPP_INFO(this->get_logger(), "Image or camera info is null for topic: %s", pair.first.c_str());
//                rclcpp::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            if (!pair.second->is_info_initialized()) {
                RCLCPP_INFO(this->get_logger(), "Camera info is setting: %s", pair.first.c_str());
                pair.second->set_camera_utils(pair.second->get_camera_info());
            }

            if (!pair.second->is_transform_initialized() && pair.second->is_info_initialized()) {
                std::optional<geometry_msgs::msg::TransformStamped> transform = (*transform_provider_ptr_)(
                        "CAM_F/camera_link", "lidar");
                if (!transform.has_value()) {
                    continue;
                }
                pair.second->set_lidar_to_camera_matrix(transform.value());
            }

//            cv_bridge::CvImageConstPtr cv_ptr;
//            try {
//                cv_ptr = cv_bridge::toCvShare(pair.second->get_image(), pair.second->get_image()->encoding);
//                cv::imshow(pair.first, cv_ptr->image);
//                cv::waitKey(1);
//
////                pair.second->set_image(nullptr);
////                pair.second->set_camera_info(nullptr);
//            } catch (cv_bridge::Exception &e) {
//                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//                return;
//            }
        }
    }

    void ColorPointCloud::point_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
        RCLCPP_INFO(this->get_logger(), "Received point cloud on topic %s", point_cloud_topic_.c_str());

        for (const auto &pair: camera_type_stdmap_) {
            if (pair.second->get_image() == nullptr || pair.second->get_camera_info() == nullptr ||
                !pair.second->is_info_initialized() || !pair.second->is_transform_initialized()) {
                RCLCPP_INFO(this->get_logger(),
                            "Cant project camera: %s \n\n Expected reasons: \n don't receive image \n don't receive \n can't get transforme \n can't init camera utils",
                            pair.first.c_str());
                continue;
            }

            cv_bridge::CvImageConstPtr cv_ptr;
            cv_ptr = cv_bridge::toCvShare(pair.second->get_image(), pair.second->get_image()->encoding);

            sensor_msgs::msg::PointCloud2 cloud_color_msg;
            sensor_msgs::PointCloud2Modifier modifier(cloud_color_msg);
            modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
            modifier.resize(msg->width * msg->height);

            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_color_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_color_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_color_msg, "z");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_color_msg, "r");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_color_msg, "g");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_color_msg, "b");

            pc2_combiner::PointCloudConst cloud{*msg};
            for (size_t i = 0; i < cloud.getPointCount(); ++i) {
                pc2_combiner::Point point{cloud.getCurrentPoint()};

                Eigen::Vector4d point4d(point.x, point.y, point.z, 1.0);
                Eigen::Vector4d point4d_transformed = pair.second->get_lidar_to_camera_matrix() * point4d;

                Eigen::Vector3d point3d_transformed_camera = pair.second->get_projection_matrix() * point4d_transformed;

                Eigen::Vector2d point2d_transformed_camera = Eigen::Vector2d(
                        point3d_transformed_camera[0] / point3d_transformed_camera[2],
                        point3d_transformed_camera[1] / point3d_transformed_camera[2]);

                double x = point2d_transformed_camera[0];
                double y = point2d_transformed_camera[1];

                if (x < 0 || x > pair.second->get_image_width() || y < 0 || y > pair.second->get_image_height() || point3d_transformed_camera[2] < 0) {

                    iter_x[0] = point.x;
                    iter_y[0] = point.y;
                    iter_z[0] = point.z;

                    iter_r[0] = 0;
                    iter_g[0] = 0;
                    iter_b[0] = 0;

                } else {
                    cv::Vec3d color = cv_ptr->image.at<cv::Vec3b>(cv::Point(x, y));
                    cv::Scalar color_scalar(color[0], color[1], color[2]);

                    cv::circle(cv_ptr->image, cv::Point(x, y), 1, color_scalar, 1);

                    iter_x[0] = point.x;
                    iter_y[0] = point.y;
                    iter_z[0] = point.z;

                    iter_r[0] = color[2];
                    iter_g[0] = color[1];
                    iter_b[0] = color[0];
                }

                ++iter_x;
                ++iter_y;
                ++iter_z;
                ++iter_r;
                ++iter_g;
                ++iter_b;

                cloud.nextPoint();
            }
            cloud_color_msg.header = msg->header;
            cloud_color_msg.height = 1;
            cloud_color_msg.width = msg->width * msg->height;
            point_cloud_publisher_->publish(cloud_color_msg);

            cv::imshow(pair.first, cv_ptr->image);
            cv::waitKey(1);

        }
    }
} // namespace color_point_cloud