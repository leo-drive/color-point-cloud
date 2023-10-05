//
// Created by bzeren on 05.10.2023.
//

#include "color_point_cloud/ColorPointCloud.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<color_point_cloud::ColorPointCloud>(options));
    rclcpp::shutdown();
    return 0;
}