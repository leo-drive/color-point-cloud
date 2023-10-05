cmake_minimum_required(VERSION 3.8)
project(color_point_cloud)

find_package(OpenCV REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(color_point_cloud
        src/color_point_cloud_node.cpp
        src/ColorPointCloud.cpp)

ament_target_dependencies(color_point_cloud
        rclcpp
        sensor_msgs
        geometry_msgs
        cv_bridge
        tf2_ros
)

target_link_libraries(color_point_cloud
        ${OpenCV_LIBRARIES}
)

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS color_point_cloud
        DESTINATION lib/${PROJECT_NAME})

ament_package()
