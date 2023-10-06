#pragma once

#include <iostream>
#include <optional>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"

namespace pc2_combiner
{

    struct Point
    {
        float x;
        float y;
        float z;
        float intensity;
        unsigned short ring;

        void transform(const geometry_msgs::msg::TransformStamped& t_transform)
        {
            const tf2::Quaternion rotation{ t_transform.transform.rotation.x,
                                            t_transform.transform.rotation.y,
                                            t_transform.transform.rotation.z,
                                            t_transform.transform.rotation.w };
            const tf2::Vector3 translation{ t_transform.transform.translation.x,
                                            t_transform.transform.translation.y,
                                            t_transform.transform.translation.z };
            const tf2::Transform transform{ rotation, translation };

            const tf2::Vector3 old_point{ x, y, z };
            const tf2::Vector3 new_point{ transform(old_point) };

            x = new_point.getX();
            y = new_point.getY();
            z = new_point.getZ();
        }
    };

/**
 * @brief This class takes the reference of an existing const PointCloud2 message and enables
 * iterating its points.
 *
 */
    class PointCloudConst
    {
    public:
        /**
         * @brief Constructs a new PointCloud object with the reference of an existing PointCloud2 msg.
         *
         * @param t_pointcloud2 The PointCloud2 message.
         */
        PointCloudConst(const sensor_msgs::msg::PointCloud2& t_pointcloud2)
                : m_pointcloud2{ t_pointcloud2 }
                , m_point_count{ m_pointcloud2.height * m_pointcloud2.width }
                , m_current_index{ 0 }
                , m_iter_x{ m_pointcloud2, "x" }
                , m_iter_y{ m_pointcloud2, "y" }
                , m_iter_z{ m_pointcloud2, "z" }
                , m_iter_intensity{ m_pointcloud2, "intensity" }
                , m_iter_ring{ m_pointcloud2, "ring" }
        {
        }

        /**
         * @brief Get the point count of the cloud.
         *
         * @return Point count.
         */
        unsigned int getPointCount() const
        {
            return m_point_count;
        }

        /**
         * @brief Get the current point of the cloud.
         *
         * @return Current point.
         */
        Point getCurrentPoint() const
        {
            const Point point{ *m_iter_x, *m_iter_y, *m_iter_z, *m_iter_intensity, *m_iter_ring };
            return point;
        }

        /**
         * @brief Increases iterators by 1 to go to next point.
         *
         */
        void nextPoint()
        {
            if (m_current_index < m_point_count)
            {
                ++m_current_index;
                ++m_iter_x;
                ++m_iter_y;
                ++m_iter_z;
                ++m_iter_intensity;
                ++m_iter_ring;
            }
        }

        size_t getCurrentIndex() const
        {
            return m_current_index;
        }

    private:
        // Reference of the existing PointCloud2.
        const sensor_msgs::msg::PointCloud2& m_pointcloud2;

        // Number of the points in the cloud.
        const unsigned int m_point_count;

        // Index of the current point.
        size_t m_current_index;

        // PointCloud2 iterators.
        sensor_msgs::PointCloud2ConstIterator<float> m_iter_x;
        sensor_msgs::PointCloud2ConstIterator<float> m_iter_y;
        sensor_msgs::PointCloud2ConstIterator<float> m_iter_z;
        sensor_msgs::PointCloud2ConstIterator<float> m_iter_intensity;
        sensor_msgs::PointCloud2ConstIterator<unsigned short> m_iter_ring;
    };

/**
 * @brief This class takes the reference of an existing const PointCloud2 message and enables
 * iterating its points.
 *
 */
    class PointCloud
    {
    public:
        /**
         * @brief Constructs a new PointCloud object with the reference of an existing PointCloud2 msg.
         *
         * @param t_pointcloud2 The PointCloud2 message.
         */
        PointCloud(sensor_msgs::msg::PointCloud2& t_pointcloud2)
                : m_pointcloud2{ t_pointcloud2 }
                , m_point_count{ m_pointcloud2.height * m_pointcloud2.width }
                , m_current_index{ 0 }
        {
        }

        /**
         * @brief Get the point count of the cloud.
         *
         * @return Point count.
         */
        unsigned int getPointCount() const
        {
            return m_point_count;
        }

        /**
         * @brief Get the current point of the cloud.
         *
         * @return Current point.
         */
        Point getCurrentPoint() const
        {
            const Point point{ *(*m_iter_x), *(*m_iter_y), *(*m_iter_z), *(*m_iter_intensity),
                               *(*m_iter_ring) };
            return point;
        }

        /**
         * @brief Increases iterators by 1 to go to next point.
         *
         */
        void nextPoint()
        {
            if (m_current_index < m_point_count)
            {
                ++*(m_iter_x);
                ++*(m_iter_y);
                ++*(m_iter_z);
                ++*(m_iter_intensity);
                ++*(m_iter_ring);
            }
        }

        /**
         * @brief Sets PointCloud2 fileds and resizes.
         *
         * @param t_point_count Number of points.
         *
         */
        void setFieldsAndResize(const size_t& t_point_count)
        {
            m_point_count = t_point_count;
            sensor_msgs::PointCloud2Modifier modifier{ m_pointcloud2 };
            modifier.resize(m_point_count);
            modifier.setPointCloud2Fields(5, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                          sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                          sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                                          sensor_msgs::msg::PointField::FLOAT32, "ring", 1,
                                          sensor_msgs::msg::PointField::UINT16);

            m_iter_x.emplace(m_pointcloud2, "x");
            m_iter_y.emplace(m_pointcloud2, "y");
            m_iter_z.emplace(m_pointcloud2, "z");
            m_iter_intensity.emplace(m_pointcloud2, "intensity");
            m_iter_ring.emplace(m_pointcloud2, "ring");

            m_pointcloud2.header.frame_id = "base_link";
        }

        /**
         * @brief Sets the current point.
         *
         * @param t_point The point to set.
         */
        void setCurrentPoint(const Point& t_point)
        {
            *(*m_iter_x) = t_point.x;
            *(*m_iter_y) = t_point.y;
            *(*m_iter_z) = t_point.z;
            *(*m_iter_intensity) = t_point.intensity;
            *(*m_iter_ring) = t_point.ring;
        }

        void append(const Point& t_point)
        {
            setCurrentPoint(t_point);
            nextPoint();
        }

    private:
        // Reference of the existing PointCloud2.
        sensor_msgs::msg::PointCloud2& m_pointcloud2;

        // Number of the points in the cloud.
        unsigned int m_point_count;

        // Index of the current point.
        size_t m_current_index;

        // PointCloud2 iterators.
        std::optional<sensor_msgs::PointCloud2Iterator<float>> m_iter_x;
        std::optional<sensor_msgs::PointCloud2Iterator<float>> m_iter_y;
        std::optional<sensor_msgs::PointCloud2Iterator<float>> m_iter_z;
        std::optional<sensor_msgs::PointCloud2Iterator<float>> m_iter_intensity;
        std::optional<sensor_msgs::PointCloud2Iterator<unsigned short>> m_iter_ring;
    };

}  // namespace pc2_combiner