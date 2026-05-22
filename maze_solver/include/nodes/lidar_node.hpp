#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <mutex>

namespace nodes {

    // Information from LIDAR
    struct LidarSectors {
        // Main sectors
        float front = std::numeric_limits<float>::infinity();
        float back  = std::numeric_limits<float>::infinity();
        float left  = std::numeric_limits<float>::infinity();
        float right = std::numeric_limits<float>::infinity();

        // Diagonals
        float front_left  = std::numeric_limits<float>::infinity();
        float front_right = std::numeric_limits<float>::infinity();
        float back_left   = std::numeric_limits<float>::infinity();
        float back_right  = std::numeric_limits<float>::infinity();
    };

    class LidarNode : public rclcpp::Node {
    public:
        LidarNode();
        ~LidarNode() override = default;

        float get_front() const;
        float get_back()  const;
        float get_left()  const;
        float get_right() const;

        LidarSectors get_sectors() const;

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;

        LidarSectors sectors_;

        mutable std::mutex mutex_;

        void on_lidar_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        float compute_sector_median(const sensor_msgs::msg::LaserScan::SharedPtr & msg, float center, float half_width) const;
    };

}