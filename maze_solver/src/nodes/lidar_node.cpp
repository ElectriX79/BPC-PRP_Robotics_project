#include "nodes/lidar_node.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace nodes {

    LidarNode::LidarNode() : Node("lidar_node") {
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/bpc_prp_robot/lidar", 10, std::bind(&LidarNode::on_lidar_scan, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "LidarNode started (sensor-only mode)");
    }

    // ---- Gettery (vlákno-bezpečné) ----
    float LidarNode::get_front() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return sectors_.front;
    }
    float LidarNode::get_back() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return sectors_.back;
    }
    float LidarNode::get_left() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return sectors_.left;
    }
    float LidarNode::get_right() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return sectors_.right;
    }
    LidarSectors LidarNode::get_sectors() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return sectors_;
    }

    void LidarNode::on_lidar_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        constexpr float half_width = 0.087f;     // ±5° sektor
        constexpr float front_center =  M_PI;    // front of robot = π
        constexpr float back_center  =  0.0f;
        constexpr float left_center  = -M_PI_2;  // -90°
        constexpr float right_center =  M_PI_2;  // +90°

        constexpr float front_left_center  = -2.3562f; // -135°
        constexpr float front_right_center =  2.3562f; // +135°
        constexpr float back_left_center   = -M_PI_4;  // -45°
        constexpr float back_right_center  =  M_PI_4;  // +45°

        LidarSectors new_sectors;
        new_sectors.front = compute_sector_median(msg, front_center, half_width);
        new_sectors.back  = compute_sector_median(msg, back_center,  half_width);
        new_sectors.left  = compute_sector_median(msg, left_center,  half_width);
        new_sectors.right = compute_sector_median(msg, right_center, half_width);

        new_sectors.front_left  = compute_sector_median(msg, front_left_center,  half_width);
        new_sectors.front_right = compute_sector_median(msg, front_right_center, half_width);
        new_sectors.back_left   = compute_sector_median(msg, back_left_center,   half_width);
        new_sectors.back_right  = compute_sector_median(msg, back_right_center,  half_width);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            sectors_ = new_sectors;
        }

        RCLCPP_DEBUG(this->get_logger(), "Sectors [m]: F=%.2f B=%.2f L=%.2f R=%.2f | FL=%.2f FR=%.2f BL=%.2f BR=%.2f", new_sectors.front, new_sectors.back, new_sectors.left, new_sectors.right, new_sectors.front_left, new_sectors.front_right, new_sectors.back_left, new_sectors.back_right);
    }

    float LidarNode::compute_sector_median(
        const sensor_msgs::msg::LaserScan::SharedPtr & msg,
        float center,
        float half_width) const
    {
        std::vector<float> valid;
        valid.reserve(64);

        auto angle_diff = [](float a, float b) {
            float d = a - b;
            while (d >  M_PI) d -= 2.0f * M_PI;
            while (d < -M_PI) d += 2.0f * M_PI;
            return d;
        };

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float angle = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
            float range = msg->ranges[i];

            if (std::isnan(range) || std::isinf(range)) continue;
            if (range < msg->range_min || range > msg->range_max) continue;

            if (std::fabs(angle_diff(angle, center)) <= half_width) {
                valid.push_back(range);
            }
        }

        if (valid.empty()) {
            return std::numeric_limits<float>::infinity();
        }

        std::sort(valid.begin(), valid.end());
        return valid[valid.size() / 2];
    }

}