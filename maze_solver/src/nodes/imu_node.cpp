#include "nodes/imu_node.hpp"
#include <cmath>

namespace nodes {

    ImuNode::ImuNode() : Node("imu_node") {
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/bpc_prp_robot/imu", 10,
            std::bind(&ImuNode::on_imu_msg, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "ImuNode started (mode = IDLE)");
    }

    void ImuNode::startCalibration(size_t num_samples) {
        std::lock_guard<std::mutex> lock(mutex_);
        calibration_samples_.clear();
        calibration_samples_.reserve(num_samples);
        calibration_target_ = num_samples;
        calibrated_ = false;
        mode_ = ImuMode::CALIBRATING;
        has_last_time_ = false;
        RCLCPP_INFO(this->get_logger(), "Calibration started - collecting %zu samples. Keep robot STILL!", num_samples);
    }

    void ImuNode::startIntegration() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!calibrated_) {
            RCLCPP_WARN(this->get_logger(), "Starting integration WITHOUT calibration - yaw will drift!");
        }
        mode_ = ImuMode::INTEGRATING;
        has_last_time_ = false; // Reset dt measurement
        RCLCPP_INFO(this->get_logger(), "Integration started");
    }

    void ImuNode::stop() {
        std::lock_guard<std::mutex> lock(mutex_);
        mode_ = ImuMode::IDLE;
    }

    ImuMode ImuNode::getMode() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return mode_;
    }

    bool ImuNode::isCalibrated() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return calibrated_;
    }

    float ImuNode::getYaw() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return integrator_.getYaw();
    }

    float ImuNode::getYawDegrees() const {
        return getYaw() * 180.0f / static_cast<float>(M_PI);
    }

    void ImuNode::resetYaw() {
        std::lock_guard<std::mutex> lock(mutex_);
        integrator_.resetYaw();
    }

    void ImuNode::on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);

        float gyro_z = static_cast<float>(msg->angular_velocity.z);

        if (mode_ == ImuMode::CALIBRATING) {
            calibration_samples_.push_back(gyro_z);

            if (calibration_samples_.size() % 50 == 0) {
                RCLCPP_INFO(this->get_logger(), "Calibration progress: %zu / %zu", calibration_samples_.size(), calibration_target_);
            }

            if (calibration_samples_.size() >= calibration_target_) {
                integrator_.setCalibration(calibration_samples_);
                calibrated_ = true;
                mode_ = ImuMode::INTEGRATING;
                has_last_time_ = false;
                RCLCPP_INFO(this->get_logger(), "Calibration DONE. Offset = %.5f rad/s. Switching to INTEGRATING.", integrator_.getOffset());
            }
        }
        else if (mode_ == ImuMode::INTEGRATING) {
            rclcpp::Time now = this->get_clock()->now();

            if (!has_last_time_) {
                last_time_ = now;
                has_last_time_ = true;
                return;
            }

            double dt = (now - last_time_).seconds();
            last_time_ = now;

            if (dt <= 0.0 || dt > 0.5) {
                RCLCPP_WARN(this->get_logger(), "Ignoring sample with dt = %.3f s (out of reasonable range)", dt);
                return;
            }

            integrator_.update(gyro_z, dt);
        }
    }

}