#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vector>
#include <mutex>
#include "algorithms/planar_imu_integrator.hpp"

namespace nodes {

    // Modes of IMU node
    enum class ImuMode {
        IDLE,
        CALIBRATING,
        INTEGRATING
    };

    class ImuNode : public rclcpp::Node {
    public:
        ImuNode();
        ~ImuNode() override = default;

        void startCalibration(size_t num_samples = 200);

        void startIntegration();

        void stop();

        ImuMode getMode() const;
        bool isCalibrated() const;     // true ak sa už kalibrácia dokončila
        float getYaw() const;          // aktuálny yaw [rad]
        float getYawDegrees() const;   // pre čitateľné logy

        void resetYaw();

    private:
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

        algorithms::PlanarImuIntegrator integrator_;

        std::vector<float> calibration_samples_;
        size_t calibration_target_ = 200;

        ImuMode mode_ = ImuMode::IDLE;
        bool calibrated_ = false;

        rclcpp::Time last_time_;
        bool has_last_time_ = false;

        mutable std::mutex mutex_;

        void on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg);
    };

}