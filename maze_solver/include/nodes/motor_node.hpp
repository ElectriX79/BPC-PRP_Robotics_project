#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

namespace nodes {
    class MotorNode : public rclcpp::Node {
    public:
        MotorNode();
        ~MotorNode() override = default;

        void set_motor_speeds(uint8_t left, uint8_t right);

    private:
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_publisher_;
        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr encoder_subscriber_;

        uint32_t encoder_left_ = 0;
        uint32_t encoder_right_ = 0;
        uint32_t prev_left_ = 0;
        uint32_t prev_right_ = 0;

        float pos_x_ = 0.0f;
        float pos_y_ = 0.0f;
        float theta_ = 0.0f;

        void on_encoder_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);
    };
}
