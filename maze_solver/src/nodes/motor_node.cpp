#include "nodes/motor_node.hpp"

namespace nodes {
    MotorNode::MotorNode() : Node("motor_node") {
        motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/set_motor_speeds", 10);
    }

    void MotorNode::set_motor_speeds(uint8_t left, uint8_t right) {
        std_msgs::msg::UInt8MultiArray msg;
        msg.data = {left, right};
        motor_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Motors: left=%d right=%d", left, right);
    }
}

