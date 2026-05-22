#include "nodes/io_node.hpp"



namespace nodes {

    IoNode::IoNode() : Node("io_node") {

        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(

            "/bpc_prp_robot/buttons", 10,

            std::bind(&IoNode::on_button_callback, this, std::placeholders::_1));

    }



    int IoNode::get_button_pressed() const {

        return button_pressed_;

    }



    void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {

        button_pressed_ = msg->data;

        RCLCPP_INFO(this->get_logger(), "Button pressed: %d", button_pressed_);

    }

}
