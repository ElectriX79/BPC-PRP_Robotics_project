#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

class LineNode : public rclcpp::Node
{
public:
    LineNode() : Node("line_node")
    {
        subscription_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/bpc_prp_robot/line_sensors",
            10,
            std::bind(&LineNode::line_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Line node started");
    }

private:
    void line_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
    {
        if(msg->data.size() < 2)
            return;

        uint16_t left = msg->data[0];
        uint16_t right = msg->data[1];
        
        RCLCPP_INFO(this->get_logger(), "Left_sensor data: %d",left);
        RCLCPP_INFO(this->get_logger(), "Right_sensor data: %d", right);

  
   
    }
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineNode>());
    rclcpp::shutdown();
    return 0;
}
