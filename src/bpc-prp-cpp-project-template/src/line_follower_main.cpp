#include <rclcpp/rclcpp.hpp>
#include "nodes/line_follower_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<nodes::LineFollowerNode>());
    rclcpp::shutdown();
    return 0;
}