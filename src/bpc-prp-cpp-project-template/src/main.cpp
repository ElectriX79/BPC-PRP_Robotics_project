#include <rclcpp/rclcpp.hpp>
#include "nodes/line_follower_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto follower = std::make_shared<nodes::LineFollowerNode>();
    executor->add_node(follower);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Line follower started. Press Ctrl+C to stop.");
    executor->spin();

    rclcpp::shutdown();
    return 0;
}