int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<nodes::LidarNode>());
    rclcpp::shutdown();
    return 0;
}
