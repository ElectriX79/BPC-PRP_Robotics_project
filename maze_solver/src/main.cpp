#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "nodes/motor_node.hpp"
#include "nodes/lidar_node.hpp"
#include "nodes/imu_node.hpp"
#include "nodes/camera_node.hpp"
#include "loops/maze_loop.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto motor_node   = std::make_shared<nodes::MotorNode>();
    auto lidar_node   = std::make_shared<nodes::LidarNode>();
    auto imu_node     = std::make_shared<nodes::ImuNode>();
    auto camera_node  = std::make_shared<CameraNode>();

    auto corridor_loop = std::make_shared<loops::CorridorLoop>(lidar_node, motor_node, imu_node, camera_node);

    executor->add_node(motor_node);
    executor->add_node(lidar_node);
    executor->add_node(imu_node);
    executor->add_node(camera_node);
    executor->add_node(corridor_loop);

    RCLCPP_INFO(rclcpp::get_logger("main"),"Robot starting. Keep still for ~3 sec for IMU calibration.");

    executor->spin();
    rclcpp::shutdown();
    return 0;
}