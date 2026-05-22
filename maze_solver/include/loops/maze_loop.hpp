#pragma once

#include <rclcpp/rclcpp.hpp>
#include "nodes/lidar_node.hpp"
#include "nodes/motor_node.hpp"
#include "nodes/imu_node.hpp"
#include "nodes/camera_node.hpp"

namespace loops {

    enum class NavState {
        WAITING,
        CALIBRATION,
        CORRIDOR_FOLLOWING,
        TURNING,
        POST_TURN_DRIVE
    };

    enum class TurnDirection {
        LEFT,
        RIGHT
    };

    class CorridorLoop : public rclcpp::Node {
    public:
        CorridorLoop(std::shared_ptr<nodes::LidarNode> lidar_node, std::shared_ptr<nodes::MotorNode> motor_node, std::shared_ptr<nodes::ImuNode> imu_node, std::shared_ptr<CameraNode> camera_node);

    private:
        std::shared_ptr<nodes::LidarNode> lidar_node_;
        std::shared_ptr<nodes::MotorNode> motor_node_;
        std::shared_ptr<nodes::ImuNode>   imu_node_;
        std::shared_ptr<CameraNode>       camera_node_;
        rclcpp::TimerBase::SharedPtr timer_;

        // States
        NavState state_ = NavState::WAITING;
        rclcpp::Time start_time_;
        rclcpp::Time state_enter_time_;
        rclcpp::Time motors_stopped_at_;
        TurnDirection turn_direction_ = TurnDirection::LEFT;
        float yaw_at_turn_start_ = 0.0f;

        float target_yaw_ = 0.0f;

        //  ArUco marker variables
        bool pending_turn_valid_ = false;
        TurnDirection pending_turn_direction_ = TurnDirection::LEFT;
        int pending_turn_marker_id_ = -1;
        rclcpp::Time pending_turn_seen_at_;
        float pending_turn_max_age_sec_ = 30.0f;
        std::string pending_turn_instruction_;

        // Detection of side branch
        float side_branch_open_threshold_ = 0.40f;
        float side_branch_center_delay_sec_ = 0.5f;

        bool side_branch_detected_ = false;
        rclcpp::Time side_branch_detected_at_;

        float base_speed_ = 155.0f;
        // Variables for PID of IMU
        float yaw_kp_ = 0.1f;
        float yaw_kd_ = 0.05f;
        float yaw_max_correction_ = 3.5f;

        // Variables for additional lateral correction
        float wall_target_     = 0.20f;
        float lateral_kp_      = 30.0f;
        float lateral_max_correction_ = 5.0f;
        float lateral_dead_zone_ = 0.04f;

        float wall_lost_threshold_ = 0.40f;

        // State of PID of IMU
        float yaw_prev_err_ = 0.0f;
        rclcpp::Time last_pid_time_;

        // Frontal wall detection
        float corner_front_threshold_ = 0.25f;

        // Variables for 90 degrees turning
        float turn_target_angle_deg_ = 90.0f;
        float turn_tolerance_deg_    = 2.0f;
        float turn_slowdown_start_deg_ = 30.0f;
        uint8_t turn_speed_max_ = 30;
        uint8_t turn_speed_min_ = 8;
        float   turn_timeout_sec_ = 4.0f;
        float   settle_time_sec_  = 0.3f;

        bool turn_motors_stopped_ = false;
        rclcpp::Time turn_stopped_at_;
        float turn_final_delta_ = 0.0f;

        float post_turn_duration_ = 0.5f;

        void timer_callback();

        void update_pending_turn_from_camera();

        void handle_waiting(double elapsed_total);
        void handle_calibration();
        void handle_corridor_following(const nodes::LidarSectors& s);
        void handle_turning(const nodes::LidarSectors& s);
        void handle_post_turn_drive(const nodes::LidarSectors& s);
        void drive_yaw_based(const nodes::LidarSectors& s);

        void transition_to(NavState new_state, const char* name);

        void snap_target_yaw_to_grid();
    };

}