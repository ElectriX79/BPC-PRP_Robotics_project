#include "loops/maze_loop.hpp"
#include <algorithm>
#include <cmath>
#include <string>

namespace loops {

    CorridorLoop::CorridorLoop(std::shared_ptr<nodes::LidarNode> lidar_node, std::shared_ptr<nodes::MotorNode> motor_node, std::shared_ptr<nodes::ImuNode> imu_node, std::shared_ptr<CameraNode> camera_node)
        : Node("corridor_loop"),
          lidar_node_(lidar_node),
          motor_node_(motor_node),
          imu_node_(imu_node),
          camera_node_(camera_node)
    {
        start_time_ = this->get_clock()->now();
        state_enter_time_ = start_time_;
        motors_stopped_at_ = start_time_;
        last_pid_time_ = start_time_;
        turn_stopped_at_ = start_time_;
        pending_turn_seen_at_ = start_time_;

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&CorridorLoop::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "CorridorLoop started in WAITING state");
    }

    void CorridorLoop::transition_to(NavState new_state, const char* name) {
        state_ = new_state;
        state_enter_time_ = this->get_clock()->now();
        motors_stopped_at_ = state_enter_time_;
        RCLCPP_INFO(this->get_logger(), "→ STATE: %s", name);
    }

    // FSM
    void CorridorLoop::timer_callback() {
        double elapsed_total = (this->get_clock()->now() - start_time_).seconds();

        update_pending_turn_from_camera();

        switch (state_) {
            case NavState::WAITING:
                handle_waiting(elapsed_total);
                break;

            case NavState::CALIBRATION:
                handle_calibration();
                break;

            case NavState::CORRIDOR_FOLLOWING: {
                auto s = lidar_node_->get_sectors();
                handle_corridor_following(s);
                break;
            }

            case NavState::TURNING: {
                auto s = lidar_node_->get_sectors();
                handle_turning(s);
                break;
            }

            case NavState::POST_TURN_DRIVE: {
                auto s = lidar_node_->get_sectors();
                handle_post_turn_drive(s);
                break;
            }
        }
    }

    void CorridorLoop::update_pending_turn_from_camera() {
        constexpr double fresh_threshold_sec = 0.3;

        auto paired = camera_node_->get_paired_detection();
        double escape_age = camera_node_->get_escape_age_seconds(this->now());

        if (!paired.escape.valid || escape_age >= fresh_threshold_sec) {
            return;
        }

        const std::string& instr = paired.escape.instruction;

        TurnDirection new_dir;
        bool valid_instruction = true;

        if (instr == "escape_left") {
            new_dir = TurnDirection::LEFT;
        }
        else if (instr == "escape_right") {
            new_dir = TurnDirection::RIGHT;
        }
        else {
            valid_instruction = false;
        }

        if (!valid_instruction) return;

        bool is_new_marker = !pending_turn_valid_ || pending_turn_marker_id_ != paired.escape.id;

        if (is_new_marker) {
            pending_turn_valid_ = true;
            pending_turn_direction_ = new_dir;
            pending_turn_marker_id_ = paired.escape.id;
            pending_turn_seen_at_ = this->now();
            pending_turn_instruction_ = instr;

            const char* dir_str = (new_dir == TurnDirection::LEFT) ? "LEFT" : "RIGHT";
            RCLCPP_INFO(this->get_logger(), "PENDING TURN saved: %s (marker_id=%d, instr=%s)", dir_str, paired.escape.id, instr.c_str());
        }
    }

    void CorridorLoop::snap_target_yaw_to_grid() {
        float snapped = std::round(target_yaw_ / 90.0f) * 90.0f;
        while (snapped > 180.0f) snapped -= 360.0f;
        while (snapped < -180.0f) snapped += 360.0f;
        RCLCPP_INFO(this->get_logger(), "Snap target_yaw: %.1f° → %.1f°", target_yaw_, snapped);
        target_yaw_ = snapped;
    }

    void CorridorLoop::handle_waiting(double elapsed_total) {
        motor_node_->set_motor_speeds(127, 127);
        if (elapsed_total > 2.0) {
            imu_node_->startCalibration(200);
            transition_to(NavState::CALIBRATION, "CALIBRATION");
        }
    }

    void CorridorLoop::handle_calibration() {
        motor_node_->set_motor_speeds(127, 127);
        if (imu_node_->isCalibrated()) {
            RCLCPP_INFO(this->get_logger(), "IMU calibrated, starting navigation");
            target_yaw_ = 0.0f;
            yaw_prev_err_ = 0.0f;
            last_pid_time_ = this->get_clock()->now();
            transition_to(NavState::CORRIDOR_FOLLOWING, "CORRIDOR_FOLLOWING");
        }
    }

    // Corridor following
    void CorridorLoop::handle_corridor_following(const nodes::LidarSectors& s) {
        bool front_blocked = std::isfinite(s.front) && s.front < corner_front_threshold_;

        if (pending_turn_valid_ && !front_blocked) {
            bool wants_left  = (pending_turn_direction_ == TurnDirection::LEFT);
            bool wants_right = (pending_turn_direction_ == TurnDirection::RIGHT);

            bool left_diag_open  = std::isfinite(s.front_left)  && s.front_left  > side_branch_open_threshold_;
            bool right_diag_open = std::isfinite(s.front_right) && s.front_right > side_branch_open_threshold_;
            bool left_perp_open  = std::isfinite(s.left)  && s.left  > side_branch_open_threshold_;
            bool right_perp_open = std::isfinite(s.right) && s.right > side_branch_open_threshold_;

            bool left_open  = left_diag_open  && left_perp_open;
            bool right_open = right_diag_open && right_perp_open;

            bool detected_now = (wants_left && left_open) || (wants_right && right_open);

            if (detected_now && !side_branch_detected_) {
                side_branch_detected_ = true;
                side_branch_detected_at_ = this->now();
                RCLCPP_INFO(this->get_logger(), "SIDE BRANCH detected (%s open: L=%.2f R=%.2f), waiting %.2fs to center...", wants_left ? "LEFT" : "RIGHT", s.front_left, s.front_right, side_branch_center_delay_sec_);
            }

            if (side_branch_detected_) {
                double waiting_for = (this->now() - side_branch_detected_at_).seconds();
                if (waiting_for >= side_branch_center_delay_sec_) {
                    turn_direction_ = pending_turn_direction_;
                    yaw_at_turn_start_ = imu_node_->getYawDegrees();
                    motor_node_->set_motor_speeds(127, 127);

                    const char* dir_str = (turn_direction_ == TurnDirection::LEFT) ? "LEFT" : "RIGHT";
                    RCLCPP_INFO(this->get_logger(), "Side branch! L=%.2f R=%.2f → turning %s (marker_id=%d, instr=%s, yaw_start=%.1f° target_yaw=%.1f°)", s.left, s.right, dir_str, pending_turn_marker_id_, pending_turn_instruction_.c_str(), yaw_at_turn_start_, target_yaw_);
                    pending_turn_valid_ = false;
                    side_branch_detected_ = false;
                    transition_to(NavState::TURNING, "TURNING");
                    return;
                }
            }
        } else {
            side_branch_detected_ = false;
        }

        if (front_blocked) {
            side_branch_detected_ = false;

            bool decision_from_marker = false;
            bool marker_side_blocked = false;

            const float side_open_threshold = 0.40f;

            if (pending_turn_valid_) {
                double pending_age = (this->now() - pending_turn_seen_at_).seconds();
                if (pending_age < pending_turn_max_age_sec_) {
                    bool wants_left  = (pending_turn_direction_ == TurnDirection::LEFT);
                    bool wants_right = (pending_turn_direction_ == TurnDirection::RIGHT);

                    bool left_open  = std::isfinite(s.left)  && s.left  > side_open_threshold;
                    bool right_open = std::isfinite(s.right) && s.right > side_open_threshold;
                    if (!std::isfinite(s.left))  left_open = false;
                    if (!std::isfinite(s.right)) right_open = false;

                    bool marker_side_open = (wants_left && left_open) || (wants_right && right_open);

                    if (marker_side_open) {
                        turn_direction_ = pending_turn_direction_;
                        decision_from_marker = true;
                    } else {
                        marker_side_blocked = true;
                        RCLCPP_INFO(this->get_logger(), "Marker says %s but that side is BLOCKED (L=%.2f R=%.2f) - this is just a CORNER, preserving pending_turn for next junction", wants_left ? "LEFT" : "RIGHT", s.left, s.right);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "Pending turn too OLD (%.1fs), discarding", pending_age);
                    pending_turn_valid_ = false;
                }
            }

            if (!decision_from_marker) {
                float left_dist  = std::isfinite(s.left)  ? s.left  : 999.0f;
                float right_dist = std::isfinite(s.right) ? s.right : 999.0f;
                if (left_dist > right_dist) {
                    turn_direction_ = TurnDirection::LEFT;
                } else {
                    turn_direction_ = TurnDirection::RIGHT;
                }
            }

            yaw_at_turn_start_ = imu_node_->getYawDegrees();
            motor_node_->set_motor_speeds(127, 127);

            const char* dir_str = (turn_direction_ == TurnDirection::LEFT) ? "LEFT" : "RIGHT";
            const char* source;
            if (decision_from_marker)          source = "ARUCO_PENDING";
            else if (marker_side_blocked)      source = "CORNER_PRESERVE_MARKER";
            else                               source = "LIDAR_FALLBACK";

            if (decision_from_marker) {
                double pending_age = (this->now() - pending_turn_seen_at_).seconds();
                RCLCPP_INFO(this->get_logger(), "Wall ahead! front=%.2f L=%.2f R=%.2f → turning %s (source=%s, marker_id=%d, age=%.2fs, yaw_start=%.1f° target_yaw=%.1f°)", s.front, s.left, s.right, dir_str, source, pending_turn_marker_id_, pending_age, yaw_at_turn_start_, target_yaw_);
            } else {
                RCLCPP_INFO(this->get_logger(), "Wall ahead! front=%.2f L=%.2f R=%.2f → turning %s (source=%s, yaw_start=%.1f° target_yaw=%.1f°)", s.front, s.left, s.right, dir_str, source, yaw_at_turn_start_, target_yaw_);
            }

            if (decision_from_marker) {
                pending_turn_valid_ = false;
            }

            transition_to(NavState::TURNING, "TURNING");
            return;
        }

        drive_yaw_based(s);
    }

    void CorridorLoop::drive_yaw_based(const nodes::LidarSectors& s) {
        rclcpp::Time now = this->get_clock()->now();
        double dt = (now - last_pid_time_).seconds();
        last_pid_time_ = now;

        float current_yaw = imu_node_->getYawDegrees();
        float yaw_err = target_yaw_ - current_yaw;
        while (yaw_err >  180.0f) yaw_err -= 360.0f;
        while (yaw_err < -180.0f) yaw_err += 360.0f;

        float yaw_err_deriv = 0.0f;
        if (dt > 0.001 && dt < 1.0) {
            yaw_err_deriv = (yaw_err - yaw_prev_err_) / static_cast<float>(dt);
        }
        yaw_prev_err_ = yaw_err;

        float yaw_correction = yaw_kp_ * yaw_err + yaw_kd_ * yaw_err_deriv;
        yaw_correction = std::clamp(yaw_correction, -yaw_max_correction_, yaw_max_correction_);

        float lateral_correction = 0.0f;
        const char* lateral_mode = "OFF";

        if (std::fabs(yaw_err) > 5.0f) {
            int left_speed_yaw  = static_cast<int>(base_speed_ - yaw_correction);
            int right_speed_yaw = static_cast<int>(base_speed_ + yaw_correction);
            left_speed_yaw  = std::clamp(left_speed_yaw,  120, 180);
            right_speed_yaw = std::clamp(right_speed_yaw, 120, 180);

            motor_node_->set_motor_speeds(
                static_cast<uint8_t>(left_speed_yaw),
                static_cast<uint8_t>(right_speed_yaw)
            );

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300, "YAW yaw=%.1f° tgt=%.1f° err=%.1f° corr_yaw=%.2f corr_lat=0.00[YAW_PRIORITY] | L=%.2f R=%.2f F=%.2f | M=%d/%d", current_yaw, target_yaw_, yaw_err, yaw_correction, s.left, s.right, s.front, left_speed_yaw, right_speed_yaw);
            return;
        }

        float left  = s.left;
        float right = s.right;
        bool left_close  = std::isfinite(left)  && left  < wall_lost_threshold_;
        bool right_close = std::isfinite(right) && right < wall_lost_threshold_;

        if (left_close && right_close) {
            float diff = left - right;
            // Dead zone - malé rozdiely ignorujeme (šum)
            if (std::fabs(diff) > lateral_dead_zone_) {
                lateral_correction = lateral_kp_ * diff;
                lateral_correction = std::clamp(lateral_correction, -lateral_max_correction_, lateral_max_correction_);
                lateral_mode = "BOTH";
            } else {
                lateral_mode = "DEAD_ZONE";
            }
        }
        else if (left_close && !right_close) {
            float err = left - wall_target_;
            if (std::fabs(err) > lateral_dead_zone_) {
                lateral_correction = lateral_kp_ * err * 0.5f;
                lateral_correction = std::clamp(lateral_correction, -lateral_max_correction_, lateral_max_correction_);
                lateral_mode = "LEFT_ONLY";
            }
        }
        else if (right_close && !left_close) {
            float err = wall_target_ - right;
            if (std::fabs(err) > lateral_dead_zone_) {
                lateral_correction = lateral_kp_ * err * 0.5f;
                lateral_correction = std::clamp(lateral_correction, -lateral_max_correction_, lateral_max_correction_);
                lateral_mode = "RIGHT_ONLY";
            }
        }
        else {
            lateral_mode = "NO_WALLS";
        }

        float total_correction = yaw_correction + lateral_correction;

        int left_speed  = static_cast<int>(base_speed_ - total_correction);
        int right_speed = static_cast<int>(base_speed_ + total_correction);

        left_speed  = std::clamp(left_speed,  120, 180);
        right_speed = std::clamp(right_speed, 120, 180);

        motor_node_->set_motor_speeds(
            static_cast<uint8_t>(left_speed),
            static_cast<uint8_t>(right_speed)
        );

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300, "YAW yaw=%.1f° tgt=%.1f° err=%.1f° corr_yaw=%.2f corr_lat=%.2f[%s] | L=%.2f R=%.2f F=%.2f | M=%d/%d", current_yaw, target_yaw_, yaw_err, yaw_correction, lateral_correction, lateral_mode, left, right, s.front, left_speed, right_speed);
    }

    void CorridorLoop::handle_turning(const nodes::LidarSectors& s) {
        double elapsed_in_state = (this->get_clock()->now() - state_enter_time_).seconds();

        float yaw_now = imu_node_->getYawDegrees();
        float delta = yaw_now - yaw_at_turn_start_;
        while (delta >  180.0f) delta -= 360.0f;
        while (delta < -180.0f) delta += 360.0f;
        float abs_delta = std::fabs(delta);

        float remaining = turn_target_angle_deg_ - abs_delta;

        bool target_reached = remaining <= turn_tolerance_deg_;
        bool safety_timeout = elapsed_in_state > turn_timeout_sec_;

        static double last_log_time = -1.0;
        if (elapsed_in_state - last_log_time > 0.2) {
            last_log_time = elapsed_in_state;
            const char* status = "rotating";
            if (target_reached) status = "TARGET REACHED → stop";
            if (safety_timeout) status = "TIMEOUT → stop";
            RCLCPP_INFO(this->get_logger(), "[TURN] t=%.2fs delta=%.1f° remaining=%.1f° front=%.2f (%s)", elapsed_in_state, delta, remaining, s.front, status);
        }

        if (!turn_motors_stopped_ && (target_reached || safety_timeout)) {
            motor_node_->set_motor_speeds(127, 127);
            turn_motors_stopped_ = true;
            turn_stopped_at_ = this->get_clock()->now();
            turn_final_delta_ = delta;

            const char* reason = target_reached ? "target reached" : "TIMEOUT";
            RCLCPP_INFO(this->get_logger(), "Turn stopping: %s. delta=%.1f° (target=%.1f°), front=%.2f. Settling for %.1fs...", reason, delta, turn_target_angle_deg_, s.front, settle_time_sec_);
            return;
        }

        if (turn_motors_stopped_) {
            motor_node_->set_motor_speeds(127, 127);
            double since_stop = (this->get_clock()->now() - turn_stopped_at_).seconds();
            if (since_stop > settle_time_sec_) {
                RCLCPP_INFO(this->get_logger(), "Turn DONE. delta_at_stop=%.1f° → delta_final=%.1f° (inertia added %.1f°)", turn_final_delta_, delta, delta - turn_final_delta_);

                target_yaw_ = imu_node_->getYawDegrees();
                snap_target_yaw_to_grid();
                yaw_prev_err_ = 0.0f;

                turn_motors_stopped_ = false;
                transition_to(NavState::POST_TURN_DRIVE, "POST_TURN_DRIVE");
                return;
            }
            return;
        }

        uint8_t current_speed;
        if (remaining > turn_slowdown_start_deg_) {
            current_speed = turn_speed_max_;
        } else if (remaining <= turn_tolerance_deg_) {
            current_speed = turn_speed_min_;
        } else {
            float ratio = (remaining - turn_tolerance_deg_) / (turn_slowdown_start_deg_ - turn_tolerance_deg_);
            ratio = std::clamp(ratio, 0.0f, 1.0f);
            float speed_f = turn_speed_min_ + ratio * (turn_speed_max_ - turn_speed_min_);
            current_speed = static_cast<uint8_t>(speed_f);
        }

        if (turn_direction_ == TurnDirection::LEFT) {
            motor_node_->set_motor_speeds(127 - current_speed, 127 + current_speed);
        } else {
            motor_node_->set_motor_speeds(127 + current_speed, 127 - current_speed);
        }
    }

    void CorridorLoop::handle_post_turn_drive(const nodes::LidarSectors& s) {
        double elapsed = (this->get_clock()->now() - state_enter_time_).seconds();

        if (elapsed > post_turn_duration_) {
            RCLCPP_INFO(this->get_logger(), "Post-turn protection ended, back to corridor following");
            yaw_prev_err_ = 0.0f;
            transition_to(NavState::CORRIDOR_FOLLOWING, "CORRIDOR_FOLLOWING");
            return;
        }

        if (std::isfinite(s.front) && s.front < 0.10f) {
            motor_node_->set_motor_speeds(127, 127);
            motor_node_->set_motor_speeds(127, 127);
            RCLCPP_WARN(this->get_logger(), "POST_TURN emergency stop! front=%.2f", s.front);
            return;
        }

        rclcpp::Time now = this->get_clock()->now();
        double dt = (now - last_pid_time_).seconds();
        last_pid_time_ = now;

        float current_yaw = imu_node_->getYawDegrees();
        float yaw_err = target_yaw_ - current_yaw;
        while (yaw_err >  180.0f) yaw_err -= 360.0f;
        while (yaw_err < -180.0f) yaw_err += 360.0f;

        float yaw_err_deriv = 0.0f;
        if (dt > 0.001 && dt < 1.0) {
            yaw_err_deriv = (yaw_err - yaw_prev_err_) / static_cast<float>(dt);
        }
        yaw_prev_err_ = yaw_err;

        float yaw_correction = yaw_kp_ * yaw_err + yaw_kd_ * yaw_err_deriv;
        yaw_correction = std::clamp(yaw_correction, -yaw_max_correction_, yaw_max_correction_);

        int left_speed  = static_cast<int>(base_speed_ - yaw_correction);
        int right_speed = static_cast<int>(base_speed_ + yaw_correction);
        left_speed  = std::clamp(left_speed,  120, 180);
        right_speed = std::clamp(right_speed, 120, 180);

        motor_node_->set_motor_speeds(
            static_cast<uint8_t>(left_speed),
            static_cast<uint8_t>(right_speed)
        );
    }

}