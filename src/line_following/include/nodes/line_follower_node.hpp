#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <algorithm>
#include <cmath>

namespace nodes
{


enum class Mode { LINE, CIRCLE, FIGURE8 };
static constexpr Mode ROBOT_MODE = Mode::LINE;



static constexpr uint16_t SENSOR_THRESHOLD    = 300;
static constexpr uint16_t INTERSECTION_THRESH = 700;

static constexpr double KP = 0.012;
static constexpr double KI = 0.0;
static constexpr double KD = 0.0;
static constexpr double INTEGRAL_CLAMP = 200.0;
static constexpr int MAX_CORRECTION = 50;

static constexpr int SPEED_NORMAL       = 132;

static constexpr int INTERSECTION_CONFIRM  = 4;
static constexpr int INTERSECTION_COOLDOWN = 35;

// ═══════════════════════════════════════════════════════════════════════════════

class LineFollowerNode : public rclcpp::Node
{
public:
    LineFollowerNode()
        : Node("line_follower_node"),
          integral_(0.0),
          prev_error_(0.0),
          last_time_(this->now()),
          intersection_count_(0),
          intersection_cooldown_(0),
          figure8_turn_right_(true)
    {
        line_sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/bpc_prp_robot/line_sensors", 10,
            std::bind(&LineFollowerNode::line_callback, this, std::placeholders::_1));

        motor_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/bpc_prp_robot/set_motor_speeds", 10);
	}

private:
    void line_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) return;

        const uint16_t left_raw  = msg->data[0];
        const uint16_t right_raw = msg->data[1];

        const bool left_on   = left_raw  > SENSOR_THRESHOLD;
        const bool right_on  = right_raw > SENSOR_THRESHOLD;
        const bool both_high = left_raw  > INTERSECTION_THRESH &&
                               right_raw > INTERSECTION_THRESH;

	// Intersection detection
        bool at_intersection = false;
        if (intersection_cooldown_ > 0) {
            intersection_cooldown_--;
        } else if (both_high) {
            intersection_count_++;
            if (intersection_count_ >= INTERSECTION_CONFIRM) {
                at_intersection = true;
                on_intersection();
                intersection_count_  = 0;
                intersection_cooldown_ = INTERSECTION_COOLDOWN;
                integral_ = 0.0;
            }
        } else {
            intersection_count_ = 0;
        }

        double error;
        
       
        const double raw_diff = static_cast<double>(right_raw) - static_cast<double>(left_raw);
        error = raw_diff;

        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt < 0.005 || dt > 0.5) dt = 0.02;
        
      
      

        // PID
        integral_ += error * dt;
        integral_  = std::clamp(integral_, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);

        const double derivative = (error - prev_error_) / dt;
        prev_error_ = error;

        double correction = KP * error + KI * integral_ + KD * derivative;
        correction = std::clamp(correction, -(double)MAX_CORRECTION, (double)MAX_CORRECTION);

        // Speed 
        int base;
        base = SPEED_NORMAL;

        int left_speed  = base + static_cast<int>(correction);
        int right_speed = base - static_cast<int>(correction);

        left_speed  = std::clamp(left_speed,  128, 254);
        right_speed = std::clamp(right_speed, 128, 254);

        publish(static_cast<uint8_t>(left_speed),
                static_cast<uint8_t>(right_speed));
        }

  
    void on_intersection()
    {
        switch (ROBOT_MODE) {
            case Mode::LINE:
            case Mode::CIRCLE:
                RCLCPP_INFO(this->get_logger(), "Intersection — going straight");
                break;
            case Mode::FIGURE8:
                figure8_turn_right_ = !figure8_turn_right_;
                RCLCPP_INFO(this->get_logger(),
                    "Intersection — next lobe: turn %s",
                    figure8_turn_right_ ? "RIGHT" : "LEFT");
                break;
        }
    }

    void publish(uint8_t left, uint8_t right)
    {
        std_msgs::msg::UInt8MultiArray msg;
        msg.data = {left, right};
        motor_pub_->publish(msg);
    }

    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr      motor_pub_;

    double integral_;
    double prev_error_;
    rclcpp::Time last_time_;

    int intersection_count_;
    int intersection_cooldown_;
    bool figure8_turn_right_;
};

} 
