#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <algorithm>
#include <cmath>

namespace nodes
{

// ═══════════════════════════════════════════════════════════════════════════════
// MODE
// ═══════════════════════════════════════════════════════════════════════════════

enum class Mode { LINE, CIRCLE, FIGURE8 };
static constexpr Mode ROBOT_MODE = Mode::LINE;

// ═══════════════════════════════════════════════════════════════════════════════
// SENSOR
//
// From your logs:
//   White (off line) : L≈25,  R≈25
//   Black (on line)  : L≈960, R≈980
//
// SENSOR_THRESHOLD  = midpoint ≈ 500  (already correct)
// INTERSECTION_THRESH = value where BOTH sensors are firmly on black ≈ 700
//
// KEY FIX: your right sensor consistently reads ~20 higher than left when
// both are on the line. We correct for this with SENSOR_BIAS.
// Set SENSOR_BIAS = average of (R - L) when robot is centred on the line.
// From your logs this is approximately +20.
// ═══════════════════════════════════════════════════════════════════════════════

static constexpr uint16_t SENSOR_THRESHOLD    = 300;
static constexpr uint16_t INTERSECTION_THRESH = 700;

// Hardware bias: right sensor reads this much higher than left when centred.
// Measure by placing robot perfectly centred on line and averaging (R - L).
// From your logs: roughly +20. Adjust after testing.
static constexpr double SENSOR_BIAS = 0.0;

// ═══════════════════════════════════════════════════════════════════════════════
// PID
// ═══════════════════════════════════════════════════════════════════════════════

static constexpr double KP = 0.012;
static constexpr double KI = 0.0;
static constexpr double KD = 0.0;
static constexpr double INTEGRAL_CLAMP = 200.0;
static constexpr int MAX_CORRECTION = 50;

// ═══════════════════════════════════════════════════════════════════════════════
// SPEED  (127 = stop, 254 = full forward)
// ═══════════════════════════════════════════════════════════════════════════════

static constexpr int SPEED_NORMAL       = 132;
static constexpr int SPEED_TURN         = 130;
static constexpr int SPEED_INTERSECTION = 145;
static constexpr int SPEED_LOST         = 135;

// ═══════════════════════════════════════════════════════════════════════════════
// INTERSECTION DEBOUNCE
// ═══════════════════════════════════════════════════════════════════════════════

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

        RCLCPP_INFO(this->get_logger(), "==============================");
        RCLCPP_INFO(this->get_logger(), "Line follower started — mode: %s",
            ROBOT_MODE == Mode::LINE    ? "LINE" :
            ROBOT_MODE == Mode::CIRCLE  ? "CIRCLE" : "FIGURE-8");
        RCLCPP_INFO(this->get_logger(), "Sensor bias correction: %.1f", SENSOR_BIAS);
        RCLCPP_INFO(this->get_logger(), "==============================");
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

        // ── Intersection detection ────────────────────────────────────────
        bool at_intersection = false;
        if (intersection_cooldown_ > 0) {
            intersection_cooldown_--;
        } else if (both_high) {
            intersection_count_++;
            if (intersection_count_ >= INTERSECTION_CONFIRM) {
                at_intersection = true;
                on_intersection();
                intersection_count_    = 0;
                intersection_cooldown_ = INTERSECTION_COOLDOWN;
                integral_ = 0.0;
            }
        } else {
            intersection_count_ = 0;
        }

        // ── Error with bias correction ────────────────────────────────────
        // Raw difference would be (R - L). We subtract SENSOR_BIAS so that
        // when the robot is perfectly centred, error = 0 regardless of the
        // hardware offset between the two sensors.
        //
        //   error > 0 → line is to the right → steer right
        //   error < 0 → line is to the left  → steer left
        //   error = 0 → centred
        double error;
        
       
        const double raw_diff = static_cast<double>(right_raw)
                                  - static_cast<double>(left_raw);
        error = raw_diff - SENSOR_BIAS;

        // ── dt ────────────────────────────────────────────────────────────
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt < 0.005 || dt > 0.5) dt = 0.02;
        
      
      

        // ── PID ───────────────────────────────────────────────────────────
        integral_ += error * dt;
        integral_  = std::clamp(integral_, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);

        const double derivative = (error - prev_error_) / dt;
        prev_error_ = error;

        double correction = KP * error + KI * integral_ + KD * derivative;
        correction = std::clamp(correction, -(double)MAX_CORRECTION, (double)MAX_CORRECTION);

        // ── Speed ─────────────────────────────────────────────────────────
        int base;
        base = SPEED_NORMAL;

        int left_speed  = base + static_cast<int>(correction);
        int right_speed = base - static_cast<int>(correction);

        left_speed  = std::clamp(left_speed,  128, 254);
        right_speed = std::clamp(right_speed, 128, 254);

        publish(static_cast<uint8_t>(left_speed),
                static_cast<uint8_t>(right_speed));

        RCLCPP_INFO(this->get_logger(),
            "RAW L=%-5d R=%-5d | on=[%s%s] | err=%+7.1f corr=%+6.1f | motors L=%-3d R=%-3d | %s",
            left_raw, right_raw,
            left_on  ? "L" : ".",
            right_on ? "R" : ".",
            error, correction,
            left_speed, right_speed,
            at_intersection         ? "INTERSECTION" :
            (!left_on && !right_on) ? "LOST" :
            (left_on && right_on)   ? "BOTH" :
            left_on                 ? "LEFT" : "RIGHT");
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

    double       integral_;
    double       prev_error_;
    rclcpp::Time last_time_;

    int  intersection_count_;
    int  intersection_cooldown_;
    bool figure8_turn_right_;
};

} // namespace nodes
