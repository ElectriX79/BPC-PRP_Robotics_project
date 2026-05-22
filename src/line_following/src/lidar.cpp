#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <cmath>
#include <algorithm>

class CorridorDriveNode : public rclcpp::Node
{
public:
  CorridorDriveNode() : Node("corridor_drive_node")
  {
    lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/bpc_prp_robot/lidar", 10,
      std::bind(&CorridorDriveNode::lidar_callback, this, std::placeholders::_1));

    motor_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(
      "/bpc_prp_robot/set_motor_speeds", 10);
  }

private:
  enum State {START, DRIVE, TURN};
  State state_ = DRIVE;

  bool turn_left_ = false;
  double best_front = INFINITY;
  double last_front = NAN;
  bool turning_left = true;
  bool turning_right = true;
  double previous_error = 0.0;
  bool first_run = true;
  rclcpp::Time last_time;

  rclcpp::Time turn_start_time;  
  bool turning_ = false;  // Flaga, ktorá označuje, či robot práve otáča
  bool correcting_ = false;  // Flaga na opravu vyváženia

  // ─────────────── LiDAR (úzky kužeľ) ───────────────
  double get_avg_range(const sensor_msgs::msg::LaserScan &scan,
                       double center_angle,
                       double window = 0.0174)
  {
    double sum = 0.0;
    int count = 0;

    for (size_t i = 0; i < scan.ranges.size(); ++i)
    {
      double angle = scan.angle_min + i * scan.angle_increment;

      if (std::abs(angle - center_angle) <= window)
      {
        float r = scan.ranges[i];

        if (std::isfinite(r) &&
            r >= scan.range_min &&
            r <= scan.range_max)
        {
          sum += r;
          count++;
        }
      }
    }

    return (count > 0) ? (sum / count) : NAN;
  }

  // ─────────────── CALLBACK ───────────────
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double left  = get_avg_range(*msg,  -M_PI/2);
    double right = get_avg_range(*msg, M_PI/2);
    double front = get_avg_range(*msg, M_PI);

    double left_front = get_avg_range(*msg, -M_PI/2.1);
    double right_front = get_avg_range(*msg, M_PI/2.1);
    RCLCPP_INFO(this->get_logger(), "left: %.2f right: %.2f front: %.2f left_front: %.2f right_front: %.2f",
            left, right, front, left_front, right_front);

    bool has_left  = std::isfinite(left);
    bool has_right = std::isfinite(right);
    static double desired = 0.15;
    double error = 0;
    double base = 140;
    double Kp = 0.0;
    double Kd = 0.0;
    double k = 20;

    rclcpp::Time now = this->now();

    double dt = 0.0;

    if (!first_run) {
      dt = (now - last_time).seconds();
    }
    else {
      first_run = false;
      last_time = now;
    }
    


    if (state_ == DRIVE) {
      if (has_left && has_right) {
          Kp = 40;
          Kd = 20;
          base = 140;
          error = right - left;
        }
      else if (!has_left && has_right) {
        Kp = 25;
        Kd = 25;
        base = 135;
        if (right > 0.5) {
          publish(140,140);
          return;
        }
        else {
          error = right - desired;
        }
      }
      else if (!has_right && has_left) {
        Kp = 25;
        Kd = 25;
        base = 135;
        if (left > 0.5) {
          publish(140, 140);
          return;
        }
        else {
          error = desired - left;
        }
      }
      if (std::abs(error) < 0.02) {
        error = 0.0;
      }

      double derivative = 0.0;
      if (dt > 0.0) {
        double raw_derivative = (error - previous_error) / dt;
        derivative = 0.7 * raw_derivative;
      }
      double control = Kp * error + Kd * derivative;
      double l = base + control;
      double r = base - control;
      l = std::clamp(l, 0.0, 255.0);
      r = std::clamp(r, 0.0, 255.0);

      publish(l, r);
      previous_error = error;
    }
  }

  // ─────────────── MOTOR ───────────────
  void publish(double left, double right)
  {
    std_msgs::msg::UInt8MultiArray msg;
    msg.data = {
      static_cast<uint8_t>(left),
      static_cast<uint8_t>(right)
    };
    motor_pub_->publish(msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CorridorDriveNode>());
  rclcpp::shutdown();
  return 0;
}
