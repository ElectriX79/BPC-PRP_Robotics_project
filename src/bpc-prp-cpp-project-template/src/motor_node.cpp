#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <atomic>

using namespace std::chrono_literals;

// Funkcia na čítanie znaku bez Enter
char getch() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    char ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

class MotorNode : public rclcpp::Node
{
public:
    MotorNode() : Node("motor_node_wasd")
    {
        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/bpc_prp_robot/set_motor_speeds", 10);

        left_wheel_.store(127);
        right_wheel_.store(127);
        base_speed_.store(150); // počiatočná rýchlosť

        running_.store(true);

        // Spustenie vlákna pre čítanie kláves
        input_thread_ = std::thread(std::bind(&MotorNode::keyboard_loop, this));

        // Timer posiela príkazy každých 100ms
        timer_ = this->create_wall_timer(
            100ms, std::bind(&MotorNode::publish_wheel_speeds, this));

        RCLCPP_INFO(this->get_logger(), "Motor node started - WASD control, T/Y speed adjust");
        RCLCPP_INFO(this->get_logger(), "Controls:");
        RCLCPP_INFO(this->get_logger(), "  W = forward  S = backward");
        RCLCPP_INFO(this->get_logger(), "  A = turn left D = turn right");
        RCLCPP_INFO(this->get_logger(), "  T = increase speed  Y = decrease speed");
        RCLCPP_INFO(this->get_logger(), "  X = stop");
    }

    ~MotorNode() {
        running_.store(false);
        if (input_thread_.joinable())
            input_thread_.join();
    }

private:
    void keyboard_loop()
    {
        while (running_.load()) {
            char c = getch();

            switch(c)
            {
                case 'w': // dopredu
                    left_wheel_.store(base_speed_.load());
                    right_wheel_.store(base_speed_.load());
                    break;
                case 's': // dozadu
                    left_wheel_.store(254 - base_speed_.load());
                    right_wheel_.store(254 - base_speed_.load());
                    break;
                case 'a': // otáča sa vľavo
                    left_wheel_.store(254 - base_speed_.load());
                    right_wheel_.store(base_speed_.load());
                    break;
                case 'd': // otáča sa vpravo
                    left_wheel_.store(base_speed_.load());
                    right_wheel_.store(254 - base_speed_.load());
                    break;
                case 't': // zvýšiť rýchlosť
                {
                    int tmp = base_speed_.load() + 5;
                    if (tmp > 255) tmp = 255;
                    base_speed_.store(tmp);
                    RCLCPP_INFO(this->get_logger(), "Speed increased to %d", tmp);
                    break;
                }
                case 'y': // znížiť rýchlosť
                {
                    int tmp = base_speed_.load() - 5;
                    if (tmp < 128) tmp = 128; // min dopredu
                    base_speed_.store(tmp);
                    RCLCPP_INFO(this->get_logger(), "Speed decreased to %d", tmp);
                    break;
                }
                case 'x': // stop
                    left_wheel_.store(127);
                    right_wheel_.store(127);
                    break;
                default:
                    break;
            }
        }
    }

    void publish_wheel_speeds()
    {
        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data.push_back(left_wheel_.load());
        msg.data.push_back(right_wheel_.load());
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread input_thread_;
    std::atomic<bool> running_;
    std::atomic<int> left_wheel_;
    std::atomic<int> right_wheel_;
    std::atomic<int> base_speed_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
