#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <string>
#include <chrono>

#ifndef _WIN32
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#endif

#define LINEAR_SPEED 0.5   // m/s
#define ANGULAR_SPEED 0.5  // rad/s

std::string msg = R"(Control Your AMR!
---------------------------
Moving around:
        w/up
   a/left s/down d/right
        x

space key, s : force stop

CTRL-C to quit
)";

#ifndef _WIN32
struct termios old_settings_;

char getKey() {
    char buf = 0;
    struct timeval tv {0, 100000}; // 0.1s
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    if (select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv) == 1) {
        read(STDIN_FILENO, &buf, 1);

        // Arrow key detection: ESC [ A/B/C/D
        if (buf == '\033') {
            char seq[2];
            if (read(STDIN_FILENO, &seq[0], 1) == 0) return 0;
            if (read(STDIN_FILENO, &seq[1], 1) == 0) return 0;
            switch(seq[1]) {
                case 'A': return 'U'; // up
                case 'B': return 'D'; // down
                case 'C': return 'R'; // right
                case 'D': return 'L'; // left
                default: return 0;
            }
        }
    }
    return buf;
}
#else
#include <conio.h>
char getKey() {
    if (_kbhit()) return _getch();
    return 0;
}
#endif

class TeleopNode : public rclcpp::Node {
public:
    TeleopNode() : Node("anscer_teleop") {
        this->declare_parameter<std::string>("teleop_topic", "/diff_drive_controller/cmd_vel_unstamped");
        std::string topic;
        this->get_parameter("teleop_topic", topic);

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>(topic, 10);

#ifndef _WIN32
        tcgetattr(STDIN_FILENO, &old_settings_);
        struct termios new_settings = old_settings_;
        new_settings.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
#endif

        std::cout << msg << std::endl;
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                         std::bind(&TeleopNode::update, this));
    }

    ~TeleopNode() {
#ifndef _WIN32
        tcsetattr(STDIN_FILENO, TCSANOW, &old_settings_);
#endif
        stopRobot();
    }

private:
    void update() {
        char key = getKey();
        geometry_msgs::msg::Twist twist;

        switch(key) {
            case 'w': case 'U':  // w or up arrow
                twist.linear.x = LINEAR_SPEED;
                twist.angular.z = 0.0;
                break;
            case 'x': case 'D':  // x or down arrow
                twist.linear.x = -LINEAR_SPEED;
                twist.angular.z = 0.0;
                break;
            case 'a': case 'L':  // a or left arrow
                twist.linear.x = 0.0;
                twist.angular.z = ANGULAR_SPEED;
                break;
            case 'd': case 'R':  // d or right arrow
                twist.linear.x = 0.0;
                twist.angular.z = -ANGULAR_SPEED;
                break;
            case ' ': case 's':  // stop
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                break;
            case 3: // Ctrl-C
                rclcpp::shutdown();
                return;
            default:
                return;
        }

        pub_->publish(twist);
    }

    void stopRobot() {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        pub_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
