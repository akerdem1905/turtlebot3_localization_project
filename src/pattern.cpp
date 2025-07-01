#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <chrono>
#include <cmath>
#include <string>

class PatternDriver : public rclcpp::Node {
public:
    PatternDriver()
    : Node("pattern_driver"), phase_(0.0) {
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

        this->declare_parameter("pattern", "circle");
        this->declare_parameter("linear_speed", 0.2);
        this->declare_parameter("angular_speed", 0.5);
        this->declare_parameter("duration", 20.0);

        start_time_ = this->now();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PatternDriver::update, this));

        RCLCPP_INFO(this->get_logger(), "PatternDriver gestartet (Muster: circle | zigzag)");
    }

private:
    void update() {
        auto now = this->now();
        double elapsed = (now - start_time_).seconds();

        std::string pattern = this->get_parameter("pattern").as_string();
        double v = this->get_parameter("linear_speed").as_double();
        double w = this->get_parameter("angular_speed").as_double();
        double duration = this->get_parameter("duration").as_double();

        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = now;
        cmd.header.frame_id = "base_link";

        if (elapsed > duration) {
            cmd.twist.linear.x = 0.0;
            cmd.twist.angular.z = 0.0;
            publisher_->publish(cmd);
            RCLCPP_INFO(this->get_logger(), "Bewegung abgeschlossen.");
            rclcpp::shutdown();
            return;
        }

        if (pattern == "circle") {
            cmd.twist.linear.x = v;
            cmd.twist.angular.z = w;
        }
        else if (pattern == "zigzag") {
            double t = fmod(elapsed, 4.0);
            cmd.twist.linear.x = v;
            cmd.twist.angular.z = (t < 2.0) ? w : -w;
        }
        else if (pattern == "circle_straight_circle") {
            if (elapsed < 10.0) {
                cmd.twist.linear.x = v;
                cmd.twist.angular.z = w;  // 1. Kreis
            } else if (elapsed < 13.0) {
                cmd.twist.linear.x = v;
                cmd.twist.angular.z = 0.0;  // gerade
            } else if (elapsed < 18.0) {
                cmd.twist.linear.x = v;
                cmd.twist.angular.z = w;  // 2. Kreis
            } else {
                cmd.twist.linear.x = 0.0;
                cmd.twist.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Muster abgeschlossen.");
                rclcpp::shutdown();
                return;
            }
        }else {
            RCLCPP_WARN_ONCE(this->get_logger(), "Unbekanntes Pattern '%s'. Fallback auf Kreis.", pattern.c_str());
            cmd.twist.linear.x = v;
            cmd.twist.angular.z = w;
        }

        publisher_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    double phase_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PatternDriver>());
    rclcpp::shutdown();
    return 0;
}
