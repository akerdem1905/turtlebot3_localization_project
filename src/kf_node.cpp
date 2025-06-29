#include "turtlebot3_localization/kf.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class KalmanFilterNode : public rclcpp::Node {
public:
    KalmanFilterNode() : Node("kalman_filter_node") {
        RCLCPP_INFO(this->get_logger(), "Kalman Filter Node gestartet");

        filter_ = std::make_unique<KalmanFilter>();
        u_ = Eigen::VectorXd::Zero(2);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&KalmanFilterNode::imuCallback, this, std::placeholders::_1));

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&KalmanFilterNode::jointCallback, this, std::placeholders::_1));

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10, std::bind(&KalmanFilterNode::cmdVelCallback, this, std::placeholders::_1));

        prediction_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/kf_pose", 10);

        last_time_ = this->now();
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double dt = (this->now() - last_time_).seconds();
        if (dt <= 0.0 || dt > 0.5) return;
        last_time_ = this->now();

        // Set angular velocity z from IMU as measurement
        Eigen::VectorXd z(3);
        z << u_(0), 0.0, msg->angular_velocity.z;

        filter_->predict(u_, dt);
        filter_->correct(z);
        prediction_pub_->publish(filter_->getPrediction());
    }

    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Compute v from wheels
        double v_left = 0.0, v_right = 0.0;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "wheel_left_joint") v_left = msg->velocity[i];
            else if (msg->name[i] == "wheel_right_joint") v_right = msg->velocity[i];
        }
        double wheel_radius = 0.033;
        u_(0) = (v_left + v_right) * 0.5 * wheel_radius;
    }

    void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        u_(0) = msg->twist.linear.x;
        u_(1) = msg->twist.angular.z;
    }

    std::unique_ptr<KalmanFilter> filter_;
    Eigen::VectorXd u_;
    rclcpp::Time last_time_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr prediction_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
