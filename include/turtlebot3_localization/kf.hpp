#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <Eigen/Dense>

class KalmanFilterNode : public rclcpp::Node {
public:
    KalmanFilterNode();

private:
    Eigen::VectorXd x_{6};
    Eigen::VectorXd u_{2};
    Eigen::MatrixXd P_{6,6}, Q_{6,6}, R_{3,3}, A_{6,6}, B_{6,2}, C_{3,6};

    rclcpp::Time last_time_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    message_filters::Subscriber<sensor_msgs::msg::JointState> joint_sub_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Imu, sensor_msgs::msg::JointState>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    void initKalman();
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu,
                  const sensor_msgs::msg::JointState::ConstSharedPtr joint);
    void predict(double dt);
    void update(const sensor_msgs::msg::Imu::ConstSharedPtr imu,
                const sensor_msgs::msg::JointState::ConstSharedPtr joint);
    void publish();
};
