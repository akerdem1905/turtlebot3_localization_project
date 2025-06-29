#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Dense>
#include <memory>
#include <chrono>

#include "turtlebot3_localization/kalman_filter.hpp"


using std::placeholders::_1;
using std::placeholders::_2;

class MainFilterNode : public rclcpp::Node {
public:
    MainFilterNode() : Node("main_filter_node") {
        odom_sub_.subscribe(this, "/odom");
        imu_sub_.subscribe(this, "/imu");

        sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), odom_sub_, imu_sub_);
        sync_->registerCallback(std::bind(&MainFilterNode::syncedCallback, this, _1, _2));

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&MainFilterNode::cmdVelCallback, this, _1));

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/prediction", 10);

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "KalmanFilter Node gestartet.");
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    std::shared_ptr<Synchronizer> sync_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;

    KalmanFilter kf_;
    Eigen::VectorXd u_ = Eigen::VectorXd::Zero(2);  // [v, omega]
    rclcpp::Time last_time_;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        u_(0) = msg->linear.x;
        u_(1) = msg->angular.z;
    }

    void syncedCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                        const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg) {
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt <= 0.0 || dt > 1.0) {
            RCLCPP_WARN(this->get_logger(), "Ungültiges dt: %.3f", dt);
            return;
        }

        // Predict mit Steuerung
        kf_.predict(u_, dt);

        // Korrektur mit Messdaten: vx (aus Odom), vy=0 (Diff-Drive), omega (aus IMU)
        Eigen::VectorXd z(3);
        z << odom_msg->twist.twist.linear.x, 0.0, imu_msg->angular_velocity.z;

        kf_.correct(z);

        // Publish Ergebnis
        auto prediction = kf_.getPrediction();
        prediction.header.stamp = now;
        publisher_->publish(prediction);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainFilterNode>());
    rclcpp::shutdown();
    return 0;
}
