#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "turtlebot3_localization/extended_kalman_filter.hpp"

class EKFNode : public rclcpp::Node
{
public:
    EKFNode() : Node("ekf_node")
    {
        using namespace std::placeholders;

        RCLCPP_INFO(this->get_logger(), "Extended Kalman Filter Node gestartet.");

        ekf_ = std::make_unique<ExtendedKalmanFilter>();

        imu_sub_.subscribe(this, "/imu");
        joint_sub_.subscribe(this, "/joint_states");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), imu_sub_, joint_sub_);
        sync_->registerCallback(std::bind(&EKFNode::sensorCallback, this, _1, _2));

        prediction_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/prediction", 10);
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Imu, sensor_msgs::msg::JointState>;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    message_filters::Subscriber<sensor_msgs::msg::JointState> joint_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr prediction_pub_;

    std::unique_ptr<ExtendedKalmanFilter> ekf_;
    rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

    void sensorCallback(
        const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
        const sensor_msgs::msg::JointState::ConstSharedPtr joint_msg)
    {
        rclcpp::Time now_ros(imu_msg->header.stamp);

        if (last_time_.nanoseconds() == 0) {
            last_time_ = now_ros;
            return;
        }

        double dt = (now_ros - last_time_).seconds();
        if (dt <= 0.0 || dt > 1.0) {
            RCLCPP_WARN(this->get_logger(), "Ungültiges dt: %.3f", dt);
            last_time_ = now_ros;
            return;
        }
        last_time_ = now_ros;

        ekf_->predict(now_ros, dt);
        ekf_->correct(imu_msg, joint_msg);

        auto prediction = ekf_->getPrediction();
        prediction.header.stamp = now_ros;
        prediction.header.frame_id = "odom";
        prediction_pub_->publish(prediction);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFNode>());
    rclcpp::shutdown();
    return 0;
}
