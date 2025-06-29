#include <cmath>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

class KalmanFilterNode : public rclcpp::Node {
public:
  KalmanFilterNode() : Node("kalman_filter_node") {
    initialize();

    imu_sub_.subscribe(this, "/imu");
    joint_sub_.subscribe(this, "/joint_states");

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), imu_sub_, joint_sub_);

    sync_->registerCallback(std::bind(&KalmanFilterNode::synchronizedCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&KalmanFilterNode::cmdVelCallback, this, std::placeholders::_1));

    filtered_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/filtered_state", 10);

    RCLCPP_INFO(this->get_logger(), "KF Node started");
  }

private:
  static constexpr int STATE_SIZE = 6;
  static constexpr int CONTROL_SIZE = 2;
  static constexpr int MEAS_SIZE = 3;

  Eigen::VectorXd x_;    // [x, vx, y, vy, theta, omega]
  Eigen::MatrixXd P_, A_, B_, Q_, R_, C_;
  Eigen::VectorXd u_;

  rclcpp::Time last_time_;
  bool initialized_ = false;
  std::mutex mutex_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Imu, sensor_msgs::msg::JointState>;

  message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
  message_filters::Subscriber<sensor_msgs::msg::JointState> joint_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_pub_;

  void initialize() {
    x_ = Eigen::VectorXd::Zero(STATE_SIZE);
    P_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    Q_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 0.05;
    R_ = Eigen::MatrixXd::Identity(MEAS_SIZE, MEAS_SIZE) * 0.1;

    A_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    B_ = Eigen::MatrixXd::Zero(STATE_SIZE, CONTROL_SIZE);
    C_ = Eigen::MatrixXd::Zero(MEAS_SIZE, STATE_SIZE);
    C_(0, 1) = 1.0;  // vx
    C_(1, 3) = 1.0;  // vy
    C_(2, 5) = 1.0;  // omega

    u_ = Eigen::VectorXd::Zero(CONTROL_SIZE);
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    u_(0) = msg->linear.x;
    u_(1) = msg->angular.z;
  }

  void synchronizedCallback(
      const sensor_msgs::msg::Imu::ConstSharedPtr imu,
      const sensor_msgs::msg::JointState::ConstSharedPtr joint) {

    std::lock_guard<std::mutex> lock(mutex_);
    auto now = this->now();
    if (!initialized_) {
      x_(5) = imu->angular_velocity.z;
      last_time_ = now;
      initialized_ = true;
      return;
    }

    double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (dt <= 0.0 || dt > 1.0) return;

    // Prediction
    A_(0, 1) = dt;
    A_(2, 3) = dt;
    A_(4, 5) = dt;

    B_.setZero();
    B_(1, 0) = 1.0;
    B_(5, 1) = 1.0;

    x_ = A_ * x_ + B_ * u_;
    P_ = A_ * P_ * A_.transpose() + Q_;

    // Measurement update
    Eigen::VectorXd z(MEAS_SIZE);
    double left_vel = 0.0, right_vel = 0.0;
    for (size_t i = 0; i < joint->name.size(); ++i) {
      if (joint->name[i] == "wheel_left_joint")
        left_vel = joint->velocity[i];
      else if (joint->name[i] == "wheel_right_joint")
        right_vel = joint->velocity[i];
    }

    double linear_v = (left_vel + right_vel) * 0.5 * 0.033;  // wheel radius
    z << linear_v, 0.0, imu->angular_velocity.z;

    Eigen::VectorXd y = z - C_ * x_;
    Eigen::MatrixXd S = C_ * P_ * C_.transpose() + R_;
    Eigen::MatrixXd K = P_ * C_.transpose() * S.inverse();

    x_ += K * y;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    P_ = (I - K * C_) * P_;

    publish();
  }

  void publish() {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_footprint";
    msg.pose.pose.position.x = x_(0);
    msg.pose.pose.position.y = x_(2);

    tf2::Quaternion q;
    q.setRPY(0, 0, x_(4));
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    msg.twist.twist.linear.x = x_(1);
    msg.twist.twist.linear.y = x_(3);
    msg.twist.twist.angular.z = x_(5);

    filtered_pub_->publish(msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KalmanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
