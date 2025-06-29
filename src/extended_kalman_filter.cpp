#include "turtlebot3_localization/extended_kalman_filter.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
  state_ = Eigen::VectorXd::Zero(6);
  covariance_ = Eigen::MatrixXd::Identity(6, 6);
  Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.01;
  R_ = Eigen::MatrixXd::Identity(3, 3) * 0.05;
}

void ExtendedKalmanFilter::predict(const rclcpp::Time& now, double dt)
{
  last_time_ = now;
  // Basic constant velocity model:
  state_(0) += state_(1) * dt;  // x
  state_(2) += state_(3) * dt;  // y
  state_(4) += state_(5) * dt;  // theta
  normalizeYaw();

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6,6);
  A(0,1) = dt;
  A(2,3) = dt;
  A(4,5) = dt;
  covariance_ = A * covariance_ * A.transpose() + Q_;
}

void ExtendedKalmanFilter::correct(
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg,
    const sensor_msgs::msg::JointState::ConstSharedPtr& joint_msg)
{
  double vx = 0.0;
  double vy = 0.0;
  for (size_t i = 0; i < joint_msg->name.size(); ++i) {
    if (joint_msg->name[i] == "wheel_left_joint" || joint_msg->name[i] == "wheel_right_joint") {
      vx += joint_msg->velocity[i];
    }
  }
  vx /= 2.0;

  double omega = imu_msg->angular_velocity.z;

  Eigen::VectorXd z(3);
  z << vx, vy, omega;

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3,6);
  H(0,1) = 1.0;
  H(1,3) = 1.0;
  H(2,5) = 1.0;

  Eigen::VectorXd y = z - H * state_;
  Eigen::MatrixXd S = H * covariance_ * H.transpose() + R_;
  Eigen::MatrixXd K = covariance_ * H.transpose() * S.inverse();

  state_ = state_ + K * y;
  covariance_ = (Eigen::MatrixXd::Identity(6,6) - K * H) * covariance_;
  normalizeYaw();
}

geometry_msgs::msg::PoseWithCovarianceStamped ExtendedKalmanFilter::getPrediction()
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = last_time_;
  msg.header.frame_id = "odom";
  msg.pose.pose.position.x = state_(0);
  msg.pose.pose.position.y = state_(2);

  tf2::Quaternion q;
  q.setRPY(0, 0, state_(4));
  msg.pose.pose.orientation = tf2::toMsg(q);

  for (int i = 0; i < 36; ++i) {
    msg.pose.covariance[i] = 0.0;
  }
  msg.pose.covariance[0] = covariance_(0,0);
  msg.pose.covariance[7] = covariance_(2,2);
  msg.pose.covariance[35] = covariance_(4,4);

  return msg;
}

void ExtendedKalmanFilter::normalizeYaw() {
  while (state_(4) > M_PI) state_(4) -= 2 * M_PI;
  while (state_(4) < -M_PI) state_(4) += 2 * M_PI;
}
