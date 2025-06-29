#ifndef EXTENDED_KALMAN_FILTER_HPP_
#define EXTENDED_KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class ExtendedKalmanFilter {
public:
  ExtendedKalmanFilter();

  void predict(const rclcpp::Time& now, double dt);
  void correct(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg,
               const sensor_msgs::msg::JointState::ConstSharedPtr& joint_msg);

  geometry_msgs::msg::PoseWithCovarianceStamped getPrediction();

private:
  Eigen::VectorXd state_;
  Eigen::MatrixXd covariance_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  rclcpp::Time last_time_;

  void normalizeYaw();
};

#endif  // EXTENDED_KALMAN_FILTER_HPP_
