#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class KalmanFilter {
public:
    KalmanFilter();

    void predict(const Eigen::VectorXd &u, double dt);
    void correct(const Eigen::VectorXd &z);
    geometry_msgs::msg::PoseWithCovarianceStamped getPrediction();

private:
    static constexpr int STATE_SIZE = 6;
    static constexpr int CONTROL_SIZE = 2;
    static constexpr int MEASUREMENT_SIZE = 3;

    Eigen::VectorXd x_;     // State vector: [x, vx, y, vy, theta, omega]
    Eigen::MatrixXd P_;     // Covariance
    Eigen::MatrixXd A_;     // State transition
    Eigen::MatrixXd B_;     // Control matrix
    Eigen::MatrixXd Q_;     // Process noise
    Eigen::MatrixXd R_;     // Measurement noise
    Eigen::MatrixXd C_;     // Measurement model
};
