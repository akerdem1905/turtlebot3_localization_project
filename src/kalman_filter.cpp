#include "turtlebot3_localization/kalman_filter.hpp"

#include <cmath>

KalmanFilter::KalmanFilter() {
    x_ = Eigen::VectorXd::Zero(STATE_SIZE);
    P_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 0.1;
    A_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    B_ = Eigen::MatrixXd::Zero(STATE_SIZE, CONTROL_SIZE);
    Q_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 0.01;
    R_ = Eigen::MatrixXd::Identity(MEASUREMENT_SIZE, MEASUREMENT_SIZE) * 0.05;

    C_ = Eigen::MatrixXd::Zero(MEASUREMENT_SIZE, STATE_SIZE);
    C_(0, 1) = 1.0; // vx
    C_(1, 3) = 1.0; // vy
    C_(2, 5) = 1.0; // omega
}

void KalmanFilter::predict(const Eigen::VectorXd &u, double dt) {
    A_.setIdentity();
    A_(0, 1) = dt;
    A_(2, 3) = dt;
    A_(4, 5) = dt;

    B_.setZero();
    B_(1, 0) = 1.0; // v -> vx
    B_(5, 1) = 1.0; // omega -> omega

    x_ = A_ * x_ + B_ * u;
    P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::correct(const Eigen::VectorXd &z) {
    Eigen::VectorXd y = z - C_ * x_;
    Eigen::MatrixXd S = C_ * P_ * C_.transpose() + R_;
    Eigen::MatrixXd K = P_ * C_.transpose() * S.inverse();

    x_ = x_ + K * y;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    P_ = (I - K * C_) * P_;
}

geometry_msgs::msg::PoseWithCovarianceStamped KalmanFilter::getPrediction() {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "odom";
    msg.pose.pose.position.x = x_(0);
    msg.pose.pose.position.y = x_(2);

    double theta = x_(4);
    msg.pose.pose.orientation.z = sin(theta / 2.0);
    msg.pose.pose.orientation.w = cos(theta / 2.0);

    // Set covariance values (nur relevante Einträge)
    for (auto& c : msg.pose.covariance) {
    c = 0.0;
}
    msg.pose.covariance[0] = P_(0, 0);   // x
    msg.pose.covariance[7] = P_(2, 2);   // y
    msg.pose.covariance[35] = P_(4, 4);  // yaw

    return msg;
}
