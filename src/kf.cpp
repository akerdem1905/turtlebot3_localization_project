#include "turtlebot3_localization/kf.hpp"

KalmanFilterNode::KalmanFilterNode() : Node("kf_node") {
    initKalman();

    imu_sub_.subscribe(this, "/imu");
    joint_sub_.subscribe(this, "/joint_states");

    sync_ = std::make_shared<Sync>(Sync(10), imu_sub_, joint_sub_);
    sync_->registerCallback(std::bind(&KalmanFilterNode::callback, this, std::placeholders::_1, std::placeholders::_2));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&KalmanFilterNode::cmdVelCallback, this, std::placeholders::_1)
    );

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/kf_odom", 10);
    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "KF Node ready.");
}

void KalmanFilterNode::initKalman() {
    x_.setZero();
    P_ = Eigen::MatrixXd::Identity(6, 6);
    Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.01;
    R_ = Eigen::MatrixXd::Identity(3, 3) * 0.05;

    C_.setZero();
    C_(0, 1) = 1.0;
    C_(1, 3) = 1.0;
    C_(2, 5) = 1.0;

    u_.setZero();
}

void KalmanFilterNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    u_(0) = msg->linear.x;
    u_(1) = msg->angular.z;
}

void KalmanFilterNode::callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu,
                                 const sensor_msgs::msg::JointState::ConstSharedPtr joint) {
    double dt = (this->now() - last_time_).seconds();
    if (dt <= 0 || dt > 1.0) {
        last_time_ = this->now();
        return;
    }

    predict(dt);
    update(imu, joint);
    publish();

    last_time_ = this->now();
}

void KalmanFilterNode::predict(double dt) {
    A_.setIdentity();
    A_(0, 1) = dt;
    A_(2, 3) = dt;
    A_(4, 5) = dt;

    B_.setZero();
    B_(1, 0) = 1.0;
    B_(5, 1) = 1.0;

    x_ = A_ * x_ + B_ * u_;
    P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilterNode::update(const sensor_msgs::msg::Imu::ConstSharedPtr imu,
                               const sensor_msgs::msg::JointState::ConstSharedPtr joint) {
    double v = 0.0;
    for (size_t i = 0; i < joint->name.size(); ++i) {
        if (joint->name[i] == "wheel_left_joint" || joint->name[i] == "wheel_right_joint") {
            v += joint->velocity[i];
        }
    }
    v = (v * 0.5) * 0.033;

    Eigen::VectorXd z(3);
    z << v, 0.0, imu->angular_velocity.z;

    Eigen::VectorXd y = z - C_ * x_;
    Eigen::MatrixXd S = C_ * P_ * C_.transpose() + R_;
    Eigen::MatrixXd K = P_ * C_.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * C_) * P_;
}

void KalmanFilterNode::publish() {
    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = this->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

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

    odom_pub_->publish(msg);
}
