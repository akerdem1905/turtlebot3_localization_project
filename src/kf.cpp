#include <cmath>
#include <random>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"

class KalmanFilterNode : public rclcpp::Node {
public:
    KalmanFilterNode() : Node("kalman_filter_node") {
        state_ = Eigen::VectorXd::Zero(3);  // [x, y, theta]
        covariance_ = Eigen::MatrixXd::Identity(3, 3) * 0.01;

        // Prozessrauschen: kleine Unsicherheit bei Bewegung
        Q_ = Eigen::MatrixXd::Zero(3, 3);
        Q_(0, 0) = 0.002;  // x
        Q_(1, 1) = 0.002;  // y
        Q_(2, 2) = 0.005;  // theta

        // Messrauschen: nur ω (gyro) wird gemessen
        R_ = Eigen::MatrixXd::Identity(1, 1);
        R_(0, 0) = 0.02;  // Gyro noise

        imu_sub_.subscribe(this, "/imu");
        joint_sub_.subscribe(this, "/joint_states");
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), imu_sub_, joint_sub_);
        sync_->registerCallback(std::bind(&KalmanFilterNode::callback, this, std::placeholders::_1, std::placeholders::_2));

        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/kf_prediction", 10);
        last_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Kalman Filter Node gestartet");
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, sensor_msgs::msg::JointState>;

    Eigen::VectorXd state_;
    Eigen::MatrixXd covariance_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    rclcpp::Time last_time_;

    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    message_filters::Subscriber<sensor_msgs::msg::JointState> joint_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;

    void callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
                  const sensor_msgs::msg::JointState::ConstSharedPtr joint_msg) {
        double dt = (this->now() - last_time_).seconds();
        if (dt <= 0.0 || dt > 1.0) {
            last_time_ = this->now();
            return;
        }
        last_time_ = this->now();

        auto [v, omega] = computeVelocities(joint_msg);
        predict(dt, v, omega);
        correct(imu_msg);
        publish();
    }

    std::pair<double, double> computeVelocities(const sensor_msgs::msg::JointState::ConstSharedPtr joint_msg) {
        double v_left = 0.0, v_right = 0.0;
        double wheel_radius = 0.033;
        double wheel_base = 0.287;

        for (size_t i = 0; i < joint_msg->name.size(); ++i) {
            if (joint_msg->name[i] == "wheel_left_joint")
                v_left = joint_msg->velocity[i] * wheel_radius;
            else if (joint_msg->name[i] == "wheel_right_joint")
                v_right = joint_msg->velocity[i] * wheel_radius;
        }

        double v = (v_left + v_right) * 0.5;
        double omega = (v_right - v_left) / wheel_base;
        return {v, omega};
    }

    void predict(double dt, double v, double omega) {
        double theta = state_(2);

        // Bewegungsmodell
        Eigen::VectorXd u(2);
        u << v * dt, omega * dt;

        Eigen::VectorXd predicted = state_;
        predicted(0) += u(0) * std::cos(theta); // x
        predicted(1) += u(0) * std::sin(theta); // y
        predicted(2) += u(1);                   // theta

        // Ableitung des Bewegungsmodells (Jacobi-Matrix)
        Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
        A(0, 2) = -u(0) * std::sin(theta);
        A(1, 2) =  u(0) * std::cos(theta);

        // Kontrollmatrix
        Eigen::MatrixXd B(3, 2);
        B << std::cos(theta), 0,
             std::sin(theta), 0,
             0, 1;

        state_ = predicted;
        covariance_ = A * covariance_ * A.transpose() + Q_;
        state_(2) = normalizeAngle(state_(2));
    }

    void correct(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) {
        double omega_measured = imu_msg->angular_velocity.z;

        // Messvektor (nur ω)
        Eigen::VectorXd z(1);
        z << omega_measured;

        // Messmatrix (bezieht sich nur auf θ)
        Eigen::MatrixXd H(1, 3);
        H << 0, 0, 1;

        // Kalman Gain
        Eigen::MatrixXd S = H * covariance_ * H.transpose() + R_;
        Eigen::MatrixXd K = covariance_ * H.transpose() * S.inverse();

        // Innovation (Messfehler)
        Eigen::VectorXd y = z - H * state_;

        state_ += K * y;
        covariance_ = (Eigen::MatrixXd::Identity(3, 3) - K * H) * covariance_;
        state_(2) = normalizeAngle(state_(2));
    }

    void publish() {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";

        msg.pose.pose.position.x = state_(0);
        msg.pose.pose.position.y = state_(1);
        msg.pose.pose.orientation.z = std::sin(state_(2) / 2.0);
        msg.pose.pose.orientation.w = std::cos(state_(2) / 2.0);

        msg.pose.covariance[0] = covariance_(0, 0);
        msg.pose.covariance[7] = covariance_(1, 1);
        msg.pose.covariance[35] = covariance_(2, 2);

        pub_->publish(msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "KF: x=%.3f, y=%.3f, θ=%.3f", state_(0), state_(1), state_(2));
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
