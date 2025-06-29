#include <cmath>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"

struct PFParticle {
    double x;
    double y;
    double heading;
    double weight;

    PFParticle() : x(0.0), y(0.0), heading(0.0), weight(1.0) {}
};

class ParticleFilterNode : public rclcpp::Node {
public:
    ParticleFilterNode()
        : Node("particle_filter_node"), gen_(std::random_device{}()) {
        
        // Declare parameters
        particle_count_ = this->declare_parameter("num_particles", 1000);
        wheel_radius_ = this->declare_parameter("wheel_radius", 0.033);
        wheel_base_ = this->declare_parameter("wheelbase", 0.287);
        noise_vel_ = this->declare_parameter("measurement_noise_v", 0.05);
        noise_omega_ = this->declare_parameter("measurement_noise_omega", 0.02);
        motion_noise_ = this->declare_parameter("motion_noise", 0.001);

        initializeParticles();

        imu_sub_.subscribe(this, "/imu");
        joint_sub_.subscribe(this, "/joint_states");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), imu_sub_, joint_sub_);
        sync_->registerCallback(std::bind(
            &ParticleFilterNode::sensorCallback, this,
            std::placeholders::_1, std::placeholders::_2));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/pf_odom", 10);
        last_update_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Particle Filter Node gestartet");
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Imu, sensor_msgs::msg::JointState>;

    std::vector<PFParticle> particles_;
    int particle_count_;
    double wheel_radius_, wheel_base_;
    double noise_vel_, noise_omega_, motion_noise_;
    rclcpp::Time last_update_;

    std::mt19937 gen_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    message_filters::Subscriber<sensor_msgs::msg::JointState> joint_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    void initializeParticles() {
        std::uniform_real_distribution<double> pos_dist(-0.1, 0.1);
        std::uniform_real_distribution<double> angle_dist(-0.1, 0.1);

        particles_.resize(particle_count_);
        for (auto& p : particles_) {
            p.x = pos_dist(gen_);
            p.y = pos_dist(gen_);
            p.heading = angle_dist(gen_);
            p.weight = 1.0 / particle_count_;
        }
    }

    void sensorCallback(
        const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
        const sensor_msgs::msg::JointState::ConstSharedPtr joint_msg) {
        
        double dt = (this->now() - last_update_).seconds();
        if (dt <= 0.0 || dt > 0.5) {
            last_update_ = this->now();
            return;
        }

        auto [v, omega] = computeWheelVelocities(joint_msg);
        double omega_measured = imu_msg->angular_velocity.z;

        predictParticles(dt, v, omega);
        applyMeasurementUpdate(v, omega_measured);
        performResampling();
        publishEstimation();

        last_update_ = this->now();
    }

    std::pair<double, double> computeWheelVelocities(
        const sensor_msgs::msg::JointState::ConstSharedPtr joint_msg) {
        double v_left = 0.0, v_right = 0.0;

        for (size_t i = 0; i < joint_msg->name.size(); ++i) {
            if (joint_msg->name[i] == "wheel_left_joint")
                v_left = joint_msg->velocity[i] * wheel_radius_;
            else if (joint_msg->name[i] == "wheel_right_joint")
                v_right = joint_msg->velocity[i] * wheel_radius_;
        }

        double v = (v_left + v_right) / 2.0;
        double omega = (v_right - v_left) / wheel_base_;
        return {v, omega};
    }

    void predictParticles(double dt, double v, double omega) {
        std::normal_distribution<double> noise(0.0, motion_noise_);

        for (auto& p : particles_) {
            if (std::abs(omega) < 1e-5) {
                p.x += v * std::cos(p.heading) * dt;
                p.y += v * std::sin(p.heading) * dt;
            } else {
                double R = v / omega;
                double dtheta = omega * dt;

                p.x += R * (std::sin(p.heading + dtheta) - std::sin(p.heading));
                p.y += R * (std::cos(p.heading) - std::cos(p.heading + dtheta));
                p.heading += dtheta;
            }

            p.x += noise(gen_) * dt;
            p.y += noise(gen_) * dt;
            p.heading = normalizeAngle(p.heading + noise(gen_) * dt);
        }
    }

    void applyMeasurementUpdate(double v_meas, double omega_meas) {
        double sum_weights = 0.0;

        for (auto& p : particles_) {
            // Simple dummy likelihood model for now
            double likelihood_v = 1.0;
            double likelihood_omega = 1.0;

            // TODO: replace with better model (optional)
            p.weight = likelihood_v * likelihood_omega;
            sum_weights += p.weight;
        }

        if (sum_weights > 0.0) {
            for (auto& p : particles_) {
                p.weight /= sum_weights;
            }
        }
    }

    void performResampling() {
        double n_eff = 0.0;
        for (const auto& p : particles_) {
            n_eff += p.weight * p.weight;
        }
        n_eff = 1.0 / n_eff;

        if (n_eff < particle_count_ / 2.0) {
            std::vector<PFParticle> resampled;
            resampled.reserve(particle_count_);

            std::uniform_real_distribution<double> dist(0.0, 1.0 / particle_count_);
            double u = dist(gen_);
            double cumsum = particles_[0].weight;
            int index = 0;

            for (int i = 0; i < particle_count_; ++i) {
                double threshold = u + i * (1.0 / particle_count_);
                while (threshold > cumsum && index < particle_count_ - 1) {
                    index++;
                    cumsum += particles_[index].weight;
                }
                PFParticle new_p = particles_[index];
                new_p.weight = 1.0 / particle_count_;
                resampled.push_back(new_p);
            }

            particles_ = std::move(resampled);
        }
    }

    void publishEstimation() {
        double mean_x = 0.0, mean_y = 0.0;
        double sin_sum = 0.0, cos_sum = 0.0;

        for (const auto& p : particles_) {
            mean_x += p.weight * p.x;
            mean_y += p.weight * p.y;
            sin_sum += p.weight * std::sin(p.heading);
            cos_sum += p.weight * std::cos(p.heading);
        }

        double avg_theta = std::atan2(sin_sum, cos_sum);

        nav_msgs::msg::Odometry est_msg;
        est_msg.header.stamp = this->now();
        est_msg.header.frame_id = "odom";
        est_msg.child_frame_id = "base_link";

        est_msg.pose.pose.position.x = mean_x;
        est_msg.pose.pose.position.y = mean_y;
        est_msg.pose.pose.orientation.z = std::sin(avg_theta / 2.0);
        est_msg.pose.pose.orientation.w = std::cos(avg_theta / 2.0);

        odom_pub_->publish(est_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Partikel-Filter: x=%.3f, y=%.3f, θ=%.3f", mean_x, mean_y, avg_theta);
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleFilterNode>());
    rclcpp::shutdown();
    return 0;
}
