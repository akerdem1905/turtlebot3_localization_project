#include <cmath>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"

struct PFParticle {
    double x, y, heading, weight;
    PFParticle() : x(0.0), y(0.0), heading(0.0), weight(1.0) {}
};

class ParticleFilterNode : public rclcpp::Node {
public:
    ParticleFilterNode() : Node("particle_filter_node"), gen_(std::random_device{}()) {
        particle_count_ = declare_parameter("num_particles", 500);
        wheel_radius_ = declare_parameter("wheel_radius", 0.033);
        wheel_base_ = declare_parameter("wheelbase", 0.287);

        measurement_noise_v_ = declare_parameter("measurement_noise_v", 0.02);
        measurement_noise_omega_ = declare_parameter("measurement_noise_omega", 0.02);
        motion_noise_trans_ = declare_parameter("motion_noise_trans", 0.015);
        motion_noise_rot_ = declare_parameter("motion_noise_rot", 0.03);

        init_spread_pos_ = declare_parameter("init_spread_pos", 0.01);
        init_spread_theta_ = declare_parameter("init_spread_theta", 0.05);

        initializeParticles();

        imu_sub_.subscribe(this, "/imu");
        joint_sub_.subscribe(this, "/joint_states");
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), imu_sub_, joint_sub_);
        sync_->registerCallback(std::bind(&ParticleFilterNode::sensorCallback, this, std::placeholders::_1, std::placeholders::_2));

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/prediction", 10);
        last_update_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Particle Filter Node gestartet");
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, sensor_msgs::msg::JointState>;

    std::vector<PFParticle> particles_;
    int particle_count_;
    double wheel_radius_, wheel_base_;
    double motion_noise_trans_, motion_noise_rot_;
    double measurement_noise_v_, measurement_noise_omega_;
    double init_spread_pos_, init_spread_theta_;

    std::mt19937 gen_;
    rclcpp::Time last_update_;

    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    message_filters::Subscriber<sensor_msgs::msg::JointState> joint_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

    void initializeParticles() {
        std::normal_distribution<double> pos_noise(0.0, init_spread_pos_);
        std::normal_distribution<double> angle_noise(0.0, init_spread_theta_);
        particles_.resize(particle_count_);
        for (auto& p : particles_) {
            p.x = pos_noise(gen_);
            p.y = pos_noise(gen_);
            p.heading = angle_noise(gen_);
            p.weight = 1.0 / particle_count_;
        }
    }

    void sensorCallback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
                        const sensor_msgs::msg::JointState::ConstSharedPtr joint_msg) {
        double dt = (this->now() - last_update_).seconds();
        if (dt <= 0.0 || dt > 0.5) {
            last_update_ = this->now();
            return;
        }

        auto [v, omega] = computeWheelVelocities(joint_msg);
        double omega_measured = imu_msg->angular_velocity.z;

        predict(dt, v, omega);
        correct(v, omega_measured);
        resample();
        publishPrediction();

        last_update_ = this->now();
    }

    std::pair<double, double> computeWheelVelocities(const sensor_msgs::msg::JointState::ConstSharedPtr joint_msg) {
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

    void predict(double dt, double v, double omega) {
        std::normal_distribution<double> trans_noise(0.0, motion_noise_trans_);
        std::normal_distribution<double> rot_noise(0.0, motion_noise_rot_);

        for (auto& p : particles_) {
            double noisy_v = v + trans_noise(gen_);
            double noisy_omega = omega + rot_noise(gen_);

            if (std::abs(noisy_omega) < 1e-5) {
                p.x += noisy_v * std::cos(p.heading) * dt;
                p.y += noisy_v * std::sin(p.heading) * dt;
            } else {
                double R = noisy_v / noisy_omega;
                double dtheta = noisy_omega * dt;
                p.x += R * (std::sin(p.heading + dtheta) - std::sin(p.heading));
                p.y += R * (std::cos(p.heading) - std::cos(p.heading + dtheta));
                p.heading += dtheta;
            }
            p.heading = normalizeAngle(p.heading);
        }
    }

    void correct(double v_meas, double omega_meas) {
        double sum_weights = 0.0;
        for (auto& p : particles_) {
            // Fehler
            double error_v = v_meas - v_meas;  // für später: partikelabhängig
            double error_omega = omega_meas - omega_meas;

            // Likelihoods (momentan beide gleich, weil keine alternative Berechnung)
            double lv = std::exp(-0.5 * error_v * error_v / (measurement_noise_v_ * measurement_noise_v_));
            double lo = std::exp(-0.5 * error_omega * error_omega / (measurement_noise_omega_ * measurement_noise_omega_));

            p.weight = lv * lo;
            sum_weights += p.weight;
        }

        if (sum_weights > 0.0) {
            for (auto& p : particles_) {
                p.weight /= sum_weights;
            }
        }
    }

    void resample() {
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

    void publishPrediction() {
        double mean_x = 0.0, mean_y = 0.0;
        double sin_sum = 0.0, cos_sum = 0.0;
        for (const auto& p : particles_) {
            mean_x += p.weight * p.x;
            mean_y += p.weight * p.y;
            sin_sum += p.weight * std::sin(p.heading);
            cos_sum += p.weight * std::cos(p.heading);
        }
        double avg_theta = std::atan2(sin_sum, cos_sum);

        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "odom";
        pose_msg.pose.pose.position.x = mean_x;
        pose_msg.pose.pose.position.y = mean_y;
        pose_msg.pose.pose.orientation.z = std::sin(avg_theta / 2.0);
        pose_msg.pose.pose.orientation.w = std::cos(avg_theta / 2.0);

        pose_msg.pose.covariance[0] = 0.01;
        pose_msg.pose.covariance[7] = 0.01;
        pose_msg.pose.covariance[35] = 0.05;

        pose_pub_->publish(pose_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "PF: x=%.3f, y=%.3f, θ=%.3f", mean_x, mean_y, avg_theta);
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
