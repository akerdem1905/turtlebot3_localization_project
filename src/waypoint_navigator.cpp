#include <memory>
#include <vector>
#include <chrono>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

using namespace std::chrono_literals;

class WaypointNavigator : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    WaypointNavigator() : Node("waypoint_navigator") {
        // Declare and retrieve parameters
        this->declare_parameter<std::vector<double>>("waypoints", {}, rcl_interfaces::msg::ParameterDescriptor{});
        std::vector<double> flat_waypoints;
        this->get_parameter("waypoints", flat_waypoints);

        if (flat_waypoints.size() % 3 != 0) {
            RCLCPP_ERROR(this->get_logger(), "Waypoints must be a flat list of (x, y, yaw) triplets.");
            return;
        }

        for (size_t i = 0; i < flat_waypoints.size(); i += 3) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = flat_waypoints[i];
            pose.pose.position.y = flat_waypoints[i + 1];

            double yaw = flat_waypoints[i + 2];
            pose.pose.orientation.z = sin(yaw / 2.0);
            pose.pose.orientation.w = cos(yaw / 2.0);

            waypoints_.push_back(pose);
        }

        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        timer_ = this->create_wall_timer(2s, std::bind(&WaypointNavigator::start_navigation, this));
    }

private:
    void start_navigation() {
        timer_->cancel();

        if (!client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available!");
            return;
        }

        send_next_goal();
    }

    void send_next_goal() {
        if (current_index_ >= waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(), "All waypoints reached.");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = waypoints_[current_index_];
        goal_msg.pose.header.stamp = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "Sending goal %lu to (%.2f, %.2f)",
                    current_index_,
                    goal_msg.pose.pose.position.x,
                    goal_msg.pose.pose.position.y);

        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
        options.goal_response_callback =
            std::bind(&WaypointNavigator::goal_response_callback, this, std::placeholders::_1);
        options.result_callback =
            std::bind(&WaypointNavigator::result_callback, this, std::placeholders::_1);

        client_->async_send_goal(goal_msg, options);
    }

    void goal_response_callback(GoalHandleNavigateToPose::SharedPtr goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return;
        }

        client_->async_get_result(goal_handle,
            std::bind(&WaypointNavigator::result_callback, this, std::placeholders::_1));
    }

    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Goal %lu reached successfully.", current_index_);
            current_index_++;
            send_next_goal();
        } else {
            RCLCPP_WARN(this->get_logger(), "Goal %lu failed or was canceled.", current_index_);
        }
    }

    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t current_index_ = 0;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointNavigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
