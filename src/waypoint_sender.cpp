#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>

class WaypointNavigator : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    WaypointNavigator() : Node("waypoint_navigator") {
        this->declare_parameter("waypoints", std::vector<double>{});

        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&WaypointNavigator::trySendNextGoal, this));

        RCLCPP_INFO(this->get_logger(), "WaypointNavigator gestartet");
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t current_goal_ = 0;
    std::vector<geometry_msgs::msg::PoseStamped> goals_;

    void trySendNextGoal() {
        if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Warte auf navigate_to_pose-Server...");
            return;
        }

        if (goals_.empty()) {
            std::vector<double> flat;
            this->get_parameter("waypoints", flat);
            if (flat.size() % 2 != 0) {
                RCLCPP_ERROR(this->get_logger(), "Wegpunkte müssen Paare aus x/y sein.");
                return;
            }

            for (size_t i = 0; i < flat.size(); i += 2) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.pose.position.x = flat[i];
                pose.pose.position.y = flat[i + 1];
                pose.pose.orientation.w = 1.0;
                goals_.push_back(pose);
            }
            RCLCPP_INFO(this->get_logger(), "%zu Wegpunkte geladen", goals_.size());
        }

        if (current_goal_ >= goals_.size()) {
            RCLCPP_INFO(this->get_logger(), "Alle Wegpunkte erreicht.");
            timer_->cancel();
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goals_[current_goal_];
        goal_msg.behavior_tree = "";

        RCLCPP_INFO(this->get_logger(), "Sende Wegpunkt %zu: [%.2f, %.2f]",
            current_goal_, goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Wegpunkt erreicht.");
                current_goal_++;
            } else {
                RCLCPP_WARN(this->get_logger(), "Navigation fehlgeschlagen.");
                current_goal_++;
            }
        };

        client_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointNavigator>());
    rclcpp::shutdown();
    return 0;
}
