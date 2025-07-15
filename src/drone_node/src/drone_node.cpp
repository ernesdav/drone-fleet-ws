#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "drone_node/drone_node.h"

DroneNode::DroneNode(const rclcpp::NodeOptions & options) : Node("drone_node", options)
{
    this->action_server_ = rclcpp_action::create_server<ExecuteMission>(
            this,
            "execute_mission",
            std::bind(&DroneNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DroneNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&DroneNode::handle_accepted, this, std::placeholders::_1));
            
            RCLCPP_INFO(this->get_logger(), "DroneNode Action Server has been started.");
}

rclcpp_action::GoalResponse DroneNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteMission::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with type %s", goal->mission_type.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DroneNode::handle_cancel(
    const std::shared_ptr<GoalHandleExecuteMission> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void DroneNode::handle_accepted(const std::shared_ptr<GoalHandleExecuteMission> goal_handle)
{
    std::thread{[goal_handle, this](){
        RCLCPP_INFO(this->get_logger(), "Executing goal...");
        rclcpp::Rate loop_rate(1);  // 1 Hz for feedback every second

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ExecuteMission::Feedback>();
        auto result = std::make_shared<ExecuteMission::Result>();

        // 5 seconds mission simulation
        for (int i=1; i <= 5 && rclcpp::ok(); ++i) {
            if(goal_handle->is_canceling()) {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // Update and publish feedback
            feedback->percent_complete = i * 20.0f;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publishing feedback: %.0f%%", feedback->percent_complete);

            loop_rate.sleep();
        }

        if (rclcpp::ok()) {
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }}.detach();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto drone_action_server = std::make_shared<DroneNode>();
    rclcpp::spin(drone_action_server);

    rclcpp::shutdown();
    return 0;
}
