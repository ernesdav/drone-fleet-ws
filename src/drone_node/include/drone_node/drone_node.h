#ifndef DRONE_NODE_H
#define DRONE_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "fleet_interfaces/action/execute_mission.hpp"

class DroneNode : public rclcpp::Node
{
public:
    using ExecuteMission = fleet_interfaces::action::ExecuteMission;
    using GoalHandleExecuteMission = rclcpp_action::ServerGoalHandle<ExecuteMission>;

    explicit DroneNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    rclcpp_action::Server<ExecuteMission>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ExecuteMission::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleExecuteMission> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleExecuteMission> goal_handle);

    void execute_mission(const std::shared_ptr<GoalHandleExecuteMission> goal_handle);
};

#endif