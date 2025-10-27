#ifndef MISSION_PLANNER__ACTIONS__RTL_ACTION_HPP_
#define MISSION_PLANNER__ACTIONS__RTL_ACTION_HPP_

#include "mission_planner/actions/action.hpp"
#include "mission_planner_msgs/action/return_to_launch.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

namespace mission_planner
{

class RtlActionHandler : public BaseActionHandler<mission_planner_msgs::action::ReturnToLaunch>
{
public:
  using ReturnToLaunchAction = mission_planner_msgs::action::ReturnToLaunch;
  using GoalHandleReturnToLaunch = rclcpp_action::ServerGoalHandle<ReturnToLaunchAction>;

  explicit RtlActionHandler(Vehicle* vehicle);

  void stop() override;

protected:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ReturnToLaunchAction::Goal> goal) override;

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleReturnToLaunch> goal_handle) override;

  void handle_accepted(const std::shared_ptr<GoalHandleReturnToLaunch> goal_handle) override;

  void execute(const std::shared_ptr<GoalHandleReturnToLaunch> goal_handle) override;

  void timer_callback() override;

private:
  mavsdk::Telemetry::Position home_position_;
  std::shared_ptr<ReturnToLaunchAction::Feedback> feedback_;
  std::shared_ptr<ReturnToLaunchAction::Result> result_;
};

}  // namespace mission_planner

#endif  // MISSION_PLANNER__ACTIONS__RTL_ACTION_HPP_
