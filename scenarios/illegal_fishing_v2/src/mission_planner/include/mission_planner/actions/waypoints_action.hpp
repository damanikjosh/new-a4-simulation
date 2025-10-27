#ifndef MISSION_PLANNER__ACTIONS__WAYPOINTS_ACTION_HPP_
#define MISSION_PLANNER__ACTIONS__WAYPOINTS_ACTION_HPP_

#include "mission_planner/actions/action.hpp"
#include "mission_planner_msgs/action/waypoints.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mission/mission.h>

namespace mission_planner
{

class WaypointsActionHandler : public BaseActionHandler<mission_planner_msgs::action::Waypoints>
{
public:
  using WaypointsAction = mission_planner_msgs::action::Waypoints;
  using GoalHandleWaypoints = rclcpp_action::ServerGoalHandle<WaypointsAction>;

  explicit WaypointsActionHandler(Vehicle* vehicle);

  void stop() override;

protected:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const WaypointsAction::Goal> goal) override;

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleWaypoints> goal_handle) override;

  void handle_accepted(const std::shared_ptr<GoalHandleWaypoints> goal_handle) override;

  void execute(const std::shared_ptr<GoalHandleWaypoints> goal_handle) override;

  void timer_callback() override;

private:
  std::vector<mavsdk::Mission::MissionItem> mission_items_;
  std::shared_ptr<WaypointsAction::Feedback> feedback_;
  std::shared_ptr<WaypointsAction::Result> result_;
};

}  // namespace mission_planner

#endif  // MISSION_PLANNER__ACTIONS__WAYPOINTS_ACTION_HPP_
