#ifndef MISSION_PLANNER__ACTIONS__WAYPOINTS_ACTION_HPP_
#define MISSION_PLANNER__ACTIONS__WAYPOINTS_ACTION_HPP_

#include "mission_planner/actions/action.hpp"
#include "mission_planner/trajectory_generator.hpp"
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

protected:
  bool initialize_task(std::shared_ptr<const typename WaypointsAction::Goal> goal) override;
  void stop_task() override;
  std::shared_ptr<WaypointsAction::Result> get_cancel_result() override;
  std::shared_ptr<WaypointsAction::Result> get_finish_result() override;
  std::shared_ptr<WaypointsAction::Feedback> get_feedback() override;
  bool is_finished() override;
  bool is_goal_valid(const std::shared_ptr<const typename WaypointsAction::Goal> goal) override;

private:
  std::vector<mavsdk::Mission::MissionItem> mission_items_;
  std::shared_ptr<WaypointsAction::Feedback> feedback_;
  std::shared_ptr<const typename WaypointsAction::Goal> current_goal_msg_;
  
  // Trajectory generation
  std::shared_ptr<TrajectoryGenerator> trajectory_generator_;
  
  // Mapping from mission item index to original waypoint index
  // -1 for intermittent waypoints, >= 0 for actual waypoints
  std::vector<int> mission_item_to_waypoint_index_;
};

}  // namespace mission_planner

#endif  // MISSION_PLANNER__ACTIONS__WAYPOINTS_ACTION_HPP_
