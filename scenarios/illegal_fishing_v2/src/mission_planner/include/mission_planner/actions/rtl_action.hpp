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

protected:
  bool initialize_task(std::shared_ptr<const typename ReturnToLaunchAction::Goal> goal) override;
  void stop_task() override;
  std::shared_ptr<ReturnToLaunchAction::Result> get_cancel_result() override;
  std::shared_ptr<ReturnToLaunchAction::Result> get_finish_result() override;
  std::shared_ptr<ReturnToLaunchAction::Feedback> get_feedback() override;
  bool is_finished() override;

private:
  mavsdk::Telemetry::Position home_position_;
  std::shared_ptr<ReturnToLaunchAction::Feedback> feedback_;
};

}  // namespace mission_planner

#endif  // MISSION_PLANNER__ACTIONS__RTL_ACTION_HPP_
