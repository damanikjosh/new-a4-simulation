#ifndef MISSION_PLANNER__ACTIONS__GOTO_ACTION_HPP_
#define MISSION_PLANNER__ACTIONS__GOTO_ACTION_HPP_

#include "mission_planner/actions/action.hpp"
#include "mission_planner_msgs/action/goto.hpp"

namespace mission_planner
{

class GotoActionHandler : public BaseActionHandler<mission_planner_msgs::action::Goto>
{
public:
  using GotoAction = mission_planner_msgs::action::Goto;
  using GoalHandleGoto = rclcpp_action::ServerGoalHandle<GotoAction>;

  explicit GotoActionHandler(Vehicle* vehicle);

  void stop() override;

protected:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GotoAction::Goal> goal) override;

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGoto> goal_handle) override;

  void handle_accepted(const std::shared_ptr<GoalHandleGoto> goal_handle) override;

  void execute(const std::shared_ptr<GoalHandleGoto> goal_handle) override;

  void timer_callback() override;

private:
  std::shared_ptr<GotoAction::Goal> current_goal_msg_;
  std::shared_ptr<GotoAction::Feedback> feedback_;
  std::shared_ptr<GotoAction::Result> result_;
};

}  // namespace mission_planner

#endif  // MISSION_PLANNER__ACTIONS__GOTO_ACTION_HPP_
