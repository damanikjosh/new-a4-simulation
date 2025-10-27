#ifndef MISSION_PLANNER__ACTIONS__FOLLOWME_ACTION_HPP_
#define MISSION_PLANNER__ACTIONS__FOLLOWME_ACTION_HPP_

#include "mission_planner/actions/action.hpp"
#include "mission_planner_msgs/action/followme.hpp"
#include "mission_planner_msgs/msg/position.hpp"

namespace mission_planner
{

class FollowMeActionHandler : public BaseActionHandler<mission_planner_msgs::action::Followme>
{
public:
  using FollowMeAction = mission_planner_msgs::action::Followme;
  using GoalHandleFollowMe = rclcpp_action::ServerGoalHandle<FollowMeAction>;

  explicit FollowMeActionHandler(Vehicle* vehicle);

  void stop() override;

protected:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowMeAction::Goal> goal) override;

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFollowMe> goal_handle) override;

  void handle_accepted(const std::shared_ptr<GoalHandleFollowMe> goal_handle) override;

  void execute(const std::shared_ptr<GoalHandleFollowMe> goal_handle) override;

  void timer_callback() override;

private:
  // FollowMe-specific members
  rclcpp::Subscription<mission_planner_msgs::msg::Position>::SharedPtr target_pos_sub_;
  std::string current_target_name_;
  mission_planner_msgs::msg::Position target_position_;
  std::shared_ptr<FollowMeAction::Feedback> feedback_;
  std::shared_ptr<FollowMeAction::Result> result_;
};

}  // namespace mission_planner

#endif  // MISSION_PLANNER__ACTIONS__FOLLOWME_ACTION_HPP_
