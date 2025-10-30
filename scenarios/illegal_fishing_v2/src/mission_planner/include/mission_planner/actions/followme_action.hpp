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

protected:
  bool initialize_task(std::shared_ptr<const typename FollowMeAction::Goal> goal) override;
  void stop_task() override;
  std::shared_ptr<FollowMeAction::Result> get_cancel_result() override;
  std::shared_ptr<FollowMeAction::Result> get_finish_result() override;
  std::shared_ptr<FollowMeAction::Feedback> get_feedback() override;
  bool is_finished() override;
  bool is_goal_valid(const std::shared_ptr<const typename FollowMeAction::Goal> goal) override;

private:
  // FollowMe-specific members
  rclcpp::Subscription<mission_planner_msgs::msg::Position>::SharedPtr target_pos_sub_;
  std::string current_target_name_;
  mission_planner_msgs::msg::Position target_position_;
  std::shared_ptr<FollowMeAction::Feedback> feedback_;
};

}  // namespace mission_planner

#endif  // MISSION_PLANNER__ACTIONS__FOLLOWME_ACTION_HPP_
