#ifndef MISSION_PLANNER__ACTIONS__SEARCH_ACTION_HPP_
#define MISSION_PLANNER__ACTIONS__SEARCH_ACTION_HPP_

#include "mission_planner/actions/action.hpp"
#include "mission_planner_msgs/action/search.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mission/mission.h>
#include "mission_planner_msgs/msg/position.hpp"

namespace mission_planner
{

class SearchActionHandler : public BaseActionHandler<mission_planner_msgs::action::Search>
{
public:
  using SearchAction = mission_planner_msgs::action::Search;
  using GoalHandleSearch = rclcpp_action::ServerGoalHandle<SearchAction>;

  explicit SearchActionHandler(Vehicle* vehicle);

protected:
  // BaseActionHandler interface
  bool initialize_task(std::shared_ptr<const SearchAction::Goal> goal) override;
  void stop_task() override;
  std::shared_ptr<SearchAction::Result> get_cancel_result() override;
  std::shared_ptr<SearchAction::Result> get_finish_result() override;
  std::shared_ptr<SearchAction::Feedback> get_feedback() override;
  bool is_finished() override;
  bool is_goal_valid(std::shared_ptr<const SearchAction::Goal> goal) override;

private:
  std::vector<rclcpp::Subscription<mission_planner_msgs::msg::Position>::SharedPtr> target_pos_subs_;
  std::vector<mavsdk::Mission::MissionItem> mission_items_;
  std::shared_ptr<SearchAction::Feedback> feedback_;
  std::shared_ptr<SearchAction::Result> result_;
  
  // Track positions of vehicles we're searching for
  std::map<std::string, mission_planner_msgs::msg::Position> target_positions_;
  float search_radius_m_;
  std::vector<std::string> vehicle_names_;
  
  // Mission progress tracking
  int total_waypoints_;
  int current_waypoint_index_;
};

}  // namespace mission_planner

#endif  // MISSION_PLANNER__ACTIONS__SEARCH_ACTION_HPP_
