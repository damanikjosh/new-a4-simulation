#include "mission_planner/actions/action.hpp"
#include "mission_planner/vehicle.hpp"
#include "mission_planner_msgs/action/goto.hpp"
#include "mission_planner_msgs/action/followme.hpp"
#include "mission_planner_msgs/action/waypoints.hpp"
#include "mission_planner_msgs/action/return_to_launch.hpp"

namespace mission_planner
{

// Template method implementations
template<typename ActionT>
typename BaseActionHandler<ActionT>::ActionServer::SharedPtr 
BaseActionHandler<ActionT>::create_server(
  rclcpp::Node* node,
  const std::string& action_name)
{
  using namespace std::placeholders;
  
  action_server_ = rclcpp_action::create_server<ActionT>(
    node,
    action_name,
    std::bind(&BaseActionHandler::handle_goal, this, _1, _2),
    std::bind(&BaseActionHandler::handle_cancel, this, _1),
    std::bind(&BaseActionHandler::handle_accepted, this, _1)
  );
  
  return action_server_;
}

template<typename ActionT>
bool BaseActionHandler<ActionT>::is_active() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_goal_ != nullptr;
}

template<typename ActionT>
void BaseActionHandler<ActionT>::stop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  // Stop timer if running
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  
  if (current_goal_) {
    current_goal_.reset();
  }
}

template<typename ActionT>
void BaseActionHandler<ActionT>::start_timer()
{
  timer_ = vehicle_->create_wall_timer(
    timer_period_,
    [this]() { this->timer_callback(); });
}

template<typename ActionT>
void BaseActionHandler<ActionT>::stop_timer()
{
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
}

// Explicit template instantiations for all action types
template class BaseActionHandler<mission_planner_msgs::action::Goto>;
template class BaseActionHandler<mission_planner_msgs::action::Followme>;
template class BaseActionHandler<mission_planner_msgs::action::Waypoints>;
template class BaseActionHandler<mission_planner_msgs::action::ReturnToLaunch>;

}  // namespace mission_planner
