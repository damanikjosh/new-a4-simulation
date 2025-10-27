#ifndef MISSION_PLANNER__ACTIONS__ACTION_HPP_
#define MISSION_PLANNER__ACTIONS__ACTION_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace mission_planner
{

// Forward declaration
class Vehicle;

template<typename ActionT>
class BaseActionHandler
{
public:
  using ActionType = ActionT;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using ActionServer = rclcpp_action::Server<ActionT>;

  explicit BaseActionHandler(Vehicle* vehicle, std::chrono::milliseconds timer_period = std::chrono::milliseconds(100))
    : vehicle_(vehicle), timer_period_(timer_period)
  {
  }

  virtual ~BaseActionHandler() = default;

  // Create and register the action server
  typename ActionServer::SharedPtr create_server(
    rclcpp::Node* node,
    const std::string& action_name);

  // Check if action is currently active
  virtual bool is_active() const;

  // Stop the action - derived classes can override for custom cleanup
  virtual void stop();

protected:
  // These must be implemented by derived classes
  virtual rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const typename ActionT::Goal> goal) = 0;

  virtual rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle) = 0;

  virtual void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) = 0;

  virtual void execute(const std::shared_ptr<GoalHandle> goal_handle) = 0;

  // Timer callback - must be implemented by derived classes
  virtual void timer_callback() = 0;

  // Helper to start timer (call this from execute after setup is complete)
  // Uses the period set during construction
  void start_timer();

  // Helper to stop timer
  void stop_timer();

  // Protected members accessible to derived classes
  Vehicle* vehicle_;
  std::shared_ptr<GoalHandle> current_goal_;
  mutable std::mutex mutex_;
  typename ActionServer::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds timer_period_;
};

}  // namespace mission_planner

#endif  // MISSION_PLANNER__ACTIONS__ACTION_HPP_
