#include <cstdio>
#include <memory>
#include <string>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "mission_planner_msgs/msg/heartbeat.hpp"

using namespace std::chrono_literals;

class MissionPlanner : public rclcpp::Node
{
public:
  MissionPlanner()
  : Node("mission_planner")
  {
    // Subscribe to heartbeat topic (/mission_planner/heartbeat)
    heartbeat_sub_ = this->create_subscription<mission_planner_msgs::msg::Heartbeat>(
      "/mission_planner/heartbeat", 10,
      std::bind(&MissionPlanner::heartbeat_callback, this, std::placeholders::_1));

    // Timer to periodically print status
    timer_ = this->create_wall_timer(
      1s, std::bind(&MissionPlanner::status_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Mission Planner node started");
  }

private:
  // Store subscription
  rclcpp::Subscription<mission_planner_msgs::msg::Heartbeat>::SharedPtr heartbeat_sub_;
  
  // Timer for status updates
  rclcpp::TimerBase::SharedPtr timer_;

  // List for storing active vehicles and their last heartbeat times
  std::map<std::string, rclcpp::Time> active_vehicles_;

  void heartbeat_callback(const mission_planner_msgs::msg::Heartbeat::SharedPtr msg)
  {
    // Update the last heartbeat time for the vehicle
    active_vehicles_[msg->vehicle_name] = this->now();
  }

  void status_callback()
  {
    rclcpp::Time now = this->now();
    for (const auto& [vehicle_name, last_heartbeat] : active_vehicles_)
    {
      auto time_diff = now - last_heartbeat;

      // Remove vehicles that haven't sent heartbeat in the last 10 seconds
      if (time_diff.seconds() > 10.0)
      {
        RCLCPP_WARN(this->get_logger(), "Vehicle %s considered inactive, removing from list",
                    vehicle_name.c_str());
        active_vehicles_.erase(vehicle_name);
      }
    }

    // Print active vehicles
    RCLCPP_INFO(this->get_logger(), "Active vehicles:");
    for (const auto& [vehicle_name, last_heartbeat] : active_vehicles_)
    {
      RCLCPP_INFO(this->get_logger(), " - %s (last heartbeat: %.2f seconds ago)",
                  vehicle_name.c_str(), (now - last_heartbeat).seconds());
    }
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
