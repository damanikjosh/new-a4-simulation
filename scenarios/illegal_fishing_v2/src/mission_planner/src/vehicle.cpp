#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "mission_planner/vehicle.hpp"
#include "mission_planner/enum_helper.hpp"
#include "mission_planner/utils.hpp"
#include "mission_planner/actions/goto_action.hpp"
#include "mission_planner/actions/followme_action.hpp"
#include "mission_planner/actions/waypoints_action.hpp"
#include "mission_planner/actions/rtl_action.hpp"

#include "mission_planner_msgs/msg/goto.hpp"
#include "mission_planner_msgs/msg/waypoints_item.hpp"
#include "mission_planner_msgs/msg/waypoints.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace mission_planner
{

  Vehicle::Vehicle()
      : Node("vehicle")
  {
    // Add vehicle name parameter
    this->declare_parameter<std::string>("vehicle_name", "uav_0");
    this->declare_parameter<int>("vehicle_type", static_cast<int>(VehicleType::UAV));
    this->declare_parameter<int>("port", 14540);

    this->get_parameter("vehicle_name", vehicle_name_);
    int vehicle_type_int;
    this->get_parameter("vehicle_type", vehicle_type_int);
    vehicle_type_ = static_cast<VehicleType>(vehicle_type_int);
    this->get_parameter("port", port_);

    mavsdk::ConnectionResult connection_result = mavsdk_.add_any_connection(
        "udpin://0.0.0.0:" + std::to_string(port_));
    if (connection_result != mavsdk::ConnectionResult::Success)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to vehicle on port %d: %s",
                   port_, connection_result_str(connection_result));
      rclcpp::shutdown();
      return;
    }

    auto system = mavsdk_.first_autopilot(30.0);
    if (!system)
    {
      RCLCPP_ERROR(this->get_logger(), "Timeout waiting for autopilot");
      rclcpp::shutdown();
      return;
    }
    system_ = std::move(*system);
    telemetry_ = std::make_shared<mavsdk::Telemetry>(system_);
    action_ = std::make_shared<mavsdk::Action>(system_);
    mission_ = std::make_shared<mavsdk::Mission>(system_);
    telemetry_->set_rate_position(1.0);

    if (vehicle_type_ == VehicleType::UAV)
    {
      follow_me_ = std::make_shared<mavsdk::FollowMe>(system_);
      
      // Configure FollowMe
      mavsdk::FollowMe::Config config;
      config.follow_height_m = 10.f;   // Minimum height
      config.follow_distance_m = 20.f; // Follow distance
      config.responsiveness = 0.2f;    // Higher responsiveness
      
      mavsdk::FollowMe::Result config_result = follow_me_->set_config(config);
      if (config_result != mavsdk::FollowMe::Result::Success)
      {
        RCLCPP_ERROR(this->get_logger(), "Setting FollowMe configuration failed");
      }
    }

    // Create heartbeat publisher
    heartbeat_pub_ = this->create_publisher<mission_planner_msgs::msg::Heartbeat>("/mission_planner/heartbeat", 10);
    position_pub_ = this->create_publisher<mission_planner_msgs::msg::Position>(vehicle_name_ + "/position", 10);

    // Timer to publish heartbeat periodically
    timer_ = this->create_wall_timer(1000ms, std::bind(&Vehicle::publish_heartbeat, this));

    // Create position subscription
    telemetry_->subscribe_position(
        std::bind(&Vehicle::position_callback, this, std::placeholders::_1));

    telemetry_->subscribe_heading(
        std::bind(&Vehicle::heading_callback, this, std::placeholders::_1));

    telemetry_->subscribe_armed(
        std::bind(&Vehicle::arm_callback, this, std::placeholders::_1));

    telemetry_->subscribe_flight_mode(
        std::bind(&Vehicle::flight_mode_callback, this, std::placeholders::_1));

    // Create action handlers
    goto_action_handler_ = std::make_unique<GotoActionHandler>(this);
    followme_action_handler_ = std::make_unique<FollowMeActionHandler>(this);
    waypoints_action_handler_ = std::make_unique<WaypointsActionHandler>(this);
    rtl_action_handler_ = std::make_unique<RtlActionHandler>(this);

    // Register action servers with this node
    goto_action_handler_->create_server(this, vehicle_name_ + std::string("/task/goto"));
    followme_action_handler_->create_server(this, vehicle_name_ + std::string("/task/followme"));
    waypoints_action_handler_->create_server(this, vehicle_name_ + std::string("/task/waypoints"));
    rtl_action_handler_->create_server(this, vehicle_name_ + std::string("/task/return_to_launch"));

    task_stop_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        vehicle_name_ + std::string("/task/stop"), 10,
        std::bind(&Vehicle::stop_all_tasks, this, std::placeholders::_1));
  }

  void Vehicle::arm_callback(bool is_armed)
  {
    if (is_armed != is_armed_)
    {
      if (is_armed)
      {
        RCLCPP_INFO(this->get_logger(), "%s is armed", vehicle_name_.c_str());
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "%s is disarmed", vehicle_name_.c_str());
      }
    }

    is_armed_ = is_armed;
  }

  void Vehicle::position_callback(mavsdk::Telemetry::Position position)
  {
    current_position_.header.stamp = this->now();
    current_position_.latitude_deg = position.latitude_deg;
    current_position_.longitude_deg = position.longitude_deg;
    current_position_.absolute_altitude_m = position.absolute_altitude_m;
    current_position_.relative_altitude_m = position.relative_altitude_m;

    // Here you can publish the position message if needed
    position_pub_->publish(current_position_);
  }

  void Vehicle::heading_callback(mavsdk::Telemetry::Heading heading)
  {
    current_position_.yaw_deg = static_cast<float>(heading.heading_deg);
  }

  void Vehicle::flight_mode_callback(mavsdk::Telemetry::FlightMode flight_mode)
  {
    if (flight_mode != flight_mode_)
    {
      RCLCPP_INFO(this->get_logger(), "%s flight mode: %s", vehicle_name_.c_str(), flight_mode_str(flight_mode));

      // Don't automatically stop tasks on flight mode change - let action handlers manage completion
    }
    flight_mode_ = flight_mode;
  }

  void Vehicle::publish_heartbeat()
  {
    auto msg = mission_planner_msgs::msg::Heartbeat();
    msg.header.stamp = this->now();
    msg.vehicle_name = vehicle_name_;
    msg.vehicle_type = static_cast<int8_t>(vehicle_type_);

    heartbeat_pub_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Published heartbeat from %s", vehicle_name_.c_str());
  }

  // Action implementations moved to dedicated handler classes in src/actions/
  // Goto/FollowMe/Waypoints/ReturnToLaunch logic has been extracted from Vehicle
  // to keep Vehicle focused and to let the handlers manage goal lifecycle.

  void Vehicle::ensure_armed_and_takeoff()
  {
    if (!is_armed_)
    {
      RCLCPP_INFO(this->get_logger(), "Arming %s", vehicle_name_.c_str());
      auto arm_result = action_->arm();
      if (arm_result != mavsdk::Action::Result::Success)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to arm %s: %s",
                     vehicle_name_.c_str(), action_result_str(arm_result));
        return;
      }
    }

    if (vehicle_type_ == VehicleType::UAV && telemetry_->in_air() == false)
    {
      RCLCPP_INFO(this->get_logger(), "Taking off %s", vehicle_name_.c_str());
      auto takeoff_result = action_->takeoff();
      if (takeoff_result != mavsdk::Action::Result::Success)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to takeoff %s: %s",
                     vehicle_name_.c_str(), action_result_str(takeoff_result));
        return;
      }

      // Wait until flight mode is Hold
      while (telemetry_->in_air() == false || flight_mode_ != mavsdk::Telemetry::FlightMode::Hold)
      {
        RCLCPP_INFO(this->get_logger(), "Waiting for %s to takeoff...", vehicle_name_.c_str());
        std::this_thread::sleep_for(1s);
      }
    }
  }

  void Vehicle::stop_all_tasks(const std_msgs::msg::Empty::SharedPtr /*msg*/)
  {
    RCLCPP_INFO(this->get_logger(), "Stopping all tasks on %s", vehicle_name_.c_str());

    // Stop all action handlers
    if (goto_action_handler_)
    {
      goto_action_handler_->stop();
    }

    if (followme_action_handler_)
    {
      followme_action_handler_->stop();
    }

    if (waypoints_action_handler_)
    {
      waypoints_action_handler_->stop();
    }

    if (rtl_action_handler_)
    {
      rtl_action_handler_->stop();
    }

    current_task_ = TaskType::None;
  }

} // namespace mission_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mission_planner::Vehicle>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
