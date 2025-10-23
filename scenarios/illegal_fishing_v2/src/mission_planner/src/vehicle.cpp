#include <chrono>
#include <memory>
#include <string>
#include <optional>
#include <vector>
#include <map>
#include <mutex>
#include <sstream>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/follow_me/follow_me.h>
#include <mavsdk/plugins/action/action.h>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "mission_planner/enum_helper.hpp"
#include "mission_planner/vehicle_type.hpp"
#include "mission_planner/mission_type.hpp"

#include "mission_planner_msgs/msg/heartbeat.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using namespace std::chrono_literals;


class Vehicle : public rclcpp::Node
{
public:
  Vehicle()
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
    if (connection_result != mavsdk::ConnectionResult::Success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to vehicle on port %d: %s",
        port_, connection_result_str(connection_result));
        rclcpp::shutdown();
        return;
    }

    auto system = mavsdk_.first_autopilot(30.0);
    if (!system) {
      RCLCPP_ERROR(this->get_logger(), "Timeout waiting for autopilot");
      rclcpp::shutdown();
      return;
    }
    system_ = std::move(*system);
    telemetry_ = std::make_shared<mavsdk::Telemetry>(system_);
    telemetry_->set_rate_position(1.0);
    action_ = std::make_shared<mavsdk::Action>(system_);

    // If UAV, initialize FollowMe plugin
    if (vehicle_type_ == VehicleType::UAV) {
      initialize_followme();
    }

    // Create heartbeat publisher
    heartbeat_pub_ = this->create_publisher<mission_planner_msgs::msg::Heartbeat>("/mission_planner/heartbeat", 10);
    position_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(vehicle_name_ + "/position", 10);

    // Timer to publish heartbeat periodically
    timer_ = this->create_wall_timer(1000ms, std::bind(&Vehicle::publish_heartbeat, this));

    // Create position subscription
    telemetry_->subscribe_position(
      std::bind(&Vehicle::position_callback, this, std::placeholders::_1)
    );

    telemetry_->subscribe_armed(
      std::bind(&Vehicle::arm_callback, this, std::placeholders::_1)
    );

    telemetry_->subscribe_flight_mode(
      std::bind(&Vehicle::flight_mode_callback, this, std::placeholders::_1)
    );

    // Mission subscribers: followme commands are std_msgs/String
    mission_followme_sub_ = this->create_subscription<std_msgs::msg::String>(
      vehicle_name_ + "/mission/followme", 10,
      std::bind(&Vehicle::followme_callback, this, std::placeholders::_1)
    );

    mission_stop_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      vehicle_name_ + "/mission/stop", 10,
      std::bind(&Vehicle::stop_all_missions, this, std::placeholders::_1)
    );

    mission_return_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      vehicle_name_ + "/mission/return", 10,
      std::bind(&Vehicle::return_to_launch, this, std::placeholders::_1)
    );

  }

private:
  std::string vehicle_name_;
  VehicleType vehicle_type_;
  int port_;

  rclcpp::Publisher<mission_planner_msgs::msg::Heartbeat>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr position_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_followme_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr mission_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr mission_return_sub_;

  // Dynamic subscription to the target vehicle's position topic
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr target_pos_sub_;
  std::string current_target_name_;
  std::mutex mission_mutex_;
  
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<mavsdk::System> system_;
  std::shared_ptr<mavsdk::Telemetry> telemetry_;
  std::shared_ptr<mavsdk::Action> action_;

  // Missions
  std::shared_ptr<mavsdk::FollowMe> follow_me_;

  bool is_armed_ = false;
  std::atomic<mavsdk::Telemetry::FlightMode> flight_mode_;
  std::atomic<MissionType> current_mission_{MissionType::None};

  // Keep mavsdk instance as a member so System's impl has a valid parent.
  mavsdk::Mavsdk mavsdk_{mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}};

  void arm_callback(bool is_armed)
  {
    if (is_armed != is_armed_) {
      if (is_armed) {
        RCLCPP_INFO(this->get_logger(), "%s is armed", vehicle_name_.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "%s is disarmed", vehicle_name_.c_str());
      }
    }

    is_armed_ = is_armed;

  }

  void position_callback(mavsdk::Telemetry::Position position)
  {
    auto msg = sensor_msgs::msg::NavSatFix();
    msg.header.stamp = this->now();
    msg.latitude = position.latitude_deg;
    msg.longitude = position.longitude_deg;
    msg.altitude = position.absolute_altitude_m;

    // Here you can publish the position message if needed
    position_pub_->publish(msg);
  }

  void flight_mode_callback(mavsdk::Telemetry::FlightMode flight_mode)
  {
    if (flight_mode != flight_mode_) {
      RCLCPP_INFO(this->get_logger(), "%s flight mode: %s", vehicle_name_.c_str(), flight_mode_str(flight_mode));

      if ((flight_mode == mavsdk::Telemetry::FlightMode::Hold || flight_mode == mavsdk::Telemetry::FlightMode::ReturnToLaunch) &&
          current_mission_ != MissionType::None) {
        RCLCPP_WARN(this->get_logger(),
          "%s entered Hold mode while on mission. Stopping current mission.",
          vehicle_name_.c_str());
          current_mission_ = MissionType::None;
          stop_all_missions();
      }
    }
    flight_mode_ = flight_mode;
  }

  void publish_heartbeat()
  {
    auto msg = mission_planner_msgs::msg::Heartbeat();
    msg.header.stamp = this->now();
    msg.vehicle_name = vehicle_name_;
    msg.vehicle_type = static_cast<int8_t>(vehicle_type_);

    heartbeat_pub_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Published heartbeat from %s", vehicle_name_.c_str());
  }

  void followme_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string target = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received followme command: %s", target.c_str());
    std::lock_guard<std::mutex> lk(mission_mutex_);

    // If already following this target, do nothing
    if (!current_target_name_.empty() && current_target_name_ == target) {
      RCLCPP_INFO(this->get_logger(), "Already following %s", target.c_str());
      return;
    }

    // unsubscribe previous target subscription if present
    if (target_pos_sub_) {
      target_pos_sub_.reset();
      current_target_name_.clear();
    }

    if (target.empty()) {
      RCLCPP_INFO(this->get_logger(), "Follow target cleared");
      return;
    }

    // Ensure vehicle is armed and start FollowMe (these may block); do them outside mutex
    ensure_armed_and_takeoff();
    if (follow_me_) {
      follow_me_->start();
    }

    // Now subscribe and set mission state under lock
    std::string topic = target + "/position";
    RCLCPP_INFO(this->get_logger(), "Subscribing to target position topic: %s", topic.c_str());

    // create subscription; callback pushes target location to FollowMe
    target_pos_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      topic, 10,
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr p) {
        float lat{0.0f}, lon{0.0f}, alt{0.0f};

        lat = static_cast<float>(p->latitude);
        lon = static_cast<float>(p->longitude);
        alt = static_cast<float>(p->altitude);

        // Call into FollowMe outside of the mutex
        try {
          if (follow_me_) {
            follow_me_->set_target_location({lat, lon, alt, 0.0f, 0.0f, 0.0f});
          } else {
            // Use action->goto_location
            action_->goto_location(lat, lon, alt, 0.0f);
          }
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "Failed to set follow target: %s", e.what());
        }
      }
    );

    current_target_name_ = target;
    current_mission_ = MissionType::FollowMe;
  }

  void initialize_followme()
  {
    mavsdk::FollowMe::Config config;
    config.follow_height_m = 12.f;  // Minimum height
    config.follow_distance_m = 20.f;  // Follow distance
    config.responsiveness = 0.2f;  // Higher responsiveness

    follow_me_ = std::make_shared<mavsdk::FollowMe>(system_);
    mavsdk::FollowMe::Result config_result = follow_me_->set_config(config);
    if (config_result != mavsdk::FollowMe::Result::Success) {
      // handle config-setting failure (in this case print error)
      // std::cout << "Setting configuration failed:" << config_result << '\n';
      RCLCPP_ERROR(this->get_logger(), "Setting FollowMe configuration failed");
    }

  }

  void ensure_armed_and_takeoff()
  {
    if (!is_armed_) {
      RCLCPP_INFO(this->get_logger(), "Arming %s", vehicle_name_.c_str());
      auto arm_result = action_->arm();
      if (arm_result != mavsdk::Action::Result::Success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to arm %s: %s",
          vehicle_name_.c_str(), action_result_str(arm_result));
        return;
      }

    }

    if (vehicle_type_ == VehicleType::UAV && telemetry_->in_air() == false) {
      RCLCPP_INFO(this->get_logger(), "Taking off %s", vehicle_name_.c_str());
      auto takeoff_result = action_->takeoff();
      if (takeoff_result != mavsdk::Action::Result::Success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to takeoff %s: %s",
        vehicle_name_.c_str(), action_result_str(takeoff_result));
        return;
      }

      // Wait until flight mode is Hold
      while (telemetry_->in_air() == false || flight_mode_ != mavsdk::Telemetry::FlightMode::Hold) {
        RCLCPP_INFO(this->get_logger(), "Waiting for %s to takeoff...", vehicle_name_.c_str());
        std::this_thread::sleep_for(1s);
      }
    }
  }

  void stop_all_missions(const std_msgs::msg::Empty::SharedPtr /*msg*/ = nullptr)
  {
    RCLCPP_INFO(this->get_logger(), "Stopping all missions on %s", vehicle_name_.c_str());

    // Stop follow me if active (call outside lock to avoid holding mutex during plugin call)
    bool was_followme = (current_mission_ == MissionType::FollowMe);
    // clear mission state and unsubscribe under lock
    if (target_pos_sub_) {
      target_pos_sub_.reset();
      current_target_name_.clear();
    }
    current_mission_ = MissionType::None;

    if (follow_me_ && was_followme) {
      follow_me_->stop();
    }
  }

  void return_to_launch(const std_msgs::msg::Empty::SharedPtr /*msg*/ = nullptr)
  {
    std::lock_guard<std::mutex> lk(mission_mutex_);
    RCLCPP_INFO(this->get_logger(), "Returning %s to launch", vehicle_name_.c_str());

    // Stop all missions first
    stop_all_missions();

    // Call RTL
    auto rtl_result = action_->return_to_launch();
    if (rtl_result != mavsdk::Action::Result::Success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to command RTL for %s: %s",
        vehicle_name_.c_str(), action_result_str(rtl_result));
      return;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Vehicle>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
