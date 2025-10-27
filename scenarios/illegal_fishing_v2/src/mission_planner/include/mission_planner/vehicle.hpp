#ifndef MISSION_PLANNER__VEHICLE_HPP_
#define MISSION_PLANNER__VEHICLE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <optional>
#include <vector>
#include <map>
#include <mutex>
#include <atomic>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/follow_me/follow_me.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>

#include "rclcpp/rclcpp.hpp"
#include "mission_planner/vehicle_type.hpp"
#include "mission_planner/task_type.hpp"

#include "mission_planner_msgs/msg/heartbeat.hpp"
#include "mission_planner_msgs/msg/position.hpp"
#include "std_msgs/msg/empty.hpp"

#include "mission_planner/actions/goto_action.hpp"
#include "mission_planner/actions/followme_action.hpp"
#include "mission_planner/actions/waypoints_action.hpp"
#include "mission_planner/actions/rtl_action.hpp"

namespace mission_planner
{

    class Vehicle : public rclcpp::Node
    {
    public:
        Vehicle();

        // Public accessors for action handlers
        std::string vehicle_name_;
        VehicleType vehicle_type_;

        std::mutex mission_mutex_;
        mission_planner_msgs::msg::Position current_position_;
        std::atomic<TaskType> current_task_{TaskType::None};

        // MAVSDK plugins (public for action handlers)
        std::shared_ptr<mavsdk::Telemetry> telemetry_;
        std::shared_ptr<mavsdk::Action> action_;
        std::shared_ptr<mavsdk::Mission> mission_;
        std::shared_ptr<mavsdk::FollowMe> follow_me_;

        // Utility functions for action handlers
        void ensure_armed_and_takeoff();
        void stop_all_tasks(const std_msgs::msg::Empty::SharedPtr msg = nullptr);

    private:
        // Configuration
        int port_;

        // Publishers
        rclcpp::Publisher<mission_planner_msgs::msg::Heartbeat>::SharedPtr heartbeat_pub_;
        rclcpp::Publisher<mission_planner_msgs::msg::Position>::SharedPtr position_pub_;

        // Action handlers
        std::unique_ptr<GotoActionHandler> goto_action_handler_;
        std::unique_ptr<FollowMeActionHandler> followme_action_handler_;
        std::unique_ptr<WaypointsActionHandler> waypoints_action_handler_;
        std::unique_ptr<RtlActionHandler> rtl_action_handler_;

        // Subscribers
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr task_stop_sub_;

        // State
        bool is_armed_ = false;
        std::atomic<mavsdk::Telemetry::FlightMode> flight_mode_;

        // Timers
        rclcpp::TimerBase::SharedPtr timer_;

        // MAVSDK objects
        mavsdk::Mavsdk mavsdk_{mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}};
        std::shared_ptr<mavsdk::System> system_;

        // Callbacks
        void arm_callback(bool is_armed);
        void position_callback(mavsdk::Telemetry::Position position);
        void heading_callback(mavsdk::Telemetry::Heading heading);
        void flight_mode_callback(mavsdk::Telemetry::FlightMode flight_mode);
        void publish_heartbeat();
    };

} // namespace mission_planner

#endif // MISSION_PLANNER__VEHICLE_HPP_
