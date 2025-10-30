#include "mission_planner/actions/search_action.hpp"
#include "mission_planner/vehicle.hpp"
#include "mission_planner/utils.hpp"
#include <thread>

namespace mission_planner
{

    SearchActionHandler::SearchActionHandler(Vehicle *vehicle)
        : BaseActionHandler(vehicle, std::chrono::milliseconds(1000)),  // 1Hz timer
          search_radius_m_(0.0f),
          total_waypoints_(0),
          current_waypoint_index_(0)
    {
        feedback_ = std::make_shared<SearchAction::Feedback>();
        result_ = std::make_shared<SearchAction::Result>();
    }

    bool SearchActionHandler::is_goal_valid(std::shared_ptr<const SearchAction::Goal> goal)
    {
        if (goal->waypoints.empty())
        {
            RCLCPP_WARN(vehicle_->get_logger(), "Empty waypoints list, rejecting goal");
            return false;
        }
        return true;
    }

    bool SearchActionHandler::initialize_task(std::shared_ptr<const SearchAction::Goal> goal)
    {
        RCLCPP_INFO(vehicle_->get_logger(), "Initializing search mission with %zu items and %zu target vehicles",
                    goal->waypoints.size(), goal->vehicle_names.size());

        // Store search parameters
        search_radius_m_ = goal->search_radius_m;
        vehicle_names_ = goal->vehicle_names;
        total_waypoints_ = static_cast<int>(goal->waypoints.size());
        current_waypoint_index_ = 0;
        
        // Clear previous subscriptions
        target_pos_subs_.clear();
        target_positions_.clear();
        
        // Subscribe to position topics for each target vehicle
        for (const auto& vehicle_name : goal->vehicle_names)
        {
            std::string topic = "/" + vehicle_name + "/position";
            RCLCPP_INFO(vehicle_->get_logger(), "Subscribing to %s for search monitoring", topic.c_str());
            
            auto sub = vehicle_->create_subscription<mission_planner_msgs::msg::Position>(
                topic, 10,
                [this, vehicle_name](const mission_planner_msgs::msg::Position::SharedPtr msg)
                {
                    target_positions_[vehicle_name] = *msg;
                });
            
            target_pos_subs_.push_back(sub);
        }

        // Get home position for altitude conversion
        auto home = vehicle_->telemetry_->home();
        float home_altitude = home.absolute_altitude_m;

        // Create a dummy mission item at current position (so first real waypoint is 2nd and respects speed)
        mavsdk::Mission::MissionItem dummy_item;
        dummy_item.latitude_deg = vehicle_->current_position_.latitude_deg;
        dummy_item.longitude_deg = vehicle_->current_position_.longitude_deg;
        
        // Convert first waypoint's absolute altitude to relative
        float first_relative_alt = goal->waypoints[0].absolute_altitude_m - home_altitude;
        dummy_item.relative_altitude_m = first_relative_alt;
        dummy_item.speed_m_s = goal->waypoints.empty() ? 5.0f : goal->waypoints[0].speed_m_s;
        dummy_item.is_fly_through = true;

        // Convert to MAVSDK mission items
        mission_items_.clear();
        mission_items_.push_back(dummy_item);  // Add dummy first

        for (const auto &item : goal->waypoints)
        {
            mavsdk::Mission::MissionItem mission_item;
            mission_item.latitude_deg = item.latitude_deg;
            mission_item.longitude_deg = item.longitude_deg;
            
            // Convert absolute altitude to relative
            mission_item.relative_altitude_m = item.absolute_altitude_m - home_altitude;
            
            mission_item.speed_m_s = item.speed_m_s;
            mission_item.is_fly_through = item.is_fly_through;
            mission_item.gimbal_pitch_deg = item.gimbal_pitch_deg;
            mission_item.gimbal_yaw_deg = item.gimbal_yaw_deg;
            mission_item.loiter_time_s = item.loiter_time_s;
            mission_items_.push_back(mission_item);
        }

        feedback_->total_waypoints = static_cast<uint32_t>(goal->waypoints.size());  // Report actual count, not including dummy

        // Clear old mission before uploading new one
        auto clear_result = vehicle_->mission_->clear_mission();
        if (clear_result != mavsdk::Mission::Result::Success)
        {
            RCLCPP_WARN(vehicle_->get_logger(), "Failed to clear old mission");
        }

        // Upload new mission
        mavsdk::Mission::MissionPlan mission_plan;
        mission_plan.mission_items = mission_items_;

        auto upload_mission_result = vehicle_->mission_->upload_mission(mission_plan);
        if (upload_mission_result != mavsdk::Mission::Result::Success)
        {
            RCLCPP_ERROR(vehicle_->get_logger(), "Failed to upload search mission");
            result_->success = false;
            result_->waypoints_completed = 0;
            return false;
        }

        RCLCPP_INFO(vehicle_->get_logger(), "Search mission uploaded successfully, arming and starting mission");

        // Ensure armed before starting mission
        bool ready = vehicle_->ensure_armed_and_takeoff();
        if (!ready)
        {
            RCLCPP_ERROR(vehicle_->get_logger(), "Vehicle not ready for Search mission");
            return false;
        }

        // Start the mission - this will automatically switch to Mission mode
        auto start_mission_result = vehicle_->mission_->start_mission();
        if (start_mission_result != mavsdk::Mission::Result::Success)
        {
            RCLCPP_ERROR(vehicle_->get_logger(), "Failed to start search mission");
            result_->success = false;
            result_->waypoints_completed = 0;
            return false;
        }

        RCLCPP_INFO(vehicle_->get_logger(), "Search mission started successfully");
        vehicle_->current_task_ = TaskType::Search;

        return true;
    }

    void SearchActionHandler::stop_task()
    {
        RCLCPP_INFO(vehicle_->get_logger(), "Stopping search task");

        // Unsubscribe from all target position topics
        target_pos_subs_.clear();
        target_positions_.clear();
        vehicle_names_.clear();

        if (vehicle_->mission_ && vehicle_->current_task_ == TaskType::Search)
        {
            // Pause the mission
            auto pause_result = vehicle_->mission_->pause_mission();
            if (pause_result != mavsdk::Mission::Result::Success)
            {
                RCLCPP_WARN(vehicle_->get_logger(), "Failed to pause mission on %s",
                            vehicle_->vehicle_name_.c_str());
            }

            // Return to hold mode
            vehicle_->action_->hold();
        }

        if (vehicle_->current_task_ == TaskType::Search)
        {
            vehicle_->current_task_ = TaskType::None;
        }
    }

    std::shared_ptr<SearchActionHandler::SearchAction::Result> SearchActionHandler::get_cancel_result()
    {
        auto progress = vehicle_->mission_->mission_progress();
        result_->success = false;
        result_->waypoints_completed = static_cast<uint32_t>(progress.current);
        return result_;
    }

    std::shared_ptr<SearchActionHandler::SearchAction::Result> SearchActionHandler::get_finish_result()
    {
        auto progress = vehicle_->mission_->mission_progress();
        result_->success = true;
        result_->waypoints_completed = static_cast<uint32_t>(progress.current);
        return result_;
    }

    std::shared_ptr<SearchActionHandler::SearchAction::Feedback> SearchActionHandler::get_feedback()
    {
        auto progress = vehicle_->mission_->mission_progress();
        
        feedback_->current_waypoint_index = std::max(0, progress.current - 1);

        // Calculate distance to current waypoint
        if (progress.current < static_cast<int>(mission_items_.size()))
        {
            const auto &current_wp = mission_items_[progress.current];

            feedback_->distance_to_current_waypoint_m = static_cast<float>(
                mission_planner::utils::haversine_distance(
                    vehicle_->current_position_.latitude_deg, vehicle_->current_position_.longitude_deg,
                    current_wp.latitude_deg, current_wp.longitude_deg));
        }

        // Check which target vehicles are within search radius
        feedback_->vehicles_in_range.clear();
        
        for (const auto& [vehicle_name, target_pos] : target_positions_)
        {
            double distance_m = mission_planner::utils::haversine_distance(
                vehicle_->current_position_.latitude_deg, 
                vehicle_->current_position_.longitude_deg,
                target_pos.latitude_deg, 
                target_pos.longitude_deg);
            
            if (distance_m <= search_radius_m_)
            {
                feedback_->vehicles_in_range.push_back(vehicle_name);
                RCLCPP_INFO(vehicle_->get_logger(), 
                            "Vehicle %s detected within search radius (%.2f m)", 
                            vehicle_name.c_str(), distance_m);
            }
        }

        return feedback_;
    }

    bool SearchActionHandler::is_finished()
    {
        auto progress = vehicle_->mission_->mission_progress();
        
        // Mission is finished when we've reached all waypoints
        if (progress.current >= progress.total && progress.total > 0)
        {
            RCLCPP_INFO(vehicle_->get_logger(), "Search mission completed: %d/%d waypoints",
                        progress.current, progress.total);
            return true;
        }
        
        return false;
    }

} // namespace mission_planner
