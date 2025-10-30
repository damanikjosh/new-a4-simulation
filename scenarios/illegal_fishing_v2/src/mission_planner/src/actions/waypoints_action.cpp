#include "mission_planner/actions/waypoints_action.hpp"
#include "mission_planner/vehicle.hpp"
#include "mission_planner/utils.hpp"
#include "mission_planner/trajectory_generator.hpp"
#include <thread>

namespace mission_planner
{

    WaypointsActionHandler::WaypointsActionHandler(Vehicle *vehicle)
        : BaseActionHandler(vehicle, std::chrono::milliseconds(1000))  // 1Hz timer
    {
        feedback_ = std::make_shared<WaypointsAction::Feedback>();
        // Initialize with simple trajectory generator (can be replaced with obstacle-aware version)
        trajectory_generator_ = std::make_shared<SimpleTrajectoryGenerator>();
    }

    bool WaypointsActionHandler::initialize_task(std::shared_ptr<const typename WaypointsAction::Goal> goal)
    {
        RCLCPP_INFO(vehicle_->get_logger(), "Executing waypoints mission with %zu items", goal->waypoints.size());

        // Store the goal for later use
        current_goal_msg_ = goal;

        float home_altitude = vehicle_->telemetry_->home().absolute_altitude_m;

        // Convert goal waypoints to GeoPoint format for trajectory generation
        std::vector<GeoPoint> original_waypoints;
        original_waypoints.reserve(goal->waypoints.size() + 1);
        
        // Add current position as first waypoint
        GeoPoint current_pos;
        current_pos.latitude_deg = vehicle_->current_position_.latitude_deg;
        current_pos.longitude_deg = vehicle_->current_position_.longitude_deg;
        original_waypoints.push_back(current_pos);
        
        // Add goal waypoints
        for (const auto &item : goal->waypoints)
        {
            GeoPoint wp;
            wp.latitude_deg = item.latitude_deg;
            wp.longitude_deg = item.longitude_deg;
            original_waypoints.push_back(wp);
        }

        // Generate trajectory with obstacle avoidance
        std::vector<TrajectoryPoint> trajectory = trajectory_generator_->generate_trajectory(original_waypoints);
        
        RCLCPP_INFO(vehicle_->get_logger(), "Generated trajectory with %zu points from %zu waypoints",
                    trajectory.size(), goal->waypoints.size());

        // Create a dummy mission item at current position (so first real waypoint is 2nd and respects speed)
        mavsdk::Mission::MissionItem dummy_item;
        dummy_item.latitude_deg = vehicle_->current_position_.latitude_deg;
        dummy_item.longitude_deg = vehicle_->current_position_.longitude_deg;
        dummy_item.relative_altitude_m = goal->waypoints[0].absolute_altitude_m - home_altitude;
        dummy_item.speed_m_s = goal->waypoints.empty() ? 5.0f : goal->waypoints[0].speed_m_s;
        dummy_item.is_fly_through = true;

        // Convert trajectory to MAVSDK mission items
        mission_items_.clear();
        mission_item_to_waypoint_index_.clear();
        
        mission_items_.push_back(dummy_item);
        mission_item_to_waypoint_index_.push_back(-1);  // Dummy item doesn't correspond to any waypoint

        for (const auto &traj_point : trajectory)
        {
            // Skip the first point as it's the current position (already handled by dummy)
            if (traj_point.original_waypoint_index == 0)
            {
                continue;
            }

            mavsdk::Mission::MissionItem mission_item;
            mission_item.latitude_deg = traj_point.point.latitude_deg;
            mission_item.longitude_deg = traj_point.point.longitude_deg;
            
            // Determine properties based on whether this is an actual waypoint or intermittent point
            int actual_waypoint_index = traj_point.original_waypoint_index - 1;  // Adjust for current position offset
            
            if (actual_waypoint_index >= 0 && actual_waypoint_index < static_cast<int>(goal->waypoints.size()))
            {
                // This is an actual waypoint from the goal
                const auto &goal_wp = goal->waypoints[actual_waypoint_index];
                mission_item.relative_altitude_m = goal_wp.absolute_altitude_m - home_altitude;
                mission_item.speed_m_s = goal_wp.speed_m_s;
                mission_item.is_fly_through = goal_wp.is_fly_through;
                mission_item.gimbal_pitch_deg = goal_wp.gimbal_pitch_deg;
                mission_item.gimbal_yaw_deg = goal_wp.gimbal_yaw_deg;
                mission_item.loiter_time_s = goal_wp.loiter_time_s;
            }
            else
            {
                // This is an intermittent point for obstacle avoidance
                // Use properties from the next actual waypoint or default values
                int next_waypoint_idx = actual_waypoint_index >= 0 ? actual_waypoint_index : 0;
                if (next_waypoint_idx >= static_cast<int>(goal->waypoints.size()))
                {
                    next_waypoint_idx = goal->waypoints.size() - 1;
                }
                
                const auto &ref_wp = goal->waypoints[next_waypoint_idx];
                mission_item.relative_altitude_m = ref_wp.absolute_altitude_m - home_altitude;
                mission_item.speed_m_s = ref_wp.speed_m_s;
                mission_item.is_fly_through = true;  // Always fly through intermittent points
                mission_item.gimbal_pitch_deg = ref_wp.gimbal_pitch_deg;
                mission_item.gimbal_yaw_deg = ref_wp.gimbal_yaw_deg;
                mission_item.loiter_time_s = 0.0f;  // Don't loiter at intermittent points
            }
            
            mission_items_.push_back(mission_item);
            mission_item_to_waypoint_index_.push_back(actual_waypoint_index);
        }

        feedback_->total_waypoints = static_cast<uint32_t>(goal->waypoints.size());  // Report actual count only

        // Clear old mission before uploading new one
        auto clear_result = vehicle_->mission_->clear_mission();
        if (clear_result != mavsdk::Mission::Result::Success)
        {
            RCLCPP_WARN(vehicle_->get_logger(), "Failed to clear old mission");
            return false;
        }

        // Upload new mission
        mavsdk::Mission::MissionPlan mission_plan;
        mission_plan.mission_items = mission_items_;

        auto upload_mission_result = vehicle_->mission_->upload_mission(mission_plan);
        if (upload_mission_result != mavsdk::Mission::Result::Success)
        {
            RCLCPP_ERROR(vehicle_->get_logger(), "Failed to upload mission");
            return false;
        }

        RCLCPP_INFO(vehicle_->get_logger(), "Mission uploaded successfully with %zu mission items for %zu waypoints",
                    mission_items_.size(), goal->waypoints.size());

        // Ensure armed before starting mission
        bool ready = vehicle_->ensure_armed_and_takeoff();
        if (!ready)
        {
            RCLCPP_ERROR(vehicle_->get_logger(), "Vehicle not ready for Waypoints mission");
            return false;
        }

        // Start the mission - this will automatically switch to Mission mode
        auto start_mission_result = vehicle_->mission_->start_mission();
        if (start_mission_result != mavsdk::Mission::Result::Success)
        {
            RCLCPP_ERROR(vehicle_->get_logger(), "Failed to start mission");
            return false;
        }

        RCLCPP_INFO(vehicle_->get_logger(), "Mission started successfully");
        return true;
    }

    void WaypointsActionHandler::stop_task()
    {
        if (vehicle_->mission_)
        {
            // Pause the mission
            auto pause_result = vehicle_->mission_->pause_mission();
            if (pause_result != mavsdk::Mission::Result::Success)
            {
                RCLCPP_WARN(vehicle_->get_logger(), "Failed to pause mission on %s",
                            vehicle_->vehicle_name_.c_str());
            }
 
            // Return to hold mode
            if (vehicle_->action_)
            {
                vehicle_->action_->hold();
            }

            // Clear the mission to prevent resuming
            auto clear_result = vehicle_->mission_->clear_mission();
            if (clear_result != mavsdk::Mission::Result::Success)
            {
                RCLCPP_WARN(vehicle_->get_logger(), "Failed to clear mission on %s",
                            vehicle_->vehicle_name_.c_str());
            }
        }

        RCLCPP_INFO(vehicle_->get_logger(), "Stopped waypoints task");
    }

    std::shared_ptr<WaypointsActionHandler::WaypointsAction::Result> WaypointsActionHandler::get_cancel_result()
    {
        std::shared_ptr<WaypointsAction::Result> result = std::make_shared<WaypointsAction::Result>();
        result->success = false;
        result->waypoints_completed = feedback_->current_waypoint_index;
        return result;
    }

    std::shared_ptr<WaypointsActionHandler::WaypointsAction::Result> WaypointsActionHandler::get_finish_result()
    {
        std::shared_ptr<WaypointsAction::Result> result = std::make_shared<WaypointsAction::Result>();
        result->success = true;
        result->waypoints_completed = feedback_->total_waypoints;
        return result;
    }

    std::shared_ptr<WaypointsActionHandler::WaypointsAction::Feedback> WaypointsActionHandler::get_feedback()
    {
        if (!current_goal_msg_)
        {
            return feedback_;
        }

        // Get mission progress
        auto progress = vehicle_->mission_->mission_progress();
        
        // Map the current mission item index to the actual waypoint index
        // We need to find the highest actual waypoint index that we've passed
        int current_waypoint_idx = 0;
        
        if (progress.current > 0 && progress.current <= static_cast<int>(mission_item_to_waypoint_index_.size()))
        {
            // Find the last actual waypoint we've reached or passed
            for (int i = 0; i < progress.current && i < static_cast<int>(mission_item_to_waypoint_index_.size()); ++i)
            {
                int wp_idx = mission_item_to_waypoint_index_[i];
                if (wp_idx >= 0)
                {
                    current_waypoint_idx = std::max(current_waypoint_idx, wp_idx + 1);  // +1 because we count completed waypoints
                }
            }
        }
        
        feedback_->current_waypoint_index = current_waypoint_idx;

        // Calculate distance to current waypoint
        // Find the next actual waypoint in the mission
        int next_actual_wp_mission_idx = -1;
        for (int i = progress.current; i < static_cast<int>(mission_item_to_waypoint_index_.size()); ++i)
        {
            if (mission_item_to_waypoint_index_[i] >= 0)
            {
                next_actual_wp_mission_idx = i;
                break;
            }
        }
        
        if (next_actual_wp_mission_idx >= 0 && next_actual_wp_mission_idx < static_cast<int>(mission_items_.size()))
        {
            const auto &target_wp = mission_items_[next_actual_wp_mission_idx];
            feedback_->distance_to_current_waypoint_m = static_cast<float>(
                mission_planner::utils::haversine_distance(
                    vehicle_->current_position_.latitude_deg, vehicle_->current_position_.longitude_deg,
                    target_wp.latitude_deg, target_wp.longitude_deg));
        }
        else
        {
            feedback_->distance_to_current_waypoint_m = 0.0f;
        }

        return feedback_;
    }

    bool WaypointsActionHandler::is_finished()
    {
        // Get mission progress and check completion
        auto progress = vehicle_->mission_->mission_progress();
        
        // Check if mission completed
        if (progress.current >= progress.total && progress.total > 0)
        {
            RCLCPP_INFO(vehicle_->get_logger(), "Waypoints mission completed: %d/%d waypoints",
                        progress.current, progress.total);
            return true;
        }

        return false;
    }

    bool WaypointsActionHandler::is_goal_valid(const std::shared_ptr<const typename WaypointsAction::Goal> goal)
    {
        if (goal->waypoints.empty())
        {
            RCLCPP_WARN(vehicle_->get_logger(), "Empty waypoints list, rejecting goal");
            return false;
        }
        return true;
    }

} // namespace mission_planner
