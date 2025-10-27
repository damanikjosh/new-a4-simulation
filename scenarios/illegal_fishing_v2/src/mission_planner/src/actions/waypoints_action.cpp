#include "mission_planner/actions/waypoints_action.hpp"
#include "mission_planner/vehicle.hpp"
#include "mission_planner/utils.hpp"
#include <thread>

namespace mission_planner
{

    WaypointsActionHandler::WaypointsActionHandler(Vehicle *vehicle)
        : BaseActionHandler(vehicle, std::chrono::milliseconds(1000))  // 1Hz timer
    {
    }

    void WaypointsActionHandler::stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Call base class stop (handles timer cleanup)
        BaseActionHandler::stop();

        {
            std::lock_guard<std::mutex> mission_lock(vehicle_->mission_mutex_);

            if (vehicle_->mission_ && vehicle_->current_task_ == TaskType::Waypoints)
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
        }

        if (vehicle_->current_task_ == TaskType::Waypoints)
        {
            vehicle_->current_task_ = TaskType::None;
        }

        RCLCPP_INFO(vehicle_->get_logger(), "Stopped waypoints task");
    }

    rclcpp_action::GoalResponse WaypointsActionHandler::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const WaypointsAction::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(vehicle_->get_logger(), "Received waypoints goal with %zu items",
                    goal->waypoints.size());

        if (goal->waypoints.empty())
        {
            RCLCPP_WARN(vehicle_->get_logger(), "Empty waypoints list, rejecting goal");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse WaypointsActionHandler::handle_cancel(
        const std::shared_ptr<GoalHandleWaypoints> goal_handle)
    {
        RCLCPP_INFO(vehicle_->get_logger(), "Received request to cancel waypoints goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void WaypointsActionHandler::handle_accepted(const std::shared_ptr<GoalHandleWaypoints> goal_handle)
    {
        // No need for separate thread - timer will handle periodic checks
        execute(goal_handle);
    }

    void WaypointsActionHandler::execute(const std::shared_ptr<GoalHandleWaypoints> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        feedback_ = std::make_shared<WaypointsAction::Feedback>();
        result_ = std::make_shared<WaypointsAction::Result>();

        RCLCPP_INFO(vehicle_->get_logger(), "Executing waypoints mission with %zu items",
                    goal->waypoints.size());

        {
            std::lock_guard<std::mutex> lk(mutex_);
            current_goal_ = goal_handle;
        }

        // Create a dummy mission item at current position (so first real waypoint is 2nd and respects speed)
        mavsdk::Mission::MissionItem dummy_item;
        dummy_item.latitude_deg = vehicle_->current_position_.latitude_deg;
        dummy_item.longitude_deg = vehicle_->current_position_.longitude_deg;
        dummy_item.relative_altitude_m = vehicle_->current_position_.relative_altitude_m;
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
            mission_item.relative_altitude_m = item.relative_altitude_m;
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
            RCLCPP_ERROR(vehicle_->get_logger(), "Failed to upload mission");
            result_->success = false;
            result_->waypoints_completed = 0;
            goal_handle->abort(result_);

            std::lock_guard<std::mutex> lk(mutex_);
            current_goal_ = nullptr;
            return;
        }

        RCLCPP_INFO(vehicle_->get_logger(), "Mission uploaded successfully, arming and starting mission");

        // Ensure armed before starting mission
        vehicle_->ensure_armed_and_takeoff();

        // Wait a moment for vehicle to stabilize in Hold mode
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Start the mission - this will automatically switch to Mission mode
        auto start_mission_result = vehicle_->mission_->start_mission();
        if (start_mission_result != mavsdk::Mission::Result::Success)
        {
            RCLCPP_ERROR(vehicle_->get_logger(), "Failed to start mission");
            result_->success = false;
            result_->waypoints_completed = 0;
            goal_handle->abort(result_);

            std::lock_guard<std::mutex> lk(mutex_);
            current_goal_ = nullptr;
            return;
        }

        RCLCPP_INFO(vehicle_->get_logger(), "Mission started successfully");
        vehicle_->current_task_ = TaskType::Waypoints;

        // Start timer for periodic feedback publishing
        start_timer();
    }

    void WaypointsActionHandler::timer_callback()
    {
        std::lock_guard<std::mutex> lk(mutex_);

        if (!current_goal_)
        {
            stop_timer();
            return;
        }

        // Get mission progress and check completion FIRST
        auto progress = vehicle_->mission_->mission_progress();
        
        // Check if mission completed
        if (progress.current >= progress.total && progress.total > 0)
        {
            stop_timer();
            RCLCPP_INFO(vehicle_->get_logger(), "Waypoints mission completed: %d/%d waypoints",
                        progress.current, progress.total);
            result_->success = true;
            result_->waypoints_completed = static_cast<uint32_t>(progress.current);
            current_goal_->succeed(result_);
            current_goal_ = nullptr;
            vehicle_->current_task_ = TaskType::None;
            return;
        }

        // Check if there's a cancel request
        if (current_goal_->is_canceling())
        {
            stop_timer();

            result_->success = false;
            result_->waypoints_completed = progress.current;
            current_goal_->canceled(result_);
            RCLCPP_INFO(vehicle_->get_logger(), "Waypoints goal canceled at waypoint %d/%d",
                        progress.current, progress.total);
            current_goal_ = nullptr;
            vehicle_->current_task_ = TaskType::None;
            return;
        }

        feedback_->current_waypoint_index = progress.current;

        // Calculate distance to current waypoint
        if (progress.current < static_cast<int>(mission_items_.size()))
        {
            const auto &current_wp = mission_items_[progress.current];
            std::lock_guard<std::mutex> position_lock(vehicle_->mission_mutex_);

            feedback_->distance_to_current_waypoint_m = static_cast<float>(
                mission_planner::utils::haversine_distance(
                    vehicle_->current_position_.latitude_deg, vehicle_->current_position_.longitude_deg,
                    current_wp.latitude_deg, current_wp.longitude_deg));
        }

        current_goal_->publish_feedback(feedback_);
    }

} // namespace mission_planner
