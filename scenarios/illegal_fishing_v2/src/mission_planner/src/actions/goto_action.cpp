#include "mission_planner/actions/goto_action.hpp"
#include "mission_planner/vehicle.hpp"
#include "mission_planner/utils.hpp"
#include <thread>
#include <cmath>

namespace mission_planner
{

    GotoActionHandler::GotoActionHandler(Vehicle *vehicle)
        : BaseActionHandler(vehicle, std::chrono::milliseconds(1000))  // 1Hz timer
    {
    }

    void GotoActionHandler::stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Call base class stop (handles timer cleanup)
        BaseActionHandler::stop();
        
        if (vehicle_->action_)
        {
            vehicle_->action_->hold();
        }

        if (vehicle_->current_task_ == TaskType::Goto)
        {
            vehicle_->current_task_ = TaskType::None;
        }

        RCLCPP_INFO(vehicle_->get_logger(), "Stopped goto task");
    }

    rclcpp_action::GoalResponse GotoActionHandler::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const GotoAction::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(vehicle_->get_logger(), "Received goto goal: lat=%f, lon=%f, alt=%f",
                    goal->latitude_deg, goal->longitude_deg, goal->absolute_altitude_m);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse GotoActionHandler::handle_cancel(
        const std::shared_ptr<GoalHandleGoto> goal_handle)
    {
        RCLCPP_INFO(vehicle_->get_logger(), "Received request to cancel goto goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void GotoActionHandler::handle_accepted(const std::shared_ptr<GoalHandleGoto> goal_handle)
    {
        // No need for a separate thread - timer will handle periodic checks
        execute(goal_handle);
    }

    void GotoActionHandler::execute(const std::shared_ptr<GoalHandleGoto> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        feedback_ = std::make_shared<GotoAction::Feedback>();
        result_ = std::make_shared<GotoAction::Result>();

        RCLCPP_INFO(vehicle_->get_logger(), "Executing goto: lat=%f, lon=%f, alt=%f",
                    goal->latitude_deg, goal->longitude_deg, goal->absolute_altitude_m);

        {
            std::lock_guard<std::mutex> lk(mutex_);
            current_goal_ = goal_handle;
            current_goal_msg_ = std::make_shared<GotoAction::Goal>(*goal);
        }

        // Ensure armed and takeoff
        vehicle_->ensure_armed_and_takeoff();

        // Create a dummy mission item at current position (so actual goal is 2nd waypoint and respects speed)
        mavsdk::Mission::MissionItem dummy_item;
        dummy_item.latitude_deg = vehicle_->current_position_.latitude_deg;
        dummy_item.longitude_deg = vehicle_->current_position_.longitude_deg;
        dummy_item.relative_altitude_m = vehicle_->current_position_.relative_altitude_m;
        dummy_item.speed_m_s = goal->speed_m_s;
        dummy_item.is_fly_through = true;

        // Create the actual mission item for the goto location
        mavsdk::Mission::MissionItem mission_item;
        mission_item.latitude_deg = goal->latitude_deg;
        mission_item.longitude_deg = goal->longitude_deg;
        mission_item.relative_altitude_m = static_cast<float>(goal->absolute_altitude_m - 
                                            vehicle_->telemetry_->home().absolute_altitude_m);
        mission_item.speed_m_s = goal->speed_m_s;
        mission_item.is_fly_through = true;

        std::vector<mavsdk::Mission::MissionItem> mission_items = {dummy_item, mission_item};

        // Clear old mission before uploading new one
        auto clear_result = vehicle_->mission_->clear_mission();
        if (clear_result != mavsdk::Mission::Result::Success)
        {
            RCLCPP_WARN(vehicle_->get_logger(), "Failed to clear old mission");
        }

        // Upload the single-item mission
        mavsdk::Mission::MissionPlan mission_plan;
        mission_plan.mission_items = mission_items;

        auto upload_result = vehicle_->mission_->upload_mission(mission_plan);
        if (upload_result != mavsdk::Mission::Result::Success)
        {
            RCLCPP_ERROR(vehicle_->get_logger(), "Failed to upload goto mission");
            result_->success = false;
            result_->final_distance_to_target_m = 0.0f;
            goal_handle->abort(result_);

            std::lock_guard<std::mutex> lk(mutex_);
            current_goal_ = nullptr;
            return;
        }

        RCLCPP_INFO(vehicle_->get_logger(), "Goto mission uploaded, starting...");

        // Wait a moment for stability
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Start the mission
        auto start_result = vehicle_->mission_->start_mission();
        if (start_result != mavsdk::Mission::Result::Success)
        {
            RCLCPP_ERROR(vehicle_->get_logger(), "Failed to start goto mission");
            result_->success = false;
            result_->final_distance_to_target_m = 0.0f;
            goal_handle->abort(result_);

            std::lock_guard<std::mutex> lk(mutex_);
            current_goal_ = nullptr;
            return;
        }

        RCLCPP_INFO(vehicle_->get_logger(), "Goto mission started");
        vehicle_->current_task_ = TaskType::Goto;

        // Start timer for periodic feedback and progress checking
        start_timer();
    }

    void GotoActionHandler::timer_callback()
    {
        std::lock_guard<std::mutex> lk(mutex_);
        
        if (!current_goal_)
        {
            stop_timer();
            return;
        }

        // Check if mission is complete FIRST (before flight mode callback can interfere)
        auto progress = vehicle_->mission_->mission_progress();
        if (progress.current >= progress.total && progress.total > 0)
        {
            stop_timer();
            
            // Calculate final distance
            {
                std::lock_guard<std::mutex> position_lock(vehicle_->mission_mutex_);
                result_->final_distance_to_target_m = static_cast<float>(
                    mission_planner::utils::haversine_distance(
                        vehicle_->current_position_.latitude_deg, vehicle_->current_position_.longitude_deg,
                        current_goal_msg_->latitude_deg, current_goal_msg_->longitude_deg));
            }
            
            result_->success = true;
            current_goal_->succeed(result_);
            RCLCPP_INFO(vehicle_->get_logger(), "Goto goal succeeded! Final distance: %.2f m",
                        result_->final_distance_to_target_m);

            current_goal_ = nullptr;
            vehicle_->current_task_ = TaskType::None;
            return;
        }

        // Check if there's a cancel request
        if (current_goal_->is_canceling())
        {
            stop_timer();
            
            result_->success = false;
            result_->final_distance_to_target_m = feedback_->distance_to_target_m;
            current_goal_->canceled(result_);
            RCLCPP_INFO(vehicle_->get_logger(), "Goto goal canceled");
            current_goal_ = nullptr;
            vehicle_->current_task_ = TaskType::None;
            return;
        }

        // Calculate distance to target and get current speed
        {
            std::lock_guard<std::mutex> position_lock(vehicle_->mission_mutex_);

            feedback_->distance_to_target_m = static_cast<float>(
                mission_planner::utils::haversine_distance(
                    vehicle_->current_position_.latitude_deg, vehicle_->current_position_.longitude_deg,
                    current_goal_msg_->latitude_deg, current_goal_msg_->longitude_deg));

            // Get current speed from telemetry
            auto velocity = vehicle_->telemetry_->velocity_ned();
            feedback_->current_speed_m_s = std::sqrt(
                velocity.north_m_s * velocity.north_m_s +
                velocity.east_m_s * velocity.east_m_s);
        }

        // Publish feedback
        current_goal_->publish_feedback(feedback_);
    }

} // namespace mission_planner
