#include "mission_planner/actions/rtl_action.hpp"
#include "mission_planner/vehicle.hpp"
#include "mission_planner/utils.hpp"
#include <thread>
#include <cmath>

namespace mission_planner
{

    RtlActionHandler::RtlActionHandler(Vehicle *vehicle)
        : BaseActionHandler(vehicle, std::chrono::milliseconds(1000))  // 1Hz timer
    {
    }

    void RtlActionHandler::stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Call base class stop (handles timer cleanup)
        BaseActionHandler::stop();

        if (vehicle_->current_task_ == TaskType::Return)
        {
            vehicle_->action_->hold();
        }

        if (vehicle_->current_task_ == TaskType::Return)
        {
            vehicle_->current_task_ = TaskType::None;
        }

        RCLCPP_INFO(vehicle_->get_logger(), "Stopped RTL task");
    }

    rclcpp_action::GoalResponse RtlActionHandler::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ReturnToLaunchAction::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(vehicle_->get_logger(), "Received return to launch goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse RtlActionHandler::handle_cancel(
        const std::shared_ptr<GoalHandleReturnToLaunch> goal_handle)
    {
        RCLCPP_INFO(vehicle_->get_logger(), "Received request to cancel return to launch goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void RtlActionHandler::handle_accepted(const std::shared_ptr<GoalHandleReturnToLaunch> goal_handle)
    {
        // No need for separate thread - timer will handle periodic checks
        execute(goal_handle);
    }

    void RtlActionHandler::execute(const std::shared_ptr<GoalHandleReturnToLaunch> goal_handle)
    {
        feedback_ = std::make_shared<ReturnToLaunchAction::Feedback>();
        result_ = std::make_shared<ReturnToLaunchAction::Result>();

        RCLCPP_INFO(vehicle_->get_logger(), "Executing return to launch for %s", vehicle_->vehicle_name_.c_str());

        {
            std::lock_guard<std::mutex> lk(mutex_);
            current_goal_ = goal_handle;
        }

        // Get home position for distance calculation
        home_position_ = vehicle_->telemetry_->home();

        // Call RTL
        auto rtl_result = vehicle_->action_->return_to_launch();
        if (rtl_result != mavsdk::Action::Result::Success)
        {
            RCLCPP_ERROR(vehicle_->get_logger(), "Failed to command RTL for %s",
                         vehicle_->vehicle_name_.c_str());
            result_->success = false;
            result_->final_distance_to_home_m = 0.0f;
            goal_handle->abort(result_);

            std::lock_guard<std::mutex> lk(mutex_);
            current_goal_.reset();
            return;
        }

        vehicle_->current_task_ = TaskType::Return;

        // Start timer for periodic feedback publishing
        start_timer();
    }

    void RtlActionHandler::timer_callback()
    {
        std::lock_guard<std::mutex> lk(mutex_);

        if (!current_goal_)
        {
            stop_timer();
            return;
        }

        // Check if landed (vehicle-agnostic completion criteria) FIRST
        if (!vehicle_->telemetry_->in_air())
        {
            stop_timer();
            
            // Calculate final distance to home
            {
                std::lock_guard<std::mutex> position_lock(vehicle_->mission_mutex_);
                result_->final_distance_to_home_m = static_cast<float>(
                    mission_planner::utils::haversine_distance(
                        vehicle_->current_position_.latitude_deg, vehicle_->current_position_.longitude_deg,
                        home_position_.latitude_deg, home_position_.longitude_deg));
            }
            
            result_->success = true;
            current_goal_->succeed(result_);
            RCLCPP_INFO(vehicle_->get_logger(), "Returned to launch and landed (%.2f m from home)",
                        result_->final_distance_to_home_m);
            current_goal_ = nullptr;
            vehicle_->current_task_ = TaskType::None;
            return;
        }

        // Check if there's a cancel request
        if (current_goal_->is_canceling())
        {
            stop_timer();
            result_->success = false;
            result_->final_distance_to_home_m = feedback_->distance_to_home_m;
            current_goal_->canceled(result_);
            RCLCPP_INFO(vehicle_->get_logger(), "Return to launch goal canceled");
            current_goal_ = nullptr;
            vehicle_->current_task_ = TaskType::None;
            return;
        }

        // Calculate distance to home and current altitude
        {
            std::lock_guard<std::mutex> position_lock(vehicle_->mission_mutex_);

            feedback_->distance_to_home_m = static_cast<float>(
                mission_planner::utils::haversine_distance(
                    vehicle_->current_position_.latitude_deg, vehicle_->current_position_.longitude_deg,
                    home_position_.latitude_deg, home_position_.longitude_deg));

            feedback_->current_altitude_m = vehicle_->current_position_.relative_altitude_m;
        }

        current_goal_->publish_feedback(feedback_);
    }

} // namespace mission_planner
