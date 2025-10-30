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
        feedback_ = std::make_shared<ReturnToLaunchAction::Feedback>();
    }

    bool RtlActionHandler::initialize_task(std::shared_ptr<const typename ReturnToLaunchAction::Goal> goal)
    {
        (void)goal;
        
        RCLCPP_INFO(vehicle_->get_logger(), "Executing return to launch for %s", vehicle_->vehicle_name_.c_str());

        // Get home position for distance calculation
        home_position_ = vehicle_->telemetry_->home();

        // Call RTL
        auto rtl_result = vehicle_->action_->return_to_launch();
        if (rtl_result != mavsdk::Action::Result::Success)
        {
            RCLCPP_ERROR(vehicle_->get_logger(), "Failed to command RTL for %s",
                         vehicle_->vehicle_name_.c_str());
            return false;
        }

        RCLCPP_INFO(vehicle_->get_logger(), "RTL command sent successfully");
        return true;
    }

    void RtlActionHandler::stop_task()
    {
        if (vehicle_->action_)
        {
            vehicle_->action_->hold();
        }
        RCLCPP_INFO(vehicle_->get_logger(), "Stopped RTL task");
    }

    std::shared_ptr<RtlActionHandler::ReturnToLaunchAction::Result> RtlActionHandler::get_cancel_result()
    {
        std::shared_ptr<ReturnToLaunchAction::Result> result = std::make_shared<ReturnToLaunchAction::Result>();
        result->success = false;
        result->final_distance_to_home_m = feedback_->distance_to_home_m;
        return result;
    }

    std::shared_ptr<RtlActionHandler::ReturnToLaunchAction::Result> RtlActionHandler::get_finish_result()
    {
        std::shared_ptr<ReturnToLaunchAction::Result> result = std::make_shared<ReturnToLaunchAction::Result>();
        result->success = true;
        result->final_distance_to_home_m = feedback_->distance_to_home_m;
        return result;
    }

    std::shared_ptr<RtlActionHandler::ReturnToLaunchAction::Feedback> RtlActionHandler::get_feedback()
    {
        // Calculate distance to home and current altitude
        feedback_->distance_to_home_m = static_cast<float>(
            mission_planner::utils::haversine_distance(
                vehicle_->current_position_.latitude_deg, vehicle_->current_position_.longitude_deg,
                home_position_.latitude_deg, home_position_.longitude_deg));

        feedback_->current_altitude_m = vehicle_->current_position_.relative_altitude_m;

        return feedback_;
    }

    bool RtlActionHandler::is_finished()
    {
        // Check if landed (vehicle-agnostic completion criteria)
        return !vehicle_->telemetry_->in_air();
    }

} // namespace mission_planner
