#include "mission_planner/actions/followme_action.hpp"
#include "mission_planner/vehicle.hpp"
#include "mission_planner/utils.hpp"
#include <thread>

namespace mission_planner
{

    FollowMeActionHandler::FollowMeActionHandler(Vehicle *vehicle)
        : BaseActionHandler(vehicle, std::chrono::milliseconds(1000))  // 1Hz timer
    {
        feedback_ = std::make_shared<FollowMeAction::Feedback>();
    }

    bool FollowMeActionHandler::initialize_task(std::shared_ptr<const typename FollowMeAction::Goal> goal)
    {
        const std::string target = goal->vehicle_name;
        
        RCLCPP_INFO(vehicle_->get_logger(), "Executing followme for target: %s", target.c_str());

        // Unsubscribe previous target if present
        if (target_pos_sub_)
        {
            target_pos_sub_.reset();
            current_target_name_.clear();
        }

        current_target_name_ = target;

        // Ensure armed and start FollowMe
        bool ready = vehicle_->ensure_armed_and_takeoff();
        if (!ready)
        {
            RCLCPP_ERROR(vehicle_->get_logger(), "Vehicle not ready for FollowMe mission");
            return false;
        }
        
        if (vehicle_->follow_me_)
        {
            auto start_result = vehicle_->follow_me_->start();
            if (start_result != mavsdk::FollowMe::Result::Success)
            {
                RCLCPP_ERROR(vehicle_->get_logger(), "Failed to start FollowMe mode");
                return false;
            }
        }

        // Subscribe to target position
        std::string topic = target + "/position";
        RCLCPP_INFO(vehicle_->get_logger(), "Subscribing to target position topic: %s", topic.c_str());

        target_pos_sub_ = vehicle_->create_subscription<mission_planner_msgs::msg::Position>(
            topic, 10,
            [this](const mission_planner_msgs::msg::Position::SharedPtr p)
            {
                target_position_ = *p;

                float lat = static_cast<float>(p->latitude_deg);
                float lon = static_cast<float>(p->longitude_deg);
                float alt = static_cast<float>(p->absolute_altitude_m);

                try
                {
                    if (vehicle_->follow_me_)
                    {
                        vehicle_->follow_me_->set_target_location({lat, lon, alt, 0.0f, 0.0f, 0.0f});
                    }
                    else
                    {
                        vehicle_->action_->goto_location(lat, lon, alt, 0.0f);
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(vehicle_->get_logger(), "Failed to set follow target: %s", e.what());
                }
            });

        RCLCPP_INFO(vehicle_->get_logger(), "FollowMe mode started successfully");
        return true;
    }

    void FollowMeActionHandler::stop_task()
    {
        if (vehicle_->follow_me_)
        {
            vehicle_->follow_me_->stop();
        }

        if (target_pos_sub_)
        {
            target_pos_sub_.reset();
            current_target_name_.clear();
        }

        RCLCPP_INFO(vehicle_->get_logger(), "Stopped followme task");
    }

    std::shared_ptr<FollowMeActionHandler::FollowMeAction::Result> FollowMeActionHandler::get_cancel_result()
    {
        std::shared_ptr<FollowMeAction::Result> result = std::make_shared<FollowMeAction::Result>();
        result->distance_to_target_m = feedback_->distance_to_target_m;
        return result;
    }

    std::shared_ptr<FollowMeActionHandler::FollowMeAction::Result> FollowMeActionHandler::get_finish_result()
    {
        // FollowMe doesn't naturally finish, so this shouldn't be called
        std::shared_ptr<FollowMeAction::Result> result = std::make_shared<FollowMeAction::Result>();
        result->distance_to_target_m = feedback_->distance_to_target_m;
        return result;
    }

    std::shared_ptr<FollowMeActionHandler::FollowMeAction::Feedback> FollowMeActionHandler::get_feedback()
    {
        // Calculate distance to target
        feedback_->distance_to_target_m = static_cast<float>(
            mission_planner::utils::haversine_distance(
                vehicle_->current_position_.latitude_deg, vehicle_->current_position_.longitude_deg,
                target_position_.latitude_deg, target_position_.longitude_deg));

        return feedback_;
    }

    bool FollowMeActionHandler::is_finished()
    {
        // FollowMe never finishes on its own - it must be canceled
        return false;
    }

    bool FollowMeActionHandler::is_goal_valid(const std::shared_ptr<const typename FollowMeAction::Goal> goal)
    {
        if (goal->vehicle_name.empty())
        {
            RCLCPP_WARN(vehicle_->get_logger(), "Empty target name, rejecting goal");
            return false;
        }
        return true;
    }

} // namespace mission_planner
