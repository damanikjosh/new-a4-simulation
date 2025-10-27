#include "mission_planner/actions/followme_action.hpp"
#include "mission_planner/vehicle.hpp"
#include "mission_planner/utils.hpp"
#include <thread>

namespace mission_planner
{

    FollowMeActionHandler::FollowMeActionHandler(Vehicle *vehicle)
        : BaseActionHandler(vehicle, std::chrono::milliseconds(1000))  // 1Hz timer
    {
    }

    void FollowMeActionHandler::stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Call base class stop (handles timer cleanup)
        BaseActionHandler::stop();

        if (vehicle_->follow_me_ && vehicle_->current_task_ == TaskType::FollowMe)
        {
            vehicle_->follow_me_->stop();
        }

        if (target_pos_sub_)
        {
            target_pos_sub_.reset();
            current_target_name_.clear();
        }

        if (vehicle_->current_task_ == TaskType::FollowMe)
        {
            vehicle_->current_task_ = TaskType::None;
        }

        RCLCPP_INFO(vehicle_->get_logger(), "Stopped followme task");
    }

    rclcpp_action::GoalResponse FollowMeActionHandler::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const FollowMeAction::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(vehicle_->get_logger(), "Received followme goal for target: %s",
                    goal->vehicle_name.c_str());

        if (goal->vehicle_name.empty())
        {
            RCLCPP_WARN(vehicle_->get_logger(), "Empty target name, rejecting goal");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse FollowMeActionHandler::handle_cancel(
        const std::shared_ptr<GoalHandleFollowMe> goal_handle)
    {
        RCLCPP_INFO(vehicle_->get_logger(), "Received request to cancel followme goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void FollowMeActionHandler::handle_accepted(const std::shared_ptr<GoalHandleFollowMe> goal_handle)
    {
        // No need for separate thread - timer will handle periodic checks
        execute(goal_handle);
    }

    void FollowMeActionHandler::execute(const std::shared_ptr<GoalHandleFollowMe> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        const std::string target = goal->vehicle_name;
        feedback_ = std::make_shared<FollowMeAction::Feedback>();
        result_ = std::make_shared<FollowMeAction::Result>();

        RCLCPP_INFO(vehicle_->get_logger(), "Executing followme for target: %s", target.c_str());

        {
            std::lock_guard<std::mutex> lk(mutex_);
            current_goal_ = goal_handle;
        }

        {
            std::lock_guard<std::mutex> lk(vehicle_->mission_mutex_);

            // Unsubscribe previous target if present
            if (target_pos_sub_)
            {
                target_pos_sub_.reset();
                current_target_name_.clear();
            }

            current_target_name_ = target;
        }

        // Ensure armed and start FollowMe
        vehicle_->ensure_armed_and_takeoff();
        if (vehicle_->follow_me_)
        {
            vehicle_->follow_me_->start();
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

        vehicle_->current_task_ = TaskType::FollowMe;

        // Start timer for periodic feedback publishing
        start_timer();
    }

    void FollowMeActionHandler::timer_callback()
    {
        std::lock_guard<std::mutex> lk(mutex_);
        
        if (!current_goal_)
        {
            stop_timer();
            return;
        }

        // Check if there's a cancel request
        if (current_goal_->is_canceling())
        {
            stop_timer();
            result_->distance_to_target_m = 0.0f;
            current_goal_->canceled(result_);
            RCLCPP_INFO(vehicle_->get_logger(), "FollowMe goal canceled");
            current_goal_ = nullptr;
            vehicle_->current_task_ = TaskType::None;
            return;
        }

        // Calculate distance to target and publish feedback
        {
            std::lock_guard<std::mutex> position_lock(vehicle_->mission_mutex_);

            // Use haversine distance for accurate GPS distance calculation
            feedback_->distance_to_target_m = static_cast<float>(
                mission_planner::utils::haversine_distance(
                    vehicle_->current_position_.latitude_deg, vehicle_->current_position_.longitude_deg,
                    target_position_.latitude_deg, target_position_.longitude_deg));
        }

        current_goal_->publish_feedback(feedback_);
    }

} // namespace mission_planner
