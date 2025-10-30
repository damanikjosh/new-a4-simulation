#ifndef MISSION_PLANNER__ACTIONS__ACTION_HPP_
#define MISSION_PLANNER__ACTIONS__ACTION_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mission_planner/vehicle.hpp"

namespace mission_planner
{
    template <typename ActionT>
    class BaseActionHandler
    {
    public:
        using ActionType = ActionT;
        using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
        using ActionServer = rclcpp_action::Server<ActionT>;

        explicit BaseActionHandler(Vehicle *vehicle, std::chrono::milliseconds timer_period = std::chrono::milliseconds(100))
            : vehicle_(vehicle), timer_period_(timer_period)
        {
        }

        virtual ~BaseActionHandler() = default;

        // Create and register the action server
        typename ActionServer::SharedPtr create_server(const std::string &action_name)
        {
            using namespace std::placeholders;

            action_server_ = rclcpp_action::create_server<ActionT>(
                vehicle_,
                action_name,
                std::bind(&BaseActionHandler::handle_goal, this, _1, _2),
                std::bind(&BaseActionHandler::handle_cancel, this, _1),
                std::bind(&BaseActionHandler::handle_accepted, this, _1));

            return action_server_;
        }

        // Check if action is currently active
        virtual bool is_active() const
        {
            return current_goal_ != nullptr;
        }

        // Stop the action - derived classes can override for custom cleanup
        void stop()
        {
            std::lock_guard<std::mutex> lock(mutex_);

            // Stop timer if running
            stop_timer();
            stop_task();

            if (current_goal_)
            {
                current_goal_.reset();
            }
        }

    protected:
        virtual bool initialize_task(std::shared_ptr<const typename ActionT::Goal> goal) = 0;
        virtual void stop_task() = 0;

        virtual std::shared_ptr<typename ActionT::Result> get_cancel_result() = 0;
        virtual std::shared_ptr<typename ActionT::Result> get_finish_result() = 0;
        virtual std::shared_ptr<typename ActionT::Feedback> get_feedback() = 0;

        virtual bool is_goal_valid(const std::shared_ptr<const typename ActionT::Goal> goal) { 
            (void)goal;
            return true;
        }
        virtual bool is_finished() = 0;

        // Handle goal request
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const typename ActionT::Goal> goal)
        {
            (void)uuid;
            RCLCPP_INFO(vehicle_->get_logger(), "Received goal");
            if (!is_goal_valid(goal))
            {
                RCLCPP_WARN(vehicle_->get_logger(), "Goal is not valid, rejecting");
                return rclcpp_action::GoalResponse::REJECT;
            }
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        // Handle cancel request
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
        {
            RCLCPP_INFO(vehicle_->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // Handle accepted goal
        void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
        {
            bool success = initialize_task(goal_handle->get_goal());
            if (!success)
            {
                RCLCPP_ERROR(vehicle_->get_logger(), "Failed to initialize task, aborting goal");
                std::shared_ptr<typename ActionT::Result> result = get_cancel_result();
                goal_handle->abort(result);
                return;
            }
            current_goal_ = goal_handle;
            start_timer();
        }

        // Timer callback for periodic updates
        void timer_callback()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!current_goal_)
            {
                stop_timer();
                return;
            }

            if (is_finished())
            {
                stop_timer();
                std::shared_ptr<typename ActionT::Result> result = get_finish_result();
                current_goal_->succeed(result);
                RCLCPP_INFO(vehicle_->get_logger(), "Task completed successfully");
                current_goal_ = nullptr;
                return;
            }

            if (current_goal_->is_canceling())
            {
                stop_timer();
                stop_task();
                std::shared_ptr<typename ActionT::Result> result = get_cancel_result();
                current_goal_->canceled(result);
                RCLCPP_INFO(vehicle_->get_logger(), "Task canceled");
                current_goal_ = nullptr;
                return;
            }

            std::shared_ptr<typename ActionT::Feedback> feedback = get_feedback();
            current_goal_->publish_feedback(feedback);
        }

        // Start the periodic timer
        void start_timer()
        {
            timer_ = vehicle_->create_wall_timer(
                timer_period_,
                [this]()
                { this->timer_callback(); });
        }

        // Stop the periodic timer
        void stop_timer()
        {
            if (timer_)
            {
                timer_->cancel();
                timer_.reset();
            }
        }

        // Protected members accessible to derived classes
        Vehicle *vehicle_;
        std::shared_ptr<GoalHandle> current_goal_;
        mutable std::mutex mutex_;
        typename ActionServer::SharedPtr action_server_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::chrono::milliseconds timer_period_;
    };

} // namespace mission_planner

#endif // MISSION_PLANNER__ACTIONS__ACTION_HPP_
