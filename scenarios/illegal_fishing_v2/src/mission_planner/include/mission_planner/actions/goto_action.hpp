#ifndef MISSION_PLANNER__ACTIONS__GOTO_ACTION_HPP_
#define MISSION_PLANNER__ACTIONS__GOTO_ACTION_HPP_

#include "mission_planner/actions/action.hpp"
#include "mission_planner_msgs/action/goto.hpp"

namespace mission_planner
{

    class GotoActionHandler : public BaseActionHandler<mission_planner_msgs::action::Goto>
    {
    public:
        using GotoAction = mission_planner_msgs::action::Goto;
        using GoalHandleGoto = rclcpp_action::ServerGoalHandle<GotoAction>;

        explicit GotoActionHandler(Vehicle *vehicle);

    protected:
        bool initialize_task(std::shared_ptr<const typename GotoAction::Goal> goal) override;
        void stop_task() override;
        std::shared_ptr<GotoAction::Result> get_cancel_result() override;
        std::shared_ptr<GotoAction::Result> get_finish_result() override;
        std::shared_ptr<GotoAction::Feedback> get_feedback() override;
        bool is_finished() override;


    private:
        std::shared_ptr<GotoAction::Feedback> feedback_;
    };

} // namespace mission_planner

#endif // MISSION_PLANNER__ACTIONS__GOTO_ACTION_HPP_
