#ifndef MISSION_PLANNER__MISSION_TYPE_HPP__
#define MISSION_PLANNER__MISSION_TYPE_HPP__

enum class TaskType
{
    None = 0,
    Goto = 1,
    FollowMe = 2,
    Waypoints = 3,
    Search = 4,
    Return = 99
};

#endif // MISSION_PLANNER__MISSION_TYPE_HPP__