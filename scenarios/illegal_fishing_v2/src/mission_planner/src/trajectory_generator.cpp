#include "mission_planner/trajectory_generator.hpp"

namespace mission_planner
{

std::vector<TrajectoryPoint> SimpleTrajectoryGenerator::generate_trajectory(const std::vector<GeoPoint>& waypoints)
{
    std::vector<TrajectoryPoint> trajectory;
    trajectory.reserve(waypoints.size());
    
    for (size_t i = 0; i < waypoints.size(); ++i)
    {
        TrajectoryPoint tp;
        tp.point = waypoints[i];
        tp.original_waypoint_index = static_cast<int>(i);
        trajectory.push_back(tp);
    }
    
    return trajectory;
}

bool SimpleTrajectoryGenerator::is_safe(const GeoPoint& /*start*/, const GeoPoint& /*end*/)
{
    // Simple implementation - always return true (no obstacles)
    return true;
}

} // namespace mission_planner
