#ifndef MISSION_PLANNER__TRAJECTORY_GENERATOR_HPP_
#define MISSION_PLANNER__TRAJECTORY_GENERATOR_HPP_

#include <vector>
#include <memory>
#include <string>

namespace mission_planner
{

struct GeoPoint
{
    double latitude_deg;
    double longitude_deg;
};

struct TrajectoryPoint
{
    GeoPoint point;
    int original_waypoint_index;  // -1 for intermittent points, >= 0 for actual waypoints
};

/**
 * @brief Interface for trajectory generation with obstacle avoidance
 */
class TrajectoryGenerator
{
public:
    virtual ~TrajectoryGenerator() = default;

    /**
     * @brief Generate a safe trajectory through waypoints, adding intermittent points to avoid obstacles
     * @param waypoints Original waypoints to navigate through
     * @return Vector of trajectory points including original waypoints and intermittent points
     */
    virtual std::vector<TrajectoryPoint> generate_trajectory(const std::vector<GeoPoint>& waypoints) = 0;

    /**
     * @brief Check if a direct path between two points is safe (no obstacle intersection)
     * @param start Starting point
     * @param end Ending point
     * @return true if path is safe, false otherwise
     */
    virtual bool is_safe(const GeoPoint& start, const GeoPoint& end) = 0;
};

/**
 * @brief Simple pass-through trajectory generator (no obstacle avoidance)
 */
class SimpleTrajectoryGenerator : public TrajectoryGenerator
{
public:
    std::vector<TrajectoryPoint> generate_trajectory(const std::vector<GeoPoint>& waypoints) override;
    bool is_safe(const GeoPoint& start, const GeoPoint& end) override;
};

} // namespace mission_planner

#endif // MISSION_PLANNER__TRAJECTORY_GENERATOR_HPP_
