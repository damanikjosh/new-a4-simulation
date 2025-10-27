#ifndef MISSION_PLANNER__UTILS_HPP_
#define MISSION_PLANNER__UTILS_HPP_

#include <cmath>

namespace mission_planner
{
    namespace utils
    {

        /**
         * @brief Calculate the haversine distance between two GPS coordinates
         * @param lat1 Latitude of first point in degrees
         * @param lon1 Longitude of first point in degrees
         * @param lat2 Latitude of second point in degrees
         * @param lon2 Longitude of second point in degrees
         * @return Distance in meters
         */
        inline double haversine_distance(double lat1, double lon1, double lat2, double lon2)
        {
            constexpr double EARTH_RADIUS_M = 6371000.0; // Earth radius in meters

            // Convert to radians
            const double lat1_rad = lat1 * M_PI / 180.0;
            const double lat2_rad = lat2 * M_PI / 180.0;
            const double delta_lat = (lat2 - lat1) * M_PI / 180.0;
            const double delta_lon = (lon2 - lon1) * M_PI / 180.0;

            // Haversine formula
            const double a = std::sin(delta_lat / 2.0) * std::sin(delta_lat / 2.0) +
                             std::cos(lat1_rad) * std::cos(lat2_rad) *
                                 std::sin(delta_lon / 2.0) * std::sin(delta_lon / 2.0);
            const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

            return EARTH_RADIUS_M * c;
        }

    } // namespace utils
} // namespace mission_planner

#endif // MISSION_PLANNER__UTILS_HPP_
