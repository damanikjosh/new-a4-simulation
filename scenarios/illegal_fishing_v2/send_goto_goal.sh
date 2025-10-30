#!/usr/bin/env bash

# ROS2 action send_goal command for Goto action
# Target: Single waypoint
# Speed: 15 m/s
# Altitude: 50 meters (absolute)

ros2 action send_goal /uav1/task/goto mission_planner_msgs/action/Goto "
latitude_deg: 34.3695622308065
longitude_deg: 127.073088439355
absolute_altitude_m: 10.0
speed_m_s: 15.0
" --feedback
