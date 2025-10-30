#!/usr/bin/env bash

# ROS2 action send_goal command for Search action
# Waypoints: 3 points
# Target vehicles: enemy8, enemy9
# Search radius: 100 meters (adjustable)

ros2 action send_goal /uav1/task/search mission_planner_msgs/action/Search "
waypoints:
- latitude_deg: 34.3686639155223
  longitude_deg: 127.073088439355
  absolute_altitude_m: 30.0
  speed_m_s: 15.0
  is_fly_through: true
  gimbal_pitch_deg: 0.0
  gimbal_yaw_deg: 0.0
  loiter_time_s: 0.0
- latitude_deg: 34.3695622308065
  longitude_deg: 127.073088439355
  absolute_altitude_m: 30.0
  speed_m_s: 15.0
  is_fly_through: true
  gimbal_pitch_deg: 0.0
  gimbal_yaw_deg: 0.0
  loiter_time_s: 0.0
- latitude_deg: 34.3704605460906
  longitude_deg: 127.073088439355
  absolute_altitude_m: 30.0
  speed_m_s: 15.0
  is_fly_through: true
  gimbal_pitch_deg: 0.0
  gimbal_yaw_deg: 0.0
  loiter_time_s: 0.0
vehicle_names: ['enemy8', 'enemy9']
search_radius_m: 100.0
" --feedback
