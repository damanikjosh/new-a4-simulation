#!/usr/bin/env bash

# ROS2 action send_goal command for Waypoints action
# Multiple waypoints mission
# Speed: 15 m/s
# Altitude: 50 meters (absolute)

ros2 action send_goal /uav1/task/waypoints mission_planner_msgs/action/Waypoints "
waypoints:
- latitude_deg: 34.3686639155223
  longitude_deg: 127.073088439355
  absolute_altitude_m: 30.0
  speed_m_s: 15.0
  is_fly_through: true
  gimbal_pitch_deg: 0.0
  gimbal_yaw_deg: 0.0
  loiter_time_s: 0.0
  camera_action: 0
  camera_photo_interval_s: 0.0
  acceptance_radius_m: 0.0
  yaw_deg: 0.0
  camera_photo_distance_m: 0.0
- latitude_deg: 34.3695622308065
  longitude_deg: 127.073088439355
  absolute_altitude_m: 30.0
  speed_m_s: 15.0
  is_fly_through: true
  gimbal_pitch_deg: 0.0
  gimbal_yaw_deg: 0.0
  loiter_time_s: 0.0
  camera_action: 0
  camera_photo_interval_s: 0.0
  acceptance_radius_m: 0.0
  yaw_deg: 0.0
  camera_photo_distance_m: 0.0
- latitude_deg: 34.3704605460906
  longitude_deg: 127.073088439355
  absolute_altitude_m: 30.0
  speed_m_s: 15.0
  is_fly_through: true
  gimbal_pitch_deg: 0.0
  gimbal_yaw_deg: 0.0
  loiter_time_s: 0.0
  camera_action: 0
  camera_photo_interval_s: 0.0
  acceptance_radius_m: 0.0
  yaw_deg: 0.0
  camera_photo_distance_m: 0.0
" --feedback
