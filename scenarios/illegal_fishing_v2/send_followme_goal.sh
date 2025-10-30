#!/usr/bin/env bash

# ROS2 action send_goal command for Followme action
# Target vehicle: enemy8 (illegal fishing vessel)
# The UAV will follow the target vehicle's position updates

ros2 action send_goal /uav1/task/followme mission_planner_msgs/action/Followme "
vehicle_name: 'usv4'
" --feedback
