#!/usr/bin/env bash

# ROS2 action send_goal command for Return to Launch (RTL) action
# Returns the UAV to its home/launch position

ros2 action send_goal /uav1/task/rtl mission_planner_msgs/action/ReturnToLaunch "{}" --feedback
