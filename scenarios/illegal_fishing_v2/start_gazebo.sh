#!/usr/bin/env bash

PX4_DIR=/home/joshua/Documents/a4_project/new-a4-simulation/PX4-Autopilot
A4_MODELS_DIR=/home/joshua/Documents/a4_project/new-a4-simulation/a4_models

WORLD=chungdo_illegal_fishing

# Start Gazebo
GZ_SIM_RESOURCE_PATH=${A4_MODELS_DIR}/models:${PX4_DIR}/Tools/simulation/gz/models gz sim -r ${A4_MODELS_DIR}/worlds/chungdo_illegal_fishing.sdf