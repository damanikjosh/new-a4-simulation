#!/usr/bin/env bash

PX4_DIR=/home/joshua/Documents/a4_project/new-a4-simulation/PX4-Autopilot
A4_MODELS_DIR=/home/joshua/Documents/a4_project/new-a4-simulation/a4_models

WORLD=chungdo_illegal_fishing

# Start Gazebo
GZ_SIM_RESOURCE_PATH=${A4_MODELS_DIR}/models:${PX4_DIR}/Tools/simulation/gz/models gz sim -r ${A4_MODELS_DIR}/worlds/chungdo_illegal_fishing.sdf &
# echo "Waiting for Gazebo..." && sleep 5

# # Spawn vehicles
# # Tutorial: https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation.html#multiple-vehicles-with-ros-2-and-gazebo
# sleep echo "Open http://127.0.0.1:5000 on browser to launch the visualization!"
# python3 /root/visualization/server.py > /dev/null 2>&1 &

# Spawn quadrotors
sleep 3 && echo "Spawning quadcopter 1..."
PX4_GZ_MODEL_POSE=-263,-464,2,0,0,-0.643501109 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_depth ${PX4_DIR}/build/px4_sitl_default/bin/px4 -i 1 > /dev/null 2>&1 &
sleep 3 && echo "Spawning quadcopter 2..."
PX4_GZ_MODEL_POSE=-260,-460,2,0,0,-0.643501109 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_depth ${PX4_DIR}/build/px4_sitl_default/bin/px4 -i 2 > /dev/null 2>&1 &
sleep 3 && echo "Spawning quadcopter 3..."
PX4_GZ_MODEL_POSE=-257,-456,2,0,0,-0.643501109 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_depth ${PX4_DIR}/build/px4_sitl_default/bin/px4 -i 3 > /dev/null 2>&1 &

# Spawn rover
sleep 3 && echo "Spawning USV 4..."
PX4_GZ_MODEL_POSE=-233,-484,0,0,0,-0.643501109 PX4_SYS_AUTOSTART=22000 PX4_SIM_MODEL=wam-v ${PX4_DIR}/build/px4_sitl_default/bin/px4 -i 4 > /dev/null 2>&1 &
sleep 3 && echo "Spawning USV 5..."
PX4_GZ_MODEL_POSE=-227,-476,0,0,0,-0.643501109 PX4_SYS_AUTOSTART=22000 PX4_SIM_MODEL=wam-v ${PX4_DIR}/build/px4_sitl_default/bin/px4 -i 5 > /dev/null 2>&1 &
sleep 3 && echo "Spawning USV 8..."
PX4_GZ_MODEL_POSE=-598,-1303,0,0,0,1.57 PX4_SYS_AUTOSTART=22001 PX4_SIM_MODEL=vessel-e_indicator ${PX4_DIR}/build/px4_sitl_default/bin/px4 -i 8 > /dev/null 2>&1 &
sleep 3 && echo "Spawning USV 9..."
PX4_GZ_MODEL_POSE=997,-953,0,0,0,0 PX4_SYS_AUTOSTART=22001 PX4_SIM_MODEL=vessel-e_indicator ${PX4_DIR}/build/px4_sitl_default/bin/px4 -i 9 > /dev/null 2>&1 &

# Run the script here
# sleep 3 && echo "Running the mission script..."
# python3 mission_multiple.py &
# MicroXRCEAgent udp4 -p 8888 &

ros2 launch mission_planner vehicles.launch.py


tail -f /dev/null
