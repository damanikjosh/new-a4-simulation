#!/usr/bin/env bash
export PYTHONUNBUFFERED=1

PX4_DIR=/home/joshua/Documents/a4_project/new-a4-simulation/PX4-Autopilot
SLEEP_TIME=1

# Create log directory with timestamp
TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
LOG_DIR="log/run_${TIMESTAMP}"
mkdir -p "${LOG_DIR}"
echo "Logs will be saved to: ${LOG_DIR}"


# Cleanup function to kill processes on exit
cleanup() {
    echo ""
    echo "Caught Ctrl+C, cleaning up..."
    echo "Killing PX4 processes..."
    pkill -9 px4
    echo "Killing ROS2 processes..."
    pkill -9 ros2
    echo "Killing Python3 processes..."
    pkill -9 python3
    echo "Cleanup complete. Exiting..."
    exit 0
}

# Trap SIGINT (Ctrl+C) and SIGTERM signals
trap cleanup SIGINT SIGTERM

# Spawn quadrotors
sleep ${SLEEP_TIME} && echo "Spawning quadcopter 1..."
PX4_GZ_MODEL_POSE=-263,-464,2,0,0,-0.643501109 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_depth ${PX4_DIR}/build/px4_sitl_default/bin/px4 -d -i 1 > "${LOG_DIR}/px4_1.log" 2>&1 &
sleep ${SLEEP_TIME} && echo "Spawning quadcopter 2..."
PX4_GZ_MODEL_POSE=-260,-460,2,0,0,-0.643501109 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_depth ${PX4_DIR}/build/px4_sitl_default/bin/px4 -d -i 2 > "${LOG_DIR}/px4_2.log" 2>&1 &
sleep ${SLEEP_TIME} && echo "Spawning quadcopter 3..."
PX4_GZ_MODEL_POSE=-257,-456,2,0,0,-0.643501109 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_depth ${PX4_DIR}/build/px4_sitl_default/bin/px4 -d -i 3 > "${LOG_DIR}/px4_3.log" 2>&1 &

# Spawn rover
sleep ${SLEEP_TIME} && echo "Spawning USV 4..."
PX4_GZ_MODEL_POSE=-233,-484,0,0,0,-0.643501109 PX4_SYS_AUTOSTART=22000 PX4_SIM_MODEL=wam-v ${PX4_DIR}/build/px4_sitl_default/bin/px4 -d -i 4 > "${LOG_DIR}/px4_4.log" 2>&1 &
sleep ${SLEEP_TIME} && echo "Spawning USV 5..."
PX4_GZ_MODEL_POSE=-227,-476,0,0,0,-0.643501109 PX4_SYS_AUTOSTART=22000 PX4_SIM_MODEL=wam-v ${PX4_DIR}/build/px4_sitl_default/bin/px4 -d -i 5 > "${LOG_DIR}/px4_5.log" 2>&1 &
sleep ${SLEEP_TIME} && echo "Spawning USV 8..."
PX4_GZ_MODEL_POSE=-598,-1303,0,0,0,1.57 PX4_SYS_AUTOSTART=22001 PX4_SIM_MODEL=vessel-e_indicator ${PX4_DIR}/build/px4_sitl_default/bin/px4 -d -i 8 > "${LOG_DIR}/px4_8.log" 2>&1 &
sleep ${SLEEP_TIME} && echo "Spawning USV 9..."
PX4_GZ_MODEL_POSE=997,-953,0,0,0,0 PX4_SYS_AUTOSTART=22001 PX4_SIM_MODEL=vessel-e_indicator ${PX4_DIR}/build/px4_sitl_default/bin/px4 -d -i 9 > "${LOG_DIR}/px4_9.log" 2>&1 &

# Run the script here
# sleep 3 && echo "Running the mission script..."
# python3 mission_multiple.py &
# MicroXRCEAgent udp4 -p 8888 &

# echo "Starting ROS2 launch (logging to ${LOG_DIR}/ros2.log)..."
# ros2 launch mission_planner vehicles.launch.py 2>&1 | tee "${LOG_DIR}/ros2.log"


tail -f /dev/null
