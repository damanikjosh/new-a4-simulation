import asyncio
import numpy as np
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

from task_planner.read_objectives import read_objectives
from task_planner.process_task import process_task
from task_planner.solve_task import solve_task
from scenarios.illegal_fishing.missions.generate_trajectory import get_chungdo_obstacles

from scenarios.illegal_fishing.missions.waypoints_mission import WaypointsMission

import os

base_path = os.path.dirname(os.path.realpath(__file__))

obstacles = get_chungdo_obstacles()

async def initialize_vehicle(port):
    vehicle = System(port=port)
    vehicle.port = port
    await vehicle.connect(system_address=f"udp://:{port}")

    print("Waiting for vehicle to connect...")
    async for state in vehicle.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to vehicle on {port}!")
            break

    return vehicle

async def get_position(vehicle):
    print(f"Waiting for vehicle to have a global position estimate...")
    async for health in vehicle.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print(f"-- Global position estimate OK")
            break

    async for position in vehicle.telemetry.position():
        return np.array([position.longitude_deg, position.latitude_deg])

async def upload_mission(vehicle, lonlat_waypoints, altitude=30):
    mission_items = []
    for lon, lat in lonlat_waypoints:
        mission_items.append(MissionItem(
            latitude_deg=lat,                               # Latitude in degrees (range: -90 to +90)
            longitude_deg=lon,                              # Longitude in degrees (range: -180 to +180)
            relative_altitude_m=altitude,                   # Altitude relative to takeoff altitude in metres
            speed_m_s=10,                                    # Speed to use after this mission item (in metres/second)
            is_fly_through=True,                            # True will make the drone fly through without stopping, while false will make the drone stop on the waypoint
            gimbal_pitch_deg=float('nan'),                  # Gimbal pitch (in degrees)
            gimbal_yaw_deg=float('nan'),                    # Gimbal yaw (in degrees)
            camera_action=MissionItem.CameraAction.NONE,    # Camera action to trigger at this mission item
            loiter_time_s=float('nan'),                     # Loiter time (in seconds)
            camera_photo_interval_s=float('nan'),           # Camera photo interval to use after this mission item (in seconds)
            acceptance_radius_m=float('nan'),                          # Radius for completing a mission item (in metres)
            yaw_deg=float('nan'),                           # Absolute yaw angle (in degrees)
            camera_photo_distance_m=float('nan'),           # Camera photo distance to use after this mission item (in meters)
            vehicle_action=MissionItem.VehicleAction.NONE)) # Vehicle action to trigger at this mission item.
    
    mission_plan = MissionPlan(mission_items)

    await vehicle.mission.set_return_to_launch_after_mission(True)

    print("-- Uploading mission")
    await vehicle.mission.upload_mission(mission_plan)


async def run():
    # Define the drone ports and waypoints
    
    drone1 = await initialize_vehicle(14541)
    drone2 = await initialize_vehicle(14542)
    drone3 = await initialize_vehicle(14543)

    drone1_position = await get_position(drone1)
    drone2_position = await get_position(drone2)
    drone3_position = await get_position(drone3)

    vehicle_points = [drone1_position, drone2_position, drone3_position]

    # obj1_points, obj2_points, obj3_points = read_objectives('data/T_Objectives_latlon.xlsx')
    obj1_points, obj2_points, obj3_points = read_objectives(os.path.join(base_path, 'data/T_Objectives_latlon.xlsx'))
    points, reqs, done = process_task(obj1_points, obj2_points, obj3_points)

    latlons = solve_task(points, reqs, done, vehicle_points)

    drone1_waypoints = generate_trajectory(np.concatenate([[drone1_position], latlons[0], [drone1_position]]), obstacles)
    drone2_waypoints = generate_trajectory(np.concatenate([[drone2_position], latlons[1], [drone2_position]]), obstacles)
    drone3_waypoints = generate_trajectory(np.concatenate([[drone3_position], latlons[2], [drone3_position]]), obstacles)

    drone1_surveilance = await WaypointsMission(drone1, 

    await upload_mission(drone1, generate_trajectory(np.concatenate([[drone1_position], latlons[0], [drone1_position]]), obstacles))
    await upload_mission(drone2, generate_trajectory(np.concatenate([[drone2_position], latlons[1], [drone2_position]]), obstacles))
    await upload_mission(drone3, generate_trajectory(np.concatenate([[drone3_position], latlons[2], [drone3_position]]), obstacles))

    # Start missions in parallel
    tasks = [start_mission(vehicle) for vehicle in [drone1, drone2, drone3]]
    await asyncio.gather(*tasks)




async def start_mission(vehicle):
    """Arm the vehicle and start its mission."""
    print(f"Waiting for vehicle on {vehicle.port} to have a global position estimate...")
    async for health in vehicle.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print(f"-- Global position estimate OK for vehicle on {vehicle.port}")
            break

    print(f"-- Arming vehicle on {vehicle.port}")
    await vehicle.action.arm()

    print(f"-- Starting mission for vehicle on {vehicle.port}")
    await vehicle.mission.start_mission()

    # Monitor mission progress
    print_mission_progress_task = asyncio.ensure_future(print_mission_progress(vehicle, vehicle.port))
    running_tasks = [print_mission_progress_task]

    # Wait for the mission to complete
    await observe_is_in_air(vehicle, running_tasks, vehicle.port)


async def print_mission_progress(vehicle, port):
    async for mission_progress in vehicle.mission.mission_progress():
        print(f"Vehicle on {port} Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")


async def observe_is_in_air(vehicle, running_tasks, port):
    """Monitors whether the vehicle is flying or not and
    returns after landing."""

    was_in_air = False

    async for is_in_air in vehicle.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            print(f"-- Vehicle on {port} has landed.")
            return


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
