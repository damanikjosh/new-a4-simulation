#!/usr/bin/env python3

import asyncio

from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

from mission.read_objectives import read_objectives
from mission.process_task import process_task
from mission.solve_task import solve_task
port = 14544


async def run():
    drone = System()
    await drone.connect(system_address=f"udp://:{port}")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    print_mission_progress_task = asyncio.ensure_future(
        print_mission_progress(drone))

    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(
        observe_is_in_air(drone, running_tasks))

    mission_items = []
    for lon, lat in lon_lat_interp:
        mission_items.append(MissionItem(
            latitude_deg=lat,                               # Latitude in degrees (range: -90 to +90)
            longitude_deg=lon,                              # Longitude in degrees (range: -180 to +180)
            relative_altitude_m=0,                         # Altitude relative to takeoff altitude in metres
            speed_m_s=10,                                    # Speed to use after this mission item (in metres/second)
            is_fly_through=False,                            # True will make the drone fly through without stopping, while false will make the drone stop on the waypoint
            gimbal_pitch_deg=float('nan'),                  # Gimbal pitch (in degrees)
            gimbal_yaw_deg=float('nan'),                    # Gimbal yaw (in degrees)
            camera_action=MissionItem.CameraAction.NONE,    # Camera action to trigger at this mission item
            loiter_time_s=float('nan'),                     # Loiter time (in seconds)
            camera_photo_interval_s=float('nan'),           # Camera photo interval to use after this mission item (in seconds)
            acceptance_radius_m=float('nan'),               # Radius for completing a mission item (in metres)
            yaw_deg=float('nan'),                           # Absolute yaw angle (in degrees)
            camera_photo_distance_m=float('nan'),           # Camera photo distance to use after this mission item (in meters)
            vehicle_action=MissionItem.VehicleAction.NONE)) # Vehicle action to trigger at this mission item.


    mission_plan = MissionPlan(mission_items)

    await drone.mission.set_return_to_launch_after_mission(True)

    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Starting mission")
    await drone.mission.start_mission()

    await termination_task


async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")


async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return


if __name__ == "__main__":
    # Run the asyncio loop


    asyncio.run(run())