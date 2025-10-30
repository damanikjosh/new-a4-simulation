import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.follow_me import (Config, FollowMeError, TargetLocation)


class Vehicle(System):
    def __init__(self, port, **kwargs):
        super().__init__(port=port, **kwargs)
        self.port = port

async def follow_me(vehicles: Vehicle, target: Vehicle):

    print("-- Arming")
    for vehicle in vehicles:
        await vehicle.action.arm()

    print("-- Taking Off")
    for vehicle in vehicles:
        await vehicle.action.takeoff()
    await asyncio.sleep(3)

    while True:
        mission_items = []
        async for position in target.telemetry.position():
            if position is None:
                continue
            print(position)

            mission_items.append(MissionItem(
                latitude_deg=position.latitude_deg,                 # Latitude in degrees (range: -90 to +90)
                longitude_deg=position.longitude_deg,               # Longitude in degrees (range: -180 to +180)
                relative_altitude_m=position.absolute_altitude_m+8, # Altitude relative to takeoff altitude in metres
                speed_m_s=10,                                       # Speed to use after this mission item (in metres/second)
                is_fly_through=True,                                # True will make the drone fly through without stopping, while false will make the drone stop on the waypoint
                gimbal_pitch_deg=float('nan'),                      # Gimbal pitch (in degrees)
                gimbal_yaw_deg=float('nan'),                        # Gimbal yaw (in degrees)
                camera_action=MissionItem.CameraAction.NONE,        # Camera action to trigger at this mission item
                loiter_time_s=float('nan'),                         # Loiter time (in seconds)
                camera_photo_interval_s=float('nan'),               # Camera photo interval to use after this mission item (in seconds)
                acceptance_radius_m=10,                             # Radius for completing a mission item (in metres)
                yaw_deg=float('nan'),                               # Absolute yaw angle (in degrees)
                camera_photo_distance_m=float('nan'),               # Camera photo distance to use after this mission item (in meters)
                vehicle_action=MissionItem.VehicleAction.NONE))     # Vehicle action to trigger at this mission item.

            break
        mission_plan = MissionPlan(mission_items)

        for vehicle in vehicles:
            await vehicle.mission.upload_mission(mission_plan)

        for vehicle in vehicles:
            await vehicle.mission.start_mission()

        await asyncio.sleep(0.1)

        

async def initialize_vehicle(port):
    vehicle = Vehicle(port=port)
    vehicle.port = port
    await vehicle.connect(system_address=f"udp://:{port}")

    print("Waiting for vehicle to connect...")
    async for state in vehicle.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to vehicle on {port}!")
            break

    return vehicle

async def run():
    # Define the drone ports and waypoints
    drone1 = await initialize_vehicle(14541)
    # drone2 = await initialize_vehicle(14542)
    # drone3 = await initialize_vehicle(14543)

    usv1 = await initialize_vehicle(14544)
    usv2 = await initialize_vehicle(14545)
    # drones = [drone1, drone2, drone3]
    # usvs = [usv1, usv2]

    await follow_me([drone1, usv1], usv2)


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
