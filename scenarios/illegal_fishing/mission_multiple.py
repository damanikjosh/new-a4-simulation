import asyncio
import numpy as np
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

from task_planner.read_objectives import read_objectives
from task_planner.process_task import process_task
from task_planner.solve_task import solve_task

from missions.waypoints_mission import WaypointsMission
from missions.surveillance_mission import SurveillanceMission
from missions.follow_mission import FollowMission
from surveillance.vehicle_positions_surveillance import VehiclePositionSurveillance

import os

base_path = os.path.dirname(os.path.realpath(__file__))

async def initialize_vehicle(port):
    vehicle = System(port=port)
    vehicle.port = port
    vehicle.found = False
    vehicle.busy = False
    vehicle.peers = []
    await vehicle.connect(system_address=f"udp://:{port}")

    print("Waiting for vehicle to connect...")
    async for state in vehicle.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to vehicle on {port}!")
            break

    return vehicle

async def get_position(vehicle):
    print("Waiting for vehicle to have a global position estimate...")
    async for health in vehicle.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print(f"-- Global position estimate OK")
            break

    async for position in vehicle.telemetry.position():
        return np.array([position.longitude_deg, position.latitude_deg])

coroutines = []
task_points = []
task_reqs = []
task_done = []
task_types = []

solution_idx = []

async def reset():
    """Cancel and await all running tasks stored in `coroutines`.

    This ensures background gRPC iterators are closed on the main thread
    instead of being implicitly destroyed on worker threads which can
    trigger "cannot join current thread" in aiogrpc.__del__.
    """
    # cancel all tasks
    for task in list(coroutines):
        try:
            task.cancel()
        except Exception:
            pass

    # await their completion
    for task in list(coroutines):
        try:
            await task
        except asyncio.CancelledError:
            pass
        except Exception as e:
            print(f"reset: task raised during shutdown: {e}")

    coroutines.clear()

async def plan_mission(vehicle, task_points, task_reqs, task_done, task_types):
    pass

async def run():

    
    # Define the drone ports and waypoints
    
    drone1 = await initialize_vehicle(14541)
    drone2 = await initialize_vehicle(14542)
    drone3 = await initialize_vehicle(14543)

    usv4 = await initialize_vehicle(14544)
    usv5 = await initialize_vehicle(14545)

    enemy8 = await initialize_vehicle(14548)
    enemy9 = await initialize_vehicle(14549)

    available_drones = [drone1, drone2, drone3]
    available_usvs = [usv4, usv5]
    available_enemies = [enemy8, enemy9]

    for drone in available_drones:
        await drone.mission.clear_mission()

    for usv in available_usvs:
        await usv.mission.clear_mission()


    # obj1_points, obj2_points, obj3_points = read_objectives('data/T_Objectives_latlon.xlsx')
    obj1_points, obj2_points, obj3_points = read_objectives(os.path.join(base_path, 'data/T_Objectives_latlon.xlsx'))
    # points, reqs, done, types = process_task(obj1_points, obj2_points, obj3_points)
    
    obj1_vehicle_points = [await get_position(drone) for drone in available_drones]
    obj1_reqs = np.zeros((len(obj1_points), len(obj1_points)))
    obj1_done = np.zeros(len(obj1_points))

    obj1_solution_points, obj1_solution_idx = solve_task(obj1_points, obj1_reqs, obj1_done, obj1_vehicle_points)
    
    enemies_surveillance = VehiclePositionSurveillance([enemy8, enemy9])

    enemy_endpoint = [127.079252,  34.376558]
    enemy8.waypoint_mission = WaypointsMission(enemy8, [enemy_endpoint], autostart=False)
    enemy9.waypoint_mission = WaypointsMission(enemy9, [enemy_endpoint], autostart=False)

    # Sample random 5 points from obj1_points
    random_indices = np.random.randint(0, len(obj1_points), size=5)
    enemy8.random_mission = WaypointsMission(enemy8, [obj1_points[i] for i in random_indices], autostart=True)

    random_indices = np.random.randint(0, len(obj1_points), size=5)
    enemy9.random_mission = WaypointsMission(enemy9, [obj1_points[i] for i in random_indices], autostart=True)

    def on_usv_arrived(vehicle, target):
        print(f"USV {vehicle.port} arrived at target {target.port}, releasing USV back to available pool")
        for peer in vehicle.peers:
            print(f"Releasing drone {peer.port} back to surveillance mission")
            peer.busy = False
            peer.follow_mission.disable()
            peer.surveillance_mission.enable()
        # Remove peers
        vehicle.peers = []
        target.random_mission.disable()
        target.waypoint_mission.enable()

    usv4.follow_mission = FollowMission(usv4, None, enemies_surveillance, autostart=False, on_arrived=on_usv_arrived)
    usv5.follow_mission = FollowMission(usv5, None, enemies_surveillance, autostart=False, on_arrived=on_usv_arrived)


    def on_drone_surveillance_enemy_found(vehicle, enemies):
        for enemy in enemies:
            if enemy.found:
                return
            enemy.found = True
            print(f"Drone {vehicle.port} found enemy vehicle {enemy.port}, assigning follow mission")
            vehicle.busy = True
            vehicle.surveillance_mission.disable()
            vehicle.follow_mission.set_target(enemy)
            vehicle.target = enemy
            vehicle.follow_mission.enable()

            for usv in available_usvs:
                if usv.busy:
                    continue
                usv.busy = True
                usv.follow_mission.set_target(enemy)
                usv.target = enemy
                usv.follow_mission.enable()
                usv.peers.append(vehicle)
                break
                
    drone1.surveillance_mission = SurveillanceMission(drone1, obj1_solution_points[0], enemies_surveillance, on_drone_surveillance_enemy_found, return_to_launch_after_mission=True)
    drone2.surveillance_mission = SurveillanceMission(drone2, obj1_solution_points[1], enemies_surveillance, on_drone_surveillance_enemy_found, return_to_launch_after_mission=True)
    drone3.surveillance_mission = SurveillanceMission(drone3, obj1_solution_points[2], enemies_surveillance, on_drone_surveillance_enemy_found, return_to_launch_after_mission=True)

    drone1.follow_mission = FollowMission(drone1, None, enemies_surveillance, autostart=False, altitude=20)
    drone2.follow_mission = FollowMission(drone2, None, enemies_surveillance, autostart=False, altitude=20)
    drone3.follow_mission = FollowMission(drone3, None, enemies_surveillance, autostart=False, altitude=20)


    await drone1.surveillance_mission.initialize()
    await drone2.surveillance_mission.initialize()
    await drone3.surveillance_mission.initialize()

    await usv4.follow_mission.initialize()
    await usv5.follow_mission.initialize()

    await enemy8.waypoint_mission.initialize()
    await enemy9.waypoint_mission.initialize()

    await enemy8.random_mission.initialize()
    await enemy9.random_mission.initialize()

    await drone1.follow_mission.initialize()
    await drone2.follow_mission.initialize()
    await drone3.follow_mission.initialize()

    # Create actual asyncio Tasks so we can cancel/await them later
    coroutines.append(asyncio.create_task(enemies_surveillance.run()))
    coroutines.append(asyncio.create_task(drone1.surveillance_mission.run()))
    coroutines.append(asyncio.create_task(drone2.surveillance_mission.run()))
    coroutines.append(asyncio.create_task(drone3.surveillance_mission.run()))
    coroutines.append(asyncio.create_task(usv4.follow_mission.run()))
    coroutines.append(asyncio.create_task(usv5.follow_mission.run()))
    coroutines.append(asyncio.create_task(enemy8.waypoint_mission.run()))
    coroutines.append(asyncio.create_task(enemy9.waypoint_mission.run()))
    coroutines.append(asyncio.create_task(enemy8.random_mission.run()))
    coroutines.append(asyncio.create_task(enemy9.random_mission.run()))
    coroutines.append(asyncio.create_task(drone1.follow_mission.run()))
    coroutines.append(asyncio.create_task(drone2.follow_mission.run()))
    coroutines.append(asyncio.create_task(drone3.follow_mission.run()))

    # Start missions in parallel and wait for them. If the main program
    # needs to shut down, call `await reset()` to cancel and join tasks
    try:
        await asyncio.gather(*coroutines)
    finally:
        await reset()


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
