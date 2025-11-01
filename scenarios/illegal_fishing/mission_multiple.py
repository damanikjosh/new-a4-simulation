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
    await vehicle.connect(system_address=f"udpin://127.0.0.1:{port}")

    print("Waiting for vehicle to connect...")
    async for state in vehicle.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to vehicle on {port}!")
            break

    return vehicle

async def get_position(vehicle):
    # print("Waiting for vehicle to have a global position estimate...")
    # async for health in vehicle.telemetry.health():
    #     if health.is_global_position_ok and health.is_home_position_ok:
    #         print(f"-- Global position estimate OK")
    #         break

    async for position in vehicle.telemetry.position():
        return np.array([position.longitude_deg, position.latitude_deg])

# Haversine distance in meters between two lon/lat points
def haversine_meters(lon1, lat1, lon2, lat2):
    R = 6371000.0  # Earth radius in meters
    phi1 = np.radians(lat1)
    phi2 = np.radians(lat2)
    dphi = np.radians(lat2 - lat1)
    dlambda = np.radians(lon2 - lon1)
    a = np.sin(dphi / 2.0) ** 2 + np.cos(phi1) * np.cos(phi2) * np.sin(dlambda / 2.0) ** 2
    c = 2.0 * np.arcsin(np.sqrt(a))
    return R * c

def sample_points_within_radius(points, center_lon_lat, radius_m=500.0, k=5):
    """Return up to k points and their original indices within radius of center.

    If fewer than k are within radius, return all of them. If none are within
    radius, fall back to the k nearest points overall.
    """
    lon0, lat0 = float(center_lon_lat[0]), float(center_lon_lat[1])
    dists = []
    for i, p in enumerate(points):
        lon, lat = float(p[0]), float(p[1])
        d = haversine_meters(lon0, lat0, lon, lat)
        dists.append((i, d))

    in_radius = [i for i, d in dists if d <= radius_m]
    if len(in_radius) == 0:
        # Fallback: choose k nearest points
        nearest = [i for i, _ in sorted(dists, key=lambda x: x[1])]
        chosen = nearest[: min(k, len(nearest))]
    else:
        if len(in_radius) <= k:
            chosen = in_radius
        else:
            # Randomly sample without replacement from candidates
            chosen = list(map(int, np.random.choice(in_radius, size=k, replace=False)))

    pts = [list(points[i]) for i in chosen]
    idx = [int(i) for i in chosen]
    return pts, idx

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

    for drone in available_drones:
        await drone.mission.clear_mission()

    for usv in available_usvs:
        await usv.mission.clear_mission()


    # obj1_points, obj2_points, obj3_points = read_objectives('data/T_Objectives_latlon.xlsx')
    obj1_points, obj2_points, obj3_points = read_objectives(os.path.join(base_path, 'data/T_Objectives_latlon.xlsx'))
    obj1_reqs = np.zeros((len(obj1_points), len(obj1_points)))
    obj1_done = np.zeros(len(obj1_points))

    # points, reqs, done, types = process_task(obj1_points, obj2_points, obj3_points)
    async def replan():
        print("Replanning missions for available drones...")
        planned_drones = [drone for drone in available_drones if not drone.busy]
        print(f"Planned drones: {[drone.port for drone in planned_drones]}")
        # Compute vehicle start positions once in the same order as planned_drones
        obj1_vehicle_points = [await get_position(drone) for drone in planned_drones]
        obj1_solution_points, obj1_solution_idx = solve_task(obj1_points, obj1_reqs, obj1_done, obj1_vehicle_points)

        # Sanity: ensure mapping between indices and points is consistent
        try:
            for i in range(len(planned_drones)):
                pts = np.asarray(obj1_solution_points[i])
                idx = np.asarray(obj1_solution_idx[i], dtype=int)
                if len(pts) != len(idx):
                    print(f"[replan] WARN len mismatch vehicle {planned_drones[i].port}: points={len(pts)} idx={len(idx)}")
                else:
                    mism = np.where(np.any(pts != np.asarray(obj1_points)[idx], axis=1))[0]
                    if len(mism) > 0:
                        print(f"[replan] WARN index->point mismatch for vehicle {planned_drones[i].port} at steps {mism[:5]} ...")
        except Exception:
            pass

        # Push plans to each drone in the same order
        for i, drone in enumerate(planned_drones):
            # Convert to plain python lists to avoid numpy dtypes downstream
            pts_list = [list(p) for p in obj1_solution_points[i]]
            idx_list = [int(v) for v in obj1_solution_idx[i]]
            drone.surveillance_mission.replan(pts_list, idx_list)

    
    # obj1_vehicle_points = [await get_position(drone) for drone in available_drones]

    # obj1_solution_points, obj1_solution_idx = solve_task(obj1_points, obj1_reqs, obj1_done, obj1_vehicle_points)
    
    
    enemies_surveillance = VehiclePositionSurveillance([enemy8, enemy9])

    enemy_endpoint = [127.079252,  34.376558]
    enemy8.waypoint_mission = WaypointsMission(enemy8, [enemy_endpoint], [0], autostart=False)
    enemy9.waypoint_mission = WaypointsMission(enemy9, [enemy_endpoint], [0], autostart=False)

    # Sample up to 5 random points within 500 m of each enemy's current location
    enemy8_pos = await get_position(enemy8)
    e8_pts, e8_idx = sample_points_within_radius(obj1_points, enemy8_pos, radius_m=500.0, k=5)
    enemy8.random_mission = WaypointsMission(enemy8, e8_pts, e8_idx, autostart=True)

    enemy9_pos = await get_position(enemy9)
    e9_pts, e9_idx = sample_points_within_radius(obj1_points, enemy9_pos, radius_m=500.0, k=5)
    enemy9.random_mission = WaypointsMission(enemy9, e9_pts, e9_idx, autostart=True)

    async def on_usv_arrived(vehicle, target):
        print(f"USV {vehicle.port} arrived at target {target.port}, releasing USV back to available pool")
        for peer in vehicle.peers:
            print(f"Releasing drone {peer.port} back to surveillance mission")
            peer.busy = False
            peer.follow_mission.disable()
            await replan()
            peer.surveillance_mission.enable()
        # Remove peers
        vehicle.peers = []
        target.random_mission.disable()
        target.waypoint_mission.enable()

    usv4.follow_mission = FollowMission(usv4, None, enemies_surveillance, autostart=False, on_arrived=on_usv_arrived)
    usv5.follow_mission = FollowMission(usv5, None, enemies_surveillance, autostart=False, on_arrived=on_usv_arrived)


    async def on_drone_surveillance_enemy_found(vehicle, enemies):
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

            await replan()

    async def on_waypoint_progress(vehicle, progress_index):
        obj1_done[progress_index] = 1

    drone1.surveillance_mission = SurveillanceMission(drone1, [], [], enemies_surveillance, on_target_found=on_drone_surveillance_enemy_found, on_progress=on_waypoint_progress, return_to_launch_after_mission=True)
    drone2.surveillance_mission = SurveillanceMission(drone2, [], [], enemies_surveillance, on_target_found=on_drone_surveillance_enemy_found, on_progress=on_waypoint_progress, return_to_launch_after_mission=True)
    drone3.surveillance_mission = SurveillanceMission(drone3, [], [], enemies_surveillance, on_target_found=on_drone_surveillance_enemy_found, on_progress=on_waypoint_progress, return_to_launch_after_mission=True)

    await replan()

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
