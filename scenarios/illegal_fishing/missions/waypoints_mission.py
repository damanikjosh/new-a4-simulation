import asyncio
import numpy as np
from missions.mission import MissionBase
from mavsdk.mission import MissionItem, MissionPlan

from missions.generate_trajectory import get_chungdo_obstacles, generate_trajectory

obstacles = get_chungdo_obstacles()

class WaypointsMission(MissionBase):

    def __init__(self, vehicle, task_points, *args, return_to_launch_after_mission=False, **kwargs):
        super().__init__(vehicle, *args, **kwargs)
        self.task_points = task_points
        # self.task_indices = np.arange(len(task_points))
        self.done = np.zeros(len(task_points))
        self.return_to_launch_after_mission = return_to_launch_after_mission

    async def generate_plan(self, relative_altitude_m=30, speed_m_s=20):
        mission_items = []
        current_position = await self.get_position()
        if not self.return_to_launch_after_mission:
            waypoints = np.concatenate([[current_position], self.task_points])
        else:
            home_position = await self.get_home_position()
            waypoints = np.concatenate([[current_position], self.task_points, [home_position]])
            
        trajectory, task_indices = generate_trajectory(waypoints, obstacles)

        for lon, lat in trajectory:
            mission_items.append(MissionItem(
                latitude_deg=lat,                               # Latitude in degrees (range: -90 to +90)
                longitude_deg=lon,                              # Longitude in degrees (range: -180 to +180)
                relative_altitude_m=relative_altitude_m,        # Altitude relative to takeoff altitude in metres
                speed_m_s=speed_m_s,                            # Speed to use after this mission item (in metres/second)
                is_fly_through=True,                            # True will make the drone fly through without stopping, while false will make the drone stop on the waypoint
                gimbal_pitch_deg=float('nan'),                  # Gimbal pitch (in degrees)
                gimbal_yaw_deg=float('nan'),                    # Gimbal yaw (in degrees)
                camera_action=MissionItem.CameraAction.NONE,    # Camera action to trigger at this mission item
                loiter_time_s=float('nan'),                     # Loiter time (in seconds)
                camera_photo_interval_s=float('nan'),           # Camera photo interval to use after this mission item (in seconds)
                acceptance_radius_m=float('nan'),               # Radius for completing a mission item (in metres)
                yaw_deg=float('nan'),                           # Absolute yaw angle (in degrees)
                camera_photo_distance_m=float('nan'),           # Camera photo distance to use after this mission item (in meters)
                vehicle_action=MissionItem.VehicleAction.NONE)) # Vehicle action to trigger at this mission item.
        
        self.mission_plan = MissionPlan(mission_items)
        await self.vehicle.mission.set_return_to_launch_after_mission(self.return_to_launch_after_mission)


    async def initialize(self, relative_altitude_m=30, speed_m_s=20):
        await super().initialize()
        

    async def before_start(self):
        await self.generate_plan()
        await self.vehicle.mission.upload_mission(self.mission_plan)
        # Delay for 1 second to ensure mission is uploaded before starting
        await asyncio.sleep(1)

    async def on_start(self):
        await self.vehicle.mission.start_mission()

    def get_coroutines(self):
        monitor_progress_cor = asyncio.ensure_future(self.monitor_progress())
        return [monitor_progress_cor]

    async def monitor_progress(self):
        
        # Monitor mission progress. Exclude first task (takeoff)
        last_task = 0
        try:
            async for mission_progress in self.vehicle.mission.mission_progress():
                current_task = mission_progress.current
                if last_task is None:
                    last_task = current_task
                total_task = mission_progress.total
                print(f"Vehicle {self.vehicle._port} is at task {current_task}/{total_task}")
                last_task = current_task

        except asyncio.CancelledError:
            pass

