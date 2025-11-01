import asyncio
import numpy as np
from missions.waypoints_mission import WaypointsMission
from mavsdk.mission import MissionItem, MissionPlan

class SurveillanceMission(WaypointsMission):

    def __init__(self, vehicle, task_points, task_indices, vehicle_surveillance, *args, on_progress=None, on_target_found=None, **kwargs):
        super().__init__(vehicle, task_points, task_indices, *args, on_progress=on_progress, **kwargs)
        self.vehicle_surveillance = vehicle_surveillance
        self.on_target_found = on_target_found

    def get_coroutines(self):
        vehicle_surveillance_task = asyncio.ensure_future(self.survey_vehicles())
        return super().get_coroutines() + [vehicle_surveillance_task]
    
    async def survey_vehicles(self):
        while True:
            current_position = await self.get_position()
            vehicles, _ = self.vehicle_surveillance.get_on_radius(current_position, 0.001)
            ports = [vehicle._port for vehicle in vehicles]
            if len(ports) > 0:
                print(f"Vehicle {self.vehicle._port} sees vehicles: {ports}")
                if self.on_target_found is not None:
                    await self.on_target_found(self.vehicle, vehicles)
            await asyncio.sleep(1)
