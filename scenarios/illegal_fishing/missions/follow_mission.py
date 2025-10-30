import asyncio
import numpy as np
from missions.mission import MissionBase
from mavsdk.mission import MissionItem, MissionPlan

from missions.generate_trajectory import get_chungdo_obstacles, generate_trajectory

obstacles = get_chungdo_obstacles()

class FollowMission(MissionBase,):

    def __init__(self, vehicle, target, vehicle_surveillance, *args, on_arrived=None, altitude=0.0, **kwargs):
        super().__init__(vehicle, *args, **kwargs)
        self.target = target
        self.vehicle_surveillance = vehicle_surveillance
        self.on_arrived = on_arrived
        self.altitude = altitude

    async def initialize(self):
        await super().initialize()

    def set_target(self, target):
        self.target = target

    def get_coroutines(self):
        # Prefer MissionBase.create_task so base tracks and can cancel it
        try:
            task = self.create_task(self.follow())
            return [task]
        except Exception:
            follow_cor = asyncio.ensure_future(self.follow())
            return [follow_cor]

    async def follow(self):
        try:
            await self.vehicle.mission.set_return_to_launch_after_mission(False)
            while True:
                try:
                    _, traget_position = self.vehicle_surveillance.get_by_port(self.target._port)
                except ValueError:
                    print(f"FollowMission: target vehicle {getattr(self.target,'_port','?')} position not found, retrying...")
                    await asyncio.sleep(1)
                    continue

                if traget_position is None:
                    await asyncio.sleep(1)
                    continue
                
                current_position = await self.get_position()

                print(f"FollowMission: vehicle {getattr(self.vehicle,'_port','?')} target pos={traget_position} current={current_position}")

                distance = np.linalg.norm(traget_position - current_position)
                if distance < 0.001:
                    print(f"Vehicle {self.vehicle._port} arrived at the target vehicle {self.target._port}")
                    if self.on_arrived is not None:
                        # call callback: await if coroutine, else schedule
                        if asyncio.iscoroutinefunction(self.on_arrived):
                            await self.on_arrived(self.vehicle, self.target)
                        else:
                            # schedule sync callback on loop
                            asyncio.get_running_loop().call_soon_threadsafe(self.on_arrived, self.vehicle, self.target)

                waypoints = np.concatenate([[current_position], [traget_position]])

                trajectory, _ = generate_trajectory(waypoints, obstacles)

                mission_items = []
                for lon, lat in trajectory:
                    mission_items.append(MissionItem(
                        latitude_deg=lat,
                        longitude_deg=lon,
                        relative_altitude_m=self.altitude,
                        speed_m_s=10,
                        is_fly_through=True,
                        gimbal_pitch_deg=float('nan'),
                        gimbal_yaw_deg=float('nan'),
                        camera_action=MissionItem.CameraAction.NONE,
                        loiter_time_s=float('nan'),
                        camera_photo_interval_s=float('nan'),
                        acceptance_radius_m=float('nan'),
                        yaw_deg=float('nan'),
                        camera_photo_distance_m=float('nan'),
                        vehicle_action=MissionItem.VehicleAction.NONE))

                mission_plan = MissionPlan(mission_items)

                # print(f"FollowMission: uploading mission for vehicle {getattr(self.vehicle,'_port','?')} ({len(mission_items)} items)")
                try:
                    await self.vehicle.mission.upload_mission(mission_plan)
                    try:
                        self.mark_mavsdk_mission_uploaded()
                    except Exception:
                        pass

                    # print(f"FollowMission: starting mission for vehicle {getattr(self.vehicle,'_port','?')}")
                    await self.vehicle.mission.start_mission()
                    # print(f"FollowMission: started mission for vehicle {getattr(self.vehicle,'_port','?')}")
                except Exception as e:
                    print(f"FollowMission: mission upload/start failed for vehicle {getattr(self.vehicle,'_port','?')}: {e}")

                await asyncio.sleep(1)
        except asyncio.CancelledError:
            # attempt to clear any uploaded mission and re-raise
            try:
                await self.vehicle.mission.clear_mission()
            except Exception as e:
                print(f"FollowMission: failed to clear mission during cancel for vehicle {getattr(self.vehicle,'_port','?')}: {e}")
            raise