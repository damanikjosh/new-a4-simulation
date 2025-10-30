import asyncio
import numpy as np

class MissionBase:
    def __init__(self, vehicle, *args, autostart=True, **kwargs):
        self.vehicle = vehicle
        self.is_enabled = autostart

        self.is_initialized = False
        self.is_run = False

    # ===============================
    # To be implemented by subclasses
    # ===============================
    async def initialize(self, *args, **kwargs):
        self.is_initialized = True

    async def on_start(self, *args, **kwargs):
        pass

    async def before_start(self, *args, **kwargs):
        pass

    def get_coroutines(self):
        return []

    # ===============================

    def enable(self):
        self.is_enabled = True
        print(f"Mission enabled for vehicle {self.vehicle._port}")

    def disable(self, restart=True):
        self.is_run = restart
        self.is_enabled = False
        self.is_initialized = False
        print(f"Mission disabled for vehicle {self.vehicle._port}")

    async def run(self, *args, **kwargs):
        self.is_run = True

        while self.is_run:
            self.is_run = False
            while not self.is_enabled:
                await asyncio.sleep(1)

            if not self.is_initialized:
                await self.initialize()


            print(f"Waiting for vehicle {self.vehicle._port} to have a global position estimate...")
            async for health in self.vehicle.telemetry.health():
                if health.is_global_position_ok and health.is_home_position_ok:
                    print(f"-- Global position estimate OK for vehicle {self.vehicle._port}")
                    break

            await self.before_start(*args, **kwargs)
            
            print(f"-- Arming vehicle on {self.vehicle._port}")
            armed_ok = True
            try:
                await self.vehicle.action.arm()
            except Exception as e:
                # Some vehicles (USVs) may not support arming in the same way.
                # Don't attempt to start a mission if arming failed as the
                # flight controller will likely DENY mission start. Log and
                # disable the mission so we don't try to upload/start it.
                armed_ok = False
                print(f"-- Warning: arming failed for vehicle {self.vehicle._port}: {e}")

            if not armed_ok:
                print(f"-- Skipping mission start for vehicle {self.vehicle._port} because arming failed")
                self.is_enabled = False
                continue

            print(f"-- Starting mission for vehicle on vehicle {self.vehicle._port}")
            try:
                await self.on_start(*args, **kwargs)
            except Exception as e:
                # Don't allow mission plugin errors (e.g. DENIED start_mission)
                # to propagate and crash the orchestrator. Log and disable the
                # mission so the system can continue.
                print(f"-- Error while starting mission for vehicle {self.vehicle._port}: {e}")
                self.is_enabled = False
                # continue the loop which will wait for re-enable or exit
                continue
            
            # Monitor mission progress
            running_tasks = self.get_coroutines()

            # Wait for the mission to complete
            await self.observe_is_in_air(running_tasks)



    async def observe_is_in_air(self, running_tasks):
        """Monitors whether the vehicle is flying or not and
        returns after landing."""

        was_in_air = False

        async for is_in_air in self.vehicle.telemetry.in_air():
            if is_in_air:
                was_in_air = is_in_air

                if not self.is_enabled or (was_in_air and not is_in_air):
                    for task in running_tasks:
                        try:
                            task.cancel()
                        except Exception:
                            pass
                    for task in running_tasks:
                        try:
                            await task
                        except asyncio.CancelledError:
                            pass
                        except Exception as e:
                            print(f"-- Warning: task for vehicle {self.vehicle._port} raised during cancel/await: {e}")
                    print(f"-- Mission for vehicle {self.vehicle._port} has ended.")
                    return
            
    async def get_home_position(self):
        async for home_position in self.vehicle.telemetry.home():
            return np.array([home_position.longitude_deg, home_position.latitude_deg])
            
    async def get_position(self):
        async for position in self.vehicle.telemetry.position():
            return np.array([position.longitude_deg, position.latitude_deg])
