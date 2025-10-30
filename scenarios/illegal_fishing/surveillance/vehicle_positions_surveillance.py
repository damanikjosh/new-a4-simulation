import numpy as np
import asyncio

async def get_position(vehicle):
    async for position in vehicle.telemetry.position():
        return np.array([position.longitude_deg, position.latitude_deg])


class VehiclePositionSurveillance:
    def __init__(self, vehicles, period=1):
        if not isinstance(vehicles, list):
            vehicles = [vehicles]
        self.vehicles = vehicles
        self.period = period

        self.positions = [None] * len(vehicles)
        self.is_run = False

    async def run(self):
        self.is_run = True
        while True:
            for i, vehicle in enumerate(self.vehicles):
                try:
                    self.positions[i] = await asyncio.wait_for(get_position(vehicle), timeout=self.period)
                except asyncio.TimeoutError:
                    print(f"Timeout for vehicle {vehicle._port}. Skipping...")
                    continue
            await asyncio.sleep(self.period)

    def get_all(self):
        if not self.is_run:
            raise ValueError("Run the task first. Call run() method.")
        return self.vehicles, self.positions

    def get_on_radius(self, lonlat, radius):
        # Return all vehicle positions that are within the radius
        # The radius is in meters
        if not self.is_run:
            raise ValueError("Run the task first. Call run() method.")
        
        positions = np.array(self.positions)
        lon, lat = lonlat
        distances = np.linalg.norm(positions - np.array([lon, lat]), axis=1)
        # print("Distances: ", distances)
        indices = np.where(distances < radius)[0]
        return [self.vehicles[i] for i in indices], positions[indices].tolist()
    
    def get_by_port(self, port):
        if not self.is_run:
            raise ValueError("Run the task first. Call run() method.")
        
        for i, vehicle in enumerate(self.vehicles):
            if vehicle._port == port:
                return vehicle, self.positions[i]
        return None, None