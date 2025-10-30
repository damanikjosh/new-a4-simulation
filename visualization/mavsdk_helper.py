import asyncio
from mavsdk import System

async def initialize_vehicle(port):
    vehicle = System(port=port)
    vehicle.port = port
    try:
        await asyncio.wait_for(vehicle.connect(system_address=f"udp://:{port}"), timeout=3)
    except asyncio.TimeoutError:
        print(f"Failed to connect to vehicle on {port}")
        return None

    print("Waiting for vehicle to connect...")
    async for state in vehicle.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to vehicle on {port}!")
            break

    return vehicle

async def is_healty(vehicle):
    async def health():
        async for health in vehicle.telemetry.health():
            return health
    try:
        await asyncio.wait_for(health(), timeout=1)
        return True
    except asyncio.TimeoutError:
        return False

async def get_position(vehicle):
    async def position():
        async for position in vehicle.telemetry.position():
            return {
                "latitude": position.latitude_deg,
                "longitude": position.longitude_deg,
                "altitude": position.relative_altitude_m
            }
    try:
        return await asyncio.wait_for(position(), timeout=1)
    except asyncio.TimeoutError:
        return None

async def get_mission(vehicle):
    mission = await vehicle.mission.download_mission()
    waypoints = []
    for item in mission.mission_items:
        waypoints.append({
            "latitude": item.latitude_deg,
            "longitude": item.longitude_deg,
            "altitude": item.relative_altitude_m
        })
    return waypoints