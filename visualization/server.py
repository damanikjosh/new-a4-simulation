import asyncio
from mavsdk_helper import initialize_vehicle, get_position, get_mission
from quart import Quart, abort, request
import json

vehicles = dict()
positions = dict()

app = Quart(__name__, static_url_path='', static_folder='static')

mavsdk_ready = True

lock = asyncio.Lock()

async def enable_vehicle(port):
    vehicle = await initialize_vehicle(port)
    if vehicle is None:
        # vehicles[port] = None
        print(f"Failed to connect to vehicle on {port}")
        return False
    vehicles[port] = vehicle
    print(f"Connected to vehicle on {port}")
    return True

async def enable_all_vehicles():
    tasks = []
    for port in range(14540, 14550):
        tasks.append(enable_vehicle(port))
    await asyncio.gather(*tasks)

async def get_positions():
    tasks = []
    ports = []
    for port, vehicle in vehicles.items():
        tasks.append(get_position(vehicle))
        ports.append(port)
    results = await asyncio.gather(*tasks)
    positions = dict()
    for i, result in enumerate(results):
        if result is not None:
            positions[ports[i]] = result

    return positions
        

@app.route('/')
async def index():
    return await app.send_static_file('index.html')

@app.route('/position/<int:port>', methods=['GET'])
async def get_vehicle_positions(port):
    position = await get_position(vehicles[port])
    return json.dumps(position)

@app.route('/mission/<int:port>', methods=['GET'])
async def get_vehicle_mission(port):
    mission = await get_mission(vehicles[port])
    return json.dumps(mission)


@app.route('/enable/<int:port>', methods=['POST'])
async def enable_vehicle_route(port):
    if port in vehicles:
        if vehicles[port] is not None:
            return '', 204
        else:
            return abort(400, "Vehicle is not connected")
    status = await enable_vehicle(port)
    if not status:
        return abort(400, "Failed to connect to vehicle")
    return '', 204

@app.route('/disable/<int:port>', methods=['POST'])
async def disable_vehicle_route(port):
    if port in vehicles:
        del vehicles[port]
        print(f"Disconnected from vehicle on {port}")
    return '', 204


if __name__ == "__main__":
    print("Open http://127.0.0.1:5000 in your browser to see the visualization")
    tasks = []
    tasks.append(app.run_task(debug=False))
    tasks.append(enable_all_vehicles())
    asyncio.get_event_loop().run_until_complete(asyncio.gather(*tasks))
