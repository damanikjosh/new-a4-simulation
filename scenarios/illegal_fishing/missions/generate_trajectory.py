import numpy as np
import geopandas as gpd
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi, cKDTree
import shapely.geometry
from heapq import heappop, heappush

def get_chungdo_obstacles():
    # Load data/chungdo_obstacles.gpkg
    gdf = gpd.read_file("/home/joshua/Documents/a4_project/new-a4-simulation/scenarios/illegal_fishing/data/chungdo_obstacles.gpkg")
    # geometries = gdf.geometry
    # For each geometry, infl
    return gdf.geometry

def voronoi_sampling(start, end, obstacle_points):
    voronoi = Voronoi(obstacle_points)
    sample_points = voronoi.vertices
    sample_points = np.vstack([sample_points, start, end])

    return sample_points

def generate_roadmap(nodes, robot_radius, obstacles):
    roadmap = []
    n_sample = nodes.shape[0]
    node_tree = cKDTree(nodes)

    for i in range(n_sample):
        start = nodes[i]
        dists, indexes = node_tree.query(start, k=n_sample)

        edge_id = []

        for j in range(1, len(indexes)):
            neighbor = nodes[indexes[j]]
            if is_safe(start, neighbor, obstacles):
                edge_id.append(indexes[j])
            if len(edge_id) >= 5:  # Consider increasing or removing this limit
                break

        roadmap.append(edge_id)
        # print(f"Node {i} connected to: {edge_id}")  # Debug output

    return roadmap

def plan(start, end, obstacles):
    # Get vertices of the obstacles polygon
    obstacle_points = []
    for obstacle in obstacles:
        if isinstance(obstacle, shapely.geometry.Polygon):
            obstacle_points.extend(obstacle.exterior.coords[:-1])
        elif isinstance(obstacle, shapely.geometry.MultiPolygon):
            for polygon in obstacle:
                obstacle_points.extend(polygon.exterior.coords[:-1])

    obstacle_points = np.array(obstacle_points)[:,:2]
    # print(obstacle_points)

    sample_points = voronoi_sampling(start, end, obstacle_points)
    roadmap = generate_roadmap(sample_points, 0.5, obstacles)
    # for i, edges in enumerate(roadmap):
        # print(i, edges)
    # plot_roadmap(roadmap, sample_points, obstacles)

    # Convert start and end from coordinates to indices by finding the closest points in sample_points
    start_index = np.argmin(np.linalg.norm(sample_points - np.array(start), axis=1))
    end_index = np.argmin(np.linalg.norm(sample_points - np.array(end), axis=1))

    # Call dijkstra_search using indices
    path = dijkstra_search(start_index, end_index, roadmap, sample_points)
    return path

def dijkstra_search(start, end, roadmap, sample_points):
    costs = {i: float('inf') for i in range(len(sample_points))}
    parents = {i: None for i in range(len(sample_points))}
    costs[start] = 0

    priority_queue = []
    heappush(priority_queue, (0, start))  # (cost, node_index)
    visited = set()

    while priority_queue:
        current_cost, current_node = heappop(priority_queue)

        # print(f"Visiting Node: {current_node} with Cost: {current_cost}")  # Debug output

        if current_node == end:
            path = []
            while current_node is not None:
                path.append(sample_points[current_node])
                current_node = parents[current_node]
            # print("Path found:", path[::-1])  # Debug output
            return path[::-1]

        if current_node in visited:
            continue
        visited.add(current_node)

        for neighbor in roadmap[current_node]:
            if neighbor in visited:
                continue

            new_cost = current_cost + np.linalg.norm(np.array(sample_points[current_node]) - np.array(sample_points[neighbor]))
            # print(f"Checking neighbor {neighbor} from node {current_node} with new cost {new_cost}")  # Debug output

            if new_cost < costs[neighbor]:
                costs[neighbor] = new_cost
                parents[neighbor] = current_node
                heappush(priority_queue, (new_cost, neighbor))

    print("No path found")  # Debug output
    return None

def plot_roadmap(roadmap, sample_points, obstacles):
    for i, edges in enumerate(roadmap):
        if len(edges) == 0:
            continue
        print(i, edges)
        for j in range(len(roadmap[i])):
            ind = roadmap[i][j]
            print(i, ind)
            plt.plot([sample_points[i][0], sample_points[ind][0]],
                     [sample_points[i][1], sample_points[ind][1]], "-k")
            
    for obstacle in obstacles:
        if isinstance(obstacle, shapely.geometry.Polygon):
            x, y = obstacle.exterior.xy
            plt.plot(x, y, "-r")
        elif isinstance(obstacle, shapely.geometry.MultiPolygon):
            for polygon in obstacle:
                x, y = polygon.exterior.xy
                plt.plot(x, y, "-r")

    plt.show()

def is_safe(start, end, obstacles):
    line = shapely.geometry.LineString([start, end])

    for obstacle in obstacles:
        if line.intersects(obstacle):
            return False

    return True

def generate_trajectory(points, obstacles):
    safe_points = [points[0]]
    safe_points_indices = [0]

    for i in range(1, len(points)):
        start = tuple(points[i-1])
        end = tuple(points[i])

        if is_safe(start, end, obstacles):
            safe_points.append(end)
            safe_points_indices.append(i)
        else:
            print("Unsafe Path:", start, end)
            # Find a safe path from start to end
            safe_path = plan(start, end, obstacles)
            # print("Safe Path:", safe_path)
            safe_points.extend(safe_path[1:])
            safe_points_indices.extend([i] * (len(safe_path) - 1))
    
    # print("len(safe_points):", len(safe_points))
    # print("len(safe_points_indices):", len(safe_points_indices))
    return np.array(safe_points), np.array(safe_points_indices)

if __name__ == "__main__":
    obstacles = get_chungdo_obstacles()
    waypoints = [[127.077422,  34.379685]]
    safe_path, safe_path_indices = generate_trajectory(waypoints, obstacles)
    print("Safe Path Coordinates:", safe_path)
    print("Safe Path Indices:", safe_path_indices)
