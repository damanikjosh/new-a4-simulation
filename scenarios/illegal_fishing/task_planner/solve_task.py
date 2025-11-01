import os
import numpy as np
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment
import shapely
from sklearn.cluster import KMeans
import mip
import matplotlib.pyplot as plt
from matplotlib import cm

# Simple toggle to enable/disable plotting at runtime (non-blocking)
# Set env var A4_PLOT=1 to enable in deployment without code changes.
PLOT_DEBUG = os.getenv("A4_PLOT", "0") == "1"
PLOT_SAVE = os.getenv("A4_PLOT_SAVE", "0") == "1"

def set_plot_enabled(enabled: bool):
    global PLOT_DEBUG
    PLOT_DEBUG = bool(enabled)

def _nb_show(fig=None):
    """Non-blocking show for matplotlib."""
    if fig is None:
        fig = plt.gcf()
    try:
        fig.tight_layout()
    except Exception:
        pass
    fig.canvas.draw_idle()
    plt.show(block=False)
    # Small pause allows UI to refresh without blocking the main loop
    plt.pause(0.01)
    try:
        fig.canvas.flush_events()
    except Exception:
        pass
    if PLOT_SAVE:
        # Save last figure snapshot for headless debugging
        out = os.getenv("A4_PLOT_SAVE_PATH", "/tmp/a4_plan_overview.png")
        try:
            fig.savefig(out, dpi=120)
        except Exception:
            pass
 

def solve_ovrp(task_points, start_points, end_points=None):
    num_tasks = len(task_points)
    num_vehicles = len(start_points)

    assert num_vehicles <= num_tasks, 'Number of vehicles should be less than or equal to number of tasks'

    dist_tasks = cdist(task_points, task_points, 'euclidean')
    dist_start = cdist(start_points, task_points, 'euclidean')
    if end_points is not None:
        dist_end = cdist(end_points, task_points, 'euclidean')

    task_set = set(range(num_tasks))
    vehicles_set = set(range(num_vehicles))
    
    model = mip.Model()
    model.verbose = False

    assign_task = [[model.add_var(var_type=mip.BINARY) for j in task_set] for i in task_set]
    assign_vehicle_start = [[model.add_var(var_type=mip.BINARY) for j in task_set] for i in vehicles_set]
    assign_vehicle_end = [[model.add_var(var_type=mip.BINARY) for j in task_set] for i in vehicles_set]

    objective_func = mip.xsum(dist_tasks[i][j] * assign_task[i][j] for i in task_set for j in task_set) + mip.xsum(dist_start[i][j] * assign_vehicle_start[i][j] for i in vehicles_set for j in task_set)
    if end_points is not None:
        objective_func += mip.xsum(dist_end[i][j] * assign_vehicle_end[i][j] for i in vehicles_set for j in task_set)

    model.objective = mip.minimize(objective_func)
    
    # Constraint 1: Leave task only once
    for i in task_set:
        model += mip.xsum(assign_task[i][j] for j in task_set - {i}) + mip.xsum(assign_vehicle_end[k][i] for k in vehicles_set) == 1
    
    # Constraint 2: Visit task only once
    for j in task_set:
        model += mip.xsum(assign_task[i][j] for i in task_set - {j}) + mip.xsum(assign_vehicle_start[k][j] for k in vehicles_set) == 1 

    # Constraint 3: Vehicle can only visit its first task once
    for k in vehicles_set:
        model += mip.xsum(assign_vehicle_start[k][j] for j in task_set) == 1
        model += mip.xsum(assign_vehicle_end[k][i] for i in task_set) == 1
        model += mip.xsum(assign_vehicle_start[k][j] for j in task_set) == mip.xsum(assign_vehicle_end[k][i] for i in task_set)

    while True:
        model.optimize()

        if not model.num_solutions:
            raise Exception('No solution found')

        print('Objective value:', model.objective_value)

        subtours = detect_subtours(assign_task, num_tasks)
        if not subtours:
            break
        
        for subtour in subtours:
            if len(subtour) < num_tasks + 2:
                model += mip.xsum(assign_task[i][j] for i in subtour for j in subtour if i != j) <= len(subtour) - 1

        print('Subtours:', subtours)

    solution = []
    for k in vehicles_set:
        solution_k = []
        last_task = None
        for i in task_set:
            if assign_vehicle_start[k][i].x > 0.5:
                last_task = i
                break
            
        if last_task is not None:
            solution_k.append(last_task)

            # print('Vehicle', k, ': Start ->', last_task, end=' ')
            while True:
                for j in task_set:
                    if assign_task[last_task][j].x > 0.5:
                        # print('->', j, end=' ')
                        last_task = j
                        solution_k.append(j)
                        break
                else:
                    break
        
        solution.append(solution_k)

        # print('-> End')

    return solution

def solve_otsp(task_points, start_point, end_point=None):
    # Calculate distances between tasks and from the vehicle to the tasks
    dist_tasks = cdist(task_points, task_points, 'euclidean')
    dist_start = cdist([start_point], task_points, 'euclidean')
    dist_start = dist_start.squeeze()
    if end_point is not None:
        dist_end = cdist([end_point], task_points, 'euclidean')
        dist_end = dist_end.squeeze()
    

    num_tasks = len(task_points)
    task_set = set(range(num_tasks))


    model = mip.Model()
    model.verbose = False

    # # Create binary variables for tasks
    assign_task = [[model.add_var(var_type=mip.BINARY) for j in task_set] for i in task_set]
    assign_vehicle_start = [model.add_var(var_type=mip.BINARY) for i in task_set]
    assign_vehicle_end = [model.add_var(var_type=mip.BINARY) for i in task_set]

    objective_func = mip.xsum(dist_tasks[i][j] * assign_task[i][j] for i in task_set for j in task_set) + mip.xsum(dist_start[j] * assign_vehicle_start[j] for j in task_set)
    if end_point is not None:
        objective_func += mip.xsum(dist_end[j] * assign_vehicle_end[j] for j in task_set)

    # Objective: minimize the total distance traveled
    model.objective = mip.minimize(objective_func)

    # Constraints
    # Each task must be left exactly once
    for i in range(num_tasks):
        model += mip.xsum(assign_task[i][j] for j in task_set - {i}) + assign_vehicle_end[i] == 1

    # Each task must be visited exactly once
    for j in range(num_tasks):
        model += mip.xsum(assign_task[i][j] for i in task_set - {j}) + assign_vehicle_start[j] == 1

    # Connect start point to the route
    model += mip.xsum(assign_vehicle_start[j] for j in task_set) == 1
    model += mip.xsum(assign_vehicle_end[i] for i in task_set) == 1

    # Solve the model
    while True:
        model.optimize()

        if not model.num_solutions:
            raise Exception('No solution found')

        # print('Objective value:', model.objective_value)

        subtours = detect_subtours(assign_task, num_tasks)
        if not subtours:
            break
        
        for subtour in subtours:
            if len(subtour) < num_tasks + 2:
                model += mip.xsum(assign_task[i][j] for i in subtour for j in subtour if i != j) <= len(subtour) - 1

        print('Subtours:', subtours)

    # Extract the solution
    solution = []
    last_task = None
    for i in task_set:
        if assign_vehicle_start[i].x > 0.5:
            last_task = i
            break

    solution.append(last_task)

    # print('Start ->', last_task, end=' ')
    while True:
        for j in task_set:
            if assign_task[last_task][j].x > 0.5:
                # print('->', j, end=' ')
                last_task = j
                solution.append(j)
                break
        else:
            break
    
    # print('-> End')

    return solution


def solve_cluster_tsp(task_points, start_points, end_points=None):
    # Cluster tasks to number of vehicles
    kmeans = KMeans(n_clusters=len(start_points), random_state=0).fit(task_points)
    task_clusters = kmeans.labels_
    task_centers = kmeans.cluster_centers_

    tasks = np.arange(len(task_points))

    # Assign tasks to vehicles using linear sum assignment
    dist = cdist(task_centers, start_points, 'euclidean')
    row_ind, col_ind = linear_sum_assignment(dist)
    
    # Solve TSP for each cluster
    solution = [[]] * len(start_points)

    for i, k in zip(col_ind, row_ind):
        # print('Vehicle', k, '-> Cluster', i)
        cluster_tasks = tasks[task_clusters == i]
        cluster_points = task_points[task_clusters == i]
        cluster_solution = solve_otsp(cluster_points, start_points[k], end_points[k] if end_points is not None else None)
        solution[k] = cluster_tasks[cluster_solution].tolist()
    return solution


def solve_hierarchical_ovrp(num_clusters, task_points, start_points, end_points=None):
    # Cluster tasks to number of vehicles
    kmeans = KMeans(n_clusters=num_clusters, random_state=0).fit(task_points)
    task_clusters = kmeans.labels_
    task_centers = kmeans.cluster_centers_

    # One-figure composite debug plot will be shown after routing

    tasks = np.arange(len(task_points))

    cluster_routes = solve_ovrp(task_centers, start_points, end_points)
    print('Cluster routes:', cluster_routes)
    solution = []
    for k, cluster_route in enumerate(cluster_routes):
        last_vehicle_point = start_points[k].copy()
        solution_k = []
        for i, cluster_i in enumerate(cluster_route):
            cluster_tasks = tasks[task_clusters == cluster_i]
            cluster_points = task_points[task_clusters == cluster_i]

            if len(cluster_tasks) == 1:
                solution_k.extend(cluster_tasks.tolist())
                continue
            
            if i < len(cluster_route) - 1:
                next_cluster_i = cluster_route[i + 1]
                subroute = solve_otsp(cluster_points, last_vehicle_point, task_centers[next_cluster_i])
            else:
                subroute = solve_otsp(cluster_points, last_vehicle_point)

            solution_k.extend(cluster_tasks[subroute].tolist())

            last_vehicle_point = cluster_points[subroute[-1]]

        solution.append(solution_k)
    
    # One-figure composite plotting: clusters + cluster routes + task routes
    debug_plot_all(task_points, task_clusters, task_centers, cluster_routes, solution,
                   start_points=start_points, end_points=end_points, title="Plan overview (clusters + routes)")
    return solution


def detect_subtours(assign_task, num_tasks):
    from collections import defaultdict

    # Create adjacency list from the decision variables
    graph = defaultdict(list)
    for i in range(num_tasks):
        for j in range(num_tasks):
            if i != j and assign_task[i][j].x > 0.5:  # Checking if the edge is part of the solution
                graph[i].append(j)

    visited = set()
    subtours = []
    
    # Helper function to perform DFS and detect cycles
    def dfs(node, path, start_node):
        stack = [(node, [node])]  # Start DFS with the node and its path
        while stack:
            current, path = stack.pop()
            for neighbor in graph[current]:
                if neighbor == start_node:  # Cycle back to start node detected
                    path.append(neighbor)
                    return path
                if neighbor not in path:  # Continue DFS if not yet visited in current path
                    stack.append((neighbor, path + [neighbor]))
        return None
    
    # Check all nodes for potential subtours starting from each node
    for node in range(num_tasks):
        if node not in visited:
            subtour = dfs(node, [], node)
            if subtour:
                subtours.append(subtour)
                visited.update(subtour)

    return subtours 

def plot_solution(task_points, vehicle_points, solution):
    plt.figure()
    plt.scatter(task_points[:, 0], task_points[:, 1], c='b', label='Tasks')
    plt.scatter(vehicle_points[:, 0], vehicle_points[:, 1], c='r', label='Vehicles')
    for k, route in enumerate(solution):
        route_points = task_points[route]
        plt.plot(route_points[:, 0], route_points[:, 1], '->', label=f'Vehicle {k}')
    plt.legend()
    _nb_show()

def plot_trajectories(trajectories):
    plt.figure()
    for trajectory in trajectories:
        trajectory = np.array(trajectory)
        plt.plot(trajectory[:, 0], trajectory[:, 1], '->')
    
    # Plot obstacles
    for obstacle in obstacles:
        if isinstance(obstacle, shapely.geometry.Polygon):
            x, y = obstacle.exterior.xy
            plt.plot(x, y, "-r")
        elif isinstance(obstacle, shapely.geometry.MultiPolygon):
            for polygon in obstacle:
                x, y = polygon.exterior.xy
                plt.plot(x, y, "-r")
    _nb_show()

def debug_plot_clusters(task_points, labels, medoid_indices, start_points=None, end_points=None, title="Clusters"):
    if not PLOT_DEBUG:
        return
    task_points = np.asarray(task_points)
    labels = np.asarray(labels)
    fig, ax = plt.subplots(figsize=(7, 6))
    cmap = cm.get_cmap('tab20')
    unique_labels = np.unique(labels)
    for lab in unique_labels:
        pts = task_points[labels == lab]
        if len(pts) == 0:
            continue
        color = cmap(int(lab) % 20)
        ax.scatter(pts[:, 0], pts[:, 1], s=15, color=color, label=f'C{lab}', alpha=0.7)
    # Highlight medoids
    if medoid_indices is not None and len(medoid_indices) > 0:
        med = task_points[np.asarray(medoid_indices, dtype=int)]
        ax.scatter(med[:, 0], med[:, 1], s=80, marker='*', edgecolors='k', facecolors='gold', label='Medoids')
    # Start/end points
    if start_points is not None:
        sp = np.asarray(start_points)
        ax.scatter(sp[:, 0], sp[:, 1], s=60, marker='^', color='red', label='Starts')
    if end_points is not None:
        ep = np.asarray(end_points)
        ax.scatter(ep[:, 0], ep[:, 1], s=60, marker='v', color='green', label='Ends')
    ax.set_title(title)
    ax.legend(loc='best', fontsize=8)
    ax.set_xlabel('Lon')
    ax.set_ylabel('Lat')
    _nb_show(fig)

def debug_plot_routes(task_points, solutions, start_points=None, end_points=None, title="Routes"):
    if not PLOT_DEBUG:
        return
    task_points = np.asarray(task_points)
    fig, ax = plt.subplots(figsize=(7, 6))
    # Plot all tasks in light color
    ax.scatter(task_points[:, 0], task_points[:, 1], s=10, color='#bbbbbb', label='Tasks')
    colors = cm.get_cmap('tab10')
    for k, route in enumerate(solutions):
        if route is None or len(route) == 0:
            continue
        pts = task_points[route]
        ax.plot(pts[:, 0], pts[:, 1], '-o', ms=3, color=colors(k % 10), label=f'Veh {k}')
    if start_points is not None:
        sp = np.asarray(start_points)
        ax.scatter(sp[:, 0], sp[:, 1], s=60, marker='^', color='red', label='Starts')
    if end_points is not None:
        ep = np.asarray(end_points)
        ax.scatter(ep[:, 0], ep[:, 1], s=60, marker='v', color='green', label='Ends')
    ax.set_title(title)
    ax.legend(loc='best', fontsize=8)
    ax.set_xlabel('Lon')
    ax.set_ylabel('Lat')
    _nb_show(fig)

def debug_plot_cluster_routes(cluster_points, cluster_routes, start_points=None, end_points=None, title="Cluster routes"):
    if not PLOT_DEBUG:
        return
    cps = np.asarray(cluster_points)
    fig, ax = plt.subplots(figsize=(7, 6))
    # Plot cluster centers/medoids
    ax.scatter(cps[:, 0], cps[:, 1], s=30, color='tab:blue', label='Clusters')
    colors = cm.get_cmap('tab10')
    for k, route in enumerate(cluster_routes):
        if route is None or len(route) == 0:
            continue
        pts = cps[route]
        ax.plot(pts[:, 0], pts[:, 1], '-o', ms=3, color=colors(k % 10), label=f'Veh Cseq {k}')
        # Draw leg from start to first cluster center for context
        if start_points is not None:
            sp = np.asarray(start_points)
            ax.plot([sp[k, 0], pts[0, 0]], [sp[k, 1], pts[0, 1]], '--', color=colors(k % 10), alpha=0.6)
    if start_points is not None:
        sp = np.asarray(start_points)
        ax.scatter(sp[:, 0], sp[:, 1], s=60, marker='^', color='red', label='Starts')
    if end_points is not None:
        ep = np.asarray(end_points)
        ax.scatter(ep[:, 0], ep[:, 1], s=60, marker='v', color='green', label='Ends')
    ax.set_title(title)
    ax.legend(loc='best', fontsize=8)
    ax.set_xlabel('Lon')
    ax.set_ylabel('Lat')
    _nb_show(fig)

def debug_plot_all(task_points, labels, cluster_points, cluster_routes, solutions,
                   start_points=None, end_points=None, title="Plan overview"):
    if not PLOT_DEBUG:
        return
    tp = np.asarray(task_points)
    cps = np.asarray(cluster_points) if cluster_points is not None else None
    labels = np.asarray(labels) if labels is not None else None

    fig, ax = plt.subplots(figsize=(8, 7))
    colors = cm.get_cmap('tab10')
    cluster_cmap = cm.get_cmap('tab20')

    # Debug counts to stdout to help when figure appears empty
    try:
        print(f"[PLOT] tasks={len(tp)} clusters={0 if cps is None else len(cps)}",
              f"solutions={[0 if r is None else len(r) for r in (solutions or [])]}",
              f"cluster_routes={[0 if r is None else len(r) for r in (cluster_routes or [])]}")
    except Exception:
        pass

    # Plot tasks (colored by cluster label if provided)
    if labels is not None:
        unique_labels = np.unique(labels)
        for lab in unique_labels:
            pts = tp[labels == lab]
            if len(pts) == 0:
                continue
            ax.scatter(pts[:, 0], pts[:, 1], s=12, color=cluster_cmap(int(lab) % 20), alpha=0.65,
                       label=f'C{int(lab)}')
    else:
        if tp.size:
            ax.scatter(tp[:, 0], tp[:, 1], s=12, color='#bbbbbb', label='Tasks')

    # Plot cluster points (centers/medoids)
    if cps is not None and len(cps) > 0:
        ax.scatter(cps[:, 0], cps[:, 1], s=70, marker='s', edgecolors='k', facecolors='gold',
                   linewidths=0.7, label='Clusters')

    # Plot per-vehicle cluster routes on top (dashed)
    if cluster_routes is not None:
        for k, route in enumerate(cluster_routes):
            if route is None or len(route) == 0:
                continue
            if cps is None:
                continue
            pts = cps[route]
            ax.plot(pts[:, 0], pts[:, 1], '--o', ms=3, color=colors(k % 10), alpha=0.7,
                    label=f'Veh {k} clusters')
            # Draw leg from start to first cluster center for context
            if start_points is not None:
                sp = np.asarray(start_points)
                ax.plot([sp[k, 0], pts[0, 0]], [sp[k, 1], pts[0, 1]], ':', color=colors(k % 10), alpha=0.6)

    # Plot per-vehicle task routes (solid)
    if solutions is not None:
        for k, route in enumerate(solutions):
            if route is None or len(route) == 0:
                continue
            pts = tp[route]
            ax.plot(pts[:, 0], pts[:, 1], '-o', ms=3, color=colors(k % 10), label=f'Veh {k} tasks')

    # Start/end points
    if start_points is not None:
        sp = np.asarray(start_points)
        ax.scatter(sp[:, 0], sp[:, 1], s=70, marker='^', color='red', label='Starts')
    if end_points is not None:
        ep = np.asarray(end_points)
        ax.scatter(ep[:, 0], ep[:, 1], s=70, marker='v', color='green', label='Ends')

    ax.set_title(title)
    ax.legend(loc='best', fontsize=8, ncol=2)
    ax.set_xlabel('Lon')
    ax.set_ylabel('Lat')
    try:
        ax.set_aspect('equal', adjustable='datalim')
    except Exception:
        pass
    _nb_show(fig)

def solve_task(task_points, task_reqs, task_done, vehicle_points):
    assert task_reqs.shape[0] == len(task_points), "Number of points should match number of requirements"
    assert task_reqs.shape[1] == len(task_points), "Number of points should match number of requirements"
    assert len(task_done) == len(task_points), "Number of points should match number of done"

    task_points = np.array(task_points)
    vehicle_points = np.array(vehicle_points)

    num_vehicle = len(vehicle_points)

    # Get index of points that are not done
    task_idx = np.arange(len(task_points))
    task_avail = [i for i in range(len(task_points)) if not task_done[i]]
    task_done = tuple(i for i in range(len(task_points)) if task_done[i])

    # Remove tasks that does not satisfy requirements
    task_avail = [i for i in task_avail if np.isin(task_idx[task_reqs[i] == 1], task_done).all()]

    # Algorithm 1: Open Vehicle Routing Problem
    # solution = solve_ovrp(task_points[task_avail], vehicle_points, vehicle_points)

    # Algorithm 2: Clustered Traveling Salesman Problem
    # solution = solve_cluster_tsp(task_points[task_avail], vehicle_points)

    # Algorithm 3: Hierarchical Open Vehicle Routing Problem
    solution = solve_hierarchical_ovrp(num_vehicle, task_points[task_avail], vehicle_points, vehicle_points)

    # IMPORTANT: routes are indices into the sliced array task_points[task_avail]
    # so map points from that same sliced view to keep indices and coordinates aligned
    solution_points = [task_points[task_avail][route] for route in solution]
    solution_idx = [np.array(task_avail)[solution_k] for solution_k in solution]
    print('Solution:')
    for k, solution_k in enumerate(solution_idx):
        print('Vehicle', k, ':', solution_k)

    return solution_points, solution_idx


if __name__ == '__main__':
    from read_objectives import read_objectives
    from process_task import process_task
    from scenarios.illegal_fishing.missions.generate_trajectory import generate_trajectory, get_chungdo_obstacles

    vehicle_points = np.array([[127.079049, 34.376794], [127.079049, 34.376794]])
    obstacles = get_chungdo_obstacles()

    obj1_points, obj2_points, obj3_points = read_objectives('data/T_Objectives_latlon.xlsx')
    points, reqs, done = process_task(obj1_points, obj2_points, obj3_points)
    print('Requirements:')
    print(np.argwhere(reqs))
    points = solve_task(points, reqs, done, vehicle_points)

    safe_points = []
    for k, points_k in enumerate(points):
        # print('Vehicle', k, ':', points_k)
        safe_points.append(generate_trajectory(np.concatenate([vehicle_points[k][None], points_k, vehicle_points[k][None]]), obstacles))

    # print(safe_points)
    # plot_trajectories(safe_points)

    