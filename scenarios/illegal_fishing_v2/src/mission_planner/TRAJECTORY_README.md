# Waypoint Mission with Obstacle Avoidance

## Overview

This implementation provides a flexible framework for waypoint missions with obstacle avoidance. It separates the concerns of:

1. **Trajectory Generation**: Planning safe paths that avoid obstacles
2. **Progress Tracking**: Accurately reporting progress on actual waypoints (not intermittent points)
3. **Mission Execution**: Converting trajectories to MAVSDK missions

## Key Concepts

### Trajectory Points

Each point in a trajectory has:
- `point`: GPS coordinates (latitude, longitude)
- `original_waypoint_index`: 
  - `>= 0` for actual waypoints from the goal
  - `-1` for intermittent points added for obstacle avoidance

### Progress Tracking

The implementation maintains a mapping from mission item indices to original waypoint indices:
- `mission_items_`: All mission items sent to the autopilot (including intermittent points)
- `mission_item_to_waypoint_index_`: Maps each mission item to its original waypoint index

This allows accurate progress reporting:
- `current_waypoint_index`: Number of actual waypoints completed (excludes intermittent points)
- `total_waypoints`: Total number of actual waypoints in the goal
- `distance_to_current_waypoint_m`: Distance to the next actual waypoint

## Usage

### Basic Usage (No Obstacles)

The default implementation uses `SimpleTrajectoryGenerator` which passes waypoints through unchanged:

```cpp
// No changes needed - works out of the box
auto goal = mission_planner_msgs::action::Waypoints::Goal();
goal.waypoints = /* your waypoints */;
// Send goal to action server...
```

### With Obstacle Avoidance

To enable obstacle avoidance, you need to:

1. **Implement obstacle detection** in your `TrajectoryGenerator`
2. **Replace the trajectory generator** in `WaypointsActionHandler`

Example:

```cpp
// In WaypointsActionHandler constructor or initialization
auto obstacle_generator = std::make_shared<ObstacleAwareTrajectoryGenerator>();
obstacle_generator->load_obstacles("/path/to/obstacles.gpkg");
trajectory_generator_ = obstacle_generator;
```

## Extending for Real Obstacle Avoidance

The `ObstacleAwareTrajectoryGenerator` is a template that needs completion. Here's what you need to implement:

### 1. Obstacle Loading

```cpp
void load_obstacles(const std::string& obstacle_file)
{
    // Use a library like GDAL/OGR to read GeoPackage files
    // Example with GDAL:
    GDALDataset* dataset = GDALDataset::Open(obstacle_file.c_str(), GDAL_OF_VECTOR);
    OGRLayer* layer = dataset->GetLayer(0);
    
    for (auto& feature : layer)
    {
        OGRGeometry* geom = feature->GetGeometryRef();
        if (geom->getGeometryType() == wkbPolygon)
        {
            OGRPolygon* polygon = static_cast<OGRPolygon*>(geom);
            // Extract vertices and add to obstacles_
        }
    }
}
```

### 2. Collision Detection

```cpp
bool is_safe(const GeoPoint& start, const GeoPoint& end)
{
    // Create line segment
    // For each obstacle polygon:
    //   - Check if line intersects polygon boundary
    //   - Return false if intersection found
    
    // You can use libraries like:
    // - CGAL (Computational Geometry Algorithms Library)
    // - Boost.Geometry
    // - Custom implementation using line-segment intersection tests
}
```

### 3. Roadmap Generation (Voronoi Method - matching Python implementation)

```cpp
void generate_roadmap()
{
    // 1. Extract all obstacle vertices
    std::vector<GeoPoint> obstacle_points;
    for (const auto& obstacle : obstacles_)
    {
        obstacle_points.insert(obstacle_points.end(), 
                             obstacle.boundary.begin(), 
                             obstacle.boundary.end());
    }
    
    // 2. Compute Voronoi diagram
    // Use a library like CGAL or Boost.Polygon
    // Or implement Fortune's algorithm
    
    // 3. Extract Voronoi vertices as roadmap nodes
    roadmap_nodes_.clear();
    for (const auto& voronoi_vertex : voronoi_vertices)
    {
        RoadmapNode node;
        node.position = voronoi_vertex;
        roadmap_nodes_.push_back(node);
    }
    
    // 4. Connect nodes that don't intersect obstacles
    for (size_t i = 0; i < roadmap_nodes_.size(); ++i)
    {
        for (size_t j = i + 1; j < roadmap_nodes_.size(); ++j)
        {
            if (is_safe(roadmap_nodes_[i].position, roadmap_nodes_[j].position))
            {
                roadmap_nodes_[i].neighbors.push_back(j);
                roadmap_nodes_[j].neighbors.push_back(i);
            }
        }
    }
}
```

### 4. Path Planning (Dijkstra's Algorithm)

```cpp
std::vector<GeoPoint> dijkstra_search(const GeoPoint& start, const GeoPoint& end)
{
    // 1. Temporarily add start and end to roadmap
    std::vector<RoadmapNode> extended_roadmap = roadmap_nodes_;
    int start_idx = extended_roadmap.size();
    int end_idx = start_idx + 1;
    
    RoadmapNode start_node, end_node;
    start_node.position = start;
    end_node.position = end;
    
    // Connect start and end to nearby nodes
    for (size_t i = 0; i < roadmap_nodes_.size(); ++i)
    {
        if (is_safe(start, roadmap_nodes_[i].position))
        {
            start_node.neighbors.push_back(i);
        }
        if (is_safe(end, roadmap_nodes_[i].position))
        {
            end_node.neighbors.push_back(i);
        }
    }
    
    extended_roadmap.push_back(start_node);
    extended_roadmap.push_back(end_node);
    
    // 2. Dijkstra's algorithm
    std::vector<double> distances(extended_roadmap.size(), 
                                  std::numeric_limits<double>::infinity());
    std::vector<int> parents(extended_roadmap.size(), -1);
    distances[start_idx] = 0.0;
    
    auto compare = [&](int a, int b) { return distances[a] > distances[b]; };
    std::priority_queue<int, std::vector<int>, decltype(compare)> pq(compare);
    pq.push(start_idx);
    
    while (!pq.empty())
    {
        int current = pq.top();
        pq.pop();
        
        if (current == end_idx)
            break;
        
        for (int neighbor : extended_roadmap[current].neighbors)
        {
            double edge_cost = utils::haversine_distance(
                extended_roadmap[current].position.latitude_deg,
                extended_roadmap[current].position.longitude_deg,
                extended_roadmap[neighbor].position.latitude_deg,
                extended_roadmap[neighbor].position.longitude_deg
            );
            
            double new_dist = distances[current] + edge_cost;
            if (new_dist < distances[neighbor])
            {
                distances[neighbor] = new_dist;
                parents[neighbor] = current;
                pq.push(neighbor);
            }
        }
    }
    
    // 3. Reconstruct path
    std::vector<GeoPoint> path;
    int current = end_idx;
    while (current != -1)
    {
        path.push_back(extended_roadmap[current].position);
        current = parents[current];
    }
    std::reverse(path.begin(), path.end());
    
    return path;
}
```

## Dependencies for Full Implementation

To complete the obstacle avoidance implementation, you may need:

### Required Libraries

1. **GDAL/OGR** - For reading GeoPackage files
   ```bash
   sudo apt-get install libgdal-dev
   ```

2. **CGAL** - For computational geometry (Voronoi diagrams, polygon operations)
   ```bash
   sudo apt-get install libcgal-dev
   ```

   Or **Boost.Geometry** as an alternative:
   ```bash
   sudo apt-get install libboost-geometry-dev
   ```

### CMakeLists.txt Updates

```cmake
find_package(GDAL REQUIRED)
find_package(CGAL REQUIRED)

target_include_directories(vehicle PRIVATE ${GDAL_INCLUDE_DIRS})
target_link_libraries(vehicle ${GDAL_LIBRARIES} CGAL::CGAL)
```

## Testing

### Unit Tests

Create tests for:
1. Trajectory generation with known obstacles
2. Progress tracking with intermittent waypoints
3. Collision detection

### Integration Tests

1. Create a test scenario with simple obstacles
2. Send waypoint goals that would intersect obstacles
3. Verify that:
   - Trajectory avoids obstacles
   - Progress reports only actual waypoints
   - All waypoints are reached

## Example Scenario

```cpp
// 1. Load obstacles from GeoPackage
auto generator = std::make_shared<ObstacleAwareTrajectoryGenerator>();
generator->load_obstacles("/path/to/chungdo_obstacles.gpkg");

// 2. Use in waypoint action handler
// (This would be done in the constructor or via a setter)
waypoint_handler->set_trajectory_generator(generator);

// 3. Send waypoint goal
auto goal = mission_planner_msgs::action::Waypoints::Goal();

// Waypoints that might cross obstacles
goal.waypoints.push_back(create_waypoint(34.379685, 127.077422, 30.0, 20.0));
goal.waypoints.push_back(create_waypoint(34.380000, 127.078000, 30.0, 20.0));
goal.waypoints.push_back(create_waypoint(34.381000, 127.079000, 30.0, 20.0));

// 4. Monitor feedback
// feedback->current_waypoint_index will only count actual waypoints
// feedback->total_waypoints = 3 (not including intermittent points)
```

## Architecture Diagram

```
Goal Waypoints (3 points)
        ↓
TrajectoryGenerator::generate_trajectory()
        ↓
    [Check each segment]
        ↓
  Is path safe?
   /         \
 YES         NO
  ↓           ↓
Direct    plan_safe_path()
        (adds intermittent points)
        ↓
Trajectory (e.g., 7 points: 3 actual + 4 intermittent)
        ↓
WaypointsActionHandler::initialize_task()
        ↓
  [Create mission items]
  [Build waypoint index mapping]
        ↓
mission_items_ (8 items: 1 dummy + 7 trajectory)
mission_item_to_waypoint_index_ ([-1, 0, -1, -1, 1, -1, 2, -1])
        ↓
  Upload to autopilot
        ↓
  Monitor progress
        ↓
Feedback: current_waypoint_index (0→1→2→3)
          (counts only actual waypoints)
```

## Troubleshooting

### Progress reporting is incorrect
- Check `mission_item_to_waypoint_index_` is populated correctly
- Verify that intermittent points have index `-1`
- Ensure actual waypoints maintain their original indices

### Path doesn't avoid obstacles
- Verify `is_safe()` correctly detects collisions
- Check obstacle loading is working
- Debug `plan_safe_path()` to ensure it finds valid paths

### Vehicle doesn't follow intermittent waypoints
- Ensure intermittent points have `is_fly_through = true`
- Check altitude and speed are set appropriately
- Verify all mission items are uploaded to autopilot
