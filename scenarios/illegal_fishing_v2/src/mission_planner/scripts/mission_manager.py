#!/usr/bin/env python3
"""
MissionManager - Centralized mission state management
Handles objective loading, task solving, and progress tracking
"""

import numpy as np
from typing import List, Tuple, Dict, Optional, Callable
from objectives import read_objectives
from task_solver import solve_task


class MissionManager:
    """
    Manages mission objectives, task solving, and progress tracking.
    Separates mission logic from ROS communication concerns.
    """
    
    def __init__(self, objectives_file: str, logger: Optional[Callable] = None):
        """
        Initialize the mission manager
        
        Args:
            objectives_file: Path to Excel file containing objectives
            logger: Optional logging function (e.g., node.get_logger().info)
        """
        self.objectives_file = objectives_file
        self.logger = logger or print
        
        # Objective data
        self.obj1_points: Optional[np.ndarray] = None
        self.obj1_reqs: Optional[np.ndarray] = None
        self.obj2_points: Optional[np.ndarray] = None
        self.obj3_points: Optional[np.ndarray] = None
        
        # Progress tracking (in-memory only)
        self.obj1_done: List[int] = []  # 0/1 bitmap of completed objectives
        
        # Per-UAV route tracking for progress updates
        self._uav_route_map: Dict[str, List[int]] = {}  # uav_name -> objective indices
        self._uav_route_snapshot: Dict[str, List[int]] = {}  # Snapshot during transitions
        self._uav_progress: Dict[str, int] = {}  # uav_name -> completed waypoint count
        
    def load_objectives(self) -> None:
        """Load objectives from Excel file and initialize progress tracking"""
        self.logger(f'Loading objectives from: {self.objectives_file}')
        
        # Read objectives
        self.obj1_points, self.obj2_points, self.obj3_points = read_objectives(
            self.objectives_file
        )
        
        # Initialize requirements matrix (no dependencies for now)
        self.obj1_reqs = np.zeros((len(self.obj1_points), len(self.obj1_points)))
        
        # Initialize progress tracking
        self.obj1_done = [0] * len(self.obj1_points)
        
        self.logger(f'Loaded {len(self.obj1_points)} obj1 objectives')
        
    def solve_for_uavs(
        self,
        uav_positions: List[Tuple[float, float]],
    ) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """
        Solve task assignment for given UAV positions
        
        Args:
            uav_positions: List of (lon, lat) positions for each UAV
            
        Returns:
            Tuple of (solution_points, solution_indices) where:
            - solution_points: List of coordinate arrays for each UAV route
            - solution_indices: List of objective index arrays for each UAV route
        """
        if self.obj1_points is None or self.obj1_reqs is None:
            raise RuntimeError("Objectives not loaded. Call load_objectives() first.")
        
        obj1_done_np = np.array(self.obj1_done, dtype=float)
        
        solution_points, solution_idx = solve_task(
            self.obj1_points,
            self.obj1_reqs,
            obj1_done_np,
            uav_positions,
            logger=self.logger,
        )
        
        return solution_points, solution_idx
    
    def assign_routes(
        self,
        uav_names: List[str],
        solution_indices: List[np.ndarray]
    ) -> None:
        """
        Assign routes to UAVs and reset their progress tracking
        
        Args:
            uav_names: List of UAV names
            solution_indices: List of objective index arrays (from solve_for_uavs)
        """
        self._uav_route_map.clear()
        for uav_name, indices in zip(uav_names, solution_indices):
            route_list = list(map(int, np.asarray(indices).tolist()))
            self._uav_route_map[uav_name] = route_list
            self._uav_progress[uav_name] = 0
            
        self.logger(f'Assigned routes to {len(uav_names)} UAVs')
    
    def update_progress(
        self,
        uav_name: str,
        current_waypoint: int
    ) -> Tuple[int, List[int]]:
        """
        Update progress for a UAV based on its current waypoint index
        Uses delta-based marking to avoid duplicate updates
        
        Args:
            uav_name: Name of the UAV
            current_waypoint: Current waypoint index (1-based from action feedback)
            
        Returns:
            Tuple of (newly_marked_count, newly_marked_indices)
        """
        # Use snapshot if available (during replan), otherwise current map
        route = self._uav_route_snapshot.get(uav_name) or self._uav_route_map.get(uav_name)
        
        if not route:
            return 0, []
        
        # Calculate delta from last committed progress
        new_count = min(current_waypoint, len(route))
        prev_count = self._uav_progress.get(uav_name, 0)
        
        if new_count <= prev_count:
            return 0, []
        
        # Mark objectives for waypoints [prev_count : new_count]
        indices_to_mark = route[prev_count:new_count]
        newly_marked = self._mark_objectives_done(indices_to_mark)
        
        # Update committed progress
        self._uav_progress[uav_name] = new_count
        
        return newly_marked, indices_to_mark
    
    def finalize_uav_progress(
        self,
        uav_name: str,
        waypoints_completed: int
    ) -> int:
        """
        Finalize progress for a UAV when its mission completes
        
        Args:
            uav_name: Name of the UAV
            waypoints_completed: Total waypoints completed
            
        Returns:
            Number of newly marked objectives
        """
        route = self._uav_route_snapshot.get(uav_name) or self._uav_route_map.get(uav_name)
        
        if not route:
            return 0
        
        prev_count = self._uav_progress.get(uav_name, 0)
        final_count = max(0, min(waypoints_completed, len(route)))
        
        if final_count <= prev_count:
            return 0
        
        indices_to_mark = route[prev_count:final_count]
        newly_marked = self._mark_objectives_done(indices_to_mark)
        self._uav_progress[uav_name] = final_count
        
        return newly_marked
    
    def prepare_for_replan(self) -> None:
        """
        Prepare for replanning by creating snapshot and persisting committed progress
        Call this BEFORE solving/assigning new routes
        """
        # Create snapshot of current routes for any delayed feedback
        self._uav_route_snapshot = self._uav_route_map.copy()
        self.logger(f'[REPLAN] Created snapshot with {len(self._uav_route_snapshot)} UAV routes')
        
        # Persist any committed progress from current routes
        for uav_name, route in list(self._uav_route_map.items()):
            progress = self._uav_progress.get(uav_name, 0)
            if progress > 0 and progress <= len(route):
                newly_marked = self._mark_objectives_done(route[:progress])
                if newly_marked > 0:
                    self.logger(
                        f'[REPLAN] Persisted {newly_marked} objectives from {uav_name} '
                        f'(was at {progress}/{len(route)})'
                    )
    
    def clear_snapshot(self) -> None:
        """Clear the route snapshot after replan transition completes"""
        self._uav_route_snapshot.clear()
        self.logger('[REPLAN] Cleared route snapshot')
    
    def get_route_for_uav(self, uav_name: str) -> Optional[List[int]]:
        """Get the assigned route (objective indices) for a UAV"""
        return self._uav_route_map.get(uav_name)
    
    def get_progress_summary(self) -> Tuple[int, int]:
        """
        Get overall progress summary
        
        Returns:
            Tuple of (completed_count, total_count)
        """
        completed = int(sum(self.obj1_done))
        total = len(self.obj1_done)
        return completed, total
    
    def _mark_objectives_done(self, indices: List[int]) -> int:
        """
        Mark objectives as done (internal helper)
        
        Args:
            indices: List of objective indices to mark
            
        Returns:
            Number of newly marked objectives (skips already-done)
        """
        newly_marked = 0
        for idx in indices:
            if 0 <= idx < len(self.obj1_done) and self.obj1_done[idx] == 0:
                self.obj1_done[idx] = 1
                newly_marked += 1
        return newly_marked
