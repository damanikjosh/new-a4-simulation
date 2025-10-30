#!/usr/bin/env python3

import os
import rclpy
import numpy as np
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from mission_planner_msgs.msg import Heartbeat, Position, Waypoint

from vehicle import Vehicle, VehicleType, VehicleDirectory, VehicleStatus
from mission_manager import MissionManager

base_path = get_package_share_directory('mission_planner')

class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')
        self.heartbeat_sub = self.create_subscription(
            Heartbeat,
            '/mission_planner/heartbeat',
            self.heartbeat_callback,
            10
        )
        
        # Timer to periodically print status (1 Hz)
        self.timer = self.create_timer(1.0, self.status_callback)

        # Vehicle directory for tracking and querying vehicles
        self.vehicles = VehicleDirectory()

        # Mission manager handles all mission logic, objectives, and progress
        objectives_file = os.path.join(base_path, 'data', 'T_Objectives_latlon.xlsx')
        self.mission = MissionManager(objectives_file, logger=self.get_logger().info)

        self.started = False
        
        self.get_logger().info('Mission Planner node started')

    def heartbeat_callback(self, msg):
        """Update the last heartbeat time for the vehicle"""
        
        if self.vehicles.get(msg.vehicle_name) is None:
            position_topic = f'/{msg.vehicle_name}/position'
            vehicle = Vehicle(
                name=msg.vehicle_name,
                vtype=VehicleType(msg.vehicle_type),
                position=(0.0, 0.0, 0.0),
                subscription=self.create_subscription(
                    Position,
                    position_topic,
                    lambda msg, vn=msg.vehicle_name: self.position_callback(vn, msg),
                    10
                )
            )
            self.vehicles.add_vehicle(vehicle)
        
        self.vehicles.update_heartbeat(msg.vehicle_name, self.get_clock().now())
    
    def log_active_vehicles(self):
        """Log the list of currently active vehicles"""
        now = self.get_clock().now()
        self.get_logger().info('Active vehicles:')
        for vehicle in self.vehicles:
            if vehicle.last_heartbeat is not None:
                time_diff = (now - vehicle.last_heartbeat).nanoseconds / 1e9  # Convert to seconds
                self.get_logger().info(
                    f' - {vehicle.name} (Type: {vehicle.vtype.name}, Last heartbeat: {time_diff:.1f} seconds ago)'
                )
    
    def log_vehicle_positions(self):
        """Log the current positions of all active vehicles"""
        self.get_logger().info('Vehicle positions:')
        for vehicle in self.vehicles:
            lat, lon, alt = vehicle.position
            self.get_logger().info(
                f' - {vehicle.name} (Type: {vehicle.vtype.name}): Lat: {lat:.6f}, Lon: {lon:.6f}, Alt: {alt:.1f} m'
            )

    def status_callback(self):
        """Periodically check and print active vehicles"""
        now = self.get_clock().now()
        
        # Remove vehicles that haven't sent heartbeat in the last 10 seconds
        vehicles_to_remove = []
        for vehicle in list(self.vehicles):
            if vehicle.last_heartbeat is not None:
                time_diff = (now - vehicle.last_heartbeat).nanoseconds / 1e9  # Convert to seconds

                if time_diff > 30.0:
                    self.get_logger().warn(
                        f'Vehicle {vehicle.name} considered inactive, removing from list'
                    )
                    vehicles_to_remove.append(vehicle.name)

        # Remove inactive vehicles
        for vehicle_name in vehicles_to_remove:
            self.vehicles.remove(vehicle_name)

        if not self.started:
            if len(self.vehicles) >= 7:
                # self.get_logger().info('All vehicles active. Starting mission planning tasks.')
                self.started = True
                self.start()
            else:
                if len(self.vehicles) > 0:
                    self.log_active_vehicles()
                else:
                    self.get_logger().info('No active vehicles')
        else:
            if len(self.vehicles) < 7:
                self.get_logger().warn('One or more vehicles inactive. Pausing mission planning tasks.')
                self.started = False
            else:
                # self.log_vehicle_positions()
                pass
            
        # If there are 7 active vehicles and not started yet, log start message
    
    def position_callback(self, vehicle_name, msg):
        """Update the last known position for the vehicle"""
        self.vehicles.update_position(
            vehicle_name,
            (
                msg.latitude_deg,
                msg.longitude_deg,
                msg.absolute_altitude_m,
            ),
        )

    def start(self):
        """Start mission planning tasks"""
        self.get_logger().info('Starting mission planning tasks')
        
        # Load objectives and initialize progress tracking
        self.mission.load_objectives()
        
        # Get available UAVs and their positions
        obj1_vehicles = self.vehicles.filter(vtype=VehicleType.UAV, status=VehicleStatus.IDLE)
        obj1_vehicle_positions = [vehicle.position[:2] for vehicle in obj1_vehicles]

        # Enemies to search for (targets of the search)
        enemies_idle = self.vehicles.filter(vtype=VehicleType.ENEMY, status=VehicleStatus.IDLE)
        
        # Solve task assignment
        obj1_solution_points, obj1_solution_idx = self.mission.solve_for_uavs(obj1_vehicle_positions)

        # Assign routes to UAVs for progress tracking
        uav_names = [uav.name for uav in obj1_vehicles]
        self.mission.assign_routes(uav_names, obj1_solution_idx)

        # Dispatch Search action goals to each idle UAV with its own trajectory
        if not obj1_vehicles:
            self.get_logger().warn('No idle UAV vehicles found to dispatch Search actions')
        elif not enemies_idle:
            self.get_logger().warn('No idle ENEMY vehicles to search for; skipping Search action dispatch')
        else:
            enemy_names = [e.name for e in enemies_idle]
            # Zip the UAV list with its corresponding route from the solver
            for uav, route_points in zip(obj1_vehicles, obj1_solution_points):
                waypoints = self._build_waypoints_from_solution([route_points], absolute_altitude_m=70.0, speed_m_s=15.0)
                if not waypoints:
                    self.get_logger().warn(f'No waypoints for {uav.name}; skipping Search action dispatch')
                    continue
                uav.start_search(
                    self,
                    waypoints=waypoints,
                    vehicle_names=enemy_names,
                    search_radius_m=100.0,
                    feedback_cb=self._on_search_feedback_vehicle,
                    result_cb=self._on_search_result_vehicle,
                )

        # Log initial progress snapshot
        completed, total = self.mission.get_progress_summary()
        self.get_logger().info(f'Obj1 progress: {completed}/{total} done at start')

    def _build_waypoints_from_solution(self, solution_points, absolute_altitude_m=0.0, speed_m_s=2.0):
        """Convert a list of arrays of [lat, lon] points into Waypoint messages.

        solution_points: List[np.ndarray] with shape (Ni, 2) for each route segment.
        Returns: List[Waypoint]
        """
        waypoints = []
        for segment in solution_points:
            # Ensure it's iterable even if provided as a Python list
            for pt in np.asarray(segment):
                if len(pt) < 2:
                    continue
                wp = Waypoint()
                # The convention in this project uses [lon, lat] or [lat, lon]? We assume [lat, lon]
                # as constructed from vehicle.position[:2] earlier.
                wp.latitude_deg = float(pt[1])
                wp.longitude_deg = float(pt[0])
                wp.absolute_altitude_m = float(absolute_altitude_m)
                wp.speed_m_s = float(speed_m_s)
                wp.is_fly_through = True
                wp.gimbal_pitch_deg = float(0.0)
                wp.gimbal_yaw_deg = float(0.0)
                wp.camera_action = Waypoint.CAMERA_ACTION_NONE
                wp.loiter_time_s = float(0.0)
                wp.camera_photo_interval_s = float(0.0)
                wp.acceptance_radius_m = float(2.0)
                wp.yaw_deg = float(0.0)
                wp.camera_photo_distance_m = float(0.0)
                waypoints.append(wp)
        return waypoints

    # Removed legacy per-node action client methods in favor of Vehicle-managed actions

    # --- Vehicle action callbacks (used by Vehicle.start_* to report back) ---
    def _on_search_feedback_vehicle(self, veh: Vehicle, feedback_msg):
        fb = feedback_msg.feedback
        # Log progress
        self.get_logger().info(
            f'[SEARCH:{veh.name}] Progress {fb.current_waypoint_index}/{fb.total_waypoints}, '
            f'dist {fb.distance_to_current_waypoint_m:.1f} m, in_range={list(fb.vehicles_in_range)}'
        )

        # Update progress using mission manager
        newly_marked, marked_indices = self.mission.update_progress(
            veh.name,
            fb.current_waypoint_index
        )
        
        if newly_marked > 0:
            completed, total = self.mission.get_progress_summary()
            self.get_logger().info(
                f'[SEARCH:{veh.name}] Progress commit +{newly_marked} '
                f'(marked obj indices {marked_indices}) -> {completed}/{total}'
            )

        # React to enemies in range only if they are idle
        for target_name in fb.vehicles_in_range:
            target = self.vehicles.get(target_name)
            if target is None or target.vtype != VehicleType.ENEMY:
                continue
            if target.status == VehicleStatus.IDLE:
                target.status = VehicleStatus.FOLLOWING_ENEMY
                self.get_logger().info(f'Enemy {target_name} found by {veh.name}. Switching UAV to FollowMe and canceling Search.')
                
                # Start followme on this UAV; Vehicle will cancel/destroy the search client first
                # Mark the UAV as not available for searching anymore
                veh.status = VehicleStatus.FOLLOWING_ENEMY
                veh.start_followme(
                    self,
                    target_enemy_name=target_name,
                    feedback_cb=self._on_follow_feedback_vehicle,
                    result_cb=self._on_follow_result_vehicle,
                )
                
                # Assign one IDLE USV to follow the same enemy
                idle_usvs = self.vehicles.filter(vtype=VehicleType.USV, status=VehicleStatus.IDLE)
                if idle_usvs:
                    usv = idle_usvs[0]  # Take the first available IDLE USV
                    usv.status = VehicleStatus.FOLLOWING_ENEMY
                    self.get_logger().info(f'Assigning USV {usv.name} to follow enemy {target_name}.')
                    usv.start_followme(
                        self,
                        target_enemy_name=target_name,
                        feedback_cb=self._on_follow_feedback_vehicle,
                        result_cb=self._on_follow_result_vehicle,
                    )
                else:
                    self.get_logger().warn(f'No IDLE USV available to follow enemy {target_name}.')
                
                # Replan remaining search tasks for available UAVs
                self._replan_search()
                break

    def _on_search_result_vehicle(self, veh: Vehicle, result):
        self.get_logger().info(
            f'[SEARCH:{veh.name}] Completed: success={result.success}, waypoints_completed={result.waypoints_completed}'
        )
        
        # Finalize progress based on waypoints_completed
        newly_marked = self.mission.finalize_uav_progress(veh.name, result.waypoints_completed)
        
        if newly_marked > 0:
            completed, total = self.mission.get_progress_summary()
            self.get_logger().info(
                f'[SEARCH:{veh.name}] Result commit +{newly_marked} -> {completed}/{total}'
            )
        
        # Replan when a search completes (vehicle becomes free or needs new route)
        self._replan_search()

    def _on_follow_feedback_vehicle(self, veh: Vehicle, target_enemy_name: str, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[FOLLOWME:{veh.name}->{target_enemy_name}] Distance {fb.distance_to_target_m:.1f} m'
        )

    def _on_follow_result_vehicle(self, veh: Vehicle, target_enemy_name: str, result):
        self.get_logger().info(
            f'[FOLLOWME:{veh.name}->{target_enemy_name}] Completed, final distance {result.distance_to_target_m:.1f} m'
        )
        # Vehicle may become IDLE after followme completes elsewhere; trigger a replan to utilize it
        self._replan_search()

    def _replan_search(self) -> None:
        """Recompute and dispatch Search actions to all available UAVs (IDLE or SEARCHING)
        for remaining, undone obj1 points."""
        
        # Available UAVs are those IDLE or SEARCHING (exclude FOLLOWING_ENEMY)
        available_uavs = [v for v in self.vehicles.values() if v.vtype == VehicleType.UAV and v.status in (VehicleStatus.IDLE, VehicleStatus.SEARCHING)]
        if not available_uavs:
            self.get_logger().info('Replan: no available UAVs (IDLE/SEARCHING)')
            return

        # Prepare for replan: snapshot routes and persist progress
        self.mission.prepare_for_replan()

        # Log current progress
        completed, total = self.mission.get_progress_summary()
        self.get_logger().info(f'[REPLAN] Starting replan with {completed}/{total} objectives complete')

        # Solve for remaining tasks
        uav_positions = [u.position[:2] for u in available_uavs]
        sol_points, sol_idx = self.mission.solve_for_uavs(uav_positions)

        # Assign new routes to available UAVs
        uav_names = [uav.name for uav in available_uavs]
        self.mission.assign_routes(uav_names, sol_idx)

        # Dispatch Search actions with new routes
        enemies_idle = self.vehicles.filter(vtype=VehicleType.ENEMY, status=VehicleStatus.IDLE)
        enemy_names = [e.name for e in enemies_idle]
        for uav, route_points, route_idxs in zip(available_uavs, sol_points, sol_idx):
            route_idxs_list = list(map(int, np.asarray(route_idxs).tolist()))
            
            self.get_logger().info(f'[REPLAN] Assigning {uav.name}: {len(route_idxs_list)} waypoints, indices {route_idxs_list[:5]}...')
            
            waypoints = self._build_waypoints_from_solution([route_points], absolute_altitude_m=70.0, speed_m_s=15.0)
            if not waypoints:
                continue
            uav.start_search(
                self,
                waypoints=waypoints,
                vehicle_names=enemy_names,
                search_radius_m=100.0,
                feedback_cb=self._on_search_feedback_vehicle,
                result_cb=self._on_search_result_vehicle,
            )
        
        # Schedule snapshot clear for 1 second after replan (after feedback settles)
        # Create a one-shot timer that destroys itself after firing
        def clear_snapshot_once():
            self.mission.clear_snapshot()
            timer.cancel()  # Destroy the timer after first execution
        
        timer = self.create_timer(1.0, clear_snapshot_once, clock=self.get_clock())


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
