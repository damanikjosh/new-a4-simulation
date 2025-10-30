from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, Tuple, Optional, Iterable, Callable, Any

import rclpy
from rclpy.time import Time
from rclpy.action import ActionClient

from mission_planner_msgs.action import Search, Followme
from std_msgs.msg import Header

class VehicleType(IntEnum):
    UAV = 0
    USV = 1
    ENEMY = 2

class VehicleStatus(IntEnum):
    IDLE = 0
    SEARCHING = 1
    FOLLOWING_ENEMY = 10
    ENEMY_FOUND = 20


@dataclass
class Vehicle:
    name: str
    vtype: VehicleType  # UAV, USV, ENEMY
    position: Tuple[float, float, float]  # latitude, longitude, altitude
    status: VehicleStatus = VehicleStatus.IDLE
    param: str = ""
    last_heartbeat: Optional[Time] = None
    subscription: Optional[rclpy.subscription.Subscription] = None
    # Single active action client/goal per vehicle (Search or Followme)
    _action_name: Optional[str] = None  # 'search' | 'followme'
    _action_client: Optional[ActionClient] = None
    _action_goal_handle: Optional[Any] = None

    # ---- Action control helpers ----
    def _cancel_current_action(self, node: rclpy.node.Node, on_done: Optional[Callable[[], None]] = None) -> None:
        """Cancel any active action and destroy the client before starting a new one."""
        gh = self._action_goal_handle
        client = self._action_client

        def _cleanup():
            # Destroy client if possible and clear state
            try:
                if client is not None:
                    client.destroy()
            except Exception:
                pass
            self._action_client = None
            self._action_goal_handle = None
            self._action_name = None
            # If we actually had something to cancel/destroy, wait 3 seconds
            # before proceeding to the next action for stability.
            if on_done:
                if gh is not None or client is not None:
                    node.get_logger().debug(f"{self.name}: Cancelled action; waiting 3s before starting next action")
                    _timer_ref: dict[str, Any] = {}

                    def _timer_cb():
                        t = _timer_ref.pop('t', None)
                        try:
                            if t is not None:
                                t.cancel()
                        except Exception:
                            pass
                        try:
                            on_done()
                        except Exception:
                            # Avoid throwing inside timer callback
                            node.get_logger().warn(f"{self.name}: Exception in post-cancel on_done callback")
                    _timer_ref['t'] = node.create_timer(1.0, _timer_cb)
                else:
                    on_done()
            else:
                # No next action is planned; return vehicle to IDLE state if something was cancelled
                if gh is not None or client is not None:
                    self.status = VehicleStatus.IDLE

        if gh is not None:
            try:
                fut = gh.cancel_goal_async()
                fut.add_done_callback(lambda _f: _cleanup())
            except Exception:
                _cleanup()
        elif client is not None:
            _cleanup()
        else:
            if on_done:
                on_done()

    def start_search(
        self,
        node: rclpy.node.Node,
        *,
        waypoints,
        vehicle_names,
        search_radius_m: float,
        feedback_cb: Optional[Callable[["Vehicle", Any], None]] = None,
        result_cb: Optional[Callable[["Vehicle", Any], None]] = None,
    ) -> None:
        """Start a Search action on this vehicle. Ensures single active client by canceling/destroying previous one."""

        def _proceed():
            topic = f'/{self.name}/task/search'
            client = ActionClient(node, Search, topic)
            if not client.wait_for_server(timeout_sec=0.5):
                node.get_logger().warn(f'SEARCH action server not available for {self.name} at {topic}')
                try:
                    client.destroy()
                except Exception:
                    pass
                return

            goal = Search.Goal()
            goal.waypoints = waypoints
            goal.vehicle_names = list(vehicle_names)
            goal.search_radius_m = float(search_radius_m)

            def _on_feedback(fb_msg):
                if self._action_name != 'search':
                    return
                if feedback_cb:
                    feedback_cb(self, fb_msg)

            self._action_client = client
            self._action_name = 'search'
            send_future = client.send_goal_async(goal, feedback_callback=_on_feedback)

            def _on_goal_response(fut):
                gh = fut.result()
                if not gh.accepted:
                    node.get_logger().warn(f'SEARCH goal rejected by {self.name}')
                    return
                node.get_logger().info(f'SEARCH goal accepted by {self.name}')
                # Mark this vehicle as actively searching
                self.status = VehicleStatus.SEARCHING
                self._action_goal_handle = gh
                res_fut = gh.get_result_async()
                res_fut.add_done_callback(lambda rf: result_cb(self, rf.result().result) if result_cb else None)

            send_future.add_done_callback(_on_goal_response)

        # Cancel/destroy then proceed
        self._cancel_current_action(node, on_done=_proceed)

    def start_followme(
        self,
        node: rclpy.node.Node,
        *,
        target_enemy_name: str,
        feedback_cb: Optional[Callable[["Vehicle", str, Any], None]] = None,
        result_cb: Optional[Callable[["Vehicle", str, Any], None]] = None,
    ) -> None:
        """Start a Followme action on this vehicle to follow the given enemy name."""

        def _proceed():
            topic = f'/{self.name}/task/followme'
            client = ActionClient(node, Followme, topic)
            if not client.wait_for_server(timeout_sec=0.5):
                node.get_logger().warn(f'FOLLOWME action server not available for {self.name} at {topic}')
                try:
                    client.destroy()
                except Exception:
                    pass
                return

            goal = Followme.Goal()
            goal.header = Header()
            goal.header.stamp = node.get_clock().now().to_msg()
            goal.header.frame_id = 'map'
            goal.vehicle_name = target_enemy_name

            def _on_feedback(fb_msg):
                if self._action_name != 'followme':
                    return
                if feedback_cb:
                    feedback_cb(self, target_enemy_name, fb_msg)

            self._action_client = client
            self._action_name = 'followme'
            send_future = client.send_goal_async(goal, feedback_callback=_on_feedback)

            def _on_goal_response(fut):
                gh = fut.result()
                if not gh.accepted:
                    node.get_logger().warn(f'FOLLOWME goal rejected by {self.name} for target {target_enemy_name}')
                    return
                node.get_logger().info(f'FOLLOWME goal accepted by {self.name} for target {target_enemy_name}')
                # Mark vehicle as following an enemy (not available for searching)
                self.status = VehicleStatus.FOLLOWING_ENEMY
                self._action_goal_handle = gh
                res_fut = gh.get_result_async()
                res_fut.add_done_callback(lambda rf: result_cb(self, target_enemy_name, rf.result().result) if result_cb else None)

            send_future.add_done_callback(_on_goal_response)

        # Cancel/destroy then proceed
        self._cancel_current_action(node, on_done=_proceed)

    def cancel_action(self, node: rclpy.node.Node) -> None:
        """Public method to cancel any current action and destroy the client."""
        self._cancel_current_action(node)


class VehicleDirectory:
    """Stupid-simple directory for vehicles with clean combo filtering."""
    def __init__(self) -> None:
        self._by_name: Dict[str, Vehicle] = {}

    def __len__(self) -> int:
        return len(self._by_name)

    def __iter__(self) -> Iterable[Vehicle]:
        return iter(self._by_name.values())

    def items(self) -> Iterable[Tuple[str, Vehicle]]:
        return self._by_name.items()

    def values(self) -> Iterable[Vehicle]:
        return self._by_name.values()

    def get(self, name: str) -> Optional[Vehicle]:
        return self._by_name.get(name)

    def add_vehicle(self, vehicle: Vehicle) -> Vehicle:
        # Only create if not present (preserve existing subscription/heartbeat/position)
        existing = self._by_name.get(vehicle.name)
        if existing is not None:
            return existing
        self._by_name[vehicle.name] = vehicle
        return vehicle

    def add(self, name: str, vtype: VehicleType, status: VehicleStatus = VehicleStatus.IDLE) -> Vehicle:
        v = Vehicle(name=name, vtype=vtype, position=(0.0, 0.0, 0.0), status=status)
        return self.add_vehicle(v)

    def remove(self, name: str) -> bool:
        return self._by_name.pop(name, None) is not None

    def set_status(self, name: str, status: VehicleStatus) -> bool:
        v = self._by_name.get(name)
        if v is None:
            return False
        v.status = status
        return True

    def update_position(self, name: str, position: Tuple[float, float, float]) -> bool:
        v = self._by_name.get(name)
        if v is None:
            return False
        v.position = position
        return True

    def update_heartbeat(self, name: str, t: Time) -> bool:
        v = self._by_name.get(name)
        if v is None:
            return False
        v.last_heartbeat = t
        return True

    def by_type(self, vtype: VehicleType) -> list[Vehicle]:
        return [v for v in self._by_name.values() if v.vtype == vtype]

    def by_status(self, status: VehicleStatus) -> list[Vehicle]:
        return [v for v in self._by_name.values() if v.status == status]

    def filter(
        self,
        *,
        vtype: Optional[VehicleType] = None,
        status: Optional[VehicleStatus] = None,
    ) -> list[Vehicle]:
        """Filter by any combo of type and status.
        - vtype: specific VehicleType, or None to ignore
        - status: ... to ignore; None for vehicles with no status; or a VehicleStatus for exact match
        """
        result = self._by_name.values()
        if vtype is not None:
            result = [v for v in result if v.vtype == vtype]
        if status is not None:
            result = [v for v in result if v.status == status]
        return list(result)
