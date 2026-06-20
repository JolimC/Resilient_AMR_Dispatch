"""Simulated AMR that executes a mission and observes warehouse hazards."""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Any

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from resilient_amr_dispatch.hazards import Hazard
from resilient_amr_dispatch.recovery_policy import (
    RecoveryResult,
    advance_along_path,
    build_recovery_exception,
    has_cleared_hazard,
    plan_local_recovery,
)
from resilient_amr_dispatch.warehouse_map import WarehouseMap
from resilient_amr_dispatch.traffic_policy import (
    PeerState,
    fleet_blocked_cells,
    path_conflicts,
)


ORDER_TOPIC = "vda5050/fleet/order"
STATE_TOPIC = "/fleet/robot_state"
HAZARD_TOPIC = "/warehouse/hazards"
EXCEPTION_TOPIC = "vda5050/fleet/events/exception"


class AmrAgent(Node):
    def __init__(self) -> None:
        super().__init__("amr_agent")
        self.declare_parameter("robot_id", "amr_01")
        self.declare_parameter("start_x", 10.0)
        self.declare_parameter("start_y", 10.0)
        self.declare_parameter("update_hz", 10.0)
        self.declare_parameter("speed", 12.0)
        self.declare_parameter("mqtt_host", "mqtt")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("robot_safety_radius", 1)

        self.robot_id = str(self.get_parameter("robot_id").value)
        self.x = float(self.get_parameter("start_x").value)
        self.y = float(self.get_parameter("start_y").value)
        self.update_hz = float(self.get_parameter("update_hz").value)
        self.speed = float(self.get_parameter("speed").value)
        self._robot_safety_radius = int(
            self.get_parameter("robot_safety_radius").value
        )
        if self.update_hz <= 0.0 or self.speed <= 0.0:
            raise ValueError("update_hz and speed must be positive")
        if self._robot_safety_radius < 0:
            raise ValueError("robot_safety_radius must not be negative")

        self._mission_id = ""
        self._mission_started_at: float | None = None
        self._nominal_duration: float | None = None
        self._goal: tuple[float, float] | None = None
        self._state = "idle"
        self._battery = 1.0
        self._path: tuple[tuple[float, float], ...] = ()
        self._recovery_state = "none"
        self._active_hazard: Hazard | None = None
        self._hazards: dict[str, Hazard] = {}
        self._affected_hazard_ids: set[str] = set()
        self._peers: dict[str, PeerState] = {}
        self._traffic_waiting = False
        self._traffic_recovery_until = 0.0
        self._last_traffic_replan_at = 0.0
        self._warehouse = WarehouseMap()
        self._lock = threading.Lock()
        self._state_publisher = self.create_publisher(String, STATE_TOPIC, 10)
        self.create_subscription(String, STATE_TOPIC, self._on_peer_state, 100)
        hazard_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(String, HAZARD_TOPIC, self._on_hazard, hazard_qos)

        self._mqtt = mqtt.Client(client_id=f"{self.robot_id}-agent")
        self._mqtt.on_connect = self._on_mqtt_connect
        self._mqtt.on_message = self._on_mqtt_message
        mqtt_host = str(self.get_parameter("mqtt_host").value)
        mqtt_port = int(self.get_parameter("mqtt_port").value)
        self._mqtt.connect_async(mqtt_host, mqtt_port, keepalive=30)
        self._mqtt.loop_start()

        self.create_timer(1.0 / self.update_hz, self._tick)
        self._log("agent_started", x=self.x, y=self.y)

    def _log(self, event: str, **fields: Any) -> None:
        payload = {"event": event, "robot_id": self.robot_id, **fields}
        self.get_logger().info(json.dumps(payload, sort_keys=True))

    def _on_mqtt_connect(
        self, client: mqtt.Client, userdata: Any, flags: dict[str, Any], rc: int
    ) -> None:
        if rc != 0:
            self.get_logger().error(
                json.dumps({"event": "mqtt_connect_failed", "rc": rc})
            )
            return
        client.subscribe(ORDER_TOPIC, qos=1)
        self._log("mqtt_connected", topic=ORDER_TOPIC)

    def _on_mqtt_message(
        self, client: mqtt.Client, userdata: Any, message: mqtt.MQTTMessage
    ) -> None:
        try:
            order = json.loads(message.payload.decode("utf-8"))
            if order.get("robot_id") != self.robot_id:
                return
            mission_id = str(order["mission_id"])
            goal_x = float(order["goal"]["x"])
            goal_y = float(order["goal"]["y"])
            if not (0.0 <= goal_x <= 100.0 and 0.0 <= goal_y <= 100.0):
                raise ValueError("goal is outside the 100 x 100 warehouse")
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().error(
                json.dumps(
                    {"event": "invalid_order", "robot_id": self.robot_id, "error": str(exc)}
                )
            )
            return

        with self._lock:
            if mission_id == self._mission_id:
                return
            if self._state not in ("idle", "completed"):
                self._log("order_rejected_busy", mission_id=mission_id)
                return
            self._mission_id = mission_id
            self._mission_started_at = time.time()
            self._nominal_duration = math.hypot(
                goal_x - self.x, goal_y - self.y
            ) / self.speed
            self._goal = (goal_x, goal_y)
            self._path = (self._goal,)
            self._state = "assigned"
            self._recovery_state = "none"
            self._active_hazard = None
            self._affected_hazard_ids.clear()
            self._traffic_waiting = False
            self._traffic_recovery_until = 0.0
            newly_affected = self._detect_affected_hazards_locked()
        self._log("mission_assigned", mission_id=mission_id, goal=order["goal"])
        self._recover_from(newly_affected)

    def _on_hazard(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            hazard = Hazard.from_payload(payload)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().error(
                json.dumps(
                    {"event": "invalid_hazard", "robot_id": self.robot_id, "error": str(exc)}
                )
            )
            return

        with self._lock:
            self._hazards[hazard.hazard_id] = hazard
            newly_affected = self._detect_affected_hazards_locked()
        self._log(
            "hazard_received",
            hazard_id=hazard.hazard_id,
            hazard_type=hazard.hazard_type,
        )
        self._recover_from(newly_affected)

    def _detect_affected_hazards_locked(self) -> list[Hazard]:
        if self._goal is None or self._state not in ("assigned", "executing"):
            return []
        newly_affected = []
        for hazard in self._hazards.values():
            if (
                hazard.severity == "blocked"
                and hazard.hazard_id not in self._affected_hazard_ids
                and hazard.bounds.intersects_segment((self.x, self.y), self._goal)
            ):
                self._affected_hazard_ids.add(hazard.hazard_id)
                newly_affected.append(hazard)
        return newly_affected

    def _on_peer_state(self, message: String) -> None:
        try:
            peer = PeerState.from_payload(json.loads(message.data))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        if peer.robot_id == self.robot_id:
            return

        traffic_event: tuple[str, int] | None = None
        with self._lock:
            self._peers[peer.robot_id] = peer
            traffic_event = self._replan_for_traffic_locked()
        if traffic_event is not None:
            event, waypoint_count = traffic_event
            self._log(
                event,
                mission_id=self._mission_id,
                waypoint_count=waypoint_count,
            )

    def _dynamic_blocked_locked(self) -> set[tuple[int, int]]:
        return fleet_blocked_cells(
            self._peers.values(),
            dimensions=self._warehouse.dimensions,
            safety_radius=self._robot_safety_radius,
        )

    def _replan_for_traffic_locked(self) -> tuple[str, int] | None:
        now = time.monotonic()
        if (
            self._state != "executing"
            or self._goal is None
            or not self._path
            or now - self._last_traffic_replan_at < 0.5
        ):
            return None
        blocked = self._dynamic_blocked_locked()
        if not path_conflicts((self.x, self.y), self._path, blocked):
            return None

        self._last_traffic_replan_at = now
        result = plan_local_recovery(
            self._warehouse,
            start=(self.x, self.y),
            goal=self._goal,
            hazards=self._hazards.values(),
            dynamic_blocked=blocked,
        )
        self._recovery_state = "rerouting"
        self._traffic_recovery_until = now + 1.0
        if result.status == "rerouting":
            self._path = result.waypoints
            self._traffic_waiting = False
            return ("traffic_reroute", len(result.waypoints))

        self._traffic_waiting = True
        return ("traffic_wait", 0)

    def _recover_from(self, hazards: list[Hazard]) -> None:
        for hazard in hazards:
            self._log(
                "active_path_affected",
                mission_id=self._mission_id,
                hazard_id=hazard.hazard_id,
                reason=hazard.hazard_type,
            )
            with self._lock:
                if self._goal is None or self._state in ("blocked", "completed"):
                    continue
                result = plan_local_recovery(
                    self._warehouse,
                    start=(self.x, self.y),
                    goal=self._goal,
                    hazards=self._hazards.values(),
                    dynamic_blocked=self._dynamic_blocked_locked(),
                )
                if result.status == "rerouting":
                    self._path = result.waypoints
                    self._recovery_state = "rerouting"
                    self._active_hazard = hazard
                else:
                    self._path = ()
                    self._state = "blocked"
                    self._recovery_state = "blocked"
                    self._active_hazard = hazard
            self._publish_recovery_exception(hazard, result)

    def _publish_recovery_exception(
        self, hazard: Hazard, result: RecoveryResult
    ) -> None:
        succeeded = result.status == "rerouting"
        payload = build_recovery_exception(
            self.robot_id, self._mission_id, hazard, result
        )
        publish_result = self._mqtt.publish(
            EXCEPTION_TOPIC,
            json.dumps(payload, separators=(",", ":")),
            qos=1,
        )
        log_method = self._log if succeeded else self.get_logger().error
        if succeeded:
            log_method(
                payload["event"],
                mission_id=self._mission_id,
                hazard_id=hazard.hazard_id,
                action=payload["action"],
                waypoint_count=len(result.waypoints),
                mqtt_rc=publish_result.rc,
            )
        else:
            log_method(json.dumps(payload, sort_keys=True))

    def _tick(self) -> None:
        with self._lock:
            if self._state == "assigned":
                self._publish_state_locked()
                self._state = "executing"
                self._log("mission_executing", mission_id=self._mission_id)
                return

            if self._state == "executing" and self._goal is not None:
                old_position = (self.x, self.y)
                new_position, remaining_path = advance_along_path(
                    old_position,
                    self._path,
                    self.speed / self.update_hz,
                )
                blocked = self._dynamic_blocked_locked()
                if path_conflicts(old_position, (new_position,), blocked):
                    new_position = old_position
                    remaining_path = self._path
                    self._traffic_waiting = True
                    self._recovery_state = "rerouting"
                else:
                    self._traffic_waiting = False
                self.x, self.y = new_position
                self._path = remaining_path
                distance_moved = math.hypot(
                    self.x - old_position[0], self.y - old_position[1]
                )
                self._battery = max(
                    0.0, self._battery - 0.0005 * distance_moved
                )

                if (
                    self._recovery_state == "rerouting"
                    and self._active_hazard is not None
                    and has_cleared_hazard(
                        (self.x, self.y), self._goal, self._active_hazard
                    )
                ):
                    cleared_hazard_id = self._active_hazard.hazard_id
                    self._active_hazard = None
                    self._log(
                        "local_recovery_complete",
                        mission_id=self._mission_id,
                        hazard_id=cleared_hazard_id,
                    )

                if (
                    self._recovery_state == "rerouting"
                    and self._active_hazard is None
                    and not self._traffic_waiting
                    and time.monotonic() >= self._traffic_recovery_until
                ):
                    self._recovery_state = "none"

                if not self._path:
                    self.x, self.y = self._goal
                    self._state = "completed"
                    self._recovery_state = "none"
                    self._active_hazard = None
                    self._log("mission_completed", mission_id=self._mission_id)

            self._publish_state_locked()

    def _publish_state_locked(self) -> None:
        goal = self._goal
        payload = {
            "robot_id": self.robot_id,
            "mission_id": self._mission_id or None,
            "mission_started_at": self._mission_started_at,
            "nominal_duration": self._nominal_duration,
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "state": self._state,
            "recovery_state": self._recovery_state,
            "battery": round(self._battery, 4),
            "goal": None if goal is None else {"x": goal[0], "y": goal[1]},
            "path_affected": self._recovery_state in ("rerouting", "blocked"),
            "affected_hazard_ids": sorted(self._affected_hazard_ids),
            "active_hazard_id": (
                None if self._active_hazard is None else self._active_hazard.hazard_id
            ),
            "planned_path": [
                {"x": waypoint[0], "y": waypoint[1]} for waypoint in self._path
            ],
            "traffic_waiting": self._traffic_waiting,
            "timestamp": time.time(),
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self._state_publisher.publish(message)

    def destroy_node(self) -> bool:
        self._mqtt.loop_stop()
        self._mqtt.disconnect()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: AmrAgent | None = None
    try:
        node = AmrAgent()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
