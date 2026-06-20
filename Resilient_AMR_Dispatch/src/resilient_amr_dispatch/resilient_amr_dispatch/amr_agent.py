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


ORDER_TOPIC = "vda5050/fleet/order"
STATE_TOPIC = "/fleet/robot_state"
HAZARD_TOPIC = "/warehouse/hazards"


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

        self.robot_id = str(self.get_parameter("robot_id").value)
        self.x = float(self.get_parameter("start_x").value)
        self.y = float(self.get_parameter("start_y").value)
        self.update_hz = float(self.get_parameter("update_hz").value)
        self.speed = float(self.get_parameter("speed").value)
        if self.update_hz <= 0.0 or self.speed <= 0.0:
            raise ValueError("update_hz and speed must be positive")

        self._mission_id = ""
        self._goal: tuple[float, float] | None = None
        self._state = "idle"
        self._battery = 1.0
        self._hazards: dict[str, Hazard] = {}
        self._affected_hazard_ids: set[str] = set()
        self._lock = threading.Lock()
        self._state_publisher = self.create_publisher(String, STATE_TOPIC, 10)
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
            self._goal = (goal_x, goal_y)
            self._state = "assigned"
            newly_affected = self._detect_affected_hazards_locked()
        self._log("mission_assigned", mission_id=mission_id, goal=order["goal"])
        self._log_newly_affected(newly_affected)

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
        self._log_newly_affected(newly_affected)

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

    def _log_newly_affected(self, hazards: list[Hazard]) -> None:
        for hazard in hazards:
            self._log(
                "active_path_affected",
                mission_id=self._mission_id,
                hazard_id=hazard.hazard_id,
                reason=hazard.hazard_type,
            )

    def _tick(self) -> None:
        with self._lock:
            if self._state == "assigned":
                self._publish_state_locked()
                self._state = "executing"
                self._log("mission_executing", mission_id=self._mission_id)
                return

            if self._state == "executing" and self._goal is not None:
                goal_x, goal_y = self._goal
                dx = goal_x - self.x
                dy = goal_y - self.y
                distance = math.hypot(dx, dy)
                step = self.speed / self.update_hz
                if distance <= step:
                    self.x = goal_x
                    self.y = goal_y
                    self._state = "completed"
                    self._log("mission_completed", mission_id=self._mission_id)
                else:
                    self.x += step * dx / distance
                    self.y += step * dy / distance
                    self._battery = max(0.0, self._battery - 0.0005)

            self._publish_state_locked()

    def _publish_state_locked(self) -> None:
        goal = self._goal
        payload = {
            "robot_id": self.robot_id,
            "mission_id": self._mission_id or None,
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "state": self._state,
            "recovery_state": "none",
            "battery": round(self._battery, 4),
            "goal": None if goal is None else {"x": goal[0], "y": goal[1]},
            "path_affected": bool(self._affected_hazard_ids),
            "affected_hazard_ids": sorted(self._affected_hazard_ids),
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
