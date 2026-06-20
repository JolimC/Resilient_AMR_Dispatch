"""Central dispatcher and mission-state observer."""

from __future__ import annotations

import json
import time
from typing import Any

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from resilient_amr_dispatch.scenario import Mission, create_missions


ORDER_TOPIC = "vda5050/fleet/order"
STATE_TOPIC = "/fleet/robot_state"


class DispatchNode(Node):
    def __init__(self) -> None:
        super().__init__("dispatch_node")
        self.declare_parameter("robot_count", 8)
        self.declare_parameter("dispatch_delay", 2.0)
        self.declare_parameter("mqtt_host", "mqtt")
        self.declare_parameter("mqtt_port", 1883)

        robot_count = int(self.get_parameter("robot_count").value)
        self._missions = create_missions(robot_count)
        self._mission_by_robot = {mission.robot_id: mission for mission in self._missions}
        self._observed_robots: set[str] = set()
        self._last_states: dict[str, str] = {}
        self._last_recovery_states: dict[str, str] = {}
        self._completed: set[str] = set()
        self._affected_robots: set[str] = set()
        self._dispatch_delay = float(self.get_parameter("dispatch_delay").value)
        self._started_at = time.monotonic()
        self._mqtt_connected = False
        self._summary_logged = False

        self.create_subscription(String, STATE_TOPIC, self._on_robot_state, 50)
        self._mqtt = mqtt.Client(client_id="central-dispatch")
        self._mqtt.on_connect = self._on_mqtt_connect
        mqtt_host = str(self.get_parameter("mqtt_host").value)
        mqtt_port = int(self.get_parameter("mqtt_port").value)
        self._mqtt.connect_async(mqtt_host, mqtt_port, keepalive=30)
        self._mqtt.loop_start()

        self.create_timer(1.0, self._dispatch_unobserved)
        self._log("dispatcher_started", robot_count=robot_count)

    def _log(self, event: str, **fields: Any) -> None:
        self.get_logger().info(json.dumps({"event": event, **fields}, sort_keys=True))

    def _on_mqtt_connect(
        self, client: mqtt.Client, userdata: Any, flags: dict[str, Any], rc: int
    ) -> None:
        self._mqtt_connected = rc == 0
        if self._mqtt_connected:
            self._log("mqtt_connected", topic=ORDER_TOPIC)
        else:
            self.get_logger().error(json.dumps({"event": "mqtt_connect_failed", "rc": rc}))

    def _dispatch_unobserved(self) -> None:
        if (
            not self._mqtt_connected
            or time.monotonic() - self._started_at < self._dispatch_delay
        ):
            return

        for mission in self._missions:
            if mission.robot_id in self._observed_robots:
                continue
            self._publish_order(mission)

    def _publish_order(self, mission: Mission) -> None:
        payload = json.dumps(mission.as_order(), separators=(",", ":"))
        result = self._mqtt.publish(ORDER_TOPIC, payload, qos=1)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().error(
                json.dumps(
                    {
                        "event": "order_publish_failed",
                        "robot_id": mission.robot_id,
                        "rc": result.rc,
                    }
                )
            )
            return
        self._log(
            "mission_order_published",
            mission_id=mission.mission_id,
            robot_id=mission.robot_id,
        )

    def _on_robot_state(self, message: String) -> None:
        try:
            state = json.loads(message.data)
            robot_id = str(state["robot_id"])
            mission_id = str(state["mission_id"])
            lifecycle = str(state["state"])
            mission = self._mission_by_robot[robot_id]
            if mission_id != mission.mission_id:
                return
        except (KeyError, TypeError, json.JSONDecodeError):
            self.get_logger().warning(json.dumps({"event": "invalid_robot_state"}))
            return

        self._observed_robots.add(robot_id)
        if self._last_states.get(robot_id) != lifecycle:
            self._last_states[robot_id] = lifecycle
            self._log(
                "mission_state_changed",
                robot_id=robot_id,
                mission_id=mission_id,
                state=lifecycle,
                x=state.get("x"),
                y=state.get("y"),
            )

        recovery_state = str(state.get("recovery_state", "none"))
        if self._last_recovery_states.get(robot_id) != recovery_state:
            self._last_recovery_states[robot_id] = recovery_state
            if recovery_state != "none":
                self._log(
                    "robot_recovery_state_changed",
                    robot_id=robot_id,
                    mission_id=mission_id,
                    recovery_state=recovery_state,
                    hazard_id=state.get("active_hazard_id"),
                )

        if lifecycle == "completed":
            self._completed.add(robot_id)
        if state.get("path_affected") and robot_id not in self._affected_robots:
            self._affected_robots.add(robot_id)
            self._log(
                "robot_path_affected",
                robot_id=robot_id,
                mission_id=mission_id,
                hazard_ids=state.get("affected_hazard_ids", []),
            )
        if len(self._completed) == len(self._missions) and not self._summary_logged:
            self._summary_logged = True
            self._log(
                "baseline_complete",
                missions_assigned=len(self._observed_robots),
                missions_completed=len(self._completed),
            )

    def destroy_node(self) -> bool:
        self._mqtt.loop_stop()
        self._mqtt.disconnect()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: DispatchNode | None = None
    try:
        node = DispatchNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
