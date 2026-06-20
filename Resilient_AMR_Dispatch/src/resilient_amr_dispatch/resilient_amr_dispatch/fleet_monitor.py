"""Fleet health, recovery-event, and final-summary monitor."""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from resilient_amr_dispatch.metrics import FleetMetrics


STATE_TOPIC = "/fleet/robot_state"
HAZARD_TOPIC = "/warehouse/hazards"
METRICS_TOPIC = "/fleet/metrics"
EXCEPTION_TOPIC = "vda5050/fleet/events/exception"


class FleetMonitor(Node):
    def __init__(self) -> None:
        super().__init__("fleet_monitor")
        self.declare_parameter("expected_robots", 8)
        self.declare_parameter("stale_after", 1.5)
        self.declare_parameter("mqtt_host", "mqtt")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("summary_path", "/workspace/captures/final_metrics.json")

        expected = int(self.get_parameter("expected_robots").value)
        self._stale_after = float(self.get_parameter("stale_after").value)
        self._summary_path = Path(str(self.get_parameter("summary_path").value))
        self._metrics = FleetMetrics(expected)
        self._lock = threading.Lock()
        self._last_published = ""
        self._summary_written = False

        event_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(String, STATE_TOPIC, self._on_state, 100)
        self.create_subscription(String, HAZARD_TOPIC, self._on_hazard, event_qos)
        self._publisher = self.create_publisher(String, METRICS_TOPIC, event_qos)
        self.create_timer(0.5, self._monitor_health)

        self._mqtt = mqtt.Client(client_id=f"fleet-monitor-{os.getpid()}")
        self._mqtt.on_connect = self._on_mqtt_connect
        self._mqtt.on_message = self._on_mqtt_message
        mqtt_host = str(self.get_parameter("mqtt_host").value)
        mqtt_port = int(self.get_parameter("mqtt_port").value)
        self._mqtt.connect_async(mqtt_host, mqtt_port, keepalive=30)
        self._mqtt.loop_start()
        self.get_logger().info(json.dumps({"event": "fleet_monitor_started"}))

    def _on_mqtt_connect(
        self, client: mqtt.Client, userdata: Any, flags: dict[str, Any], rc: int
    ) -> None:
        if rc == 0:
            client.subscribe(EXCEPTION_TOPIC, qos=1)
        else:
            self.get_logger().error(json.dumps({"event": "mqtt_connect_failed", "rc": rc}))

    def _on_mqtt_message(
        self, client: mqtt.Client, userdata: Any, message: mqtt.MQTTMessage
    ) -> None:
        try:
            payload = json.loads(message.payload.decode("utf-8"))
            with self._lock:
                self._metrics.observe_exception(payload)
            self._publish_metrics()
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignored invalid exception event: {exc}")

    def _on_state(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            with self._lock:
                self._metrics.observe_state(payload, time.monotonic())
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignored invalid robot state: {exc}")
            return
        self._publish_metrics()

    def _on_hazard(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            with self._lock:
                self._metrics.observe_hazard(payload)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignored invalid hazard event: {exc}")
            return
        self._publish_metrics()

    def _monitor_health(self) -> None:
        with self._lock:
            stale_robots = self._metrics.check_stale(
                time.monotonic(), self._stale_after
            )
        for robot_id in stale_robots:
            self.get_logger().warning(
                json.dumps({"event": "stale_telemetry", "robot_id": robot_id})
            )
        self._publish_metrics()

    def _publish_metrics(self) -> None:
        with self._lock:
            snapshot = self._metrics.snapshot()
            encoded = json.dumps(snapshot, sort_keys=True)
            if encoded == self._last_published:
                return
            self._last_published = encoded
            should_write_summary = snapshot["final"] and not self._summary_written
            if should_write_summary:
                self._summary_written = True
        message = String()
        message.data = encoded
        self._publisher.publish(message)
        self.get_logger().info(
            json.dumps({"event": "fleet_metrics", **snapshot}, sort_keys=True)
        )
        if should_write_summary:
            self._write_summary(snapshot)

    def _write_summary(self, snapshot: dict[str, Any]) -> None:
        self._summary_path.parent.mkdir(parents=True, exist_ok=True)
        self._summary_path.write_text(json.dumps(snapshot, indent=2, sort_keys=True) + "\n")
        self.get_logger().info(
            json.dumps({"event": "final_summary", **snapshot}, sort_keys=True)
        )

    def write_current_summary(self) -> None:
        with self._lock:
            snapshot = self._metrics.snapshot()
        self._write_summary(snapshot)

    def destroy_node(self) -> bool:
        self._mqtt.loop_stop()
        self._mqtt.disconnect()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    cli_args = list(sys.argv[1:] if args is None else args)
    summary_requested = "--summary" in cli_args
    ros_args = (
        [argument for argument in cli_args if argument != "--summary"]
        if summary_requested
        else args
    )
    rclpy.init(args=ros_args)
    node: FleetMonitor | None = None
    try:
        node = FleetMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if summary_requested:
                node.write_current_summary()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
