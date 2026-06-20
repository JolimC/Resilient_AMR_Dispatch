"""Inject a deterministic warehouse disruption after missions begin."""

from __future__ import annotations

import json
import time
from typing import Any

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from resilient_amr_dispatch.hazards import PHASE_3_SPILL


HAZARD_ROS_TOPIC = "/warehouse/hazards"
HAZARD_MQTT_TOPIC = "vda5050/fleet/events/hazard"


class HazardInjector(Node):
    def __init__(self) -> None:
        super().__init__("hazard_injector")
        self.declare_parameter("injection_delay", 2.5)
        self.declare_parameter("mqtt_host", "mqtt")
        self.declare_parameter("mqtt_port", 1883)
        self._injection_delay = float(self.get_parameter("injection_delay").value)
        if self._injection_delay < 0.0:
            raise ValueError("injection_delay must not be negative")

        hazard_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.create_publisher(String, HAZARD_ROS_TOPIC, hazard_qos)
        self._started_at = time.monotonic()
        self._injected = False
        self._mqtt_connected = False

        self._mqtt = mqtt.Client(client_id="hazard-injector")
        self._mqtt.on_connect = self._on_mqtt_connect
        mqtt_host = str(self.get_parameter("mqtt_host").value)
        mqtt_port = int(self.get_parameter("mqtt_port").value)
        self._mqtt.connect_async(mqtt_host, mqtt_port, keepalive=30)
        self._mqtt.loop_start()
        self.create_timer(0.1, self._inject_when_ready)

    def _on_mqtt_connect(
        self, client: mqtt.Client, userdata: Any, flags: dict[str, Any], rc: int
    ) -> None:
        self._mqtt_connected = rc == 0
        if not self._mqtt_connected:
            self.get_logger().error(json.dumps({"event": "mqtt_connect_failed", "rc": rc}))

    def _inject_when_ready(self) -> None:
        if (
            self._injected
            or not self._mqtt_connected
            or time.monotonic() - self._started_at < self._injection_delay
        ):
            return

        payload = PHASE_3_SPILL.as_payload()
        encoded = json.dumps(payload, separators=(",", ":"))
        ros_message = String()
        ros_message.data = encoded
        self._publisher.publish(ros_message)
        result = self._mqtt.publish(HAZARD_MQTT_TOPIC, encoded, qos=1)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().error(
                json.dumps({"event": "hazard_mqtt_publish_failed", "rc": result.rc})
            )
        self._injected = True
        self.get_logger().warning(
            json.dumps(
                {
                    "event": "hazard_injected",
                    "hazard_id": PHASE_3_SPILL.hazard_id,
                    "type": PHASE_3_SPILL.hazard_type,
                    "bounds": PHASE_3_SPILL.bounds.as_dict(),
                },
                sort_keys=True,
            )
        )

    def destroy_node(self) -> bool:
        self._mqtt.loop_stop()
        self._mqtt.disconnect()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: HazardInjector | None = None
    try:
        node = HazardInjector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
