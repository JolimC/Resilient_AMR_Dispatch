"""Launch the Phase 1 centralized dispatch baseline."""

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from resilient_amr_dispatch.scenario import create_missions


def _launch_nodes(context: LaunchContext) -> list[Node]:
    robot_count = int(LaunchConfiguration("robot_count").perform(context))
    mqtt_host = LaunchConfiguration("mqtt_host").perform(context)
    mqtt_port = int(LaunchConfiguration("mqtt_port").perform(context))
    missions = create_missions(robot_count)

    nodes = [
        Node(
            package="resilient_amr_dispatch",
            executable="amr_agent",
            name=mission.robot_id,
            output="screen",
            parameters=[
                {
                    "robot_id": mission.robot_id,
                    "start_x": mission.start_x,
                    "start_y": mission.start_y,
                    "mqtt_host": mqtt_host,
                    "mqtt_port": mqtt_port,
                }
            ],
        )
        for mission in missions
    ]
    nodes.append(
        Node(
            package="resilient_amr_dispatch",
            executable="dispatch_node",
            name="dispatch_node",
            output="screen",
            parameters=[
                {
                    "robot_count": robot_count,
                    "mqtt_host": mqtt_host,
                    "mqtt_port": mqtt_port,
                }
            ],
        )
    )
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_count", default_value="8"),
            DeclareLaunchArgument("mqtt_host", default_value="mqtt"),
            DeclareLaunchArgument("mqtt_port", default_value="1883"),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
