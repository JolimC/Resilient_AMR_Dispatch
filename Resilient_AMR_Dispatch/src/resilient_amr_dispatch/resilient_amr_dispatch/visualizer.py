"""Live Matplotlib visualization for the warehouse fleet."""

from __future__ import annotations

from bisect import bisect_right
from collections import Counter
import json
import threading
import time
from typing import Any

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Rectangle
from matplotlib.widgets import Button, Slider
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from resilient_amr_dispatch.hazards import Hazard


STATE_TOPIC = "/fleet/robot_state"
HAZARD_TOPIC = "/warehouse/hazards"
STATE_COLORS = {
    "idle": "#7f8c8d",
    "assigned": "#8e44ad",
    "executing": "#1976d2",
    "rerouting": "#f9a825",
    "completed": "#2e7d32",
    "blocked": "#c62828",
}

# Shelves occupy the gaps between the Phase 1 travel lanes.
SHELVES = (
    (25.0, 15.0, 18.0, 3.0),
    (57.0, 15.0, 18.0, 3.0),
    (25.0, 37.0, 18.0, 3.0),
    (57.0, 37.0, 18.0, 3.0),
    (25.0, 60.0, 18.0, 3.0),
    (57.0, 60.0, 18.0, 3.0),
    (25.0, 82.0, 18.0, 3.0),
    (57.0, 82.0, 18.0, 3.0),
)


class FleetVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("visualizer")
        self._states: dict[str, dict[str, Any]] = {}
        self._hazards: dict[str, dict[str, Any]] = {}
        self._artists: dict[str, dict[str, Any]] = {}
        self._hazard_artists: dict[str, tuple[Rectangle, Any]] = {}
        self._history: list[
            tuple[float, dict[str, dict[str, Any]], dict[str, dict[str, Any]]]
        ] = []
        self._recording_started = time.monotonic()
        self._last_recorded_signature: tuple[Any, ...] | None = None
        self._playback_index = 0
        self._playing = True
        self._follow_live = True
        self._updating_slider = False
        self._lock = threading.Lock()
        self.create_subscription(String, STATE_TOPIC, self._on_state, 100)
        hazard_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(String, HAZARD_TOPIC, self._on_hazard, hazard_qos)

        self._figure, (self._map_axis, self._status_axis) = plt.subplots(
            1,
            2,
            figsize=(13.5, 7.5),
            gridspec_kw={"width_ratios": [4.0, 1.25]},
        )
        self._configure_map()
        self._configure_status()
        self._figure.subplots_adjust(bottom=0.16, wspace=0.18)
        self._configure_playback_controls()
        self._figure.canvas.manager.set_window_title("Resilient AMR Dispatch")
        self._animation = FuncAnimation(
            self._figure,
            self._draw_frame,
            interval=50,
            cache_frame_data=False,
        )

    def _configure_map(self) -> None:
        axis = self._map_axis
        axis.set_title("Warehouse Mission Execution", fontsize=15, weight="bold")
        axis.set_xlim(0.0, 100.0)
        axis.set_ylim(0.0, 100.0)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("X position")
        axis.set_ylabel("Y position")
        axis.set_xticks(range(0, 101, 10))
        axis.set_yticks(range(0, 101, 10))
        axis.grid(color="#dfe6e9", linewidth=0.7, zorder=0)
        axis.axvspan(4.0, 14.0, color="#d6eaf8", alpha=0.45, zorder=0)
        axis.axvspan(86.0, 96.0, color="#d5f5e3", alpha=0.45, zorder=0)
        axis.text(9.0, 97.0, "STAGING", ha="center", va="top", color="#2874a6")
        axis.text(91.0, 97.0, "DOCKS", ha="center", va="top", color="#1e8449")

        for x, y, width, height in SHELVES:
            axis.add_patch(
                Rectangle(
                    (x, y),
                    width,
                    height,
                    facecolor="#566573",
                    edgecolor="#2c3e50",
                    linewidth=1.0,
                    zorder=1,
                )
            )

        legend_items = [
            Patch(facecolor="#566573", label="Shelf"),
            Patch(facecolor="#e74c3c", alpha=0.45, label="Hazard"),
            Line2D([], [], color="#1976d2", marker="o", linestyle="", label="Executing"),
            Line2D([], [], color="#2e7d32", marker="o", linestyle="", label="Completed"),
            Line2D([], [], color="#f9a825", marker="o", linestyle="", label="Rerouting"),
            Line2D([], [], color="#c62828", marker="o", linestyle="", label="Blocked"),
            Line2D([], [], color="#34495e", marker="*", linestyle="", label="Goal"),
        ]
        axis.legend(handles=legend_items, loc="lower center", ncol=4, fontsize=8)

    def _configure_status(self) -> None:
        self._status_axis.set_title("Fleet Status", fontsize=13, weight="bold")
        self._status_axis.set_axis_off()
        self._status_text = self._status_axis.text(
            0.02,
            0.98,
            "Waiting for robot telemetry...",
            transform=self._status_axis.transAxes,
            va="top",
            family="monospace",
            fontsize=10,
        )

    def _configure_playback_controls(self) -> None:
        timeline_axis = self._figure.add_axes((0.16, 0.045, 0.52, 0.035))
        play_axis = self._figure.add_axes((0.72, 0.035, 0.09, 0.055))
        pause_axis = self._figure.add_axes((0.82, 0.035, 0.09, 0.055))
        self._timeline = Slider(
            timeline_axis,
            "Timeline",
            valmin=0.0,
            valmax=0.1,
            valinit=0.0,
            valfmt="%0.1f s",
        )
        self._play_button = Button(play_axis, "Play")
        self._pause_button = Button(pause_axis, "Pause")
        self._timeline.on_changed(self._on_timeline_changed)
        self._play_button.on_clicked(self._on_play)
        self._pause_button.on_clicked(self._on_pause)

    def _on_timeline_changed(self, selected_time: float) -> None:
        if self._updating_slider or not self._history:
            return
        times = [entry[0] for entry in self._history]
        self._playback_index = max(
            0, min(len(times) - 1, bisect_right(times, selected_time) - 1)
        )
        self._playing = False
        self._follow_live = False

    def _on_play(self, event: Any) -> None:
        if not self._history:
            return
        self._playing = True
        self._follow_live = self._playback_index >= len(self._history) - 1

    def _on_pause(self, event: Any) -> None:
        if self._history and self._follow_live:
            self._playback_index = len(self._history) - 1
        self._playing = False
        self._follow_live = False

    def _on_state(self, message: String) -> None:
        try:
            state = json.loads(message.data)
            robot_id = str(state["robot_id"])
            x = float(state["x"])
            y = float(state["y"])
            lifecycle = str(state["state"])
            if lifecycle not in STATE_COLORS:
                raise ValueError(f"unknown state: {lifecycle}")
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignored invalid robot telemetry: {exc}")
            return

        with self._lock:
            self._states[robot_id] = state

    def _on_hazard(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            hazard = Hazard.from_payload(payload)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignored invalid hazard telemetry: {exc}")
            return
        with self._lock:
            self._hazards[hazard.hazard_id] = payload

    def _record_snapshot(self) -> None:
        with self._lock:
            states = {robot_id: dict(state) for robot_id, state in self._states.items()}
            hazards = {
                hazard_id: dict(hazard) for hazard_id, hazard in self._hazards.items()
            }
        if not states:
            return

        state_signature = tuple(
            (
                robot_id,
                state.get("mission_id"),
                state.get("x"),
                state.get("y"),
                state.get("state"),
                state.get("path_affected"),
                json.dumps(state.get("goal"), sort_keys=True),
            )
            for robot_id, state in sorted(states.items())
        )
        hazard_signature = tuple(
            (hazard_id, json.dumps(hazard, sort_keys=True))
            for hazard_id, hazard in sorted(hazards.items())
        )
        signature = (state_signature, hazard_signature)
        if signature == self._last_recorded_signature:
            return

        elapsed = time.monotonic() - self._recording_started
        self._history.append((elapsed, states, hazards))
        self._last_recorded_signature = signature

    def _select_playback_frame(
        self,
    ) -> tuple[float, dict[str, dict[str, Any]], dict[str, dict[str, Any]]]:
        if self._follow_live:
            self._playback_index = len(self._history) - 1
        elif self._playing:
            self._playback_index = min(
                self._playback_index + 1, len(self._history) - 1
            )
            if self._playback_index == len(self._history) - 1:
                self._follow_live = True
        return self._history[self._playback_index]

    def _trails_at(self, playback_index: int) -> dict[str, list[tuple[float, float]]]:
        trails: dict[str, list[tuple[float, float]]] = {}
        for _, states, _ in self._history[: playback_index + 1]:
            for robot_id, state in states.items():
                point = (float(state["x"]), float(state["y"]))
                robot_trail = trails.setdefault(robot_id, [])
                if not robot_trail or robot_trail[-1] != point:
                    robot_trail.append(point)
        return trails

    def _update_timeline(self, selected_time: float) -> None:
        latest_time = max(0.1, self._history[-1][0])
        self._timeline.valmax = latest_time
        self._timeline.ax.set_xlim(0.0, latest_time)
        self._updating_slider = True
        try:
            self._timeline.set_val(selected_time)
        finally:
            self._updating_slider = False

    def _create_robot_artists(self, robot_id: str) -> dict[str, Any]:
        path_line, = self._map_axis.plot(
            [], [], linestyle="--", linewidth=0.8, color="#95a5a6", zorder=2
        )
        trail_line, = self._map_axis.plot(
            [], [], linewidth=1.8, color="#1976d2", alpha=0.85, zorder=3
        )
        point, = self._map_axis.plot(
            [], [], marker="o", markersize=9, linestyle="", zorder=5
        )
        goal, = self._map_axis.plot(
            [], [], marker="*", markersize=13, linestyle="", color="#34495e", zorder=4
        )
        label = self._map_axis.text(0.0, 0.0, robot_id, fontsize=7, zorder=6)
        return {
            "path": path_line,
            "trail": trail_line,
            "point": point,
            "goal": goal,
            "label": label,
        }

    def _create_hazard_artists(self, hazard: dict[str, Any]) -> tuple[Rectangle, Any]:
        bounds = hazard["bounds"]
        rectangle = Rectangle(
            (float(bounds["x_min"]), float(bounds["y_min"])),
            float(bounds["x_max"]) - float(bounds["x_min"]),
            float(bounds["y_max"]) - float(bounds["y_min"]),
            facecolor="#e74c3c",
            edgecolor="#922b21",
            hatch="///",
            linewidth=2.0,
            alpha=0.45,
            zorder=4,
        )
        self._map_axis.add_patch(rectangle)
        label = self._map_axis.text(
            (float(bounds["x_min"]) + float(bounds["x_max"])) / 2.0,
            float(bounds["y_max"]) + 1.5,
            f"HAZARD: {hazard['hazard_id']}",
            color="#922b21",
            fontsize=9,
            weight="bold",
            ha="center",
            zorder=7,
        )
        return rectangle, label

    def _draw_frame(self, frame: int) -> list[Any]:
        self._record_snapshot()
        if not self._history:
            return [self._status_text]

        selected_time, states, hazards = self._select_playback_frame()
        trails = self._trails_at(self._playback_index)
        self._update_timeline(selected_time)

        for robot_id, artists in self._artists.items():
            visible = robot_id in states
            for artist in artists.values():
                artist.set_visible(visible)

        for hazard_id, artists in self._hazard_artists.items():
            visible = hazard_id in hazards
            for artist in artists:
                artist.set_visible(visible)

        for hazard_id, hazard in sorted(hazards.items()):
            if hazard_id not in self._hazard_artists:
                self._hazard_artists[hazard_id] = self._create_hazard_artists(hazard)

        for robot_id in sorted(states):
            state = states[robot_id]
            if robot_id not in self._artists:
                self._artists[robot_id] = self._create_robot_artists(robot_id)
            artists = self._artists[robot_id]
            x = float(state["x"])
            y = float(state["y"])
            color = STATE_COLORS[str(state["state"])]
            trail = trails.get(robot_id, ())

            artists["point"].set_data([x], [y])
            artists["point"].set_color(color)
            artists["label"].set_position((x + 1.0, y + 1.0))
            artists["trail"].set_data(
                [position[0] for position in trail],
                [position[1] for position in trail],
            )
            artists["trail"].set_color(color)

            goal = state.get("goal")
            if isinstance(goal, dict):
                goal_x = float(goal["x"])
                goal_y = float(goal["y"])
                artists["goal"].set_data([goal_x], [goal_y])
                if trail:
                    artists["path"].set_data(
                        [trail[0][0], goal_x], [trail[0][1], goal_y]
                    )
            else:
                artists["goal"].set_data([], [])
                artists["path"].set_data([], [])

        counts = Counter(str(state["state"]) for state in states.values())
        mode = "LIVE" if self._follow_live else "PLAY" if self._playing else "PAUSED"
        status_lines = [
            f"mode:      {mode:<7}",
            f"time:      {selected_time:5.1f}s",
            "",
            f"robots:     {len(states):2d}",
            f"assigned:   {counts['assigned']:2d}",
            f"executing:  {counts['executing']:2d}",
            f"completed:  {counts['completed']:2d}",
            f"rerouting:  {counts['rerouting']:2d}",
            f"blocked:    {counts['blocked']:2d}",
            f"hazards:    {len(hazards):2d}",
            f"affected:   {sum(bool(state.get('path_affected')) for state in states.values()):2d}",
            "",
        ]
        status_lines.extend(
            f"{robot_id:<7} {str(states[robot_id]['state']):<10}"
            for robot_id in sorted(states)
        )
        self._status_text.set_text("\n".join(status_lines))

        result: list[Any] = [self._status_text]
        for artists in self._artists.values():
            result.extend(artists.values())
        for artists in self._hazard_artists.values():
            result.extend(artists)
        return result

    def show(self) -> None:
        plt.show()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FleetVisualizer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.show()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
