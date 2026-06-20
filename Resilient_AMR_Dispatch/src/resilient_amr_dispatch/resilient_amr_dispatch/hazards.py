"""Hazard message model and geometric path-intersection helpers."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any


@dataclass(frozen=True)
class Bounds:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def __post_init__(self) -> None:
        if self.x_min > self.x_max or self.y_min > self.y_max:
            raise ValueError("hazard bounds minimum must not exceed maximum")

    def intersects_segment(
        self, start: tuple[float, float], end: tuple[float, float]
    ) -> bool:
        """Return whether a line segment intersects this axis-aligned box."""
        t_min = 0.0
        t_max = 1.0
        for start_value, end_value, lower, upper in (
            (start[0], end[0], self.x_min, self.x_max),
            (start[1], end[1], self.y_min, self.y_max),
        ):
            delta = end_value - start_value
            if delta == 0.0:
                if start_value < lower or start_value > upper:
                    return False
                continue
            near = (lower - start_value) / delta
            far = (upper - start_value) / delta
            if near > far:
                near, far = far, near
            t_min = max(t_min, near)
            t_max = min(t_max, far)
            if t_min > t_max:
                return False
        return True

    def as_dict(self) -> dict[str, float]:
        return {
            "x_min": self.x_min,
            "x_max": self.x_max,
            "y_min": self.y_min,
            "y_max": self.y_max,
        }


@dataclass(frozen=True)
class Hazard:
    hazard_id: str
    hazard_type: str
    bounds: Bounds
    severity: str = "blocked"

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "Hazard":
        bounds = payload["bounds"]
        return cls(
            hazard_id=str(payload["hazard_id"]),
            hazard_type=str(payload["type"]),
            severity=str(payload["severity"]),
            bounds=Bounds(
                x_min=float(bounds["x_min"]),
                x_max=float(bounds["x_max"]),
                y_min=float(bounds["y_min"]),
                y_max=float(bounds["y_max"]),
            ),
        )

    def as_payload(self, timestamp: float | None = None) -> dict[str, Any]:
        return {
            "hazard_id": self.hazard_id,
            "type": self.hazard_type,
            "bounds": self.bounds.as_dict(),
            "severity": self.severity,
            "timestamp": time.time() if timestamp is None else timestamp,
        }


PHASE_3_SPILL = Hazard(
    hazard_id="spill_001",
    hazard_type="hazmat_spill",
    bounds=Bounds(x_min=45.0, x_max=55.0, y_min=30.0, y_max=70.0),
)
