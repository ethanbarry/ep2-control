from __future__ import annotations

from dataclasses import dataclass, field

from src.domain.models import Vector3

# class to record the path of the drone. Could use for plotting or in case the drone needs to return

@dataclass
class PathRecorder:
    waypoints: list[Vector3] = field(default_factory=list)

    def append(self, waypoint: Vector3) -> None:
        if self.waypoints and self.waypoints[-1] == waypoint:
            return
        self.waypoints.append(waypoint)

    def snapshot(self) -> list[Vector3]:
        return list(self.waypoints)
