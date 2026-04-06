from __future__ import annotations

import math

from src.domain.models import Vector3
from src.rf.signal_model import normalize_angle_deg


# class to calc waypoints for the mission.
# for example, given the strongest signal 
# bucket/direction, calc a perpendicular offset point for the drone to travel on
# to get the next spot to scan from.
class WaypointPlanner:
    def compute_perpendicular_offset(
        self,
        origin: Vector3,
        strongest_bearing_deg: float,
        distance_m: float,
        prefer_left: bool,
    ) -> Vector3:
        assert distance_m > 0.0
        turn_deg = 90.0 if prefer_left else -90.0
        heading_deg = normalize_angle_deg(strongest_bearing_deg + turn_deg)
        return self.project(origin, heading_deg, distance_m)

    def project(self, origin: Vector3, bearing_deg: float, distance_m: float) -> Vector3:
        radians = math.radians(bearing_deg)
        return Vector3(
            x=origin.x + distance_m * math.cos(radians),
            y=origin.y + distance_m * math.sin(radians),
            z=origin.z,
        )
