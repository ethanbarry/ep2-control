from __future__ import annotations

import math

from src.domain.models import BearingObservation, TriangulationEstimate, Vector3
from src.rf.signal_model import shortest_angular_distance_deg


class BearingTriangulator:
    def estimate(
        self,
        observations: list[BearingObservation],
        min_intersection_angle_deg: float,
        max_residual_m: float,
        force_invalid: bool = False,
    ) -> TriangulationEstimate:
        assert len(observations) >= 2, "triangulation requires at least two bearing observations"
        best_angle = self._best_pairwise_angle_deg(observations)
        if force_invalid:
            return TriangulationEstimate(
                position=None,
                valid=False,
                confidence=0.0,
                residual_m=float("inf"),
                reason="scenario forced invalid triangulation",
            )
        if best_angle < min_intersection_angle_deg:
            return TriangulationEstimate(
                position=None,
                valid=False,
                confidence=0.0,
                residual_m=float("inf"),
                reason=f"bearing lines too parallel ({best_angle:.1f} deg best angle)",
            )

        a00 = 0.0
        a01 = 0.0
        a11 = 0.0
        b0 = 0.0
        b1 = 0.0
        for observation in observations:
            theta = math.radians(observation.bearing_deg)
            nx = -math.sin(theta)
            ny = math.cos(theta)
            rhs = nx * observation.origin.x + ny * observation.origin.y
            weight = max(0.05, observation.confidence)
            a00 += weight * nx * nx
            a01 += weight * nx * ny
            a11 += weight * ny * ny
            b0 += weight * rhs * nx
            b1 += weight * rhs * ny

        determinant = a00 * a11 - a01 * a01
        if abs(determinant) < 1e-6:
            return TriangulationEstimate(
                position=None,
                valid=False,
                confidence=0.0,
                residual_m=float("inf"),
                reason="least-squares system is singular",
            )

        estimate_x = (b0 * a11 - b1 * a01) / determinant
        estimate_y = (a00 * b1 - a01 * b0) / determinant
        estimate = Vector3(x=estimate_x, y=estimate_y, z=0.0)
        residuals = [self._point_to_line_distance_m(estimate, obs) for obs in observations]
        average_residual = sum(residuals) / len(residuals)
        if average_residual > max_residual_m:
            return TriangulationEstimate(
                position=estimate,
                valid=False,
                confidence=0.0,
                residual_m=average_residual,
                reason=f"triangulation residual too high ({average_residual:.2f} m)",
            )

        confidence = min(1.0, max(0.0, (best_angle / 90.0) * (1.0 / (1.0 + average_residual))))
        return TriangulationEstimate(
            position=estimate,
            valid=True,
            confidence=confidence,
            residual_m=average_residual,
            reason="triangulation stable",
        )

    def _point_to_line_distance_m(self, point: Vector3, observation: BearingObservation) -> float:
        theta = math.radians(observation.bearing_deg)
        nx = -math.sin(theta)
        ny = math.cos(theta)
        rhs = nx * observation.origin.x + ny * observation.origin.y
        return abs(nx * point.x + ny * point.y - rhs)

    def _best_pairwise_angle_deg(self, observations: list[BearingObservation]) -> float:
        best_angle = 0.0
        for index, first in enumerate(observations):
            for second in observations[index + 1 :]:
                delta = abs(shortest_angular_distance_deg(first.bearing_deg, second.bearing_deg))
                acute_angle = min(delta, abs(180.0 - delta))
                best_angle = max(best_angle, acute_angle)
        return best_angle
