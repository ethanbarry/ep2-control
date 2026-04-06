from __future__ import annotations

from src.domain.models import BearingObservation, Vector3
from src.rf.triangulation import BearingTriangulator


def test_triangulation_solves_expected_intersection() -> None:
    triangulator = BearingTriangulator()
    observations = [
        BearingObservation(origin=Vector3(0.0, 0.0, 20.0), bearing_deg=45.0, confidence=1.0, source_label="scan_1"),
        BearingObservation(origin=Vector3(10.0, 0.0, 20.0), bearing_deg=90.0, confidence=1.0, source_label="scan_2"),
    ]

    estimate = triangulator.estimate(observations, min_intersection_angle_deg=20.0, max_residual_m=5.0)

    assert estimate.valid
    assert estimate.position is not None
    assert abs(estimate.position.x - 10.0) < 0.01
    assert abs(estimate.position.y - 10.0) < 0.01


def test_triangulation_rejects_parallel_lines() -> None:
    triangulator = BearingTriangulator()
    observations = [
        BearingObservation(origin=Vector3(0.0, 0.0, 20.0), bearing_deg=10.0, confidence=1.0, source_label="scan_1"),
        BearingObservation(origin=Vector3(20.0, 0.0, 20.0), bearing_deg=12.0, confidence=1.0, source_label="scan_2"),
    ]

    estimate = triangulator.estimate(observations, min_intersection_angle_deg=20.0, max_residual_m=5.0)

    assert not estimate.valid
    assert estimate.position is None
    assert "parallel" in estimate.reason


def test_triangulation_accepts_redundant_observation_when_good_geometry_exists() -> None:
    triangulator = BearingTriangulator()
    observations = [
        BearingObservation(origin=Vector3(0.0, 0.0, 20.0), bearing_deg=45.0, confidence=1.0, source_label="scan_1"),
        BearingObservation(origin=Vector3(10.0, 0.0, 20.0), bearing_deg=90.0, confidence=1.0, source_label="scan_2"),
        BearingObservation(origin=Vector3(20.0, 20.0, 20.0), bearing_deg=225.0, confidence=1.0, source_label="scan_3"),
    ]

    estimate = triangulator.estimate(observations, min_intersection_angle_deg=20.0, max_residual_m=5.0)

    assert estimate.valid
    assert estimate.position is not None
    assert abs(estimate.position.x - 10.0) < 0.05
    assert abs(estimate.position.y - 10.0) < 0.05
