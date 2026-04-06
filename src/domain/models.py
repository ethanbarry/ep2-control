from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from src.domain.enums import DspFailureMode, MissionState


@dataclass(frozen=True)
class Vector3:
    x: float
    y: float
    z: float = 0.0

    def distance_to(self, other: "Vector3") -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    def xy_distance_to(self, other: "Vector3") -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        return (dx * dx + dy * dy) ** 0.5

    def interpolate(self, other: "Vector3", ratio: float) -> "Vector3":
        assert 0.0 <= ratio <= 1.0
        return Vector3(
            x=self.x + (other.x - self.x) * ratio,
            y=self.y + (other.y - self.y) * ratio,
            z=self.z + (other.z - self.z) * ratio,
        )


@dataclass(frozen=True)
class Pose:
    position: Vector3
    yaw_deg: float


@dataclass(frozen=True)
class FoxConfig:
    position: Vector3
    search_radius_m: float = 0.0
    carrier_frequency_hz: float = 915_000_000.0
    tx_power_dbm: float = 14.0
    path_loss_exponent: float = 2.0
    antenna_front_gain_db: float = 9.0
    antenna_beamwidth_deg: float = 60.0


@dataclass(frozen=True)
class DspConfig:
    noise_std_db: float = 1.25
    outlier_probability: float = 0.0
    outlier_magnitude_db: float = 10.0
    timeout_probability: float = 0.0
    crash_after_samples: int | None = None
    timeout_after_samples: int | None = None
    failure_mode: DspFailureMode = DspFailureMode.NORMAL
    forced_bearing_bias_deg: float = 0.0


@dataclass(frozen=True)
class BatteryConfig:
    start_pct: float = 100.0
    low_battery_threshold_pct: float = 25.0
    drain_per_meter_pct: float = 0.08
    drain_per_spin_sample_pct: float = 0.2
    drain_per_takeoff_pct: float = 2.0
    drain_per_hover_second_pct: float = 0.03


@dataclass(frozen=True)
class MissionConfig:
    scan_altitude_m: float = 20.0
    spin_bucket_deg: int = 10
    offset_distance_m: float = 70.0
    refinement_passes: int = 2
    refinement_offset_distance_m: float = 25.0
    refinement_convergence_distance_m: float = 4.0
    cruise_speed_m_s: float = 8.0
    scan_sample_period_s: float = 0.4
    verification_radius_m: float = 20.0
    verify_target_buckets: int = 6
    prefer_left_perpendicular: bool = True
    position_tolerance_m: float = 2.0
    min_intersection_angle_deg: float = 20.0
    max_triangulation_residual_m: float = 15.0
    max_target_error_m: float = 30.0
    hold_after_failure_s: float = 5.0


@dataclass(frozen=True)
class FailureConfig:
    invalid_triangulation: bool = False


@dataclass(frozen=True)
class ScenarioConfig:
    name: str
    description: str
    home_position: Vector3
    fox: FoxConfig
    mission: MissionConfig = field(default_factory=MissionConfig)
    battery: BatteryConfig = field(default_factory=BatteryConfig)
    dsp: DspConfig = field(default_factory=DspConfig)
    failures: FailureConfig = field(default_factory=FailureConfig)
    random_seed: int = 7


@dataclass
class BatteryState:
    current_pct: float

    def consume(self, amount_pct: float) -> None:
        assert amount_pct >= 0.0
        self.current_pct = max(0.0, self.current_pct - amount_pct)

    def is_low(self, threshold_pct: float) -> bool:
        return self.current_pct <= threshold_pct


@dataclass(frozen=True)
class BucketMeasurement:
    yaw_deg: int
    bucket_deg: int
    correlation_score: float
    rssi_dbm: float
    inferred_bearing_deg: float


@dataclass(frozen=True)
class ScanBucket:
    bucket_deg: int
    sample_count: int
    average_rssi_dbm: float
    average_correlation: float


@dataclass(frozen=True)
class ScanResult:
    scan_label: str
    origin: Vector3
    bucket_interval_deg: int
    buckets: list[ScanBucket]
    strongest_bucket_deg: int
    estimated_bearing_deg: float
    strongest_rssi_dbm: float


@dataclass(frozen=True)
class BearingObservation:
    origin: Vector3
    bearing_deg: float
    confidence: float
    source_label: str


@dataclass(frozen=True)
class TriangulationEstimate:
    position: Vector3 | None
    valid: bool
    confidence: float
    residual_m: float
    reason: str


@dataclass
class MissionOutcome:
    final_state: MissionState
    state_history: list[MissionState]
    battery_remaining_pct: float
    actual_path: list[Vector3]
    replay_path: list[Vector3]
    scan_results: list[ScanResult]
    triangulation_estimates: list[TriangulationEstimate]
    failure_reason: str | None
    returned_home_via_replay: bool
    artifacts_dir: Path


MILES_TO_METERS = 1609.344


def _vector3_from_mapping(raw: dict[str, Any]) -> Vector3:
    return Vector3(
        x=float(raw["x"]),
        y=float(raw["y"]),
        z=float(raw.get("z", 0.0)),
    )


def _enum_from_value(value: str, enum_type: type[DspFailureMode]) -> DspFailureMode:
    return enum_type(value)


def _fox_location_from_mapping(
    fox_raw: dict[str, Any],
    home_position: Vector3,
    random_seed: int,
) -> tuple[Vector3, float]:
    randomize_within_miles = float(fox_raw.get("randomize_within_miles", 0.0))
    position_raw = fox_raw.get("position")
    if randomize_within_miles <= 0.0:
        assert isinstance(position_raw, dict), (
            "fox.position is required unless fox.randomize_within_miles is set"
        )
        return _vector3_from_mapping(position_raw), 0.0

    radius_m = randomize_within_miles * MILES_TO_METERS
    fox_z = 0.0 if not isinstance(position_raw, dict) else float(position_raw.get("z", 0.0))
    rng = random.Random(random_seed)
    distance_m = radius_m * math.sqrt(rng.random())
    heading_rad = 2.0 * math.pi * rng.random()
    return (
        Vector3(
            x=home_position.x + distance_m * math.cos(heading_rad),
            y=home_position.y + distance_m * math.sin(heading_rad),
            z=fox_z,
        ),
        radius_m,
    )


def scenario_from_dict(raw: dict[str, Any]) -> ScenarioConfig:
    random_seed = int(raw.get("random_seed", 7))
    home_position = _vector3_from_mapping(raw["home_position"])
    fox_raw = raw["fox"]
    mission_raw = raw.get("mission", {})
    battery_raw = raw.get("battery", {})
    dsp_raw = raw.get("dsp", {})
    failure_raw = raw.get("failures", {})
    fox_position, fox_search_radius_m = _fox_location_from_mapping(fox_raw, home_position, random_seed)
    return ScenarioConfig(
        name=str(raw["name"]),
        description=str(raw.get("description", "")),
        home_position=home_position,
        fox=FoxConfig(
            position=fox_position,
            search_radius_m=fox_search_radius_m,
            carrier_frequency_hz=float(fox_raw.get("carrier_frequency_hz", 915_000_000.0)),
            tx_power_dbm=float(fox_raw.get("tx_power_dbm", 14.0)),
            path_loss_exponent=float(fox_raw.get("path_loss_exponent", 2.0)),
            antenna_front_gain_db=float(fox_raw.get("antenna_front_gain_db", 9.0)),
            antenna_beamwidth_deg=float(fox_raw.get("antenna_beamwidth_deg", 60.0)),
        ),
        mission=MissionConfig(
            scan_altitude_m=float(mission_raw.get("scan_altitude_m", 20.0)),
            spin_bucket_deg=int(mission_raw.get("spin_bucket_deg", 10)),
            offset_distance_m=float(mission_raw.get("offset_distance_m", 70.0)),
            refinement_passes=int(mission_raw.get("refinement_passes", 2)),
            refinement_offset_distance_m=float(mission_raw.get("refinement_offset_distance_m", 25.0)),
            refinement_convergence_distance_m=float(
                mission_raw.get("refinement_convergence_distance_m", 4.0)
            ),
            cruise_speed_m_s=float(mission_raw.get("cruise_speed_m_s", 8.0)),
            scan_sample_period_s=float(mission_raw.get("scan_sample_period_s", 0.4)),
            verification_radius_m=float(mission_raw.get("verification_radius_m", 20.0)),
            verify_target_buckets=int(mission_raw.get("verify_target_buckets", 6)),
            prefer_left_perpendicular=bool(mission_raw.get("prefer_left_perpendicular", True)),
            position_tolerance_m=float(mission_raw.get("position_tolerance_m", 2.0)),
            min_intersection_angle_deg=float(mission_raw.get("min_intersection_angle_deg", 20.0)),
            max_triangulation_residual_m=float(mission_raw.get("max_triangulation_residual_m", 15.0)),
            max_target_error_m=float(mission_raw.get("max_target_error_m", 30.0)),
            hold_after_failure_s=float(mission_raw.get("hold_after_failure_s", 5.0)),
        ),
        battery=BatteryConfig(
            start_pct=float(battery_raw.get("start_pct", 100.0)),
            low_battery_threshold_pct=float(battery_raw.get("low_battery_threshold_pct", 25.0)),
            drain_per_meter_pct=float(battery_raw.get("drain_per_meter_pct", 0.08)),
            drain_per_spin_sample_pct=float(battery_raw.get("drain_per_spin_sample_pct", 0.2)),
            drain_per_takeoff_pct=float(battery_raw.get("drain_per_takeoff_pct", 2.0)),
            drain_per_hover_second_pct=float(battery_raw.get("drain_per_hover_second_pct", 0.03)),
        ),
        dsp=DspConfig(
            noise_std_db=float(dsp_raw.get("noise_std_db", 1.25)),
            outlier_probability=float(dsp_raw.get("outlier_probability", 0.0)),
            outlier_magnitude_db=float(dsp_raw.get("outlier_magnitude_db", 10.0)),
            timeout_probability=float(dsp_raw.get("timeout_probability", 0.0)),
            crash_after_samples=(
                int(dsp_raw["crash_after_samples"])
                if dsp_raw.get("crash_after_samples") is not None
                else None
            ),
            timeout_after_samples=(
                int(dsp_raw["timeout_after_samples"])
                if dsp_raw.get("timeout_after_samples") is not None
                else None
            ),
            failure_mode=_enum_from_value(
                str(dsp_raw.get("failure_mode", DspFailureMode.NORMAL.value)),
                DspFailureMode,
            ),
            forced_bearing_bias_deg=float(dsp_raw.get("forced_bearing_bias_deg", 0.0)),
        ),
        failures=FailureConfig(
            invalid_triangulation=bool(failure_raw.get("invalid_triangulation", False))
        ),
        random_seed=random_seed,
    )
