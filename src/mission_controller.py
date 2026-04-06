from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

from src.domain.enums import MissionState
from src.domain.models import (
    BatteryState,
    BearingObservation,
    MissionOutcome,
    ScenarioConfig,
    ScanResult,
    TriangulationEstimate,
    Vector3,
)
from src.flight.path_recorder import PathRecorder
from src.flight.rth_replay import RthReplayPlanner
from src.flight.state_machine import MissionStateMachine
from src.flight.waypoint_planner import WaypointPlanner
from src.mavlink_client import BaseMavlinkClient
from src.rf.bucket_sampler import BucketSampler
from src.rf.dsp_interface import DspPipeline, DspPipelineError, DspTimeoutError
from src.rf.triangulation import BearingTriangulator
from src.telemetry.logger import SimulationLogger


class LowBatteryTriggered(RuntimeError):
    """Raised when the simulated battery crosses the low threshold."""


@dataclass
class MissionController:
    scenario: ScenarioConfig
    client: BaseMavlinkClient
    dsp_pipeline: DspPipeline
    logger: SimulationLogger
    artifacts_dir: Path

    def __post_init__(self) -> None:
        self.state_machine = MissionStateMachine()
        self.bucket_sampler = BucketSampler(self.scenario.mission.spin_bucket_deg)
        self.waypoint_planner = WaypointPlanner()
        self.path_recorder = PathRecorder()
        self.rth_replay = RthReplayPlanner()
        self.triangulator = BearingTriangulator()
        self.battery = BatteryState(self.scenario.battery.start_pct)
        self.scan_results: list[ScanResult] = []
        self.triangulation_estimates: list[TriangulationEstimate] = []
        self.replay_path: list[Vector3] = []
        self.failure_reason: str | None = None
        self.mission_time_s = 0.0
        self.home_position = Vector3(
            self.scenario.home_position.x,
            self.scenario.home_position.y,
            self.scenario.home_position.z,
        )
        self.returned_home_via_replay = False

    def run(self) -> MissionOutcome:
        try:
            self._transition(MissionState.WAIT_FOR_GUIDED, "connecting to vehicle")
            self.client.connect()
            self.client.set_guided_mode()

            self._transition(MissionState.ACQUIRE_HOME, "arming and taking off")
            self.path_recorder.append(self.home_position)
            self.logger.log_path_point(self.mission_time_s, "home", self.home_position, False)
            self.client.arm_and_takeoff(self.scenario.mission.scan_altitude_m)
            self._consume_takeoff()
            takeoff_pose = self.client.get_pose()
            self.path_recorder.append(takeoff_pose.position)
            self.logger.log_path_point(self.mission_time_s, "takeoff", takeoff_pose.position, False)
            self._ensure_battery("after takeoff")

            self._transition(MissionState.SPIN_SCAN_1, "collecting first bearing sweep")
            scan_1 = self._perform_spin_scan("scan_1")
            self.scan_results.append(scan_1)
            observation_1 = self._observation_from_scan(scan_1)
            observations = [observation_1]

            self._transition(MissionState.COMPUTE_PERPENDICULAR_OFFSET, "planning offset waypoint")
            offset_waypoint = self.waypoint_planner.compute_perpendicular_offset(
                origin=scan_1.origin,
                strongest_bearing_deg=observation_1.bearing_deg,
                distance_m=self._offset_distance_m(),
                prefer_left=self.scenario.mission.prefer_left_perpendicular,
            )

            self._transition(MissionState.TRANSIT_TO_OFFSET, "flying to perpendicular offset")
            offset_waypoint = Vector3(offset_waypoint.x, offset_waypoint.y, self.scenario.mission.scan_altitude_m)
            self._fly_to_waypoint("offset_waypoint", offset_waypoint)

            self._transition(MissionState.SPIN_SCAN_2, "collecting second bearing sweep")
            scan_2 = self._perform_spin_scan("scan_2")
            self.scan_results.append(scan_2)
            observation_2 = self._observation_from_scan(scan_2)
            observations.append(observation_2)

            self._transition(MissionState.TRIANGULATE, "estimating fox position")
            estimate = self._triangulate(observations)
            if not estimate.valid or estimate.position is None:
                self.failure_reason = estimate.reason
                self._enter_safe_hold(estimate.reason)
                return self._build_outcome()

            target_waypoint = self._target_waypoint_from_estimate(estimate)
            self._transition(MissionState.TRANSIT_TO_TARGET, "flying toward triangulated position")
            self._fly_to_waypoint("triangulated_target", target_waypoint)
            refined_target = self._execute_refinement_passes(observations, target_waypoint)
            if refined_target is None:
                return self._build_outcome()
            target_waypoint = refined_target

            self._transition(MissionState.VERIFY_TARGET, "verifying target solution")
            if not self._verify_target(target_waypoint):
                return self._build_outcome()

            self._transition(MissionState.COMPLETE, "mission complete")
            return self._build_outcome()
        except LowBatteryTriggered as exc:
            self.failure_reason = str(exc)
            self._replay_home_path(str(exc))
            return self._build_outcome()
        except DspTimeoutError as exc:
            self.failure_reason = str(exc)
            self._enter_safe_hold(str(exc))
            return self._build_outcome()
        except DspPipelineError as exc:
            self.failure_reason = str(exc)
            self._abort_to_stabilize(str(exc))
            return self._build_outcome()
        finally:
            self.logger.flush_csv()
            self.client.close()

    def _observation_from_scan(self, scan: ScanResult) -> BearingObservation:
        return BearingObservation(
            origin=scan.origin,
            bearing_deg=scan.estimated_bearing_deg,
            confidence=self._scan_confidence(scan),
            source_label=scan.scan_label,
        )

    def _triangulate(self, observations: list[BearingObservation]) -> TriangulationEstimate:
        estimate = self.triangulator.estimate(
            observations=observations,
            min_intersection_angle_deg=self.scenario.mission.min_intersection_angle_deg,
            max_residual_m=self.scenario.mission.max_triangulation_residual_m,
            force_invalid=self.scenario.failures.invalid_triangulation,
        )
        self.triangulation_estimates.append(estimate)
        self.logger.log_estimate(self.mission_time_s, estimate)
        return estimate

    def _target_waypoint_from_estimate(self, estimate: TriangulationEstimate) -> Vector3:
        assert estimate.position is not None, "estimate must have a position to form a waypoint"
        return Vector3(
            x=estimate.position.x,
            y=estimate.position.y,
            z=self.scenario.mission.scan_altitude_m,
        )

    def _execute_refinement_passes(
        self,
        observations: list[BearingObservation],
        current_target: Vector3,
    ) -> Vector3 | None:
        assert self.scenario.mission.refinement_passes >= 0, "refinement passes must be non-negative"
        assert (
            self.scenario.mission.refinement_offset_distance_m > 0.0
        ), "refinement offset distance must be positive"
        assert (
            self.scenario.mission.refinement_convergence_distance_m >= 0.0
        ), "refinement convergence distance must be non-negative"
        for pass_index in range(1, self.scenario.mission.refinement_passes + 1):
            self.logger.info(f"starting refinement pass {pass_index}")
            anchor_scan = self._perform_spin_scan(f"refinement_{pass_index}_anchor")
            self.scan_results.append(anchor_scan)
            anchor_observation = self._observation_from_scan(anchor_scan)
            observations.append(anchor_observation)

            prefer_left = self.scenario.mission.prefer_left_perpendicular if pass_index % 2 == 1 else (
                not self.scenario.mission.prefer_left_perpendicular
            )
            refinement_offset = self.waypoint_planner.compute_perpendicular_offset(
                origin=anchor_scan.origin,
                strongest_bearing_deg=anchor_observation.bearing_deg,
                distance_m=self.scenario.mission.refinement_offset_distance_m,
                prefer_left=prefer_left,
            )
            refinement_offset = Vector3(
                refinement_offset.x,
                refinement_offset.y,
                self.scenario.mission.scan_altitude_m,
            )
            self._fly_to_waypoint(f"refinement_offset_{pass_index}", refinement_offset)

            offset_scan = self._perform_spin_scan(f"refinement_{pass_index}_offset")
            self.scan_results.append(offset_scan)
            offset_observation = self._observation_from_scan(offset_scan)
            observations.append(offset_observation)

            estimate = self._triangulate(observations)
            if not estimate.valid or estimate.position is None:
                self.failure_reason = estimate.reason
                self._enter_safe_hold(estimate.reason)
                return None

            refined_target = self._target_waypoint_from_estimate(estimate)
            target_shift_m = current_target.xy_distance_to(refined_target)
            self.logger.info(
                f"refinement pass {pass_index} shifted target by {target_shift_m:.2f} m"
            )
            self._fly_to_waypoint(f"refined_target_{pass_index}", refined_target)
            current_target = refined_target
            if target_shift_m <= self.scenario.mission.refinement_convergence_distance_m:
                self.logger.info(
                    f"refinement converged after pass {pass_index} with shift {target_shift_m:.2f} m"
                )
                break
        return current_target

    def _perform_spin_scan(self, scan_label: str) -> ScanResult:
        pose = self.client.get_pose()

        def provider(yaw_deg: int):
            self.client.condition_yaw(float(yaw_deg))
            current_pose = self.client.get_pose()
            measurement = self.dsp_pipeline.measure(current_pose, self.scenario.fox, yaw_deg)
            self.logger.log_measurement(self.mission_time_s, scan_label, current_pose.position, measurement)
            self._advance_time(self.scenario.mission.scan_sample_period_s)
            self._consume_hover(self.scenario.mission.scan_sample_period_s)
            self.battery.consume(self.scenario.battery.drain_per_spin_sample_pct)
            self._ensure_battery(f"during {scan_label}")
            return measurement

        scan_result, _ = self.bucket_sampler.collect_spin(
            scan_label=scan_label,
            origin=pose.position,
            provider=provider,
        )
        self.logger.info(
            f"{scan_label} strongest bucket={scan_result.strongest_bucket_deg} deg "
            f"estimated bearing={scan_result.estimated_bearing_deg:.2f} deg "
            f"rssi={scan_result.strongest_rssi_dbm:.2f} dBm"
        )
        return scan_result

    def _scan_confidence(self, scan: ScanResult) -> float:
        ordered = sorted(scan.buckets, key=lambda bucket: bucket.average_rssi_dbm, reverse=True)
        if len(ordered) == 1:
            return 1.0
        margin = ordered[0].average_rssi_dbm - ordered[1].average_rssi_dbm
        return max(0.1, min(1.0, 0.5 + margin / 12.0))

    def _offset_distance_m(self) -> float:
        if self.scenario.fox.search_radius_m <= 0.0:
            return self.scenario.mission.offset_distance_m
        buffered_angle_deg = min(89.0, self.scenario.mission.min_intersection_angle_deg + 5.0)
        required_offset_m = self.scenario.fox.search_radius_m * math.sin(math.radians(buffered_angle_deg))
        return max(self.scenario.mission.offset_distance_m, required_offset_m)

    def _fly_to_waypoint(self, label: str, waypoint: Vector3) -> None:
        distance_m = self.client.goto_position(waypoint)
        self._advance_time(distance_m / max(0.1, self.scenario.mission.cruise_speed_m_s))
        self.battery.consume(distance_m * self.scenario.battery.drain_per_meter_pct)
        self.path_recorder.append(waypoint)
        self.logger.log_path_point(self.mission_time_s, label, waypoint, False)
        self._ensure_battery(f"after reaching {label}")

    def _verify_target(self, target_waypoint: Vector3) -> bool:
        verification_error_m = target_waypoint.xy_distance_to(self.scenario.fox.position)
        self._advance_time(self.scenario.mission.verify_target_buckets * self.scenario.mission.scan_sample_period_s)
        self._consume_hover(self.scenario.mission.verify_target_buckets * self.scenario.mission.scan_sample_period_s)
        if verification_error_m > self.scenario.mission.max_target_error_m:
            reason = (
                f"target verification failed: estimate error {verification_error_m:.2f} m "
                f"exceeds {self.scenario.mission.max_target_error_m:.2f} m"
            )
            self.failure_reason = reason
            self._enter_safe_hold(reason)
            return False
        return True

    def _replay_home_path(self, reason: str) -> None:
        self._transition(MissionState.RETURN_TO_HOME_REPLAY, reason)
        history = self.path_recorder.snapshot()
        if not history:
            history = [self.home_position]
        replay_path = self.rth_replay.build_replay_path(history, self.home_position)
        self.replay_path = replay_path
        self.returned_home_via_replay = True
        for index, waypoint in enumerate(replay_path):
            if self.client.get_pose().position == waypoint:
                continue
            distance_m = self.client.goto_position(waypoint)
            self._advance_time(distance_m / max(0.1, self.scenario.mission.cruise_speed_m_s))
            self.logger.log_path_point(self.mission_time_s, f"rth_replay_{index}", waypoint, True)
        self._transition(MissionState.COMPLETE, "returned home by replay")

    def _enter_safe_hold(self, reason: str) -> None:
        if self.state_machine.current_state != MissionState.SAFE_HOLD:
            self._transition(MissionState.SAFE_HOLD, reason)
        self.client.hold_position()
        self._advance_time(self.scenario.mission.hold_after_failure_s)
        self._consume_hover(self.scenario.mission.hold_after_failure_s)

    def _abort_to_stabilize(self, reason: str) -> None:
        self._enter_safe_hold(reason)
        if self.state_machine.current_state != MissionState.ABORT_TO_STABILIZE:
            self._transition(MissionState.ABORT_TO_STABILIZE, "hard fault abort")

    def _consume_takeoff(self) -> None:
        self.battery.consume(self.scenario.battery.drain_per_takeoff_pct)
        self._advance_time(self.scenario.mission.scan_altitude_m / max(0.1, self.scenario.mission.cruise_speed_m_s))

    def _consume_hover(self, seconds: float) -> None:
        self.battery.consume(seconds * self.scenario.battery.drain_per_hover_second_pct)

    def _advance_time(self, seconds: float) -> None:
        self.mission_time_s += max(0.0, seconds)

    def _ensure_battery(self, note: str) -> None:
        if self.battery.is_low(self.scenario.battery.low_battery_threshold_pct):
            raise LowBatteryTriggered(
                f"low battery at {self.battery.current_pct:.2f}% {note}; replaying mission path home"
            )

    def _transition(self, next_state: MissionState, note: str = "") -> None:
        previous = self.state_machine.current_state
        self.state_machine.transition_to(next_state)
        self.logger.log_state_transition(self.mission_time_s, previous, next_state, note)

    def _build_outcome(self) -> MissionOutcome:
        return MissionOutcome(
            final_state=self.state_machine.current_state,
            state_history=list(self.state_machine.history),
            battery_remaining_pct=self.battery.current_pct,
            actual_path=self.client.actual_path,
            replay_path=list(self.replay_path),
            scan_results=list(self.scan_results),
            triangulation_estimates=list(self.triangulation_estimates),
            failure_reason=self.failure_reason,
            returned_home_via_replay=self.returned_home_via_replay,
            artifacts_dir=self.artifacts_dir,
        )
