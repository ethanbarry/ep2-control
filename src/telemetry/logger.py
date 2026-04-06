from __future__ import annotations

import csv
import logging
from pathlib import Path
from typing import Any

from src.domain.enums import MissionState
from src.domain.models import BucketMeasurement, TriangulationEstimate, Vector3


class SimulationLogger:
    def __init__(self, artifacts_dir: Path) -> None:
        self.artifacts_dir = artifacts_dir
        self.artifacts_dir.mkdir(parents=True, exist_ok=True)
        self._logger = logging.getLogger(f"demo_drone_pilot.{artifacts_dir.name}")
        self._logger.setLevel(logging.INFO)
        self._logger.propagate = False
        self._logger.handlers.clear()
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
        self._logger.addHandler(handler)

        self.state_rows: list[dict[str, Any]] = []
        self.measurement_rows: list[dict[str, Any]] = []
        self.estimate_rows: list[dict[str, Any]] = []
        self.path_rows: list[dict[str, Any]] = []

    def info(self, message: str) -> None:
        self._logger.info(message)

    def log_state_transition(
        self,
        mission_time_s: float,
        from_state: MissionState,
        to_state: MissionState,
        note: str = "",
    ) -> None:
        row = {
            "mission_time_s": round(mission_time_s, 3),
            "from_state": from_state.value,
            "to_state": to_state.value,
            "note": note,
        }
        self.state_rows.append(row)
        self.info(f"state {from_state.value} -> {to_state.value}{': ' + note if note else ''}")

    def log_measurement(
        self,
        mission_time_s: float,
        scan_label: str,
        position: Vector3,
        measurement: BucketMeasurement,
    ) -> None:
        self.measurement_rows.append(
            {
                "mission_time_s": round(mission_time_s, 3),
                "scan_label": scan_label,
                "x_m": round(position.x, 3),
                "y_m": round(position.y, 3),
                "z_m": round(position.z, 3),
                "yaw_deg": measurement.yaw_deg,
                "bucket_deg": measurement.bucket_deg,
                "correlation_score": round(measurement.correlation_score, 5),
                "rssi_dbm": round(measurement.rssi_dbm, 3),
                "inferred_bearing_deg": round(measurement.inferred_bearing_deg, 3),
            }
        )

    def log_estimate(
        self,
        mission_time_s: float,
        estimate: TriangulationEstimate,
    ) -> None:
        self.estimate_rows.append(
            {
                "mission_time_s": round(mission_time_s, 3),
                "x_m": None if estimate.position is None else round(estimate.position.x, 3),
                "y_m": None if estimate.position is None else round(estimate.position.y, 3),
                "valid": estimate.valid,
                "confidence": round(estimate.confidence, 5),
                "residual_m": round(estimate.residual_m, 5) if estimate.residual_m != float("inf") else "inf",
                "reason": estimate.reason,
            }
        )
        self.info(
            "triangulation "
            f"{'valid' if estimate.valid else 'invalid'}"
            f" confidence={estimate.confidence:.2f} residual={estimate.residual_m:.2f} reason={estimate.reason}"
        )

    def log_path_point(
        self,
        mission_time_s: float,
        label: str,
        point: Vector3,
        is_replay: bool,
    ) -> None:
        self.path_rows.append(
            {
                "mission_time_s": round(mission_time_s, 3),
                "label": label,
                "x_m": round(point.x, 3),
                "y_m": round(point.y, 3),
                "z_m": round(point.z, 3),
                "is_replay": is_replay,
            }
        )

    def flush_csv(self) -> None:
        self._write_csv(
            self.artifacts_dir / "state_transitions.csv",
            self.state_rows,
            ["mission_time_s", "from_state", "to_state", "note"],
        )
        self._write_csv(
            self.artifacts_dir / "measurements.csv",
            self.measurement_rows,
            [
                "mission_time_s",
                "scan_label",
                "x_m",
                "y_m",
                "z_m",
                "yaw_deg",
                "bucket_deg",
                "correlation_score",
                "rssi_dbm",
                "inferred_bearing_deg",
            ],
        )
        self._write_csv(
            self.artifacts_dir / "triangulation_estimates.csv",
            self.estimate_rows,
            ["mission_time_s", "x_m", "y_m", "valid", "confidence", "residual_m", "reason"],
        )
        self._write_csv(
            self.artifacts_dir / "path_trace.csv",
            self.path_rows,
            ["mission_time_s", "label", "x_m", "y_m", "z_m", "is_replay"],
        )

    def _write_csv(self, path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
