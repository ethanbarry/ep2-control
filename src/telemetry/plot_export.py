from __future__ import annotations

import math
from pathlib import Path

import matplotlib.pyplot as plt

from src.domain.models import MissionOutcome, ScenarioConfig


def export_plot(scenario: ScenarioConfig, outcome: MissionOutcome, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    figure, axis = plt.subplots(figsize=(10, 8))
    axis.set_title(f"Fox-Hunt Mission: {scenario.name}")
    axis.set_xlabel("X (m)")
    axis.set_ylabel("Y (m)")
    axis.grid(True, alpha=0.3)
    axis.set_aspect("equal", adjustable="box")

    if outcome.actual_path:
        axis.plot(
            [point.x for point in outcome.actual_path],
            [point.y for point in outcome.actual_path],
            marker="o",
            linewidth=2.0,
            label="Flown path",
        )

    if outcome.replay_path:
        axis.plot(
            [point.x for point in outcome.replay_path],
            [point.y for point in outcome.replay_path],
            marker="x",
            linestyle="--",
            linewidth=1.5,
            label="RTH replay",
        )

    axis.scatter(
        [scenario.home_position.x],
        [scenario.home_position.y],
        marker="s",
        s=90,
        label="Home",
    )
    axis.scatter(
        [scenario.fox.position.x],
        [scenario.fox.position.y],
        marker="*",
        s=150,
        label="Fox",
    )

    for scan in outcome.scan_results:
        axis.scatter([scan.origin.x], [scan.origin.y], s=60, label=f"{scan.scan_label} origin")
        ray_length = max(40.0, scenario.mission.offset_distance_m)
        angle_rad = math.radians(scan.strongest_bucket_deg)
        axis.plot(
            [scan.origin.x, scan.origin.x + ray_length * math.cos(angle_rad)],
            [scan.origin.y, scan.origin.y + ray_length * math.sin(angle_rad)],
            linewidth=1.0,
            alpha=0.8,
        )

    valid_estimates = [estimate for estimate in outcome.triangulation_estimates if estimate.position is not None]
    if valid_estimates:
        axis.scatter(
            [estimate.position.x for estimate in valid_estimates],
            [estimate.position.y for estimate in valid_estimates],
            marker="D",
            s=70,
            label="Triangulated estimates",
        )

    handles, labels = axis.get_legend_handles_labels()
    deduped: dict[str, object] = {}
    for handle, label in zip(handles, labels):
        if label not in deduped:
            deduped[label] = handle
    axis.legend(deduped.values(), deduped.keys(), loc="best")

    figure.tight_layout()
    figure.savefig(output_path, dpi=200)
    plt.close(figure)
