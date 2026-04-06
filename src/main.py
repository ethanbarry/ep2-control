from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

from src.mavlink_client import MockMavlinkClient, PymavlinkClient
from src.mission_controller import MissionController
from src.rf.mock_dsp import MockDspPipeline
from src.scenario import load_scenario
from src.telemetry.logger import SimulationLogger

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Drone RF fox-hunting simulator")
    parser.add_argument("--scenario", required=True, help="Path to scenario YAML")
    parser.add_argument(
        "--backend",
        choices=["mock", "mavlink"],
        default="mock",
        help="Vehicle backend to use",
    )
    parser.add_argument(
        "--connection",
        default="udp:127.0.0.1:14550",
        help="MAVLink connection string used by the mavlink backend",
    )
    parser.add_argument(
        "--artifacts-dir",
        default="artifacts",
        help="Directory where CSV logs and plots are written",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Export a matplotlib mission plot",
    )
    return parser.parse_args()

def main() -> int:
    args = parse_args()
    scenario = load_scenario(Path(args.scenario))
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    artifacts_dir = Path(args.artifacts_dir) / f"{scenario.name}_{timestamp}"
    logger = SimulationLogger(artifacts_dir)

    client = (
        MockMavlinkClient(scenario.home_position)
        if args.backend == "mock"
        else PymavlinkClient(
            connection_string=args.connection,
            position_tolerance_m=scenario.mission.position_tolerance_m,
        )
    )
    controller = MissionController(
        scenario=scenario,
        client=client,
        dsp_pipeline=MockDspPipeline(scenario.dsp, seed=scenario.random_seed),
        logger=logger,
        artifacts_dir=artifacts_dir,
    )
    outcome = controller.run()

    if args.plot:
        from src.telemetry.plot_export import export_plot

        export_plot(scenario, outcome, artifacts_dir / "mission_plot.png")

    print(
        "Mission finished",
        f"state={outcome.final_state.value}",
        f"battery={outcome.battery_remaining_pct:.2f}%",
        f"fox=({scenario.fox.position.x:.2f}, {scenario.fox.position.y:.2f}, {scenario.fox.position.z:.2f})",
        f"artifacts={artifacts_dir}",
        f"failure={outcome.failure_reason or 'none'}",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
