from __future__ import annotations

from src.domain.enums import MissionState
from src.domain.models import BatteryConfig, DspConfig, FailureConfig, FoxConfig, MissionConfig, ScenarioConfig, Vector3
from src.mavlink_client import MockMavlinkClient
from src.mission_controller import MissionController
from src.rf.mock_dsp import MockDspPipeline
from src.telemetry.logger import SimulationLogger


def test_low_battery_replays_path_home(tmp_path) -> None:
    scenario = ScenarioConfig(
        name="low_battery_test",
        description="unit test",
        home_position=Vector3(0.0, 0.0, 0.0),
        fox=FoxConfig(position=Vector3(120.0, 80.0, 0.0)),
        mission=MissionConfig(offset_distance_m=70.0, spin_bucket_deg=10, scan_sample_period_s=0.2),
        battery=BatteryConfig(
            start_pct=50.0,
            low_battery_threshold_pct=32.0,
            drain_per_meter_pct=0.06,
            drain_per_spin_sample_pct=0.24,
            drain_per_takeoff_pct=2.8,
            drain_per_hover_second_pct=0.04,
        ),
        dsp=DspConfig(noise_std_db=0.0, outlier_probability=0.0),
        failures=FailureConfig(invalid_triangulation=False),
        random_seed=7,
    )
    controller = MissionController(
        scenario=scenario,
        client=MockMavlinkClient(scenario.home_position),
        dsp_pipeline=MockDspPipeline(scenario.dsp, seed=scenario.random_seed),
        logger=SimulationLogger(tmp_path / "artifacts"),
        artifacts_dir=tmp_path / "artifacts",
    )

    outcome = controller.run()

    assert outcome.returned_home_via_replay is True
    assert MissionState.RETURN_TO_HOME_REPLAY in outcome.state_history
    assert outcome.final_state == MissionState.COMPLETE
    assert outcome.replay_path[-1] == scenario.home_position
    assert outcome.actual_path[-1] == scenario.home_position
