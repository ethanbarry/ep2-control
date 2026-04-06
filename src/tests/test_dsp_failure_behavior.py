from __future__ import annotations

from src.domain.enums import MissionState
from src.domain.models import BatteryConfig, DspConfig, FailureConfig, FoxConfig, MissionConfig, ScenarioConfig, Vector3
from src.mavlink_client import MockMavlinkClient
from src.mission_controller import MissionController
from src.rf.mock_dsp import MockDspPipeline
from src.telemetry.logger import SimulationLogger


def test_dsp_timeout_enters_safe_hold(tmp_path) -> None:
    scenario = ScenarioConfig(
        name="dsp_timeout_test",
        description="unit test",
        home_position=Vector3(0.0, 0.0, 0.0),
        fox=FoxConfig(position=Vector3(120.0, 80.0, 0.0)),
        mission=MissionConfig(offset_distance_m=70.0, spin_bucket_deg=10, scan_sample_period_s=0.2),
        battery=BatteryConfig(),
        dsp=DspConfig(noise_std_db=0.0, outlier_probability=0.0, timeout_after_samples=4),
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

    assert outcome.final_state == MissionState.SAFE_HOLD
    assert MissionState.SAFE_HOLD in outcome.state_history
    assert outcome.failure_reason is not None
    assert "DSP" in outcome.failure_reason.upper() or "timeout" in outcome.failure_reason.lower()
