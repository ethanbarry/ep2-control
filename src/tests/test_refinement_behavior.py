from __future__ import annotations

from src.domain.enums import MissionState
from src.domain.models import BatteryConfig, DspConfig, FailureConfig, FoxConfig, MissionConfig, ScenarioConfig, Vector3
from src.mavlink_client import MockMavlinkClient
from src.mission_controller import MissionController
from src.rf.mock_dsp import MockDspPipeline
from src.telemetry.logger import SimulationLogger


def test_refinement_passes_add_scans_and_do_not_worsen_estimate(tmp_path) -> None:
    scenario = ScenarioConfig(
        name="refinement_test",
        description="unit test",
        home_position=Vector3(0.0, 0.0, 0.0),
        fox=FoxConfig(
            position=Vector3(120.0, 80.0, 0.0),
            antenna_beamwidth_deg=75.0,
        ),
        mission=MissionConfig(
            spin_bucket_deg=20,
            offset_distance_m=70.0,
            refinement_passes=1,
            refinement_offset_distance_m=20.0,
            refinement_convergence_distance_m=1.0,
            scan_sample_period_s=0.2,
        ),
        battery=BatteryConfig(
            start_pct=100.0,
            low_battery_threshold_pct=10.0,
            drain_per_meter_pct=0.02,
            drain_per_spin_sample_pct=0.05,
            drain_per_takeoff_pct=1.0,
            drain_per_hover_second_pct=0.01,
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

    assert outcome.final_state == MissionState.COMPLETE
    assert len(outcome.scan_results) == 4
    assert len(outcome.triangulation_estimates) == 2
    assert outcome.triangulation_estimates[0].position is not None
    assert outcome.triangulation_estimates[-1].position is not None

    fox_position = scenario.fox.position
    initial_error = outcome.triangulation_estimates[0].position.xy_distance_to(fox_position)
    refined_error = outcome.triangulation_estimates[-1].position.xy_distance_to(fox_position)

    assert refined_error <= initial_error
