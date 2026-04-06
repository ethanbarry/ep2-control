from __future__ import annotations

from src.domain.models import MILES_TO_METERS, Vector3, scenario_from_dict
from src.scenario import render_world_with_fox


def test_scenario_from_dict_keeps_fixed_fox_position() -> None:
    scenario = scenario_from_dict(
        {
            "name": "fixed_fox",
            "description": "unit test",
            "random_seed": 7,
            "home_position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "fox": {"position": {"x": 120.0, "y": 80.0, "z": 0.0}},
        }
    )

    assert scenario.fox.position == Vector3(120.0, 80.0, 0.0)
    assert scenario.fox.search_radius_m == 0.0


def test_scenario_from_dict_randomizes_fox_within_requested_radius() -> None:
    raw = {
        "name": "random_fox",
        "description": "unit test",
        "random_seed": 11,
        "home_position": {"x": 10.0, "y": -5.0, "z": 0.0},
        "fox": {
            "randomize_within_miles": 1.0,
            "position": {"z": 0.0},
        },
    }

    scenario = scenario_from_dict(raw)
    same_seed_scenario = scenario_from_dict(raw)

    assert scenario.fox.position.xy_distance_to(scenario.home_position) <= MILES_TO_METERS
    assert scenario.fox.search_radius_m == MILES_TO_METERS
    assert same_seed_scenario.fox.position == scenario.fox.position


def test_render_world_with_fox_updates_marker_pose(tmp_path) -> None:
    source_world = tmp_path / "source.sdf"
    output_world = tmp_path / "output.sdf"
    source_world.write_text(
        """
<sdf version="1.9">
  <world name="test">
    <include>
      <uri>model://fox_beacon</uri>
      <pose>120 80 0.25 0 0 0</pose>
    </include>
  </world>
</sdf>
""".strip(),
        encoding="utf-8",
    )

    render_world_with_fox(source_world, output_world, Vector3(12.5, -8.25, 0.0))

    assert "<pose>12.500 -8.250 0.250 0 0 0</pose>" in output_world.read_text(encoding="utf-8")
