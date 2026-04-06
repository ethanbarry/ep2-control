from __future__ import annotations

import argparse
import re
from pathlib import Path

import yaml

from src.domain.models import ScenarioConfig, Vector3, scenario_from_dict

FOX_BEACON_PATTERN = re.compile(
    r"(<include>\s*<uri>model://fox_beacon</uri>\s*<pose>)([^<]+)(</pose>\s*</include>)",
    re.DOTALL,
)
FOX_MARKER_HEIGHT_M = 0.25


def load_scenario(path: Path) -> ScenarioConfig:
    with path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    assert isinstance(raw, dict), "scenario YAML must decode to a mapping"
    return scenario_from_dict(raw)


def render_world_with_fox(source_world: Path, output_world: Path, fox_position: Vector3) -> Path:
    marker_pose = (
        f"{fox_position.x:.3f} "
        f"{fox_position.y:.3f} "
        f"{fox_position.z + FOX_MARKER_HEIGHT_M:.3f} 0 0 0"
    )
    world_text = source_world.read_text(encoding="utf-8")
    rendered_world, replacements = FOX_BEACON_PATTERN.subn(
        lambda match: f"{match.group(1)}{marker_pose}{match.group(3)}",
        world_text,
        count=1,
    )
    assert replacements == 1, "world must contain a single fox_beacon include with a pose"
    output_world.parent.mkdir(parents=True, exist_ok=True)
    output_world.write_text(rendered_world, encoding="utf-8")
    return output_world


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Prepare a Gazebo world from a scenario")
    parser.add_argument("--scenario", required=True, help="Path to scenario YAML")
    parser.add_argument("--world-input", required=True, help="Source Gazebo world")
    parser.add_argument("--world-output", required=True, help="Rendered Gazebo world")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    scenario = load_scenario(Path(args.scenario))
    render_world_with_fox(
        source_world=Path(args.world_input),
        output_world=Path(args.world_output),
        fox_position=scenario.fox.position,
    )
    print(
        "Prepared world",
        f"path={args.world_output}",
        f"fox=({scenario.fox.position.x:.2f}, {scenario.fox.position.y:.2f}, {scenario.fox.position.z:.2f})",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
