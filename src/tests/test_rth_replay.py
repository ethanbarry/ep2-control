from __future__ import annotations

from src.domain.models import Vector3
from src.flight.rth_replay import RthReplayPlanner


def test_rth_replay_reverses_recorded_path_and_ends_at_home() -> None:
    planner = RthReplayPlanner()
    home = Vector3(0.0, 0.0, 0.0)
    history = [
        home,
        Vector3(0.0, 0.0, 20.0),
        Vector3(30.0, 50.0, 20.0),
        Vector3(60.0, 70.0, 20.0),
    ]

    replay = planner.build_replay_path(history, home)

    assert replay == [
        Vector3(60.0, 70.0, 20.0),
        Vector3(30.0, 50.0, 20.0),
        Vector3(0.0, 0.0, 20.0),
        Vector3(0.0, 0.0, 0.0),
    ]
