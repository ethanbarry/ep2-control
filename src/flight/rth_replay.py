from __future__ import annotations

from dataclasses import dataclass

from src.domain.models import Vector3

# per
@dataclass
class RthReplayPlanner:
    include_home: bool = True

    def build_replay_path(self, history: list[Vector3], home: Vector3) -> list[Vector3]:
        assert history, "replay history cannot be empty"
        reversed_history: list[Vector3] = []
        for waypoint in reversed(history):
            if reversed_history and reversed_history[-1] == waypoint:
                continue
            reversed_history.append(waypoint)
        if self.include_home and reversed_history[-1] != home:
            reversed_history.append(home)
        return reversed_history
