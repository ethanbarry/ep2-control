from __future__ import annotations

import math
import time
from abc import ABC, abstractmethod
from collections.abc import Callable
from typing import Any

from src.domain.models import Pose, Vector3
from src.rf.signal_model import normalize_angle_deg

try:
    from pymavlink import mavutil
except ImportError:  # pragma: no cover - exercised only when pymavlink is unavailable
    mavutil = None


class BaseMavlinkClient(ABC):
    @abstractmethod
    def connect(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def set_guided_mode(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def arm_and_takeoff(self, target_altitude_m: float) -> None:
        raise NotImplementedError

    @abstractmethod
    def goto_position(self, target: Vector3) -> float:
        raise NotImplementedError

    @abstractmethod
    def condition_yaw(self, yaw_deg: float) -> None:
        raise NotImplementedError

    @abstractmethod
    def hold_position(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def get_pose(self) -> Pose:
        raise NotImplementedError

    @property
    @abstractmethod
    def actual_path(self) -> list[Vector3]:
        raise NotImplementedError

    def close(self) -> None:
        return None


class MockMavlinkClient(BaseMavlinkClient):
    def __init__(self, home: Vector3) -> None:
        self._pose = Pose(position=home, yaw_deg=0.0)
        self._actual_path: list[Vector3] = [home]

    def connect(self) -> None:
        return None

    def set_guided_mode(self) -> None:
        return None

    def arm_and_takeoff(self, target_altitude_m: float) -> None:
        assert target_altitude_m > 0.0
        target = Vector3(self._pose.position.x, self._pose.position.y, target_altitude_m)
        self._move_to(target)

    def goto_position(self, target: Vector3) -> float:
        return self._move_to(target)

    def condition_yaw(self, yaw_deg: float) -> None:
        self._pose = Pose(position=self._pose.position, yaw_deg=normalize_angle_deg(yaw_deg))

    def hold_position(self) -> None:
        return None

    def get_pose(self) -> Pose:
        return self._pose

    @property
    def actual_path(self) -> list[Vector3]:
        return list(self._actual_path)

    def _move_to(self, target: Vector3) -> float:
        start = self._pose.position
        distance = start.distance_to(target)
        steps = max(1, int(distance / 5.0))
        for step in range(1, steps + 1):
            waypoint = start.interpolate(target, step / steps)
            self._actual_path.append(waypoint)
        self._pose = Pose(position=target, yaw_deg=self._pose.yaw_deg)
        return distance


class PymavlinkClient(BaseMavlinkClient):
    def __init__(
        self,
        connection_string: str,
        position_tolerance_m: float = 2.0,
        command_timeout_s: float = 90.0,
    ) -> None:
        if mavutil is None:  # pragma: no cover - depends on external package
            raise RuntimeError("pymavlink is required for the mavlink backend")
        self._connection_string = connection_string
        self._position_tolerance_m = position_tolerance_m
        self._command_timeout_s = command_timeout_s
        self._master = None
        self._latest_pose = Pose(position=Vector3(0.0, 0.0, 0.0), yaw_deg=0.0)
        self._actual_path: list[Vector3] = [self._latest_pose.position]
        self._last_status_text = ""

    def connect(self) -> None:
        self._master = mavutil.mavlink_connection(self._connection_string, autoreconnect=True)
        self._master.wait_heartbeat(timeout=60)
        self._poll_pose(5.0)

    def set_guided_mode(self) -> None:
        assert self._master is not None
        deadline = time.time() + min(self._command_timeout_s, 20.0)
        while time.time() < deadline:
            self._master.set_mode_apm("GUIDED")
            if self._wait_for_flight_mode("GUIDED", 3.0):
                return
            time.sleep(1.0)
        raise TimeoutError(f"timed out switching to GUIDED{self._status_suffix()}")

    def arm_and_takeoff(self, target_altitude_m: float) -> None:
        assert self._master is not None
        assert target_altitude_m > 0.0
        self._arm_vehicle()
        self._master.mav.command_long_send(
            self._master.target_system,
            self._master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            target_altitude_m,
        )
        self._wait_until(lambda pose: abs(pose.position.z - target_altitude_m) <= self._position_tolerance_m)

    def goto_position(self, target: Vector3) -> float:
        assert self._master is not None
        current = self.get_pose().position
        type_mask = 0b0000_1101_1111_1000
        self._master.mav.set_position_target_local_ned_send(
            0,
            self._master.target_system,
            self._master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            target.x,
            target.y,
            -target.z,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self._wait_until(lambda pose: pose.position.distance_to(target) <= self._position_tolerance_m)
        return current.distance_to(target)

    def condition_yaw(self, yaw_deg: float) -> None:
        assert self._master is not None
        self._master.mav.command_long_send(
            self._master.target_system,
            self._master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            yaw_deg,
            15,
            1,
            0,
            0,
            0,
            0,
        )
        self._poll_pose(1.0)

    def hold_position(self) -> None:
        current = self.get_pose().position
        self.goto_position(current)

    def get_pose(self) -> Pose:
        self._poll_pose(0.25)
        return self._latest_pose

    @property
    def actual_path(self) -> list[Vector3]:
        return list(self._actual_path)

    def close(self) -> None:
        if self._master is not None:
            self._master.close()

    def _arm_vehicle(self) -> None:
        assert self._master is not None
        deadline = time.time() + self._command_timeout_s
        while time.time() < deadline:
            self._master.arducopter_arm()
            if self._wait_for_armed(5.0):
                return
            time.sleep(1.0)
        raise TimeoutError(f"timed out waiting for motors to arm{self._status_suffix()}")

    def _wait_until(self, predicate: Callable[[Pose], bool]) -> None:
        deadline = time.time() + self._command_timeout_s
        while time.time() < deadline:
            pose = self.get_pose()
            if predicate(pose):
                return
            time.sleep(0.25)
        raise TimeoutError(f"timed out waiting for vehicle state{self._status_suffix()}")

    def _wait_for_armed(self, timeout_s: float) -> bool:
        assert self._master is not None
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            message = self._recv_message(["HEARTBEAT", "STATUSTEXT"], max(0.1, deadline - time.time()))
            if message is None:
                continue
            if message.get_type() == "HEARTBEAT" and self._master.motors_armed():
                return True
        return False

    def _wait_for_flight_mode(self, expected_mode: str, timeout_s: float) -> bool:
        assert self._master is not None
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            message = self._recv_message(["HEARTBEAT", "STATUSTEXT"], max(0.1, deadline - time.time()))
            if message is None:
                continue
            if message.get_type() == "HEARTBEAT" and self._master.flightmode == expected_mode:
                return True
        return False

    def _recv_message(self, message_types: list[str], timeout_s: float) -> Any:
        assert self._master is not None
        message = self._master.recv_match(
            type=message_types,
            blocking=True,
            timeout=max(0.1, timeout_s),
        )
        if message is None:
            return None
        if message.get_type() == "STATUSTEXT":
            self._last_status_text = getattr(message, "text", "")
        return message

    def _status_suffix(self) -> str:
        if not self._last_status_text:
            return ""
        return f"; last status='{self._last_status_text}'"

    def _poll_pose(self, timeout_s: float) -> None:
        assert self._master is not None
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            message = self._recv_message(
                ["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"],
                max(0.1, deadline - time.time()),
            )
            if message is None:
                continue
            message_type = message.get_type()
            if message_type == "LOCAL_POSITION_NED":
                self._latest_pose = Pose(
                    position=Vector3(float(message.x), float(message.y), float(-message.z)),
                    yaw_deg=self._latest_pose.yaw_deg,
                )
                if not self._actual_path or self._actual_path[-1] != self._latest_pose.position:
                    self._actual_path.append(self._latest_pose.position)
            if message_type == "ATTITUDE":
                self._latest_pose = Pose(
                    position=self._latest_pose.position,
                    yaw_deg=normalize_angle_deg(math.degrees(float(message.yaw))),
                )
