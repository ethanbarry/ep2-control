from __future__ import annotations

import math

from src.domain.models import FoxConfig, Pose


def normalize_angle_deg(angle_deg: float) -> float:
    value = angle_deg % 360.0
    return value if value >= 0.0 else value + 360.0


def shortest_angular_distance_deg(from_deg: float, to_deg: float) -> float:
    delta = (to_deg - from_deg + 180.0) % 360.0 - 180.0
    return delta


def bearing_deg_between(origin: Pose | tuple[float, float] | tuple[int, int], target_x: float, target_y: float) -> float:
    if isinstance(origin, Pose):
        origin_x = origin.position.x
        origin_y = origin.position.y
    else:
        origin_x, origin_y = float(origin[0]), float(origin[1])
    return normalize_angle_deg(math.degrees(math.atan2(target_y - origin_y, target_x - origin_x)))


def directional_gain_db(relative_bearing_deg: float, fox: FoxConfig) -> float:
    offset = abs(shortest_angular_distance_deg(0.0, relative_bearing_deg))
    beamwidth = max(fox.antenna_beamwidth_deg, 1.0)
    if offset <= beamwidth:
        normalized = offset / beamwidth
        return fox.antenna_front_gain_db - 3.0 * normalized * normalized
    sidelobe_progress = (offset - beamwidth) / max(1.0, 180.0 - beamwidth)
    return fox.antenna_front_gain_db - 3.0 - 18.0 * min(1.0, sidelobe_progress)


def inverse_distance_rssi_dbm(
    pose: Pose,
    fox: FoxConfig,
    yaw_deg: float,
    bias_bearing_deg: float = 0.0,
) -> tuple[float, float]:
    distance_m = max(1.0, pose.position.distance_to(fox.position))
    true_bearing = bearing_deg_between(pose, fox.position.x, fox.position.y)
    relative = shortest_angular_distance_deg(yaw_deg, true_bearing + bias_bearing_deg)
    gain_db = directional_gain_db(relative, fox)
    path_loss_db = 42.0 + 10.0 * fox.path_loss_exponent * math.log10(distance_m)
    rssi_dbm = fox.tx_power_dbm + gain_db - path_loss_db
    return rssi_dbm, true_bearing
