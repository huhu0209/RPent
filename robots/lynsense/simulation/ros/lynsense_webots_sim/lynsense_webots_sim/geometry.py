from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from lynsense_webots_sim.state_machine import MotionProfile


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def angular_error(current_rad: float, target_rad: float) -> float:
    return normalize_angle(target_rad - current_rad)


def pi_symmetric_angular_error(
    current_rad: float, target_rad: float, tolerance_rad: float = 0.05
) -> float:
    error = angular_error(current_rad, target_rad)
    if abs(abs(error) - math.pi) <= tolerance_rad:
        return 0.0
    return error


def bearing_to_goal(pose: Pose2, target: Pose2) -> float:
    return math.atan2(target.y - pose.y, target.x - pose.x)


def clamp(value: float, minimum: float, maximum: float) -> float:
    if not math.isfinite(value):
        raise ValueError("value must be finite")
    return min(max(value, minimum), maximum)


def wheel_rates(
    linear_velocity: float,
    angular_velocity: float,
    profile: "MotionProfile",
) -> tuple[float, float]:
    v = clamp(
        linear_velocity,
        -profile.max_linear_velocity_mps,
        profile.max_linear_velocity_mps,
    )
    omega = clamp(
        angular_velocity,
        -profile.max_angular_velocity_radps,
        profile.max_angular_velocity_radps,
    )
    left = (v - omega * profile.wheel_separation_m / 2) / profile.wheel_radius_m
    right = (v + omega * profile.wheel_separation_m / 2) / profile.wheel_radius_m
    limit = profile.max_wheel_rate_radps
    scale = 1.0
    if max(abs(left), abs(right)) > limit:
        scale = limit / max(abs(left), abs(right))
    return left * scale, right * scale


def webots_motor_rate(logical_wheel_rate_radps: float) -> float:
    if not math.isfinite(logical_wheel_rate_radps):
        raise ValueError("logical_wheel_rate_radps must be finite")
    return logical_wheel_rate_radps
