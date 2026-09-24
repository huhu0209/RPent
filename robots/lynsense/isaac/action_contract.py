# Copyright 2026 The RPent Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping

from robots.lynsense.isaac.contracts import IsaacProbeError

ACTION_JOINT_NAMES = ("left_joint1", "right_joint1")
ACTION_DELTA_RAD = 0.1
MAX_ACTION_EFFORT_NM = 200.0
MAX_ACTION_SPEED_RAD_S = 0.314
MIN_PROGRESS_RAD = 0.05
MAX_FINAL_ERROR_RAD = 0.02
MAX_FINAL_SPEED_RAD_S = 0.05
MAX_OVERSHOOT_RAD = 0.02
MAX_ACTION_DURATION_S = 10.0
FINAL_SETTLE_WINDOW_S = 0.5
MAX_BASE_TRANSLATION_M = 0.001
MAX_BASE_ROTATION_RAD = math.radians(0.5)
PLAN_TARGET_TOLERANCE_RAD = 1e-12
TRAJECTORY_INITIAL_TOLERANCE_RAD = 1e-9


@dataclass(frozen=True)
class DriveGains:
    stiffness: float = 200.0
    damping: float = 16.6


@dataclass(frozen=True)
class DriveLimits:
    max_effort_nm: float = MAX_ACTION_EFFORT_NM
    max_velocity_rad_s: float = MAX_ACTION_SPEED_RAD_S


@dataclass(frozen=True)
class DriveConfiguration:
    gains: DriveGains
    limits: DriveLimits
    verified: bool = False

    def __post_init__(self) -> None:
        gain_values = (self.gains.stiffness, self.gains.damping)
        if any(value < 0.0 or not math.isfinite(value) for value in gain_values):
            raise IsaacProbeError("drive gains must be non-negative and finite")
        limit_values = (
            self.limits.max_effort_nm,
            self.limits.max_velocity_rad_s,
        )
        if any(value < 0.0 or not math.isfinite(value) for value in limit_values):
            raise IsaacProbeError("drive limits must be non-negative and finite")

    def to_dict(self) -> dict[str, object]:
        return {
            "gains": {
                "stiffness": self.gains.stiffness,
                "damping": self.gains.damping,
            },
            "limits": {
                "max_effort_nm": self.limits.max_effort_nm,
                "max_velocity_rad_s": self.limits.max_velocity_rad_s,
            },
            "verified": self.verified,
        }


@dataclass(frozen=True)
class JointActionPlan:
    joint_name: str
    initial_position: float
    target_position: float
    lower_limit: float
    upper_limit: float


@dataclass(frozen=True)
class JointSample:
    time_s: float
    position_rad: float
    velocity_rad_s: float


@dataclass(frozen=True)
class TransformSample:
    time_s: float
    translation: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]


def _require_finite(label: str, *values: float) -> None:
    if any(not math.isfinite(value) for value in values):
        raise IsaacProbeError(f"{label} must be finite")


def _validate_joint_action_plan(mapping_name: str, plan: JointActionPlan) -> None:
    if plan.joint_name != mapping_name:
        raise IsaacProbeError(
            f"plan joint name {plan.joint_name!r} does not match mapping key "
            f"{mapping_name!r}"
        )
    _require_finite(
        f"{mapping_name} action plan initial position, limits, and target",
        plan.initial_position,
        plan.lower_limit,
        plan.upper_limit,
        plan.target_position,
    )
    if plan.lower_limit >= plan.upper_limit:
        raise IsaacProbeError(
            f"{mapping_name} action plan limits must satisfy lower < upper"
        )
    if (
        plan.initial_position < plan.lower_limit
        or plan.initial_position > plan.upper_limit
    ):
        raise IsaacProbeError(
            f"action target for {mapping_name} requires an initial position "
            "inside the plan limits"
        )
    expected_target = plan.initial_position + ACTION_DELTA_RAD
    if not math.isclose(
        plan.target_position,
        expected_target,
        rel_tol=0.0,
        abs_tol=PLAN_TARGET_TOLERANCE_RAD,
    ):
        raise IsaacProbeError(
            f"action target for {mapping_name} must equal initial position "
            f"plus {ACTION_DELTA_RAD} rad"
        )
    if plan.target_position > plan.upper_limit:
        raise IsaacProbeError(
            f"action target for {mapping_name} exceeds the plan upper limit"
        )


def _validate_joint_action_plans(plans: Mapping[str, JointActionPlan]) -> None:
    for mapping_name, plan in plans.items():
        _validate_joint_action_plan(mapping_name, plan)


def plan_joint_action(
    joint_name: str, initial_position: float, lower_limit: float, upper_limit: float
) -> JointActionPlan:
    target_position = initial_position + ACTION_DELTA_RAD
    plan = JointActionPlan(
        joint_name=joint_name,
        initial_position=initial_position,
        target_position=target_position,
        lower_limit=lower_limit,
        upper_limit=upper_limit,
    )
    _validate_joint_action_plan(joint_name, plan)
    return plan


def _joint_times(trajectory: tuple[JointSample, ...]) -> tuple[float, ...]:
    return tuple(sample.time_s for sample in trajectory)


def _validate_joint_trajectory(
    joint_name: str,
    trajectory: tuple[JointSample, ...],
    plan: JointActionPlan,
) -> None:
    if len(trajectory) < 2:
        raise IsaacProbeError(
            f"{joint_name} trajectory must contain at least two samples"
        )
    times = _joint_times(trajectory)
    _require_finite(f"{joint_name} trajectory times", *times)
    if any(right <= left for left, right in zip(times, times[1:])):
        raise IsaacProbeError(
            f"{joint_name} trajectory times must be strictly increasing"
        )
    if times[-1] - times[0] > MAX_ACTION_DURATION_S:
        raise IsaacProbeError(f"{joint_name} trajectory duration exceeds 10 seconds")
    if times[-1] - times[0] < FINAL_SETTLE_WINDOW_S:
        raise IsaacProbeError(
            f"{joint_name} trajectory is missing its final settle window"
        )
    for index, sample in enumerate(trajectory):
        _require_finite(
            f"{joint_name} trajectory sample {index}",
            sample.position_rad,
            sample.velocity_rad_s,
        )
        if (
            sample.position_rad < plan.lower_limit
            or sample.position_rad > plan.upper_limit
        ):
            raise IsaacProbeError(
                f"{joint_name} trajectory sample {index} position exceeds "
                "the plan limits"
            )
        if abs(sample.velocity_rad_s) > MAX_ACTION_SPEED_RAD_S:
            raise IsaacProbeError(
                f"{joint_name} trajectory sample {index} speed exceeds "
                f"{MAX_ACTION_SPEED_RAD_S} rad/s"
            )


def _overshoot_rad(initial: float, target: float, position: float) -> float:
    direction = 1.0 if target >= initial else -1.0
    return max(0.0, direction * (position - target))


def evaluate_joint_action(
    trajectories: Mapping[str, tuple[JointSample, ...]],
    plans: Mapping[str, JointActionPlan],
) -> dict[str, object]:
    expected_names = set(ACTION_JOINT_NAMES)
    if set(trajectories) != expected_names or set(plans) != expected_names:
        raise IsaacProbeError(
            "joint action evaluation requires exactly left_joint1 and right_joint1"
        )
    _validate_joint_action_plans(plans)

    first_trajectory = trajectories[ACTION_JOINT_NAMES[0]]
    reference_times = _joint_times(first_trajectory)
    for joint_name in ACTION_JOINT_NAMES:
        trajectory = trajectories[joint_name]
        _validate_joint_trajectory(joint_name, trajectory, plans[joint_name])
        if _joint_times(trajectory) != reference_times:
            raise IsaacProbeError(
                f"{joint_name} trajectory times must match the other joint"
            )
        if not math.isclose(
            trajectory[0].position_rad,
            plans[joint_name].initial_position,
            rel_tol=0.0,
            abs_tol=TRAJECTORY_INITIAL_TOLERANCE_RAD,
        ):
            raise IsaacProbeError(
                f"{joint_name} trajectory initial position does not match its plan"
            )

    joint_metrics: dict[str, dict[str, float]] = {}
    for joint_name in ACTION_JOINT_NAMES:
        plan = plans[joint_name]
        trajectory = trajectories[joint_name]
        initial = trajectory[0].position_rad
        direction = 1.0 if plan.target_position >= initial else -1.0
        progress = direction * (trajectory[-1].position_rad - initial)
        target_errors = [
            abs(sample.position_rad - plan.target_position) for sample in trajectory
        ]
        overshoots = [
            _overshoot_rad(initial, plan.target_position, sample.position_rad)
            for sample in trajectory
        ]
        settle_start = trajectory[-1].time_s - FINAL_SETTLE_WINDOW_S
        settle_samples = [
            sample for sample in trajectory if sample.time_s >= settle_start
        ]
        settle_errors = [
            abs(sample.position_rad - plan.target_position) for sample in settle_samples
        ]
        settle_speeds = [abs(sample.velocity_rad_s) for sample in settle_samples]
        if progress < MIN_PROGRESS_RAD:
            raise IsaacProbeError(
                f"{joint_name} has insufficient progress toward its target"
            )
        if max(overshoots) > MAX_OVERSHOOT_RAD:
            raise IsaacProbeError(
                f"{joint_name} overshoot exceeds the action tolerance"
            )
        if max(settle_errors) > MAX_FINAL_ERROR_RAD:
            raise IsaacProbeError(
                f"{joint_name} final settle error exceeds the action tolerance"
            )
        if max(settle_speeds) > MAX_FINAL_SPEED_RAD_S:
            raise IsaacProbeError(
                f"{joint_name} final settle speed exceeds the action tolerance"
            )
        joint_metrics[joint_name] = {
            "progress_rad": progress,
            "max_target_error_rad": max(target_errors),
            "max_settle_error_rad": max(settle_errors),
            "max_settle_speed_rad_s": max(settle_speeds),
            "max_overshoot_rad": max(overshoots),
        }

    return {
        "passed": True,
        "final_time_s": reference_times[-1],
        "joints": joint_metrics,
    }


def _validate_base_samples(
    samples: tuple[TransformSample, ...],
) -> tuple[TransformSample, ...]:
    if not samples:
        raise IsaacProbeError("base motion requires at least one sample")
    times = tuple(sample.time_s for sample in samples)
    _require_finite("base sample times", *times)
    if any(right <= left for left, right in zip(times, times[1:])):
        raise IsaacProbeError("base sample times must be strictly increasing")
    for index, sample in enumerate(samples):
        _require_finite(
            f"base sample {index} translation",
            *sample.translation,
        )
        _require_finite(
            f"base sample {index} quaternion",
            *sample.rotation_xyzw,
        )
        quaternion_norm = math.hypot(*sample.rotation_xyzw)
        if quaternion_norm < 1e-9:
            raise IsaacProbeError(
                f"base sample {index} quaternion norm is too close to zero"
            )
    return samples


def _relative_rotation_rad(
    first_xyzw: tuple[float, float, float, float],
    current_xyzw: tuple[float, float, float, float],
) -> float:
    first_norm = math.hypot(*first_xyzw)
    current_norm = math.hypot(*current_xyzw)
    first = tuple(value / first_norm for value in first_xyzw)
    current = tuple(value / current_norm for value in current_xyzw)
    inverse_first = (first[0], first[1], first[2], -first[3])
    x1, y1, z1, w1 = current
    x2, y2, z2, w2 = inverse_first
    relative = (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )
    vector_norm = math.sqrt(
        relative[0] * relative[0]
        + relative[1] * relative[1]
        + relative[2] * relative[2]
    )
    return 2.0 * math.atan2(vector_norm, abs(relative[3]))


def _base_motion_metrics(samples: tuple[TransformSample, ...]) -> dict[str, float]:
    first = samples[0]
    max_translation = max(
        math.dist(first.translation, sample.translation) for sample in samples
    )
    max_rotation = max(
        _relative_rotation_rad(first.rotation_xyzw, sample.rotation_xyzw)
        for sample in samples
    )
    return {
        "max_translation_m": max_translation,
        "max_rotation_rad": max_rotation,
    }


def _validate_base_motion_limits(metrics: dict[str, float]) -> None:
    if metrics["max_translation_m"] > MAX_BASE_TRANSLATION_M:
        raise IsaacProbeError("base translation exceeds the fixed-base tolerance")
    if metrics["max_rotation_rad"] > MAX_BASE_ROTATION_RAD:
        raise IsaacProbeError("base rotation exceeds the fixed-base tolerance")


def evaluate_base_motion(
    samples: tuple[TransformSample, ...],
) -> dict[str, object]:
    validated = _validate_base_samples(samples)
    metrics = _base_motion_metrics(validated)
    _validate_base_motion_limits(metrics)
    return {
        "passed": True,
        "final_time_s": validated[-1].time_s,
        **metrics,
    }


def validate_step_state(
    plans: Mapping[str, JointActionPlan],
    latest_samples: Mapping[str, JointSample],
    base_history: tuple[TransformSample, ...],
) -> None:
    expected_names = set(ACTION_JOINT_NAMES)
    if set(plans) != expected_names or set(latest_samples) != expected_names:
        raise IsaacProbeError(
            "streaming validation requires exactly left_joint1 and right_joint1"
        )
    _validate_joint_action_plans(plans)
    for joint_name in ACTION_JOINT_NAMES:
        sample = latest_samples[joint_name]
        _require_finite(
            f"{joint_name} latest sample",
            sample.time_s,
            sample.position_rad,
            sample.velocity_rad_s,
        )
        plan = plans[joint_name]
        if (
            _overshoot_rad(
                plan.initial_position, plan.target_position, sample.position_rad
            )
            > MAX_OVERSHOOT_RAD
        ):
            raise IsaacProbeError(
                f"{joint_name} current overshoot exceeds the action tolerance"
            )

    validated_base = _validate_base_samples(base_history)
    _validate_base_motion_limits(_base_motion_metrics(validated_base))


def joint_trajectories_to_dict(
    trajectories: Mapping[str, tuple[JointSample, ...]],
) -> dict[str, list[dict[str, float]]]:
    return {
        joint_name: [
            {
                "time_s": sample.time_s,
                "position_rad": sample.position_rad,
                "velocity_rad_s": sample.velocity_rad_s,
            }
            for sample in trajectory
        ]
        for joint_name, trajectory in trajectories.items()
    }


def base_motion_to_dict(
    samples: tuple[TransformSample, ...],
) -> list[dict[str, object]]:
    return [
        {
            "time_s": sample.time_s,
            "translation": list(sample.translation),
            "rotation_xyzw": list(sample.rotation_xyzw),
        }
        for sample in samples
    ]
