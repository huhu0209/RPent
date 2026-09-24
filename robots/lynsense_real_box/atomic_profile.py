"""Strict site profile for LynrotControl-backed atomic capabilities."""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any


class AtomicProfileError(ValueError):
    """Raised when an atomic-capability profile is invalid."""


@dataclass(frozen=True, slots=True)
class DualArmJointProfile:
    left_joints_deg: tuple[tuple[float, ...], ...]
    right_joints_deg: tuple[tuple[float, ...], ...]
    tolerance_deg: float
    max_joint_step_deg: float
    start_tolerance_deg: float
    timeout_s: float


@dataclass(frozen=True, slots=True)
class WaistProfile:
    position_mm: float
    tolerance_mm: float
    timeout_s: float


@dataclass(frozen=True, slots=True)
class GripperProfile:
    position_rad: float
    tolerance_rad: float
    timeout_s: float


@dataclass(frozen=True, slots=True)
class ChassisProfile:
    kind: str
    goal: str | None
    distance_m: float | None
    angle_deg: float | None
    timeout_s: float
    carries_box: bool = False


@dataclass(frozen=True, slots=True)
class RuntimeBinding:
    site_id: str
    robot_id: str
    lynrotcontrol_instance: str
    lynrotcontrol_config_sha256: str
    execution_host: str
    ros_domain_id: int
    source_artifacts: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class ChassisRuntimeProfile:
    kind: str
    odom_topic: str
    cmd_vel_topic: str
    max_linear_speed_m_s: float
    max_angular_speed_rad_s: float
    max_linear_accel_m_s2: float
    max_angular_accel_rad_s2: float
    command_rate_hz: float
    watchdog_timeout_s: float
    stop_settle_timeout_s: float
    zero_twist_confirm_samples: int
    odom_max_age_s: float
    source_artifacts: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class GripperRuntimeProfile:
    failure_policy: str
    requires_stop_api: bool
    stop_timeout_s: float
    zero_motion_confirm_samples: int
    source_artifacts: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class PerceptionBinding:
    pose_topic: str
    status_topic: str
    trigger_service: str
    expected_frame: str
    pose_max_age_s: float
    trigger_timeout_s: float
    result_deadline_s: float
    source_artifacts: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class SubsystemFreshness:
    left_arm_state_s: float
    right_arm_state_s: float
    left_arm_force_s: float
    right_arm_force_s: float
    left_gripper_s: float
    right_gripper_s: float
    waist_s: float
    chassis_s: float


@dataclass(frozen=True, slots=True)
class CarryGuard:
    arm_profile: str
    gripper_command: str
    box_control_status: str
    force_monitor: str
    source_artifacts: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class AtomicReview:
    status: str
    review_id: str
    safety_evidence: tuple[str, ...]
    reviewer: str | None


@dataclass(frozen=True, slots=True)
class ForceGates:
    max_abs_force_n: tuple[float, float, float]
    max_abs_torque_nm: tuple[float, float, float]
    hold_s: float


@dataclass(frozen=True, slots=True)
class AtomicCapabilityProfile:
    profile_id: str
    version: int
    mode: str
    joint_axes: int
    joint_limits_deg: tuple[tuple[float, float], ...]
    dual_arm_profiles: Mapping[str, DualArmJointProfile]
    waist_profiles: Mapping[str, WaistProfile]
    gripper_profiles: Mapping[str, GripperProfile]
    chassis_profiles: Mapping[str, ChassisProfile]
    review: AtomicReview
    force_gates: ForceGates
    budgets: Mapping[str, float]
    robot_state_max_age_s: float
    max_chassis_distance_m: float
    max_chassis_angle_deg: float
    runtime_binding: RuntimeBinding | None = None
    chassis_runtime: ChassisRuntimeProfile | None = None
    gripper_runtime: GripperRuntimeProfile | None = None
    perception_binding: PerceptionBinding | None = None
    subsystem_freshness: SubsystemFreshness | None = None
    carry_guards: Mapping[str, CarryGuard] = MappingProxyType({})


_TOP_LEVEL_KEYS = frozenset(
    {
        "profile_id",
        "version",
        "mode",
        "joint_axes",
        "joint_limits_deg",
        "dual_arm_profiles",
        "waist_profiles",
        "gripper_profiles",
        "chassis_profiles",
        "review",
        "force_gates",
        "budgets",
        "robot_state_max_age_s",
        "max_chassis_distance_m",
        "max_chassis_angle_deg",
    }
)
_V2_TOP_LEVEL_KEYS = _TOP_LEVEL_KEYS | frozenset(
    {
        "runtime_binding",
        "chassis_runtime",
        "gripper_runtime",
        "perception_binding",
        "subsystem_freshness",
        "carry_guards",
    }
)
_REVIEW_KEYS = frozenset(
    {"status", "review_id", "safety_evidence", "reviewer"}
)
_FORCE_KEYS = frozenset(
    {"max_abs_force_n", "max_abs_torque_nm", "hold_s"}
)
_BUDGET_KEYS = frozenset(
    {"chassis_s", "waist_s", "dual_arms_s", "grippers_s", "stop_s"}
)
_DUAL_ARM_KEYS = frozenset(
    {
        "left_joints_deg",
        "right_joints_deg",
        "tolerance_deg",
        "max_joint_step_deg",
        "start_tolerance_deg",
        "timeout_s",
    }
)
_WAIST_KEYS = frozenset(
    {"position_mm", "tolerance_mm", "timeout_s"}
)
_GRIPPER_KEYS = frozenset(
    {"position_rad", "tolerance_rad", "timeout_s"}
)
_RUNTIME_BINDING_KEYS = frozenset(
    {
        "site_id",
        "robot_id",
        "lynrotcontrol_instance",
        "lynrotcontrol_config_sha256",
        "execution_host",
        "ros_domain_id",
        "source_artifacts",
    }
)
_CHASSIS_RUNTIME_KEYS = frozenset(
    {
        "kind",
        "odom_topic",
        "cmd_vel_topic",
        "max_linear_speed_m_s",
        "max_angular_speed_rad_s",
        "max_linear_accel_m_s2",
        "max_angular_accel_rad_s2",
        "command_rate_hz",
        "watchdog_timeout_s",
        "stop_settle_timeout_s",
        "zero_twist_confirm_samples",
        "odom_max_age_s",
        "source_artifacts",
    }
)
_GRIPPER_RUNTIME_KEYS = frozenset(
    {
        "failure_policy",
        "requires_stop_api",
        "stop_timeout_s",
        "zero_motion_confirm_samples",
        "source_artifacts",
    }
)
_PERCEPTION_BINDING_KEYS = frozenset(
    {
        "pose_topic",
        "status_topic",
        "trigger_service",
        "expected_frame",
        "pose_max_age_s",
        "trigger_timeout_s",
        "result_deadline_s",
        "source_artifacts",
    }
)
_SUBSYSTEM_FRESHNESS_KEYS = frozenset(
    {
        "left_arm_state_s",
        "right_arm_state_s",
        "left_arm_force_s",
        "right_arm_force_s",
        "left_gripper_s",
        "right_gripper_s",
        "waist_s",
        "chassis_s",
    }
)
_CARRY_GUARD_KEYS = frozenset(
    {
        "arm_profile",
        "gripper_command",
        "box_control_status",
        "force_monitor",
        "source_artifacts",
    }
)


def validate_atomic_capability_profile(
    source: dict[str, Any],
) -> AtomicCapabilityProfile:
    """Validate and normalize an atomic-capability site profile."""

    raw_item = _object(source, "atomic profile")
    profile_id = _string(raw_item.get("profile_id"), "profile_id")
    version = _integer(raw_item.get("version"), "version", minimum=1)
    if version not in {1, 2}:
        raise AtomicProfileError("version must be 1 or 2")

    mode = _string(raw_item.get("mode"), "mode")
    if mode not in {"dry_run", "live"}:
        raise AtomicProfileError("mode must be dry_run or live")
    if version == 1 and mode == "live":
        raise AtomicProfileError("live profiles require version 2")

    item = _exact_keys(
        source,
        _V2_TOP_LEVEL_KEYS if version == 2 else _TOP_LEVEL_KEYS,
        "atomic profile",
    )
    joint_axes = _integer(item["joint_axes"], "joint_axes", minimum=6)
    if joint_axes > 7:
        raise AtomicProfileError("joint_axes must be 6 or 7")

    joint_limits = _joint_limits(item["joint_limits_deg"], joint_axes)
    dual_arms = _dual_arm_profiles(
        item["dual_arm_profiles"], joint_axes, joint_limits
    )
    waist = _waist_profiles(item["waist_profiles"])
    grippers = _gripper_profiles(item["gripper_profiles"])

    max_distance = _positive(
        item["max_chassis_distance_m"], "max_chassis_distance_m"
    )
    max_angle = _positive(
        item["max_chassis_angle_deg"], "max_chassis_angle_deg"
    )
    chassis = _chassis_profiles(
        item["chassis_profiles"],
        max_distance,
        max_angle,
        require_carry_role=version == 2,
    )
    review = _review(item["review"], mode)
    force = _force_gates(item["force_gates"])
    budgets = _positive_fields(item["budgets"], _BUDGET_KEYS, "budgets")
    freshness = _positive(
        item["robot_state_max_age_s"], "robot_state_max_age_s"
    )
    runtime_binding = (
        _runtime_binding(item["runtime_binding"])
        if version == 2
        else None
    )
    chassis_runtime = (
        _chassis_runtime(
            item["chassis_runtime"],
            budgets["stop_s"],
        )
        if version == 2
        else None
    )
    gripper_runtime = (
        _gripper_runtime(
            item["gripper_runtime"],
            budgets["stop_s"],
        )
        if version == 2
        else None
    )
    perception_binding = (
        _perception_binding(item["perception_binding"])
        if version == 2
        else None
    )
    subsystem_freshness = (
        _subsystem_freshness(item["subsystem_freshness"])
        if version == 2
        else None
    )
    carry_guards = (
        _carry_guards(
            item["carry_guards"],
            chassis,
            dual_arms,
            grippers,
        )
        if version == 2
        else {}
    )
    if version == 2:
        carrying_profiles = {
            name for name, item in chassis.items() if item.carries_box
        }
        if set(carry_guards) != carrying_profiles:
            raise AtomicProfileError(
                "every v2 carries_box chassis profile requires exactly one guard"
            )

    if budgets["chassis_s"] < min(item.timeout_s for item in chassis.values()):
        raise AtomicProfileError("chassis budget is smaller than a profile timeout")
    if budgets["waist_s"] < min(item.timeout_s for item in waist.values()):
        raise AtomicProfileError("waist budget is smaller than a profile timeout")
    if budgets["dual_arms_s"] < min(
        item.timeout_s for item in dual_arms.values()
    ):
        raise AtomicProfileError(
            "dual-arm budget is smaller than a profile timeout"
        )
    if budgets["grippers_s"] < min(
        item.timeout_s for item in grippers.values()
    ):
        raise AtomicProfileError(
            "gripper budget is smaller than a profile timeout"
        )

    return AtomicCapabilityProfile(
        profile_id=profile_id,
        version=version,
        mode=mode,
        joint_axes=joint_axes,
        joint_limits_deg=joint_limits,
        dual_arm_profiles=MappingProxyType(dual_arms),
        waist_profiles=MappingProxyType(waist),
        gripper_profiles=MappingProxyType(grippers),
        chassis_profiles=MappingProxyType(chassis),
        review=review,
        force_gates=force,
        budgets=MappingProxyType(budgets),
        robot_state_max_age_s=freshness,
        max_chassis_distance_m=max_distance,
        max_chassis_angle_deg=max_angle,
        runtime_binding=runtime_binding,
        chassis_runtime=chassis_runtime,
        gripper_runtime=gripper_runtime,
        perception_binding=perception_binding,
        subsystem_freshness=subsystem_freshness,
        carry_guards=MappingProxyType(carry_guards),
    )


def load_atomic_capability_profile(path: Path) -> AtomicCapabilityProfile:
    """Load and validate one atomic-capability profile JSON file."""

    try:
        source = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise AtomicProfileError(f"cannot read atomic profile {path}") from error
    return validate_atomic_capability_profile(source)


def atomic_profile_hash(profile: AtomicCapabilityProfile) -> str:
    """Return the stable SHA-256 identity used by capability evidence."""

    normalized = _public_document(profile)
    encoded = json.dumps(
        normalized,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _public_document(profile: AtomicCapabilityProfile) -> dict[str, Any]:
    return {
        "profile_id": profile.profile_id,
        "version": profile.version,
        "mode": profile.mode,
        "joint_axes": profile.joint_axes,
        "joint_limits_deg": [list(item) for item in profile.joint_limits_deg],
        "dual_arm_profiles": {
            name: {
                "left_joints_deg": [
                    list(point) for point in item.left_joints_deg
                ],
                "right_joints_deg": [
                    list(point) for point in item.right_joints_deg
                ],
                "tolerance_deg": item.tolerance_deg,
                "max_joint_step_deg": item.max_joint_step_deg,
                "start_tolerance_deg": item.start_tolerance_deg,
                "timeout_s": item.timeout_s,
            }
            for name, item in profile.dual_arm_profiles.items()
        },
        "waist_profiles": {
            name: {
                "position_mm": item.position_mm,
                "tolerance_mm": item.tolerance_mm,
                "timeout_s": item.timeout_s,
            }
            for name, item in profile.waist_profiles.items()
        },
        "gripper_profiles": {
            name: {
                "position_rad": item.position_rad,
                "tolerance_rad": item.tolerance_rad,
                "timeout_s": item.timeout_s,
            }
            for name, item in profile.gripper_profiles.items()
        },
        "chassis_profiles": {
            name: {
                "kind": item.kind,
                "goal": item.goal,
                "distance_m": item.distance_m,
                "angle_deg": item.angle_deg,
                "timeout_s": item.timeout_s,
                "carries_box": item.carries_box,
            }
            for name, item in profile.chassis_profiles.items()
        },
        "review": {
            "status": profile.review.status,
            "review_id": profile.review.review_id,
            "safety_evidence": list(profile.review.safety_evidence),
            "reviewer": profile.review.reviewer,
        },
        "force_gates": {
            "max_abs_force_n": list(profile.force_gates.max_abs_force_n),
            "max_abs_torque_nm": list(
                profile.force_gates.max_abs_torque_nm
            ),
            "hold_s": profile.force_gates.hold_s,
        },
        "budgets": dict(profile.budgets),
        "robot_state_max_age_s": profile.robot_state_max_age_s,
        "max_chassis_distance_m": profile.max_chassis_distance_m,
        "max_chassis_angle_deg": profile.max_chassis_angle_deg,
        "runtime_binding": None
        if profile.runtime_binding is None
        else {
            "site_id": profile.runtime_binding.site_id,
            "robot_id": profile.runtime_binding.robot_id,
            "lynrotcontrol_instance": (
                profile.runtime_binding.lynrotcontrol_instance
            ),
            "lynrotcontrol_config_sha256": (
                profile.runtime_binding.lynrotcontrol_config_sha256
            ),
            "execution_host": profile.runtime_binding.execution_host,
            "ros_domain_id": profile.runtime_binding.ros_domain_id,
            "source_artifacts": list(
                profile.runtime_binding.source_artifacts
            ),
        },
        "chassis_runtime": None
        if profile.chassis_runtime is None
        else {
            key: getattr(profile.chassis_runtime, key)
            for key in _CHASSIS_RUNTIME_KEYS
        },
        "gripper_runtime": None
        if profile.gripper_runtime is None
        else {
            key: getattr(profile.gripper_runtime, key)
            for key in _GRIPPER_RUNTIME_KEYS
        },
        "perception_binding": None
        if profile.perception_binding is None
        else {
            key: getattr(profile.perception_binding, key)
            for key in _PERCEPTION_BINDING_KEYS
        },
        "subsystem_freshness": None
        if profile.subsystem_freshness is None
        else {
            key: getattr(profile.subsystem_freshness, key)
            for key in _SUBSYSTEM_FRESHNESS_KEYS
        },
        "carry_guards": {
            name: {
                key: getattr(guard, key)
                for key in _CARRY_GUARD_KEYS
            }
            for name, guard in profile.carry_guards.items()
        },
    }


def _object(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise AtomicProfileError(f"{label} must be an object")
    return value


def _exact_keys(
    value: Any,
    expected: frozenset[str],
    label: str,
) -> dict[str, Any]:
    item = _object(value, label)
    missing = expected - item.keys()
    extra = item.keys() - expected
    if missing or extra:
        raise AtomicProfileError(
            f"{label} has missing keys {sorted(missing)} "
            f"and extra keys {sorted(extra)}"
        )
    return item


def _string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise AtomicProfileError(f"{label} must be a nonempty string")
    return value


def _integer(value: Any, label: str, *, minimum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise AtomicProfileError(f"{label} must be an integer >= {minimum}")
    return value


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AtomicProfileError(f"{label} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise AtomicProfileError(f"{label} must be finite")
    return result


def _positive(value: Any, label: str) -> float:
    result = _finite(value, label)
    if result <= 0.0:
        raise AtomicProfileError(f"{label} must be greater than zero")
    return result


def _nonnegative(value: Any, label: str) -> float:
    result = _finite(value, label)
    if result < 0.0:
        raise AtomicProfileError(f"{label} must be nonnegative")
    return result


def _boolean(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise AtomicProfileError(f"{label} must be a boolean")
    return value


def _topic(value: Any, label: str) -> str:
    topic = _string(value, label)
    if not topic.startswith("/") or topic != topic.strip():
        raise AtomicProfileError(f"{label} must be a canonical ROS name")
    return topic


def _sample_count(value: Any, label: str) -> int:
    return _integer(value, label, minimum=1)


def _source_artifacts(value: Any, label: str) -> tuple[str, ...]:
    artifacts = _string_list(value, label)
    if not artifacts or len(set(artifacts)) != len(artifacts):
        raise AtomicProfileError(f"{label} must contain unique evidence")
    return artifacts


def _string_list(value: Any, label: str) -> tuple[str, ...]:
    if not isinstance(value, list):
        raise AtomicProfileError(f"{label} must be an array")
    return tuple(_string(item, f"{label}[{index}]") for index, item in enumerate(value))


def _positive_fields(
    value: Any,
    expected: frozenset[str],
    label: str,
) -> dict[str, float]:
    fields = _exact_keys(value, expected, label)
    return {key: _positive(fields[key], f"{label}.{key}") for key in expected}


def _joint_limits(
    value: Any,
    joint_axes: int,
) -> tuple[tuple[float, float], ...]:
    if not isinstance(value, list) or len(value) != joint_axes:
        raise AtomicProfileError(
            f"joint_limits_deg must contain {joint_axes} entries"
        )
    result: list[tuple[float, float]] = []
    for index, item in enumerate(value):
        if not isinstance(item, list) or len(item) != 2:
            raise AtomicProfileError(
                f"joint_limits_deg[{index}] must contain min and max"
            )
        lower = _finite(item[0], f"joint_limits_deg[{index}][0]")
        upper = _finite(item[1], f"joint_limits_deg[{index}][1]")
        if lower >= upper:
            raise AtomicProfileError(
                f"joint_limits_deg[{index}] requires min < max"
            )
        result.append((lower, upper))
    return tuple(result)


def _joint_path(
    value: Any,
    label: str,
    joint_axes: int,
    limits: tuple[tuple[float, float], ...],
) -> tuple[tuple[float, ...], ...]:
    if not isinstance(value, list) or not value:
        raise AtomicProfileError(f"{label} must be a nonempty path array")
    result: list[tuple[float, ...]] = []
    for point_index, point in enumerate(value):
        if not isinstance(point, list) or len(point) != joint_axes:
            raise AtomicProfileError(
                f"{label}[{point_index}] must contain {joint_axes} joints"
            )
        normalized: list[float] = []
        for axis, raw in enumerate(point):
            joint = _finite(raw, f"{label}[{point_index}][{axis}]")
            lower, upper = limits[axis]
            if joint < lower or joint > upper:
                raise AtomicProfileError(
                    f"{label}[{point_index}][{axis}] is outside joint limits"
                )
            normalized.append(joint)
        result.append(tuple(normalized))
    return tuple(result)


def _named_objects(value: Any, label: str) -> dict[str, dict[str, Any]]:
    item = _object(value, label)
    if not item:
        raise AtomicProfileError(f"{label} must contain at least one profile")
    return item


def _dual_arm_profiles(
    value: Any,
    joint_axes: int,
    limits: tuple[tuple[float, float], ...],
) -> dict[str, DualArmJointProfile]:
    source = _named_objects(value, "dual_arm_profiles")
    result: dict[str, DualArmJointProfile] = {}
    for raw_name, raw_item in source.items():
        name = _string(raw_name, "dual_arm_profiles.name")
        fields = _exact_keys(raw_item, _DUAL_ARM_KEYS, f"dual_arm_profiles.{name}")
        left = _joint_path(
            fields["left_joints_deg"],
            f"dual_arm_profiles.{name}.left_joints_deg",
            joint_axes,
            limits,
        )
        right = _joint_path(
            fields["right_joints_deg"],
            f"dual_arm_profiles.{name}.right_joints_deg",
            joint_axes,
            limits,
        )
        if len(left) != len(right):
            raise AtomicProfileError(
                f"dual_arm_profiles.{name} requires equal path lengths"
            )
        max_step = _positive(
            fields["max_joint_step_deg"],
            f"dual_arm_profiles.{name}.max_joint_step_deg",
        )
        start_tolerance = _positive(
            fields["start_tolerance_deg"],
            f"dual_arm_profiles.{name}.start_tolerance_deg",
        )
        if start_tolerance > max_step:
            raise AtomicProfileError(
                f"dual_arm_profiles.{name}.start_tolerance_deg must not "
                "exceed max_joint_step_deg"
            )
        for side, path in (("left", left), ("right", right)):
            for previous, current in zip(path, path[1:]):
                for axis, (before, after) in enumerate(zip(previous, current)):
                    if abs(after - before) > max_step:
                        raise AtomicProfileError(
                            f"dual_arm_profiles.{name}.{side} exceeds "
                            f"max_joint_step_deg at joint {axis}"
                        )
        result[name] = DualArmJointProfile(
            left_joints_deg=left,
            right_joints_deg=right,
            tolerance_deg=_positive(
                fields["tolerance_deg"],
                f"dual_arm_profiles.{name}.tolerance_deg",
            ),
            max_joint_step_deg=max_step,
            start_tolerance_deg=start_tolerance,
            timeout_s=_positive(
                fields["timeout_s"],
                f"dual_arm_profiles.{name}.timeout_s",
            ),
        )
    return result


def _waist_profiles(value: Any) -> dict[str, WaistProfile]:
    source = _named_objects(value, "waist_profiles")
    result: dict[str, WaistProfile] = {}
    for raw_name, raw_item in source.items():
        name = _string(raw_name, "waist_profiles.name")
        fields = _exact_keys(raw_item, _WAIST_KEYS, f"waist_profiles.{name}")
        result[name] = WaistProfile(
            position_mm=_finite(
                fields["position_mm"], f"waist_profiles.{name}.position_mm"
            ),
            tolerance_mm=_positive(
                fields["tolerance_mm"],
                f"waist_profiles.{name}.tolerance_mm",
            ),
            timeout_s=_positive(
                fields["timeout_s"],
                f"waist_profiles.{name}.timeout_s",
            ),
        )
    return result


def _gripper_profiles(value: Any) -> dict[str, GripperProfile]:
    source = _named_objects(value, "gripper_profiles")
    if set(source) != {"open", "close"}:
        raise AtomicProfileError("gripper_profiles must be open and close")
    result: dict[str, GripperProfile] = {}
    for raw_name, raw_item in source.items():
        name = _string(raw_name, "gripper_profiles.name")
        fields = _exact_keys(
            raw_item, _GRIPPER_KEYS, f"gripper_profiles.{name}"
        )
        result[name] = GripperProfile(
            position_rad=_finite(
                fields["position_rad"],
                f"gripper_profiles.{name}.position_rad",
            ),
            tolerance_rad=_positive(
                fields["tolerance_rad"],
                f"gripper_profiles.{name}.tolerance_rad",
            ),
            timeout_s=_positive(
                fields["timeout_s"],
                f"gripper_profiles.{name}.timeout_s",
            ),
        )
    separation = (
        result["open"].position_rad - result["close"].position_rad
    )
    if separation <= 0.0:
        raise AtomicProfileError("open gripper position must exceed close")
    tolerance_sum = (
        result["open"].tolerance_rad + result["close"].tolerance_rad
    )
    if tolerance_sum >= separation:
        raise AtomicProfileError(
            "gripper tolerances must not overlap open and close targets"
        )
    return result


def _chassis_profiles(
    value: Any,
    max_distance_m: float,
    max_angle_deg: float,
    *,
    require_carry_role: bool,
) -> dict[str, ChassisProfile]:
    source = _named_objects(value, "chassis_profiles")
    result: dict[str, ChassisProfile] = {}
    for raw_name, raw_item in source.items():
        name = _string(raw_name, "chassis_profiles.name")
        item = _object(raw_item, f"chassis_profiles.{name}")
        kind = _string(item.get("kind"), f"chassis_profiles.{name}.kind")
        timeout = _positive(
            item.get("timeout_s"), f"chassis_profiles.{name}.timeout_s"
        )
        carries_box = (
            _boolean(item.get("carries_box"), f"chassis_profiles.{name}.carries_box")
            if require_carry_role
            else False
        )
        if kind == "nav_goal":
            fields = _exact_keys(
                item,
                frozenset({"kind", "goal", "timeout_s", "carries_box"})
                if require_carry_role
                else frozenset({"kind", "goal", "timeout_s"}),
                f"chassis_profiles.{name}",
            )
            result[name] = ChassisProfile(
                kind=kind,
                goal=_string(
                    fields["goal"], f"chassis_profiles.{name}.goal"
                ),
                distance_m=None,
                angle_deg=None,
                timeout_s=timeout,
                carries_box=carries_box,
            )
            continue
        if kind == "move_distance":
            fields = _exact_keys(
                item,
                frozenset(
                    {"kind", "distance_m", "angle_deg", "timeout_s", "carries_box"}
                )
                if require_carry_role
                else frozenset(
                    {"kind", "distance_m", "angle_deg", "timeout_s"}
                ),
                f"chassis_profiles.{name}",
            )
            distance = _finite(
                fields["distance_m"],
                f"chassis_profiles.{name}.distance_m",
            )
            angle = _finite(
                fields["angle_deg"],
                f"chassis_profiles.{name}.angle_deg",
            )
            if distance == 0.0 and angle == 0.0:
                raise AtomicProfileError(
                    f"chassis_profiles.{name} must move distance or angle"
                )
            if abs(distance) > max_distance_m:
                raise AtomicProfileError(
                    f"chassis_profiles.{name}.distance_m exceeds site bound"
                )
            if abs(angle) > max_angle_deg:
                raise AtomicProfileError(
                    f"chassis_profiles.{name}.angle_deg exceeds site bound"
                )
            result[name] = ChassisProfile(
                kind=kind,
                goal=None,
                distance_m=distance,
                angle_deg=angle,
                timeout_s=timeout,
                carries_box=carries_box,
            )
            continue
        raise AtomicProfileError(
            f"chassis_profiles.{name}.kind must be nav_goal or move_distance"
        )
    return result


def _runtime_binding(value: Any) -> RuntimeBinding:
    fields = _exact_keys(value, _RUNTIME_BINDING_KEYS, "runtime_binding")
    config_sha256 = _string(
        fields["lynrotcontrol_config_sha256"],
        "runtime_binding.lynrotcontrol_config_sha256",
    )
    if (
        len(config_sha256) != 64
        or any(char not in "0123456789abcdef" for char in config_sha256)
    ):
        raise AtomicProfileError(
            "runtime_binding.lynrotcontrol_config_sha256 must be lowercase SHA-256"
        )
    ros_domain_id = _integer(
        fields["ros_domain_id"],
        "runtime_binding.ros_domain_id",
        minimum=0,
    )
    if ros_domain_id > 232:
        raise AtomicProfileError(
            "runtime_binding.ros_domain_id must be a valid ROS domain"
        )
    return RuntimeBinding(
        site_id=_string(fields["site_id"], "runtime_binding.site_id"),
        robot_id=_string(fields["robot_id"], "runtime_binding.robot_id"),
        lynrotcontrol_instance=_string(
            fields["lynrotcontrol_instance"],
            "runtime_binding.lynrotcontrol_instance",
        ),
        lynrotcontrol_config_sha256=config_sha256,
        execution_host=_string(
            fields["execution_host"],
            "runtime_binding.execution_host",
        ),
        ros_domain_id=ros_domain_id,
        source_artifacts=_source_artifacts(
            fields["source_artifacts"],
            "runtime_binding.source_artifacts",
        ),
    )


def _chassis_runtime(
    value: Any,
    stop_budget_s: float,
) -> ChassisRuntimeProfile:
    fields = _exact_keys(value, _CHASSIS_RUNTIME_KEYS, "chassis_runtime")
    kind = _string(fields["kind"], "chassis_runtime.kind")
    if kind != "bounded_odom_cmd_vel":
        raise AtomicProfileError(
            "chassis_runtime.kind must be bounded_odom_cmd_vel"
        )
    odom_topic = _topic(
        fields["odom_topic"],
        "chassis_runtime.odom_topic",
    )
    cmd_vel_topic = _topic(
        fields["cmd_vel_topic"],
        "chassis_runtime.cmd_vel_topic",
    )
    if odom_topic == cmd_vel_topic:
        raise AtomicProfileError("chassis_runtime topics must be distinct")

    linear_speed = _nonnegative(
        fields["max_linear_speed_m_s"],
        "chassis_runtime.max_linear_speed_m_s",
    )
    angular_speed = _nonnegative(
        fields["max_angular_speed_rad_s"],
        "chassis_runtime.max_angular_speed_rad_s",
    )
    linear_accel = _nonnegative(
        fields["max_linear_accel_m_s2"],
        "chassis_runtime.max_linear_accel_m_s2",
    )
    angular_accel = _nonnegative(
        fields["max_angular_accel_rad_s2"],
        "chassis_runtime.max_angular_accel_rad_s2",
    )
    if linear_speed <= 0.0 and angular_speed <= 0.0:
        raise AtomicProfileError("chassis_runtime requires a positive speed")
    if linear_accel <= 0.0 and angular_accel <= 0.0:
        raise AtomicProfileError(
            "chassis_runtime requires a positive acceleration"
        )

    watchdog_s = _positive(
        fields["watchdog_timeout_s"],
        "chassis_runtime.watchdog_timeout_s",
    )
    settle_s = _positive(
        fields["stop_settle_timeout_s"],
        "chassis_runtime.stop_settle_timeout_s",
    )
    if watchdog_s > stop_budget_s or settle_s > stop_budget_s:
        raise AtomicProfileError(
            "chassis_runtime stop timing exceeds budgets.stop_s"
        )
    return ChassisRuntimeProfile(
        kind=kind,
        odom_topic=odom_topic,
        cmd_vel_topic=cmd_vel_topic,
        max_linear_speed_m_s=linear_speed,
        max_angular_speed_rad_s=angular_speed,
        max_linear_accel_m_s2=linear_accel,
        max_angular_accel_rad_s2=angular_accel,
        command_rate_hz=_positive(
            fields["command_rate_hz"],
            "chassis_runtime.command_rate_hz",
        ),
        watchdog_timeout_s=watchdog_s,
        stop_settle_timeout_s=settle_s,
        zero_twist_confirm_samples=_sample_count(
            fields["zero_twist_confirm_samples"],
            "chassis_runtime.zero_twist_confirm_samples",
        ),
        odom_max_age_s=_positive(
            fields["odom_max_age_s"],
            "chassis_runtime.odom_max_age_s",
        ),
        source_artifacts=_source_artifacts(
            fields["source_artifacts"],
            "chassis_runtime.source_artifacts",
        ),
    )


def _gripper_runtime(
    value: Any,
    stop_budget_s: float,
) -> GripperRuntimeProfile:
    fields = _exact_keys(value, _GRIPPER_RUNTIME_KEYS, "gripper_runtime")
    failure_policy = _string(
        fields["failure_policy"],
        "gripper_runtime.failure_policy",
    )
    if failure_policy != "confirmed_stop_then_operator_review":
        raise AtomicProfileError(
            "gripper_runtime.failure_policy is not confirmed stop"
        )
    if (
        _boolean(
            fields["requires_stop_api"],
            "gripper_runtime.requires_stop_api",
        )
        is not True
    ):
        raise AtomicProfileError(
            "gripper_runtime.requires_stop_api must be true"
        )
    stop_timeout_s = _positive(
        fields["stop_timeout_s"],
        "gripper_runtime.stop_timeout_s",
    )
    if stop_timeout_s > stop_budget_s:
        raise AtomicProfileError(
            "gripper_runtime.stop_timeout_s exceeds budgets.stop_s"
        )
    return GripperRuntimeProfile(
        failure_policy=failure_policy,
        requires_stop_api=True,
        stop_timeout_s=stop_timeout_s,
        zero_motion_confirm_samples=_sample_count(
            fields["zero_motion_confirm_samples"],
            "gripper_runtime.zero_motion_confirm_samples",
        ),
        source_artifacts=_source_artifacts(
            fields["source_artifacts"],
            "gripper_runtime.source_artifacts",
        ),
    )


def _perception_binding(value: Any) -> PerceptionBinding:
    fields = _exact_keys(
        value,
        _PERCEPTION_BINDING_KEYS,
        "perception_binding",
    )
    pose_topic = _topic(
        fields["pose_topic"],
        "perception_binding.pose_topic",
    )
    status_topic = _topic(
        fields["status_topic"],
        "perception_binding.status_topic",
    )
    trigger_service = _topic(
        fields["trigger_service"],
        "perception_binding.trigger_service",
    )
    if len({pose_topic, status_topic, trigger_service}) != 3:
        raise AtomicProfileError(
            "perception_binding interfaces must be distinct"
        )
    trigger_timeout_s = _positive(
        fields["trigger_timeout_s"],
        "perception_binding.trigger_timeout_s",
    )
    result_deadline_s = _positive(
        fields["result_deadline_s"],
        "perception_binding.result_deadline_s",
    )
    if result_deadline_s < trigger_timeout_s:
        raise AtomicProfileError(
            "perception_binding.result_deadline_s is below trigger timeout"
        )
    return PerceptionBinding(
        pose_topic=pose_topic,
        status_topic=status_topic,
        trigger_service=trigger_service,
        expected_frame=_string(
            fields["expected_frame"],
            "perception_binding.expected_frame",
        ),
        pose_max_age_s=_positive(
            fields["pose_max_age_s"],
            "perception_binding.pose_max_age_s",
        ),
        trigger_timeout_s=trigger_timeout_s,
        result_deadline_s=result_deadline_s,
        source_artifacts=_source_artifacts(
            fields["source_artifacts"],
            "perception_binding.source_artifacts",
        ),
    )


def _subsystem_freshness(value: Any) -> SubsystemFreshness:
    fields = _exact_keys(
        value,
        _SUBSYSTEM_FRESHNESS_KEYS,
        "subsystem_freshness",
    )
    values = {
        key: _positive(fields[key], f"subsystem_freshness.{key}")
        for key in _SUBSYSTEM_FRESHNESS_KEYS
    }
    return SubsystemFreshness(**values)


def _carry_guards(
    value: Any,
    chassis_profiles: Mapping[str, ChassisProfile],
    arm_profiles: Mapping[str, DualArmJointProfile],
    gripper_profiles: Mapping[str, GripperProfile],
) -> dict[str, CarryGuard]:
    source = _object(value, "carry_guards")
    result: dict[str, CarryGuard] = {}
    for raw_name, raw_item in source.items():
        name = _string(raw_name, "carry_guards.name")
        if name not in chassis_profiles:
            raise AtomicProfileError(
                f"carry_guards.{name} must reference a chassis profile"
            )
        fields = _exact_keys(
            raw_item,
            _CARRY_GUARD_KEYS,
            f"carry_guards.{name}",
        )
        arm_profile = _string(
            fields["arm_profile"],
            f"carry_guards.{name}.arm_profile",
        )
        if arm_profile not in arm_profiles:
            raise AtomicProfileError(
                f"carry_guards.{name}.arm_profile is unknown"
            )
        gripper_command = _string(
            fields["gripper_command"],
            f"carry_guards.{name}.gripper_command",
        )
        if (
            gripper_command != "close"
            or gripper_command not in gripper_profiles
        ):
            raise AtomicProfileError(
                f"carry_guards.{name}.gripper_command must be close"
            )
        box_control_status = _string(
            fields["box_control_status"],
            f"carry_guards.{name}.box_control_status",
        )
        if box_control_status != "held":
            raise AtomicProfileError(
                f"carry_guards.{name}.box_control_status must be held"
            )
        force_monitor = _string(
            fields["force_monitor"],
            f"carry_guards.{name}.force_monitor",
        )
        if force_monitor != "both_arms":
            raise AtomicProfileError(
                f"carry_guards.{name}.force_monitor must be both_arms"
            )
        result[name] = CarryGuard(
            arm_profile=arm_profile,
            gripper_command=gripper_command,
            box_control_status=box_control_status,
            force_monitor=force_monitor,
            source_artifacts=_source_artifacts(
                fields["source_artifacts"],
                f"carry_guards.{name}.source_artifacts",
            ),
        )
    return result


def _review(value: Any, mode: str) -> AtomicReview:
    fields = _exact_keys(value, _REVIEW_KEYS, "review")
    status = _string(fields["status"], "review.status")
    evidence = _string_list(
        fields["safety_evidence"], "review.safety_evidence"
    )
    reviewer = fields["reviewer"]
    if mode == "live":
        if status != "site_confirmed":
            raise AtomicProfileError(
                "live review.status must be site_confirmed"
            )
        reviewer = _string(reviewer, "review.reviewer")
        if not evidence:
            raise AtomicProfileError("live review requires safety evidence")
    else:
        if status != "offline_fixture_only":
            raise AtomicProfileError(
                "dry_run review.status must be offline_fixture_only"
            )
        if reviewer is not None:
            raise AtomicProfileError(
                "dry_run review.reviewer must be null"
            )
    return AtomicReview(
        status=status,
        review_id=_string(fields["review_id"], "review.review_id"),
        safety_evidence=evidence,
        reviewer=reviewer,
    )


def _force_gates(value: Any) -> ForceGates:
    fields = _exact_keys(value, _FORCE_KEYS, "force_gates")
    forces = _positive_vector(
        fields["max_abs_force_n"], "force_gates.max_abs_force_n"
    )
    torques = _positive_vector(
        fields["max_abs_torque_nm"], "force_gates.max_abs_torque_nm"
    )
    return ForceGates(
        max_abs_force_n=forces,
        max_abs_torque_nm=torques,
        hold_s=_positive(fields["hold_s"], "force_gates.hold_s"),
    )


def _positive_vector(value: Any, label: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise AtomicProfileError(f"{label} must contain three positive values")
    return tuple(
        _positive(item, f"{label}[{index}]") for index, item in enumerate(value)
    )
