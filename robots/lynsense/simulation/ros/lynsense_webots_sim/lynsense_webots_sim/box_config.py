from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


EXPECTED_TARGETS = frozenset(
    {
        "dualjo:joints_s",
        "dualjo:joints_br",
        "dualposi_armbase_abso:pt_1f1_ready",
        "dualposi_armbase_abso:pt_up",
    }
)


class ConfigError(ValueError):
    pass


@dataclass(frozen=True)
class BoxGeometry:
    dimensions_m: tuple[float, float, float]
    mass_kg: float
    initial_pose_frame: str
    initial_pose_m: tuple[float, float, float]
    initial_yaw_rad: float
    place_target_frame: str
    place_target_m: tuple[float, float, float]
    place_target_yaw_rad: float


@dataclass(frozen=True)
class WaistConfig:
    motor_name: str
    position_sensor_name: str
    lower_m: float
    upper_m: float
    max_velocity_mps: float
    height_reference_m: float
    height_positive_up: bool
    arrival_tolerance_m: float


@dataclass(frozen=True)
class JointConfig:
    joint_names: tuple[str, ...]
    max_velocity_radps: tuple[float, ...]
    max_acceleration_radps2: tuple[float, ...]
    limits_rad: dict[str, tuple[float, float]]


@dataclass(frozen=True)
class GripperConfig:
    motor_name: str
    position_sensor_name: str
    closed_position: float
    open_position: float
    arrival_tolerance: float
    max_velocity: float


@dataclass(frozen=True)
class NamedTarget:
    waist_height_mm: float
    gripper_position: float
    left_joints_rad: tuple[float, ...]
    right_joints_rad: tuple[float, ...]


@dataclass(frozen=True)
class Thresholds:
    joint_arrival_rad: float
    waist_arrival_m: float
    gripper_arrival: float
    grasp_alignment_m: float
    grasp_yaw_rad: float
    carry_drift_m: float
    place_alignment_m: float
    place_yaw_rad: float
    release_roll_pitch_rad: float
    release_settle_s: float


@dataclass(frozen=True)
class Timeouts:
    waist_s: float
    named_config_s: float
    gripper_s: float
    box_phase_s: float
    stall_window_s: float
    joint_stall_threshold_rad: float
    total_budget_s: float


@dataclass(frozen=True)
class BoxConfig:
    box: BoxGeometry
    waist: WaistConfig
    left_arm: JointConfig
    right_arm: JointConfig
    grippers: dict[str, GripperConfig]
    named_targets: dict[str, NamedTarget]
    thresholds: Thresholds
    timeouts: Timeouts


def load_box_config(path: Path) -> BoxConfig:
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigError(f"could not read box config: {exc}") from exc
    if not isinstance(raw, dict):
        raise ConfigError("box config must be a mapping")

    box = _box(raw["box"])
    waist = _waist(raw["waist"])
    left_arm = _arm(raw["left_arm"], "left")
    right_arm = _arm(raw["right_arm"], "right")
    grippers = _grippers(raw["grippers"])
    named_targets = _targets(raw["named_targets"], left_arm, right_arm, waist, grippers)
    thresholds = _thresholds(raw["thresholds"])
    timeouts = _timeouts(raw["timeouts"])
    return BoxConfig(
        box=box,
        waist=waist,
        left_arm=left_arm,
        right_arm=right_arm,
        grippers=grippers,
        named_targets=named_targets,
        thresholds=thresholds,
        timeouts=timeouts,
    )


def _mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ConfigError(f"{name} must be a mapping")
    return value


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ConfigError(f"{name} must be finite")
    return result


def _positive(value: Any, name: str) -> float:
    result = _number(value, name)
    if result <= 0.0:
        raise ConfigError(f"{name} must be positive")
    return result


def _vector3(value: Any, name: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ConfigError(f"{name} must contain three values")
    return tuple(_number(item, f"{name}[{index}]") for index, item in enumerate(value))


def _six(value: Any, name: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != 6:
        raise ConfigError(f"{name} must contain six values")
    return tuple(_number(item, f"{name}[{index}]") for index, item in enumerate(value))


def _box(value: Any) -> BoxGeometry:
    source = _mapping(value, "box")
    dimensions = _vector3(source.get("dimensions_m"), "box.dimensions_m")
    if any(dimension <= 0.0 for dimension in dimensions):
        raise ConfigError("box dimensions must be positive")
    return BoxGeometry(
        dimensions_m=dimensions,
        mass_kg=_positive(source.get("mass_kg"), "box.mass_kg"),
        initial_pose_frame=_frame(source.get("initial_pose_frame"), "box.initial_pose_frame"),
        initial_pose_m=_vector3(source.get("initial_pose_m"), "box.initial_pose_m"),
        initial_yaw_rad=_number(source.get("initial_yaw_rad"), "box.initial_yaw_rad"),
        place_target_frame=_frame(source.get("place_target_frame"), "box.place_target_frame"),
        place_target_m=_vector3(source.get("place_target_m"), "box.place_target_m"),
        place_target_yaw_rad=_number(source.get("place_target_yaw_rad"), "box.place_target_yaw_rad"),
    )


def _frame(value: Any, name: str) -> str:
    if value != "world":
        raise ConfigError(f"{name} must be world")
    return str(value)


def _waist(value: Any) -> WaistConfig:
    source = _mapping(value, "waist")
    lower = _number(source.get("lower_m"), "waist.lower_m")
    upper = _number(source.get("upper_m"), "waist.upper_m")
    if lower >= upper:
        raise ConfigError("waist lower limit must be below upper limit")
    positive_up = source.get("height_positive_up")
    if not isinstance(positive_up, bool) or not positive_up:
        raise ConfigError("waist.height_positive_up must be true")
    reference = _number(source.get("height_reference_m"), "waist.height_reference_m")
    if not lower <= reference <= upper:
        raise ConfigError("waist height reference is outside limits")
    return WaistConfig(
        motor_name=_device(source.get("motor_name"), "waist.motor_name"),
        position_sensor_name=_device(source.get("position_sensor_name"), "waist.position_sensor_name"),
        lower_m=lower,
        upper_m=upper,
        max_velocity_mps=_positive(source.get("max_velocity_mps"), "waist.max_velocity_mps"),
        height_reference_m=reference,
        height_positive_up=positive_up,
        arrival_tolerance_m=_positive(source.get("arrival_tolerance_m"), "waist.arrival_tolerance_m"),
    )


def _arm(value: Any, side: str) -> JointConfig:
    source = _mapping(value, f"{side}_arm")
    expected = tuple(f"{side}_joint{index}" for index in range(1, 7))
    names_value = source.get("joint_names")
    if not isinstance(names_value, list):
        raise ConfigError(f"{side}_arm.joint_names must be a list")
    names = tuple(str(item) for item in names_value)
    if len(set(names)) != len(names):
        raise ConfigError(f"{side}_arm.joint_names contains a duplicate joint name")
    if names != expected or len(set(names)) != 6:
        raise ConfigError(f"{side}_arm.joint_names must be the six ordered {side} joints")
    velocity = _six(source.get("max_velocity_radps"), f"{side}_arm.max_velocity_radps")
    acceleration = _six(source.get("max_acceleration_radps2"), f"{side}_arm.max_acceleration_radps2")
    if any(item <= 0.0 for item in velocity + acceleration):
        raise ConfigError(f"{side}_arm velocity and acceleration must be positive")
    limits_value = _mapping(source.get("limits_rad"), f"{side}_arm.limits_rad")
    if set(limits_value) != set(names):
        raise ConfigError(f"{side}_arm.limits_rad does not match joint names")
    limits: dict[str, tuple[float, float]] = {}
    for name in names:
        pair = limits_value[name]
        if not isinstance(pair, list) or len(pair) != 2:
            raise ConfigError(f"{side}_arm.limits_rad.{name} must contain two values")
        lower = _number(pair[0], f"{side}_arm.limits_rad.{name}.lower")
        upper = _number(pair[1], f"{side}_arm.limits_rad.{name}.upper")
        if lower >= upper:
            raise ConfigError(f"{side}_arm.limits_rad.{name} lower must be below upper")
        limits[name] = (lower, upper)
    return JointConfig(names, velocity, acceleration, limits)


def _grippers(value: Any) -> dict[str, GripperConfig]:
    source = _mapping(value, "grippers")
    result: dict[str, GripperConfig] = {}
    for side in ("left", "right"):
        item = _mapping(source.get(side), f"grippers.{side}")
        closed = _number(item.get("closed_position"), f"grippers.{side}.closed_position")
        opened = _number(item.get("open_position"), f"grippers.{side}.open_position")
        if closed >= opened:
            raise ConfigError(f"grippers.{side} closed position must be below open position")
        result[side] = GripperConfig(
            motor_name=_device(item.get("motor_name"), f"grippers.{side}.motor_name"),
            position_sensor_name=_device(item.get("position_sensor_name"), f"grippers.{side}.position_sensor_name"),
            closed_position=closed,
            open_position=opened,
            arrival_tolerance=_positive(item.get("arrival_tolerance"), f"grippers.{side}.arrival_tolerance"),
            max_velocity=_positive(item.get("max_velocity"), f"grippers.{side}.max_velocity"),
        )
    if set(source) != set(result):
        raise ConfigError("grippers must contain only left and right")
    return result


def _targets(
    value: Any,
    left_arm: JointConfig,
    right_arm: JointConfig,
    waist: WaistConfig,
    grippers: dict[str, GripperConfig],
) -> dict[str, NamedTarget]:
    source = _mapping(value, "named_targets")
    if set(source) != set(EXPECTED_TARGETS):
        raise ConfigError("named_targets does not match the allowed whitelist")
    result: dict[str, NamedTarget] = {}
    for name, raw_item in source.items():
        item = _mapping(raw_item, f"named_targets.{name}")
        left = _six(item.get("left_joints_rad"), f"named_targets.{name}.left_joints_rad")
        right = _six(item.get("right_joints_rad"), f"named_targets.{name}.right_joints_rad")
        _within_limits(left, left_arm, name, "left")
        _within_limits(right, right_arm, name, "right")
        height = _number(item.get("waist_height_mm"), f"named_targets.{name}.waist_height_mm")
        waist_position = waist.height_reference_m + height / 1000.0
        if not waist.lower_m <= waist_position <= waist.upper_m:
            raise ConfigError(f"named_targets.{name} waist height is outside limits")
        gripper = _number(item.get("gripper_position"), f"named_targets.{name}.gripper_position")
        closed = min(config.closed_position for config in grippers.values())
        opened = max(config.open_position for config in grippers.values())
        if not closed <= gripper <= opened:
            raise ConfigError(f"named_targets.{name} gripper position is outside limits")
        result[name] = NamedTarget(height, gripper, left, right)
    return result


def _within_limits(values: tuple[float, ...], arm: JointConfig, target: str, side: str) -> None:
    for joint, value in zip(arm.joint_names, values):
        lower, upper = arm.limits_rad[joint]
        if not lower <= value <= upper:
            raise ConfigError(f"named target {target} {side} joint {joint} is outside limits")


def _thresholds(value: Any) -> Thresholds:
    source = _mapping(value, "thresholds")
    fields = {
        "joint_arrival_rad": "joint_arrival_rad",
        "waist_arrival_m": "waist_arrival_m",
        "gripper_arrival": "gripper_arrival",
        "grasp_alignment_m": "grasp_alignment_m",
        "grasp_yaw_rad": "grasp_yaw_rad",
        "carry_drift_m": "carry_drift_m",
        "place_alignment_m": "place_alignment_m",
        "place_yaw_rad": "place_yaw_rad",
        "release_roll_pitch_rad": "release_roll_pitch_rad",
        "release_settle_s": "release_settle_s",
    }
    if set(source) != set(fields):
        raise ConfigError("thresholds fields do not match the contract")
    result = {field: _positive(source[field], f"thresholds.{field}") for field in fields}
    for field in ("grasp_yaw_rad", "place_yaw_rad", "release_roll_pitch_rad"):
        if result[field] > math.pi:
            raise ConfigError(f"thresholds.{field} must not exceed pi")
    return Thresholds(**result)


def _timeouts(value: Any) -> Timeouts:
    source = _mapping(value, "timeouts")
    fields = {
        "waist_s": "waist_s",
        "named_config_s": "named_config_s",
        "gripper_s": "gripper_s",
        "box_phase_s": "box_phase_s",
        "stall_window_s": "stall_window_s",
        "joint_stall_threshold_rad": "joint_stall_threshold_rad",
        "total_budget_s": "total_budget_s",
    }
    if set(source) != set(fields):
        raise ConfigError("timeouts fields do not match the contract")
    return Timeouts(**{field: _positive(source[field], f"timeouts.{field}") for field in fields})


def _device(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ConfigError(f"{name} must be a non-empty string")
    return value
