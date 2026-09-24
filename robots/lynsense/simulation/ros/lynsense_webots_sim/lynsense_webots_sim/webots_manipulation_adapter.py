from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from lynsense_webots_sim.box_config import BoxConfig
from lynsense_webots_sim.geometry import Pose2, pi_symmetric_angular_error
from lynsense_webots_sim.manipulation_runtime import (
    ManipulationCommands,
    ManipulationState,
)


BOX_NODE_NAME = "BOX_1"
WHEEL_MOTOR_NAMES = ("left_wheel_motor", "right_wheel_motor")
MOTOR_NAMES = (
    *WHEEL_MOTOR_NAMES,
    "waist_motor",
    *(f"left_joint{i}_motor" for i in range(1, 7)),
    *(f"right_joint{i}_motor" for i in range(1, 7)),
    "left_gripper_motor",
    "right_gripper_motor",
)
SENSOR_NAMES = (
    "waist_sensor",
    *(f"left_joint{i}_sensor" for i in range(1, 7)),
    *(f"right_joint{i}_sensor" for i in range(1, 7)),
    "left_gripper_sensor",
    "right_gripper_sensor",
)


@dataclass(frozen=True)
class _CarryTransform:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float


class WebotsManipulationAdapter:
    def __init__(self, robot: Any, config: BoxConfig) -> None:
        self._robot = robot
        self._config = config
        self._motors: dict[str, Any] = {}
        self._sensors: dict[str, Any] = {}
        self._valid_sensors: set[str] = set()
        self._box_node: Any | None = None
        self._last_robot_pose: Pose2 | None = None
        self._attachment: _CarryTransform | None = None
        self._last_motor_velocities = {name: 0.0 for name in MOTOR_NAMES}

        expected_motors = (
            config.waist.motor_name,
            *(
                f"{joint}_motor"
                for joint in (*config.left_arm.joint_names, *config.right_arm.joint_names)
            ),
            config.grippers["left"].motor_name,
            config.grippers["right"].motor_name,
        )
        expected_sensors = (
            config.waist.position_sensor_name,
            *(
                f"{joint}_sensor"
                for joint in (*config.left_arm.joint_names, *config.right_arm.joint_names)
            ),
            config.grippers["left"].position_sensor_name,
            config.grippers["right"].position_sensor_name,
        )
        if set(expected_motors) | set(WHEEL_MOTOR_NAMES) != set(MOTOR_NAMES):
            raise ValueError("box config devices do not match the adapter contract")
        if set(expected_sensors) != set(SENSOR_NAMES):
            raise ValueError("box config sensors do not match the adapter contract")

        for name in MOTOR_NAMES:
            motor = self._motor(name)
            motor.setPosition(math.inf)
            self._set_motor_velocity(motor, name, 0.0)

    @property
    def last_motor_velocities(self) -> dict[str, float]:
        return dict(self._last_motor_velocities)

    def read(self, robot_pose: Pose2, sim_time_s: float) -> ManipulationState:
        _require_finite(sim_time_s, "sim_time_s")
        _require_finite(robot_pose.x, "robot_pose.x")
        _require_finite(robot_pose.y, "robot_pose.y")
        _require_finite(robot_pose.yaw, "robot_pose.yaw")
        self._last_robot_pose = robot_pose

        waist = self._sensor_value(self._config.waist.position_sensor_name)
        arm_positions = tuple(
            self._sensor_value(f"{joint}_sensor")
            for joint in (
                *self._config.left_arm.joint_names,
                *self._config.right_arm.joint_names,
            )
        )
        gripper_positions = (
            self._sensor_value(self._config.grippers["left"].position_sensor_name),
            self._sensor_value(self._config.grippers["right"].position_sensor_name),
        )
        box_position, box_orientation = self._read_box(robot_pose)
        return ManipulationState(
            sim_time_s=float(sim_time_s),
            waist_position_m=waist,
            arm_positions_rad=arm_positions,
            gripper_positions=gripper_positions,
            box_position_m=box_position,
            box_orientation_rad=box_orientation,
            robot_pose=robot_pose,
            box_attached=self._attachment is not None,
        )

    def write(self, commands: ManipulationCommands) -> None:
        velocities = (
            commands.waist_velocity_mps,
            *commands.arm_velocities_radps,
            *commands.gripper_velocities,
        )
        if len(commands.arm_velocities_radps) != 12 or len(commands.gripper_velocities) != 2:
            raise ValueError("manipulation command cardinality is invalid")
        if not all(math.isfinite(float(value)) for value in velocities):
            raise ValueError("manipulation command values must be finite")

        mapped = (
            *WHEEL_MOTOR_NAMES,
            self._config.waist.motor_name,
            *(f"{joint}_motor" for joint in self._config.left_arm.joint_names),
            *(f"{joint}_motor" for joint in self._config.right_arm.joint_names),
            self._config.grippers["left"].motor_name,
            self._config.grippers["right"].motor_name,
        )
        values = (
            0.0,
            0.0,
            commands.waist_velocity_mps,
            *commands.arm_velocities_radps[:6],
            *commands.arm_velocities_radps[6:],
            *commands.gripper_velocities,
        )
        for name, velocity in zip(mapped, values):
            self._set_motor_velocity(self._motor(name), name, float(velocity))

    def attach(self) -> None:
        robot_pose = self._last_robot_pose
        if robot_pose is None:
            raise RuntimeError("cannot attach before reading robot pose")
        position, orientation = self._node_pose()
        dx = position[0] - robot_pose.x
        dy = position[1] - robot_pose.y
        self._attachment = _CarryTransform(
            x_m=math.cos(robot_pose.yaw) * dx + math.sin(robot_pose.yaw) * dy,
            y_m=-math.sin(robot_pose.yaw) * dx + math.cos(robot_pose.yaw) * dy,
            z_m=position[2],
            yaw_rad=pi_symmetric_angular_error(
                robot_pose.yaw, orientation[2]
            ),
        )

    def release(self) -> None:
        self._attachment = None

    def stop_all(self) -> None:
        for name in MOTOR_NAMES:
            self._set_motor_velocity(self._motor(name), name, 0.0)

    def _set_motor_velocity(self, motor: Any, name: str, velocity: float) -> None:
        motor.setVelocity(velocity)
        self._last_motor_velocities[name] = float(velocity)

    def _motor(self, name: str) -> Any:
        if name not in self._motors:
            motor = self._robot.getDevice(name)
            if motor is None:
                raise RuntimeError(f"Webots motor is unavailable: {name}")
            self._motors[name] = motor
        return self._motors[name]

    def _sensor_value(self, name: str) -> float:
        if name not in self._sensors:
            sensor = self._robot.getDevice(name)
            if sensor is None:
                raise RuntimeError(f"Webots position sensor is unavailable: {name}")
            sensor.enable(32)
            self._sensors[name] = sensor
        raw_value = float(self._sensors[name].getValue())
        if math.isnan(raw_value) and name not in self._valid_sensors:
            return 0.0
        self._valid_sensors.add(name)
        value = raw_value
        _require_finite(value, f"sensor {name}")
        return value

    def _read_box(
        self, robot_pose: Pose2
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        position, orientation = self._node_pose()
        attachment = self._attachment
        if attachment is None:
            return position, orientation

        x = robot_pose.x + math.cos(robot_pose.yaw) * attachment.x_m - math.sin(
            robot_pose.yaw
        ) * attachment.y_m
        y = robot_pose.y + math.sin(robot_pose.yaw) * attachment.x_m + math.cos(
            robot_pose.yaw
        ) * attachment.y_m
        roll, pitch, _ = orientation
        yaw = robot_pose.yaw + attachment.yaw_rad
        carried_position = (float(x), float(y), position[2])
        carried_orientation = (roll, pitch, float(yaw))
        self._write_box_pose(carried_position, carried_orientation)
        return carried_position, carried_orientation

    def _node_pose(
        self,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        if self._box_node is None:
            node = self._robot.getFromDef(BOX_NODE_NAME)
            if node is None:
                raise RuntimeError(f"Webots supervisor node is unavailable: {BOX_NODE_NAME}")
            self._box_node = node
        position_values = self._box_node.getPosition()
        orientation_values = self._box_node.getOrientation()
        if len(position_values) != 3 or len(orientation_values) != 9:
            raise ValueError("Webots box node returned invalid pose dimensions")
        if not all(
            math.isfinite(float(value))
            for value in (*position_values, *orientation_values)
        ):
            raise ValueError("Webots box node pose values must be finite")
        position = tuple(float(value) for value in position_values)
        orientation = _euler_from_orientation(tuple(float(value) for value in orientation_values))
        return position, orientation

    def _write_box_pose(
        self,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float],
    ) -> None:
        if self._box_node is None:
            raise RuntimeError("box node has not been read")
        translation_field = self._box_node.getField("translation")
        rotation_field = self._box_node.getField("rotation")
        translation_field.setSFVec3f(list(position))
        rotation_field.setSFRotation(_rotation_from_euler(orientation))


def _euler_from_orientation(
    values: tuple[float, ...]
) -> tuple[float, float, float]:
    roll = math.atan2(values[7], values[8])
    pitch = math.asin(_clamp(-values[6], -1.0, 1.0))
    yaw = math.atan2(values[3], values[0])
    return float(roll), float(pitch), float(yaw)


def _rotation_from_euler(
    values: tuple[float, float, float],
) -> list[float]:
    roll, pitch, yaw = values
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    quaternion = (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm == 0.0:
        return [0.0, 0.0, 1.0, 0.0]
    x, y, z, w = (value / norm for value in quaternion)
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    angle = 2.0 * math.acos(_clamp(w, -1.0, 1.0))
    axis_norm = math.sqrt(max(0.0, 1.0 - w * w))
    if axis_norm < 1e-12:
        return [0.0, 0.0, 1.0, 0.0]
    return [x / axis_norm, y / axis_norm, z / axis_norm, angle]


def _angle_delta(angle: float, reference: float) -> float:
    return (angle - reference + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))


def _require_finite(value: float, name: str) -> None:
    if not math.isfinite(float(value)):
        raise ValueError(f"{name} must be finite")
