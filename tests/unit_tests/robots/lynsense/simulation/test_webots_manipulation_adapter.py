from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pytest

from lynsense_webots_sim.box_config import load_box_config
from lynsense_webots_sim.geometry import Pose2
from lynsense_webots_sim.manipulation_runtime import ManipulationCommands
from lynsense_webots_sim.webots_manipulation_adapter import (
    MOTOR_NAMES,
    SENSOR_NAMES,
    WebotsManipulationAdapter,
)


CONFIG = load_box_config(
    Path("robots")
    / "lynsense"
    / "simulation"
    / "ros"
    / "lynsense_webots_sim"
    / "config"
    / "box.yaml"
)
WHEEL_NAMES = ("left_wheel_motor", "right_wheel_motor")
UPPER_SENSOR_NAMES = (
    CONFIG.waist.position_sensor_name,
    *(f"left_joint{i}_sensor" for i in range(1, 7)),
    *(f"right_joint{i}_sensor" for i in range(1, 7)),
    CONFIG.grippers["left"].position_sensor_name,
    CONFIG.grippers["right"].position_sensor_name,
)
ARM_POSITIONS = tuple(float(i + 1) / 20.0 for i in range(12))


@dataclass
class FakeMotor:
    name: str
    calls: dict[str, list[float]]

    def setPosition(self, position: float) -> None:
        self.calls["setPosition"].append(float(position))

    def setVelocity(self, velocity: float) -> None:
        self.calls["setVelocity"].append(float(velocity))


@dataclass
class FakePositionSensor:
    name: str
    value: float = 0.0
    enabled: list[int] | None = None

    def __post_init__(self) -> None:
        if self.enabled is None:
            self.enabled = []

    def enable(self, sampling_period_ms: int) -> None:
        self.enabled.append(sampling_period_ms)

    def getValue(self) -> float:
        return self.value


@dataclass
class FakeField:
    name: str
    value: list[float]

    def setSFVec3f(self, value: list[float]) -> None:
        self.value[:] = [float(item) for item in value]

    def setSFRotation(self, value: list[float]) -> None:
        self.value[:] = [float(item) for item in value]


@dataclass
class FakeNode:
    translation: list[float]
    rotation: list[float]

    def getField(self, name: str) -> FakeField:
        if name == "translation":
            return FakeField(name, self.translation)
        if name == "rotation":
            return FakeField(name, self.rotation)
        raise KeyError(name)

    def getPosition(self) -> list[float]:
        return list(self.translation)

    def getOrientation(self) -> list[float]:
        yaw = self.rotation[3]
        return [
            math.cos(yaw), -math.sin(yaw), 0.0,
            math.sin(yaw), math.cos(yaw), 0.0,
            0.0, 0.0, 1.0,
        ]


class FakeRobot:
    def __init__(self) -> None:
        self.device_counts: dict[str, int] = defaultdict(int)
        self.motors: dict[str, FakeMotor] = {}
        self.sensors: dict[str, FakePositionSensor] = {}
        for name in MOTOR_NAMES:
            motor = FakeMotor(name, defaultdict(list))
            self.motors[name] = motor
        self.sensors = {
            name: FakePositionSensor(name) for name in UPPER_SENSOR_NAMES
        }
        self.node = FakeNode(list(CONFIG.box.initial_pose_m), [0.0, 0.0, 1.0, 0.0])
        self.node_counts: dict[str, int] = defaultdict(int)

    def getDevice(self, name: str) -> Any:
        self.device_counts[name] += 1
        if name in MOTOR_NAMES:
            return self.motors[name]
        sensor = self.sensors.get(name)
        if sensor is None:
            sensor = FakePositionSensor(name)
            self.sensors[name] = sensor
        return sensor

    def getFromDef(self, definition_name: str) -> FakeNode:
        self.node_counts[definition_name] += 1
        return self.node


def set_sensor_values(robot: FakeRobot) -> None:
    values = (-0.478, *ARM_POSITIONS, 0.25, -0.35)
    for name, value in zip(UPPER_SENSOR_NAMES, values):
        robot.sensors[name].value = value


def adapter_and_robot() -> tuple[WebotsManipulationAdapter, FakeRobot]:
    robot = FakeRobot()
    adapter = WebotsManipulationAdapter(robot, CONFIG)
    set_sensor_values(robot)
    return adapter, robot


def test_initialization_acquires_all_motors_and_velocity_mode() -> None:
    adapter, robot = adapter_and_robot()
    assert len(MOTOR_NAMES) == 17
    assert len(SENSOR_NAMES) == 15
    assert set(SENSOR_NAMES) == set(UPPER_SENSOR_NAMES)
    assert set(MOTOR_NAMES) == {*WHEEL_NAMES, CONFIG.waist.motor_name, *(
        f"left_joint{i}_motor" for i in range(1, 7)
    ), *(
        f"right_joint{i}_motor" for i in range(1, 7)
    ), CONFIG.grippers["left"].motor_name, CONFIG.grippers["right"].motor_name}
    assert robot.device_counts == {name: 1 for name in MOTOR_NAMES}
    for motor in robot.motors.values():
        assert motor.calls["setPosition"] == [math.inf]
        assert motor.calls["setVelocity"] == [0.0]


def test_read_maps_sensors_and_box_once() -> None:
    adapter, robot = adapter_and_robot()
    pose = Pose2(1.0, -2.0, 0.25)
    state = adapter.read(pose, 12.5)
    assert state.sim_time_s == 12.5
    assert state.waist_position_m == pytest.approx(-0.478)
    assert state.arm_positions_rad == pytest.approx(ARM_POSITIONS)
    assert state.gripper_positions == pytest.approx((0.25, -0.35))
    assert state.box_position_m == pytest.approx(CONFIG.box.initial_pose_m)
    assert state.box_orientation_rad == (0.0, 0.0, 0.0)
    assert state.robot_pose == pose
    assert state.box_attached is False
    assert robot.device_counts == {
        **{name: 1 for name in MOTOR_NAMES},
        **{name: 1 for name in SENSOR_NAMES},
    }
    assert robot.node_counts == {"BOX_1": 1}
    assert all(sensor.enabled == [32] for sensor in robot.sensors.values())


def test_read_rejects_non_finite_values() -> None:
    adapter, robot = adapter_and_robot()
    adapter.read(Pose2(0.0, 0.0, 0.0), 1.0)
    robot.sensors[CONFIG.waist.position_sensor_name].value = float("nan")
    with pytest.raises(ValueError, match="finite"):
        adapter.read(Pose2(0.0, 0.0, 0.0), 1.0)

    robot.sensors[CONFIG.waist.position_sensor_name].value = -0.478
    with pytest.raises(ValueError, match="finite"):
        adapter.read(Pose2(float("nan"), 0.0, 0.0), 1.0)


def test_write_and_stop_all_map_each_motor_without_cross_writes() -> None:
    adapter, robot = adapter_and_robot()
    commands = ManipulationCommands(
        waist_velocity_mps=0.02,
        arm_velocities_radps=tuple(float(value + 1) for value in ARM_POSITIONS),
        gripper_velocities=(0.2, -0.3),
    )
    adapter.write(commands)
    expected = {
        "left_wheel_motor": 0.0,
        "right_wheel_motor": 0.0,
        "waist_motor": 0.02,
        **{
            f"left_joint{i}_motor": ARM_POSITIONS[i - 1] + 1.0
            for i in range(1, 7)
        },
        **{
            f"right_joint{i}_motor": ARM_POSITIONS[i + 5] + 1.0
            for i in range(1, 7)
        },
        "left_gripper_motor": 0.2,
        "right_gripper_motor": -0.3,
    }
    for name, velocity in expected.items():
        assert robot.motors[name].calls["setVelocity"][-1] == velocity

    adapter.stop_all()
    assert all(
        motor.calls["setVelocity"][-1] == 0.0
        for motor in robot.motors.values()
    )
    assert robot.device_counts == {name: 1 for name in MOTOR_NAMES}


def test_attach_carry_and_release_do_not_teleport_on_release() -> None:
    adapter, robot = adapter_and_robot()
    initial_pose = Pose2(2.0, -2.5, 0.25)
    adapter.read(initial_pose, 10.0)
    adapter.attach()
    moved_pose = Pose2(2.2, -2.5, 0.45)
    carried = adapter.read(moved_pose, 10.1)
    assert carried.box_attached is True
    assert carried.box_orientation_rad[2] == pytest.approx(0.2)
    assert robot.node.translation == pytest.approx(list(carried.box_position_m))
    assert robot.node.rotation[3] == pytest.approx(0.2)

    before_release = list(robot.node.translation)
    adapter.release()
    after_move = adapter.read(Pose2(2.4, -2.5, 0.65), 10.2)
    assert after_move.box_attached is False
    assert after_move.box_position_m == pytest.approx(before_release)
    assert robot.node.translation == pytest.approx(before_release)


def test_attach_normalizes_pi_equivalent_box_yaw() -> None:
    adapter, robot = adapter_and_robot()
    adapter.read(Pose2(2.024931, -2.493846, math.pi), 10.0)
    adapter.attach()
    carried = adapter.read(Pose2(2.024931, -2.493846, 0.0), 10.1)
    assert carried.box_attached is True
    assert carried.box_orientation_rad[2] == pytest.approx(0.0)
    assert robot.node.rotation[3] == pytest.approx(0.0)
