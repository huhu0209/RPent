from __future__ import annotations

import math
import sys
import threading
import types
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pytest

from lynsense_webots_sim.action_runtime import ActionRuntime
from lynsense_webots_sim.box_config import load_box_config
from lynsense_webots_sim.geometry import Pose2
from lynsense_webots_sim.manipulation_runtime import (
    ManipulationGoal,
    ManipulationKind,
    ManipulationRuntime,
)
from lynsense_webots_sim.state_machine import MotionProfile
from lynsense_webots_sim.webots_controller import ActionServerBinding
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
ACTION_ORDER = (
    "nav_to_pose",
    "move_distance",
    "move_waist",
    "move_named_config",
    "set_gripper",
    "box_phase",
)
TARGET = CONFIG.named_targets["dualjo:joints_br"]
ARM_POSITIONS = TARGET.left_joints_rad + TARGET.right_joints_rad


@dataclass
class FakeMotor:
    name: str
    positions: list[float]
    velocities: list[float]

    def setPosition(self, position: float) -> None:
        self.positions.append(float(position))

    def setVelocity(self, velocity: float) -> None:
        self.velocities.append(float(velocity))


@dataclass
class FakeSensor:
    name: str
    value: float

    def enable(self, _duration_ms: int) -> None:
        return None

    def getValue(self) -> float:
        return self.value


@dataclass
class FakeField:
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
            return FakeField(self.translation)
        if name == "rotation":
            return FakeField(self.rotation)
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


@dataclass
class FakeSelf:
    position: list[float]
    orientation: list[float]

    def getPosition(self) -> list[float]:
        return self.position

    def getOrientation(self) -> list[float]:
        return self.orientation


class FakeRobot:
    def __init__(self) -> None:
        self.motors = {
            name: FakeMotor(name, [], []) for name in MOTOR_NAMES
        }
        self.sensors = {name: FakeSensor(name, 0.0) for name in SENSOR_NAMES}
        self.box = FakeNode(list(CONFIG.box.initial_pose_m), [0.0, 0.0, 1.0, 0.0])
        self.self = FakeSelf(
            [2.024931, -2.493846, 0.08],
            [
                math.cos(3.124139), -math.sin(3.124139), 0.0,
                math.sin(3.124139), math.cos(3.124139), 0.0,
                0.0, 0.0, 1.0,
            ],
        )
        self.step_results = [0, -1]
        self.step_index = 0
        self.sim_time_s = 10.0
        self.set_upper_state()

    def set_upper_state(self) -> None:
        self.sensors[CONFIG.waist.position_sensor_name].value = -0.278
        for joint, value in zip(
            (*CONFIG.left_arm.joint_names, *CONFIG.right_arm.joint_names),
            ARM_POSITIONS,
        ):
            self.sensors[f"{joint}_sensor"].value = value
        self.sensors[CONFIG.grippers["left"].position_sensor_name].value = 0.0
        self.sensors[CONFIG.grippers["right"].position_sensor_name].value = 0.0

    def getDevice(self, name: str) -> Any:
        if name in self.motors:
            return self.motors[name]
        return self.sensors[name]

    def getFromDef(self, _name: str) -> FakeNode:
        return self.box

    def getSelf(self) -> FakeSelf:
        return self.self

    def getTime(self) -> float:
        return self.sim_time_s

    def step(self, _duration_ms: int) -> int:
        result = self.step_results[min(self.step_index, len(self.step_results) - 1)]
        self.step_index += 1
        if isinstance(result, Exception):
            raise result
        self.sim_time_s += 0.032
        return result


class FakeMessage:
    pass


class FakeRecorder:
    def __init__(self) -> None:
        self.events: list[dict[str, Any]] = []

    def record(self, event: dict[str, Any]) -> None:
        self.events.append(dict(event))

    def close(self) -> None:
        pass


def fake_bindings() -> tuple[ActionServerBinding, ...]:
    def build(kind: str):
        def builder(_node: Any, callbacks: dict[str, Any]) -> Any:
            server = FakeMessage()
            server.kind = kind
            server.callbacks = callbacks
            return server

        return builder

    return tuple(
        ActionServerBinding(kind, action_name, build(kind), FakeMessage, FakeMessage)
        for kind, action_name in zip(
            ("nav", "move", "move_waist", "move_named_config", "set_gripper", "box_phase"),
            ACTION_ORDER,
        )
    )


def controller_factory():
    from lynsense_webots_sim.webots_controller import LynsenseWebotsController

    robot = FakeRobot()
    adapter = WebotsManipulationAdapter(robot, CONFIG)
    runtime_factory = lambda initial_time_s: ManipulationRuntime(
        CONFIG, initial_time_s=initial_time_s
    )
    controller = LynsenseWebotsController(
        robot=robot,
        node=FakeMessage(),
        action_servers=fake_bindings(),
        event_recorder=FakeRecorder(),
        runtime=ActionRuntime(
            MotionProfile(),
            {"搬箱子1": Pose2(1.0, -2.0, 0.0), "放箱子1_1": Pose2(2.0, -2.0, 0.0)},
        ),
        config={
            "goals": {},
            "motion": {},
            "physics": {},
            "world": {"controller_step_ms": 32},
            "timeouts": {},
            "actuators": {
                "left_wheel_motor": {},
                "right_wheel_motor": {},
            },
            "scripts": {},
        },
        manipulation_adapter=adapter,
        manipulation_runtime=runtime_factory(10.0),
        manipulation_runtime_factory=runtime_factory,
    )
    return controller, robot, adapter


def install_goal_response(monkeypatch: pytest.MonkeyPatch) -> type:
    class GoalResponse:
        ACCEPT = object()
        REJECT = object()

    class CancelResponse:
        ACCEPT = GoalResponse.ACCEPT
        REJECT = GoalResponse.REJECT

    rclpy = types.ModuleType("rclpy")
    action = types.ModuleType("rclpy.action")
    action.GoalResponse = GoalResponse
    action.CancelResponse = CancelResponse
    rclpy.action = action
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.action", action)
    return GoalResponse


class GoalHandle:
    def __init__(self) -> None:
        self.feedback: list[Any] = []
        self.succeeded_count = 0
        self.abort_count = 0
        self.canceled_count = 0
        self.is_cancel_requested = False

    def publish_feedback(self, feedback: Any) -> None:
        self.feedback.append(feedback)

    def succeed(self) -> None:
        self.succeeded_count += 1

    def abort(self) -> None:
        self.abort_count += 1

    def canceled(self) -> None:
        self.canceled_count += 1


def start_execute(controller, binding: ActionServerBinding) -> tuple[GoalHandle, threading.Thread]:
    handle = GoalHandle()
    thread = threading.Thread(
        target=lambda: binding.callbacks["execute"](handle), daemon=True
    )
    thread.start()
    for _ in range(100):
        if controller._active_goal is not None:
            break
        threading.Event().wait(0.001)
    return handle, thread


def test_controller_ready_lists_six_actions_and_box_state() -> None:
    controller, _robot, _adapter = controller_factory()
    assert controller._record_controller_ready() is True
    event = controller._event_recorder.events[0]
    assert event["action_names"] == list(ACTION_ORDER)
    assert event["box"]["attached"] is False


def test_navigation_and_manipulation_are_mutually_exclusive(monkeypatch) -> None:
    goal_response = install_goal_response(monkeypatch)
    controller, _robot, _adapter = controller_factory()
    assert controller._record_controller_ready() is True
    bindings = {server.kind: server for server in controller._action_servers}

    request = FakeMessage()
    request.goal_name = "搬箱子1"
    assert bindings["nav"].callbacks["goal"](request) is goal_response.ACCEPT
    waist = FakeMessage()
    waist.height_mm = 200.0
    rejected = bindings["move_waist"].callbacks["goal"](waist)
    assert rejected is goal_response.REJECT
    assert controller._event_recorder.events[-1]["reason"] == "navigation_active"

    controller, _robot, _adapter = controller_factory()
    assert controller._record_controller_ready() is True
    bindings = {server.kind: server for server in controller._action_servers}
    pick = FakeMessage()
    pick.action = "pick"
    pick.flow = "flow"
    pick.config = "box1"
    pick.initok = True
    assert bindings["box_phase"].callbacks["goal"](pick) is goal_response.ACCEPT
    nav = FakeMessage()
    nav.goal_name = "放箱子1_1"
    assert bindings["nav"].callbacks["goal"](nav) is goal_response.REJECT
    assert controller._event_recorder.events[-1]["reason"] == "manipulation_active"


def test_fault_stops_all_motors_and_reports_zeroed_names() -> None:
    controller, _robot, adapter = controller_factory()
    assert controller._record_controller_ready() is True
    assert controller._abort_for_fault("simulation_fault:test") == 1
    assert controller.zero_velocity_motor_names == set(MOTOR_NAMES)
    assert adapter.last_motor_velocities == {
        name: 0.0 for name in MOTOR_NAMES
    }


def test_pick_publishes_feedback_result_and_box_events(monkeypatch) -> None:
    goal_response = install_goal_response(monkeypatch)
    controller, robot, _adapter = controller_factory()
    assert controller._record_controller_ready() is True
    bindings = {server.kind: server for server in controller._action_servers}
    binding = controller._action_servers[5]
    request = FakeMessage()
    request.action = "pick"
    request.flow = "flow"
    request.config = "box1"
    request.initok = True
    assert binding.callbacks["goal"](request) is goal_response.ACCEPT
    callbacks = binding.callbacks
    handle, thread = start_execute(controller, binding)
    controller._start_executor = lambda: None
    robot.step_results = [0, 0, -1]

    assert controller.run() == 0
    thread.join(timeout=1.0)
    assert not thread.is_alive()
    assert handle.succeeded_count == 1
    assert [feedback.phase for feedback in handle.feedback] == ["verifying", "attached"]
    result = controller._active_goal_response
    assert result.success is True
    feedback, terminal = controller._event_recorder.events[-2:]
    assert feedback["kind"] == "action_feedback"
    assert feedback["action"] == "box_phase"
    attached_events = [event for event in controller._event_recorder.events if event["kind"] == "action_feedback"]
    assert [event["box"]["attached"] for event in attached_events] == [False, True]
    assert terminal["kind"] == "action_result"
    assert terminal["box"]["attached"] is True
    assert terminal["success"] is True

    move = FakeMessage()
    move.distance = 0.0
    move.angle = 0.0
    assert bindings["move"].callbacks["goal"](move) is goal_response.ACCEPT


def test_failed_manipulation_retains_lockout_and_cleanup_is_idempotent(monkeypatch) -> None:
    goal_response = install_goal_response(monkeypatch)
    controller, robot, _adapter = controller_factory()
    assert controller._record_controller_ready() is True
    robot.box.translation[0] += 0.1
    binding = controller._action_servers[5]
    request = FakeMessage()
    request.action = "pick"
    request.flow = "flow"
    request.config = "box1"
    request.initok = True
    assert binding.callbacks["goal"](request) is goal_response.ACCEPT
    callbacks = binding.callbacks
    handle, thread = start_execute(controller, binding)
    controller._start_executor = lambda: None
    robot.step_results = [0, -1]
    assert controller.run() == 1
    thread.join(timeout=1.0)
    assert handle.abort_count == 1

    waist = FakeMessage()
    waist.height_mm = 200.0
    rejected = callbacks["goal"](waist)
    assert rejected is goal_response.REJECT
    assert controller._event_recorder.events[-1]["reason"] == "terminal_lockout"

    for _ in range(2):
        controller._safe_stop_motors()
        controller._destroy_action_servers()
        controller._destroy_node()
        controller._shutdown_executor()
    assert controller.cleanup_complete is True
