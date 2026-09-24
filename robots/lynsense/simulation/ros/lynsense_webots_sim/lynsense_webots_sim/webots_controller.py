from __future__ import annotations

import math
import os
from collections.abc import Callable, Mapping
from dataclasses import dataclass, replace
from pathlib import Path
import signal
import threading
import time
from typing import Any

import yaml

from lynsense_webots_sim.action_runtime import (
    Acceptance,
    ActionRuntime,
    FaultSignal,
    ExpectedAction,
    Outcome,
    RuntimeObservation,
)
from lynsense_webots_sim.events import EventRecorder
from lynsense_webots_sim.geometry import Pose2, webots_motor_rate
from lynsense_webots_sim.manipulation_runtime import (
    ManipulationCommands,
    ManipulationGoal,
    ManipulationKind,
    ManipulationObservation,
    ManipulationRuntime,
    ManipulationState,
)
from lynsense_webots_sim.state_machine import MotionCommand, MotionProfile
from lynsense_webots_sim.webots_manipulation_adapter import (
    MOTOR_NAMES as ALL_MOTOR_NAMES,
    WebotsManipulationAdapter,
)


MOTOR_NAMES = ("left_wheel_motor", "right_wheel_motor")
CONTROLLER_STEP_MS = 32
ACTION_NAMES = {
    "nav": "nav_to_pose",
    "move": "move_distance",
    "move_waist": "move_waist",
    "move_named_config": "move_named_config",
    "set_gripper": "set_gripper",
    "box_phase": "box_phase",
}
ACTION_ENDPOINTS = {
    "nav": "/lynsense/nav_to_pose",
    "move": "/lynsense/move_distance",
    "move_waist": "/lynsense/move_waist",
    "move_named_config": "/lynsense/move_named_config",
    "set_gripper": "/lynsense/set_gripper",
    "box_phase": "/lynsense/box_phase",
}
ACTION_TYPES = {
    ACTION_NAMES["nav"]: "lynsense_utils/action/NavToPose",
    ACTION_NAMES["move"]: "lynsense_utils/action/MoveDistance",
    "move_waist": "lynsense_utils/action/MoveWaist",
    "move_named_config": "lynsense_utils/action/MoveNamedConfig",
    "set_gripper": "lynsense_utils/action/SetGripper",
    "box_phase": "lynsense_utils/action/BoxPhase",
}
NAVIGATION_KINDS = frozenset(("nav", "move"))
MANIPULATION_KINDS = frozenset(("move_waist", "move_named_config", "set_gripper", "box_phase"))


@dataclass(frozen=True)
class ActionServerBinding:
    kind: str
    action_name: str
    build: Callable[[Any, dict[str, Callable[..., Any]]], Any]
    new_feedback: Callable[[], Any]
    new_result: Callable[[], Any]


@dataclass(frozen=True)
class GoalDescriptor:
    kind: str
    action_name: str
    goal_name: str | None
    distance_m: float | None
    angle_deg: float | None
    height_mm: float | None = None
    target: str | None = None
    gripper_position: float | None = None
    box_action: str | None = None
    box_flow: str | None = None
    box_config: str | None = None


@dataclass(frozen=True)
class GoalTelemetry:
    pose: Pose2
    sim_time_s: float
    left_wheel_rate_radps: float
    right_wheel_rate_radps: float
    waist_position_m: float | None = None
    arm_positions_rad: tuple[float, ...] | None = None
    gripper_positions: tuple[float, float] | None = None
    box_position_m: tuple[float, float, float] | None = None
    box_orientation_rad: tuple[float, float, float] | None = None
    box_attached: bool | None = None
    waist_velocity_mps: float | None = None
    arm_velocities_radps: tuple[float, ...] | None = None
    gripper_velocities: tuple[float, float] | None = None


class LynsenseWebotsController:
    def __init__(
        self,
        robot: Any,
        node: Any,
        action_servers: tuple[ActionServerBinding, ...] | list[ActionServerBinding],
        event_recorder: EventRecorder,
        runtime: ActionRuntime,
        config: Mapping[str, Any],
        *,
        shutdown_requested: threading.Event | None = None,
        owns_rclpy_context: bool = False,
        frame_capture: Any | None = None,
        manipulation_adapter: WebotsManipulationAdapter | None = None,
        manipulation_runtime: ManipulationRuntime | None = None,
        manipulation_runtime_factory: Callable[[float], ManipulationRuntime] | None = None,
    ) -> None:
        self._robot = robot
        self._node = node
        self._action_server_bindings = tuple(action_servers)
        action_names = [binding.action_name for binding in self._action_server_bindings]
        if len(action_names) != len(set(action_names)):
            raise ValueError("action server names must be unique")
        if any(ACTION_NAMES.get(binding.kind) != binding.action_name for binding in self._action_server_bindings):
            raise ValueError("action server kind and name do not match")
        required_actions = {"nav_to_pose", "move_distance"} | (
            {ACTION_NAMES[kind] for kind in MANIPULATION_KINDS}
            if manipulation_adapter is not None
            else set()
        )
        if set(action_names) != required_actions:
            raise ValueError("action servers do not match the controller resource mode")
        self._event_recorder = event_recorder
        self._runtime = runtime
        self._config = config
        self._manipulation_adapter = manipulation_adapter
        self._manipulation_runtime = manipulation_runtime
        self._manipulation_runtime_factory = manipulation_runtime_factory
        if manipulation_adapter is not None and (
            manipulation_runtime is None or manipulation_runtime_factory is None
        ):
            raise ValueError("manipulation adapter requires a runtime and factory")
        if manipulation_adapter is None and (
            manipulation_runtime is not None or manipulation_runtime_factory is not None
        ):
            raise ValueError("manipulation runtime requires an adapter")
        self._owns_rclpy_context = owns_rclpy_context
        self._frame_capture = frame_capture
        self._shutdown_requested = shutdown_requested or threading.Event()
        self._faults = FaultSignal()
        self._lock = threading.Lock()
        self._device_lock = threading.Lock()
        self._executor_stop_requested = threading.Event()
        self._executor: Any = None
        self._executor_thread: threading.Thread | None = None
        self._action_servers: list[Any] = []
        self._motors: dict[str, Any] = {}
        self._reserved_goal: GoalDescriptor | None = None
        self._active_goal: GoalDescriptor | None = None
        self._active_goal_handle: Any | None = None
        self._active_goal_response: Any | None = None
        self._active_goal_outcome: Outcome | None = None
        self._active_goal_done = threading.Event()
        self._latest_telemetry = GoalTelemetry(Pose2(0.0, 0.0, 0.0), 0.0, 0.0, 0.0)
        self._last_terminal_success = True
        self._ready = False
        self.cleanup_complete = False

        configured_motor_names = tuple(self._config["actuators"])
        if configured_motor_names != MOTOR_NAMES:
            raise ValueError("controller requires exactly left and right wheel motors")

    @property
    def zero_velocity_motor_names(self) -> set[str]:
        if self._manipulation_adapter is None:
            return set()
        return {
            name
            for name, velocity in self._manipulation_adapter.last_motor_velocities.items()
            if math.isfinite(velocity) and abs(velocity) == 0.0
        }

    def install_signal_handlers(self) -> None:
        def request_shutdown(signum: int, _frame: Any) -> None:
            self._shutdown_requested.set()

        signal.signal(signal.SIGINT, request_shutdown)
        signal.signal(signal.SIGTERM, request_shutdown)

    def run(self) -> int:
        try:
            if not self._record_controller_ready():
                fault = self._faults.take()
                return self._abort_for_fault(
                    fault or "controller_ready_failed:startup did not complete"
                )

            self._start_executor()
            while self._robot.step(32) != -1:
                fault = self._faults.take()
                if fault is not None:
                    return self._abort_for_fault(fault)

                sim_time_s = self._robot.getTime()
                if self._frame_capture is not None:
                    self._frame_capture.sample(sim_time_s)
                pose = self._read_pose()
                with self._lock:
                    descriptor = self._active_goal
                    navigation_active = (
                        self._runtime.active
                        and descriptor is not None
                        and descriptor.kind in NAVIGATION_KINDS
                    )
                    manipulation_active = (
                        self._manipulation_runtime is not None
                        and self._manipulation_runtime.active
                        and descriptor is not None
                        and descriptor.kind in MANIPULATION_KINDS
                    )
                    observation = None
                    manipulation_observation = None
                    manipulation_state = None
                    idle_command = None
                    idle_fault = ""
                    if navigation_active:
                        if self._manipulation_adapter is not None and self._manipulation_runtime is not None:
                            manipulation_state = self._read_manipulation(
                                pose, sim_time_s
                            )
                            idle_fault = self._manipulation_runtime.attachment_fault(
                                manipulation_state
                            )
                        observation = self._runtime.observe(pose, sim_time_s)
                    elif manipulation_active and self._manipulation_runtime is not None:
                        manipulation_state = self._read_manipulation(
                            pose, sim_time_s
                        )
                        manipulation_observation = self._manipulation_runtime.observe(
                            manipulation_state
                        )
                    else:
                        idle_command = MotionCommand(0.0, 0.0)
                        if self._manipulation_adapter is not None and self._manipulation_runtime is not None:
                            manipulation_state = self._read_manipulation(
                                pose, sim_time_s
                            )
                            idle_fault = self._manipulation_runtime.attachment_fault(
                                manipulation_state
                            )
                    if manipulation_state is not None:
                        self._latest_telemetry = GoalTelemetry(
                            pose,
                            sim_time_s,
                            0.0,
                            0.0,
                            waist_position_m=manipulation_state.waist_position_m,
                            arm_positions_rad=manipulation_state.arm_positions_rad,
                            gripper_positions=manipulation_state.gripper_positions,
                            box_position_m=manipulation_state.box_position_m,
                            box_orientation_rad=manipulation_state.box_orientation_rad,
                            box_attached=manipulation_state.box_attached,
                        )
                    else:
                        self._latest_telemetry = GoalTelemetry(
                            pose,
                            sim_time_s,
                            webots_motor_rate(observation.command.left_wheel_rate_radps)
                            if navigation_active and observation is not None
                            else 0.0,
                            webots_motor_rate(observation.command.right_wheel_rate_radps)
                            if navigation_active and observation is not None
                            else 0.0,
                        )
                fault = self._faults.take()
                if fault is not None or idle_fault:
                    return self._abort_for_fault(fault or f"webots_failed:{idle_fault}")

                if observation is None and manipulation_observation is None:
                    self._write_motors(idle_command)
                    if self._shutdown_requested.is_set():
                        break
                    continue

                if manipulation_observation is not None:
                    self._write_manipulation(manipulation_observation)
                    self._publish_feedback_and_result(manipulation_observation)
                elif observation is not None:
                    self._write_motors(observation.command)
                    self._publish_feedback_and_result(observation)
                if self._shutdown_requested.is_set():
                    break
            return self._shutdown()
        except BaseException as exc:
            fault = self._faults.take()
            reason = fault or f"webots_failed:{_fault_detail(exc)}"
            return self._abort_for_fault(reason)
        finally:
            self._safe_stop_motors()
            self._destroy_action_servers()
            self._destroy_node()
            self._shutdown_executor()
            self._shutdown_rclpy()
            self.cleanup_complete = True

    def _record_controller_ready(self) -> bool:
        try:
            self._action_servers = [
                binding.build(
                    self._node,
                    {
                        "goal": self._goal_callback(binding.kind),
                        "execute": self._execute_callback(
                            binding.kind, binding.new_result
                        ),
                        "cancel": self._cancel_callback,
                    },
                )
                for binding in self._action_server_bindings
            ]
            sim_time_s = float(self._robot.getTime())
            pose = self._read_pose()
            manipulation_state = (
                self._read_manipulation(pose, sim_time_s)
                if self._manipulation_adapter is not None
                else None
            )
            self._write_motors(MotionCommand.stopped())
            if manipulation_state is None:
                self._latest_telemetry = GoalTelemetry(pose, sim_time_s, 0.0, 0.0)
            else:
                self._latest_telemetry = GoalTelemetry(
                    pose,
                    sim_time_s,
                    0.0,
                    0.0,
                    waist_position_m=manipulation_state.waist_position_m,
                    arm_positions_rad=manipulation_state.arm_positions_rad,
                    gripper_positions=manipulation_state.gripper_positions,
                    box_position_m=manipulation_state.box_position_m,
                    box_orientation_rad=manipulation_state.box_orientation_rad,
                    box_attached=manipulation_state.box_attached,
                )
            ready_event = {
                "kind": "controller_ready",
                "action": "controller",
                "sim_time_s": sim_time_s,
                "action_names": [binding.action_name for binding in self._action_server_bindings],
                "action_types": {
                    name: action_type
                    for name, action_type in ACTION_TYPES.items()
                    if name in {binding.action_name for binding in self._action_server_bindings}
                },
                "motor_names": list(
                    ALL_MOTOR_NAMES
                    if self._manipulation_adapter is not None
                    else MOTOR_NAMES
                ),
                "pose": _pose_fields(pose),
                "motor_rates": {
                    "left_wheel_rate_radps": 0.0,
                    "right_wheel_rate_radps": 0.0,
                },
                "phase": "ready",
                "outcome": "ready",
                "reason": "",
            }
            if manipulation_state is not None:
                ready_event.update(_manipulation_fields(manipulation_state))
            if not self._record_event(ready_event):
                return False
            self._ready = True
            return True
        except BaseException as exc:
            self._faults.request(f"controller_ready_failed:{_fault_detail(exc)}")
            return False

    def _start_executor(self) -> None:
        from rclpy.executors import MultiThreadedExecutor

        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(
            target=self._executor_loop,
            name="lynsense-webots-controller-ros",
            daemon=True,
        )
        self._executor_thread.start()

    def _executor_loop(self) -> None:
        try:
            while not self._executor_stop_requested.is_set():
                self._executor.spin_once(timeout_sec=0.01)
        except BaseException as exc:
            self._faults.request(f"executor_failed:{_fault_detail(exc)}")

    def _goal_callback(self, kind: str) -> Callable[[Any], Any]:
        def accept_goal(request: Any) -> Any:
            from rclpy.action import GoalResponse

            descriptor = _goal_descriptor(kind, request)
            manipulation_state = None
            if descriptor.kind in MANIPULATION_KINDS:
                if self._manipulation_adapter is None or self._manipulation_runtime is None:
                    acceptance = Acceptance(False, "manipulation_unavailable")
                    telemetry = self._latest_telemetry
                    event = _goal_event(
                        "goal_rejected",
                        descriptor,
                        telemetry,
                        "idle",
                        "rejected",
                        acceptance.reason,
                    )
                    self._record_event(event)
                    return GoalResponse.REJECT
                pose = self._read_pose()
                manipulation_state = self._read_manipulation(
                    pose, float(self._robot.getTime())
                )
            with self._lock:
                navigation_owned = (
                    self._runtime.active
                    or any(
                        goal is not None and goal.kind in NAVIGATION_KINDS
                        for goal in (self._reserved_goal, self._active_goal)
                    )
                )
                manipulation_owned = (
                    self._manipulation_runtime is not None
                    and (
                        self._manipulation_runtime.active
                        or any(
                            goal is not None and goal.kind in MANIPULATION_KINDS
                            for goal in (self._reserved_goal, self._active_goal)
                        )
                    )
                )
                if descriptor.kind in MANIPULATION_KINDS and navigation_owned:
                    acceptance = Acceptance(False, "navigation_active")
                elif descriptor.kind in NAVIGATION_KINDS and manipulation_owned:
                    acceptance = Acceptance(False, "manipulation_active")
                elif descriptor.kind in MANIPULATION_KINDS:
                    acceptance = self._manipulation_runtime.submit(
                        _manipulation_goal(descriptor), manipulation_state
                    )
                elif kind == "nav":
                    acceptance = self._runtime.submit_nav(request.goal_name)
                else:
                    acceptance = self._runtime.submit_move(
                        request.distance, request.angle
                    )
                accepted = acceptance.accepted
                if accepted:
                    self._reserved_goal = descriptor
                if manipulation_state is not None:
                    telemetry = GoalTelemetry(
                        manipulation_state.robot_pose,
                        manipulation_state.sim_time_s,
                        self._latest_telemetry.left_wheel_rate_radps,
                        self._latest_telemetry.right_wheel_rate_radps,
                        waist_position_m=manipulation_state.waist_position_m,
                        arm_positions_rad=manipulation_state.arm_positions_rad,
                        gripper_positions=manipulation_state.gripper_positions,
                        box_position_m=manipulation_state.box_position_m,
                        box_orientation_rad=manipulation_state.box_orientation_rad,
                        box_attached=manipulation_state.box_attached,
                    )
                else:
                    telemetry = self._latest_telemetry

            event = _goal_event(
                "goal_accepted" if accepted else "goal_rejected",
                descriptor,
                telemetry,
                "pending" if accepted else "idle",
                "accepted" if accepted else "rejected",
                "" if accepted and acceptance.reason == "accepted" else acceptance.reason,
            )
            if telemetry.box_position_m is not None:
                event.update(_manipulation_fields(_telemetry_state(telemetry)))
            self._record_event(event)
            return GoalResponse.ACCEPT if accepted else GoalResponse.REJECT

        return accept_goal

    def _execute_callback(
        self, kind: str, new_result: Callable[[], Any]
    ) -> Callable[[Any], Any]:
        def execute(goal_handle: Any) -> Any:
            with self._lock:
                descriptor = self._reserved_goal
                if (
                    descriptor is None
                    or self._shutdown_requested.is_set()
                    or self._executor_stop_requested.is_set()
                ):
                    return new_result()
                self._reserved_goal = None
                self._active_goal = descriptor
                self._active_goal_handle = goal_handle
                self._active_goal_response = None
                self._active_goal_outcome = None
                self._active_goal_done.clear()
            self._active_goal_done.wait()
            with self._lock:
                response = self._active_goal_response
                outcome = self._active_goal_outcome
            if goal_handle is not None and outcome is not None:
                if outcome is Outcome.SUCCEEDED:
                    goal_handle.succeed()
                elif outcome is Outcome.CANCELED:
                    for _ in range(500):
                        if goal_handle.is_cancel_requested:
                            break
                        time.sleep(0.01)
                    else:
                        goal_handle.abort()
                        return new_result()
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
            return response if response is not None else new_result()

        return execute

    def _cancel_callback(self, _goal_handle: Any) -> Any:
        from rclpy.action import CancelResponse

        with self._lock:
            descriptor = self._reserved_goal or self._active_goal
            if (
                descriptor is not None
                and descriptor.kind in MANIPULATION_KINDS
                and self._manipulation_runtime is not None
            ):
                canceled = self._manipulation_runtime.request_cancel()
            else:
                canceled = self._runtime.request_cancel()
        return CancelResponse.ACCEPT if canceled else CancelResponse.REJECT

    def _read_manipulation(self, pose: Pose2, sim_time_s: float) -> ManipulationState:
        if self._manipulation_adapter is None:
            raise RuntimeError("manipulation adapter is unavailable")
        with self._device_lock:
            return self._manipulation_adapter.read(pose, sim_time_s)

    def _read_pose(self) -> Pose2:
        pose_object = self._robot.getSelf()
        position = pose_object.getPosition()
        orientation = pose_object.getOrientation()
        if len(position) != 3 or len(orientation) != 9:
            raise ValueError("Supervisor returned invalid pose dimensions")
        values = (
            position[0],
            position[1],
            math.atan2(orientation[3], orientation[0]),
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("Supervisor pose values must be finite")
        return Pose2(*(float(value) for value in values))

    def _write_motors(self, command: MotionCommand) -> None:
        rates = (
            webots_motor_rate(command.left_wheel_rate_radps),
            webots_motor_rate(command.right_wheel_rate_radps),
        )
        if self._manipulation_adapter is not None:
            self._write_manipulation_commands(ManipulationCommands())
        for name, rate in zip(MOTOR_NAMES, rates):
            motor = self._motor(name)
            motor.setVelocity(rate)
        self._latest_telemetry = _replace_telemetry_rates(
            self._latest_telemetry, rates[0], rates[1]
        )

    def _write_manipulation(self, observation: ManipulationObservation) -> None:
        if self._manipulation_adapter is None:
            raise RuntimeError("manipulation adapter is unavailable")
        commands = self._manipulation_runtime.commands()
        self._write_manipulation_commands(commands)
        rates = self._manipulation_adapter.last_motor_velocities
        self._latest_telemetry = _replace_telemetry_rates(
            self._latest_telemetry,
            rates["left_wheel_motor"],
            rates["right_wheel_motor"],
            waist_velocity_mps=rates["waist_motor"],
            arm_velocities_radps=tuple(
                rates[f"{side}_joint{index}_motor"]
                for side in ("left", "right")
                for index in range(1, 7)
            ),
            gripper_velocities=(
                rates["left_gripper_motor"],
                rates["right_gripper_motor"],
            ),
        )

    def _write_manipulation_commands(self, commands: ManipulationCommands) -> None:
        if self._manipulation_adapter is None:
            raise RuntimeError("manipulation adapter is unavailable")
        with self._device_lock:
            self._manipulation_adapter.write(commands)
            if commands.attach_box:
                self._manipulation_adapter.attach()
            if commands.release_box:
                self._manipulation_adapter.release()

    def _motor(self, name: str) -> Any:
        if name not in self._motors:
            motor = self._robot.getDevice(name)
            motor.setPosition(float("inf"))
            self._motors[name] = motor
        return self._motors[name]

    def _publish_feedback_and_result(
        self, observation: RuntimeObservation | ManipulationObservation
    ) -> None:
        with self._lock:
            descriptor = self._active_goal
            goal_handle = self._active_goal_handle
        if descriptor is None:
            return
        binding = self._binding_for_action(descriptor.action_name)
        telemetry = self._latest_telemetry

        feedback = binding.new_feedback()
        if descriptor.kind in MANIPULATION_KINDS:
            _apply_manipulation_feedback(descriptor.kind, feedback, observation)
        else:
            navigation_observation = observation
            feedback.distance_remaining = navigation_observation.feedback.distance_remaining_m
            if descriptor.kind == "move":
                feedback.angle_remaining = navigation_observation.feedback.angle_remaining_deg
        if goal_handle is not None:
            goal_handle.publish_feedback(feedback)
        feedback_event = _action_event(
                "action_feedback", descriptor, observation, telemetry,
            )
        if not self._record_event(feedback_event):
            self._abort_for_fault("event_recorder_failed:feedback write failed")
            return

        if observation.terminal:
            result = binding.new_result()
            success, message = _terminal_fields(observation)
            result.success = success
            result.message = message

            result_event = _action_event(
                "action_result", descriptor, observation, telemetry
            )
            result_event["success"] = success
            if not self._record_event(result_event):
                self._abort_for_fault("event_recorder_failed:result write failed")
                return

            with self._lock:
                self._active_goal_response = result
            self._last_terminal_success = success
            self._finish_active_goal(observation.outcome, success)

    def _finish_active_goal(
        self, outcome: Outcome, success: bool | None = None
    ) -> None:
        with self._lock:
            done = self._active_goal_done
            self._active_goal_outcome = outcome
            self._active_goal = None
            self._active_goal_handle = None
            if success is True and self._manipulation_runtime is not None:
                self._runtime.reset()
                self._manipulation_runtime.reset(
                    self._latest_telemetry.sim_time_s
                )
        done.set()

    def _abort_for_fault(self, reason: str) -> int:
        self._safe_stop_motors()
        with self._lock:
            descriptor = self._active_goal
        if descriptor is not None:
            self._finish_active_goal(Outcome.FAILED)
        self._last_terminal_success = False

        fault_event = {
            "kind": "controller_failed",
            "action": descriptor.action_name if descriptor else "controller",
            "sim_time_s": self._latest_telemetry.sim_time_s,
            "pose": _pose_fields(self._latest_telemetry.pose),
            "motor_rates": {
                "left_wheel_rate_radps": self._latest_telemetry.left_wheel_rate_radps,
                "right_wheel_rate_radps": self._latest_telemetry.right_wheel_rate_radps,
            },
            "phase": "fault",
            "outcome": "failed",
            "reason": reason,
        }
        if self._latest_telemetry.box_position_m is not None:
            fault_event.update(_manipulation_fields(_telemetry_state(self._latest_telemetry)))
        self._record_event(fault_event)
        self._shutdown_requested.set()
        self._executor_stop_requested.set()
        return 1

    def _shutdown(self) -> int:
        self._safe_stop_motors()
        fault = self._faults.take()
        if fault is not None:
            return self._abort_for_fault(fault)

        with self._lock:
            navigation_active = self._runtime.active
            manipulation_active = (
                self._manipulation_runtime is not None
                and self._manipulation_runtime.active
            )
            descriptor = self._active_goal or self._reserved_goal
            if (navigation_active or manipulation_active) and descriptor is None:
                self._reserved_goal = None
                descriptor = None
            else:
                self._reserved_goal = None
            terminal_result = (
                None if navigation_active else self._runtime.terminal_result
            )

        if navigation_active or manipulation_active:
            terminated_event = {
                "kind": "webots_terminated",
                "action": descriptor.action_name if descriptor else "controller",
                "sim_time_s": self._latest_telemetry.sim_time_s,
                "pose": _pose_fields(self._latest_telemetry.pose),
                "motor_rates": {
                    "left_wheel_rate_radps": self._latest_telemetry.left_wheel_rate_radps,
                    "right_wheel_rate_radps": self._latest_telemetry.right_wheel_rate_radps,
                },
                "phase": "terminated",
                "outcome": "failed",
                "reason": "webots_terminated_with_active_goal",
            }
            if self._latest_telemetry.box_position_m is not None:
                terminated_event.update(
                    _manipulation_fields(_telemetry_state(self._latest_telemetry))
                )
            self._record_event(terminated_event)
            self._finish_active_goal(Outcome.FAILED)
            self._last_terminal_success = False
            return 1

        terminal_failed = (
            terminal_result is not None and not terminal_result.success
        )
        return 1 if terminal_failed or not self._last_terminal_success else 0

    def _record_event(self, event: dict[str, Any]) -> bool:
        try:
            self._event_recorder.record(event)
            return True
        except BaseException as exc:
            self._faults.request(f"event_recorder_failed:{_fault_detail(exc)}")
            return False

    def _safe_stop_motors(self) -> None:
        self._executor_stop_requested.set()
        if self._manipulation_adapter is not None:
            try:
                with self._device_lock:
                    self._manipulation_adapter.stop_all()
            except BaseException as exc:
                self._faults.request(f"webots_failed:{_fault_detail(exc)}")
        self._latest_telemetry = _replace_telemetry_rates(
            self._latest_telemetry,
            0.0,
            0.0,
            waist_velocity_mps=0.0 if self._manipulation_adapter is not None else None,
            arm_velocities_radps=(0.0,) * 12 if self._manipulation_adapter is not None else None,
            gripper_velocities=(0.0, 0.0) if self._manipulation_adapter is not None else None,
        )
        for name in MOTOR_NAMES:
            try:
                self._motor(name).setVelocity(0.0)
            except BaseException as exc:
                self._faults.request(f"webots_failed:{_fault_detail(exc)}")

    def _destroy_action_servers(self) -> None:
        for action_server in self._action_servers:
            try:
                action_server.destroy()
            except BaseException:
                pass
        self._action_servers.clear()

    def _destroy_node(self) -> None:
        try:
            self._node.destroy_node()
        except BaseException:
            pass

    def _shutdown_executor(self) -> None:
        self._executor_stop_requested.set()
        if self._executor is not None:
            try:
                self._executor.shutdown()
            except BaseException:
                pass
        if self._executor_thread is not None:
            self._executor_thread.join(timeout=1.0)

    def _shutdown_rclpy(self) -> None:
        if not self._owns_rclpy_context:
            return
        try:
            import rclpy

            rclpy.shutdown()
        except BaseException:
            pass

    def _binding_for_action(self, action_name: str) -> ActionServerBinding:
        return next(
            binding
            for binding in self._action_server_bindings
            if binding.action_name == action_name
        )


def _pose_fields(pose: Pose2) -> dict[str, float]:
    return {"x": pose.x, "y": pose.y, "yaw": pose.yaw}


def _goal_descriptor(kind: str, request: Any) -> GoalDescriptor:
    if kind == "nav":
        goal_name = getattr(request, "goal_name", None)
        return GoalDescriptor("nav", ACTION_NAMES["nav"], goal_name, None, None)
    if kind == "move_waist":
        return GoalDescriptor(
            kind,
            ACTION_NAMES[kind],
            None,
            None,
            None,
            height_mm=_finite_optional_float(getattr(request, "height_mm", None)),
        )
    if kind == "move_named_config":
        return GoalDescriptor(
            kind,
            ACTION_NAMES[kind],
            None,
            None,
            None,
            target=_optional_string(getattr(request, "target", None)),
        )
    if kind == "set_gripper":
        return GoalDescriptor(
            kind,
            ACTION_NAMES[kind],
            None,
            None,
            None,
            gripper_position=_finite_optional_float(
                getattr(request, "position", None)
            ),
        )
    if kind == "box_phase":
        return GoalDescriptor(
            kind,
            ACTION_NAMES[kind],
            None,
            None,
            None,
            box_action=_optional_string(getattr(request, "action", None)),
            box_flow=_optional_string(getattr(request, "flow", None)),
            box_config=_optional_string(getattr(request, "config", None)),
        )
    distance = _finite_optional_float(getattr(request, "distance", None))
    angle = _finite_optional_float(getattr(request, "angle", None))
    return GoalDescriptor("move", ACTION_NAMES["move"], None, distance, angle)


def _optional_string(value: Any) -> str | None:
    return value if isinstance(value, str) else None


def _manipulation_goal(descriptor: GoalDescriptor) -> ManipulationGoal:
    if descriptor.kind == "move_waist":
        return ManipulationGoal(
            ManipulationKind.WAIST, height_mm=descriptor.height_mm
        )
    if descriptor.kind == "move_named_config":
        return ManipulationGoal(
            ManipulationKind.NAMED_CONFIG, target=descriptor.target
        )
    if descriptor.kind == "set_gripper":
        return ManipulationGoal(
            ManipulationKind.GRIPPER, gripper_position=descriptor.gripper_position
        )
    return ManipulationGoal(
        ManipulationKind.BOX_PHASE,
        box_action=descriptor.box_action,
        flow=descriptor.box_flow,
        config_name=descriptor.box_config,
    )


def _finite_optional_float(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    converted = float(value)
    if not math.isfinite(converted):
        return None
    return converted
    return float(value)


def _goal_event(
    kind: str,
    descriptor: GoalDescriptor,
    telemetry: GoalTelemetry,
    phase: str,
    outcome: str,
    reason: str,
) -> dict[str, Any]:
    return {
        "kind": kind,
        "action": descriptor.action_name,
        "sim_time_s": telemetry.sim_time_s,
        "goal_name": descriptor.goal_name,
        "distance_m": descriptor.distance_m,
        "angle_deg": descriptor.angle_deg,
        "height_mm": descriptor.height_mm,
        "target": descriptor.target,
        "gripper_position": descriptor.gripper_position,
        "box_action": descriptor.box_action,
        "box_flow": descriptor.box_flow,
        "box_config": descriptor.box_config,
        "phase": phase,
        "outcome": outcome,
        "reason": reason,
        "pose": _pose_fields(telemetry.pose),
        "motor_rates": {
            "left_wheel_rate_radps": telemetry.left_wheel_rate_radps,
            "right_wheel_rate_radps": telemetry.right_wheel_rate_radps,
        },
    }
    if telemetry.box_position_m is not None:
        event.update(_manipulation_fields(_telemetry_state(telemetry)))
    return event


def _action_event(
    kind: str,
    descriptor: GoalDescriptor,
    observation: RuntimeObservation | ManipulationObservation,
    telemetry: GoalTelemetry,
) -> dict[str, Any]:
    event = {
        "kind": kind,
        "action": descriptor.action_name,
        "sim_time_s": telemetry.sim_time_s,
        "goal_name": descriptor.goal_name,
        "distance_m": descriptor.distance_m,
        "angle_deg": descriptor.angle_deg,
        "height_mm": descriptor.height_mm,
        "target": descriptor.target,
        "gripper_position": descriptor.gripper_position,
        "box_action": descriptor.box_action,
        "box_flow": descriptor.box_flow,
        "box_config": descriptor.box_config,
        "reason": observation.reason,
        "pose": _pose_fields(telemetry.pose),
        "motor_rates": {
            "left_wheel_rate_radps": telemetry.left_wheel_rate_radps,
            "right_wheel_rate_radps": telemetry.right_wheel_rate_radps,
        },
    }
    if descriptor.kind in MANIPULATION_KINDS:
        manipulation = observation
        event.update(
            {
                "phase": manipulation.phase,
                "outcome": manipulation.outcome.value,
                "progress": manipulation.progress,
                "max_joint_error_rad": manipulation.max_joint_error_rad,
                "box_error_m": manipulation.box_error_m,
                "position_mm": manipulation.position_mm,
                "remaining_mm": manipulation.remaining_mm,
                "gripper_position": manipulation.gripper_position,
                "gripper_remaining": manipulation.gripper_remaining,
            }
        )
    else:
        navigation = observation
        event.update(
            {
                "distance_remaining_m": navigation.feedback.distance_remaining_m,
                "angle_remaining_deg": navigation.feedback.angle_remaining_deg,
                "phase": navigation.phase.value,
                "outcome": navigation.outcome.value,
            }
        )
    if telemetry.box_position_m is not None:
        event.update(_manipulation_fields(_telemetry_state(telemetry)))
    if telemetry.waist_velocity_mps is not None:
        event["manipulation_rates"] = {
            "waist_velocity_mps": telemetry.waist_velocity_mps,
            "arm_velocities_radps": telemetry.arm_velocities_radps,
            "gripper_velocities": telemetry.gripper_velocities,
        }
    return event


def _apply_manipulation_feedback(
    kind: str, feedback: Any, observation: ManipulationObservation
) -> None:
    feedback.phase = observation.phase
    if kind == "move_waist":
        feedback.position_mm = observation.position_mm
        feedback.remaining_mm = observation.remaining_mm
    elif kind == "move_named_config":
        feedback.progress = observation.progress
        feedback.max_joint_error_rad = observation.max_joint_error_rad
    elif kind == "set_gripper":
        feedback.position = observation.gripper_position
        feedback.remaining = observation.gripper_remaining
    else:
        feedback.progress = observation.progress
        feedback.box_error_m = observation.box_error_m


def _terminal_fields(
    observation: RuntimeObservation | ManipulationObservation,
) -> tuple[bool, str]:
    if isinstance(observation, ManipulationObservation):
        return observation.success, observation.message or observation.reason
    if observation.terminal_result is None:
        return False, observation.reason
    return observation.terminal_result.success, observation.terminal_result.message


def _telemetry_state(telemetry: GoalTelemetry) -> ManipulationState:
    return ManipulationState(
        sim_time_s=telemetry.sim_time_s,
        waist_position_m=telemetry.waist_position_m or 0.0,
        arm_positions_rad=telemetry.arm_positions_rad or (0.0,) * 12,
        gripper_positions=telemetry.gripper_positions or (0.0, 0.0),
        box_position_m=telemetry.box_position_m or (0.0, 0.0, 0.0),
        box_orientation_rad=telemetry.box_orientation_rad or (0.0, 0.0, 0.0),
        robot_pose=telemetry.pose,
        box_attached=telemetry.box_attached is True,
    )


def _manipulation_fields(state: ManipulationState) -> dict[str, Any]:
    return {
        "waist_position_m": state.waist_position_m,
        "arm_positions_rad": state.arm_positions_rad,
        "gripper_positions": state.gripper_positions,
        "box": {
            "position_m": state.box_position_m,
            "orientation_rad": state.box_orientation_rad,
            "attached": state.box_attached,
        },
    }


def _replace_telemetry_rates(
    telemetry: GoalTelemetry,
    left_rate: float,
    right_rate: float,
    *,
    waist_velocity_mps: float | None = None,
    arm_velocities_radps: tuple[float, ...] | None = None,
    gripper_velocities: tuple[float, float] | None = None,
) -> GoalTelemetry:
    return replace(
        telemetry,
        left_wheel_rate_radps=left_rate,
        right_wheel_rate_radps=right_rate,
        waist_velocity_mps=(
            waist_velocity_mps
            if waist_velocity_mps is not None
            else telemetry.waist_velocity_mps
        ),
        arm_velocities_radps=(
            arm_velocities_radps
            if arm_velocities_radps is not None
            else telemetry.arm_velocities_radps
        ),
        gripper_velocities=(
            gripper_velocities
            if gripper_velocities is not None
            else telemetry.gripper_velocities
        ),
    )


def _fault_detail(exc: BaseException) -> str:
    detail = f"{type(exc).__name__}: {exc}"
    return detail.replace("\n", " ").replace("\r", " ")


def main() -> int:
    import rclpy
    from controller import Supervisor

    from lynsense_utils.action import (
        BoxPhase,
        MoveDistance,
        MoveNamedConfig,
        MoveWaist,
        NavToPose,
        SetGripper,
    )

    event_recorder: EventRecorder | None = None
    node: Any | None = None
    controller: LynsenseWebotsController | None = None
    owns_rclpy_context = False

    try:
        config_path = Path(os.environ["LYNSENSE_SIM_CONFIG"])
        event_path = Path(os.environ["LYNSENSE_SIM_EVENT_FILE"])
        script_name = os.environ["LYNSENSE_SIM_SCRIPT"]
        run_id = os.environ["LYNSENSE_SIM_RUN_ID"]
        if script_name not in {"interface", "smoke", "blocked", "match", "box"}:
            raise ValueError(
                "LYNSENSE_SIM_SCRIPT must be interface, smoke, blocked, match, or box"
            )

        navigation_config_path = (
            Path(
                os.environ.get(
                    "LYNSENSE_SIM_NAV_CONFIG",
                    str(config_path.parent / "smoke.yaml"),
                )
            )
            if script_name == "box"
            else config_path
        )
        config = yaml.safe_load(navigation_config_path.read_text(encoding="utf-8"))
        profile = MotionProfile(**config["motion"])
        goals = {
            name: Pose2(
                float(values["x"]),
                float(values["y"]),
                math.radians(float(values["yaw_deg"])),
            )
            for name, values in config["goals"].items()
        }
        script = (
            None
            if script_name in {"interface", "box"}
            else config["scripts"][script_name]
        )
        allowed_actions = tuple(
            ExpectedAction.nav(entry["goal_name"])
            if entry["action"] == "nav_to_pose"
            else ExpectedAction.move(float(entry["distance"]), float(entry["angle"]))
            for entry in script
        ) if script is not None else None
        runtime = ActionRuntime(profile, goals, allowed_actions)

        event_recorder = EventRecorder(event_path, run_id)
        event_recorder.record(
            {
                "kind": "run_started",
                "action": "controller",
                "sim_time_s": 0.0,
            }
        )

        def build_action_server(
            action_type: Any, endpoint: str
        ) -> Callable[[Any, dict[str, Callable[..., Any]]], Any]:
            def builder(node: Any, callbacks: dict[str, Callable[..., Any]]) -> Any:
                from rclpy.action import ActionServer

                return ActionServer(
                    node,
                    action_type,
                    endpoint,
                    goal_callback=callbacks["goal"],
                    execute_callback=callbacks["execute"],
                    cancel_callback=callbacks["cancel"],
                    callback_group=callback_group,
                )

            return builder

        def binding(
            kind: str,
            action_type: Any,
        ) -> ActionServerBinding:
            return ActionServerBinding(
                kind,
                ACTION_NAMES[kind],
                build_action_server(action_type, ACTION_ENDPOINTS[kind]),
                action_type.Feedback,
                action_type.Result,
            )

        action_servers = (
            binding("nav", NavToPose),
            binding("move", MoveDistance),
        )
        if script_name == "box":
            action_servers = action_servers + (
                binding("move_waist", MoveWaist),
                binding("move_named_config", MoveNamedConfig),
                binding("set_gripper", SetGripper),
                binding("box_phase", BoxPhase),
            )

        rclpy.init()
        owns_rclpy_context = True
        node = rclpy.create_node("lynsense_webots_controller")
        from rclpy.callback_groups import ReentrantCallbackGroup

        callback_group = ReentrantCallbackGroup()
        robot = Supervisor()
        manipulation_adapter = None
        manipulation_runtime = None
        manipulation_runtime_factory = None
        if script_name == "box":
            from lynsense_webots_sim.box_config import load_box_config

            box_config = load_box_config(
                Path(
                    os.environ.get(
                        "LYNSENSE_SIM_BOX_CONFIG",
                        str(config_path.parent / "box.yaml"),
                    )
                )
            )
            manipulation_adapter = WebotsManipulationAdapter(robot, box_config)
            initial_time_s = float(robot.getTime())

            def runtime_factory(initial_time: float) -> ManipulationRuntime:
                return ManipulationRuntime(
                    box_config, initial_time_s=initial_time
                )

            manipulation_runtime = runtime_factory(initial_time_s)
            manipulation_runtime_factory = runtime_factory

        from lynsense_webots_sim.viewer_capture import capture_from_environment

        controller = LynsenseWebotsController(
            robot,
            node,
            action_servers,
            event_recorder,
            runtime,
            config,
            owns_rclpy_context=True,
            frame_capture=capture_from_environment(robot),
            manipulation_adapter=manipulation_adapter,
            manipulation_runtime=manipulation_runtime,
            manipulation_runtime_factory=manipulation_runtime_factory,
        )
        controller.install_signal_handlers()
        return controller.run()
    except BaseException as exc:
        if controller is not None and not controller.cleanup_complete:
            controller._abort_for_fault(f"startup_failed:{_fault_detail(exc)}")
        if node is not None and (controller is None or not controller.cleanup_complete):
            try:
                node.destroy_node()
            except BaseException:
                pass
        if owns_rclpy_context and (controller is None or not controller.cleanup_complete):
            try:
                rclpy.shutdown()
            except BaseException:
                pass
        return 1
    finally:
        if event_recorder is not None:
            try:
                event_recorder.close()
            except BaseException:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
