from __future__ import annotations

import enum
import math
import threading
from dataclasses import dataclass, replace
from typing import Any

from lynsense_webots_sim.action_runtime import Acceptance, Outcome
from lynsense_webots_sim.box_config import BoxConfig
from lynsense_webots_sim.geometry import Pose2, pi_symmetric_angular_error


class ManipulationKind(enum.Enum):
    WAIST = "waist"
    NAMED_CONFIG = "named_config"
    GRIPPER = "gripper"
    BOX_PHASE = "box_phase"


@dataclass(frozen=True)
class ManipulationGoal:
    kind: ManipulationKind
    height_mm: float | None = None
    target: str | None = None
    gripper_position: float | None = None
    box_action: str | None = None
    flow: str | None = None
    config_name: str | None = None


@dataclass(frozen=True)
class ManipulationState:
    sim_time_s: float
    waist_position_m: float
    arm_positions_rad: tuple[float, ...]
    gripper_positions: tuple[float, float]
    box_position_m: tuple[float, float, float]
    box_orientation_rad: tuple[float, float, float]
    robot_pose: Pose2
    box_attached: bool


@dataclass(frozen=True)
class ManipulationCommands:
    waist_velocity_mps: float = 0.0
    arm_velocities_radps: tuple[float, ...] = (0.0,) * 12
    gripper_velocities: tuple[float, float] = (0.0, 0.0)
    attach_box: bool = False
    release_box: bool = False


@dataclass(frozen=True)
class ManipulationObservation:
    phase: str
    outcome: Outcome
    reason: str
    terminal: bool
    success: bool
    message: str
    feedback_kind: str
    position_mm: float = 0.0
    remaining_mm: float = 0.0
    progress: float = 0.0
    max_joint_error_rad: float = 0.0
    gripper_position: float = 0.0
    gripper_remaining: float = 0.0
    box_error_m: float = 0.0


@dataclass(frozen=True)
class _AttachmentOffset:
    x_m: float
    y_m: float
    yaw_rad: float


class ManipulationRuntime:
    def __init__(self, config: BoxConfig, *, initial_time_s: float) -> None:
        if not math.isfinite(initial_time_s):
            raise ValueError("initial_time_s must be finite")
        self._config = config
        self._initial_time_s = initial_time_s
        self._goal: ManipulationGoal | None = None
        self._phase = "idle"
        self._started_s = initial_time_s
        self._terminal: ManipulationObservation | None = None
        self._cancel_requested = False
        self._commands = ManipulationCommands()
        self._stall_error = math.inf
        self._stall_time_s = initial_time_s
        self._release_started_s: float | None = None
        self._attachment_offset: _AttachmentOffset | None = None
        self._lock = threading.Lock()

    @property
    def active(self) -> bool:
        with self._lock:
            return self._goal is not None

    def submit(
        self, goal: ManipulationGoal, state: ManipulationState
    ) -> Acceptance:
        self._validate_state(state)
        with self._lock:
            if self._terminal is not None:
                return Acceptance(False, "terminal_lockout")
            if self._goal is not None:
                return Acceptance(False, "active_manipulation_goal")
            reason = self._validate_goal(goal)
            if reason:
                return Acceptance(False, reason)
            if (
                goal.kind is ManipulationKind.BOX_PHASE
                and goal.box_action == "place"
                and (not state.box_attached or self._attachment_offset is None)
            ):
                return Acceptance(False, "place_requires_pick")
            self._goal = goal
            self._phase = "accepted"
            self._started_s = state.sim_time_s
            self._stall_error = self._initial_stall_error(goal, state)
            self._stall_time_s = state.sim_time_s
            self._release_started_s = None
            self._commands = ManipulationCommands()
            return Acceptance(True, "accepted")

    def request_cancel(self) -> bool:
        with self._lock:
            if self._terminal is not None or self._goal is None:
                return False
            self._cancel_requested = True
            return True

    def reset(self, initial_time_s: float) -> None:
        if not math.isfinite(initial_time_s):
            raise ValueError("initial_time_s must be finite")
        with self._lock:
            if self._goal is not None:
                raise RuntimeError("cannot reset an active manipulation goal")
            self._initial_time_s = initial_time_s
            self._phase = "idle"
            self._started_s = initial_time_s
            self._terminal = None
            self._cancel_requested = False
            self._commands = ManipulationCommands()
            self._stall_error = math.inf
            self._stall_time_s = initial_time_s
            self._release_started_s = None

    def observe(self, state: ManipulationState) -> ManipulationObservation:
        self._validate_state(state)
        with self._lock:
            if self._terminal is not None:
                return self._terminal
            if self._goal is None:
                fault = self._attachment_fault_locked(state)
                if fault:
                    return self._finish_locked(
                        Outcome.FAILED,
                        fault,
                        "terminal",
                        state.sim_time_s,
                    )
                return ManipulationObservation(
                    "idle",
                    Outcome.PENDING,
                    "",
                    False,
                    False,
                    "",
                    "none",
                )
            if self._cancel_requested:
                return self._finish_locked(
                    Outcome.CANCELED,
                    "cancel_requested",
                    "terminal",
                    state.sim_time_s,
                )

            fault = self._attachment_fault_locked(state)
            if fault:
                return self._finish_locked(Outcome.FAILED, fault, "fault", state.sim_time_s)

            timeout = self._timeout_for(self._goal)
            if state.sim_time_s - self._started_s > timeout:
                return self._finish_locked(
                    Outcome.FAILED,
                    "motion_timeout",
                    "timeout",
                    state.sim_time_s,
                )

            result = self._observe_goal_locked(self._goal, state)
            if result.terminal:
                return self._finish_locked(
                    result.outcome,
                    result.reason,
                    result.phase,
                    state.sim_time_s,
                    result,
                )
            self._phase = result.phase
            self._commands = self._commands_for(self._goal, state, result)
            return result

    def commands(self) -> ManipulationCommands:
        with self._lock:
            return self._commands

    def attachment_fault(self, state: ManipulationState) -> str:
        self._validate_state(state)
        with self._lock:
            return self._attachment_fault_locked(state)

    def _observe_goal_locked(
        self, goal: ManipulationGoal, state: ManipulationState
    ) -> ManipulationObservation:
        if goal.kind is ManipulationKind.WAIST:
            return self._observe_waist(goal, state)
        if goal.kind is ManipulationKind.NAMED_CONFIG:
            return self._observe_named(goal, state)
        if goal.kind is ManipulationKind.GRIPPER:
            return self._observe_gripper(goal, state)
        if goal.box_action == "pick":
            return self._observe_pick(goal, state)
        return self._observe_place(goal, state)

    def _observe_waist(
        self, goal: ManipulationGoal, state: ManipulationState
    ) -> ManipulationObservation:
        target = self._waist_target(goal.height_mm)
        error = target - state.waist_position_m
        remaining_mm = abs(error) * 1000.0
        progress = self._progress(remaining_mm / 1000.0, self._waist_span())
        if self._stalled(state, abs(error), self._config.timeouts.joint_stall_threshold_rad):
            return self._failure("joint_stall", "stalled", "waist", state, progress)
        if abs(error) <= self._config.thresholds.waist_arrival_m:
            return self._success("settling", "waist reached target", "waist", state, progress)
        return ManipulationObservation(
            "moving",
            Outcome.RUNNING,
            "",
            False,
            False,
            "",
            "waist",
            position_mm=state.waist_position_m * 1000.0,
            remaining_mm=remaining_mm,
            progress=progress,
        )

    def _observe_named(
        self, goal: ManipulationGoal, state: ManipulationState
    ) -> ManipulationObservation:
        target = self._config.named_targets[goal.target or ""]
        errors = self._arm_errors(target, state)
        max_error = max(errors)
        progress = self._progress(max_error, math.pi)
        if self._stalled(state, max_error, self._config.timeouts.joint_stall_threshold_rad):
            return self._failure("joint_stall", "stalled", "named_config", state, progress)
        if max_error <= self._config.thresholds.joint_arrival_rad:
            return self._success(
                "settling", "named config reached target", "named_config", state, progress
            )
        return ManipulationObservation(
            "interpolating",
            Outcome.RUNNING,
            "",
            False,
            False,
            "",
            "named_config",
            progress=progress,
            max_joint_error_rad=max_error,
        )

    def _observe_gripper(
        self, goal: ManipulationGoal, state: ManipulationState
    ) -> ManipulationObservation:
        target = goal.gripper_position or 0.0
        errors = tuple(abs(target - value) for value in state.gripper_positions)
        remaining = max(errors)
        span = self._gripper_span()
        progress = self._progress(remaining, span)
        if remaining <= self._config.thresholds.gripper_arrival:
            return self._success(
                "settling", "grippers reached target", "gripper", state, progress
            )
        return ManipulationObservation(
            "closing_or_opening",
            Outcome.RUNNING,
            "",
            False,
            False,
            "",
            "gripper",
            progress=progress,
            gripper_position=sum(state.gripper_positions) / 2.0,
            gripper_remaining=remaining,
        )

    def _observe_pick(
        self, goal: ManipulationGoal, state: ManipulationState
    ) -> ManipulationObservation:
        target = self._config.named_targets["dualjo:joints_br"]
        arm_error = max(self._arm_errors(target, state))
        box_error, yaw_error = self._box_error(state, self._config.box.initial_pose_m, self._config.box.initial_yaw_rad)
        progress = self._progress(box_error, self._config.thresholds.grasp_alignment_m * 10.0)
        if box_error > self._config.thresholds.grasp_alignment_m or yaw_error > self._config.thresholds.grasp_yaw_rad:
            return self._failure("box_alignment_failed", "box is not at grasp pose", "box_phase", state, progress)
        if arm_error > self._config.thresholds.joint_arrival_rad:
            return self._failure("box_alignment_failed", "arms are not at grasp pose", "box_phase", state, progress)
        if any(
            abs(value - gripper.closed_position)
            > self._config.thresholds.gripper_arrival
            for value, gripper in zip(state.gripper_positions, self._ordered_grippers())
        ):
            return self._failure("grasp_failed", "grippers are not closed", "box_phase", state, progress)
        if state.box_attached:
            self._remember_attachment(state)
            return self._success("attached", "box attached", "box_phase", state, 1.0)
        return ManipulationObservation(
            "verifying",
            Outcome.RUNNING,
            "",
            False,
            False,
            "",
            "box_phase",
            progress=1.0,
            box_error_m=box_error,
        )

    def _observe_place(
        self, goal: ManipulationGoal, state: ManipulationState
    ) -> ManipulationObservation:
        target = self._config.named_targets["dualposi_armbase_abso:pt_1f1_ready"]
        arm_error = max(self._arm_errors(target, state))
        box_error, yaw_error = self._box_error(
            state, self._config.box.place_target_m, self._config.box.place_target_yaw_rad
        )
        progress = self._progress(box_error, self._config.thresholds.place_alignment_m * 10.0)
        if (
            box_error > self._config.thresholds.place_alignment_m
            or yaw_error > self._config.thresholds.place_yaw_rad
            or arm_error > self._config.thresholds.joint_arrival_rad
        ):
            return self._failure(
                "place_alignment_failed", "box or arms are not at place pose", "box_phase", state, progress
            )
        opened = all(
            abs(value - gripper.open_position) <= self._config.thresholds.gripper_arrival
            for value, gripper in zip(state.gripper_positions, self._ordered_grippers())
        )
        if not opened:
            if not state.box_attached:
                return self._failure(
                    "release_settle_failed",
                    "box detached before grippers opened",
                    "box_phase",
                    state,
                    progress,
                )
            return ManipulationObservation(
                "opening",
                Outcome.RUNNING,
                "",
                False,
                False,
                "",
                "box_phase",
                progress=progress,
                box_error_m=box_error,
            )
        if state.box_attached:
            return ManipulationObservation(
                "releasing",
                Outcome.RUNNING,
                "",
                False,
                False,
                "",
                "box_phase",
                progress=progress,
                box_error_m=box_error,
            )
        if self._release_started_s is None:
            self._release_started_s = state.sim_time_s
        elapsed = (
            0.0
            if self._release_started_s is None
            else state.sim_time_s - self._release_started_s
        )
        if elapsed + 1e-9 < self._config.thresholds.release_settle_s:
            roll, pitch, _ = state.box_orientation_rad
            if abs(roll) > self._config.thresholds.release_roll_pitch_rad or abs(
                pitch
            ) > self._config.thresholds.release_roll_pitch_rad:
                return self._failure(
                    "release_settle_failed",
                    "released box did not settle upright",
                    "box_phase",
                    state,
                    progress,
                )
            return ManipulationObservation(
                "settling",
                Outcome.RUNNING,
                "",
                False,
                False,
                "",
                "box_phase",
                progress=progress,
                box_error_m=box_error,
            )
        roll, pitch, _ = state.box_orientation_rad
        if abs(roll) > self._config.thresholds.release_roll_pitch_rad or abs(pitch) > self._config.thresholds.release_roll_pitch_rad:
            return self._failure("release_settle_failed", "released box did not settle upright", "box_phase", state, 1.0)
        return self._success("settling", "box placed", "box_phase", state, 1.0)

    def _commands_for(
        self,
        goal: ManipulationGoal,
        state: ManipulationState,
        observation: ManipulationObservation,
    ) -> ManipulationCommands:
        if goal.kind is ManipulationKind.WAIST:
            target = self._waist_target(goal.height_mm)
            error = target - state.waist_position_m
            speed = min(
                self._config.waist.max_velocity_mps,
                math.sqrt(2.0 * 0.05 * abs(error)),
            )
            velocity = (
                0.0
                if abs(error) <= self._config.thresholds.waist_arrival_m
                else math.copysign(speed, error)
            )
            return ManipulationCommands(waist_velocity_mps=velocity)
        if goal.kind is ManipulationKind.NAMED_CONFIG:
            target = self._config.named_targets[goal.target or ""]
            desired = target.left_joints_rad + target.right_joints_rad
            limits = self._config.left_arm.max_velocity_radps + self._config.right_arm.max_velocity_radps
            velocities = tuple(
                0.0 if abs(target_value - current_value) <= self._config.thresholds.joint_arrival_rad
                else math.copysign(limit, target_value - current_value)
                for target_value, current_value, limit in zip(desired, state.arm_positions_rad, limits)
            )
            waist_error = self._waist_target(target.waist_height_mm) - state.waist_position_m
            gripper_error = target.gripper_position - sum(state.gripper_positions) / 2.0
            gripper_velocity = 0.0 if abs(gripper_error) <= self._config.thresholds.gripper_arrival else math.copysign(
                min(config.max_velocity for config in self._ordered_grippers()), gripper_error
            )
            return ManipulationCommands(
                waist_velocity_mps=0.0 if abs(waist_error) <= self._config.thresholds.waist_arrival_m else math.copysign(
                    self._config.waist.max_velocity_mps, waist_error
                ),
                arm_velocities_radps=velocities,
                gripper_velocities=(gripper_velocity, gripper_velocity),
            )
        if goal.kind is ManipulationKind.GRIPPER:
            target = goal.gripper_position or 0.0
            velocity = 0.0 if observation.phase == "settling" else math.copysign(
                min(config.max_velocity for config in self._ordered_grippers()),
                target - sum(state.gripper_positions) / 2.0,
            )
            return ManipulationCommands(gripper_velocities=(velocity, velocity))
        if goal.box_action == "pick":
            return ManipulationCommands(attach_box=observation.phase == "verifying")
        if observation.phase == "opening":
            open_target = sum(
                config.open_position for config in self._ordered_grippers()
            ) / len(self._ordered_grippers())
            error = open_target - sum(state.gripper_positions) / len(state.gripper_positions)
            velocity = (
                0.0
                if abs(error) <= self._config.thresholds.gripper_arrival
                else math.copysign(
                    min(config.max_velocity for config in self._ordered_grippers()), error
                )
            )
            return ManipulationCommands(gripper_velocities=(velocity, velocity))
        return ManipulationCommands(release_box=observation.phase == "releasing")

    def _finish_locked(
        self,
        outcome: Outcome,
        reason: str,
        phase: str,
        sim_time_s: float,
        result: ManipulationObservation | None = None,
    ) -> ManipulationObservation:
        if result is None:
            progress = 0.0
            if self._goal is not None:
                progress = self._phase_progress(self._goal)
            result = ManipulationObservation(
                phase,
                outcome,
                reason,
                True,
                outcome is Outcome.SUCCEEDED,
                reason,
                self._feedback_kind(self._goal),
                progress=progress,
            )
        else:
            result = replace(result, terminal=True, outcome=outcome, reason=reason, message=reason)
        self._commands = ManipulationCommands()
        self._goal = None
        self._cancel_requested = False
        self._phase = phase
        self._terminal = result
        return result

    def _success(
        self, phase: str, message: str, feedback_kind: str, state: ManipulationState, progress: float
    ) -> ManipulationObservation:
        return ManipulationObservation(
            phase,
            Outcome.SUCCEEDED,
            "",
            True,
            True,
            message,
            feedback_kind,
            progress=progress,
            position_mm=state.waist_position_m * 1000.0,
        )

    def _failure(
        self, reason: str, message: str, feedback_kind: str, state: ManipulationState, progress: float
    ) -> ManipulationObservation:
        return ManipulationObservation(
            "failed",
            Outcome.FAILED,
            reason,
            True,
            False,
            message,
            feedback_kind,
            progress=progress,
            position_mm=state.waist_position_m * 1000.0,
        )

    def _initial_stall_error(
        self, goal: ManipulationGoal, state: ManipulationState
    ) -> float:
        if goal.kind is ManipulationKind.WAIST:
            return abs(self._waist_target(goal.height_mm) - state.waist_position_m)
        if goal.kind is ManipulationKind.NAMED_CONFIG:
            return max(
                self._arm_errors(self._config.named_targets[goal.target or ""], state)
            )
        if goal.kind is ManipulationKind.GRIPPER:
            return max(
                abs((goal.gripper_position or 0.0) - value)
                for value in state.gripper_positions
            )
        return math.inf

    def _stalled(self, state: ManipulationState, error: float, threshold: float) -> bool:
        elapsed = state.sim_time_s - self._stall_time_s
        if elapsed < self._config.timeouts.stall_window_s:
            return False
        stalled = abs(self._stall_error - error) < threshold
        self._stall_error = error
        self._stall_time_s = state.sim_time_s
        return stalled

    def _attachment_fault_locked(self, state: ManipulationState) -> str:
        if not state.box_attached or self._attachment_offset is None:
            return ""
        current = self._attachment_offset_from_state(state)
        translation_error = math.hypot(current.x_m - self._attachment_offset.x_m, current.y_m - self._attachment_offset.y_m)
        yaw_error = abs(_angle_delta(current.yaw_rad, self._attachment_offset.yaw_rad))
        if translation_error > self._config.thresholds.carry_drift_m or yaw_error > self._config.thresholds.place_yaw_rad:
            return "carry_lost"
        return ""

    def _remember_attachment(self, state: ManipulationState) -> None:
        self._attachment_offset = self._attachment_offset_from_state(state)

    def _attachment_offset_from_state(self, state: ManipulationState) -> _AttachmentOffset:
        dx = state.box_position_m[0] - state.robot_pose.x
        dy = state.box_position_m[1] - state.robot_pose.y
        yaw = state.robot_pose.yaw
        return _AttachmentOffset(
            x_m=math.cos(yaw) * dx + math.sin(yaw) * dy,
            y_m=-math.sin(yaw) * dx + math.cos(yaw) * dy,
            yaw_rad=pi_symmetric_angular_error(
                yaw, state.box_orientation_rad[2]
            ),
        )

    def _arm_errors(self, target: Any, state: ManipulationState) -> tuple[float, ...]:
        desired = target.left_joints_rad + target.right_joints_rad
        return tuple(abs(expected - actual) for expected, actual in zip(desired, state.arm_positions_rad))

    def _box_error(
        self, state: ManipulationState, target: tuple[float, float, float], target_yaw: float
    ) -> tuple[float, float]:
        translation_error = math.hypot(
            state.box_position_m[0] - target[0], state.box_position_m[1] - target[1]
        )
        yaw_error = abs(
            pi_symmetric_angular_error(
                state.box_orientation_rad[2], target_yaw
            )
        )
        return translation_error, yaw_error

    def _validate_goal(self, goal: ManipulationGoal) -> str:
        if goal.kind is ManipulationKind.WAIST:
            if goal.height_mm is None or not math.isfinite(goal.height_mm):
                return "invalid_height"
            position = self._waist_target(goal.height_mm)
            if not self._config.waist.lower_m <= position <= self._config.waist.upper_m:
                return "waist_out_of_range"
            return ""
        if goal.kind is ManipulationKind.NAMED_CONFIG:
            if goal.target not in self._config.named_targets:
                return "unknown_target"
            return ""
        if goal.kind is ManipulationKind.GRIPPER:
            if goal.gripper_position is None or not math.isfinite(goal.gripper_position):
                return "invalid_gripper_position"
            closed = min(config.closed_position for config in self._config.grippers.values())
            opened = max(config.open_position for config in self._config.grippers.values())
            if not closed <= goal.gripper_position <= opened:
                return "gripper_out_of_range"
            return ""
        if goal.box_action not in ("pick", "place"):
            return "invalid_box_action"
        if goal.flow != "flow":
            return "unsupported_flow"
        if goal.config_name != "box1":
            return "unknown_box_config"
        return ""

    def _validate_state(self, state: ManipulationState) -> None:
        scalar_values = (
            state.sim_time_s,
            state.waist_position_m,
            *state.arm_positions_rad,
            *state.gripper_positions,
            *state.box_position_m,
            *state.box_orientation_rad,
            state.robot_pose.x,
            state.robot_pose.y,
            state.robot_pose.yaw,
        )
        if (
            len(state.arm_positions_rad) != 12
            or len(state.gripper_positions) != 2
            or len(state.box_position_m) != 3
            or len(state.box_orientation_rad) != 3
            or not all(math.isfinite(float(value)) for value in scalar_values)
        ):
            raise ValueError("manipulation state values must be finite and correctly sized")
        if not isinstance(state.box_attached, bool):
            raise ValueError("box_attached must be boolean")

    def _waist_target(self, height_mm: float | None) -> float:
        return self._config.waist.height_reference_m + float(height_mm or 0.0) / 1000.0

    def _ordered_grippers(self) -> tuple[Any, Any]:
        return self._config.grippers["left"], self._config.grippers["right"]

    def _gripper_span(self) -> float:
        return max(config.open_position - config.closed_position for config in self._ordered_grippers())

    def _waist_span(self) -> float:
        return self._config.waist.upper_m - self._config.waist.lower_m

    def _timeout_for(self, goal: ManipulationGoal) -> float:
        if goal.kind is ManipulationKind.WAIST:
            return self._config.timeouts.waist_s
        if goal.kind is ManipulationKind.NAMED_CONFIG:
            return self._config.timeouts.named_config_s
        if goal.kind is ManipulationKind.GRIPPER:
            return self._config.timeouts.gripper_s
        return self._config.timeouts.box_phase_s

    def _feedback_kind(self, goal: ManipulationGoal | None) -> str:
        if goal is None:
            return "none"
        if goal.kind is ManipulationKind.WAIST:
            return "waist"
        if goal.kind is ManipulationKind.NAMED_CONFIG:
            return "named_config"
        if goal.kind is ManipulationKind.GRIPPER:
            return "gripper"
        return "box_phase"

    def _phase_progress(self, goal: ManipulationGoal | None) -> float:
        if goal is None:
            return 0.0
        if goal.kind is ManipulationKind.WAIST:
            return self._progress(abs((goal.height_mm or 0.0) / 1000.0), self._waist_span())
        if goal.kind is ManipulationKind.NAMED_CONFIG:
            return self._progress(0.0, math.pi)
        return 1.0

    @staticmethod
    def _progress(remaining: float, span: float) -> float:
        if span <= 0.0:
            return 1.0
        return min(1.0, max(0.0, 1.0 - remaining / span))


def _angle_delta(angle: float, reference: float) -> float:
    return math.atan2(math.sin(angle - reference), math.cos(angle - reference))
