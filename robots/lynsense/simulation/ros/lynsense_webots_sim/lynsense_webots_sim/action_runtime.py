from __future__ import annotations

import enum
import math
import threading
import unicodedata
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

from lynsense_webots_sim.geometry import Pose2
from lynsense_webots_sim.state_machine import (
    ActionFeedback,
    MotionCommand,
    MotionPhase,
    MotionProfile,
    NavigationStateMachine,
    Outcome as StateMachineOutcome,
)


MAX_GOAL_NAME_BYTES = 128
MAX_DISTANCE_M = 10.0
MAX_ANGLE_DEG = 360.0


class Outcome(enum.Enum):
    PENDING = "pending"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"


@dataclass(frozen=True)
class Acceptance:
    accepted: bool
    reason: str


@dataclass(frozen=True)
class ExpectedAction:
    kind: str
    goal_name: str | None = None
    distance_m: float | None = None
    angle_deg: float | None = None

    @classmethod
    def nav(cls, goal_name: str) -> "ExpectedAction":
        return cls("nav", goal_name=goal_name)

    @classmethod
    def move(cls, distance_m: float, angle_deg: float) -> "ExpectedAction":
        return cls("move", distance_m=distance_m, angle_deg=angle_deg)

    def matches(self, other: "ExpectedAction") -> bool:
        return (
            self.kind == other.kind
            and self.goal_name == other.goal_name
            and self.distance_m == other.distance_m
            and self.angle_deg == other.angle_deg
        )


@dataclass(frozen=True)
class TerminalResult:
    success: bool
    message: str
    outcome: Outcome


@dataclass(frozen=True)
class RuntimeObservation:
    command: MotionCommand
    feedback: ActionFeedback
    phase: MotionPhase
    outcome: Outcome
    reason: str
    terminal: bool
    terminal_result: TerminalResult | None


@dataclass(frozen=True)
class _PendingAction:
    action: ExpectedAction


def _goal_name_is_valid(goal_name: str) -> bool:
    if not isinstance(goal_name, str) or not goal_name:
        return False
    try:
        encoded = goal_name.encode("utf-8")
    except UnicodeEncodeError:
        return False
    if len(encoded) > MAX_GOAL_NAME_BYTES:
        return False
    return not any(unicodedata.category(character) == "Cc" for character in goal_name)


def _move_is_valid(distance_m: float, angle_deg: float) -> bool:
    return (
        math.isfinite(distance_m)
        and -MAX_DISTANCE_M <= distance_m <= MAX_DISTANCE_M
        and math.isfinite(angle_deg)
        and -MAX_ANGLE_DEG <= angle_deg <= MAX_ANGLE_DEG
    )


def _nonnegative_finite(value: float) -> float:
    if not math.isfinite(value) or value <= 0.0:
        return 0.0
    return float(value)


class ActionRuntime:
    def __init__(
        self,
        profile: MotionProfile,
        goals: dict[str, Pose2],
        allowed_actions: Sequence[ExpectedAction] | None = None,
    ) -> None:
        self._profile = profile
        self._goals = dict(goals)
        self._allowed_actions = None if allowed_actions is None else tuple(allowed_actions)
        if self._allowed_actions is not None and not self._allowed_actions:
            raise ValueError("allowed_actions must not be empty")
        self._machine: NavigationStateMachine | None = None
        self._pending: _PendingAction | None = None
        self._script_index = 0
        self._terminal: RuntimeObservation | None = None
        self._last_observation: RuntimeObservation | None = None
        self._cancel_requested = False
        self._lock = threading.Lock()

    @property
    def active(self) -> bool:
        with self._lock:
            return self._pending is not None or self._machine is not None

    @property
    def terminal_result(self) -> TerminalResult | None:
        with self._lock:
            if self._terminal is None:
                return None
            return self._terminal.terminal_result

    def submit_nav(self, goal_name: str) -> Acceptance:
        return self._submit(ExpectedAction.nav(goal_name))

    def submit_move(self, distance_m: float, angle_deg: float) -> Acceptance:
        return self._submit(ExpectedAction.move(distance_m, angle_deg))

    def request_cancel(self) -> bool:
        with self._lock:
            if self._terminal is not None or (self._pending is None and self._machine is None):
                return False
            self._cancel_requested = True
            return True

    def reset(self) -> None:
        with self._lock:
            if self._pending is not None or self._machine is not None:
                raise RuntimeError("cannot reset an active navigation goal")
            self._terminal = None
            self._last_observation = None
            self._cancel_requested = False

    def observe(self, pose: Pose2, sim_time_s: float) -> RuntimeObservation:
        with self._lock:
            if self._terminal is not None:
                return self._terminal
            if not math.isfinite(sim_time_s):
                raise ValueError("sim_time_s must be finite")

            if self._cancel_requested:
                self._finish_terminal(Outcome.CANCELED, "cancel_requested")
                return self._terminal

            if self._machine is None:
                if self._pending is None:
                    if self._last_observation is not None:
                        return self._last_observation
                    raise RuntimeError("no action has been submitted")
                if not isinstance(pose, Pose2) or not all(
                    math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)
                ):
                    raise ValueError("pose values must be finite")
                self._machine = NavigationStateMachine(self._profile)
                action = self._pending.action
                if action.kind == "nav":
                    self._machine.start_nav(
                        pose,
                        self._goals[action.goal_name],
                        sim_time_s,
                    )
                else:
                    self._machine.start_move(
                        pose,
                        action.distance_m,
                        action.angle_deg,
                        sim_time_s,
                    )
                self._pending = None

            result = self._machine.update(pose, sim_time_s)
            state_outcome = result.outcome
            if state_outcome is StateMachineOutcome.SUCCEEDED:
                outcome = Outcome.SUCCEEDED
            elif state_outcome is StateMachineOutcome.FAILED:
                outcome = Outcome.FAILED
            elif state_outcome is StateMachineOutcome.PENDING:
                outcome = Outcome.PENDING
            else:
                outcome = Outcome.RUNNING

            if outcome in (Outcome.SUCCEEDED, Outcome.FAILED):
                self._finish_terminal(outcome, result.reason)
                if self._terminal is None:
                    self._last_observation = self._terminal_observation(outcome, result.reason)
                    self._machine = None
                    return self._last_observation
            else:
                self._last_observation = RuntimeObservation(
                    command=result.command,
                    feedback=result.feedback,
                    phase=result.phase,
                    outcome=outcome,
                    reason=result.reason,
                    terminal=False,
                    terminal_result=None,
                )
                return self._last_observation
            return self._terminal

    def _submit(self, action: ExpectedAction) -> Acceptance:
        with self._lock:
            if self._terminal is not None:
                return Acceptance(False, "terminal_lockout")

            if self._allowed_actions is not None:
                expected = self._allowed_actions[self._script_index]
                if not expected.matches(action):
                    return Acceptance(False, "script_out_of_order")

            if self._pending is not None or self._machine is not None:
                return Acceptance(False, "action_active")
            if action.kind == "nav":
                if not _goal_name_is_valid(action.goal_name):
                    return Acceptance(False, "invalid_goal_name")
                if action.goal_name not in self._goals:
                    return Acceptance(False, "unknown_goal")
            else:
                if not _move_is_valid(action.distance_m, action.angle_deg):
                    return Acceptance(False, "invalid_move")

            self._pending = _PendingAction(action)
            return Acceptance(True, "")

    def _finish_terminal(self, outcome: Outcome, message: str) -> None:
        terminal = self._terminal_observation(outcome, message)
        self._pending = None
        self._machine = None
        self._cancel_requested = False
        if self._allowed_actions is not None:
            current_index = self._script_index
            is_final_index = current_index == len(self._allowed_actions) - 1
            if outcome is Outcome.SUCCEEDED and not is_final_index:
                self._script_index += 1
                return

        self._terminal = terminal

    def _terminal_observation(self, outcome: Outcome, message: str) -> RuntimeObservation:
        feedback = ActionFeedback.settled()
        if self._machine is not None:
            feedback = ActionFeedback(
                distance_remaining_m=_nonnegative_finite(
                    self._machine.feedback.distance_remaining_m
                ),
                angle_remaining_deg=_nonnegative_finite(
                    self._machine.feedback.angle_remaining_deg
                ),
            )
        terminal_result = TerminalResult(
            success=outcome is Outcome.SUCCEEDED,
            message=message,
            outcome=outcome,
        )
        return RuntimeObservation(
            command=MotionCommand.stopped(),
            feedback=feedback,
            phase=MotionPhase.TERMINAL,
            outcome=outcome,
            reason=message,
            terminal=True,
            terminal_result=terminal_result,
        )


class FaultSignal:
    def __init__(self) -> None:
        self._reason: str | None = None
        self._lock = threading.Lock()

    def request(self, reason: str) -> None:
        with self._lock:
            if self._reason is None:
                self._reason = reason

    def take(self) -> str | None:
        with self._lock:
            reason = self._reason
            self._reason = None
            return reason
