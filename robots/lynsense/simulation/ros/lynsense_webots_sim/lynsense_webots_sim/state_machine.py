from __future__ import annotations

import math
from dataclasses import dataclass
import enum

from lynsense_webots_sim.geometry import (
    Pose2,
    angular_error,
    bearing_to_goal,
    normalize_angle,
    wheel_rates,
)


def _require_finite_positive(name: str, value: float) -> float:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return value


@dataclass(frozen=True)
class MotionProfile:
    wheel_separation_m: float = 0.461
    wheel_radius_m: float = 0.08
    max_linear_velocity_mps: float = 2.0
    max_angular_velocity_radps: float = 1.5
    max_linear_acceleration_mps2: float = 1.0
    max_angular_acceleration_radps2: float = 1.0
    smoke_linear_velocity_mps: float = 0.3
    smoke_angular_velocity_radps: float = 0.5
    smoke_linear_acceleration_mps2: float = 0.5
    smoke_angular_acceleration_radps2: float = 1.0
    position_tolerance_m: float = 0.05
    yaw_tolerance_rad: float = math.radians(2)
    stall_window_s: float = 1.0
    yaw_stall_threshold_rad: float = math.radians(0.5)
    distance_stall_threshold_m: float = 0.005
    total_budget_s: float = 60.0
    timeout_factor: float = 3.0
    timeout_margin_s: float = 2.0
    minimum_stage_timeout_s: float = 5.0

    def __post_init__(self) -> None:
        finite_positive = (
            "wheel_separation_m",
            "wheel_radius_m",
            "max_linear_velocity_mps",
            "max_angular_velocity_radps",
            "max_linear_acceleration_mps2",
            "max_angular_acceleration_radps2",
            "smoke_linear_velocity_mps",
            "smoke_angular_velocity_radps",
            "smoke_linear_acceleration_mps2",
            "smoke_angular_acceleration_radps2",
            "position_tolerance_m",
            "yaw_tolerance_rad",
            "stall_window_s",
            "yaw_stall_threshold_rad",
            "distance_stall_threshold_m",
            "total_budget_s",
            "timeout_factor",
            "timeout_margin_s",
            "minimum_stage_timeout_s",
        )
        for name in finite_positive:
            _require_finite_positive(name, getattr(self, name))
        if self.yaw_tolerance_rad >= math.pi:
            raise ValueError("yaw_tolerance_rad must be below pi")
        if self.smoke_linear_velocity_mps > self.max_linear_velocity_mps:
            raise ValueError("smoke linear velocity exceeds hardware limit")
        if self.smoke_angular_velocity_radps > self.max_angular_velocity_radps:
            raise ValueError("smoke angular velocity exceeds hardware limit")
        if self.smoke_linear_acceleration_mps2 > self.max_linear_acceleration_mps2:
            raise ValueError("smoke linear acceleration exceeds hardware limit")
        if self.smoke_angular_acceleration_radps2 > self.max_angular_acceleration_radps2:
            raise ValueError("smoke angular acceleration exceeds hardware limit")

    @property
    def max_wheel_rate_radps(self) -> float:
        return self.max_linear_velocity_mps / self.wheel_radius_m


class MotionPhase(enum.Enum):
    IDLE = "idle"
    NAV_TURN = "nav_turn"
    NAV_APPROACH = "nav_approach"
    NAV_FINAL_YAW = "nav_final_yaw"
    MOVE_TURN = "move_turn"
    MOVE_TRANSLATE = "move_translate"
    TERMINAL = "terminal"


class Outcome(enum.Enum):
    PENDING = "pending"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"


@dataclass(frozen=True)
class MotionCommand:
    left_wheel_rate_radps: float
    right_wheel_rate_radps: float

    @classmethod
    def stopped(cls) -> "MotionCommand":
        return cls(0.0, 0.0)


@dataclass(frozen=True)
class ActionFeedback:
    distance_remaining_m: float
    angle_remaining_deg: float

    @property
    def distance_remaining(self) -> float:
        return self.distance_remaining_m

    @property
    def angle_remaining(self) -> float:
        return self.angle_remaining_deg

    @classmethod
    def settled(cls) -> "ActionFeedback":
        return cls(0.0, 0.0)


@dataclass(frozen=True)
class StateMachineUpdate:
    command: MotionCommand
    feedback: ActionFeedback
    phase: MotionPhase
    outcome: Outcome
    reason: str


def _pose_is_finite(pose: Pose2) -> bool:
    return math.isfinite(pose.x) and math.isfinite(pose.y) and math.isfinite(pose.yaw)


class NavigationStateMachine:
    def __init__(self, profile: MotionProfile) -> None:
        self.profile = profile
        self.phase = MotionPhase.IDLE
        self.outcome = Outcome.PENDING
        self.reason = ""
        self.command = MotionCommand.stopped()
        self.feedback = ActionFeedback.settled()
        self._start_pose = Pose2(0.0, 0.0, 0.0)
        self._goal = Pose2(0.0, 0.0, 0.0)
        self._translation_target = Pose2(0.0, 0.0, 0.0)
        self._turn_target_yaw = 0.0
        self._move_requested_distance_m = 0.0
        self._stage_start_time_s = 0.0
        self._stage_timeout_s = 0.0
        self._stall_time_s = 0.0
        self._stall_distance_m = 0.0
        self._stall_angle_rad = 0.0
        self._previous_v = 0.0
        self._previous_omega = 0.0
        self._previous_control_time_s = 0.0
        self._start_time_s = 0.0

    def start_nav(self, start: Pose2, goal: Pose2, sim_time_s: float) -> None:
        self._validate_start_time(sim_time_s)
        if not _pose_is_finite(start) or not _pose_is_finite(goal):
            raise ValueError("start and goal poses must be finite")
        self._start_pose = start
        self._goal = goal
        self._start_time_s = sim_time_s
        self.feedback = ActionFeedback.settled()
        self._move_requested_distance_m = 0.0
        self._reset_motion_limits()
        distance = math.hypot(goal.x - start.x, goal.y - start.y)
        initial_yaw_error = abs(angular_error(start.yaw, bearing_to_goal(start, goal)))
        if distance <= self.profile.position_tolerance_m:
            yaw_error = abs(angular_error(start.yaw, goal.yaw))
            if yaw_error <= self.profile.yaw_tolerance_rad:
                self._finish(Outcome.SUCCEEDED)
            else:
                self._begin_stage(
                    MotionPhase.NAV_FINAL_YAW,
                    start,
                    sim_time_s,
                    distance=0.0,
                    angle=yaw_error,
                )
            self._turn_target_yaw = goal.yaw
        else:
            self._begin_stage(
                MotionPhase.NAV_TURN,
                start,
                sim_time_s,
                distance=0.0,
                angle=initial_yaw_error,
            )
            self._turn_target_yaw = bearing_to_goal(start, goal)

    def start_move(
        self,
        start: Pose2,
        distance_m: float,
        angle_deg: float,
        sim_time_s: float,
    ) -> None:
        self._validate_start_time(sim_time_s)
        if not _pose_is_finite(start):
            raise ValueError("start pose must be finite")
        if not math.isfinite(distance_m) or not math.isfinite(angle_deg):
            raise ValueError("distance and angle must be finite")
        self._start_pose = start
        self._goal = start
        self._start_time_s = sim_time_s
        self._reset_motion_limits()
        angle_rad = math.radians(angle_deg)
        self._turn_target_yaw = normalize_angle(start.yaw + angle_rad)
        self._translation_target = Pose2(
            start.x + distance_m * math.cos(self._turn_target_yaw),
            start.y + distance_m * math.sin(self._turn_target_yaw),
            self._turn_target_yaw,
        )
        distance_error = abs(
            math.hypot(
                self._translation_target.x - start.x,
                self._translation_target.y - start.y,
            )
        )
        yaw_error = abs(angular_error(start.yaw, self._turn_target_yaw))
        self._move_requested_distance_m = distance_error
        self.feedback = ActionFeedback(
            distance_remaining_m=distance_error,
            angle_remaining_deg=math.degrees(yaw_error),
        )
        if distance_error <= self.profile.position_tolerance_m:
            if yaw_error <= self.profile.yaw_tolerance_rad:
                self._finish(Outcome.SUCCEEDED)
            else:
                self._begin_stage(
                    MotionPhase.MOVE_TURN,
                    start,
                    sim_time_s,
                    distance=distance_error,
                    angle=yaw_error,
                )
        else:
            self._begin_stage(
                MotionPhase.MOVE_TURN,
                start,
                sim_time_s,
                distance=0.0,
                angle=yaw_error,
            )

    def update(self, pose: Pose2, sim_time_s: float) -> StateMachineUpdate:
        if not math.isfinite(sim_time_s):
            self._finish(Outcome.FAILED, "invalid_sim_time")
        elif self.phase is MotionPhase.TERMINAL:
            self.command = MotionCommand.stopped()
        elif not _pose_is_finite(pose):
            self._finish(Outcome.FAILED, "invalid_pose")

        if self.phase is MotionPhase.TERMINAL:
            return self._result()
        if self.phase is MotionPhase.IDLE:
            self.command = MotionCommand.stopped()
            return self._result()

        elapsed_total = sim_time_s - self._start_time_s
        if elapsed_total > self.profile.total_budget_s:
            self._finish(Outcome.FAILED, "total_timeout")
            return self._result()

        stage_elapsed = sim_time_s - self._stage_start_time_s
        if stage_elapsed > self._stage_timeout_s:
            self._finish(Outcome.FAILED, f"{self.phase.value}_timeout")
            return self._result()

        distance, angle, target_yaw = self._stage_state(pose)
        self.feedback = ActionFeedback(
            distance_remaining_m=distance,
            angle_remaining_deg=math.degrees(abs(angle)),
        )
        self._check_stall(pose, sim_time_s)
        if self.phase is MotionPhase.TERMINAL:
            return self._result()

        transitioned = self._advance_phase(pose, distance, angle, sim_time_s)
        if transitioned:
            if self.phase is MotionPhase.TERMINAL:
                return self._result()
            distance, angle, target_yaw = self._stage_state(pose)

        self.feedback = ActionFeedback(
            distance_remaining_m=distance,
            angle_remaining_deg=math.degrees(abs(angle)),
        )
        self._control(pose, sim_time_s, target_yaw)
        return self._result()

    def _validate_start_time(self, sim_time_s: float) -> None:
        if not math.isfinite(sim_time_s):
            raise ValueError("sim_time_s must be finite")

    def _reset_motion_limits(self) -> None:
        self._previous_v = 0.0
        self._previous_omega = 0.0
        self._previous_control_time_s = self._start_time_s

    def _begin_stage(
        self,
        phase: MotionPhase,
        pose: Pose2,
        sim_time_s: float,
        *,
        distance: float,
        angle: float,
    ) -> None:
        travel_time = distance / self.profile.smoke_linear_velocity_mps
        turn_time = angle / self.profile.smoke_angular_velocity_radps
        nominal_time = max(travel_time, turn_time)
        self.phase = phase
        self.outcome = Outcome.RUNNING
        self.reason = ""
        self._stage_start_time_s = sim_time_s
        self._stage_timeout_s = max(
            self.profile.minimum_stage_timeout_s,
            self.profile.timeout_factor * nominal_time + self.profile.timeout_margin_s,
        )
        self.feedback = ActionFeedback(
            distance_remaining_m=distance,
            angle_remaining_deg=math.degrees(angle),
        )
        self._reset_stall(pose, sim_time_s, distance=distance, angle=abs(angle))

    def _reset_stall(
        self,
        pose: Pose2,
        sim_time_s: float,
        *,
        distance: float,
        angle: float,
    ) -> None:
        self._stall_time_s = sim_time_s
        self._stall_distance_m = distance
        self._stall_angle_rad = angle

    def _stage_state(self, pose: Pose2) -> tuple[float, float, float]:
        if self.phase in (
            MotionPhase.NAV_TURN,
            MotionPhase.MOVE_TURN,
        ):
            angle = angular_error(pose.yaw, self._turn_target_yaw)
            if self.phase is MotionPhase.MOVE_TURN:
                distance = self._move_requested_distance_m
            else:
                distance = math.hypot(
                    self._goal.x - pose.x,
                    self._goal.y - pose.y,
                )
            return distance, angle, self._turn_target_yaw
        if self.phase in (MotionPhase.NAV_APPROACH, MotionPhase.NAV_FINAL_YAW):
            distance = math.hypot(self._goal.x - pose.x, self._goal.y - pose.y)
            if self.phase is MotionPhase.NAV_APPROACH:
                target_yaw = bearing_to_goal(pose, self._goal)
            else:
                target_yaw = self._goal.yaw
            angle = angular_error(pose.yaw, target_yaw)
            return distance, angle, target_yaw
        if self.phase is MotionPhase.MOVE_TRANSLATE:
            distance = math.hypot(
                self._translation_target.x - pose.x,
                self._translation_target.y - pose.y,
            )
            angle = angular_error(pose.yaw, self._turn_target_yaw)
            return distance, angle, self._turn_target_yaw
        return 0.0, 0.0, pose.yaw

    def _check_stall(self, pose: Pose2, sim_time_s: float) -> None:
        elapsed = sim_time_s - self._stall_time_s
        if self.phase in (
            MotionPhase.NAV_TURN,
            MotionPhase.MOVE_TURN,
            MotionPhase.NAV_FINAL_YAW,
        ):
            current_error = abs(self._stage_state(pose)[1])
            previous_error = self._stall_angle_rad
            threshold = self.profile.yaw_stall_threshold_rad
            reason = "turn_stalled"
        else:
            current_error = self._stage_state(pose)[0]
            previous_error = self._stall_distance_m
            threshold = self.profile.distance_stall_threshold_m
            reason = "translate_stalled"
        progress = previous_error - current_error

        if progress >= threshold:
            distance, angle, _ = self._stage_state(pose)
            self._reset_stall(pose, sim_time_s, distance=distance, angle=abs(angle))
        elif elapsed >= self.profile.stall_window_s:
            self._finish(Outcome.FAILED, reason)

    def _advance_phase(
        self,
        pose: Pose2,
        distance: float,
        angle: float,
        sim_time_s: float,
    ) -> bool:
        if self.phase is MotionPhase.NAV_TURN:
            if abs(angle) <= self.profile.yaw_tolerance_rad:
                self._begin_stage(
                    MotionPhase.NAV_APPROACH,
                    pose,
                    sim_time_s,
                    distance=distance,
                    angle=angle,
                )
                return True
            return False
        if self.phase is MotionPhase.NAV_APPROACH:
            if distance <= self.profile.position_tolerance_m:
                angle = angular_error(pose.yaw, self._goal.yaw)
                self._begin_stage(
                    MotionPhase.NAV_FINAL_YAW,
                    pose,
                    sim_time_s,
                    distance=distance,
                    angle=abs(angle),
                )
                return True
            return False
        if self.phase is MotionPhase.NAV_FINAL_YAW:
            if (
                distance <= self.profile.position_tolerance_m
                and abs(angle) <= self.profile.yaw_tolerance_rad
            ):
                self._finish(Outcome.SUCCEEDED)
                return True
            if distance > self.profile.position_tolerance_m:
                approach_angle = angular_error(
                    pose.yaw,
                    bearing_to_goal(pose, self._goal),
                )
                self._begin_stage(
                    MotionPhase.NAV_APPROACH,
                    pose,
                    sim_time_s,
                    distance=distance,
                    angle=approach_angle,
                )
                return True
            return False
        if self.phase is MotionPhase.MOVE_TURN:
            if abs(angle) <= self.profile.yaw_tolerance_rad:
                if distance <= self.profile.position_tolerance_m:
                    self._finish(Outcome.SUCCEEDED)
                else:
                    self._begin_stage(
                        MotionPhase.MOVE_TRANSLATE,
                        pose,
                        sim_time_s,
                        distance=distance,
                        angle=angle,
                    )
                return True
            return False
        if self.phase is MotionPhase.MOVE_TRANSLATE:
            if (
                distance <= self.profile.position_tolerance_m
                and abs(angle) <= self.profile.yaw_tolerance_rad
            ):
                self._finish(Outcome.SUCCEEDED)
                return True
        return False

    def _control(self, pose: Pose2, sim_time_s: float, target_yaw: float) -> None:
        dt = max(0.0, sim_time_s - self._previous_control_time_s)
        desired_v = 0.0
        desired_omega = angular_error(pose.yaw, target_yaw) * 1.5
        if self.phase is MotionPhase.NAV_APPROACH:
            target = self._goal
        elif self.phase is MotionPhase.MOVE_TRANSLATE:
            target = self._translation_target
        else:
            target = None
        if target is not None:
            forward_error = (
                (target.x - pose.x) * math.cos(pose.yaw)
                + (target.y - pose.y) * math.sin(pose.yaw)
            )
            direction = 1.0 if forward_error >= 0.0 else -1.0
            desired_v = min(
                self.profile.smoke_linear_velocity_mps,
                self.feedback.distance_remaining_m * 2.0,
            )
            desired_v *= direction
        desired_v = min(desired_v, self.profile.smoke_linear_velocity_mps)
        desired_omega = max(
            -self.profile.smoke_angular_velocity_radps,
            min(self.profile.smoke_angular_velocity_radps, desired_omega),
        )

        v_step = self.profile.smoke_linear_acceleration_mps2 * dt
        omega_step = self.profile.smoke_angular_acceleration_radps2 * dt
        self._previous_v = max(
            self._previous_v - v_step,
            min(desired_v, self._previous_v + v_step),
        )
        self._previous_omega = max(
            self._previous_omega - omega_step,
            min(desired_omega, self._previous_omega + omega_step),
        )
        self._previous_control_time_s = sim_time_s
        left, right = wheel_rates(self._previous_v, self._previous_omega, self.profile)
        self.command = MotionCommand(left, right)

    def _finish(self, outcome: Outcome, reason: str = "") -> None:
        self.phase = MotionPhase.TERMINAL
        self.outcome = outcome
        self.reason = reason
        self.command = MotionCommand.stopped()

    def _result(self) -> StateMachineUpdate:
        return StateMachineUpdate(
            command=self.command,
            feedback=self.feedback,
            phase=self.phase,
            outcome=self.outcome,
            reason=self.reason,
        )
