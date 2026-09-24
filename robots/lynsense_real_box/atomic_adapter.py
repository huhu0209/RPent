"""Transport-independent LynrotControl atomic-capability coordinator."""

from __future__ import annotations

import copy
import math
import threading
import time
from collections.abc import Callable, Mapping
from concurrent.futures import Future, ThreadPoolExecutor
from concurrent.futures import TimeoutError as FutureTimeoutError
from dataclasses import dataclass
from typing import Any, Protocol

from robots.lynsense_real_box.atomic_contract import atomic_call_rejection
from robots.lynsense_real_box.atomic_profile import (
    AtomicCapabilityProfile,
    atomic_profile_hash,
)


class LynrotControlRuntime(Protocol):
    """The narrow public-object shape consumed by this coordinator."""

    arms: Any
    grippers: Any
    waist: Any


class ChassisRuntime(Protocol):
    """A separately reviewed chassis transport outside LynrotControl."""

    def read(self) -> Any:
        """Return fresh chassis motion state."""

    def navigate(self, goal: str) -> Any:
        """Start one named navigation goal and return its task."""

    def move_distance(
        self,
        distance_m: float,
        angle_deg: float,
    ) -> Any:
        """Start one bounded displacement and return its task."""

    def stop(self) -> Any:
        """Request and confirm a chassis stop."""


class BoxControlRuntime(Protocol):
    """A fail-closed reader for coupled box-control evidence."""

    def read(self) -> Any:
        """Return the current reviewed box-control status."""


class RuntimeIdentityReader(Protocol):
    """Read a fresh receipt from the selected runtime composition."""

    def read(self) -> Any:
        """Return the runtime identity handshake receipt."""


@dataclass(frozen=True, slots=True)
class _Outcome:
    ok: bool
    code: Any
    message: str
    data: Any


@dataclass(slots=True)
class _ForceGateState:
    max_abs_force_n: tuple[float, float, float]
    max_abs_torque_nm: tuple[float, float, float]
    hold_s: float
    previous_sequence: int | float | None = None
    high_since_s: float | None = None


class LynrotControlAtomicAdapter:
    """Coordinate reviewed atomic capabilities without owning transport."""

    def __init__(
        self,
        *,
        profile: AtomicCapabilityProfile,
        robot: LynrotControlRuntime,
        chassis: ChassisRuntime,
        box_control: BoxControlRuntime | None = None,
        runtime_identity: RuntimeIdentityReader | None = None,
        now_s: Callable[[], float] = time.time,
        monotonic_s: Callable[[], float] = time.monotonic,
        sleep_s: Callable[[float], None] = time.sleep,
        poll_interval_s: float = 0.02,
        live_dispatch_enabled: bool = False,
    ) -> None:
        self._profile = profile
        self._robot = robot
        self._chassis = chassis
        self._box_control = box_control
        self._runtime_identity = runtime_identity
        self._now_s = now_s
        self._monotonic_s = monotonic_s
        self._sleep_s = sleep_s
        self._poll_interval_s = _positive(
            poll_interval_s, "poll_interval_s"
        )
        self._live_dispatch_enabled = live_dispatch_enabled
        self._profile_sha256 = atomic_profile_hash(profile)
        self._connected = False
        self._capability_mode = "idle"
        self._dispatch_lock = threading.Lock()
        self._stop_intent_lock = threading.Lock()
        self._stop_pending = threading.Event()
        self._stop_generation = 0
        self._dispatch_generation = 0
        self._stop_in_progress = 0
        self._stop_completion_unknown = False
        self._stop_uncertainty_permanent = False
        self._unsettled_stop_futures: list[Future[Any]] = []
        self._joint_sequences: dict[str, float | None] = {
            "left": None,
            "right": None,
        }
        self._box_control_sequence: float | None = None
        self._sample_sequences: dict[str, float | None] = {
            name: None
            for name in (
                "left_gripper",
                "right_gripper",
                "waist",
                "chassis",
            )
        }

    @property
    def capability_mode(self) -> str:
        """Return the coordinator's resource-safety mode."""

        if self._stop_completion_unknown and self._capability_mode != "closed":
            return "operator_review"
        return self._capability_mode

    @property
    def profile_sha256(self) -> str:
        """Return the hash binding the coordinator to its profile."""

        return self._profile_sha256

    def runtime_identity_reason(self) -> str | None:
        """Recheck the runtime receipt before an effectful dispatch."""

        return self._runtime_identity_reason()

    def connect(self) -> dict[str, Any]:
        """Preflight all reviewed subsystems without sending motion."""

        if self._capability_mode == "closed":
            return {"status": "closed", "capability_mode": "closed"}
        if self._stop_completion_unknown:
            return self._uncertain_stop_rejection()
        if self._capability_mode != "idle":
            return {
                "status": "rejected",
                "reason": "operator_review_open",
                "capability_mode": self._capability_mode,
            }
        if self._connected:
            return {
                "status": "rejected",
                "reason": "already_connected",
                "capability_mode": self._capability_mode,
            }
        if self._profile.mode == "live" and not self._live_dispatch_enabled:
            return {
                "status": "rejected",
                "reason": "live_dispatch_not_authorized",
                "capability_mode": self._capability_mode,
            }
        identity_reason = self._runtime_identity_reason()
        if identity_reason is not None:
            return {
                "status": "rejected",
                "reason": identity_reason,
                "capability_mode": self._capability_mode,
            }
        runtime_reason = self._runtime_api_reason()
        if runtime_reason is not None:
            return {
                "status": "rejected",
                "reason": runtime_reason,
                "capability_mode": self._capability_mode,
            }
        try:
            snapshot = self._snapshot()
        except Exception:
            return {
                "status": "rejected",
                "reason": "adapter_exception",
                "capability_mode": self._capability_mode,
            }
        reason = self._connection_reason(snapshot)
        if reason is not None:
            return {
                "status": "rejected",
                "reason": reason,
                "capability_mode": self._capability_mode,
            }
        self._connected = True
        return {
            "status": "ok",
            "mode": self._profile.mode,
            "profile_id": self._profile.profile_id,
            "profile_sha256": self._profile_sha256,
        }

    def acknowledge_operator_review(self) -> dict[str, Any]:
        """Record programmatic operator review acknowledgement."""

        if self._capability_mode == "closed":
            return {"status": "closed", "capability_mode": "closed"}
        if self._stop_completion_unknown:
            return self._uncertain_stop_rejection()
        if self._capability_mode != "operator_review":
            return {
                "status": "rejected",
                "reason": "operator_review_not_open",
                "capability_mode": self._capability_mode,
            }
        self._capability_mode = "operator_review_acknowledged"
        return {
            "status": "ok",
            "capability_mode": self._capability_mode,
        }

    def resume_from_acknowledged_review(self) -> dict[str, Any]:
        """Resume idle operation only after acknowledgement and fresh state."""

        if self._capability_mode == "closed":
            return {"status": "closed", "capability_mode": "closed"}
        if self._stop_completion_unknown:
            return self._uncertain_stop_rejection()
        if self._capability_mode != "operator_review_acknowledged":
            return {
                "status": "rejected",
                "reason": "operator_review_not_acknowledged",
                "capability_mode": self._capability_mode,
            }
        try:
            snapshot = self._snapshot()
        except Exception:
            return {
                "status": "rejected",
                "reason": "adapter_exception",
                "capability_mode": self._capability_mode,
            }
        reason = self._connection_reason(snapshot)
        if reason is not None:
            return {
                "status": "rejected",
                "reason": reason,
                "capability_mode": self._capability_mode,
            }
        with self._stop_intent_lock, self._dispatch_lock:
            if (
                self._stop_pending.is_set()
                or self._capability_mode != "operator_review_acknowledged"
            ):
                return self._interrupted_dispatch()
            self._capability_mode = "idle"
        return {"status": "ok", "capability_mode": "idle"}

    def read_capability_state(self) -> dict[str, Any]:
        """Read and normalize all subsystem state without dispatch."""

        if self._capability_mode == "closed":
            return {"status": "closed", "capability_mode": "closed"}
        if not self._connected:
            return {
                "status": "rejected",
                "reason": "not_connected",
                "capability_mode": self._capability_mode,
            }
        try:
            snapshot = self._snapshot()
        except Exception:
            return {
                "status": "failed",
                "reason": "adapter_exception",
                "capability_mode": self._capability_mode,
            }
        return {
            "status": "ok",
            "capability_mode": self.capability_mode,
            "profile_id": self._profile.profile_id,
            "profile_sha256": self._profile_sha256,
            "robot_state": copy.deepcopy(snapshot["robot_state"]),
            "subsystems": copy.deepcopy(snapshot["subsystems"]),
        }

    def begin_perception(self) -> dict[str, Any]:
        """Enter the non-motion perception mode from idle only."""

        if self._capability_mode == "closed":
            return {"status": "closed", "capability_mode": "closed"}
        if self._stop_completion_unknown:
            return self._uncertain_stop_rejection()
        if self._capability_mode != "idle":
            return {
                "status": "rejected",
                "reason": "capability_busy",
                "capability_mode": self._capability_mode,
            }
        self._capability_mode = "perception_running"
        return {
            "status": "ok",
            "capability_mode": self._capability_mode,
        }

    def end_perception(self) -> dict[str, Any]:
        """Leave perception mode; transport failures do not stop hardware."""

        if self._capability_mode == "perception_running":
            self._capability_mode = "idle"
        return {
            "status": "ok",
            "capability_mode": self._capability_mode,
        }

    def move_chassis(self, profile_name: str) -> dict[str, Any]:
        """Execute one reviewed chassis navigation or displacement."""

        if self._capability_mode == "closed":
            return {"status": "closed", "capability_mode": "closed"}
        guarded = self._guard(
            "move_chassis", {"profile_name": profile_name}
        )
        if guarded is not None:
            return guarded
        profile = self._profile.chassis_profiles[profile_name]
        guard = self._profile.carry_guards.get(profile_name)
        if profile.carries_box and guard is None:
            return self._review_failure("carry_guard_missing")
        if not profile.carries_box and guard is not None:
            return self._review_failure("carry_guard_unexpected")
        if guard is not None:
            carry_reason = self._carry_guard_reason(guard)
            if carry_reason is not None:
                return self._review_failure(carry_reason)
        identity_rejection = self._dispatch_identity_rejection()
        if identity_rejection is not None:
            return identity_rejection
        try:
            with self._dispatch_lock:
                if (
                    self._capability_mode != "idle"
                    or self._stop_completion_unknown
                    or self._stop_pending.is_set()
                ):
                    return self._interrupted_dispatch()
                self._capability_mode = "chassis_moving"
                self._dispatch_generation += 1
                if profile.kind == "nav_goal":
                    assert profile.goal is not None
                    submission = _outcome(self._chassis.navigate(profile.goal))
                    request = {"kind": "nav_goal", "goal": profile.goal}
                else:
                    assert profile.distance_m is not None
                    assert profile.angle_deg is not None
                    submission = _outcome(
                        self._chassis.move_distance(
                            profile.distance_m,
                            profile.angle_deg,
                        )
                    )
                    request = {
                        "kind": "move_distance",
                        "distance_m": profile.distance_m,
                        "angle_deg": profile.angle_deg,
                    }
            task = _task_from_submission(submission)
            if guard is None:
                waited = _wait_for_task(task, profile.timeout_s)
            else:
                carry_wait_reason, waited = self._wait_for_carry_chassis(
                    task,
                    profile.timeout_s,
                    guard,
                )
                if carry_wait_reason is not None:
                    return self._motion_failure(
                        carry_wait_reason,
                        stop_grippers=True,
                    )
            if not submission.ok or task is None or not waited.ok:
                return self._motion_failure(
                    "chassis_timeout" if _is_timeout(waited) else "chassis_failed",
                    stop_grippers=guard is not None,
                )
            final = _outcome(self._chassis.read())
            reason = self._chassis_final_reason(final)
            if reason is not None:
                return self._motion_failure(
                    reason,
                    stop_grippers=guard is not None,
                )
            if guard is not None:
                carry_reason = self._carry_guard_reason(guard)
                if carry_reason is not None:
                    return self._motion_failure(
                        carry_reason,
                        stop_grippers=True,
                    )
        except Exception:
            return self._motion_failure(
                "adapter_exception",
                stop_grippers=guard is not None,
            )
        if not self._finish_motion("chassis_moving"):
            return self._interrupted_dispatch()
        return {
            "status": "ok",
            "profile_name": profile_name,
            "request": request,
        }

    def move_waist(self, profile_name: str) -> dict[str, Any]:
        """Move the reviewed waist axis and confirm feedback tolerance."""

        if self._capability_mode == "closed":
            return {"status": "closed", "capability_mode": "closed"}
        guarded = self._guard("move_waist", {"profile_name": profile_name})
        if guarded is not None:
            return guarded
        profile = self._profile.waist_profiles[profile_name]
        identity_rejection = self._dispatch_identity_rejection()
        if identity_rejection is not None:
            return identity_rejection
        try:
            with self._dispatch_lock:
                if (
                    self._capability_mode != "idle"
                    or self._stop_completion_unknown
                    or self._stop_pending.is_set()
                ):
                    return self._interrupted_dispatch()
                self._capability_mode = "waist_moving"
                self._dispatch_generation += 1
                submission = _outcome(
                    self._robot.waist.lift.move(profile.position_mm)
                )
            task = _task_from_submission(submission)
            waited = _wait_for_task(task, profile.timeout_s)
            if not submission.ok or task is None or not waited.ok:
                return self._motion_failure(
                    "waist_timeout" if _is_timeout(waited) else "waist_failed"
                )
            deadline = self._monotonic_s() + profile.timeout_s
            while True:
                feedback = _outcome(self._robot.waist.lift.read())
                value = _sample_value(feedback, "waist")
                reason = _sample_reason(
                    feedback,
                    self._now_s(),
                    self._freshness_max_age_s("waist"),
                    "waist",
                )
                if reason is not None or not self._advance_sample_sequence(
                    "waist",
                    _mapping(feedback.data, "waist"),
                ):
                    return self._motion_failure(reason or "waist_stale")
                assert value is not None
                if abs(value - profile.position_mm) <= profile.tolerance_mm:
                    break
                if self._monotonic_s() >= deadline:
                    return self._motion_failure("waist_timeout")
                self._sleep_s(self._poll_interval_s)
        except Exception:
            return self._motion_failure("adapter_exception")
        if not self._finish_motion("waist_moving"):
            return self._interrupted_dispatch()
        return {
            "status": "ok",
            "profile_name": profile_name,
            "position_mm": profile.position_mm,
            "feedback_mm": value,
        }

    def move_dual_arms(self, profile_name: str) -> dict[str, Any]:
        """Submit one exact paired bounded joint path to both arms."""

        if self._capability_mode == "closed":
            return {"status": "closed", "capability_mode": "closed"}
        guarded = self._guard(
            "move_dual_arms", {"profile_name": profile_name}
        )
        if guarded is not None:
            return guarded
        with self._dispatch_lock:
            if (
                self._capability_mode != "idle"
                or self._stop_completion_unknown
                or self._stop_pending.is_set()
            ):
                return self._interrupted_dispatch()
            self._capability_mode = "dual_arms_moving"
            self._dispatch_generation += 1
        profile = self._profile.dual_arm_profiles[profile_name]
        arms = {
            "left": self._robot.arms.left,
            "right": self._robot.arms.right,
        }
        paths = {
            "left": profile.left_joints_deg,
            "right": profile.right_joints_deg,
        }
        gates = {
            side: _ForceGateState(
                max_abs_force_n=self._profile.force_gates.max_abs_force_n,
                max_abs_torque_nm=self._profile.force_gates.max_abs_torque_nm,
                hold_s=self._profile.force_gates.hold_s,
            )
            for side in arms
        }
        try:
            force_reason, force_side = self._check_forces(
                gates,
                preflight=True,
            )
            if force_reason is not None:
                return self._motion_failure(
                    force_reason, force_side=force_side
                )
            start_reason = self._arm_start_reason(arms, paths, profile)
            if start_reason is not None:
                return self._review_failure(start_reason)
            identity_rejection = self._dispatch_identity_rejection()
            if identity_rejection is not None:
                return identity_rejection

            tasks: dict[str, Any] = {}
            submissions: dict[str, _Outcome] = {}
            with self._dispatch_lock:
                if (
                    self._capability_mode != "dual_arms_moving"
                    or self._stop_completion_unknown
                    or self._stop_pending.is_set()
                ):
                    return self._interrupted_dispatch()
                for side, arm in arms.items():
                    if (
                        self._stop_completion_unknown
                        or self._stop_pending.is_set()
                        or self._capability_mode != "dual_arms_moving"
                    ):
                        return self._interrupted_dispatch()
                    submissions[side] = _outcome(
                        arm.motion.movejoint([list(point) for point in paths[side]])
                    )
                    tasks[side] = _task_from_submission(submissions[side])
            if any(
                not submissions[side].ok or tasks[side] is None
                for side in arms
            ):
                return self._motion_failure("arm_submission_failed")
            if any(not hasattr(tasks[side], "poll") for side in arms):
                return self._motion_failure("arm_submission_failed")

            deadline = self._monotonic_s() + profile.timeout_s
            while True:
                force_reason, force_side = self._check_forces(gates)
                if force_reason is not None:
                    return self._motion_failure(
                        force_reason, force_side=force_side
                    )

                finished: dict[str, bool] = {}
                for side, task in tasks.items():
                    polled = _outcome(task.poll())
                    status = _mapping(polled.data, "arm task poll")
                    state = status.get("state")
                    if state == "SUCCEEDED":
                        result = status.get("result")
                        final = (
                            _outcome(result)
                            if result is not None
                            else _Outcome(False, None, "missing result", None)
                        )
                        if not final.ok:
                            return self._motion_failure("arm_task_failed")
                        finished[side] = True
                    elif state in {"FAILED", "CANCELLED"}:
                        return self._motion_failure("arm_task_failed")
                    elif state in {
                        "RUNNING",
                        "PREPARING",
                        "RESUMING",
                        "PAUSING",
                        "CANCELLING",
                    }:
                        finished[side] = False
                    else:
                        return self._motion_failure("arm_task_state_invalid")

                if all(finished.values()):
                    break
                if self._monotonic_s() >= deadline:
                    return self._motion_failure("arm_task_timeout")
                self._sleep_s(self._poll_interval_s)

            joints: dict[str, list[float]] = {}
            for side, arm in arms.items():
                feedback = _outcome(arm.reading.get_joints())
                values = _joint_values(
                    feedback,
                    self._profile.joint_axes,
                )
                reason = _sample_reason(
                    feedback,
                    self._now_s(),
                    self._freshness_max_age_s(f"{side}_arm_joints"),
                    f"{side}_arm_joints",
                )
                sequence = _joint_sequence(feedback)
                if (
                    values is None
                    or reason is not None
                    or not self._advance_joint_sequence(side, sequence)
                ):
                    return self._motion_failure("arm_arrival_mismatch")
                target = paths[side][-1]
                if any(
                    abs(actual - wanted) > profile.tolerance_deg
                    for actual, wanted in zip(values, target)
                ):
                    return self._motion_failure("arm_arrival_mismatch")
                state_reason = self._arm_idle_reason(arm)
                if state_reason is not None:
                    return self._motion_failure(state_reason)
                joints[side] = values
        except Exception:
            return self._motion_failure("adapter_exception")
        if not self._finish_motion("dual_arms_moving"):
            return self._interrupted_dispatch()
        return {
            "status": "ok",
            "profile_name": profile_name,
            "final_joints_deg": joints,
        }

    def set_dual_grippers(self, command: str) -> dict[str, Any]:
        """Command both grippers and require fresh paired agreement."""

        if self._capability_mode == "closed":
            return {"status": "closed", "capability_mode": "closed"}
        guarded = self._guard("set_dual_grippers", {"command": command})
        if guarded is not None:
            return guarded
        profile = self._profile.gripper_profiles[command]
        grippers = {
            "left": self._robot.grippers.left,
            "right": self._robot.grippers.right,
        }
        identity_rejection = self._dispatch_identity_rejection()
        if identity_rejection is not None:
            return identity_rejection
        try:
            with self._dispatch_lock:
                if (
                    self._capability_mode != "idle"
                    or self._stop_completion_unknown
                    or self._stop_pending.is_set()
                ):
                    return self._interrupted_dispatch()
                self._capability_mode = "grippers_moving"
                self._dispatch_generation += 1
                submissions = {}
                for side, gripper in grippers.items():
                    if (
                        self._stop_completion_unknown
                        or self._stop_pending.is_set()
                        or self._capability_mode != "grippers_moving"
                    ):
                        return self._interrupted_dispatch()
                    submissions[side] = _outcome(
                        gripper.move(profile.position_rad)
                    )
            tasks = {
                side: _task_from_submission(submission)
                for side, submission in submissions.items()
            }
            if any(
                not submissions[side].ok or tasks[side] is None
                for side in grippers
            ):
                return self._gripper_failure("gripper_submission_failed")

            for side, task in tasks.items():
                waited = _wait_for_task(task, profile.timeout_s)
                if not waited.ok:
                    return self._gripper_failure(
                        "gripper_timeout"
                        if _is_timeout(waited)
                        else "gripper_failed"
                    )

            deadline = self._monotonic_s() + profile.timeout_s
            while True:
                values: dict[str, float] = {}
                for side, gripper in grippers.items():
                    feedback = _outcome(gripper.read())
                    value = _sample_value(feedback, f"{side}_gripper")
                    reason = _sample_reason(
                        feedback,
                        self._now_s(),
                        self._freshness_max_age_s(f"{side}_gripper"),
                        f"{side}_gripper",
                    )
                    if value is None or reason is not None or not (
                        self._advance_sample_sequence(
                            f"{side}_gripper",
                            _mapping(feedback.data, f"{side}_gripper"),
                        )
                    ):
                        return self._gripper_failure(
                            reason or "gripper_invalid"
                        )
                    values[side] = value
                in_tolerance = {
                    side: abs(value - profile.position_rad)
                    <= profile.tolerance_rad
                    for side, value in values.items()
                }
                if all(in_tolerance.values()):
                    break
                if not any(in_tolerance.values()):
                    if self._monotonic_s() >= deadline:
                        return self._gripper_failure("gripper_timeout")
                else:
                    return self._gripper_failure("gripper_disagreement")
                self._sleep_s(self._poll_interval_s)
        except Exception:
            return self._gripper_failure("adapter_exception")
        if not self._finish_motion("grippers_moving"):
            return self._interrupted_dispatch()
        return {
            "status": "ok",
            "command": command,
            "positions_rad": values,
        }

    def stop_all(self) -> dict[str, Any]:
        """Request confirmed stops for every actuator with a reviewed stop API."""

        if self._capability_mode == "closed":
            return {"status": "closed", "capability_mode": "closed"}
        if not self._connected:
            return {
                "status": "rejected",
                "reason": "not_connected",
                "capability_mode": self._capability_mode,
            }
        with self._stop_intent_lock:
            if self._capability_mode == "closed":
                return {"status": "closed", "capability_mode": "closed"}
            self._stop_generation += 1
            stop_generation = self._stop_generation
            self._stop_in_progress += 1
            self._stop_pending.set()
        try:
            return self._stop_all_started(stop_generation)
        finally:
            with self._stop_intent_lock:
                self._stop_in_progress -= 1

    def _stop_all_started(self, stop_generation: int) -> dict[str, Any]:
        """Run a registered stop attempt and retain its uncertainty."""

        if not self._dispatch_lock.acquire(timeout=self._profile.budgets["stop_s"]):
            # A blocked synchronous submit cannot be cancelled here. No stop
            # request has been issued, so leave recovery permanently locked out.
            self._stop_completion_unknown = True
            self._stop_uncertainty_permanent = True
            self._capability_mode = "operator_review"
            return {
                "status": "failed",
                "reason": "stop_dispatch_blocked",
                "capability_mode": "operator_review",
                "stop_requested": False,
            }
        try:
            if self._capability_mode in {
                "chassis_moving",
                "waist_moving",
                "dual_arms_moving",
                "grippers_moving",
                "stopping",
            }:
                self._stop_completion_unknown = True
                self._stop_uncertainty_permanent = True
            self._capability_mode = "stopping"
        finally:
            self._dispatch_lock.release()
        include_grippers = self._profile.gripper_runtime is not None
        try:
            stop = self._request_stop(include_grippers=include_grippers)
        except Exception:
            self._stop_completion_unknown = True
            self._capability_mode = "operator_review"
            return {
                "status": "failed",
                "reason": "adapter_exception",
                "capability_mode": "operator_review",
            }
        self._capability_mode = "operator_review"
        result: dict[str, Any] = {
            "capability_mode": "operator_review",
            "gripper_stop_requested": include_grippers,
            "stop_timeout_s": self._profile.budgets["stop_s"],
            "subsystems": stop,
        }
        confirmed = all(item["confirmed"] for item in stop.values())
        if not confirmed:
            self._stop_completion_unknown = True
        if confirmed and not self._stop_uncertainty_permanent:
            self._stop_completion_unknown = False
            with self._stop_intent_lock:
                if self._stop_generation == stop_generation:
                    self._stop_pending.clear()
            return {**result, "status": "ok"}
        reason = (
            "stop_unconfirmed"
            if not confirmed
            else "stop_completion_unknown"
        )
        return {**result, "status": "failed", "reason": reason}

    def close(self) -> None:
        """Release coordinator ownership; this is not an emergency stop."""

        if any(not future.done() for future in self._unsettled_stop_futures):
            raise RuntimeError("cannot close: stop request still running")
        if self._dispatch_lock.locked():
            raise RuntimeError("cannot close: transport submission still running")
        if not self._connected:
            self._connected = False
            self._capability_mode = "closed"
            return
        if self._capability_mode in {
            "chassis_moving",
            "waist_moving",
            "dual_arms_moving",
            "grippers_moving",
            "stopping",
        }:
            raise RuntimeError(
                "cannot close while an atomic capability is active; request "
                "and confirm stop first"
            )
        stop_generation = self._stop_generation
        dispatch_generation = self._dispatch_generation
        try:
            _, confirmed = self._confirm_stability(
                self._monotonic_s() + self._profile.budgets["stop_s"]
            )
        except Exception as error:
            raise RuntimeError(
                "cannot close without confirmed subsystem stability"
            ) from error
        if not confirmed:
            raise RuntimeError(
                "cannot close without confirmed subsystem stability"
            )
        with self._stop_intent_lock:
            if not self._dispatch_lock.acquire(blocking=False):
                raise RuntimeError(
                    "cannot close: transport submission still running"
                )
            try:
                if (
                    self._stop_in_progress
                    or self._stop_generation != stop_generation
                    or self._dispatch_generation != dispatch_generation
                    or self._capability_mode in {
                        "chassis_moving",
                        "waist_moving",
                        "dual_arms_moving",
                        "grippers_moving",
                        "stopping",
                    }
                ):
                    raise RuntimeError(
                        "cannot close: atomic capability changed during "
                        "stability check"
                    )
                self._connected = False
                self._capability_mode = "closed"
            finally:
                self._dispatch_lock.release()

    def _runtime_api_reason(self) -> str | None:
        for side in ("left", "right"):
            arm = getattr(self._robot.arms, side)
            if not callable(getattr(arm.motion, "cancel", None)):
                return f"{side}_arm_stop_unavailable"
        if not callable(getattr(self._robot.waist.lift, "cancel", None)):
            return "waist_stop_unavailable"
        if self._profile.gripper_runtime is not None:
            for side in ("left", "right"):
                gripper = getattr(self._robot.grippers, side)
                if not callable(getattr(gripper, "cancel", None)):
                    return f"{side}_gripper_stop_unavailable"
        if self._profile.chassis_runtime is not None and not callable(
            getattr(self._chassis, "stop", None)
        ):
            return "chassis_stop_unavailable"
        return None

    def _runtime_identity_reason(self) -> str | None:
        if self._profile.version < 2:
            return None
        if self._runtime_identity is None:
            return "runtime_identity_missing"
        try:
            result = _outcome(self._runtime_identity.read())
            reason = _sample_reason(
                result,
                self._now_s(),
                self._profile.robot_state_max_age_s,
                "runtime_identity",
            )
            if reason is not None:
                return reason
            receipt = _mapping(result.data, "runtime_identity")
            chassis_receipt = _mapping(
                receipt.get("chassis"),
                "runtime_identity.chassis",
            )
            binding = self._profile.runtime_binding
            chassis = self._profile.chassis_runtime
            assert binding is not None
            assert chassis is not None
            expected = {
                "profile_sha256": self._profile_sha256,
                "site_id": binding.site_id,
                "robot_id": binding.robot_id,
                "lynrotcontrol_instance": binding.lynrotcontrol_instance,
                "lynrotcontrol_config_sha256": (
                    binding.lynrotcontrol_config_sha256
                ),
                "execution_host": binding.execution_host,
                "ros_domain_id": binding.ros_domain_id,
            }
            expected_chassis = {
                "kind": chassis.kind,
                "odom_topic": chassis.odom_topic,
                "cmd_vel_topic": chassis.cmd_vel_topic,
            }
            if (
                set(receipt) != {"valid", "timestamp", "chassis", *expected}
                or set(chassis_receipt) != set(expected_chassis)
                or any(
                    receipt.get(key) != value
                    for key, value in expected.items()
                )
                or any(
                    chassis_receipt.get(key) != value
                    for key, value in expected_chassis.items()
                )
            ):
                return "runtime_identity_mismatch"
            return None
        except Exception:
            return "runtime_identity_invalid"

    def _freshness_max_age_s(self, label: str) -> float:
        freshness = self._profile.subsystem_freshness
        if freshness is None:
            return self._profile.robot_state_max_age_s
        if label.startswith("left_arm_"):
            return (
                freshness.left_arm_force_s
                if label == "left_arm_force"
                else freshness.left_arm_state_s
            )
        if label.startswith("right_arm_"):
            return (
                freshness.right_arm_force_s
                if label == "right_arm_force"
                else freshness.right_arm_state_s
            )
        if label == "left_gripper":
            return freshness.left_gripper_s
        if label == "right_gripper":
            return freshness.right_gripper_s
        if label == "waist":
            return freshness.waist_s
        if label == "chassis":
            return freshness.chassis_s
        if label == "arm_state":
            return min(
                freshness.left_arm_state_s,
                freshness.right_arm_state_s,
            )
        return min(
            freshness.left_arm_force_s,
            freshness.right_arm_force_s,
        )

    def _advance_sample_sequence(
        self,
        label: str,
        data: Mapping[str, Any],
    ) -> bool:
        if self._profile.version < 2:
            return True
        sequence = _finite(data.get("sequence"))
        if sequence is None:
            return False
        previous = self._sample_sequences[label]
        if previous is not None and sequence <= previous:
            return False
        self._sample_sequences[label] = sequence
        return True

    def _peripheral_stability(
        self,
        label: str,
        result: _Outcome,
        data: Mapping[str, Any],
    ) -> bool:
        reason = _sample_reason(
            result,
            self._now_s(),
            self._freshness_max_age_s(label),
            label,
        )
        return (
            reason is None
            and data.get("moving") is False
            and self._advance_sample_sequence(label, data)
        )

    def _carry_guard_reason(self, guard: Any) -> str | None:
        arm_profile = self._profile.dual_arm_profiles[guard.arm_profile]
        close_profile = self._profile.gripper_profiles["close"]
        for side in ("left", "right"):
            arm = getattr(self._robot.arms, side)
            feedback = _outcome(arm.reading.get_joints())
            values = _joint_values(feedback, self._profile.joint_axes)
            sequence = _joint_sequence(feedback)
            reason = _sample_reason(
                feedback,
                self._now_s(),
                self._freshness_max_age_s(f"{side}_arm_joints"),
                f"{side}_arm_joints",
            )
            if values is None or reason is not None:
                return f"carry_{side}_arm_feedback_invalid"
            if not self._advance_joint_sequence(side, sequence):
                return f"carry_{side}_arm_feedback_stale"
            target = arm_profile.left_joints_deg[-1] if side == "left" else arm_profile.right_joints_deg[-1]
            if any(
                abs(actual - wanted) > arm_profile.tolerance_deg
                for actual, wanted in zip(values, target)
            ):
                return f"carry_{side}_arm_pose_mismatch"

            gripper = getattr(self._robot.grippers, side)
            result = _outcome(gripper.read())
            value = _sample_value(result, f"{side}_gripper")
            reason = _sample_reason(
                result,
                self._now_s(),
                self._freshness_max_age_s(f"{side}_gripper"),
                f"{side}_gripper",
            )
            sequence_advanced = self._advance_sample_sequence(
                f"{side}_gripper",
                result.data,
            )
            if value is None or reason is not None:
                return f"carry_{side}_gripper_feedback_invalid"
            if not sequence_advanced:
                return f"carry_{side}_gripper_feedback_stale"
            if abs(value - close_profile.position_rad) > close_profile.tolerance_rad:
                return f"carry_{side}_gripper_not_closed"

        box_reason = self._carry_box_reason(guard)
        if box_reason is not None:
            return box_reason

        gates = {
            side: _ForceGateState(
                max_abs_force_n=self._profile.force_gates.max_abs_force_n,
                max_abs_torque_nm=self._profile.force_gates.max_abs_torque_nm,
                hold_s=self._profile.force_gates.hold_s,
            )
            for side in ("left", "right")
        }
        force_reason, force_side = self._check_forces(gates, preflight=True)
        if force_reason is not None:
            return f"carry_{force_side}_force_{force_reason}"
        return None

    def _dispatch_identity_rejection(self) -> dict[str, Any] | None:
        reason = self._runtime_identity_reason()
        if reason is None:
            return None
        return {
            "status": "rejected",
            "reason": reason,
            "capability_mode": self._capability_mode,
        }

    def _carry_box_reason(self, guard: Any) -> str | None:
        if self._box_control is None:
            return "carry_box_control_missing"
        box_result = _outcome(self._box_control.read())
        box_max_age_s = (
            self._profile.perception_binding.pose_max_age_s
            if self._profile.perception_binding is not None
            else self._profile.robot_state_max_age_s
        )
        reason = _sample_reason(
            box_result,
            self._now_s(),
            box_max_age_s,
            "box_control",
        )
        if reason is not None:
            return reason
        try:
            box = _mapping(box_result.data, "box_control")
        except ValueError:
            return "box_control_invalid"
        box_sequence = _finite(box.get("sequence"))
        if box_sequence is None:
            return "box_control_invalid"
        if (
            self._box_control_sequence is not None
            and box_sequence <= self._box_control_sequence
        ):
            return "box_control_stale"
        self._box_control_sequence = box_sequence
        if box.get("status") != guard.box_control_status:
            return "carry_box_not_held"
        return None

    def _wait_for_carry_chassis(
        self,
        task: Any,
        timeout_s: float,
        guard: Any,
    ) -> tuple[str | None, _Outcome]:
        if not hasattr(task, "poll"):
            return "carry_chassis_task_not_monitorable", _Outcome(
                False,
                None,
                "task has no poll API",
                None,
            )
        gates = {
            side: _ForceGateState(
                max_abs_force_n=self._profile.force_gates.max_abs_force_n,
                max_abs_torque_nm=self._profile.force_gates.max_abs_torque_nm,
                hold_s=self._profile.force_gates.hold_s,
            )
            for side in ("left", "right")
        }
        deadline_s = self._monotonic_s() + timeout_s
        while True:
            carry_reason = self._carry_guard_reason(guard)
            if carry_reason is not None:
                return carry_reason, _Outcome(
                    False,
                    None,
                    carry_reason,
                    None,
                )
            force_reason, force_side = self._check_forces(gates)
            if force_reason is not None:
                return f"carry_{force_side}_force_{force_reason}", _Outcome(
                    False,
                    None,
                    force_reason,
                    None,
                )
            polled = _outcome(task.poll())
            try:
                status = _mapping(polled.data, "chassis task poll")
            except ValueError:
                return "carry_chassis_task_invalid", polled
            state = status.get("state")
            if state == "SUCCEEDED":
                result = status.get("result")
                final = (
                    _outcome(result)
                    if result is not None
                    else _Outcome(False, None, "missing result", None)
                )
                if not final.ok:
                    return "carry_chassis_task_failed", final
                return None, final
            if state not in {
                "RUNNING",
                "PREPARING",
                "RESUMING",
                "PAUSING",
                "CANCELLING",
            }:
                return "carry_chassis_task_state_invalid", polled
            if self._monotonic_s() >= deadline_s:
                return "carry_chassis_task_timeout", _Outcome(
                    False,
                    5001,
                    "timeout",
                    None,
                )
            self._sleep_s(self._poll_interval_s)

    def _snapshot(self) -> dict[str, Any]:
        snapshot: dict[str, Any] = {}
        reasons: list[str] = []
        arm_errors: dict[str, bool] = {}
        ages: list[float] = []

        for side in ("left", "right"):
            arm = getattr(self._robot.arms, side)
            state_result = _outcome(arm.reading.get_state())
            joints_result = _outcome(arm.reading.get_joints())
            force_result = _outcome(arm.reading.get_wrench_torque())
            state = _mapping(state_result.data, f"{side}_arm_state")
            joints = _mapping(joints_result.data, f"{side}_arm_joints")
            force = _mapping(force_result.data, f"{side}_arm_force")
            joint_sequence = _joint_sequence(joints_result)
            joint_sequence_advanced = self._advance_joint_sequence(
                side,
                joint_sequence,
            )
            state_reason = _sample_reason(
                state_result,
                self._now_s(),
                self._freshness_max_age_s(f"{side}_arm_state"),
                f"{side}_arm_state",
                require_valid=False,
            )
            joints_reason = _sample_reason(
                joints_result,
                self._now_s(),
                self._freshness_max_age_s(f"{side}_arm_joints"),
                f"{side}_arm_joints",
            )
            force_reason = _force_reason(
                force_result,
                force,
                self._now_s(),
                self._freshness_max_age_s(f"{side}_arm_force"),
            )
            reason = state_reason or joints_reason or force_reason
            if joint_sequence is None:
                reason = reason or f"{side}_arm_joints_invalid"
            elif not joint_sequence_advanced:
                reason = reason or f"{side}_arm_joints_stale"
            if reason is not None:
                reasons.append(reason)
            if _fault(state.get("fault")):
                arm_errors[side] = True
                reasons.append(f"{side}_arm_error")
            else:
                arm_errors[side] = False
            ages.append(
                _sample_age(
                    self._now_s(),
                    state,
                    joints,
                    force,
                )
            )
            snapshot[side] = {
                "state": state,
                "joints": joints,
                "force": force,
            }

        for side in ("left", "right"):
            gripper = getattr(self._robot.grippers, side)
            result = _outcome(gripper.read())
            mapping = _mapping(result.data, f"{side}_gripper")
            reason = _sample_reason(
                result,
                self._now_s(),
                self._freshness_max_age_s(f"{side}_gripper"),
                f"{side}_gripper",
            )
            if reason is not None:
                reasons.append(reason)
            ages.append(_sample_age(self._now_s(), mapping))
            snapshot[f"{side}_gripper"] = mapping

        waist_result = _outcome(self._robot.waist.lift.read())
        waist = _mapping(waist_result.data, "waist")
        waist_reason = _sample_reason(
            waist_result,
            self._now_s(),
            self._freshness_max_age_s("waist"),
            "waist",
        )
        if waist_reason is not None:
            reasons.append(waist_reason)
        elif not self._advance_sample_sequence("waist", waist):
            reasons.append("waist_stale")
        ages.append(_sample_age(self._now_s(), waist))
        snapshot["waist"] = waist

        chassis_result = _outcome(self._chassis.read())
        chassis = _mapping(chassis_result.data, "chassis")
        chassis_reason = _sample_reason(
            chassis_result,
            self._now_s(),
            self._freshness_max_age_s("chassis"),
            "chassis",
        )
        if chassis_reason is not None:
            reasons.append(chassis_reason)
        elif not self._advance_sample_sequence("chassis", chassis):
            reasons.append("chassis_stale")
        ages.append(_sample_age(self._now_s(), chassis))
        snapshot["chassis"] = chassis

        healthy = not reasons
        robot_state = {
            "age_s": max(ages, default=math.inf),
            "healthy": healthy,
            "mode": self._profile.mode,
            "left_arm_error": arm_errors["left"],
            "right_arm_error": arm_errors["right"],
        }
        return {
            "robot_state": robot_state,
            "subsystems": {
                "left_arm": snapshot["left"],
                "right_arm": snapshot["right"],
                "left_gripper": snapshot["left_gripper"],
                "right_gripper": snapshot["right_gripper"],
                "waist": snapshot["waist"],
                "chassis": snapshot["chassis"],
                "left_force": snapshot["left"]["force"],
                "right_force": snapshot["right"]["force"],
            },
            "reason": None if not reasons else reasons[0],
        }

    def _guard(
        self,
        tool: str,
        arguments: Mapping[str, Any],
    ) -> dict[str, Any] | None:
        if self._stop_completion_unknown:
            return self._uncertain_stop_rejection()
        if not self._connected:
            return {
                "status": "rejected",
                "reason": "not_connected",
                "capability_mode": self._capability_mode,
            }
        try:
            snapshot = self._snapshot()
        except Exception:
            self._capability_mode = "operator_review"
            return {
                "status": "rejected",
                "reason": "adapter_exception",
                "capability_mode": self._capability_mode,
            }
        rejection = atomic_call_rejection(
            tool=tool,
            arguments=arguments,
            profile=self._profile,
            capability_mode=self._capability_mode,
            robot_state=snapshot["robot_state"],
        )
        if rejection is None:
            rejection = self._busy_reason(snapshot)
        if rejection is not None:
            return {
                "status": "rejected",
                "reason": rejection,
                "capability_mode": self._capability_mode,
            }
        return None

    def _uncertain_stop_rejection(self) -> dict[str, Any]:
        return {
            "status": "rejected",
            "reason": "stop_completion_unknown",
            "capability_mode": "operator_review",
        }

    def _interrupted_dispatch(self) -> dict[str, Any]:
        return {
            "status": "rejected",
            "reason": "interrupted_by_stop",
            "capability_mode": self.capability_mode,
        }

    def _finish_motion(self, mode: str) -> bool:
        with self._dispatch_lock:
            if self._capability_mode != mode or self._stop_completion_unknown:
                return False
            self._capability_mode = "idle"
            return True

    def _check_forces(
        self,
        gates: Mapping[str, _ForceGateState],
        *,
        preflight: bool = False,
    ) -> tuple[str | None, str | None]:
        for side, gate in gates.items():
            arm = getattr(self._robot.arms, side)
            result = _outcome(arm.reading.get_wrench_torque())
            data = _mapping(result.data, f"{side}_arm_force")
            reason = _force_gate_update(
                gate,
                result,
                data,
                self._now_s(),
                self._freshness_max_age_s(f"{side}_arm_force"),
                self._monotonic_s(),
                preflight,
            )
            if reason is not None:
                return reason, side
        return None, None

    def _request_stop(
        self,
        *,
        include_grippers: bool = False,
    ) -> dict[str, dict[str, Any]]:
        requests = {
            "left_arm": self._robot.arms.left.motion.cancel,
            "right_arm": self._robot.arms.right.motion.cancel,
            "waist": self._robot.waist.lift.cancel,
            "chassis": self._chassis.stop,
        }
        if include_grippers:
            requests.update(
                {
                    "left_gripper": self._robot.grippers.left.cancel,
                    "right_gripper": self._robot.grippers.right.cancel,
                }
            )
        result: dict[str, dict[str, Any]] = {}
        deadline_s = self._monotonic_s() + self._profile.budgets["stop_s"]
        executor = ThreadPoolExecutor(
            max_workers=len(requests),
            thread_name_prefix="atomic-stop",
        )
        futures: dict[str, Future[Any]] = {}
        try:
            for name, request in requests.items():
                futures[name] = executor.submit(request)
            for name, future in futures.items():
                try:
                    remaining_s = deadline_s - self._monotonic_s()
                    outcome = _outcome(
                        future.result(timeout=max(0.0, remaining_s))
                    )
                    request_confirmed = _cancel_confirmed(outcome)
                    result[name] = {
                        "requested": True,
                        "request_confirmed": request_confirmed,
                        "stability_confirmed": False,
                        "confirmed": False,
                        "code": outcome.code,
                        "message": outcome.message,
                    }
                except FutureTimeoutError:
                    self._stop_completion_unknown = True
                    self._stop_uncertainty_permanent = True
                    self._unsettled_stop_futures.append(future)
                    result[name] = {
                        "requested": True,
                        "request_confirmed": False,
                        "stability_confirmed": False,
                        "confirmed": False,
                        "code": 5001,
                        "message": "stop request budget expired",
                    }
                except Exception as error:
                    result[name] = {
                        "requested": True,
                        "request_confirmed": False,
                        "stability_confirmed": False,
                        "confirmed": False,
                        "code": None,
                        "message": str(error),
                    }
        except BaseException:
            self._stop_completion_unknown = True
            self._unsettled_stop_futures.extend(futures.values())
            raise
        finally:
            executor.shutdown(wait=False, cancel_futures=True)

        stability, stable = self._confirm_stability(deadline_s)
        for name, item in result.items():
            item["stability_confirmed"] = stability.get(name, False)
            item["confirmed"] = (
                item["request_confirmed"] and item["stability_confirmed"]
            )
        return result

    def _stop_confirmation_requirements(self) -> tuple[int, float]:
        samples = 1
        settle_s = 0.0
        chassis = self._profile.chassis_runtime
        if chassis is not None:
            samples = max(samples, chassis.zero_twist_confirm_samples)
            settle_s = max(settle_s, chassis.stop_settle_timeout_s)
        gripper = self._profile.gripper_runtime
        if gripper is not None:
            samples = max(samples, gripper.zero_motion_confirm_samples)
        return samples, settle_s

    def _confirm_stability(
        self,
        deadline_s: float,
    ) -> tuple[dict[str, bool], bool]:
        required_samples, settle_s = self._stop_confirmation_requirements()
        settle_deadline = min(
            deadline_s,
            self._monotonic_s() + settle_s,
        )
        stable_samples = 0
        stability: dict[str, bool] = {}
        while True:
            stability = self._stop_stability()
            all_stable = bool(stability) and all(stability.values())
            if self._monotonic_s() < settle_deadline:
                stable_samples = 0
            else:
                stable_samples = stable_samples + 1 if all_stable else 0
            if stable_samples >= required_samples:
                return stability, True
            if self._monotonic_s() >= deadline_s:
                return stability, False
            remaining_s = deadline_s - self._monotonic_s()
            self._sleep_s(min(self._poll_interval_s, remaining_s))

    def _stop_stability(self) -> dict[str, bool]:
        stability: dict[str, bool] = {}
        for side in ("left", "right"):
            arm = getattr(self._robot.arms, side)
            try:
                state_result = _outcome(arm.reading.get_state())
                state = _mapping(state_result.data, f"{side}_arm_state")
                reason = _sample_reason(
                    state_result,
                    self._now_s(),
                    self._freshness_max_age_s(f"{side}_arm_state"),
                    f"{side}_arm_state",
                    require_valid=False,
                )
                stability[f"{side}_arm"] = (
                    reason is None
                    and _idle_state_reason(state, f"{side}_arm") is None
                )
            except Exception:
                stability[f"{side}_arm"] = False
            gripper = getattr(self._robot.grippers, side)
            try:
                result = _outcome(gripper.read())
                data = _mapping(result.data, f"{side}_gripper")
                stability[f"{side}_gripper"] = self._peripheral_stability(
                    f"{side}_gripper",
                    result,
                    data,
                )
            except Exception:
                stability[f"{side}_gripper"] = False
        try:
            result = _outcome(self._robot.waist.lift.read())
            data = _mapping(result.data, "waist")
            stability["waist"] = self._peripheral_stability(
                "waist",
                result,
                data,
            )
        except Exception:
            stability["waist"] = False
        try:
            result = _outcome(self._chassis.read())
            data = _mapping(result.data, "chassis")
            stability["chassis"] = self._peripheral_stability(
                "chassis",
                result,
                data,
            )
        except Exception:
            stability["chassis"] = False
        return stability

    def _motion_failure(
        self,
        reason: str,
        *,
        stop_grippers: bool = False,
        **details: Any,
    ) -> dict[str, Any]:
        self._capability_mode = "stopping"
        try:
            stop = self._request_stop(include_grippers=stop_grippers)
        except Exception:
            self._stop_completion_unknown = True
            stop = {}
        self._capability_mode = "operator_review"
        confirmed = bool(stop) and all(item["confirmed"] for item in stop.values())
        if not confirmed:
            self._stop_completion_unknown = True
        return {
            "status": "failed",
            "reason": reason,
            "capability_mode": "operator_review",
            "stop_status": "confirmed" if confirmed else "unconfirmed",
            "gripper_stop_requested": stop_grippers,
            "stop_subsystems": stop,
            **details,
        }

    def _gripper_failure(self, reason: str) -> dict[str, Any]:
        runtime = self._profile.gripper_runtime
        if runtime is None:
            self._capability_mode = "operator_review"
            return {
                "status": "failed",
                "reason": reason,
                "capability_mode": "operator_review",
                "gripper_stop_requested": False,
            }
        self._capability_mode = "stopping"
        try:
            stop = self._stop_grippers()
        except Exception:
            self._stop_completion_unknown = True
            stop = {
                side: {
                    "requested": True,
                    "request_confirmed": False,
                    "stability_confirmed": False,
                    "confirmed": False,
                    "code": None,
                    "message": "gripper stop exception",
                }
                for side in ("left_gripper", "right_gripper")
            }
        self._capability_mode = "operator_review"
        confirmed = all(item["confirmed"] for item in stop.values())
        if not confirmed:
            self._stop_completion_unknown = True
        return {
            "status": "failed",
            "reason": reason,
            "capability_mode": "operator_review",
            "gripper_stop_requested": True,
            "gripper_stop_status": (
                "confirmed" if confirmed else "unconfirmed"
            ),
            "gripper_stop_subsystems": stop,
        }

    def _stop_grippers(self) -> dict[str, dict[str, Any]]:
        runtime = self._profile.gripper_runtime
        assert runtime is not None
        requests = {
            "left_gripper": self._robot.grippers.left.cancel,
            "right_gripper": self._robot.grippers.right.cancel,
        }
        result: dict[str, dict[str, Any]] = {}
        deadline_s = self._monotonic_s() + runtime.stop_timeout_s
        executor = ThreadPoolExecutor(
            max_workers=len(requests),
            thread_name_prefix="atomic-gripper-stop",
        )
        futures: dict[str, Future[Any]] = {}
        try:
            for name, request in requests.items():
                futures[name] = executor.submit(request)
            for name, future in futures.items():
                try:
                    remaining_s = deadline_s - self._monotonic_s()
                    outcome = _outcome(
                        future.result(timeout=max(0.0, remaining_s))
                    )
                    result[name] = {
                        "requested": True,
                        "request_confirmed": _cancel_confirmed(outcome),
                        "stability_confirmed": False,
                        "confirmed": False,
                        "code": outcome.code,
                        "message": outcome.message,
                    }
                except FutureTimeoutError:
                    self._stop_completion_unknown = True
                    self._unsettled_stop_futures.append(future)
                    result[name] = {
                        "requested": True,
                        "request_confirmed": False,
                        "stability_confirmed": False,
                        "confirmed": False,
                        "code": 5001,
                        "message": "gripper stop budget expired",
                    }
                except Exception as error:
                    result[name] = {
                        "requested": True,
                        "request_confirmed": False,
                        "stability_confirmed": False,
                        "confirmed": False,
                        "code": None,
                        "message": str(error),
                    }
        except BaseException:
            self._stop_completion_unknown = True
            self._unsettled_stop_futures.extend(futures.values())
            raise
        finally:
            executor.shutdown(wait=False, cancel_futures=True)

        stable_samples = 0
        while True:
            stable = True
            for side in ("left", "right"):
                gripper = getattr(self._robot.grippers, side)
                try:
                    feedback = _outcome(gripper.read())
                    data = _mapping(feedback.data, f"{side}_gripper")
                    stable = stable and self._peripheral_stability(
                        f"{side}_gripper",
                        feedback,
                        data,
                    )
                except Exception:
                    stable = False
            stable_samples = stable_samples + 1 if stable else 0
            for name in result:
                result[name]["stability_confirmed"] = stable
                result[name]["confirmed"] = (
                    result[name]["request_confirmed"] and stable
                )
            if (
                stable_samples >= runtime.zero_motion_confirm_samples
                and all(item["confirmed"] for item in result.values())
            ):
                return result
            if self._monotonic_s() >= deadline_s:
                return result
            remaining_s = deadline_s - self._monotonic_s()
            self._sleep_s(min(self._poll_interval_s, remaining_s))

    def _review_failure(self, reason: str) -> dict[str, Any]:
        self._capability_mode = "operator_review"
        return {
            "status": "failed",
            "reason": reason,
            "capability_mode": "operator_review",
            "stop_status": "not_required_before_dispatch",
        }

    def _connection_reason(
        self,
        snapshot: Mapping[str, Any],
    ) -> str | None:
        state = snapshot["robot_state"]
        if state["healthy"] is not True:
            return snapshot["reason"] or "robot_state_invalid"
        return self._busy_reason(snapshot)

    def _busy_reason(self, snapshot: Mapping[str, Any]) -> str | None:
        subsystems = snapshot["subsystems"]
        for side in ("left", "right"):
            state = subsystems[f"{side}_arm"]["state"]
            reason = _idle_state_reason(state, f"{side}_arm")
            if reason is not None:
                return reason
        for name in ("waist", "chassis"):
            item = subsystems[name]
            moving = item.get("moving")
            if moving is True:
                return f"{name}_busy"
            if moving is not False:
                return f"{name}_state_invalid"
        for side in ("left", "right"):
            moving = subsystems[f"{side}_gripper"].get("moving")
            if moving is True:
                return f"{side}_gripper_busy"
            if moving is not False:
                return f"{side}_gripper_state_invalid"
        return None

    def _arm_idle_reason(self, arm: Any) -> str | None:
        result = _outcome(arm.reading.get_state())
        state = _mapping(result.data, "arm state")
        reason = _sample_reason(
            result,
            self._now_s(),
            self._freshness_max_age_s("arm_state"),
            "arm_state",
            require_valid=False,
        )
        if reason is not None:
            return "arm_arrival_mismatch"
        return _idle_state_reason(state, "arm")

    def _arm_start_reason(
        self,
        arms: Mapping[str, Any],
        paths: Mapping[str, tuple[tuple[float, ...], ...]],
        profile: Any,
    ) -> str | None:
        for side, arm in arms.items():
            feedback = _outcome(arm.reading.get_joints())
            values = _joint_values(
                feedback,
                self._profile.joint_axes,
            )
            reason = _sample_reason(
                feedback,
                self._now_s(),
                self._freshness_max_age_s(f"{side}_arm_joints"),
                f"{side}_arm_joints",
            )
            if values is None or reason is not None:
                return f"{side}_arm_start_feedback_invalid"
            sequence = _joint_sequence(feedback)
            if sequence is None:
                return f"{side}_arm_start_feedback_invalid"
            if not self._advance_joint_sequence(side, sequence):
                return f"{side}_arm_start_feedback_stale"
            for axis, value in enumerate(values):
                lower, upper = self._profile.joint_limits_deg[axis]
                if value < lower or value > upper:
                    return f"{side}_arm_start_outside_limits"
            if any(
                abs(current - target) > profile.start_tolerance_deg
                for current, target in zip(values, paths[side][0])
            ):
                return f"{side}_arm_start_mismatch"
        return None

    def _chassis_final_reason(self, result: _Outcome) -> str | None:
        reason = _sample_reason(
            result,
            self._now_s(),
            self._freshness_max_age_s("chassis"),
            "chassis",
        )
        if reason is not None:
            return reason
        data = _mapping(result.data, "chassis")
        if data.get("moving") is not False:
            return "chassis_state_invalid"
        if not self._advance_sample_sequence("chassis", data):
            return "chassis_stale"
        return None

    def _advance_joint_sequence(
        self,
        side: str,
        sequence: float | None,
    ) -> bool:
        if sequence is None:
            return False
        previous = self._joint_sequences[side]
        if previous is not None and sequence <= previous:
            return False
        self._joint_sequences[side] = sequence
        return True


def _outcome(value: Any) -> _Outcome:
    if hasattr(value, "ok"):
        code = getattr(value, "code", None)
        message = getattr(value, "message", "")
        data = getattr(value, "data", None)
        return _Outcome(value.ok is True, code, str(message), data)
    if isinstance(value, Mapping) and "code" in value:
        code = value["code"]
        return _Outcome(
            code == 0,
            code,
            str(value.get("message", "")),
            value.get("data"),
        )
    return _Outcome(
        False,
        None,
        "runtime result has no recognized ok/code shape",
        None,
    )


def _task_from_submission(outcome: _Outcome) -> Any | None:
    if not outcome.ok or outcome.data is None:
        return None
    return outcome.data if hasattr(outcome.data, "wait") else None


def _wait_for_task(task: Any, timeout_s: float) -> _Outcome:
    if task is None or not hasattr(task, "wait"):
        return _Outcome(False, None, "task has no wait API", None)
    return _outcome(task.wait(timeout_s))


def _is_timeout(outcome: _Outcome) -> bool:
    return outcome.code == 5001 or "timeout" in outcome.message.lower()


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    return value


def _positive(value: float, label: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{label} must be finite and positive")
    return result


def _finite(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def _timestamp(value: Mapping[str, Any]) -> float | None:
    return _finite(value.get("timestamp"))


def _sample_age(now_s: float, *values: Mapping[str, Any]) -> float:
    stamps = [_timestamp(item) for item in values]
    if any(stamp is None for stamp in stamps):
        return math.inf
    assert all(stamp is not None for stamp in stamps)
    return max(now_s - stamp for stamp in stamps)


def _sample_reason(
    outcome: _Outcome,
    now_s: float,
    max_age_s: float,
    label: str,
    *,
    require_valid: bool = True,
) -> str | None:
    if not outcome.ok:
        return f"{label}_invalid"
    try:
        data = _mapping(outcome.data, label)
    except ValueError:
        return f"{label}_invalid"
    if require_valid and data.get("valid") is not True:
        return f"{label}_invalid"
    stamp = _timestamp(data)
    if stamp is None:
        return f"{label}_invalid"
    age_s = now_s - stamp
    if age_s < 0.0 or age_s > max_age_s:
        return f"{label}_stale"
    return None


def _sample_value(outcome: _Outcome, label: str) -> float | None:
    try:
        data = _mapping(outcome.data, label)
    except ValueError:
        return None
    return _finite(data.get("value"))


def _joint_values(
    outcome: _Outcome,
    axes: int,
) -> list[float] | None:
    try:
        data = _mapping(outcome.data, "arm joints")
    except ValueError:
        return None
    raw = data.get("values")
    if not isinstance(raw, list) or len(raw) != axes:
        return None
    values = [_finite(item) for item in raw]
    return values if all(item is not None for item in values) else None


def _joint_sequence(outcome: _Outcome) -> float | None:
    try:
        data = _mapping(outcome.data, "arm joints")
    except ValueError:
        return None
    return _finite(data.get("sequence"))


def _force_reason(
    outcome: _Outcome,
    data: Mapping[str, Any],
    now_s: float,
    max_age_s: float,
) -> str | None:
    del data
    return _sample_reason(outcome, now_s, max_age_s, "force")


def _force_gate_update(
    gate: _ForceGateState,
    outcome: _Outcome,
    data: Mapping[str, Any],
    now_s: float,
    max_age_s: float,
    monotonic_s: float,
    preflight: bool,
) -> str | None:
    reason = _sample_reason(outcome, now_s, max_age_s, "force_feedback")
    if reason is not None:
        return reason
    sequence = _finite(data.get("sequence"))
    if sequence is None:
        return "force_feedback_invalid"
    if (
        gate.previous_sequence is not None
        and sequence <= gate.previous_sequence
    ):
        return "force_feedback_stale"
    gate.previous_sequence = sequence
    values = data.get("values")
    if not isinstance(values, list) or len(values) != 6:
        return "force_feedback_invalid"
    numeric = [_finite(item) for item in values]
    if any(item is None for item in numeric):
        return "force_feedback_invalid"
    assert all(item is not None for item in numeric)
    exceeded = (
        any(
            abs(value) >= limit
            for value, limit in zip(numeric[:3], gate.max_abs_force_n)
        )
        or any(
            abs(value) >= limit
            for value, limit in zip(numeric[3:], gate.max_abs_torque_nm)
        )
    )
    if not exceeded:
        gate.high_since_s = None
        return None
    if preflight:
        return "force_threshold"
    if gate.high_since_s is None:
        gate.high_since_s = monotonic_s
        return None
    if monotonic_s - gate.high_since_s >= gate.hold_s:
        return "force_threshold"
    return None


def _fault(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return value != 0
    return value is not None


def _idle_state_reason(
    state: Mapping[str, Any],
    label: str,
) -> str | None:
    if state.get("moving") is not False:
        return f"{label}_busy"
    if state.get("paused") is not False:
        return f"{label}_busy"
    if state.get("drag_active") is not False:
        return f"{label}_busy"
    if state.get("drag_enabled") is not False:
        return f"{label}_busy"
    if state.get("task_state") != "IDLE":
        return f"{label}_busy"
    return None


def _cancel_confirmed(outcome: _Outcome) -> bool:
    if not outcome.ok:
        return False
    if outcome.data is None:
        return True
    if not isinstance(outcome.data, Mapping):
        return False
    data = outcome.data
    if data.get("stopped") is True or data.get("already_completed") is True:
        return True
    if data.get("stop_confirmed") is False:
        return False
    state = data.get("state")
    if state is not None:
        return state in {"SUCCEEDED", "CANCELLED", "IDLE"}
    return False
