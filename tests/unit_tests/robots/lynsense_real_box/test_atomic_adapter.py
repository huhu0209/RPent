from __future__ import annotations

import ast
import math
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
from threading import Event, Thread, current_thread
from types import SimpleNamespace
from typing import Any

import robots.lynsense_real_box.atomic_adapter as adapter_module
import pytest

from robots.lynsense_real_box.atomic_adapter import LynrotControlAtomicAdapter
from robots.lynsense_real_box.atomic_profile import (
    atomic_profile_hash,
    validate_atomic_capability_profile,
)
from tests.unit_tests.robots.lynsense_real_box.test_atomic_profile import (
    live_profile,
    valid_v2_profile,
    valid_atomic_profile,
)


class MutableClock:
    def __init__(self) -> None:
        self.now_s = 1000.0
        self.monotonic_s = 5000.0

    def now(self) -> float:
        return self.now_s

    def monotonic(self) -> float:
        return self.monotonic_s

    def sleep(self, duration_s: float) -> None:
        self.now_s += duration_s
        self.monotonic_s += duration_s


@dataclass
class FakeResult:
    code: int = 0
    message: str = ""
    data: Any = None
    error: dict[str, Any] | None = None

    @property
    def ok(self) -> bool:
        return self.code == 0

    def wire(self) -> dict[str, Any]:
        return {
            "code": self.code,
            "message": self.message,
            "data": self.data,
            "error": self.error,
        }


@dataclass
class FakeTask:
    polls_to_finish: int = 2
    final_result: FakeResult = field(default_factory=FakeResult)
    polls: int = 0
    on_poll: Any = None

    def poll(self) -> FakeResult:
        self.polls += 1
        state = (
            "SUCCEEDED"
            if self.polls >= self.polls_to_finish
            else "RUNNING"
        )
        result = self.final_result.wire() if state == "SUCCEEDED" else None
        if state == "SUCCEEDED" and self.on_poll is not None:
            self.on_poll()
        return FakeResult(
            data={"state": state, "result": result, "polls": self.polls}
        )

    def wait(self, timeout_s: float | None = None) -> FakeResult:
        del timeout_s
        self.polls = max(self.polls, self.polls_to_finish)
        return self.final_result


class FakeArm:
    def __init__(self, clock: MutableClock, name: str) -> None:
        self.clock = clock
        self.name = name
        self.current_joints = [0.0] * 6
        self.moving = False
        self.fault: int | bool = 0
        self.force_values = [0.0] * 6
        self.force_valid = True
        self.force_invalid_after = math.inf
        self.force_sequence = 0
        self.force_high_after = math.inf
        self.force_timestamp_offset = 0.0
        self.force_stale_after = math.inf
        self.force_fixed_sequence: int | None = None
        self.joint_sequence_value = 0
        self.joint_fixed_sequence: int | None = None
        self.fail_move = False
        self.joint_offset = 0.0
        self.joint_read_count = 0
        self.joint_offset_after_reads = math.inf
        self.move_calls: list[list[list[float]]] = []
        self.cancel_calls = 0
        self.cancel_result = FakeResult()
        self.cancel_keeps_moving = False
        self.task: FakeTask | None = None

        def get_state() -> FakeResult:
            return FakeResult(
                data={
                    "native_state": 1,
                    "moving": self.moving,
                    "fault": self.fault,
                    "task_state": "MOVING" if self.moving else "IDLE",
                    "paused": False,
                    "drag_enabled": False,
                    "drag_active": False,
                    "timestamp": self.clock.now(),
                }
            )

        def get_joints() -> FakeResult:
            self.joint_read_count += 1
            self.joint_sequence_value += 1
            joint_offset = (
                self.joint_offset
                if self.joint_read_count >= self.joint_offset_after_reads
                else 0.0
            )
            return FakeResult(
                data={
                    "values": [
                        value + joint_offset
                        for value in self.current_joints
                    ],
                    "unit": "deg",
                    "timestamp": self.clock.now(),
                    "sequence": (
                        self.joint_fixed_sequence
                        if self.joint_fixed_sequence is not None
                        else self.joint_sequence_value
                    ),
                    "valid": True,
                }
            )

        def get_wrench_torque() -> FakeResult:
            self.force_sequence += 1
            values = list(self.force_values)
            valid = self.force_valid
            timestamp_offset = self.force_timestamp_offset
            if self.force_sequence >= self.force_high_after:
                values[2] = 40.0
            if self.force_sequence >= self.force_invalid_after:
                valid = False
            if self.force_sequence >= self.force_stale_after:
                timestamp_offset = 10.0
            return FakeResult(
                data={
                    "values": values,
                    "timestamp": self.clock.now() - timestamp_offset,
                    "sequence": (
                        self.force_fixed_sequence
                        if self.force_fixed_sequence is not None
                        else self.force_sequence
                    ),
                    "valid": valid,
                    "source": "test",
                    "frame": "flange",
                }
            )

        def movejoint(targets: list[list[float]]) -> FakeResult:
            self.move_calls.append([list(point) for point in targets])
            if self.fail_move:
                return FakeResult(5001, "submission rejected")
            self.current_joints = list(targets[-1])
            self.moving = True
            self.task = FakeTask()
            self.task.polls_to_finish = 6
            self.task.on_poll = self._on_polled
            return FakeResult(data=self.task)

        def on_polled() -> None:
            if self.task is not None and self.task.polls >= self.task.polls_to_finish:
                self.moving = False

        self.reading = SimpleNamespace(
            get_state=get_state,
            get_joints=get_joints,
            get_wrench_torque=get_wrench_torque,
        )
        self.motion = SimpleNamespace(movejoint=movejoint, cancel=self.cancel)
        self._on_polled = on_polled

    def cancel(self) -> FakeResult:
        self.cancel_calls += 1
        if not self.cancel_keeps_moving:
            self.moving = False
        self.task = None
        return self.cancel_result


class FakePeripheral:
    def __init__(
        self,
        clock: MutableClock,
        *,
        value: float,
        unit: str,
    ) -> None:
        self.clock = clock
        self.value = value
        self.unit = unit
        self.valid = True
        self.moving = False
        self.move_updates_value = True
        self.move_calls: list[float] = []
        self.cancel_calls = 0
        self.cancel_result = FakeResult(data={"stopped": True})
        self.read_sequence_value = 0
        self.fixed_sequence: int | None = None

    def read(self) -> FakeResult:
        self.read_sequence_value += 1
        return FakeResult(
            data={
                "value": self.value,
                "unit": self.unit,
                "timestamp": self.clock.now(),
                "sequence": (
                    self.fixed_sequence
                    if self.fixed_sequence is not None
                    else self.read_sequence_value
                ),
                "valid": self.valid,
                "moving": self.moving,
            }
        )

    def move(self, value: float) -> FakeResult:
        self.move_calls.append(value)
        if self.move_updates_value:
            self.value = value
        self.moving = not self.move_updates_value
        return FakeResult(data=FakeTask(final_result=FakeResult()))

    def cancel(self) -> FakeResult:
        self.cancel_calls += 1
        self.moving = False
        return self.cancel_result


class FakeChassis:
    def __init__(self, clock: MutableClock) -> None:
        self.clock = clock
        self.moving = False
        self.valid = True
        self.navigate_calls: list[str] = []
        self.move_distance_calls: list[tuple[float, float]] = []
        self.stop_calls = 0
        self.read_sequence_value = 0
        self.fixed_sequence: int | None = None

    def read(self) -> FakeResult:
        self.read_sequence_value += 1
        return FakeResult(
            data={
                "moving": self.moving,
                "valid": self.valid,
                "timestamp": self.clock.now(),
                "mode": "IDLE" if not self.moving else "MOVING",
                "sequence": (
                    self.fixed_sequence
                    if self.fixed_sequence is not None
                    else self.read_sequence_value
                ),
            }
        )

    def navigate(self, goal: str) -> FakeResult:
        self.navigate_calls.append(goal)
        self.moving = False
        return FakeResult(data=FakeTask(final_result=FakeResult()))

    def move_distance(
        self,
        distance_m: float,
        angle_deg: float,
    ) -> FakeResult:
        self.move_distance_calls.append((distance_m, angle_deg))
        self.moving = False
        return FakeResult(data=FakeTask(final_result=FakeResult()))

    def stop(self) -> FakeResult:
        self.stop_calls += 1
        self.moving = False
        return FakeResult(data={"stopped": True})


class FakeRuntimeIdentity:
    def __init__(
        self,
        clock: MutableClock,
        profile: Any,
        *,
        changes: dict[str, Any] | None = None,
    ) -> None:
        self.clock = clock
        self.profile = profile
        self.changes = changes or {}

    def read(self) -> FakeResult:
        binding = self.profile.runtime_binding
        chassis = self.profile.chassis_runtime
        assert binding is not None
        assert chassis is not None
        receipt = {
            "valid": True,
            "timestamp": self.clock.now(),
            "profile_sha256": atomic_profile_hash(self.profile),
            "site_id": binding.site_id,
            "robot_id": binding.robot_id,
            "lynrotcontrol_instance": binding.lynrotcontrol_instance,
            "lynrotcontrol_config_sha256": (
                binding.lynrotcontrol_config_sha256
            ),
            "execution_host": binding.execution_host,
            "ros_domain_id": binding.ros_domain_id,
            "chassis": {
                "kind": chassis.kind,
                "odom_topic": chassis.odom_topic,
                "cmd_vel_topic": chassis.cmd_vel_topic,
            },
        }
        receipt.update(self.changes)
        return FakeResult(data=receipt)


class FakeBoxControl:
    def __init__(
        self,
        clock: MutableClock,
        *,
        status: str = "held",
        valid: bool = True,
        fixed_sequence: int | None = None,
        release_after_reads: int | None = None,
    ) -> None:
        self.clock = clock
        self.status = status
        self.valid = valid
        self.read_calls = 0
        self.fixed_sequence = fixed_sequence
        self.release_after_reads = release_after_reads

    def read(self) -> FakeResult:
        self.read_calls += 1
        if (
            self.release_after_reads is not None
            and self.read_calls >= self.release_after_reads
        ):
            self.status = "released"
        return FakeResult(
            data={
                "status": self.status,
                "valid": self.valid,
                "timestamp": self.clock.now(),
                "sequence": (
                    self.fixed_sequence
                    if self.fixed_sequence is not None
                    else self.read_calls
                ),
            }
        )


@dataclass
class FakeRobot:
    arms: SimpleNamespace
    grippers: SimpleNamespace
    waist: SimpleNamespace


def make_runtime(clock: MutableClock) -> tuple[FakeRobot, FakeChassis]:
    left = FakeArm(clock, "left")
    right = FakeArm(clock, "right")
    robot = FakeRobot(
        arms=SimpleNamespace(left=left, right=right),
        grippers=SimpleNamespace(
            left=FakePeripheral(clock, value=0.5, unit="rad"),
            right=FakePeripheral(clock, value=0.5, unit="rad"),
        ),
        waist=SimpleNamespace(
            lift=FakePeripheral(clock, value=200.0, unit="mm")
        ),
    )
    return robot, FakeChassis(clock)


def adapter(
    *,
    stop_budget_s: float | None = None,
    **changes: Any,
) -> tuple[LynrotControlAtomicAdapter, FakeRobot, FakeChassis, MutableClock]:
    clock = MutableClock()
    robot, chassis = make_runtime(clock)
    for target_name, value in changes.pop("runtime", {}).items():
        target = robot
        if target_name.startswith("chassis."):
            target = chassis
            target_name = target_name.removeprefix("chassis.")
        for part in target_name.split("."):
            target = getattr(target, part)
        if callable(value):
            value(target)
        else:
            setattr(target, target_name.rsplit(".", 1)[-1], value)

    profile_data = valid_atomic_profile()
    if stop_budget_s is not None:
        profile_data["budgets"]["stop_s"] = stop_budget_s
    item = LynrotControlAtomicAdapter(
        profile=validate_atomic_capability_profile(profile_data),
        robot=robot,
        chassis=chassis,
        now_s=clock.now,
        monotonic_s=clock.monotonic,
        sleep_s=clock.sleep,
        poll_interval_s=0.02,
    )
    assert item.connect()["status"] == "ok"
    return item, robot, chassis, clock


def v2_adapter(
    *,
    box_control: FakeBoxControl | None = None,
    connect: bool = True,
) -> tuple[
    LynrotControlAtomicAdapter,
    FakeRobot,
    FakeChassis,
    MutableClock,
    FakeBoxControl | None,
]:
    clock = MutableClock()
    robot, chassis = make_runtime(clock)
    profile = validate_atomic_capability_profile(valid_v2_profile())
    item = LynrotControlAtomicAdapter(
        profile=profile,
        robot=robot,
        chassis=chassis,
        box_control=box_control,
        runtime_identity=FakeRuntimeIdentity(clock, profile),
        now_s=clock.now,
        monotonic_s=clock.monotonic,
        sleep_s=clock.sleep,
        poll_interval_s=0.02,
    )
    if connect:
        assert item.connect()["status"] == "ok"
    return item, robot, chassis, clock, box_control


def finish_arm_tasks(robot: FakeRobot) -> None:
    robot.arms.left._on_polled()
    robot.arms.right._on_polled()


def test_connect_and_read_state_normalize_all_reviewed_subsystems():
    item, _, _, _ = adapter()

    result = item.read_capability_state()

    assert result["status"] == "ok"
    assert result["capability_mode"] == "idle"
    assert result["robot_state"]["healthy"] is True
    assert result["robot_state"]["left_arm_error"] is False
    assert result["robot_state"]["right_arm_error"] is False
    assert set(result["subsystems"]) == {
        "left_arm",
        "right_arm",
        "left_gripper",
        "right_gripper",
        "waist",
        "chassis",
        "left_force",
        "right_force",
    }


def test_module_has_no_transport_or_company_task_flow_imports():
    source = Path(
        "robots/lynsense_real_box/atomic_adapter.py"
    ).read_text(encoding="utf-8")
    tree = ast.parse(source)
    imports = {
        node.names[0].name.split(".")[0]
        for node in ast.walk(tree)
        if isinstance(node, ast.Import)
    }
    imports |= {
        (node.module or "").split(".")[0]
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom)
    }
    assert not imports & {
        "rclpy",
        "rosidl",
        "lynrotcontrol",
        "lynsense_pytrees",
        "socket",
    }


def test_live_profile_is_rejected_until_separately_enabled():
    clock = MutableClock()
    robot, chassis = make_runtime(clock)
    item = LynrotControlAtomicAdapter(
        profile=validate_atomic_capability_profile(live_profile()),
        robot=robot,
        chassis=chassis,
        now_s=clock.now,
        monotonic_s=clock.monotonic,
        sleep_s=clock.sleep,
    )

    assert item.connect() == {
        "status": "rejected",
        "reason": "live_dispatch_not_authorized",
        "capability_mode": "idle",
    }


def test_v2_runtime_identity_handshake_is_required_and_exact():
    clock = MutableClock()
    robot, chassis = make_runtime(clock)
    profile = validate_atomic_capability_profile(valid_v2_profile())

    missing = LynrotControlAtomicAdapter(
        profile=profile,
        robot=robot,
        chassis=chassis,
        now_s=clock.now,
        monotonic_s=clock.monotonic,
        sleep_s=clock.sleep,
    )
    assert missing.connect()["reason"] == "runtime_identity_missing"

    mismatched = LynrotControlAtomicAdapter(
        profile=profile,
        robot=robot,
        chassis=chassis,
        runtime_identity=FakeRuntimeIdentity(
            clock,
            profile,
            changes={"ros_domain_id": 42},
        ),
        now_s=clock.now,
        monotonic_s=clock.monotonic,
        sleep_s=clock.sleep,
    )
    assert mismatched.connect()["reason"] == "runtime_identity_mismatch"

    invalid = LynrotControlAtomicAdapter(
        profile=profile,
        robot=robot,
        chassis=chassis,
        runtime_identity=FakeRuntimeIdentity(
            clock,
            profile,
            changes={"valid": False},
        ),
        now_s=clock.now,
        monotonic_s=clock.monotonic,
        sleep_s=clock.sleep,
    )
    assert invalid.connect()["reason"] == "runtime_identity_invalid"


def test_v2_runtime_identity_is_rechecked_before_transport_dispatch():
    clock = MutableClock()
    robot, chassis = make_runtime(clock)
    profile = validate_atomic_capability_profile(valid_v2_profile())

    class ChangesAfterConnect(FakeRuntimeIdentity):
        reads = 0

        def read(self) -> FakeResult:
            self.reads += 1
            result = super().read()
            if self.reads >= 2:
                result.data["ros_domain_id"] = 42
            return result

    item = LynrotControlAtomicAdapter(
        profile=profile,
        robot=robot,
        chassis=chassis,
        runtime_identity=ChangesAfterConnect(clock, profile),
        now_s=clock.now,
        monotonic_s=clock.monotonic,
        sleep_s=clock.sleep,
    )
    assert item.connect()["status"] == "ok"

    result = item.move_waist("safe")

    assert result["status"] == "rejected"
    assert result["reason"] == "runtime_identity_mismatch"
    assert robot.waist.lift.move_calls == []


def test_dual_arms_submit_exact_pairs_and_wait_for_both_arrival():
    item, robot, _, _ = adapter()
    left_path = list(robot.arms.left.move_calls)
    right_path = list(robot.arms.right.move_calls)

    result = item.move_dual_arms("safe")
    finish_arm_tasks(robot)

    profile = validate_atomic_capability_profile(valid_atomic_profile())
    expected = profile.dual_arm_profiles["safe"]
    assert result["status"] == "ok"
    assert robot.arms.left.move_calls != left_path
    assert robot.arms.right.move_calls != right_path
    assert robot.arms.left.move_calls[-1] == [
        list(point) for point in expected.left_joints_deg
    ]
    assert robot.arms.right.move_calls[-1] == [
        list(point) for point in expected.right_joints_deg
    ]
    assert robot.arms.left.task is not None
    assert robot.arms.right.task is not None
    assert robot.arms.left.task.polls >= 3
    assert robot.arms.right.task.polls >= 3
    assert robot.arms.left.cancel_calls == 0
    assert robot.arms.right.cancel_calls == 0
    assert item.read_capability_state()["capability_mode"] == "idle"


def test_dual_arm_start_mismatch_rejects_before_any_submission():
    item, robot, _, _ = adapter()
    robot.arms.right.current_joints[0] = 10.0

    result = item.move_dual_arms("safe")

    assert result["status"] == "failed"
    assert result["reason"] == "right_arm_start_mismatch"
    assert result["stop_status"] == "not_required_before_dispatch"
    assert robot.arms.left.move_calls == []
    assert robot.arms.right.move_calls == []
    assert robot.arms.left.cancel_calls == 0
    assert robot.arms.right.cancel_calls == 0
    assert item.read_capability_state()["capability_mode"] == "operator_review"


def test_dual_arm_start_rejects_replayed_joint_feedback():
    item, robot, _, _ = adapter()
    robot.arms.right.joint_fixed_sequence = 2

    result = item.move_dual_arms("safe")

    assert result["status"] == "failed"
    assert result["reason"] == "right_arm_start_feedback_stale"
    assert robot.arms.left.move_calls == []
    assert robot.arms.right.move_calls == []
    assert item.read_capability_state()["capability_mode"] == "operator_review"


def test_joint_sequence_regression_cannot_reset_the_high_water_mark():
    item, robot, _, _ = adapter()
    assert item.read_capability_state()["status"] == "ok"
    high_water = robot.arms.right.joint_sequence_value
    robot.arms.right.joint_sequence_value = high_water - 2

    result = item.move_dual_arms("safe")

    assert result["status"] == "rejected"
    assert result["reason"] == "robot_unhealthy"
    assert item._joint_sequences["right"] == high_water
    assert robot.arms.left.move_calls == []
    assert robot.arms.right.move_calls == []


def test_final_joint_feedback_must_also_advance_sequence():
    item, robot, _, _ = adapter()
    original_move = robot.arms.right.motion.movejoint

    def replay_final_feedback(targets: list[list[float]]) -> FakeResult:
        robot.arms.right.joint_fixed_sequence = (
            robot.arms.right.joint_sequence_value
        )
        return original_move(targets)

    robot.arms.right.motion.movejoint = replay_final_feedback

    result = item.move_dual_arms("safe")

    assert result["status"] == "failed"
    assert result["reason"] == "arm_arrival_mismatch"
    assert len(robot.arms.right.move_calls) == 1
    assert robot.arms.right.cancel_calls == 1


def test_one_arm_submission_failure_still_submits_and_cancels_both():
    def fail_left(left: FakeArm) -> None:
        left.fail_move = True

    item, robot, _, _ = adapter(runtime={"arms.left": fail_left})

    result = item.move_dual_arms("safe")

    assert result["status"] == "failed"
    assert result["reason"] == "arm_submission_failed"
    assert len(robot.arms.left.move_calls) == 1
    assert len(robot.arms.right.move_calls) == 1
    assert robot.arms.left.cancel_calls == 1
    assert robot.arms.right.cancel_calls == 1
    assert item.read_capability_state()["capability_mode"] == "operator_review"


@pytest.mark.parametrize(
    ("failure_kind", "expected_reason"),
    [
        ("failed", "arm_task_failed"),
        ("timeout", "arm_task_timeout"),
    ],
)
def test_either_arm_terminal_failure_or_timeout_routes_to_stop(
    failure_kind: str,
    expected_reason: str,
):
    def fail_right(right: FakeArm) -> None:
        def movejoint(targets: list[list[float]]) -> FakeResult:
            right.move_calls.append([list(point) for point in targets])
            right.current_joints = list(targets[-1])
            right.moving = True
            right.task = FakeTask(
                polls_to_finish=2 if failure_kind == "failed" else 10000,
                final_result=FakeResult(5001, "path failed"),
            )
            right.task.on_poll = right._on_polled
            return FakeResult(data=right.task)

        right.motion = SimpleNamespace(movejoint=movejoint, cancel=right.cancel)

    item, robot, _, _ = adapter(runtime={"arms.right": fail_right})

    result = item.move_dual_arms("safe")

    assert result["status"] == "failed"
    assert result["reason"] == expected_reason
    assert robot.arms.left.cancel_calls == 1
    assert robot.arms.right.cancel_calls == 1
    assert item.read_capability_state()["capability_mode"] == "operator_review"


def test_force_threshold_cancels_both_arms():
    def arm_force_high_after_start(arm: FakeArm) -> None:
        arm.force_high_after = 2

    item, robot, _, _ = adapter(
        runtime={
            "arms.left": arm_force_high_after_start,
        }
    )
    robot.arms.left.task = None
    robot.arms.left.force_high_after = 2

    result = item.move_dual_arms("safe")

    assert result["status"] == "failed"
    assert result["reason"] == "force_threshold"
    assert result["force_side"] == "left"
    assert robot.arms.left.cancel_calls == 1
    assert robot.arms.right.cancel_calls == 1


def test_preflight_force_already_above_gate_rejects_before_submission():
    item, robot, _, _ = adapter()
    robot.arms.left.force_values = [0.0, 0.0, 40.0, 0.0, 0.0, 0.0]

    result = item.move_dual_arms("safe")

    assert result["status"] == "failed"
    assert result["reason"] == "force_threshold"
    assert robot.arms.left.move_calls == []
    assert robot.arms.right.move_calls == []
    assert item.read_capability_state()["capability_mode"] == "operator_review"


@pytest.mark.parametrize(
    ("invalid_kind", "expected_reason"),
    [
        ("invalid", "force_feedback_invalid"),
        ("stale", "force_feedback_stale"),
        ("not_new", "force_feedback_stale"),
    ],
)
def test_invalid_or_stale_force_feedback_routes_to_stop(
    invalid_kind: str,
    expected_reason: str,
):
    item, robot, _, clock = adapter()

    if invalid_kind == "invalid":
        robot.arms.right.force_invalid_after = 3
    elif invalid_kind == "stale":
        robot.arms.right.force_stale_after = 3
    else:
        robot.arms.right.force_fixed_sequence = 1

    result = item.move_dual_arms("safe")

    assert result["status"] == "failed"
    assert result["reason"] == expected_reason
    assert robot.arms.left.cancel_calls == 1
    assert robot.arms.right.cancel_calls == 1
    assert item.read_capability_state()["capability_mode"] == "operator_review"


def test_final_joint_disagreement_is_not_accepted_and_stops_both():
    item, robot, _, _ = adapter()

    result = item.move_dual_arms("safe")
    finish_arm_tasks(robot)
    robot.arms.right.joint_offset = 10.0
    robot.arms.right.joint_read_count = 0
    robot.arms.right.joint_offset_after_reads = 3
    final_result = item.move_dual_arms("safe")

    assert final_result["status"] == "failed"
    assert final_result["reason"] == "arm_arrival_mismatch"
    assert robot.arms.left.cancel_calls == 1
    assert robot.arms.right.cancel_calls == 1
    assert result["status"] == "ok"


def test_grippers_are_commanded_as_one_pair_with_fresh_feedback():
    item, robot, _, _ = adapter()

    result = item.set_dual_grippers("close")

    assert result["status"] == "ok"
    assert robot.grippers.left.move_calls == [0.1]
    assert robot.grippers.right.move_calls == [0.1]
    assert item.read_capability_state()["capability_mode"] == "idle"


def test_gripper_disagreement_fails_closed_without_inventing_gripper_stop():
    def misread_right(right: FakePeripheral) -> None:
        original_read = right.read

        def read() -> FakeResult:
            result = original_read()
            result.data["value"] = 0.5
            return result

        right.read = read

    item, robot, _, _ = adapter(runtime={"grippers.right": misread_right})

    result = item.set_dual_grippers("close")

    assert result["status"] == "failed"
    assert result["reason"] == "gripper_disagreement"
    assert robot.arms.left.cancel_calls == 0
    assert robot.arms.right.cancel_calls == 0
    assert item.read_capability_state()["capability_mode"] == "operator_review"


def test_waist_uses_tolerance_feedback_and_timeout_cancels():
    item, robot, _, _ = adapter()
    result = item.move_waist("safe")
    assert result["status"] == "ok"
    assert robot.waist.lift.move_calls == [200.0]

    robot.waist.lift.value = 240.0
    robot.waist.lift.move_updates_value = False
    timeout_result = item.move_waist("safe")
    assert timeout_result["status"] == "failed"
    assert timeout_result["reason"] == "waist_timeout"
    assert robot.waist.lift.cancel_calls == 1
    assert item.read_capability_state()["capability_mode"] == "operator_review"


@pytest.mark.parametrize(
    ("profile_name", "expected_calls"),
    [
        ("pickup", None),
        ("retreat", (-0.6, 0.0)),
    ],
)
def test_chassis_profiles_dispatch_only_reviewed_arguments(
    profile_name: str,
    expected_calls: tuple[float, float] | None,
):
    item, _, chassis, _ = adapter()

    result = item.move_chassis(profile_name)

    assert result["status"] == "ok"
    if expected_calls is None:
        assert chassis.navigate_calls == ["pickup"]
        assert chassis.move_distance_calls == []
    else:
        assert chassis.navigate_calls == []
        assert chassis.move_distance_calls == [expected_calls]


def test_v2_gripper_runtime_requires_stop_api_before_connection():
    clock = MutableClock()
    robot, chassis = make_runtime(clock)
    robot.grippers.right.cancel = None
    profile = validate_atomic_capability_profile(valid_v2_profile())
    item = LynrotControlAtomicAdapter(
        profile=profile,
        robot=robot,
        chassis=chassis,
        runtime_identity=FakeRuntimeIdentity(clock, profile),
        now_s=clock.now,
        monotonic_s=clock.monotonic,
        sleep_s=clock.sleep,
    )

    result = item.connect()

    assert result == {
        "status": "rejected",
        "reason": "right_gripper_stop_unavailable",
        "capability_mode": "idle",
    }


def test_carry_chassis_requires_box_control_evidence_before_dispatch():
    item, robot, chassis, _, _ = v2_adapter()
    assert item.move_dual_arms("safe")["status"] == "ok"
    assert item.set_dual_grippers("close")["status"] == "ok"

    result = item.move_chassis("retreat")

    assert result["status"] == "failed"
    assert result["reason"] == "carry_box_control_missing"
    assert chassis.move_distance_calls == []
    assert item.capability_mode == "operator_review"


def test_carry_chassis_requires_held_box_control_before_dispatch():
    clock = MutableClock()
    box = FakeBoxControl(clock, status="released")
    item, robot, chassis, _, box = v2_adapter(box_control=box)
    assert item.move_dual_arms("safe")["status"] == "ok"
    assert item.set_dual_grippers("close")["status"] == "ok"
    result = item.move_chassis("retreat")

    assert result["status"] == "failed"
    assert result["reason"] == "carry_box_not_held"
    assert box.read_calls > 0
    assert chassis.move_distance_calls == []


def test_carry_chassis_rejects_replayed_box_control_evidence():
    clock = MutableClock()
    box = FakeBoxControl(clock, fixed_sequence=1)
    item, robot, chassis, _, box = v2_adapter(box_control=box)
    assert item.move_dual_arms("safe")["status"] == "ok"
    assert item.set_dual_grippers("close")["status"] == "ok"
    item._box_control_sequence = 1

    result = item.move_chassis("retreat")

    assert result["status"] == "failed"
    assert result["reason"] == "box_control_stale"
    assert chassis.move_distance_calls == []


def test_carry_chassis_requires_both_grippers_closed_before_dispatch():
    clock = MutableClock()
    box = FakeBoxControl(clock)
    item, _, chassis, _, box = v2_adapter(box_control=box)
    assert item.move_dual_arms("safe")["status"] == "ok"

    result = item.move_chassis("retreat")

    assert result["status"] == "failed"
    assert result["reason"] == "carry_left_gripper_not_closed"
    assert box.read_calls == 0
    assert chassis.move_distance_calls == []


def test_reviewed_carry_state_allows_bounded_chassis_dispatch():
    clock = MutableClock()
    box = FakeBoxControl(clock)
    item, robot, chassis, _, box = v2_adapter(box_control=box)
    assert item.move_dual_arms("safe")["status"] == "ok"
    assert item.set_dual_grippers("close")["status"] == "ok"

    result = item.move_chassis("retreat")

    assert result["status"] == "ok"
    assert chassis.move_distance_calls == [(-0.6, 0.0)]
    assert box.read_calls >= 2
    assert item.capability_mode == "idle"


def test_carry_chassis_force_threshold_stops_dispatch():
    clock = MutableClock()
    box = FakeBoxControl(clock)
    item, robot, chassis, _, box = v2_adapter(box_control=box)
    assert item.move_dual_arms("safe")["status"] == "ok"
    assert item.set_dual_grippers("close")["status"] == "ok"

    def long_move(distance_m: float, angle_deg: float) -> FakeResult:
        chassis.move_distance_calls.append((distance_m, angle_deg))
        return FakeResult(data=FakeTask(polls_to_finish=100))

    chassis.move_distance = long_move
    robot.arms.left.force_high_after = robot.arms.left.force_sequence + 4

    result = item.move_chassis("retreat")

    assert result["status"] == "failed"
    assert result["reason"] == "carry_left_force_force_threshold"
    assert chassis.move_distance_calls == [(-0.6, 0.0)]
    assert chassis.stop_calls == 1
    assert robot.arms.left.cancel_calls == 1
    assert robot.arms.right.cancel_calls == 1
    assert robot.grippers.left.cancel_calls == 1
    assert robot.grippers.right.cancel_calls == 1
    assert result["gripper_stop_requested"] is True
    assert item.capability_mode == "operator_review"


def test_carry_chassis_monitors_box_control_during_motion():
    clock = MutableClock()
    box = FakeBoxControl(clock, release_after_reads=2)
    item, _, chassis, _, box = v2_adapter(box_control=box)
    assert item.move_dual_arms("safe")["status"] == "ok"
    assert item.set_dual_grippers("close")["status"] == "ok"

    task = FakeTask(polls_to_finish=100)

    def long_move(distance_m: float, angle_deg: float) -> FakeResult:
        chassis.move_distance_calls.append((distance_m, angle_deg))
        return FakeResult(data=task)

    chassis.move_distance = long_move

    result = item.move_chassis("retreat")

    assert result["status"] == "failed"
    assert result["reason"] == "carry_box_not_held"
    assert task.polls < task.polls_to_finish
    assert chassis.stop_calls == 1
    assert item.capability_mode == "operator_review"


def test_carry_chassis_monitors_grippers_during_motion():
    clock = MutableClock()
    box = FakeBoxControl(clock)
    item, robot, chassis, _, box = v2_adapter(box_control=box)
    assert item.move_dual_arms("safe")["status"] == "ok"
    assert item.set_dual_grippers("close")["status"] == "ok"

    left = robot.grippers.left
    original_read = left.read
    read_calls = 0

    def open_on_third_read() -> FakeResult:
        nonlocal read_calls
        read_calls += 1
        result = original_read()
        if read_calls >= 3:
            result.data["value"] = 0.5
        return result

    left.read = open_on_third_read
    task = FakeTask(polls_to_finish=100)

    def long_move(distance_m: float, angle_deg: float) -> FakeResult:
        chassis.move_distance_calls.append((distance_m, angle_deg))
        return FakeResult(data=task)

    chassis.move_distance = long_move

    result = item.move_chassis("retreat")

    assert result["status"] == "failed"
    assert result["reason"] == "carry_left_gripper_not_closed"
    assert read_calls >= 3
    assert task.polls < task.polls_to_finish
    assert chassis.stop_calls == 1
    assert item.capability_mode == "operator_review"


def test_stop_all_rejects_replayed_peripheral_feedback():
    item, robot, _, _, _ = v2_adapter()
    high_water = robot.waist.lift.read_sequence_value
    robot.waist.lift.fixed_sequence = high_water

    result = item.stop_all()

    assert result["status"] == "failed"
    assert result["reason"] == "stop_unconfirmed"
    assert result["subsystems"]["waist"]["stability_confirmed"] is False


def test_v2_stop_requires_multiple_fresh_stability_samples():
    item, robot, chassis, _, _ = v2_adapter()

    result = item.stop_all()

    assert result["status"] == "ok"
    assert result["gripper_stop_requested"] is True
    assert robot.grippers.left.cancel_calls == 1
    assert robot.grippers.right.cancel_calls == 1
    assert result["subsystems"]["left_gripper"]["confirmed"] is True
    assert result["subsystems"]["right_gripper"]["confirmed"] is True
    assert robot.waist.lift.read_sequence_value >= 2
    assert robot.grippers.left.read_sequence_value >= 2
    assert robot.grippers.right.read_sequence_value >= 2
    assert chassis.read_sequence_value >= 2


def test_v2_gripper_failure_uses_confirmed_gripper_stop():
    item, robot, _, _, _ = v2_adapter()
    right = robot.grippers.right
    original_read = right.read

    def misread_right() -> FakeResult:
        result = original_read()
        result.data["value"] = 0.5
        return result

    right.read = misread_right

    result = item.set_dual_grippers("close")

    assert result["status"] == "failed"
    assert result["reason"] == "gripper_disagreement"
    assert result["gripper_stop_requested"] is True
    assert result["gripper_stop_status"] == "confirmed"
    assert robot.grippers.left.cancel_calls == 1
    assert robot.grippers.right.cancel_calls == 1
    assert item.capability_mode == "operator_review"


def test_stop_all_is_separate_from_gripper_stop():
    item, robot, chassis, _ = adapter()
    robot.arms.left.moving = True
    robot.arms.right.moving = True
    robot.waist.lift.moving = True
    chassis.moving = True

    result = item.stop_all()

    assert result["status"] == "ok"
    assert result["gripper_stop_requested"] is False
    assert robot.arms.left.cancel_calls == 1
    assert robot.arms.right.cancel_calls == 1
    assert robot.waist.lift.cancel_calls == 1
    assert chassis.stop_calls == 1
    assert item.read_capability_state()["capability_mode"] == "operator_review"


def test_unknown_cancel_state_routes_to_operator_review_not_success():
    def unknown_cancel(arm: FakeArm) -> None:
        arm.cancel_result = FakeResult(data={"state": "UNKNOWN"})

    item, robot, _, _ = adapter(runtime={"arms.left": unknown_cancel})

    result = item.stop_all()

    assert result["status"] == "failed"
    assert result["reason"] == "stop_unconfirmed"
    assert robot.arms.left.cancel_calls == 1
    assert robot.arms.right.cancel_calls == 1
    assert item.read_capability_state()["capability_mode"] == "operator_review"


def test_timed_out_stop_cannot_resume_or_close_while_cancel_is_running():
    item, robot, _, _ = adapter()
    entered = Event()
    release = Event()
    completed = Event()
    original_cancel = robot.arms.left.motion.cancel

    def late_cancel() -> FakeResult:
        entered.set()
        try:
            assert release.wait(timeout=10)
            return original_cancel()
        finally:
            completed.set()

    robot.arms.left.motion.cancel = late_cancel
    try:
        result = item.stop_all()

        assert entered.is_set()
        assert result["status"] == "failed"
        assert result["reason"] == "stop_unconfirmed"
        assert item.acknowledge_operator_review()["reason"] == "stop_completion_unknown"
        assert (
            item.resume_from_acknowledged_review()["reason"]
            == "stop_completion_unknown"
        )
        assert item.move_waist("safe")["reason"] == "stop_completion_unknown"
        with pytest.raises(RuntimeError, match="stop request still running"):
            item.close()
    finally:
        release.set()
        assert completed.wait(timeout=5)

    assert item.acknowledge_operator_review()["reason"] == "stop_completion_unknown"
    item._capability_mode = "idle"  # Simulate the interrupted tool completing late.
    assert item.capability_mode == "operator_review"
    assert item.read_capability_state()["capability_mode"] == "operator_review"
    assert item.move_waist("safe")["reason"] == "stop_completion_unknown"
    assert item.stop_all()["reason"] == "stop_completion_unknown"
    item.close()
    assert item.capability_mode == "closed"


def test_timed_out_gripper_stop_cannot_resume_after_late_cancel():
    item, robot, _, _, _ = v2_adapter()
    release = Event()
    completed = Event()
    left = robot.grippers.left
    right = robot.grippers.right
    original_cancel = left.cancel
    original_read = right.read

    def late_cancel() -> FakeResult:
        try:
            assert release.wait(timeout=10)
            return original_cancel()
        finally:
            completed.set()

    def disagreement() -> FakeResult:
        result = original_read()
        result.data["value"] = 0.5
        return result

    left.cancel = late_cancel
    right.read = disagreement
    try:
        result = item.set_dual_grippers("close")
        assert result["status"] == "failed"
        assert result["gripper_stop_status"] == "unconfirmed"
        assert item.acknowledge_operator_review()["reason"] == "stop_completion_unknown"
        assert item.move_waist("safe")["reason"] == "stop_completion_unknown"
        with pytest.raises(RuntimeError, match="stop request still running"):
            item.close()
    finally:
        release.set()
        assert completed.wait(timeout=5)

    assert item.acknowledge_operator_review()["reason"] == "stop_completion_unknown"
    item.close()


def test_repeated_confirmed_stop_can_clear_nonpermanent_uncertainty():
    item, _, chassis, _, _ = v2_adapter()
    original_stop = chassis.stop

    def unconfirmed_stop() -> FakeResult:
        return FakeResult(1, "stop unavailable")

    chassis.stop = unconfirmed_stop
    failed = item.stop_all()
    assert failed["status"] == "failed"
    assert failed["reason"] == "stop_unconfirmed"
    assert item._stop_pending.is_set()

    chassis.stop = original_stop
    confirmed = item.stop_all()
    assert confirmed["status"] == "ok"
    assert not item._stop_pending.is_set()
    assert item.acknowledge_operator_review()["status"] == "ok"
    assert item.resume_from_acknowledged_review()["status"] == "ok"


def test_stop_exception_locks_out_recovery():
    item, _, _, _ = adapter()

    def unavailable_stop(**_kwargs: Any) -> None:
        raise OSError("stop transport unavailable")

    item._request_stop = unavailable_stop
    result = item.stop_all()

    assert result["status"] == "failed"
    assert result["reason"] == "adapter_exception"
    assert item.acknowledge_operator_review()["reason"] == "stop_completion_unknown"
    assert item.resume_from_acknowledged_review()["reason"] == "stop_completion_unknown"
    assert item.move_waist("safe")["reason"] == "stop_completion_unknown"


def test_motion_failure_stop_exception_locks_out_recovery():
    item, robot, _, _ = adapter()
    robot.arms.left.fail_move = True

    def unavailable_stop(**_kwargs: Any) -> None:
        raise OSError("stop transport unavailable")

    item._request_stop = unavailable_stop
    result = item.move_dual_arms("safe")

    assert result["status"] == "failed"
    assert result["reason"] == "arm_submission_failed"
    assert result["stop_status"] == "unconfirmed"
    assert item.acknowledge_operator_review()["reason"] == "stop_completion_unknown"
    assert item.move_waist("safe")["reason"] == "stop_completion_unknown"


def test_partial_stop_submission_tracks_in_flight_request(monkeypatch):
    item, robot, _, _ = adapter()
    entered = Event()
    release = Event()
    completed = Event()

    def blocking_cancel() -> FakeResult:
        entered.set()
        try:
            assert release.wait(timeout=5)
            return FakeResult()
        finally:
            completed.set()

    class FailingSubmit(ThreadPoolExecutor):
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, **kwargs)
            self.submitted = 0

        def submit(self, fn: Any, *args: Any, **kwargs: Any) -> Any:
            self.submitted += 1
            if self.submitted == 2:
                raise RuntimeError("second stop submission unavailable")
            future = super().submit(fn, *args, **kwargs)
            assert entered.wait(timeout=5)
            return future

    robot.arms.left.motion.cancel = blocking_cancel
    monkeypatch.setattr(adapter_module, "ThreadPoolExecutor", FailingSubmit)
    try:
        result = item.stop_all()
        assert result["status"] == "failed"
        assert item.acknowledge_operator_review()["reason"] == "stop_completion_unknown"
        with pytest.raises(RuntimeError, match="stop request still running"):
            item.close()
    finally:
        release.set()
        assert completed.wait(timeout=5)
    item.close()


@pytest.mark.parametrize(
    ("tool", "argument"),
    [
        ("move_chassis", "pickup"),
        ("move_waist", "safe"),
        ("move_dual_arms", "safe"),
        ("set_dual_grippers", "open"),
    ],
)
def test_stop_before_dispatch_prevents_late_motion(tool, argument):
    item, robot, chassis, _ = adapter()
    entered = Event()
    release = Event()
    result: list[dict[str, Any]] = []
    original_check = item._dispatch_identity_rejection

    def pause_before_dispatch():
        entered.set()
        assert release.wait(timeout=5)
        return original_check()

    item._dispatch_identity_rejection = pause_before_dispatch
    worker = Thread(target=lambda: result.append(getattr(item, tool)(argument)))
    worker.start()
    try:
        assert entered.wait(timeout=5)
        assert item.stop_all()["status"] in {"ok", "failed"}
    finally:
        release.set()
        worker.join(timeout=5)
    assert not worker.is_alive()
    assert result[0]["status"] != "ok"
    assert item.capability_mode == "operator_review"
    assert chassis.navigate_calls == []
    assert chassis.move_distance_calls == []
    assert robot.waist.lift.move_calls == []
    assert robot.arms.left.move_calls == []
    assert robot.arms.right.move_calls == []
    assert robot.grippers.left.move_calls == []
    assert robot.grippers.right.move_calls == []


@pytest.mark.parametrize(
    ("tool", "argument", "transport_path"),
    [
        ("move_chassis", "pickup", "chassis.navigate"),
        ("move_waist", "safe", "waist.lift.move"),
        ("move_dual_arms", "safe", "arms.left.motion.movejoint"),
        ("set_dual_grippers", "open", "grippers.left.move"),
    ],
)
def test_blocked_submit_returns_unconfirmed_stop_without_second_side(
    tool, argument, transport_path
):
    item, robot, chassis, _ = adapter(stop_budget_s=0.05)
    entered = Event()
    release = Event()
    results: list[dict[str, Any]] = []
    is_chassis = transport_path.startswith("chassis.")
    target: Any = chassis if is_chassis else robot
    parts = transport_path.removeprefix("chassis.").split(".")
    for part in parts[:-1]:
        target = getattr(target, part)
    method_name = parts[-1]
    original_submit = getattr(target, method_name)

    def blocked_submit(*args: Any) -> FakeResult:
        entered.set()
        assert release.wait(timeout=5)
        return original_submit(*args)

    setattr(target, method_name, blocked_submit)
    worker = Thread(target=lambda: results.append(getattr(item, tool)(argument)))
    worker.start()
    try:
        assert entered.wait(timeout=5)
        start = time.monotonic()
        stopped = item.stop_all()
        assert time.monotonic() - start < 1.0
        assert stopped["status"] == "failed"
        assert stopped["reason"] == "stop_dispatch_blocked"
        assert stopped["stop_requested"] is False
        assert item.capability_mode == "operator_review"
        assert item.acknowledge_operator_review()["reason"] == "stop_completion_unknown"
        assert item.move_waist("safe")["reason"] == "stop_completion_unknown"
        assert robot.arms.left.cancel_calls == robot.arms.right.cancel_calls == 0
        assert chassis.stop_calls == 0
        with pytest.raises(RuntimeError, match="transport submission still running"):
            item.close()
    finally:
        release.set()
        worker.join(timeout=5)

    assert not worker.is_alive()
    assert results[0]["status"] != "ok"
    assert item.capability_mode == "operator_review"
    assert robot.arms.right.move_calls == []
    assert robot.grippers.right.move_calls == []


def test_stop_during_motion_cannot_be_overwritten_by_late_success():
    item, robot, _, _ = adapter()
    entered = Event()
    release = Event()
    result: list[dict[str, Any]] = []
    original_move = robot.waist.lift.move

    def slow_move(position_mm: float) -> FakeResult:
        submitted = original_move(position_mm)
        task = submitted.data
        original_wait = task.wait

        def wait(timeout_s: float | None = None) -> FakeResult:
            entered.set()
            assert release.wait(timeout=5)
            return original_wait(timeout_s)

        task.wait = wait
        return submitted

    robot.waist.lift.move = slow_move
    worker = Thread(target=lambda: result.append(item.move_waist("safe")))
    worker.start()
    try:
        assert entered.wait(timeout=5)
        assert item.stop_all()["status"] in {"ok", "failed"}
    finally:
        release.set()
        worker.join(timeout=5)
    assert not worker.is_alive()
    assert result[0]["status"] != "ok"
    assert item.capability_mode == "operator_review"
    assert item.acknowledge_operator_review()["reason"] == "stop_completion_unknown"


def test_cancel_accepted_but_physical_motion_remains_unconfirmed():
    item, robot, _, chassis = adapter()
    robot.arms.left.moving = True
    robot.arms.left.cancel_keeps_moving = True
    robot.arms.right.moving = True
    robot.waist.lift.moving = True
    chassis.moving = True

    result = item.stop_all()

    assert result["status"] == "failed"
    assert result["reason"] == "stop_unconfirmed"
    assert result["subsystems"]["left_arm"]["request_confirmed"] is True
    assert result["subsystems"]["left_arm"]["stability_confirmed"] is False
    assert result["subsystems"]["left_arm"]["confirmed"] is False
    assert result["subsystems"]["right_arm"]["confirmed"] is True
    assert item.read_capability_state()["capability_mode"] == "operator_review"
    assert item.acknowledge_operator_review()["reason"] == "stop_completion_unknown"
    with pytest.raises(RuntimeError, match="confirmed subsystem stability"):
        item.close()

    robot.arms.left.moving = False
    item.close()
    assert item.capability_mode == "closed"


def test_connect_and_close_cannot_reset_or_bypass_operator_review():
    item, _, _, _ = adapter()
    stopped = item.stop_all()
    assert stopped["status"] == "ok"

    assert item.connect()["reason"] == "operator_review_open"

    acknowledged = item.acknowledge_operator_review()
    assert acknowledged["capability_mode"] == "operator_review_acknowledged"
    assert item.connect()["reason"] == "operator_review_open"

    resumed = item.resume_from_acknowledged_review()
    assert resumed == {"status": "ok", "capability_mode": "idle"}
    assert item.connect()["reason"] == "already_connected"

    item._capability_mode = "dual_arms_moving"
    with pytest.raises(RuntimeError, match="cannot close"):
        item.close()
    assert item.capability_mode == "dual_arms_moving"


def test_stop_during_resume_cannot_restore_idle():
    item, _, _, _ = adapter()
    assert item.stop_all()["status"] == "ok"
    assert item.acknowledge_operator_review()["status"] == "ok"
    entered = Event()
    release = Event()
    results: list[dict[str, Any]] = []
    original_snapshot = item._snapshot

    def paused_snapshot() -> dict[str, Any]:
        entered.set()
        assert release.wait(timeout=5)
        return original_snapshot()

    item._snapshot = paused_snapshot
    worker = Thread(target=lambda: results.append(item.resume_from_acknowledged_review()))
    worker.start()
    try:
        assert entered.wait(timeout=5)
        assert item.stop_all()["status"] == "ok"
    finally:
        release.set()
        worker.join(timeout=5)
    assert not worker.is_alive()
    assert results[0]["status"] == "rejected"
    assert item.capability_mode == "operator_review"


@pytest.mark.parametrize("operation", ["stop", "waist"])
def test_close_rejects_operation_started_during_stability_check(operation):
    item, robot, _, _ = adapter()
    checking = Event()
    release_check = Event()
    submitting = Event()
    release_submit = Event()
    close_errors: list[Exception] = []
    results: list[dict[str, Any]] = []
    original_confirm = item._confirm_stability

    def paused_confirm(deadline_s: float):
        if current_thread().name == "closing-adapter":
            checking.set()
            assert release_check.wait(timeout=5)
        return original_confirm(deadline_s)

    item._confirm_stability = paused_confirm
    if operation == "stop":
        original_submit = robot.arms.left.motion.cancel

        def blocked_submit() -> FakeResult:
            submitting.set()
            assert release_submit.wait(timeout=5)
            return original_submit()

        robot.arms.left.motion.cancel = blocked_submit
    else:
        original_submit = robot.waist.lift.move

        def blocked_submit(position: float) -> FakeResult:
            submitting.set()
            assert release_submit.wait(timeout=5)
            return original_submit(position)

        robot.waist.lift.move = blocked_submit

    def close_adapter() -> None:
        try:
            item.close()
        except Exception as error:
            close_errors.append(error)

    closer = Thread(target=close_adapter, name="closing-adapter")
    closer.start()
    operation_thread: Thread | None = None
    try:
        assert checking.wait(timeout=5)
        operation_thread = Thread(
            target=lambda: results.append(
                item.stop_all() if operation == "stop" else item.move_waist("safe")
            )
        )
        operation_thread.start()
        assert submitting.wait(timeout=5)
        release_check.set()
        closer.join(timeout=5)
        assert not closer.is_alive()
        assert len(close_errors) == 1
        assert isinstance(close_errors[0], RuntimeError)
        assert item.capability_mode != "closed"
    finally:
        release_check.set()
        release_submit.set()
        closer.join(timeout=5)
        if operation_thread is not None:
            operation_thread.join(timeout=5)

    assert operation_thread is not None and not operation_thread.is_alive()
    assert results
    assert item.capability_mode != "closed"


def test_adapter_enforces_capability_mode_and_close_without_side_effects():
    item, robot, _, _ = adapter()
    stopped = item.stop_all()
    assert stopped["status"] == "ok"

    busy = item.move_waist("safe")
    assert busy["status"] == "rejected"
    assert busy["reason"] == "operator_review_open"
    assert robot.waist.lift.move_calls == []
    unknown = item.move_chassis("invented")
    assert unknown == {
        "status": "rejected",
        "reason": "unknown_profile",
        "capability_mode": "operator_review",
    }

    item.close()
    closed = item.move_dual_arms("safe")
    assert closed == {
        "status": "closed",
        "capability_mode": "closed",
    }
