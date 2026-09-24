from __future__ import annotations

import ast
import math
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import pytest

from robots.lynsense_real_box.atomic_profile import (
    validate_atomic_capability_profile,
)
from robots.lynsense_real_box.live.chassis_runtime import (
    BoundedOdomCmdVelChassisRuntime,
    ChassisRuntimeError,
)
from tests.unit_tests.robots.lynsense_real_box.test_atomic_profile import (
    valid_v2_profile,
)


@dataclass
class FakeClock:
    now_s: float = 1000.0
    monotonic_s: float = 5000.0

    def now(self) -> float:
        return self.now_s

    def monotonic(self) -> float:
        return self.monotonic_s

    def sleep(self, seconds: float) -> None:
        self.now_s += seconds
        self.monotonic_s += seconds


@dataclass
class FakeVector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class FakeQuaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class FakePose:
    position: FakeVector3 = field(default_factory=FakeVector3)
    orientation: FakeQuaternion = field(default_factory=FakeQuaternion)


@dataclass
class FakePoseWithPose:
    pose: FakePose = field(default_factory=FakePose)


@dataclass
class FakeTwist:
    linear: FakeVector3 = field(default_factory=FakeVector3)
    angular: FakeVector3 = field(default_factory=FakeVector3)


@dataclass
class FakeOdomTwist:
    twist: FakeTwist = field(default_factory=FakeTwist)


@dataclass
class FakeOdometry:
    header: dict[str, Any] = field(default_factory=dict)
    pose: FakePoseWithPose = field(default_factory=FakePoseWithPose)
    twist: FakeOdomTwist = field(default_factory=FakeOdomTwist)


class FakePublisher:
    def __init__(self, node: FakeNode) -> None:
        self.node = node
        self.destroyed = False
        self.twists: list[FakeTwist] = []
        self.trace: list[tuple[float, float]] = []

    def publish(self, twist: FakeTwist) -> None:
        self.twists.append(twist)
        self.node.published_twists.append(twist)
        self.trace.append(
            (twist.linear.x, self.node.odom.pose.pose.position.x)
        )

    def destroy(self) -> None:
        self.destroyed = True


class FakeSubscription:
    def __init__(self) -> None:
        self.destroyed = False

    def destroy(self) -> None:
        self.destroyed = True


class FakeContext:
    def init(self, args: list[str]) -> None:
        self.init_args = args
        self.initialized = True


class FakeNode:
    def __init__(self, clock: FakeClock) -> None:
        self.clock = clock
        self.context: FakeContext | None = None
        self.subscriptions: list[tuple[Any, str]] = []
        self.subscription: FakeSubscription | None = None
        self.publishers: list[FakePublisher] = []
        self.published_twists: list[FakeTwist] = []
        self.destroyed = False
        self.pending: list[FakeOdometry] = []
        self.odom = FakeOdometry()
        self.feedback_enabled = True
        self.on_spin: Any = None

    def get_clock(self) -> FakeClock:
        return self.clock

    def create_subscription(
        self,
        message_type: Any,
        topic: str,
        callback: Any,
        qos_depth: int,
    ) -> FakeSubscription:
        del qos_depth
        self.subscriptions.append((message_type, topic))
        self.odom_callback = callback
        self.subscription = FakeSubscription()
        return self.subscription

    def create_publisher(
        self,
        message_type: Any,
        topic: str,
        qos_depth: int,
    ) -> FakePublisher:
        del message_type, qos_depth
        publisher = FakePublisher(self)
        self.publishers.append(publisher)
        return publisher

    def publish_odom(
        self,
        *,
        x_m: float = 0.0,
        yaw_rad: float = 0.0,
        linear_x: float = 0.0,
        angular_z: float = 0.0,
    ) -> None:
        self.odom = FakeOdometry(
            pose=FakePoseWithPose(
                FakePose(
                    position=FakeVector3(x=x_m),
                    orientation=FakeQuaternion(
                        z=math.sin(yaw_rad / 2.0),
                        w=math.cos(yaw_rad / 2.0),
                    ),
                )
            ),
            twist=FakeOdomTwist(
                twist=FakeTwist(
                    linear=FakeVector3(x=linear_x),
                    angular=FakeVector3(z=angular_z),
                )
            ),
        )
        self.pending.append(self.odom)

    def spin_once(self, timeout_sec: float) -> None:
        if timeout_sec > 0.0:
            self.clock.sleep(timeout_sec)
        if self.on_spin is not None:
            self.on_spin()
        if self.feedback_enabled and self.pending:
            self.odom_callback(self.pending.pop(0))

    def destroy_node(self) -> None:
        self.destroyed = True


class FakeRclpy:
    def __init__(self, node: FakeNode, default_context: FakeContext) -> None:
        self.node = node
        self.default_context = default_context
        self.Context = FakeContext
        self.created_nodes: list[FakeNode] = []
        self.shutdown_contexts: list[FakeContext] = []

    def create_node(self, name: str, *, context: FakeContext) -> FakeNode:
        self.node.context = context
        self.created_nodes.append(self.node)
        self.node.created_name = name
        return self.node

    def spin_once(self, node: FakeNode, timeout_sec: float) -> None:
        node.spin_once(timeout_sec)

    def shutdown(self, *, context: FakeContext) -> None:
        self.shutdown_contexts.append(context)


def runtime_profile():
    source = valid_v2_profile()
    return validate_atomic_capability_profile(source)


def make_runtime(node: FakeNode, rclpy: FakeRclpy):
    return BoundedOdomCmdVelChassisRuntime(
        runtime_profile(),
        rclpy,
        clock=node.clock,
        message_types=(FakeOdometry, FakeTwist),
    )


def fresh_runtime():
    clock = FakeClock()
    node = FakeNode(clock)
    node.publish_odom()
    rclpy = FakeRclpy(node, FakeContext())
    return make_runtime(node, rclpy), node, rclpy


def test_private_context_preflight_defers_cmd_vel_publisher():
    runtime, node, rclpy = fresh_runtime()
    private_context = node.context
    assert private_context is not None
    assert private_context.initialized
    assert private_context.init_args == []
    assert private_context is not rclpy.default_context
    assert node.subscriptions == [(FakeOdometry, "/odom")]
    assert node.publishers == []

    state = runtime.read()
    assert state.ok
    assert state.data == {
        "moving": False,
        "valid": True,
        "timestamp": node.clock.now_s,
        "mode": "IDLE",
        "sequence": 1,
    }
    assert node.published_twists == []


def test_preflight_without_fresh_odom_fails_closed_and_shuts_private_context():
    clock = FakeClock()
    node = FakeNode(clock)
    rclpy = FakeRclpy(node, FakeContext())
    with pytest.raises(ChassisRuntimeError, match="preflight timed out"):
        make_runtime(node, rclpy)
    assert node.publishers == []
    assert node.subscriptions[0][1] == "/odom"
    assert node.destroyed
    assert rclpy.shutdown_contexts == [node.context]


def test_construction_failure_after_context_init_shuts_private_context():
    clock = FakeClock()
    rclpy = FakeRclpy(FakeNode(clock), FakeContext())

    def fail_node_factory(_context: FakeContext) -> FakeNode:
        raise RuntimeError("node creation unavailable")

    with pytest.raises(RuntimeError, match="node creation unavailable"):
        BoundedOdomCmdVelChassisRuntime(
            runtime_profile(),
            rclpy,
            clock=clock,
            node_factory=fail_node_factory,
        )

    assert len(rclpy.shutdown_contexts) == 1
    assert rclpy.shutdown_contexts[0] is not rclpy.default_context


def test_move_distance_integrates_and_bounds_velocity_acceleration_and_rate():
    runtime, node, _ = fresh_runtime()
    period_s = 0.05

    def apply_command() -> None:
        if not node.published_twists:
            return
        twist = node.published_twists[-1]
        node.publish_odom(
            x_m=node.odom.pose.pose.position.x + twist.linear.x * period_s,
            yaw_rad=node.odom.pose.pose.orientation.z * 2.0
            + twist.angular.z * period_s,
            linear_x=twist.linear.x,
            angular_z=twist.angular.z,
        )

    node.on_spin = apply_command
    submission = runtime.move_distance(0.6, 0.0)
    assert submission.ok
    task = submission.data
    result = task.wait(9.0)
    assert result.ok, result.message
    assert result.data == {"state": "SUCCEEDED"}

    nonzero = [item for item in node.published_twists if item.linear.x != 0.0]
    assert nonzero
    assert all(
        abs(item.linear.x) <= 0.15 + 1e-12
        and abs(item.angular.z) <= 0.25 + 1e-12
        for item in nonzero
    )
    previous = 0.0
    for item in nonzero:
        assert abs(item.linear.x - previous) <= 0.1 * period_s + 1e-12
        previous = item.linear.x
    final = node.odom.pose.pose.position.x
    assert final == pytest.approx(0.6, abs=0.01)
    assert node.odom.twist.twist.linear.x == 0.0


def test_move_distance_rejects_a_value_outside_the_atomic_profile():
    runtime, node, _ = fresh_runtime()
    result = runtime.move_distance(1.1, 0.0)
    assert not result.ok
    assert result.data["reason"] == "displacement_outside_profile"
    assert node.publishers == []
    assert node.published_twists == []


def test_displacement_watchdog_publishes_zero_and_fails_without_retry():
    runtime, node, _ = fresh_runtime()

    def freeze_feedback() -> None:
        if node.published_twists:
            node.feedback_enabled = False

    node.on_spin = freeze_feedback
    task = runtime.move_distance(0.3, 0.0).data
    result = task.wait(2.0)
    assert not result.ok
    assert result.message == "odom_watchdog"
    assert node.published_twists[-1].linear.x == 0.0
    assert node.published_twists[-1].angular.z == 0.0


def test_stop_creates_publisher_and_requires_fresh_zero_twist_samples():
    runtime, node, _ = fresh_runtime()
    node.publish_odom(linear_x=0.2)
    assert runtime.read().data["moving"]
    node.on_spin = lambda: node.publish_odom(
        x_m=node.odom.pose.pose.position.x,
    )
    result = runtime.stop()
    assert result.ok
    assert result.data == {"stopped": True}
    assert len(node.publishers) == 1
    assert node.published_twists
    assert node.published_twists[0].linear.x == 0.0
    assert node.published_twists[0].angular.z == 0.0


def test_unconfirmed_stop_latches_displacement_closed():
    runtime, node, _ = fresh_runtime()
    node.publish_odom(linear_x=0.2)
    def keep_moving() -> None:
        node.publish_odom(linear_x=0.2)

    node.on_spin = keep_moving
    result = runtime.stop()
    assert not result.ok
    assert result.data == {"stopped": False}
    rejected = runtime.move_distance(0.1, 0.0)
    assert not rejected.ok
    assert rejected.data["reason"] == "stop_unconfirmed"

    node.on_spin = lambda: node.publish_odom(
        x_m=node.odom.pose.pose.position.x
    )
    retried_stop = runtime.stop()
    assert retried_stop.ok
    assert runtime.read().ok


def test_stale_odom_stop_still_publishes_zero_before_failing_confirmation():
    runtime, node, _ = fresh_runtime()
    node.feedback_enabled = False
    node.clock.now_s += 0.3

    result = runtime.stop()

    assert not result.ok
    assert result.message == "odom not fresh"
    assert len(node.publishers) == 1
    assert len(node.published_twists) == 1
    assert node.published_twists[0].linear.x == 0.0
    assert node.published_twists[0].angular.z == 0.0


def test_navigation_always_fails_closed_without_a_publisher():
    runtime, node, _ = fresh_runtime()
    result = runtime.navigate("pickup")
    assert not result.ok
    assert result.message == "navigation disabled"
    assert node.publishers == []
    assert node.published_twists == []


def test_close_is_bounded_repeatable_and_destroys_publisher():
    runtime, node, rclpy = fresh_runtime()
    node.on_spin = lambda: node.publish_odom(
        x_m=node.odom.pose.pose.position.x,
    )
    result = runtime.stop()
    assert result.ok
    publisher = node.publishers[0]
    assert node.subscription is not None
    subscription = node.subscription
    runtime.close()
    runtime.close()
    assert publisher.destroyed
    assert subscription.destroyed
    assert node.destroyed
    assert rclpy.shutdown_contexts == [node.context]
    with pytest.raises(ChassisRuntimeError, match="closed"):
        runtime.read()


def test_rclpy_spin_once_has_one_locked_entry_point():
    source = Path(
        "robots/lynsense_real_box/live/chassis_runtime.py"
    ).read_text(encoding="utf-8")
    tree = ast.parse(source)
    direct_calls = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        function = node.func
        if (
            isinstance(function, ast.Attribute)
            and function.attr == "spin_once"
            and isinstance(function.value, ast.Attribute)
            and function.value.attr == "_rclpy"
            and isinstance(function.value.value, ast.Name)
            and function.value.value.id == "self"
        ):
            direct_calls.append(node)

    assert len(direct_calls) == 1
    spin_method = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "_spin_once"
    )
    assert any(node is direct_calls[0] for node in ast.walk(spin_method))


def test_spin_once_calls_are_serialized():
    runtime, _, _ = fresh_runtime()
    active = 0
    maximum = 0
    tracker_lock = threading.Lock()

    def track_spin() -> None:
        nonlocal active, maximum
        with tracker_lock:
            active += 1
            maximum = max(maximum, active)
        threading.Event().wait(timeout=0.02)
        with tracker_lock:
            active -= 1

    runtime._node.on_spin = track_spin
    workers = [
        threading.Thread(target=lambda: runtime._spin_once(0.0))
        for _ in range(2)
    ]
    for worker in workers:
        worker.start()
    for worker in workers:
        worker.join(timeout=1.0)

    assert not any(worker.is_alive() for worker in workers)
    assert maximum == 1
