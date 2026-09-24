from __future__ import annotations

import json
import threading
from types import SimpleNamespace
from typing import Any

import pytest

from robots.lynsense_real_box.atomic_profile import (
    validate_atomic_capability_profile,
)
from robots.lynsense_real_box.live.perception_transport import LivePerceptionTransport
from robots.lynsense_real_box.perception_client import BoxPerceptionClient
from tests.unit_tests.robots.lynsense_real_box.test_atomic_profile import (
    live_profile,
)


class FakeClock:
    def __init__(self, now_s: float = 100.0) -> None:
        self.now_s = now_s

    def __call__(self) -> float:
        return self.now_s

    def advance(self, seconds: float) -> None:
        self.now_s += seconds


class FakeContext:
    def __init__(self, token: int) -> None:
        self.token = token


class FakeSubscription:
    def __init__(self, name: str, callback: Any) -> None:
        self.name = name
        self.callback = callback
        self.destroy_calls = 0

    def destroy(self) -> None:
        self.destroy_calls += 1


class FakeClient:
    def __init__(self, name: str) -> None:
        self.name = name
        self.requests: list[Any] = []
        self.destroy_calls = 0

    def call_async(self, request: Any) -> Any:
        self.requests.append(request)
        return SimpleNamespace(done=lambda: False, cancelled=lambda: False)

    def destroy(self) -> None:
        self.destroy_calls += 1


class FakeNode:
    def __init__(self, context: FakeContext, clock: FakeClock) -> None:
        self.context = context
        self.clock = clock
        self.calls: list[tuple[str, str]] = []
        self.subscriptions: list[FakeSubscription] = []
        self.clients: list[FakeClient] = []
        self.destroy_calls = 0

    def create_subscription(
        self,
        message_type: Any,
        topic: str,
        callback: Any,
        qos: int,
    ) -> FakeSubscription:
        assert qos == 10
        self.calls.append(("subscription", topic))
        subscription = FakeSubscription(topic, callback)
        self.subscriptions.append(subscription)
        return subscription

    def create_client(self, service_type: Any, service: str) -> FakeClient:
        self.calls.append(("client", service))
        client = FakeClient(service)
        self.clients.append(client)
        return client

    def create_publisher(self, *_: Any, **__: Any) -> None:
        raise AssertionError("box perception transport must not create a publisher")

    def destroy_node(self) -> None:
        self.destroy_calls += 1

    def deliver_pose(self, message: Any) -> None:
        self.subscriptions[0].callback(message)

    def deliver_status(self, message: Any) -> None:
        self.subscriptions[1].callback(message)


class FakeRclpy:
    class SignalHandlerOptions:
        NO = "no"

    def __init__(self, node: FakeNode, clock: FakeClock) -> None:
        self.node = node
        self.clock = clock
        self.contexts: list[FakeContext] = []
        self.initialized_contexts: list[FakeContext] = []
        self.shutdown_contexts: list[FakeContext] = []
        self.spin_once_calls = 0
        self.spin_timeout_s: list[float] = []

    class Context:
        def __init__(self) -> None:
            self.token = 1

    def init(
        self,
        *,
        args: list[Any],
        context: FakeContext,
        signal_handler_options: Any,
    ) -> None:
        assert args == []
        assert signal_handler_options == self.SignalHandlerOptions.NO
        self.initialized_contexts.append(context)

    def create_node(self, name: str, *, context: FakeContext) -> FakeNode:
        assert name == "lynsense_real_box_perception"
        self.contexts.append(context)
        return self.node

    def spin_until_future_complete(
        self,
        node: FakeNode,
        future: Any,
        *,
        timeout_sec: float,
        context: FakeContext,
    ) -> None:
        assert node is self.node
        assert context is self.initialized_contexts[0]

    def spin_once(
        self,
        node: FakeNode,
        *,
        timeout_sec: float,
        context: FakeContext,
    ) -> None:
        assert node is self.node
        assert context is self.initialized_contexts[0]
        self.spin_once_calls += 1
        self.spin_timeout_s.append(timeout_sec)
        self.clock.advance(timeout_sec)
        node.pose_message = pose_message(self.clock.now_s)
        node.status_message = status_message(
            {
                "success": True,
                "ros_source_stamp": {
                    "sec": int(self.clock.now_s - 0.25),
                    "nanosec": 750_000_000,
                },
            }
        )
        node.deliver_pose(node.pose_message)
        node.deliver_status(node.status_message)

    def shutdown(self, *, context: FakeContext) -> None:
        self.shutdown_contexts.append(context)


def fake_rclpy(clock: FakeClock) -> tuple[FakeRclpy, SimpleNamespace]:
    context = FakeContext(7)
    node = FakeNode(context, clock)
    rclpy = FakeRclpy(node, clock)
    return rclpy, node


def message_modules() -> tuple[SimpleNamespace, SimpleNamespace, SimpleNamespace]:
    geometry = SimpleNamespace(msg=SimpleNamespace(PoseStamped=object))
    std_msgs = SimpleNamespace(msg=SimpleNamespace(String=object))
    std_srvs = SimpleNamespace(
        srv=SimpleNamespace(Trigger=SimpleNamespace(Request=type("Request", (), {})))
    )
    return geometry, std_msgs, std_srvs


def make_transport(
    clock: FakeClock,
) -> tuple[
    LivePerceptionTransport,
    FakeRclpy,
    FakeNode,
]:
    rclpy, node = fake_rclpy(clock)
    geometry, std_msgs, std_srvs = message_modules()
    transport = LivePerceptionTransport(
        profile=validate_atomic_capability_profile(live_profile()),
        rclpy_module=rclpy,
        geometry_msgs_module=geometry,
        std_msgs_module=std_msgs,
        std_srvs_module=std_srvs,
        now_s=clock,
    )
    node.pose_message = pose_message(clock.now_s)
    node.status_message = status_message()
    return transport, rclpy, node


def pose_message(received_at_s: float) -> Any:
    source_epoch_s = received_at_s - 0.25
    sec = int(source_epoch_s)
    nanosec = int((source_epoch_s - sec) * 1_000_000_000)
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id="base_link",
            stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
        ),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=0.887, y=-0.036, z=0.428),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=1.0, w=0.0),
        ),
    )


def status_message(payload: dict[str, Any] | None = None) -> Any:
    value = payload or {
        "success": True,
        "ros_source_stamp": {"sec": 99, "nanosec": 750_000_000},
    }
    return SimpleNamespace(data=json.dumps(value))


def test_private_context_subscribes_before_one_trigger_and_topics_decide():
    clock = FakeClock()
    transport, rclpy, node = make_transport(clock)
    client = BoxPerceptionClient(
        profile=validate_atomic_capability_profile(live_profile()),
        transport=transport,
        now_s=clock,
    )

    result = client.detect_box()
    transport.close()

    assert rclpy.initialized_contexts == [rclpy.contexts[0]]
    assert not hasattr(rclpy, "get_default_context")
    assert node.calls == [
        ("subscription", "/industrial_box/pose_base"),
        ("subscription", "/industrial_box/status"),
        ("client", "/industrial_box/trigger"),
    ]
    assert len(node.clients[0].requests) == 1
    assert client.trigger_count == 1
    assert result["status"] == "ok"
    assert result["trigger_receipt"]["request_submitted"] is True
    assert result["trigger_receipt"]["response"] == {"status": "timeout"}
    assert result["frame"] == "base_link"
    assert result["source_stamp_s"] == pytest.approx(result["received_at_s"] - 0.25)
    assert result["pose"]["yaw_rad"] == 3.141592653589793


def test_service_wait_exception_still_lets_topic_results_complete():
    clock = FakeClock()
    transport, rclpy, node = make_transport(clock)
    client = BoxPerceptionClient(
        profile=validate_atomic_capability_profile(live_profile()),
        transport=transport,
        now_s=clock,
    )

    def fail_service_wait(*_: Any, **__: Any) -> None:
        raise RuntimeError("service discovery failed")

    rclpy.spin_until_future_complete = fail_service_wait  # type: ignore[method-assign]
    result = client.detect_box()
    transport.close()

    assert len(node.clients[0].requests) == 1
    assert result["status"] == "ok"
    assert result["trigger_receipt"]["request_submitted"] is True
    assert result["trigger_receipt"]["response"]["status"] == "unavailable"


def test_callbacks_copy_message_values_and_reject_non_mapping_status_json():
    clock = FakeClock()
    transport, _, node = make_transport(clock)
    pose_values: list[Any] = []
    status_values: list[Any] = []
    transport.start_subscriptions(pose_values.append, status_values.append)
    node.pose_message = pose_message(clock.now_s)
    node.status_message = status_message({"success": True, "nested": {"a": 1}})
    original_status = node.status_message.data

    node.deliver_pose(node.pose_message)
    node.deliver_status(node.status_message)
    if (
        isinstance(status_values[0].payload, dict)
        and "nested" in status_values[0].payload
    ):
        status_values[0].payload["nested"]["a"] = 2
    assert status_values[0].payload["nested"]["a"] == 2
    assert original_status == json.dumps({"success": True, "nested": {"a": 1}})
    assert pose_values[0].source_stamp_s == clock.now_s - 0.25
    assert pose_values[0].pose == {
        "x": 0.887,
        "y": -0.036,
        "z": 0.428,
        "yaw_rad": 3.141592653589793,
    }

    node.status_message = status_message({"success": True})
    node.status_message.data = "{not-json"
    node.deliver_status(node.status_message)
    assert status_values[-1].payload["success"] is False
    assert status_values[-1].payload["error"] == "status_json_invalid"
    transport.close()


def test_close_times_out_then_retry_completes_private_context_shutdown():
    clock = FakeClock()
    transport, rclpy, node = make_transport(clock)
    release = threading.Event()
    original_destroy = node.subscriptions[0].destroy

    def slow_destroy() -> None:
        release.wait(timeout=1.0)
        original_destroy()

    node.subscriptions[0].destroy = slow_destroy  # type: ignore[method-assign]
    try:
        transport.close(timeout_s=0.01)
    except TimeoutError:
        pass
    else:
        raise AssertionError("first close should time out")
    assert not rclpy.shutdown_contexts

    release.set()
    transport.close(timeout_s=1.0)
    assert node.subscriptions[0].destroy_calls == 1
    assert node.subscriptions[1].destroy_calls == 1
    assert node.clients[0].destroy_calls == 1
    assert node.destroy_calls == 1
    assert rclpy.shutdown_contexts == [rclpy.initialized_contexts[0]]
    transport.close()


def test_failed_cleanup_is_reported_and_not_retried():
    clock = FakeClock()
    transport, rclpy, node = make_transport(clock)
    destroy_calls = 0
    original_destroy = node.subscriptions[0].destroy

    def failing_destroy() -> None:
        nonlocal destroy_calls
        destroy_calls += 1
        raise RuntimeError("destroy blocked")

    node.subscriptions[0].destroy = failing_destroy  # type: ignore[method-assign]
    with pytest.raises(RuntimeError, match="destroy blocked"):
        transport.close(timeout_s=1.0)
    with pytest.raises(RuntimeError, match="destroy blocked"):
        transport.close(timeout_s=1.0)

    assert destroy_calls == 1
    assert node.subscriptions[1].destroy_calls == 1
    assert node.destroy_calls == 1
    assert rclpy.shutdown_contexts == [rclpy.initialized_contexts[0]]
