from __future__ import annotations

import math
from typing import Any

import pytest

from robots.lynsense_real_box.perception_client import (
    BoxPerceptionClient,
    OfflinePerceptionTransport,
    PoseSample,
    StatusSample,
)
from tests.unit_tests.robots.lynsense_real_box.test_site_profile import (
    valid_profile,
)


class SequentialTransport:
    def __init__(self, *, deliver: Any = None) -> None:
        self.deliver = deliver
        self.events: list[str] = []
        self.trigger_count = 0

    def start_subscriptions(self, pose_callback, status_callback) -> None:
        self.events.append("subscribe")
        self.pose_callback = pose_callback
        self.status_callback = status_callback

    def start_trigger(self, timeout_s: float) -> dict[str, Any]:
        self.trigger_count += 1
        self.events.append("trigger")
        return {
            "request_submitted": True,
            "timeout_s": timeout_s,
            "response": {"status": "timeout"},
        }

    def wait_until(self, deadline_s: float) -> None:
        self.events.append("wait")
        if self.deliver is not None:
            self.deliver(self)

    def close(self) -> None:
        self.events.append("close")


class MutableClock:
    def __init__(self, now_s: float = 100.0) -> None:
        self.now_s = now_s

    def __call__(self) -> float:
        return self.now_s


def profile():
    from robots.lynsense_real_box.site_profile import validate_site_profile

    return validate_site_profile(valid_profile())


def pose_sample(
    clock: MutableClock,
    *,
    age_s: float = 0.1,
    frame: str = "offline_box_fixture",
    transform_valid: bool = True,
    values: dict[str, float] | None = None,
) -> PoseSample:
    now = clock.now_s
    return PoseSample(
        source_stamp_s=now - age_s,
        received_at_s=now,
        frame=frame,
        transform_valid=transform_valid,
        pose=values
        or {"x": 1.0, "y": -2.0, "z": 0.1, "yaw_rad": 0.0},
    )


def status_sample(
    clock: MutableClock,
    *,
    age_s: float = 0.1,
    payload: dict[str, Any] | None = None,
) -> StatusSample:
    now = clock.now_s
    return StatusSample(
        source_stamp_s=now - age_s,
        received_at_s=now,
        payload=payload or {"success": True},
    )


def deliver_success(transport: SequentialTransport) -> None:
    transport.status_callback(transport.status)
    transport.pose_callback(transport.pose)


def make_client(transport: Any, clock: MutableClock) -> BoxPerceptionClient:
    return BoxPerceptionClient(
        profile=profile(),
        transport=transport,
        now_s=clock,
    )


def test_subscriptions_start_before_one_trigger_and_topic_result_wins():
    clock = MutableClock()
    transport = SequentialTransport(deliver=deliver_success)
    transport.status = status_sample(clock)
    transport.pose = pose_sample(clock)
    client = make_client(transport, clock)

    result = client.detect_box()

    assert transport.events[:3] == ["subscribe", "trigger", "wait"]
    assert transport.trigger_count == 1
    assert client.trigger_count == 1
    assert result["status"] == "ok"
    assert result["frame"] == "offline_box_fixture"
    assert result["pose"] == {
        "x": 1.0,
        "y": -2.0,
        "z": 0.1,
        "yaw_rad": 0.0,
    }
    assert result["trigger_receipt"]["response"] == {"status": "timeout"}


def test_samples_before_the_trigger_request_are_ignored():
    clock = MutableClock()
    transport = SequentialTransport()
    client = make_client(transport, clock)
    client._on_pose(pose_sample(clock))
    client._on_status(status_sample(clock))

    result = client.detect_box()

    assert result["status"] == "failed"
    assert result["reason"] == "status_timeout"


def test_trigger_start_exception_closes_the_callback_window():
    class TriggerFails:
        def __init__(self) -> None:
            self.failed_once = False

        def start_subscriptions(self, pose_callback, status_callback) -> None:
            self.pose_callback = pose_callback
            self.status_callback = status_callback

        def start_trigger(self, timeout_s: float) -> dict[str, Any]:
            if not self.failed_once:
                self.failed_once = True
                raise RuntimeError("transport unavailable")
            return {"request_submitted": True}

        def wait_until(self, deadline_s: float) -> None:
            pass

        def close(self) -> None:
            pass

    clock = MutableClock()
    transport = TriggerFails()
    client = make_client(transport, clock)
    result = client.detect_box()

    assert result["status"] == "failed"
    assert result["reason"] == "trigger_request_failed"
    transport.pose_callback(pose_sample(clock))
    transport.status_callback(status_sample(clock))
    assert client.detect_box()["reason"] == "status_timeout"


def test_failed_status_is_returned_without_another_trigger():
    clock = MutableClock()

    def deliver_failure(transport: SequentialTransport) -> None:
        transport.status_callback(
            status_sample(
                clock,
                payload={
                    "success": False,
                    "error_type": "RuntimeError",
                    "error": "no valid depth pixels",
                },
            )
        )

    transport = SequentialTransport(deliver=deliver_failure)
    client = make_client(transport, clock)
    result = client.detect_box()

    assert result == {
        "status": "failed",
        "reason": "perception_failed",
        "error_type": "RuntimeError",
        "error": "no valid depth pixels",
        "trigger_receipt": result["trigger_receipt"],
        "requested_at_s": 100.0,
    }
    assert transport.trigger_count == 1


def test_missing_pose_and_stale_results_have_distinct_reasons():
    clock = MutableClock()
    status_only = SequentialTransport(
        deliver=lambda transport: transport.status_callback(status_sample(clock))
    )
    assert make_client(status_only, clock).detect_box()["reason"] == "pose_timeout"

    stale = SequentialTransport(deliver=deliver_success)
    stale.status = status_sample(clock, age_s=2.0)
    stale.pose = pose_sample(clock, age_s=2.0)
    assert make_client(stale, clock).detect_box()["reason"] == "perception_stale"


@pytest.mark.parametrize(
    ("pose_kwargs", "reason"),
    [
        ({"frame": "camera_link"}, "wrong_frame"),
        ({"transform_valid": False}, "transform_invalid"),
        (
            {
                "values": {
                    "x": 1.0,
                    "y": -2.0,
                    "z": 0.1,
                    "yaw_rad": math.nan,
                }
            },
            "perception_invalid",
        ),
    ],
)
def test_invalid_pose_results_are_rejected(pose_kwargs, reason):
    clock = MutableClock()
    transport = SequentialTransport(deliver=deliver_success)
    transport.status = status_sample(clock)
    transport.pose = pose_sample(clock, **pose_kwargs)
    result = make_client(transport, clock).detect_box()
    assert result["reason"] == reason


def test_transport_rejection_does_not_wait_for_topics():
    class NotSubmitted:
        def start_subscriptions(self, pose_callback, status_callback) -> None:
            del pose_callback, status_callback

        def start_trigger(self, timeout_s: float) -> dict[str, Any]:
            return {
                "request_submitted": False,
                "reason": "service_unavailable",
                "timeout_s": timeout_s,
            }

        def wait_until(self, deadline_s: float) -> None:
            raise AssertionError("wait_until must not run")

        def close(self) -> None:
            pass

    clock = MutableClock()
    result = make_client(NotSubmitted(), clock).detect_box()
    assert result["status"] == "failed"
    assert result["reason"] == "service_unavailable"


def test_offline_transport_emits_status_and_fixture_pose():
    clock = MutableClock()
    site = profile()
    transport = OfflinePerceptionTransport(
        now_s=clock,
        pose={
            "age_s": 0.2,
            "frame": site.perception.frame,
            "transform_valid": True,
            "pose": {"x": 1.0, "y": -2.0, "z": 0.1, "yaw_rad": 0.0},
        },
    )
    result = make_client(transport, clock).detect_box()

    assert result["status"] == "ok"
    assert result["age_s"] == pytest.approx(0.2)
    assert result["perception_status"] == {"success": True}
    assert transport.trigger_requests == 1


def test_offline_transport_preserves_nonfinite_fixture_age_as_invalid():
    clock = MutableClock()
    site = profile()
    transport = OfflinePerceptionTransport(
        now_s=clock,
        pose={
            "age_s": math.inf,
            "frame": site.perception.frame,
            "transform_valid": True,
            "pose": {"x": 1.0, "y": -2.0, "z": 0.1, "yaw_rad": 0.0},
        },
    )
    result = make_client(transport, clock).detect_box()
    assert result["status"] == "failed"
    assert result["reason"] == "perception_invalid"
