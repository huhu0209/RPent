"""Transport-independent box perception request coordination."""

from __future__ import annotations

import copy
import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any, Protocol

from robots.lynsense_real_box.site_profile import SiteProfile


@dataclass(frozen=True, slots=True)
class PoseSample:
    source_stamp_s: float
    received_at_s: float
    frame: str
    transform_valid: bool
    pose: Mapping[str, float]


@dataclass(frozen=True, slots=True)
class StatusSample:
    source_stamp_s: float
    received_at_s: float
    payload: Mapping[str, Any]


class PerceptionTransport(Protocol):
    """The narrow transport boundary implemented by a future ROS adapter."""

    def start_subscriptions(
        self,
        pose_callback: Callable[[PoseSample], None],
        status_callback: Callable[[StatusSample], None],
    ) -> None:
        """Install the two result callbacks before a trigger is started."""

    def start_trigger(self, timeout_s: float) -> dict[str, Any]:
        """Start exactly one inference request without waiting for its result."""

    def wait_until(self, deadline_s: float) -> None:
        """Pump subscriptions until the deadline or transport-specific signal."""

    def close(self) -> None:
        """Release transport-owned subscriptions and pending clients."""


class OfflinePerceptionTransport:
    """Deterministic transport used by offline tests and future dry runs."""

    def __init__(
        self,
        *,
        now_s: Callable[[], float],
        pose: Mapping[str, Any] | None,
        status_payload: Mapping[str, Any] | None = None,
    ) -> None:
        self._now_s = now_s
        self._pose = pose
        self._status_payload = status_payload
        self._pose_callback: Callable[[PoseSample], None] | None = None
        self._status_callback: Callable[[StatusSample], None] | None = None
        self.trigger_requests = 0

    def start_subscriptions(
        self,
        pose_callback: Callable[[PoseSample], None],
        status_callback: Callable[[StatusSample], None],
    ) -> None:
        self._pose_callback = pose_callback
        self._status_callback = status_callback

    def start_trigger(self, timeout_s: float) -> dict[str, Any]:
        self.trigger_requests += 1
        return {
            "request_submitted": True,
            "timeout_s": timeout_s,
            "response": {"status": "dry_run"},
        }

    def wait_until(self, deadline_s: float) -> None:
        del deadline_s
        now = self._now_s()
        if self._status_callback is not None:
            payload = self._status_payload or {"success": True}
            stamp = _fixture_stamp(self._pose, now)
            self._status_callback(
                StatusSample(
                    source_stamp_s=stamp,
                    received_at_s=now,
                    payload=payload,
                )
            )
        if self._pose_callback is not None and self._pose is not None:
            pose = self._pose
            self._pose_callback(
                PoseSample(
                    source_stamp_s=_fixture_stamp(pose, now),
                    received_at_s=now,
                    frame=str(pose.get("frame", "")),
                    transform_valid=pose.get("transform_valid") is True,
                    pose=pose.get("pose", {}),
                )
            )

    def close(self) -> None:
        self._pose_callback = None
        self._status_callback = None


class BoxPerceptionClient:
    """Coordinate one bounded inference request and validate its topic result."""

    def __init__(
        self,
        *,
        profile: SiteProfile,
        transport: PerceptionTransport,
        now_s: Callable[[], float],
    ) -> None:
        self._profile = profile
        self._transport = transport
        self._now_s = now_s
        self._closed = False
        self._armed = False
        self._latest_pose: PoseSample | None = None
        self._latest_status: StatusSample | None = None
        self.trigger_count = 0

    def detect_box(self) -> dict[str, Any]:
        """Run one perception attempt; never retry inside this method."""

        if self._closed:
            return {"status": "failed", "reason": "client_closed"}

        self._latest_pose = None
        self._latest_status = None
        try:
            self._transport.start_subscriptions(self._on_pose, self._on_status)
        except Exception:
            return {"status": "failed", "reason": "subscription_start_failed"}

        requested_at_s = self._now_s()
        self._armed = True
        try:
            receipt = self._transport.start_trigger(
                self._profile.perception.trigger_timeout_s
            )
        except Exception:
            self._armed = False
            return {"status": "failed", "reason": "trigger_request_failed"}
        finally:
            self.trigger_count += 1

        if not isinstance(receipt, dict) or receipt.get(
            "request_submitted"
        ) is not True:
            self._armed = False
            reason = "trigger_not_submitted"
            if isinstance(receipt, dict) and isinstance(
                receipt.get("reason"), str
            ):
                reason = receipt["reason"]
            return {
                "status": "failed",
                "reason": reason,
                "trigger_receipt": copy.deepcopy(receipt),
            }

        try:
            self._transport.wait_until(
                requested_at_s + self._profile.perception.result_timeout_s
            )
        except Exception:
            return {"status": "failed", "reason": "result_wait_failed"}
        finally:
            self._armed = False

        return self._result(receipt, requested_at_s)

    def close(self) -> None:
        """Close the coordinator and its transport exactly once."""

        if self._closed:
            return
        self._closed = True
        self._armed = False
        self._transport.close()

    def _on_pose(self, sample: PoseSample) -> None:
        if self._armed:
            self._latest_pose = sample

    def _on_status(self, sample: StatusSample) -> None:
        if self._armed:
            self._latest_status = sample

    def _result(
        self,
        receipt: dict[str, Any],
        requested_at_s: float,
    ) -> dict[str, Any]:
        status = self._latest_status
        pose = self._latest_pose
        common: dict[str, Any] = {
            "trigger_receipt": copy.deepcopy(receipt),
            "requested_at_s": requested_at_s,
        }

        if status is None:
            return {**common, "status": "failed", "reason": "status_timeout"}
        payload = status.payload
        if not isinstance(payload, Mapping):
            return {**common, "status": "failed", "reason": "status_invalid"}
        if payload.get("success") is not True:
            result = {**common, "status": "failed", "reason": "perception_failed"}
            if isinstance(payload.get("error_type"), str):
                result["error_type"] = payload["error_type"]
            if isinstance(payload.get("error"), str):
                result["error"] = payload["error"]
            return result

        if pose is None:
            return {**common, "status": "failed", "reason": "pose_timeout"}
        if pose.frame != self._profile.perception.frame:
            return {**common, "status": "failed", "reason": "wrong_frame"}
        if pose.transform_valid is not True:
            return {**common, "status": "failed", "reason": "transform_invalid"}

        now = self._now_s()
        pose_age_s = now - pose.source_stamp_s
        status_age_s = now - status.source_stamp_s
        max_age_s = self._profile.freshness["perception_max_age_s"]
        if (
            not math.isfinite(pose_age_s)
            or not math.isfinite(status_age_s)
            or pose_age_s < 0.0
            or status_age_s < 0.0
        ):
            return {**common, "status": "failed", "reason": "perception_invalid"}
        if pose_age_s > max_age_s or status_age_s > max_age_s:
            return {**common, "status": "failed", "reason": "perception_stale"}

        normalized_pose = _normalized_pose(pose.pose)
        if normalized_pose is None:
            return {**common, "status": "failed", "reason": "perception_invalid"}

        return {
            **common,
            "status": "ok",
            "frame": pose.frame,
            "pose": normalized_pose,
            "source_stamp_s": pose.source_stamp_s,
            "received_at_s": pose.received_at_s,
            "age_s": pose_age_s,
            "perception_status": copy.deepcopy(dict(payload)),
        }


def _fixture_stamp(pose: Mapping[str, Any] | None, now: float) -> float:
    if pose is not None:
        raw_age = pose.get("age_s")
        if not isinstance(raw_age, bool) and isinstance(raw_age, (int, float)):
            return now - float(raw_age)
    return now


def _normalized_pose(value: Any) -> dict[str, float] | None:
    if not isinstance(value, Mapping) or set(value) != {
        "x",
        "y",
        "z",
        "yaw_rad",
    }:
        return None
    result: dict[str, float] = {}
    for key in ("x", "y", "z", "yaw_rad"):
        raw = value[key]
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            return None
        number = float(raw)
        if not math.isfinite(number):
            return None
        result[key] = number
    return result
