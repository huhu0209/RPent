"""Offline-testable ROS transport for one Robot One box perception request."""

from __future__ import annotations

import copy
import json
import math
import time
from collections.abc import Callable, Mapping
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import TimeoutError as FutureTimeout
from threading import RLock
from typing import Any

from robots.lynsense_real_box.atomic_profile import AtomicCapabilityProfile
from robots.lynsense_real_box.perception_client import PoseSample, StatusSample


class LivePerceptionTransport:
    """Own one private ROS context and the three box-perception endpoints.

    ROS modules are injected to keep imports and every test offline. The
    constructor creates subscriptions before the service client, while the
    service request is created only in :meth:`start_trigger`.
    """

    def __init__(
        self,
        *,
        profile: AtomicCapabilityProfile,
        rclpy_module: Any,
        geometry_msgs_module: Any,
        std_msgs_module: Any,
        std_srvs_module: Any,
        now_s: Callable[[], float] | None = None,
    ) -> None:
        binding = profile.perception_binding
        if binding is None:
            raise ValueError("profile must contain perception_binding")

        self._profile = profile
        self._binding = binding
        self._rclpy = rclpy_module
        self._geometry_msgs = geometry_msgs_module
        self._std_msgs = std_msgs_module
        self._std_srvs = std_srvs_module
        self._now_s = now_s if now_s is not None else time.time
        self._closed = False
        self._context_shutdown = False
        self._pose_callback: Callable[[PoseSample], None] | None = None
        self._status_callback: Callable[[StatusSample], None] | None = None
        self._latest_pose: PoseSample | None = None
        self._latest_status: StatusSample | None = None
        self._cleanup_lock = RLock()
        self._cleanup_tasks: dict[str, Any] = {}
        self._cleanup_executor: ThreadPoolExecutor | None = None
        self._cleanup_failures: dict[str, str] = {}
        self._node: Any = None
        self._pose_subscription: Any = None
        self._status_subscription: Any = None
        self._trigger_client: Any = None

        self._context = self._rclpy.Context()
        self._rclpy.init(
            args=[],
            context=self._context,
            signal_handler_options=self._rclpy.SignalHandlerOptions.NO,
        )
        try:
            self._node = self._rclpy.create_node(
                "lynsense_real_box_perception",
                context=self._context,
            )
            pose_message = self._geometry_msgs.msg.PoseStamped
            status_message = self._std_msgs.msg.String
            trigger_service = self._std_srvs.srv.Trigger
            self._pose_subscription = self._node.create_subscription(
                pose_message,
                binding.pose_topic,
                self._on_pose_message,
                10,
            )
            self._status_subscription = self._node.create_subscription(
                status_message,
                binding.status_topic,
                self._on_status_message,
                10,
            )
            self._trigger_client = self._node.create_client(
                trigger_service,
                binding.trigger_service,
            )
        except Exception:
            self.close(timeout_s=2.0)
            raise

    def start_subscriptions(
        self,
        pose_callback: Callable[[PoseSample], None],
        status_callback: Callable[[StatusSample], None],
    ) -> None:
        """Retain the coordinator callbacks after ROS subscriptions exist."""

        self._pose_callback = pose_callback
        self._status_callback = status_callback
        if self._closed:
            raise RuntimeError("perception transport is closed")

    def identity(self) -> dict[str, str]:
        """Return the exact endpoint binding owned by this transport."""

        return {
            "pose_topic": self._binding.pose_topic,
            "status_topic": self._binding.status_topic,
            "trigger_service": self._binding.trigger_service,
            "expected_frame": self._binding.expected_frame,
        }

    def start_trigger(self, timeout_s: float) -> dict[str, Any]:
        """Send exactly one async Trigger request and wait only for its receipt."""

        if self._closed:
            raise RuntimeError("perception transport is closed")
        if timeout_s <= 0.0 or not math.isfinite(timeout_s):
            raise ValueError("trigger timeout must be finite and positive")

        with self._cleanup_lock:
            if self._closed:
                raise RuntimeError("perception transport is closed")
            request = self._std_srvs.srv.Trigger.Request()
            future = self._trigger_client.call_async(request)
            try:
                self._rclpy.spin_until_future_complete(
                    self._node,
                    future,
                    timeout_sec=timeout_s,
                    context=self._context,
                )
            except Exception as exc:
                return {
                    "request_submitted": True,
                    "timeout_s": timeout_s,
                    "response": {
                        "status": "unavailable",
                        "error_type": type(exc).__name__,
                    },
                }

        response: Mapping[str, Any] = {"status": "timeout"}
        if future.done() and not future.cancelled():
            try:
                result = future.result()
                response = {
                    "status": "ok",
                    "success": bool(result.success),
                    "message": str(result.message),
                }
            except Exception as exc:
                response = {
                    "status": "unavailable",
                    "error_type": type(exc).__name__,
                }

        return {
            "request_submitted": True,
            "timeout_s": timeout_s,
            "response": copy.deepcopy(dict(response)),
        }

    def wait_until(self, deadline_s: float) -> None:
        """Pump the private context until the coordinator's result deadline."""

        while not self._closed:
            remaining_s = deadline_s - self._now_s()
            if remaining_s <= 0.0:
                return
            spin_timeout_s = min(0.05, remaining_s)
            spin_started_at = time.monotonic()
            with self._cleanup_lock:
                if self._closed:
                    return
                self._rclpy.spin_once(
                    self._node,
                    timeout_sec=spin_timeout_s,
                    context=self._context,
                )
            if deadline_s - self._now_s() <= 0.0:
                return
            if self._latest_pose is not None and self._latest_status is not None:
                return
            wall_remaining_s = spin_timeout_s - (time.monotonic() - spin_started_at)
            if wall_remaining_s > 0.0:
                time.sleep(wall_remaining_s)

    def close(self, *, timeout_s: float = 2.0) -> None:
        """Release resources within a bounded budget; safe to retry."""

        if timeout_s <= 0.0 or not math.isfinite(timeout_s):
            raise ValueError("close timeout must be finite and positive")

        with self._cleanup_lock:
            self._closed = True
            if self._context_shutdown:
                if self._cleanup_failures:
                    self._raise_cleanup_failure(timeout_s)
                return
            if self._cleanup_executor is None:
                self._cleanup_executor = ThreadPoolExecutor(
                    max_workers=1,
                    thread_name_prefix="lynsense-box-perception-close",
                )

            deadline = time.monotonic() + timeout_s
            resources: list[tuple[str, Callable[[], Any]]] = []
            if self._pose_subscription is not None:
                resources.append(("pose_subscription", self._pose_subscription.destroy))
            if self._status_subscription is not None:
                resources.append(
                    ("status_subscription", self._status_subscription.destroy)
                )
            if self._trigger_client is not None:
                resources.append(("trigger_client", self._trigger_client.destroy))
            if self._node is not None:
                resources.append(("node", self._node.destroy_node))
            resources.append(
                (
                    "context",
                    lambda: self._rclpy.shutdown(context=self._context),
                )
            )
            for name, operation in resources:
                if self._run_cleanup(name, operation, deadline):
                    continue
                self._raise_cleanup_failure(timeout_s)
                return

            self._context_shutdown = True
            if self._cleanup_executor is not None:
                self._cleanup_executor.shutdown(wait=False, cancel_futures=True)
                self._cleanup_executor = None
            if self._cleanup_failures:
                details = ", ".join(
                    f"{name}: {reason}"
                    for name, reason in sorted(self._cleanup_failures.items())
                )
                raise RuntimeError(
                    f"perception transport close failed: {details}"
                )

    def _run_cleanup(
        self,
        name: str,
        operation: Callable[[], Any],
        deadline: float,
    ) -> bool:
        assert self._cleanup_executor is not None
        if name not in self._cleanup_tasks:
            self._cleanup_tasks[name] = self._cleanup_executor.submit(operation)
        try:
            self._cleanup_tasks[name].result(
                timeout=max(0.0, deadline - time.monotonic())
            )
        except FutureTimeout:
            return False
        except Exception as exc:
            self._cleanup_failures[name] = f"{type(exc).__name__}: {exc}"
            return True

        self._cleanup_tasks.pop(name, None)
        self._cleanup_failures.pop(name, None)
        return True

    def _raise_cleanup_failure(self, requested_timeout_s: float) -> None:
        if self._cleanup_failures:
            details = ", ".join(
                f"{name}: {reason}"
                for name, reason in sorted(self._cleanup_failures.items())
            )
            raise RuntimeError(f"perception transport close failed: {details}")
        raise TimeoutError(
            f"perception transport close did not finish in {requested_timeout_s} s"
        )

    def _on_pose_message(self, message: Any) -> None:
        if self._closed:
            return
        quaternion = message.pose.orientation
        yaw_rad, quaternion_valid = _quaternion_to_yaw(quaternion)
        self._latest_pose = PoseSample(
            source_stamp_s=_stamp_to_epoch_s(message.header.stamp),
            received_at_s=self._now_s(),
            frame=str(message.header.frame_id),
            transform_valid=quaternion_valid,
            pose={
                "x": float(message.pose.position.x),
                "y": float(message.pose.position.y),
                "z": float(message.pose.position.z),
                "yaw_rad": yaw_rad,
            },
        )
        if self._pose_callback is not None:
            self._pose_callback(self._latest_pose)

    def _on_status_message(self, message: Any) -> None:
        if self._closed:
            return
        received_at_s = self._now_s()
        try:
            payload_value = json.loads(message.data)
        except (TypeError, ValueError):
            payload: dict[str, Any] = {
                "success": False,
                "error": "status_json_invalid",
                "raw": copy.deepcopy(message.data),
            }
            source_stamp_s = received_at_s
        else:
            if isinstance(payload_value, Mapping):
                payload = copy.deepcopy(dict(payload_value))
            else:
                payload = {
                    "success": False,
                    "error": "status_json_invalid",
                    "raw": copy.deepcopy(payload_value),
                }
            source_stamp_s = _payload_stamp_to_epoch_s(
                payload.get("ros_source_stamp"),
                received_at_s,
            )

        self._latest_status = StatusSample(
            source_stamp_s=source_stamp_s,
            received_at_s=received_at_s,
            payload=payload,
        )
        if self._status_callback is not None:
            self._status_callback(self._latest_status)


def _stamp_to_epoch_s(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _payload_stamp_to_epoch_s(value: Any, fallback_s: float) -> float:
    if not isinstance(value, Mapping):
        return fallback_s
    sec = value.get("sec")
    nanosec = value.get("nanosec")
    if isinstance(sec, bool) or isinstance(nanosec, bool):
        return fallback_s
    if not isinstance(sec, (int, float)) or not isinstance(
        nanosec,
        (int, float),
    ):
        return fallback_s
    epoch_s = float(sec) + float(nanosec) / 1_000_000_000.0
    return epoch_s if math.isfinite(epoch_s) else fallback_s


def _quaternion_to_yaw(quaternion: Any) -> tuple[float, bool]:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    magnitude = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(magnitude) or magnitude <= 0.0:
        return float("nan"), False
    x /= magnitude
    y /= magnitude
    z /= magnitude
    w /= magnitude
    yaw_rad = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return yaw_rad, True
