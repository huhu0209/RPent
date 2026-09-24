"""Profile-bounded ``/odom`` to ``/cmd_vel`` chassis transport."""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from threading import Lock, RLock, Thread
from typing import Any, Protocol

from robots.lynsense_real_box.atomic_profile import AtomicCapabilityProfile


class ChassisRuntimeError(RuntimeError):
    """Raised when the bounded chassis transport cannot operate safely."""


class _Clock(Protocol):
    """The small deterministic clock surface used by this transport."""

    def now(self) -> float: ...

    def monotonic(self) -> float: ...

    def sleep(self, seconds: float) -> None: ...


class _SystemClock:
    """Adapt process time to the transport's injected-clock interface."""

    def now(self) -> float:
        """Return the wall-clock receipt time."""

        return time.time()

    def monotonic(self) -> float:
        """Return an unrelated monotonic scheduling time."""

        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        """Sleep for a bounded control period."""

        time.sleep(seconds)


@dataclass(frozen=True, slots=True)
class RuntimeResult:
    """A small result shape recognized by the atomic adapter."""

    code: int | None
    message: str
    data: Any = None

    @property
    def ok(self) -> bool:
        """Return whether the result represents success."""

        return self.code == 0


@dataclass(frozen=True, slots=True)
class _OdomSample:
    valid: bool
    moving: bool
    timestamp_s: float
    sequence: int
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0


@dataclass(frozen=True, slots=True)
class _MovePlan:
    distance_m: float
    angle_rad: float
    x_m: float
    y_m: float
    yaw_rad: float
    sequence: int
    deadline_s: float


def _load_odometry_type() -> Any:
    from nav_msgs.msg import Odometry

    return Odometry


def _load_twist_type() -> Any:
    from geometry_msgs.msg import Twist

    return Twist


class _DisplacementTask:
    """Wait for one already-dispatched bounded displacement."""

    def __init__(self, worker: Callable[[], None]) -> None:
        self._stop_requested = False
        self._result: RuntimeResult | None = None
        self._thread = Thread(
            target=self._run,
            args=(worker,),
            name="rpent-bounded-chassis-move",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def request_stop(self) -> None:
        self._stop_requested = True

    def wait(self, timeout_s: float) -> RuntimeResult:
        timeout_s = float(timeout_s)
        if not math.isfinite(timeout_s) or timeout_s < 0.0:
            raise ValueError("timeout_s must be finite and nonnegative")
        self._thread.join(timeout_s)
        if self._thread.is_alive():
            return RuntimeResult(
                code=5001,
                message="displacement budget expired",
                data={"state": "RUNNING"},
            )
        assert self._result is not None
        return self._result

    @property
    def running(self) -> bool:
        return self._thread.is_alive()

    @property
    def stop_requested(self) -> bool:
        return self._stop_requested

    def _run(self, worker: Callable[[], None]) -> None:
        try:
            self._result = worker()
        except BaseException as error:
            self._result = RuntimeResult(
                code=1,
                message="displacement runtime exception",
                data={"state": "FAILED", "reason": str(error)},
            )


class BoundedOdomCmdVelChassisRuntime:
    """Execute only reviewed displacements with fresh odom feedback.

    The caller supplies ``rclpy``.  Normal composition supplies the real
    module; offline tests may supply a fake.  This class never initializes or
    reads the process-global ROS context.
    """

    def __init__(
        self,
        profile: AtomicCapabilityProfile,
        rclpy: Any,
        *,
        clock: _Clock | None = None,
        node_factory: Callable[[Any], Any] | None = None,
        message_types: tuple[Any, Any] | None = None,
    ) -> None:
        if profile.version != 2:
            raise ChassisRuntimeError("chassis runtime requires profile version 2")
        chassis_profile = profile.chassis_runtime
        if chassis_profile is None:
            raise ChassisRuntimeError("profile has no bounded chassis runtime")
        if chassis_profile.kind != "bounded_odom_cmd_vel":
            raise ChassisRuntimeError("unsupported chassis runtime kind")

        self._profile = profile
        self._chassis_profile = chassis_profile
        self._rclpy = rclpy
        self._clock = clock or _SystemClock()
        self._context = rclpy.Context()
        try:
            self._context.init(args=[])
            self._node = (
                node_factory(self._context)
                if node_factory is not None
                else rclpy.create_node(
                    "rpent_bounded_chassis",
                    context=self._context,
                )
            )
            self._odom_type, self._twist_type = message_types or (
                _load_odometry_type(),
                _load_twist_type(),
            )
            self._state_lock = RLock()
            self._command_lock = Lock()
            self._spin_lock = Lock()
            self._closing = False
            self._closed = False
            self._stop_failed = False
            self._publisher: Any | None = None
            self._subscription: Any | None = None
            self._active_task: _DisplacementTask | None = None
            self._next_sequence = 0
            self._sample: _OdomSample | None = None
            self._subscription = self._node.create_subscription(
                self._odom_type,
                chassis_profile.odom_topic,
                self._on_odom,
                1,
            )
            self._preflight()
        except BaseException as error:
            try:
                self._shutdown_failed_preflight()
            except BaseException as cleanup_error:
                raise ChassisRuntimeError(
                    "chassis construction failed and private-context cleanup failed: "
                    f"{cleanup_error}"
                ) from error
            raise

    def read(self) -> RuntimeResult:
        """Return one fresh odom-derived chassis state without retrying."""

        with self._state_lock:
            self._ensure_open()
        self._spin_once(0.0)
        sample = self._current_sample()
        if sample is None:
            return RuntimeResult(
                code=1,
                message="odom unavailable",
                data=self._state_data(None),
            )
        if not self._is_fresh(sample):
            return RuntimeResult(
                code=1,
                message="odom stale",
                data=self._state_data(sample),
            )
        return RuntimeResult(
            code=0,
            message="",
            data=self._state_data(sample),
        )

    def identity(self) -> dict[str, str]:
        """Return the exact bounded transport created for this runtime."""

        return {
            "kind": self._chassis_profile.kind,
            "odom_topic": self._chassis_profile.odom_topic,
            "cmd_vel_topic": self._chassis_profile.cmd_vel_topic,
        }

    def navigate(self, goal: str) -> RuntimeResult:
        """Reject navigation unconditionally; no goal transport is exposed."""

        del goal
        return RuntimeResult(
            code=1,
            message="navigation disabled",
            data=None,
        )

    def move_distance(self, distance_m: float, angle_deg: float) -> RuntimeResult:
        """Start one profile-bounded displacement and return its task."""

        distance_m = float(distance_m)
        angle_deg = float(angle_deg)
        if not math.isfinite(distance_m) or not math.isfinite(angle_deg):
            return self._move_rejection("displacement_not_finite")
        if (
            abs(distance_m) > self._profile.max_chassis_distance_m
            or abs(math.radians(angle_deg))
            > self._profile.max_chassis_angle_deg
        ):
            return self._move_rejection("displacement_outside_profile")

        with self._state_lock:
            self._ensure_open()
            if self._stop_failed:
                return self._move_rejection("stop_unconfirmed")
            if self._active_task is not None:
                return self._move_rejection("displacement_active")
            sample = self._current_sample_locked()
            if sample is None or not self._is_fresh(sample) or not sample.valid:
                return self._move_rejection("odom_not_fresh")
            if sample.moving:
                return self._move_rejection("chassis_busy")
            try:
                self._create_publisher_locked()
            except Exception as error:
                self._stop_failed = True
                return RuntimeResult(
                    code=1,
                    message="publisher creation failed",
                    data={"reason": "publisher_creation_failed", "error": str(error)},
                )

        deadline_s = self._monotonic() + self._motion_budget_s(
            distance_m,
            angle_deg,
        )
        plan = _MovePlan(
            distance_m=distance_m,
            angle_rad=math.radians(angle_deg),
            x_m=sample.x_m,
            y_m=sample.y_m,
            yaw_rad=sample.yaw_rad,
            sequence=sample.sequence,
            deadline_s=deadline_s,
        )
        task = _DisplacementTask(
            lambda: self._run_displacement(plan),
        )
        with self._state_lock:
            if self._closing or self._closed:
                return self._move_rejection("closed")
            self._active_task = task
        task.start()
        return RuntimeResult(
            code=0,
            message="",
            data=task,
        )

    def stop(self) -> RuntimeResult:
        """Publish zero and confirm fresh zero-twist samples once."""

        return self._confirm_stop()

    def close(self) -> None:
        """Close resources without motion and allow caller-initiated retry."""

        with self._state_lock:
            if self._closed:
                return
            if self._closing:
                raise ChassisRuntimeError("close already running")
            if self._active_task is not None and self._active_task.running:
                raise ChassisRuntimeError(
                    "cannot close while a displacement is running"
                )
            self._closing = True

        if not self._command_lock.acquire(
            timeout=self._chassis_profile.stop_settle_timeout_s,
        ):
            with self._state_lock:
                self._closing = False
            raise ChassisRuntimeError("close budget expired while publishing")
        try:
            if self._publisher is not None:
                self._publisher.destroy()
                self._publisher = None
            if self._subscription is not None:
                if hasattr(
                    self._subscription,
                    "destroy",
                ):
                    self._subscription.destroy()
                self._subscription = None
            if hasattr(self._node, "destroy_node"):
                self._node.destroy_node()
            self._rclpy.shutdown(context=self._context)
            with self._state_lock:
                self._closed = True
        finally:
            with self._state_lock:
                self._closing = False
            self._command_lock.release()

    def _preflight(self) -> None:
        deadline_s = (
            self._monotonic() + self._chassis_profile.watchdog_timeout_s
        )
        try:
            while self._current_sample() is None:
                remaining_s = deadline_s - self._monotonic()
                if remaining_s <= 0.0:
                    raise ChassisRuntimeError("odom preflight timed out")
                self._spin_once(min(remaining_s, 0.1))
                sample = self._current_sample()
                if sample is not None and (not sample.valid or not self._is_fresh(sample)):
                    raise ChassisRuntimeError("odom preflight sample invalid")
        except Exception:
            raise

    def _shutdown_failed_preflight(self) -> None:
        subscription = getattr(self, "_subscription", None)
        node = getattr(self, "_node", None)
        if subscription is not None and hasattr(subscription, "destroy"):
            subscription.destroy()
            self._subscription = None
        if node is not None and hasattr(node, "destroy_node"):
            node.destroy_node()
        self._rclpy.shutdown(context=self._context)

    def _on_odom(self, message: Any) -> None:
        sequence: int
        with self._state_lock:
            if self._closed:
                return
            self._next_sequence += 1
            sequence = self._next_sequence
            timestamp_s = self._clock.now()
        try:
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            twist = message.twist.twist
            x_m = float(position.x)
            y_m = float(position.y)
            yaw_rad = self._yaw_from_quaternion(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            )
            linear_x = float(twist.linear.x)
            angular_z = float(twist.angular.z)
            values = (x_m, y_m, yaw_rad, linear_x, angular_z)
            valid = all(math.isfinite(value) for value in values)
            moving = valid and (
                abs(linear_x) > 1e-12 or abs(angular_z) > 1e-12
            )
            sample = _OdomSample(
                valid=valid,
                moving=moving,
                timestamp_s=timestamp_s,
                sequence=sequence,
                x_m=x_m,
                y_m=y_m,
                yaw_rad=yaw_rad,
            )
        except (AttributeError, TypeError, ValueError):
            sample = _OdomSample(
                valid=False,
                moving=False,
                timestamp_s=timestamp_s,
                sequence=sequence,
            )
        with self._state_lock:
            if not self._closed:
                self._sample = sample

    def _run_displacement(self, plan: _MovePlan) -> RuntimeResult:
        period_s = 1.0 / self._chassis_profile.command_rate_hz
        linear_speed = 0.0
        angular_speed = 0.0
        previous_s = self._monotonic()
        last_sample_sequence = plan.sequence
        last_feedback_s = previous_s
        reached = False

        try:
            while not reached:
                if self._monotonic() >= plan.deadline_s:
                    return self._fail_displacement("displacement_budget_expired")
                if self._stop_requested():
                    return self._fail_displacement("stop_requested")

                remaining_s = plan.deadline_s - self._monotonic()
                spin_started_s = self._monotonic()
                self._spin_once(min(period_s, max(0.0, remaining_s)))
                spin_elapsed_s = self._monotonic() - spin_started_s
                sample = self._current_sample()
                if sample is None or not self._is_fresh(sample) or not sample.valid:
                    if sample is None or not sample.valid:
                        return self._fail_displacement("odom_invalid")
                    return self._fail_displacement("odom_watchdog")
                if sample.sequence > last_sample_sequence:
                    last_sample_sequence = sample.sequence
                    last_feedback_s = self._monotonic()
                if (
                    self._monotonic() - last_feedback_s
                    > self._chassis_profile.watchdog_timeout_s
                ):
                    return self._fail_displacement("odom_watchdog")
                along_m = self._forward_coordinate(
                    sample.x_m,
                    sample.y_m,
                    plan.yaw_rad,
                )
                base_along_m = self._forward_coordinate(
                    plan.x_m,
                    plan.y_m,
                    plan.yaw_rad,
                )
                linear_remaining_m = plan.distance_m - (
                    along_m - base_along_m
                )
                angular_remaining_rad = self._normalize_angle(
                    plan.angle_rad - self._normalize_angle(
                        sample.yaw_rad - plan.yaw_rad,
                    )
                )
                now_s = self._monotonic()
                elapsed_s = min(
                    max(now_s - previous_s, 0.0),
                    2.0 * period_s,
                )
                previous_s = now_s
                linear_speed = self._ramp_speed(
                    linear_speed,
                    linear_remaining_m,
                    self._chassis_profile.max_linear_speed_m_s,
                    self._chassis_profile.max_linear_accel_m_s2,
                    elapsed_s,
                )
                angular_speed = self._ramp_speed(
                    angular_speed,
                    angular_remaining_rad,
                    self._chassis_profile.max_angular_speed_rad_s,
                    self._chassis_profile.max_angular_accel_rad_s2,
                    elapsed_s,
                )
                linear_tolerance_s = max(
                    1e-9,
                    self._chassis_profile.max_linear_speed_m_s * period_s,
                )
                angular_tolerance_s = max(
                    1e-9,
                    self._chassis_profile.max_angular_speed_rad_s * period_s,
                )
                reached = (
                    abs(linear_remaining_m) <= linear_tolerance_s
                    and abs(angular_remaining_rad) <= angular_tolerance_s
                )
                if not self._publish_command(linear_speed, angular_speed):
                    return self._fail_displacement("publish_failed")

                if reached:
                    break
                self._clock.sleep(
                    max(0.0, period_s - spin_elapsed_s),
                )

            stopped = self._confirm_stop(
                timeout_s=self._chassis_profile.stop_settle_timeout_s,
                from_displacement=True,
            )
            if not stopped.ok:
                return RuntimeResult(
                    code=1,
                    message="post-move stop unconfirmed",
                    data={"reason": "stop_unconfirmed"},
                )
            return RuntimeResult(
                code=0,
                message="",
                data={"state": "SUCCEEDED"},
            )
        finally:
            with self._state_lock:
                if self._active_task is not None:
                    self._active_task = None

    def _confirm_stop(
        self,
        *,
        timeout_s: float | None = None,
        from_displacement: bool = False,
    ) -> RuntimeResult:
        if timeout_s is None:
            timeout_s = self._chassis_profile.stop_settle_timeout_s
        active = self._active_task if not from_displacement else None
        if active is not None:
            active.request_stop()

        with self._state_lock:
            self._ensure_open()
            sample = self._current_sample_locked()
            try:
                self._create_publisher_locked()
            except Exception:
                self._stop_failed = True
                return self._stop_result(False, "publisher creation failed")
            if sample is None or not self._is_fresh(sample) or not sample.valid:
                self._publish_command(0.0, 0.0)
                self._stop_failed = True
                return self._stop_result(False, "odom not fresh")

        deadline_s = self._monotonic() + float(timeout_s)
        required_samples = self._chassis_profile.zero_twist_confirm_samples
        baseline_sequence = sample.sequence
        confirmed_sequence = baseline_sequence
        stable_samples = 0
        period_s = 1.0 / self._chassis_profile.command_rate_hz

        while self._monotonic() < deadline_s:
            if not self._publish_command(0.0, 0.0):
                self._stop_failed = True
                return self._stop_result(False, "zero publish failed")
            spin_started_s = self._monotonic()
            self._spin_once(
                min(period_s, max(0.0, deadline_s - self._monotonic()))
            )
            current = self._current_sample()
            if (
                current is not None
                and current.sequence > confirmed_sequence
                and current.valid
                and not current.moving
                and self._is_fresh(current)
            ):
                confirmed_sequence = current.sequence
                stable_samples += 1
            else:
                stable_samples = 0
            if stable_samples >= required_samples:
                self._stop_failed = False
                return self._stop_result(True, "")
            spin_elapsed_s = self._monotonic() - spin_started_s
            self._clock.sleep(
                max(
                    0.0,
                    min(
                        period_s,
                        deadline_s - self._monotonic(),
                    )
                    - spin_elapsed_s,
                )
            )

        self._stop_failed = True
        return self._stop_result(False, "stop confirmation timed out")

    def _fail_displacement(self, reason: str) -> RuntimeResult:
        self._confirm_stop(from_displacement=True)
        return RuntimeResult(
            code=1,
            message=reason,
            data={"state": "FAILED", "reason": reason},
        )

    def _publish_command(self, linear_speed: float, angular_speed: float) -> bool:
        with self._command_lock:
            with self._state_lock:
                if self._closing or self._closed:
                    return False
            try:
                twist = self._twist_type()
                twist.linear.x = linear_speed
                twist.angular.z = angular_speed
                publisher = self._publisher
                if publisher is None:
                    return False
                publisher.publish(twist)
                return True
            except Exception:
                return False

    def _spin_once(self, timeout_s: float) -> None:
        with self._spin_lock:
            self._rclpy.spin_once(self._node, timeout_sec=timeout_s)

    def _stop_requested(self) -> bool:
        with self._state_lock:
            task = self._active_task
        return task is not None and task.stop_requested

    def _create_publisher_locked(self) -> None:
        if self._publisher is None:
            self._publisher = self._node.create_publisher(
                self._twist_type,
                self._chassis_profile.cmd_vel_topic,
                1,
            )

    def _current_sample(self) -> _OdomSample | None:
        with self._state_lock:
            return self._current_sample_locked()

    def _current_sample_locked(self) -> _OdomSample | None:
        return self._sample

    def _is_fresh(self, sample: _OdomSample) -> bool:
        age_s = self._clock.now() - sample.timestamp_s
        return (
            sample.valid
            and age_s >= 0.0
            and age_s <= self._chassis_profile.odom_max_age_s
        )

    def _state_data(self, sample: _OdomSample | None) -> dict[str, Any]:
        if sample is None:
            return {
                "moving": False,
                "valid": False,
                "timestamp": self._clock.now(),
                "mode": "UNKNOWN",
                "sequence": 0,
            }
        mode = (
            "INVALID"
            if not sample.valid
            else "MOVING"
            if sample.moving
            else "IDLE"
        )
        return {
            "moving": sample.valid and sample.moving,
            "valid": sample.valid,
            "timestamp": sample.timestamp_s,
            "mode": mode,
            "sequence": sample.sequence,
        }

    def _ensure_open(self) -> None:
        if self._closing or self._closed:
            raise ChassisRuntimeError("chassis runtime is closed")

    def _monotonic(self) -> float:
        return self._clock.monotonic()

    def _motion_budget_s(self, distance_m: float, angle_deg: float) -> float:
        distance_s = (
            abs(distance_m) / self._chassis_profile.max_linear_speed_m_s
            if self._chassis_profile.max_linear_speed_m_s > 0.0
            else 0.0
        )
        angle_s = (
            abs(math.radians(angle_deg))
            / self._chassis_profile.max_angular_speed_rad_s
            if self._chassis_profile.max_angular_speed_rad_s > 0.0
            else 0.0
        )
        linear_accel_s = (
            self._chassis_profile.max_linear_speed_m_s
            / self._chassis_profile.max_linear_accel_m_s2
            if self._chassis_profile.max_linear_accel_m_s2 > 0.0
            else 0.0
        )
        angular_accel_s = (
            self._chassis_profile.max_angular_speed_rad_s
            / self._chassis_profile.max_angular_accel_rad_s2
            if self._chassis_profile.max_angular_accel_rad_s2 > 0.0
            else 0.0
        )
        motion_s = max(
            distance_s + linear_accel_s,
            angle_s + angular_accel_s,
        )
        return (
            self._chassis_profile.watchdog_timeout_s
            + self._chassis_profile.stop_settle_timeout_s
            + 4.0 * motion_s
        )

    def _ramp_speed(
        self,
        current_speed: float,
        remaining: float,
        max_speed: float,
        max_accel: float,
        elapsed_s: float,
    ) -> float:
        if abs(remaining) <= 1e-9 or max_speed <= 0.0:
            target_speed = 0.0
        elif max_accel <= 0.0:
            target_speed = math.copysign(max_speed, remaining)
        else:
            target_speed = math.copysign(
                min(
                    max_speed,
                    math.sqrt(2.0 * max_accel * abs(remaining)),
                ),
                remaining,
            )
        delta_s = target_speed - current_speed
        max_delta_s = max_accel * elapsed_s
        if max_accel <= 0.0 or abs(delta_s) <= max_delta_s:
            return target_speed
        return current_speed + math.copysign(max_delta_s, delta_s)

    def _forward_coordinate(self, x_m: float, y_m: float, yaw_rad: float) -> float:
        return x_m * math.cos(yaw_rad) + y_m * math.sin(yaw_rad)

    def _normalize_angle(self, angle_rad: float) -> float:
        return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi

    def _yaw_from_quaternion(
        self,
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> float:
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

    @staticmethod
    def _move_rejection(reason: str) -> RuntimeResult:
        return RuntimeResult(
            code=1,
            message=reason,
            data={"reason": reason, "state": "REJECTED"},
        )

    @staticmethod
    def _stop_result(stopped: bool, message: str) -> RuntimeResult:
        return RuntimeResult(
            code=0 if stopped else 1,
            message=message,
            data={"stopped": stopped},
        )
