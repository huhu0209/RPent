"""Fail-closed composition root for reviewed Robot One live capabilities."""

from __future__ import annotations

import hashlib
import os
import platform
from time import time as wall_time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from rpent.dashboard.events import DashboardEventSink
from rpent.memory import MemoryManager

from robots.lynsense_real_box.atomic_adapter import LynrotControlAtomicAdapter
from robots.lynsense_real_box.atomic_profile import AtomicCapabilityProfile
from robots.lynsense_real_box.evidence import EvidenceRecorder
from robots.lynsense_real_box.live.toolkit import LynsenseLiveAtomicToolkit


class LiveCompositionError(RuntimeError):
    """Raised when live resources cannot be safely composed."""


class OwnedRuntimeLike(Protocol):
    """The narrow dependency shape required from LynrotControl."""

    arms: Any
    grippers: Any
    waist: Any
    force: Any

    def identity(self) -> Any: ...
    def service_identity(self) -> dict[str, Any]: ...
    def gripper_cancel_supported(self) -> bool: ...
    def release(self) -> None: ...


@dataclass(frozen=True, slots=True)
class LiveCompositionDependencies:
    """Injected factories so tests never import ROS or LynrotControl."""

    owned_runtime: Callable[[AtomicCapabilityProfile], OwnedRuntimeLike]
    chassis: Callable[[AtomicCapabilityProfile], Any]
    perception_transport: Callable[[AtomicCapabilityProfile], Any]
    perception_client: Callable[[AtomicCapabilityProfile], Any, Callable[[], float], Any]
    now_s: Callable[[], float] = wall_time


class RuntimeIdentityReader:
    """Produce the exact fresh receipt checked by the atomic adapter."""

    def __init__(
        self,
        *,
        profile: AtomicCapabilityProfile,
        owned_runtime: OwnedRuntimeLike,
        chassis_identity: Any,
        now_s: Callable[[], float],
    ) -> None:
        self._profile = profile
        self._owned_runtime = owned_runtime
        self._chassis_identity = chassis_identity
        self._now_s = now_s

    def read(self) -> dict[str, Any]:
        binding = self._profile.runtime_binding
        chassis = self._profile.chassis_runtime
        assert binding is not None
        assert chassis is not None
        identity = self._owned_runtime.identity()
        configured = identity["configured"]
        if (
            configured["site_id"] != binding.site_id
            or configured["robot_id"] != binding.robot_id
            or configured["instance"] != binding.lynrotcontrol_instance
        ):
            return {
                "code": 5002,
                "message": "owned runtime identity mismatch",
            }
        expected_chassis = {
            "kind": chassis.kind,
            "odom_topic": chassis.odom_topic,
            "cmd_vel_topic": chassis.cmd_vel_topic,
        }
        try:
            observed_chassis = self._chassis_identity.identity()
        except Exception:
            return {
                "code": 5002,
                "message": "chassis transport identity unavailable",
            }
        if observed_chassis != expected_chassis:
            return {
                "code": 5002,
                "message": "chassis transport identity mismatch",
            }
        return {
            "code": 0,
            "data": {
                "valid": True,
                "timestamp": self._now_s(),
                "profile_sha256": _profile_sha256(self._profile),
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
            },
        }


class PerceptionIdentityReader:
    """Produce the exact perception receipt checked by the Toolkit."""

    def __init__(
        self,
        *,
        profile: AtomicCapabilityProfile,
        transport: Any,
        now_s: Callable[[], float],
    ) -> None:
        self._profile = profile
        self._transport = transport
        self._now_s = now_s

    def read(self) -> dict[str, Any]:
        binding = self._profile.perception_binding
        assert binding is not None
        expected = {
            "pose_topic": binding.pose_topic,
            "status_topic": binding.status_topic,
            "trigger_service": binding.trigger_service,
            "expected_frame": binding.expected_frame,
        }
        try:
            observed = self._transport.identity()
        except Exception:
            return {
                "profile_sha256": _profile_sha256(self._profile),
                "error": "perception_transport_identity_unavailable",
            }
        if observed != expected:
            return {
                "profile_sha256": _profile_sha256(self._profile),
                "error": "perception_transport_identity_mismatch",
            }
        return {
            "profile_sha256": _profile_sha256(self._profile),
            **observed,
        }


def default_owned_runtime(profile: AtomicCapabilityProfile) -> OwnedRuntimeLike:
    """Initialize the reviewed LynrotControl instance under endpoint ownership."""

    binding = profile.runtime_binding
    assert binding is not None
    if platform.node() != binding.execution_host:
        raise LiveCompositionError("execution host does not match live profile")
    if os.environ.get("ROS_DOMAIN_ID") != str(binding.ros_domain_id):
        raise LiveCompositionError("ROS domain does not match live profile")

    from lynrotcontrol.implementation.config import ROOT as LYNROTCONTROL_ROOT
    from lynrotcontrol.interfaces.ownership import initialize_under_ownership
    from lynrotcontrol.lynarmcontrol.implementation import service_ownership
    from lynrotcontrol.lynarmcontrol.implementation.endpoint_inventory import (
        instance_endpoints,
    )

    config = LYNROTCONTROL_ROOT / "config" / f"{binding.lynrotcontrol_instance}.yaml"
    try:
        observed = hashlib.sha256(config.read_bytes()).hexdigest()
    except OSError as error:
        raise LiveCompositionError(
            "cannot read hashed LynrotControl config"
        ) from error
    if observed != binding.lynrotcontrol_config_sha256:
        raise LiveCompositionError(
            "LynrotControl config digest does not match live profile"
        )
    endpoints = instance_endpoints(binding.lynrotcontrol_instance)
    scope = service_ownership.endpoint_scope_digest(
        binding.site_id,
        binding.robot_id,
        endpoints,
    )
    return initialize_under_ownership(
        binding.lynrotcontrol_instance,
        site_id=binding.site_id,
        robot_id=binding.robot_id,
        endpoint_scope_sha256=scope,
        ros_domain_id=binding.ros_domain_id,
        approved_effects=service_ownership.APPROVED_EFFECTS,
        expected_endpoints=endpoints,
    )


def default_chassis(profile: AtomicCapabilityProfile) -> Any:
    import rclpy

    from robots.lynsense_real_box.live.chassis_runtime import (
        BoundedOdomCmdVelChassisRuntime,
    )

    return BoundedOdomCmdVelChassisRuntime(profile, rclpy)


def default_perception_transport(profile: AtomicCapabilityProfile) -> Any:
    import rclpy
    import geometry_msgs.msg as geometry_msgs
    import std_msgs.msg as std_msgs
    import std_srvs.srv as std_srvs

    from robots.lynsense_real_box.live.perception_transport import (
        LivePerceptionTransport,
    )

    assert profile.perception_binding is not None
    return LivePerceptionTransport(
        profile=profile,
        rclpy_module=rclpy,
        geometry_msgs_module=geometry_msgs,
        std_msgs_module=std_msgs,
        std_srvs_module=std_srvs,
    )


def default_perception_client(
    profile: AtomicCapabilityProfile,
    transport: Any,
    now_s: Callable[[], float],
) -> Any:
    from robots.lynsense_real_box.perception_client import BoxPerceptionClient

    return BoxPerceptionClient(
        profile=profile,
        transport=transport,
        now_s=now_s,
    )


DEFAULT_LIVE_DEPENDENCIES = LiveCompositionDependencies(
    owned_runtime=default_owned_runtime,
    chassis=default_chassis,
    perception_transport=default_perception_transport,
    perception_client=default_perception_client,
)


def build_live_atomic_toolkit(
    *,
    profile: AtomicCapabilityProfile,
    output_dir: Path,
    dashboard_events: DashboardEventSink,
    memory: MemoryManager,
    dependencies: LiveCompositionDependencies = DEFAULT_LIVE_DEPENDENCIES,
) -> LynsenseLiveAtomicToolkit:
    """Construct exactly one live Toolkit or close every partial resource."""

    _require_reviewed_live_profile(profile)
    owned_runtime: OwnedRuntimeLike | None = None
    chassis_transport: Any = None
    perception_transport: Any = None
    perception: Any = None
    evidence: EvidenceRecorder | None = None
    adapter: LynrotControlAtomicAdapter | None = None
    toolkit: LynsenseLiveAtomicToolkit | None = None
    try:
        owned_runtime = dependencies.owned_runtime(profile)
        if not owned_runtime.gripper_cancel_supported():
            raise LiveCompositionError("gripper stop API is unavailable")
        chassis_transport = dependencies.chassis(profile)
        perception_transport = dependencies.perception_transport(profile)
        perception = dependencies.perception_client(
            profile,
            perception_transport,
            dependencies.now_s,
        )
        evidence = EvidenceRecorder(
            output_dir / "events-lynsense_real_box_atomic.jsonl",
            profile,
        )
        adapter = LynrotControlAtomicAdapter(
            profile=profile,
            robot=owned_runtime,
            chassis=chassis_transport,
            runtime_identity=RuntimeIdentityReader(
                profile=profile,
                owned_runtime=owned_runtime,
                chassis_identity=chassis_transport,
                now_s=dependencies.now_s,
            ),
            now_s=dependencies.now_s,
            live_dispatch_enabled=True,
        )
        connection = adapter.connect()
        if connection.get("status") != "ok":
            raise LiveCompositionError(
                f"live adapter preflight failed: {connection.get('reason')}"
            )
        toolkit = LynsenseLiveAtomicToolkit(
            adapter=adapter,
            perception=perception,
            perception_identity=PerceptionIdentityReader(
                profile=profile,
                transport=perception_transport,
                now_s=dependencies.now_s,
            ),
            chassis_transport=chassis_transport,
            owned_runtime=owned_runtime,
            evidence=evidence,
            dashboard_events=dashboard_events,
            memory=memory,
            profile=profile,
        )
        return toolkit
    except BaseException as error:
        cleanup_errors = _close_partial(
            toolkit=toolkit,
            adapter=adapter,
            perception=perception if perception is not None else perception_transport,
            chassis_transport=chassis_transport,
            owned_runtime=owned_runtime,
            evidence=evidence,
        )
        if cleanup_errors:
            raise LiveCompositionError(
                "live construction failed and cleanup was incomplete: "
                + "; ".join(str(item) for item in cleanup_errors)
            ) from error
        raise


def _require_reviewed_live_profile(profile: AtomicCapabilityProfile) -> None:
    if profile.version < 2 or profile.mode != "live":
        raise LiveCompositionError("live composition requires profile v2 mode live")
    if (
        profile.runtime_binding is None
        or profile.chassis_runtime is None
        or profile.gripper_runtime is None
        or profile.perception_binding is None
        or profile.subsystem_freshness is None
    ):
        raise LiveCompositionError("live profile runtime binding is incomplete")
    if (
        profile.review.status != "site_confirmed"
        or not profile.review.reviewer
        or not profile.review.safety_evidence
    ):
        raise LiveCompositionError("live profile review is not site-confirmed")
    carrying_profiles = [
        name
        for name, item in profile.chassis_profiles.items()
        if item.carries_box
    ]
    if not carrying_profiles:
        raise LiveCompositionError(
            "live profile must declare at least one box-carrying chassis profile"
        )
    missing_guards = [
        name for name in carrying_profiles if name not in profile.carry_guards
    ]
    if missing_guards:
        raise LiveCompositionError(
            "box-carrying chassis profiles require runtime carry guards: "
            + ", ".join(sorted(missing_guards))
        )


def _close_partial(
    *,
    toolkit: LynsenseLiveAtomicToolkit | None,
    adapter: LynrotControlAtomicAdapter | None,
    perception: Any,
    chassis_transport: Any,
    owned_runtime: OwnedRuntimeLike | None,
    evidence: EvidenceRecorder | None,
) -> list[BaseException]:
    errors: list[BaseException] = []
    if toolkit is not None:
        try:
            toolkit.close()
            return errors
        except BaseException as error:
            errors.append(error)
            return errors
    for close in (
        getattr(perception, "close", None),
        getattr(chassis_transport, "close", None),
        getattr(adapter, "close", None),
        getattr(owned_runtime, "release", None),
        getattr(evidence, "close", None),
    ):
        if close is None:
            continue
        try:
            close()
        except BaseException as error:
            errors.append(error)
    return errors


def _profile_sha256(profile: AtomicCapabilityProfile) -> str:
    from robots.lynsense_real_box.atomic_profile import atomic_profile_hash

    return atomic_profile_hash(profile)
