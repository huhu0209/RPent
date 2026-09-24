from __future__ import annotations

import ast
import sys
from pathlib import Path
from typing import Any

import pytest

from robots.lynsense_real_box import live as live_package
from robots.lynsense_real_box.atomic_profile import (
    atomic_profile_hash,
    validate_atomic_capability_profile,
)
from robots.lynsense_real_box.live import composition_root
from robots.lynsense_real_box.live import chassis_runtime as chassis_runtime_module
from robots.lynsense_real_box.live.composition_root import (
    LiveCompositionDependencies,
    LiveCompositionError,
    build_live_atomic_toolkit,
)
from tests.unit_tests.robots.lynsense_real_box.test_atomic_adapter import (
    MutableClock,
    make_runtime,
)
from tests.unit_tests.robots.lynsense_real_box.test_atomic_profile import (
    live_profile,
    valid_v2_profile,
)
from tests.unit_tests.robots.lynsense_real_box.test_atomic_toolkit import (
    DashboardSink,
    EXPECTED_TOOLS,
    Memory,
)


class FakeOwnedRuntime:
    def __init__(
        self,
        profile: Any,
        clock: MutableClock,
        *,
        cancel_supported: bool = True,
        identity_changes: dict[str, Any] | None = None,
        close_order: list[str] | None = None,
    ) -> None:
        self._profile = profile
        self._cancel_supported = cancel_supported
        self._identity_changes = identity_changes or {}
        self._close_order = [] if close_order is None else close_order
        robot, _ = make_runtime(clock)
        self.arms = robot.arms
        self.grippers = robot.grippers
        self.waist = robot.waist
        self.force = object()
        self.released = False

    def identity(self) -> dict[str, Any]:
        binding = self._profile.runtime_binding
        assert binding is not None
        configured = {
            "site_id": binding.site_id,
            "robot_id": binding.robot_id,
            "instance": binding.lynrotcontrol_instance,
        }
        configured.update(self._identity_changes)
        return {"configured": configured}

    def service_identity(self) -> dict[str, Any]:
        return {"owned": True}

    def gripper_cancel_supported(self) -> bool:
        return self._cancel_supported

    def release(self) -> None:
        if self.released:
            raise AssertionError("owned runtime was released twice")
        self.released = True
        self._close_order.append("owned_runtime")


class FakeChassisTransport:
    def __init__(
        self,
        close_order: list[str] | None = None,
        identity_changes: dict[str, str] | None = None,
    ) -> None:
        self.closed = False
        self.read_count = 0
        self._identity_changes = identity_changes or {}
        self._close_order = [] if close_order is None else close_order

    def identity(self) -> dict[str, str]:
        identity = {
            "kind": "bounded_odom_cmd_vel",
            "odom_topic": "/odom",
            "cmd_vel_topic": "/cmd_vel",
        }
        identity.update(self._identity_changes)
        return identity

    def read(self) -> dict[str, Any]:
        self.read_count += 1
        return {
            "code": 0,
            "data": {
                "moving": False,
                "valid": True,
                "timestamp": 1000.0,
                "mode": "IDLE",
                "sequence": self.read_count,
            },
        }

    def navigate(self, goal: str) -> dict[str, Any]:
        del goal
        return {"code": 0, "data": {"state": "SUCCEEDED"}}

    def move_distance(self, distance_m: float, angle_deg: float) -> dict[str, Any]:
        del distance_m, angle_deg
        return {"code": 0, "data": {"state": "SUCCEEDED"}}

    def stop(self) -> dict[str, Any]:
        return {"code": 0, "data": {"stopped": True}}

    def close(self) -> None:
        if self.closed:
            raise AssertionError("chassis transport was closed twice")
        self.closed = True
        self._close_order.append("chassis")


class FakePerceptionTransport:
    def __init__(
        self,
        close_order: list[str] | None = None,
        identity_changes: dict[str, str] | None = None,
    ) -> None:
        self.closed = False
        self._identity_changes = identity_changes or {}
        self._close_order = [] if close_order is None else close_order

    def identity(self) -> dict[str, str]:
        identity = {
            "pose_topic": "/industrial_box/pose_base",
            "status_topic": "/industrial_box/status",
            "trigger_service": "/industrial_box/trigger",
            "expected_frame": "base_link",
        }
        identity.update(self._identity_changes)
        return identity

    def close(self) -> None:
        if self.closed:
            raise AssertionError("perception transport was closed twice")
        self.closed = True
        self._close_order.append("perception_transport")


class FakePerceptionClient:
    def __init__(self, close_order: list[str] | None = None) -> None:
        self.closed = False
        self._close_order = [] if close_order is None else close_order

    def detect_box(self) -> dict[str, Any]:
        return {"status": "ok", "frame": "base_link"}

    def close(self) -> None:
        if self.closed:
            raise AssertionError("perception client was closed twice")
        self.closed = True
        self._close_order.append("perception_client")


class RecordingDependencies:
    def __init__(
        self,
        *,
        cancel_supported: bool = True,
        identity_changes: dict[str, Any] | None = None,
        perception_client_error: Exception | None = None,
        chassis_identity_changes: dict[str, str] | None = None,
        perception_identity_changes: dict[str, str] | None = None,
    ) -> None:
        self.profile: Any = None
        self.cancel_supported = cancel_supported
        self.identity_changes = identity_changes
        self.perception_client_error = perception_client_error
        self.chassis_identity_changes = chassis_identity_changes
        self.perception_identity_changes = perception_identity_changes
        self.clock = MutableClock()
        self.close_order: list[str] = []
        self.owned_runtime: FakeOwnedRuntime | None = None
        self.chassis: FakeChassisTransport | None = None
        self.perception_transport: FakePerceptionTransport | None = None
        self.perception: FakePerceptionClient | None = None
        self.calls: list[str] = []

    def dependencies(self) -> LiveCompositionDependencies:
        return LiveCompositionDependencies(
            owned_runtime=self._owned_runtime,
            chassis=self._chassis,
            perception_transport=self._perception_transport,
            perception_client=self._perception_client,
            now_s=self.clock.now,
        )

    def _owned_runtime(self, profile: Any) -> FakeOwnedRuntime:
        self.calls.append("owned_runtime")
        self.profile = profile
        self.owned_runtime = FakeOwnedRuntime(
            profile,
            self.clock,
            cancel_supported=self.cancel_supported,
            identity_changes=self.identity_changes,
            close_order=self.close_order,
        )
        return self.owned_runtime

    def _chassis(self, profile: Any) -> FakeChassisTransport:
        del profile
        self.calls.append("chassis")
        self.chassis = FakeChassisTransport(
            self.close_order,
            self.chassis_identity_changes,
        )
        return self.chassis

    def _perception_transport(self, profile: Any) -> FakePerceptionTransport:
        del profile
        self.calls.append("perception_transport")
        self.perception_transport = FakePerceptionTransport(
            self.close_order,
            self.perception_identity_changes,
        )
        return self.perception_transport

    def _perception_client(
        self,
        profile: Any,
        transport: Any,
        now_s: Any,
    ) -> FakePerceptionClient:
        del profile, transport, now_s
        self.calls.append("perception_client")
        if self.perception_client_error is not None:
            raise self.perception_client_error
        self.perception = FakePerceptionClient(self.close_order)
        return self.perception


def live() -> Any:
    return validate_atomic_capability_profile(live_profile())


def test_composition_root_rejects_non_live_profile_before_resource_creation(tmp_path):
    recording = RecordingDependencies()

    with pytest.raises(LiveCompositionError, match="profile v2 mode live"):
        build_live_atomic_toolkit(
            profile=validate_atomic_capability_profile(valid_v2_profile()),
            output_dir=tmp_path,
            dashboard_events=DashboardSink(),
            memory=Memory(),
            dependencies=recording.dependencies(),
        )

    assert recording.calls == []


def test_composition_root_rejects_review_that_is_not_site_confirmed(tmp_path):
    profile = live()
    object.__setattr__(profile.review, "status", "approved")
    recording = RecordingDependencies()

    with pytest.raises(LiveCompositionError, match="site-confirmed"):
        build_live_atomic_toolkit(
            profile=profile,
            output_dir=tmp_path,
            dashboard_events=DashboardSink(),
            memory=Memory(),
            dependencies=recording.dependencies(),
        )

    assert recording.calls == []


def test_default_chassis_factory_uses_bounded_odom_chassis_runtime(monkeypatch):
    class FakeBoundedRuntime:
        def __init__(self, profile: Any, rclpy: Any) -> None:
            self.profile = profile
            self.rclpy = rclpy

    fake_rclpy = object()
    monkeypatch.setattr(
        chassis_runtime_module,
        "BoundedOdomCmdVelChassisRuntime",
        FakeBoundedRuntime,
    )
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)

    runtime = composition_root.default_chassis(live())

    assert isinstance(runtime, FakeBoundedRuntime)
    assert runtime.profile == live()
    assert runtime.rclpy is fake_rclpy


def test_live_composition_requires_a_box_carrying_chassis_profile(tmp_path):
    source = live_profile()
    source["chassis_profiles"]["retreat"]["carries_box"] = False
    source["carry_guards"].pop("retreat")
    profile = validate_atomic_capability_profile(source)
    recording = RecordingDependencies()

    with pytest.raises(
        LiveCompositionError,
        match="at least one box-carrying chassis profile",
    ):
        build_live_atomic_toolkit(
            profile=profile,
            output_dir=tmp_path,
            dashboard_events=DashboardSink(),
            memory=Memory(),
            dependencies=recording.dependencies(),
        )

    assert recording.calls == []


def test_box_carrying_chassis_profile_requires_a_runtime_guard(tmp_path):
    profile = live()
    object.__setattr__(profile, "carry_guards", {})
    recording = RecordingDependencies()

    with pytest.raises(
        LiveCompositionError,
        match="carry guards: retreat",
    ):
        build_live_atomic_toolkit(
            profile=profile,
            output_dir=tmp_path,
            dashboard_events=DashboardSink(),
            memory=Memory(),
            dependencies=recording.dependencies(),
        )

    assert recording.calls == []


def test_composition_root_rejects_runtime_without_gripper_stop_and_releases_it(tmp_path):
    recording = RecordingDependencies(cancel_supported=False)

    with pytest.raises(LiveCompositionError, match="gripper stop API"):
        build_live_atomic_toolkit(
            profile=live(),
            output_dir=tmp_path,
            dashboard_events=DashboardSink(),
            memory=Memory(),
            dependencies=recording.dependencies(),
        )

    assert recording.calls == ["owned_runtime"]
    assert recording.owned_runtime is not None
    assert recording.owned_runtime.released is True
    assert recording.chassis is None
    assert recording.perception_transport is None


def test_failed_perception_composition_closes_partial_resources(tmp_path):
    recording = RecordingDependencies(
        perception_client_error=OSError("perception unavailable")
    )

    with pytest.raises(OSError, match="perception unavailable"):
        build_live_atomic_toolkit(
            profile=live(),
            output_dir=tmp_path,
            dashboard_events=DashboardSink(),
            memory=Memory(),
            dependencies=recording.dependencies(),
        )

    assert recording.calls == [
        "owned_runtime",
        "chassis",
        "perception_transport",
        "perception_client",
    ]
    assert recording.perception_transport is not None
    assert recording.perception_transport.closed is True
    assert recording.chassis is not None
    assert recording.chassis.closed is True
    assert recording.owned_runtime is not None
    assert recording.owned_runtime.released is True
    assert recording.close_order == [
        "perception_transport",
        "chassis",
        "owned_runtime",
    ]


def test_failed_adapter_preflight_closes_every_partial_resource(tmp_path):
    recording = RecordingDependencies(
        identity_changes={"site_id": "another-site"},
    )

    with pytest.raises(LiveCompositionError, match="adapter preflight failed"):
        build_live_atomic_toolkit(
            profile=live(),
            output_dir=tmp_path,
            dashboard_events=DashboardSink(),
            memory=Memory(),
            dependencies=recording.dependencies(),
        )

    assert recording.owned_runtime is not None
    assert recording.owned_runtime.released is True
    assert recording.perception is not None
    assert recording.perception.closed is True
    assert recording.chassis is not None
    assert recording.chassis.closed is True


def test_chassis_transport_identity_must_match_live_profile(tmp_path):
    recording = RecordingDependencies(
        chassis_identity_changes={"odom_topic": "/robot/odom"},
    )

    with pytest.raises(
        LiveCompositionError,
        match="adapter preflight failed: runtime_identity_invalid",
    ):
        build_live_atomic_toolkit(
            profile=live(),
            output_dir=tmp_path,
            dashboard_events=DashboardSink(),
            memory=Memory(),
            dependencies=recording.dependencies(),
        )

    assert recording.owned_runtime is not None
    assert recording.owned_runtime.released is True
    assert recording.perception is not None
    assert recording.perception.closed is True
    assert recording.chassis is not None
    assert recording.chassis.closed is True


def test_perception_transport_identity_must_match_live_profile(tmp_path):
    recording = RecordingDependencies(
        perception_identity_changes={"expected_frame": "camera_link"},
    )

    with pytest.raises(
        ValueError,
        match="perception identity rejected: perception_identity_mismatch",
    ):
        build_live_atomic_toolkit(
            profile=live(),
            output_dir=tmp_path,
            dashboard_events=DashboardSink(),
            memory=Memory(),
            dependencies=recording.dependencies(),
        )

    assert recording.owned_runtime is not None
    assert recording.owned_runtime.released is True
    assert recording.perception is not None
    assert recording.perception.closed is True
    assert recording.chassis is not None
    assert recording.chassis.closed is True


def test_successful_composition_exposes_exact_live_atomic_tools_and_close_order(tmp_path):
    recording = RecordingDependencies()
    toolkit = build_live_atomic_toolkit(
        profile=live(),
        output_dir=tmp_path,
        dashboard_events=DashboardSink(),
        memory=Memory(),
        dependencies=recording.dependencies(),
    )

    assert [item["name"] for item in toolkit.get_tools_spec()] == EXPECTED_TOOLS
    assert toolkit.execute_tool("read_capability_state", {}).result["status"] == "ok"
    assert toolkit._profile_sha256 == atomic_profile_hash(live())
    original_adapter_close = toolkit._adapter.close

    def close_adapter() -> None:
        recording.close_order.append("adapter")
        original_adapter_close()

    toolkit._adapter.close = close_adapter
    original_evidence_close = toolkit._evidence.close

    def close_evidence() -> None:
        recording.close_order.append("evidence")
        original_evidence_close()

    toolkit._evidence.close = close_evidence
    toolkit.close()
    toolkit.close()

    assert recording.close_order == [
        "perception_client",
        "chassis",
        "adapter",
        "owned_runtime",
        "evidence",
    ]
    assert recording.perception is not None
    assert recording.perception.closed is True
    assert recording.chassis is not None
    assert recording.chassis.closed is True
    assert recording.owned_runtime is not None
    assert recording.owned_runtime.released is True


def test_live_close_failure_retry_does_not_repeat_owned_release(tmp_path):
    recording = RecordingDependencies()
    toolkit = build_live_atomic_toolkit(
        profile=live(),
        output_dir=tmp_path,
        dashboard_events=DashboardSink(),
        memory=Memory(),
        dependencies=recording.dependencies(),
    )
    original_evidence_close = toolkit._evidence.close
    failed_once = [False]

    def fail_evidence_close_once() -> None:
        if not failed_once[0]:
            failed_once[0] = True
            raise OSError("evidence temporarily unavailable")
        original_evidence_close()

    toolkit._evidence.close = fail_evidence_close_once
    with pytest.raises(RuntimeError, match="live atomic Toolkit close failed"):
        toolkit.close()
    toolkit.close()

    assert recording.perception is not None
    assert recording.perception.closed is True
    assert recording.chassis is not None
    assert recording.chassis.closed is True
    assert recording.owned_runtime is not None
    assert recording.owned_runtime.released is True
    assert recording.close_order.count("owned_runtime") == 1


def test_failed_owned_release_is_terminal_for_automatic_close_retry(tmp_path):
    recording = RecordingDependencies()
    toolkit = build_live_atomic_toolkit(
        profile=live(),
        output_dir=tmp_path,
        dashboard_events=DashboardSink(),
        memory=Memory(),
        dependencies=recording.dependencies(),
    )
    assert recording.owned_runtime is not None
    release_calls: list[str] = []

    def fail_release() -> None:
        release_calls.append("release")
        raise OSError("release disposition unknown")

    recording.owned_runtime.release = fail_release
    with pytest.raises(RuntimeError, match="release disposition unknown"):
        toolkit.close()
    with pytest.raises(
        RuntimeError,
        match="owned runtime release is unresolved",
    ):
        toolkit.close()

    assert release_calls == ["release"]
    assert recording.perception is not None
    assert recording.perception.closed is True
    assert recording.chassis is not None
    assert recording.chassis.closed is True


def test_composition_module_and_package_import_do_not_load_robot_transports():
    source = Path(composition_root.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    top_level_imports: set[str] = set()
    for node in tree.body:
        if isinstance(node, ast.Import):
            top_level_imports.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            top_level_imports.add((node.module or "").split(".")[0])
    assert not top_level_imports & {"rclpy", "rosidl", "lynrotcontrol"}
    assert live_package.__name__ == "robots.lynsense_real_box.live"
