from __future__ import annotations

import dataclasses
import sys
from types import SimpleNamespace

import pytest

from robots.lynsense_real_box import commissioning_binding as binding
from robots.lynsense_real_box.commissioning_contract import (
    ResultNormalizationError,
    ServiceOwnershipUnavailable,
)
from robots.lynsense_real_box.commissioning_binding import (
    FakeCommissioningBinding,
    RealLynrotControlBinding,
)

GROUPS = (
    "left_arm", "right_arm", "left_gripper", "right_gripper",
    "waist_lift", "left_force", "right_force",
)
FORCE_UNITS = ("N", "N", "N", "N.m", "N.m", "N.m")


def _manifest_without_unresolvable_inventory(manifest):
    return dataclasses.replace(manifest, source_artifact_sha256={}, config_sha256={})


class RecordingGate:
    def __init__(self, runtime=None):
        self.calls = []
        self.runtime = runtime

    def initialize_under_guard(
        self, *, instance, site_id, robot_id, endpoints, endpoint_scope_sha256,
        approved_effects, ros_domain_id,
    ):
        self.calls.append({
            "instance": instance, "site_id": site_id, "robot_id": robot_id,
            "endpoints": endpoints, "endpoint_scope_sha256": endpoint_scope_sha256,
            "approved_effects": approved_effects, "ros_domain_id": ros_domain_id,
        })
        if self.runtime is None:
            raise ServiceOwnershipUnavailable("actual spawn/reuse is not guarded")
        return self.runtime


def _payload(group="left_arm", **overrides):
    value = {
        "group": group, "values": [0.1, 0.2], "units": "rad",
        "frame": "joint_space", "side": "left", "axes": ["joint_1", "joint_2"],
        "timestamp": None, "sequence": 3, "provenance": "device_sequence",
    }
    value.update(overrides)
    return value


class FakeOwnedRuntime:
    def __init__(self):
        self.access = []
        self.released = 0

    def arm_identity(self, side):
        self.access.append(("arm_identity", side))
        return {
            "model": "arm", "ip": f"127.0.0.{1 if side == 'left' else 2}",
            "side": side, "axes": 6,
        }

    def read_state(self, group):
        self.access.append(("read_state", group))
        if group in ("left_force", "right_force"):
            side = "left" if group.startswith("left") else "right"
            return _payload(group, values=[0.0] * 6, units=list(FORCE_UNITS), side=side)
        if group in ("left_gripper", "right_gripper"):
            side = "left" if group.startswith("left") else "right"
            return _payload(group, values=[0.25], axes=["gripper_joint"],
                            frame="gripper_joint", side=side)
        if group == "waist_lift":
            return _payload(group, values=[12.0], units="mm", axes=["lift"], frame="lift", side=None)
        return _payload(group, side="right" if group == "right_arm" else "left")

    def service_identity(self):
        self.access.append(("service_identity", None))
        return {"service_pid": 123, "service_start_id": "start", "service_epoch": "epoch"}

    def operator_verification(self):
        return {"left_arm": "operator-verified"}

    def owned_ros_resources(self):
        self.access.append(("owned_ros_resources", None))
        return ("left-arm-service", "right-arm-service")

    def release(self):
        self.released += 1

    def __getattr__(self, name):
        if name.startswith("_"):
            raise AttributeError(name)
        self.access.append(("rejected", name))
        raise AttributeError(name)


def test_fake_binding_is_offline_and_traces_release_once(valid_manifest):
    commissioning = FakeCommissioningBinding()
    runtime = commissioning.initialize(valid_manifest.instance)
    [runtime.read_group(group) for group in GROUPS]
    runtime.release()
    with pytest.raises(ResultNormalizationError):
        runtime.release()
    assert commissioning.operation_trace[0] == ("initialize", valid_manifest.instance)
    assert commissioning.operation_trace[-1] == ("release", "fixed")
    assert commissioning.operation_trace.count(("release", "fixed")) == 1


def test_bad_artifact_blocks_gate_before_dependency_import(
    valid_manifest, tmp_path, monkeypatch
):
    monkeypatch.delitem(sys.modules, "lynrotcontrol", raising=False)
    request = SimpleNamespace(
        ownership_protocol_artifact=tmp_path / "missing.json",
        ownership_protocol_sha256="0" * 64,
        ownership_protocol_id="lynrotcontrol.service-ownership",
        ownership_protocol_version=1,
        ownership_read_allowlist_sha256="3" * 64,
        endpoint_authority_root=valid_manifest.endpoint_authority_root,
        ownership_runtime_protocol=15,
        site_id=valid_manifest.site_id, robot_id=valid_manifest.robot_id,
        approved_effects=valid_manifest.approved_effects,
    )
    with pytest.raises(ServiceOwnershipUnavailable, match="ownership protocol artifact"):
        binding.build_effectful_gate(request)
    assert "lynrotcontrol" not in sys.modules


def test_exact_artifact_calls_dependency_once_and_validates_request(
    valid_manifest, monkeypatch
):
    raw = FakeOwnedRuntime()
    imported = []
    class CanonicalEndpoint:
        def __init__(self, subsystem, endpoint_id, transport, locator):
            self.arguments = (subsystem, endpoint_id, transport, locator)

    calls = []
    def initialize(*args, **kwargs):
        calls.append((args, kwargs))
        return raw
    module = SimpleNamespace(
        CanonicalEndpoint=CanonicalEndpoint, initialize_under_ownership=initialize
    )
    monkeypatch.setattr(
        binding.importlib, "import_module",
        lambda name: imported.append(name) or module,
    )
    gate = binding.build_effectful_gate(valid_manifest)
    runtime = gate.initialize_under_guard(
        instance=valid_manifest.instance, site_id=valid_manifest.site_id,
        robot_id=valid_manifest.robot_id, endpoints=tuple(valid_manifest.endpoints),
        endpoint_scope_sha256=binding.endpoint_scope_digest(valid_manifest),
        approved_effects=valid_manifest.approved_effects,
        ros_domain_id=valid_manifest.ros_domain_id,
    )
    assert runtime is raw
    assert imported == ["lynrotcontrol.interfaces.ownership"]
    assert calls[0][1]["expected_endpoints"][0].arguments == (
        "arm", "left-arm", "tcp", "robot-one/left"
    )


def test_dependency_exception_maps_to_ownership_unavailable(valid_manifest, monkeypatch):
    def fail(*args, **kwargs):
        raise NotImplementedError("old dependency")

    module = SimpleNamespace(
        CanonicalEndpoint=lambda *args: None, initialize_under_ownership=fail
    )
    monkeypatch.setattr(binding.importlib, "import_module", lambda name: module)
    gate = binding.build_effectful_gate(valid_manifest)
    with pytest.raises(ServiceOwnershipUnavailable, match="NotImplementedError"):
        gate.initialize_under_guard(
            instance=valid_manifest.instance, site_id=valid_manifest.site_id,
            robot_id=valid_manifest.robot_id, endpoints=tuple(valid_manifest.endpoints),
            endpoint_scope_sha256=binding.endpoint_scope_digest(valid_manifest),
            approved_effects=valid_manifest.approved_effects,
            ros_domain_id=valid_manifest.ros_domain_id,
        )


def test_real_binding_passes_complete_ownership_arguments(valid_manifest):
    gate = RecordingGate(FakeOwnedRuntime())
    runtime = RealLynrotControlBinding(
        gate=gate, manifest=_manifest_without_unresolvable_inventory(valid_manifest)
    ).initialize(valid_manifest.instance)
    assert gate.calls[0]["site_id"] == valid_manifest.site_id
    assert gate.calls[0]["robot_id"] == valid_manifest.robot_id
    assert gate.calls[0]["endpoint_scope_sha256"] == binding.endpoint_scope_digest(valid_manifest)
    assert runtime.service_identity().pid == 123


def test_narrow_facade_never_traverses_raw_objects(valid_manifest):
    raw = FakeOwnedRuntime()
    manifest = _manifest_without_unresolvable_inventory(valid_manifest)
    runtime = RealLynrotControlBinding(
        gate=RecordingGate(raw), manifest=manifest
    ).initialize(manifest.instance)
    identity = runtime.identity()
    assert identity.device_reported["arms"]["left"]["axes"] == 6
    assert identity.operator_verified == {}
    for group in GROUPS:
        runtime.read_group(group)
    runtime.owned_ros_resources()
    assert all(item[0] != "rejected" for item in raw.access)
    assert not any(
        name in {"arms", "grippers", "waist", "forces", "robot"}
        for name, _ in raw.access
    )


@pytest.mark.parametrize(
    ("group", "units"),
    [
        ("left_arm", "rad"), ("right_arm", "rad"),
        ("left_gripper", "rad"), ("right_gripper", "rad"),
        ("waist_lift", "mm"), ("left_force", FORCE_UNITS),
        ("right_force", FORCE_UNITS),
    ],
)
def test_fixed_units_are_enforced(valid_manifest, group, units):
    manifest = _manifest_without_unresolvable_inventory(valid_manifest)
    runtime = RealLynrotControlBinding(
        gate=RecordingGate(FakeOwnedRuntime()), manifest=manifest
    ).initialize(manifest.instance)
    assert runtime.read_group(group).payload["units"] == units


def test_force_rejects_scalar_unit_and_wrong_length(valid_manifest):
    raw = FakeOwnedRuntime()
    raw.read_state = lambda group: _payload(
        "left_force", values=[0.0], units="N", side="left"
    )
    manifest = _manifest_without_unresolvable_inventory(valid_manifest)
    runtime = RealLynrotControlBinding(
        gate=RecordingGate(raw), manifest=manifest
    ).initialize(manifest.instance)
    with pytest.raises(ResultNormalizationError):
        runtime.read_group("left_force")
