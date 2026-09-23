from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone

import pytest

from robots.lynsense_real_box.commissioning_contract import (
    EndpointIdentity, IdentityEvidence, ResourceDisposition, ServiceEvidence,
    StateEvidence, InitializationWorkerRequest,
)
from robots.lynsense_real_box.commissioning_worker import (
    REQUIRED_GROUPS, SubprocessInitializationSupervisor, _child_main,
    request_to_dict, run_worker, worker_result_from_dict,
    worker_result_to_dict,
)


ARTIFACT_BYTES = (
    '{\n'
    '  "endpoint_authority_root": "/tmp/lynrotcontrol-authority-1000",\n'
    '  "protocol_id": "lynrotcontrol.service-ownership",\n'
    '  "protocol_version": 1,\n'
    '  "read_allowlist_sha256": "38015e139b4b7ae2c0ca7f2261be2a90cb4d40ee12d35420875c6d150962ceee",\n'
    '  "runtime_protocol": 15\n'
    '}\n'
)
ARTIFACT_DIGEST = hashlib.sha256(ARTIFACT_BYTES.encode()).hexdigest()
OPERATOR = {"left_arm": "operator-verified", "right_arm": "operator-verified"}


class Runtime:
    def __init__(self):
        self.trace = []
        self.sequence = 0
        self.release_error = None

    def service_identity(self):
        self.trace.append("service_identity")
        return ServiceEvidence(123, "start-1", "epoch-1")

    def operator_verification(self):
        self.trace.append("operator_verification")
        return dict(OPERATOR)

    def identity(self):
        self.trace.append("identity")
        return IdentityEvidence(
            {"site_id": "site-a", "robot_id": "robot-one", "instance": "instance"},
            {"arms": {
                side: {"side": side, "model": "arm", "ip": "127.0.0.1", "axes": 6}
                for side in ("left", "right")
            }},
            {},
        )

    def read_state(self, group):
        self.trace.append(group)
        self.sequence += 1
        if "force" in group:
            units = ("N", "N", "N", "N.m", "N.m", "N.m")
        elif group == "waist_lift":
            units = "mm"
        else:
            units = "rad"
        values = [0.0] * 6 if "force" in group else [0.0]
        return {
            "group": group, "values": values, "units": units, "frame": "fixed",
            "side": "left" if group.startswith("left") else "right" if group.startswith("right") else None,
            "axes": ["value"], "timestamp": None, "sequence": self.sequence,
            "provenance": "device_sequence",
        }

    def owned_ros_resources(self):
        self.trace.append("owned_ros_resources")
        return ("left-arm-service", "right-arm-service")

    def release(self):
        self.trace.append("release")
        if self.release_error is not None:
            raise RuntimeError(self.release_error)


class Gate:
    def __init__(self, runtime):
        self.runtime, self.calls = runtime, []

    def initialize_under_guard(self, **kwargs):
        self.calls.append(kwargs)
        return self.runtime


def _request(tmp_path):
    source = tmp_path / "worker-source.py"
    protocol = tmp_path / "OWNERSHIP_PROTOCOL.json"
    config = tmp_path / "worker-config.yaml"
    source.write_text("source", encoding="utf-8")
    protocol.write_text(ARTIFACT_BYTES, encoding="utf-8")
    config.write_text("config", encoding="utf-8")
    initialization_deadline = __import__("time").monotonic() + 30
    return InitializationWorkerRequest(
        "attempt-1", "a" * 64, "b" * 64, "instance", 3,
        (EndpointIdentity("arm", "left_arm", "tcp", "left"),),
        ("initialize_configuration", "fixed_read_ros_resource_creation",
         "fixed_read_ros_parameter_query"),
        REQUIRED_GROUPS, 2,
        initialization_deadline, initialization_deadline + 5,
        tmp_path / "attempt-1.result.json",
        (source, protocol, config),
        {
            str(source): hashlib.sha256(source.read_bytes()).hexdigest(),
            str(protocol): ARTIFACT_DIGEST,
        },
        "site-a", "robot-one", protocol, ARTIFACT_DIGEST,
        "lynrotcontrol.service-ownership", 1,
        "38015e139b4b7ae2c0ca7f2261be2a90cb4d40ee12d35420875c6d150962ceee",
        "/tmp/lynrotcontrol-authority-1000", 15, dict(OPERATOR),
        {str(config): hashlib.sha256(config.read_bytes()).hexdigest()},
    )


def test_worker_initializes_collects_reads_then_releases(tmp_path):
    runtime = Runtime()
    result = run_worker(_request(tmp_path), Gate(runtime))
    assert result.status == "settled_success"
    assert result.disposition.status == "settled"
    assert len(result.observations) == 2 * len(REQUIRED_GROUPS)
    assert runtime.trace[0] == "service_identity"
    assert runtime.trace[-1] == "release"


def test_worker_default_gate_fails_closed_without_importing_driver(
    tmp_path, monkeypatch
):
    import sys
    monkeypatch.delitem(sys.modules, "lynrotcontrol", raising=False)
    result = run_worker(_request(tmp_path))
    assert result.status == "ownership_unavailable"
    assert result.observations == ()
    assert "lynrotcontrol" not in sys.modules


def test_worker_result_round_trips_through_json_protocol(tmp_path):
    result = run_worker(_request(tmp_path), Gate(Runtime()))
    encoded = worker_result_to_dict(result)
    restored = worker_result_from_dict(json.loads(json.dumps(encoded)))
    assert restored == result
    assert not {"token", "token_hex", "token_digest"}.intersection(encoded)


def test_late_initialization_performs_no_identity_reads_or_release(
    tmp_path, monkeypatch
):
    import robots.lynsense_real_box.commissioning_worker as worker
    runtime = Runtime()
    request = _request(tmp_path)
    monkeypatch.setattr(
        worker.time, "monotonic", lambda: request.deadline_monotonic + 1
    )
    result = worker.run_worker(request, Gate(runtime))
    assert result.status == "late_settled"
    assert runtime.trace == ["service_identity", "owned_ros_resources", "service_identity"]
    assert "release" not in runtime.trace


def test_subprocess_worker_returns_serialized_fail_closed_result(tmp_path):
    request = _request(tmp_path)
    result = SubprocessInitializationSupervisor().run_worker(request)
    assert result.status == "ownership_unavailable"
    assert request.result_path.is_file()
    assert not request.result_path.with_name(f".{request.attempt_id}.request.json").exists()


def test_worker_request_rejects_noncanonical_fields(tmp_path):
    data = request_to_dict(_request(tmp_path))
    data["required_groups"] = ["left_arm"]
    from robots.lynsense_real_box.commissioning_worker import request_from_dict
    with pytest.raises(ValueError, match="fixed protocol"):
        request_from_dict(data)


def test_worker_rechecks_artifacts_before_gate(tmp_path):
    request = _request(tmp_path)
    request.source_artifacts[0].write_text("changed", encoding="utf-8")
    gate = Gate(Runtime())
    result = run_worker(request, gate)
    assert result.status == "precontact_rejected"
    assert gate.calls == []


def test_worker_release_failure_retains_unknown_disposition(tmp_path):
    runtime = Runtime()
    runtime.release_error = "authority disappeared"
    result = run_worker(_request(tmp_path), Gate(runtime))
    assert result.status == "known_failure"
    assert result.disposition.status == "unknown"
    assert result.disposition.reason == "claim_release_unknown"
    assert "release" in runtime.trace


def test_worker_rechecks_service_identity_after_resource_inventory(tmp_path):
    runtime = Runtime()
    state = {"epoch": "epoch-1"}

    def service_identity():
        runtime.trace.append("service_identity")
        return ServiceEvidence(123, "start-1", state["epoch"])

    def owned_ros_resources():
        runtime.trace.append("owned_ros_resources")
        state["epoch"] = "epoch-2"
        return ("left-arm-service", "right-arm-service")

    runtime.service_identity = service_identity
    runtime.owned_ros_resources = owned_ros_resources
    result = run_worker(_request(tmp_path), Gate(runtime))
    assert result.status == "known_failure"
    assert "resource inventory" in result.error
    assert "release" not in runtime.trace


def test_child_request_is_removed_after_run(tmp_path, monkeypatch):
    request = _request(tmp_path)
    request_path = tmp_path / "child-request.json"
    request_path.write_text(json.dumps(request_to_dict(request)), encoding="utf-8")
    module = SimpleNamespaceModule()
    monkeypatch.setattr(
        __import__("robots.lynsense_real_box.commissioning_binding", fromlist=["importlib"]).importlib,
        "import_module", lambda name: module,
    )
    assert _child_main(request_path) == 0
    assert not request_path.exists()
    assert request.result_path.is_file()


class SimpleNamespaceModule:
    raw = Runtime()

    def initialize_under_ownership(self, *args, **kwargs):
        return self.raw
