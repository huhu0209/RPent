from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from collections.abc import Mapping
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

import pytest


def _digest(data: Mapping[str, object], excluded: str | None = None) -> str:
    payload = {key: value for key, value in data.items() if key != excluded}
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _scope_digest(robot_id: str, site_id: str, endpoints: list[dict[str, str]]) -> str:
    ordered = sorted(endpoints, key=lambda e: (e["subsystem"], e["endpoint_id"], e["transport"], e["locator"]))
    return _digest({"robot_id": robot_id, "site_id": site_id, "endpoints": ordered})


@pytest.fixture
def write_json():
    def write(path: Path, data: Mapping[str, object]) -> Path:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, sort_keys=True), encoding="utf-8")
        return path

    return write


@pytest.fixture
def valid_manifest_data(tmp_path: Path):
    def build(root: Path = tmp_path) -> dict[str, object]:
        lock_root = root / "trusted-locks"
        lock_root.mkdir(exist_ok=True)
        source = root / "lynrotcontrol-source.py"
        config = root / "robot-one-site.yaml"
        protocol = root / "OWNERSHIP_PROTOCOL.json"
        source.write_text("offline source fixture\n", encoding="utf-8")
        config.write_text("site: offline-fixture\n", encoding="utf-8")
        protocol.write_text(
            '{\n'
            '  "endpoint_authority_root": "/tmp/lynrotcontrol-authority-1008",\n'
            '  "protocol_id": "lynrotcontrol.service-ownership",\n'
            '  "protocol_version": 1,\n'
            '  "read_allowlist_sha256": "38015e139b4b7ae2c0ca7f2261be2a90cb4d40ee12d35420875c6d150962ceee",\n'
            '  "runtime_protocol": 15\n'
            '}\n',
            encoding="utf-8",
        )
        protocol_digest = hashlib.sha256(protocol.read_bytes()).hexdigest()
        endpoints = [
            {"subsystem": "arm", "endpoint_id": "left-arm", "transport": "tcp", "locator": "robot-one/left"},
            {"subsystem": "arm", "endpoint_id": "right-arm", "transport": "tcp", "locator": "robot-one/right"},
        ]
        host_record = {
            "site_id": "site-a", "machine_id": "a" * 32,
            "hostname": "commissioning-host", "account": "operator", "enabled": True,
        }
        data: dict[str, object] = {
            "site_id": "site-a", "robot_id": "robot-one", "instance": "robot-one-instance",
            "ros_domain_id": 3, "execution_host_machine_id": host_record["machine_id"],
            "execution_hostname": host_record["hostname"], "execution_account": host_record["account"],
            "execution_interpreter": "/usr/bin/python3",
            "trusted_lock_root": str(lock_root),
            "lock_path": str(lock_root / _scope_digest("robot-one", "site-a", endpoints) / "commissioning.lock"),
            "evidence_dir": str(root / "evidence"),
            "host_enrollment_sha256": _digest({"authority_id": "authority-1", "records": [host_record]}),
            "authorization_id": "auth-001", "authorization_expires_at": "2099-01-01T00:00:00+00:00",
            "manifest_sha256": "",
            "source_artifacts": [str(source), str(protocol), str(config)],
            "source_artifact_sha256": {
                str(source): hashlib.sha256(source.read_bytes()).hexdigest(),
                str(protocol): protocol_digest,
            },
            "ownership_protocol_artifact": str(protocol),
            "ownership_protocol_sha256": protocol_digest,
            "ownership_protocol_id": "lynrotcontrol.service-ownership",
            "ownership_protocol_version": 1,
            "ownership_read_allowlist_sha256": "38015e139b4b7ae2c0ca7f2261be2a90cb4d40ee12d35420875c6d150962ceee",
            "endpoint_authority_root": "/tmp/lynrotcontrol-authority-1008",
            "ownership_runtime_protocol": 15,
            "config_sha256": {str(config): hashlib.sha256(config.read_bytes()).hexdigest()},
            "required_samples": 2, "budgets": {"initialize": 30.0, "read": 5.0},
            "freshness_s": {"state": 2.0},
            "approved_effects": [
                "initialize_configuration",
                "fixed_read_ros_resource_creation",
                "fixed_read_ros_parameter_query",
            ],
            "endpoints": endpoints, "peripheral_mapping": {"left_arm": "left", "right_arm": "right"},
            "expected_identity": {
                "left_arm": {"side": "left", "model": "arm", "ip": "127.0.0.1", "axes": 6},
                "right_arm": {"side": "right", "model": "arm", "ip": "127.0.0.2", "axes": 6},
            },
            "expected_operator_verification": {
                "left_arm": "operator-verified", "right_arm": "operator-verified"
            },
            "expected_owned_ros_resources": ["left-arm-service", "right-arm-service"],
            "retention_policy": "retain_unknown",
        }
        data["manifest_sha256"] = _digest(data, "manifest_sha256")
        return data

    return build


@pytest.fixture
def valid_manifest(tmp_path: Path, valid_manifest_data, write_json):
    from robots.lynsense_real_box.commissioning_contract import load_commissioning_manifest

    return load_commissioning_manifest(write_json(tmp_path / "manifest.json", valid_manifest_data()))


@pytest.fixture
def valid_authority(valid_manifest):
    from robots.lynsense_real_box.commissioning_contract import (
        AuthorizationRecord, HostEnrollmentAuthority, HostEnrollmentRecord, HostIdentity,
        authorization_digest,
    )

    record = HostEnrollmentRecord(
        valid_manifest.site_id, valid_manifest.execution_host_machine_id,
        valid_manifest.execution_hostname, valid_manifest.execution_account, True,
    )
    authority = HostEnrollmentAuthority(
        "authority-1", valid_manifest.host_enrollment_sha256, (record,),
    )
    host = HostIdentity(record.machine_id, record.hostname, record.account, "/usr/bin/python3")
    auth_data = {
        "authorization_id": valid_manifest.authorization_id,
        "purpose": "commissioning_run",
        "attempt_id": "attempt-001",
        "authorization_sha256": "",
        "manifest_sha256": valid_manifest.manifest_sha256,
        "site_id": valid_manifest.site_id,
        "robot_id": valid_manifest.robot_id,
        "machine_id": record.machine_id,
        "host_enrollment_sha256": valid_manifest.host_enrollment_sha256,
        "policy_sha256": "e" * 64,
        "operator": record.account,
        "ros_domain_id": 3,
        "expires_at": valid_manifest.authorization_expires_at.isoformat(),
    }
    authorization = AuthorizationRecord(
        auth_data["authorization_id"], auth_data["purpose"], auth_data["attempt_id"],
        authorization_digest(auth_data), auth_data["manifest_sha256"],
        auth_data["site_id"], auth_data["robot_id"], auth_data["machine_id"],
        auth_data["host_enrollment_sha256"], auth_data["policy_sha256"],
        auth_data["operator"], auth_data["ros_domain_id"],
        valid_manifest.authorization_expires_at,
    )
    return authority, host, authorization


def _acquire_in_child(manifest, attempt_id: str) -> dict[str, object]:
    manifest_path = manifest.trusted_lock_root / f".child-{attempt_id}.json"
    manifest_path.write_text(
        json.dumps(
            asdict(manifest),
            default=lambda value: value.isoformat() if hasattr(value, "isoformat") else str(value),
        ),
        encoding="utf-8",
    )
    script = """
import json, sys
from datetime import datetime, timezone
from pathlib import Path
from robots.lynsense_real_box.commissioning_contract import LockRecord, ResourceDisposition, endpoint_scope_digest, load_commissioning_manifest
from robots.lynsense_real_box.commissioning_lock import CommissioningLock, LockExistsError
m = load_commissioning_manifest(Path(sys.argv[1]))
r = LockRecord(m.site_id, m.robot_id, endpoint_scope_digest(m), m.manifest_sha256, sys.argv[2], m.authorization_id, m.execution_host_machine_id, datetime.now(timezone.utc), 'held', None, None, None)
try:
    handle = CommissioningLock.acquire(m, r)
except LockExistsError:
    print(json.dumps({'status': 'rejected', 'reason': 'lock_exists'}), flush=True)
else:
    print(json.dumps({'status': 'acquired', 'attempt_id': r.attempt_id}), flush=True)
    sys.stdin.readline()
    handle.release_after_settled(ResourceDisposition('settled', None, None, None, (), (), 'test-operator', None))
"""
    process = subprocess.Popen(
        [sys.executable, "-c", script, str(manifest_path), attempt_id],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    result = json.loads(process.stdout.readline())
    if result["status"] != "acquired":
        process.wait(timeout=5)
        manifest_path.unlink(missing_ok=True)
        return result
    result["_process"] = process
    result["_manifest_path"] = manifest_path
    return result


def _release_child(result: dict[str, object]) -> None:
    process = result.get("_process")
    if process is None:
        return
    assert isinstance(process, subprocess.Popen)
    assert process.stdin is not None
    process.stdin.write("release\n")
    process.stdin.flush()
    process.stdin.close()
    assert process.wait(timeout=5) == 0
    manifest_path = result.get("_manifest_path")
    if isinstance(manifest_path, Path):
        manifest_path.unlink(missing_ok=True)

@pytest.fixture
def acquire_in_child():
    return _acquire_in_child

@pytest.fixture
def release_child():
    return _release_child
