from __future__ import annotations

import json
import subprocess
import sys
import builtins
from dataclasses import replace
from datetime import datetime, timezone

import pytest

from robots.lynsense_real_box import commissioning_cli as cli
from robots.lynsense_real_box.commissioning_contract import (
    AuthorizationRecord, LockRecord, ResourceDisposition, ServiceEvidence,
    endpoint_scope_digest,
)
from robots.lynsense_real_box.commissioning_lock import (
    CommissioningLock, consume_attempt, read_lock_record,
)
from robots.lynsense_real_box import commissioning_binding
from robots.lynsense_real_box.commissioning_worker import (
    InitializationWorkerResult, worker_result_to_dict,
)


@pytest.fixture
def cli_files(tmp_path, valid_manifest, valid_manifest_data, valid_authority, write_json, monkeypatch):
    authority, host, authorization = valid_authority
    enrollment_path = write_json(tmp_path / "enrollment.json", {
        "authority_id": authority.authority_id,
        "authority_sha256": authority.authority_sha256,
        "records": [
            {"site_id": row.site_id, "machine_id": row.machine_id,
             "hostname": row.hostname, "account": row.account, "enabled": row.enabled}
            for row in authority.records
        ],
    })
    unsigned = {"trusted_lock_root": str(valid_manifest.trusted_lock_root),
                "allowed_operators": [authorization.operator]}
    import hashlib
    from robots.lynsense_real_box.commissioning_contract import authorization_digest
    auth_data = {
        "authorization_id": authorization.authorization_id,
        "purpose": "commissioning_run",
        "attempt_id": authorization.attempt_id,
        "authorization_sha256": "",
        "manifest_sha256": authorization.manifest_sha256,
        "site_id": authorization.site_id, "robot_id": authorization.robot_id,
        "machine_id": authorization.machine_id,
        "host_enrollment_sha256": authorization.host_enrollment_sha256,
        "policy_sha256": "",
        "operator": authorization.operator, "ros_domain_id": authorization.ros_domain_id,
        "expires_at": authorization.expires_at.isoformat(),
    }
    auth_data["authorization_sha256"] = authorization_digest(auth_data)
    reconcile_data = {
        **auth_data,
        "authorization_id": "reconcile-001",
        "purpose": "commissioning_reconcile",
        "authorization_sha256": "",
    }
    reconcile_data["authorization_sha256"] = authorization_digest(reconcile_data)
    unsigned["approved_authorizations"] = {
        authorization.authorization_id: auth_data["authorization_sha256"],
        "reconcile-001": reconcile_data["authorization_sha256"],
    }
    encoded = json.dumps(unsigned, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()
    auth_data["policy_sha256"] = hashlib.sha256(encoded).hexdigest()
    reconcile_data["policy_sha256"] = auth_data["policy_sha256"]
    policy_path = write_json(tmp_path / "policy.json", {**unsigned, "sha256": hashlib.sha256(encoded).hexdigest()})
    auth_path = write_json(tmp_path / "authorization.json", auth_data)
    reconcile_path = write_json(tmp_path / "reconciliation.json", reconcile_data)
    monkeypatch.setattr(cli, "DEFAULT_POLICY_PATH", policy_path)
    monkeypatch.setattr(cli, "_actual_host", lambda: host)
    from robots.lynsense_real_box.commissioning_contract import load_trusted_operator_policy
    monkeypatch.setattr(cli, "_load_policy", lambda: load_trusted_operator_policy(policy_path))
    return valid_manifest, enrollment_path, auth_path, reconcile_path, policy_path, authorization


@pytest.mark.parametrize(("status", "code"), [
    ("accepted", 0),
    ("rejected_before_initialization", 2),
    ("failed", 3),
    ("initialization_outcome_unknown", 4),
    ("service_ownership_enforcement_unavailable", 5),
])
def test_exit_mapping_is_fixed(status, code):
    assert cli.exit_code_for_status(status) == code


def test_unknown_status_has_no_success_fallback():
    with pytest.raises(ValueError):
        cli.exit_code_for_status("unexpected")


def test_help_and_validate_do_not_import_driver_or_worker(monkeypatch, cli_files):
    manifest, enrollment, _, _, _, _ = cli_files
    forbidden = {"lynrotcontrol", "rclpy"}
    assert forbidden.isdisjoint(sys.modules)
    assert cli.main(["--help"]) == 0
    original_import = builtins.__import__
    def guarded_import(name, *args, **kwargs):
        assert not (name == "commissioning_binding" or name.endswith("commissioning_worker"))
        return original_import(name, *args, **kwargs)
    monkeypatch.setattr(builtins, "__import__", guarded_import)
    assert cli.main(["validate", "--manifest", str(manifest_path(manifest)),
                     "--host-enrollment", str(enrollment)]) == 0
    assert forbidden.isdisjoint(sys.modules)


def manifest_path(manifest):
    return manifest.evidence_dir.parent / "manifest.json"


def test_validate_rejects_unknown_flags_and_effect_expressions(cli_files):
    manifest, enrollment, _, _, _, _ = cli_files
    path = manifest_path(manifest)
    assert cli.main(["validate", "--manifest", str(path), "--host-enrollment",
                     str(enrollment), "--python", "import os"]) == 2
    assert cli.main(["validate", "--manifest", str(path), "--host-enrollment",
                     str(enrollment), "--", "shell"]) == 2


def test_run_requires_explicit_authorization(cli_files):
    manifest, enrollment, _, _, _, _ = cli_files
    assert cli.main(["run", "--manifest", str(manifest_path(manifest)),
                     "--host-enrollment", str(enrollment)]) == 2


def test_run_unavailable_gate_fails_closed_with_code_five(cli_files):
    manifest, enrollment, authorization_path, _, _, _ = cli_files
    assert cli.main(["run", "--manifest", str(manifest_path(manifest)),
                     "--host-enrollment", str(enrollment), "--authorization-file",
                     str(authorization_path)]) == 5


def test_wrong_authorization_is_rejected_before_worker(cli_files, monkeypatch):
    manifest, enrollment, auth_path, _, _, _ = cli_files
    data = json.loads(auth_path.read_text())
    data["manifest_sha256"] = "0" * 64
    auth_path.write_text(json.dumps(data))
    monkeypatch.setattr(cli, "_run_worker", lambda *_: pytest.fail("worker started"), raising=False)
    assert cli.main(["run", "--manifest", str(manifest_path(manifest)),
                     "--host-enrollment", str(enrollment), "--authorization-file",
                     str(auth_path)]) == 2


def test_authorization_policy_digest_must_match_fixed_policy(cli_files):
    manifest, enrollment, auth_path, _, _, _ = cli_files
    data = json.loads(auth_path.read_text())
    data["policy_sha256"] = "f" * 64
    auth_path.write_text(json.dumps(data))
    assert cli.main(["run", "--manifest", str(manifest_path(manifest)),
                     "--host-enrollment", str(enrollment), "--authorization-file",
                     str(auth_path)]) == 2


def test_unknown_effect_name_is_rejected_by_static_validation(cli_files):
    import hashlib

    manifest, enrollment, _, _, _, _ = cli_files
    path = manifest_path(manifest)
    data = json.loads(path.read_text())
    data["approved_effects"] = ["python:import os"]
    unsigned = {key: value for key, value in data.items() if key != "manifest_sha256"}
    data["manifest_sha256"] = hashlib.sha256(json.dumps(
        unsigned, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
    ).encode("utf-8")).hexdigest()
    path.write_text(json.dumps(data))
    assert cli.main(["validate", "--manifest", str(path),
                     "--host-enrollment", str(enrollment)]) == 2


def test_empty_effect_inventory_is_rejected_by_static_validation(cli_files):
    from robots.lynsense_real_box.commissioning_contract import manifest_digest

    manifest, enrollment, _, _, _, _ = cli_files
    path = manifest_path(manifest)
    data = json.loads(path.read_text())
    data["approved_effects"] = []
    data["manifest_sha256"] = manifest_digest(data)
    path.write_text(json.dumps(data))
    assert cli.main(["validate", "--manifest", str(path),
                     "--host-enrollment", str(enrollment)]) == 2


def test_reconcile_requires_matching_unknown_lock_and_never_runs_worker(cli_files, monkeypatch):
    manifest, _, _, reconcile_path, _, authorization = cli_files
    scope = endpoint_scope_digest(manifest)
    attempt = authorization.attempt_id
    record = LockRecord(manifest.site_id, manifest.robot_id, scope,
                        manifest.manifest_sha256, attempt, authorization.authorization_id,
                        manifest.execution_host_machine_id, datetime.now(timezone.utc),
                        "held", None, None, None)
    consume_attempt(manifest, record)
    handle = CommissioningLock.acquire(manifest, record)
    handle.retain_unknown("worker outcome unknown")
    service = ServiceEvidence(4321, "proc-start", "service-epoch")
    disposition = ResourceDisposition("settled", service.pid, service.start_id,
                                      service.epoch, tuple(e.endpoint_id for e in manifest.endpoints),
                                      manifest.expected_owned_ros_resources,
                                      authorization.operator, "operator verified settled")
    result = InitializationWorkerResult(
        "late_settled", attempt, manifest.manifest_sha256, scope, None, (), service,
        disposition, None, 123, "worker-start",
    )
    manifest.evidence_dir.mkdir(parents=True, exist_ok=True)
    result_path = manifest.evidence_dir / f"{attempt}.result.json"
    result_path.write_text(json.dumps(worker_result_to_dict(result)))
    monkeypatch.setattr(cli, "_run_worker", lambda *_: pytest.fail("worker/contact on reconcile"), raising=False)
    monkeypatch.setattr(
        commissioning_binding, "inspect_dependency_claim", lambda _: "no_claim"
    )

    code = cli.main(["reconcile", "--manifest", str(manifest_path(manifest)),
                     "--lock", str(manifest.lock_path), "--authorization-file", str(reconcile_path)])

    assert code == 0
    assert not manifest.lock_path.exists()
    assert (manifest.evidence_dir / f"{attempt}.reconcile.report.json").is_file()


def test_reconcile_rejects_mismatched_retained_result_without_contact(
    cli_files, monkeypatch
):
    manifest, _, _, reconcile_path, _, authorization = cli_files
    scope = endpoint_scope_digest(manifest)
    record = LockRecord(manifest.site_id, manifest.robot_id, scope,
                        manifest.manifest_sha256, "attempt-mismatch", authorization.authorization_id,
                        manifest.execution_host_machine_id, datetime.now(timezone.utc),
                        "unknown", None, None, None)
    run_record = replace(record, state="held")
    consume_attempt(manifest, run_record)
    CommissioningLock.acquire(manifest, run_record).retain_unknown("unknown")
    wrong = InitializationWorkerResult(
        "late_settled", "different-attempt", manifest.manifest_sha256, scope, None, (),
        ServiceEvidence(4321, "proc-start", "epoch"),
        ResourceDisposition("settled", 4321, "proc-start", "epoch", (), (), authorization.operator, None),
        None, 123, "worker-start",
    )
    manifest.evidence_dir.mkdir(parents=True, exist_ok=True)
    (manifest.evidence_dir / "attempt-mismatch.result.json").write_text(json.dumps(worker_result_to_dict(wrong)))
    monkeypatch.setattr(
        commissioning_binding, "inspect_dependency_claim", lambda _: "claim_released"
    )

    assert cli.main(["reconcile", "--manifest", str(manifest_path(manifest)),
                     "--lock", str(manifest.lock_path), "--authorization-file", str(reconcile_path)]) == 2
    assert read_lock_record(manifest.lock_path).state == "unknown"


def test_reconcile_blocking_dependency_claim_is_unavailable_and_retained(
    cli_files, monkeypatch
):
    manifest, _, _, reconcile_path, _, run_authorization = cli_files
    authorization_data = json.loads(reconcile_path.read_text(encoding="utf-8"))
    authorization = replace(
        run_authorization,
        authorization_id=authorization_data["authorization_id"],
        purpose=authorization_data["purpose"],
        authorization_sha256=authorization_data["authorization_sha256"],
    )
    scope = endpoint_scope_digest(manifest)
    attempt = authorization.attempt_id
    run_record = LockRecord(
        manifest.site_id, manifest.robot_id, scope, manifest.manifest_sha256,
        attempt, manifest.authorization_id, manifest.execution_host_machine_id,
        datetime.now(timezone.utc), "held", None, None, None,
    )
    consume_attempt(manifest, run_record)
    CommissioningLock.acquire(manifest, run_record).retain_unknown("late worker")
    result = InitializationWorkerResult(
        "late_settled", attempt, manifest.manifest_sha256, scope, None, (),
        ServiceEvidence(4321, "proc-start", "epoch"),
        ResourceDisposition("settled", 4321, "proc-start", "epoch",
                            tuple(e.endpoint_id for e in manifest.endpoints),
                            manifest.expected_owned_ros_resources,
                            authorization.operator, None),
        None, 123, "worker-start",
    )
    manifest.evidence_dir.mkdir(parents=True, exist_ok=True)
    (manifest.evidence_dir / f"{attempt}.result.json").write_text(
        json.dumps(worker_result_to_dict(result))
    )
    monkeypatch.setattr(
        commissioning_binding, "inspect_dependency_claim", lambda _: "claim_orphaned"
    )

    code = cli.main(["reconcile", "--manifest", str(manifest_path(manifest)),
                     "--lock", str(manifest.lock_path), "--authorization-file",
                     str(reconcile_path)])

    assert code == 4
    assert read_lock_record(manifest.lock_path).state == "unknown"
    report = manifest.evidence_dir / f"{attempt}.reconcile.report.json"
    assert report.is_file()
    assert "dependency_reconciliation_unavailable" in report.read_text()


def test_cli_subprocess_help_is_offline():
    result = subprocess.run(
        [sys.executable, "-m", "robots.lynsense_real_box.commissioning_cli", "--help"],
        check=False, capture_output=True, text=True,
    )
    assert result.returncode == 0
    assert "validate" in result.stdout and "run" in result.stdout and "reconcile" in result.stdout


def test_readme_and_runbook_capture_operator_contract():
    from pathlib import Path

    root = Path(__file__).resolve().parents[4]
    readme = (root / "robots/lynsense_real_box/README.md").read_text(encoding="utf-8")
    runbook_path = root / "docs/superpowers/plans/2026-09-23-robot-one-lynrotcontrol-first-connection-runbook.md"
    runbook = runbook_path.read_text(encoding="utf-8")
    for text in (readme, runbook):
        assert "operator-only" in text
        assert "ROS_DOMAIN_ID=3" in text
        assert "no motion" in text.lower() or "no trajectory" in text.lower()
        assert "reconcile" in text
        assert "service-ownership" in text or "ownership gate" in text
    assert "pytree" in readme and "plan_arm_simple" in readme
    assert "build_effectful_gate" in runbook and "exit `5`" in runbook
    assert "78c45f6796cd90fdf63f51ac407ac261c04e42ab454bb6d0897749d81904c84a" in readme
    assert "38015e139b4b7ae2c0ca7f2261be2a90cb4d40ee12d35420875c6d150962ceee" in readme
    assert "/tmp/lynrotcontrol-authority-1000" in readme
    assert "runtime protocol `15`" in readme
    assert "deployment and robot one operation require separate approvals" in " ".join(readme.lower().split())
