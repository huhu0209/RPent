from __future__ import annotations

import json
from dataclasses import asdict, replace
from datetime import datetime, timezone
from pathlib import Path

import pytest

from robots.lynsense_real_box.commissioning_contract import (
    LockRecord,
    ResourceDisposition,
    ServiceEvidence,
    endpoint_scope_digest,
)
from robots.lynsense_real_box.commissioning_lock import (
    CommissioningLock,
    CommissioningLockError,
    LockExistsError,
    LockRecordMismatch,
    read_lock_record,
    reconcile_lock,
)


def _record(manifest, attempt_id: str = "attempt-1") -> LockRecord:
    return LockRecord(
        site_id=manifest.site_id,
        robot_id=manifest.robot_id,
        endpoint_scope_sha256=endpoint_scope_digest(manifest),
        manifest_sha256=manifest.manifest_sha256,
        attempt_id=attempt_id,
        authorization_id=manifest.authorization_id,
        host_machine_id=manifest.execution_host_machine_id,
        created_at=datetime.now(timezone.utc),
        state="held",
        service_pid=1234,
        service_start_id="start-1",
        service_epoch="epoch-1",
    )


def _settled() -> ResourceDisposition:
    return ResourceDisposition(
        "settled", 1234, "start-1", "epoch-1", (), (), "operator", None,
    )


def test_acquire_writes_durable_record_and_matching_release_removes_lock(valid_manifest):
    manifest = valid_manifest
    record = _record(manifest)
    handle = CommissioningLock.acquire(manifest, record)

    assert read_lock_record(manifest.lock_path) == record
    handle.release_after_settled(_settled())
    assert not manifest.lock_path.exists()


def test_duplicate_acquisition_rejects_without_overwriting_record(valid_manifest):
    manifest = valid_manifest
    first_record = _record(manifest)
    handle = CommissioningLock.acquire(manifest, first_record)

    with pytest.raises(LockExistsError, match="lock_exists"):
        CommissioningLock.acquire(manifest, _record(manifest, "attempt-2"))

    assert read_lock_record(manifest.lock_path) == first_record
    handle.release_after_settled(_settled())


def test_malformed_existing_lock_is_retained_and_rejected(valid_manifest):
    manifest = valid_manifest
    manifest.lock_path.mkdir(parents=True)
    (manifest.lock_path / "record.json").write_text("{", encoding="utf-8")

    with pytest.raises(LockExistsError):
        CommissioningLock.acquire(manifest, _record(manifest))
    with pytest.raises(CommissioningLockError, match="malformed"):
        read_lock_record(manifest.lock_path)
    assert manifest.lock_path.exists()


def test_old_held_record_is_not_auto_expired(valid_manifest):
    manifest = valid_manifest
    record = _record(manifest)
    old = LockRecord(**{**asdict(record), "created_at": datetime(2000, 1, 1, tzinfo=timezone.utc)})
    CommissioningLock.acquire(manifest, old)

    with pytest.raises(LockExistsError):
        CommissioningLock.acquire(manifest, _record(manifest, "new-attempt"))
    assert read_lock_record(manifest.lock_path).attempt_id == old.attempt_id


def test_release_rejects_attempt_digest_and_scope_mismatch(valid_manifest):
    manifest = valid_manifest
    record = _record(manifest)
    handle = CommissioningLock.acquire(manifest, record)
    durable_path = manifest.lock_path / "record.json"
    data = json.loads(durable_path.read_text(encoding="utf-8"))
    data["attempt_id"] = "other-attempt"
    durable_path.write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(LockRecordMismatch, match="attempt"):
        handle.release_after_settled(_settled())
    assert manifest.lock_path.exists()


def test_release_rejects_authorization_mismatch(valid_manifest):
    manifest = valid_manifest
    handle = CommissioningLock.acquire(manifest, _record(manifest))
    durable_path = manifest.lock_path / "record.json"
    data = json.loads(durable_path.read_text(encoding="utf-8"))
    data["authorization_id"] = "different-authorization"
    durable_path.write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(LockRecordMismatch, match="authorization"):
        handle.release_after_settled(_settled())
    assert manifest.lock_path.exists()


def test_unknown_is_atomically_retained_until_explicit_matching_reconcile(valid_manifest):
    manifest = valid_manifest
    record = _record(manifest)
    handle = CommissioningLock.acquire(manifest, record)
    handle.retain_unknown("worker deadline elapsed")

    assert read_lock_record(manifest.lock_path).state == "unknown"
    with pytest.raises(LockRecordMismatch):
        reconcile_lock(
            manifest.lock_path,
            operator="operator",
            authorization_id="auth-001",
            expected_attempt_id="wrong-attempt",
            expected_manifest_sha256=record.manifest_sha256,
            expected_endpoint_scope_sha256=record.endpoint_scope_sha256,
            disposition=_settled(),
        )
    with pytest.raises(LockRecordMismatch, match="manifest digest"):
        reconcile_lock(
            manifest.lock_path,
            operator="operator",
            authorization_id="auth-001",
            expected_attempt_id=record.attempt_id,
            expected_manifest_sha256="0" * 64,
            expected_endpoint_scope_sha256=record.endpoint_scope_sha256,
            disposition=_settled(),
        )
    with pytest.raises(LockRecordMismatch, match="authorization"):
        reconcile_lock(
            manifest.lock_path,
            operator="operator",
            authorization_id="different-authorization",
            expected_attempt_id=record.attempt_id,
            expected_manifest_sha256=record.manifest_sha256,
            expected_endpoint_scope_sha256=record.endpoint_scope_sha256,
            disposition=_settled(),
        )
    with pytest.raises(LockRecordMismatch, match="endpoint scope"):
        reconcile_lock(
            manifest.lock_path,
            operator="operator",
            authorization_id="auth-001",
            expected_attempt_id=record.attempt_id,
            expected_manifest_sha256=record.manifest_sha256,
            expected_endpoint_scope_sha256="0" * 64,
            disposition=_settled(),
        )
    with pytest.raises(CommissioningLockError, match="settled"):
        handle.release_after_settled(ResourceDisposition(
            "unknown", None, None, None, (), (), None, "uncertain",
        ))

    reconcile_lock(
        manifest.lock_path,
        operator="operator",
        authorization_id="auth-001",
        expected_attempt_id=record.attempt_id,
        expected_manifest_sha256=record.manifest_sha256,
        expected_endpoint_scope_sha256=record.endpoint_scope_sha256,
        disposition=_settled(),
    )
    assert not manifest.lock_path.exists()


def test_reconcile_requires_explicit_operator_and_authorization(valid_manifest):
    manifest = valid_manifest
    handle = CommissioningLock.acquire(manifest, _record(manifest))
    handle.retain_unknown("uncertain")

    with pytest.raises(ValueError, match="operator and authorization"):
        reconcile_lock(
            manifest.lock_path,
            operator="",
            authorization_id="auth-001",
            expected_attempt_id="attempt-1",
            expected_manifest_sha256="0" * 64,
            expected_endpoint_scope_sha256="0" * 64,
            disposition=_settled(),
        )
    assert manifest.lock_path.exists()


def test_reconcile_rejects_unverified_service_identity(valid_manifest):
    manifest = valid_manifest
    record = replace(
        _record(manifest), service_pid=None, service_start_id=None, service_epoch=None
    )
    handle = CommissioningLock.acquire(manifest, record)
    handle.retain_unknown("uncertain")

    with pytest.raises(LockRecordMismatch, match="service identity"):
        reconcile_lock(
            manifest.lock_path,
            operator="operator",
            authorization_id=manifest.authorization_id,
            expected_attempt_id=record.attempt_id,
            expected_manifest_sha256=record.manifest_sha256,
            expected_endpoint_scope_sha256=record.endpoint_scope_sha256,
            disposition=_settled(),
        )
    assert manifest.lock_path.exists()


def test_record_service_identity_then_reconcile_unknown_lock(valid_manifest):
    manifest = valid_manifest
    record = replace(
        _record(manifest), service_pid=None, service_start_id=None, service_epoch=None
    )
    handle = CommissioningLock.acquire(manifest, record)
    handle.record_service_identity(ServiceEvidence(1234, "start-1", "epoch-1"))
    handle.retain_unknown("worker completed after parent deadline")

    reconcile_lock(
        manifest.lock_path,
        operator="operator",
        authorization_id=manifest.authorization_id,
        expected_attempt_id=record.attempt_id,
        expected_manifest_sha256=record.manifest_sha256,
        expected_endpoint_scope_sha256=record.endpoint_scope_sha256,
        disposition=_settled(),
    )
    assert not manifest.lock_path.exists()


def test_static_identity_update_supports_late_worker_reconciliation(valid_manifest):
    manifest = valid_manifest
    record = replace(
        _record(manifest), service_pid=None, service_start_id=None, service_epoch=None
    )
    handle = CommissioningLock.acquire(manifest, record)
    handle.retain_unknown("parent timed out")
    CommissioningLock.record_service_identity(
        manifest.lock_path,
        expected_attempt_id=record.attempt_id,
        expected_authorization_id=record.authorization_id,
        expected_manifest_sha256=record.manifest_sha256,
        expected_endpoint_scope_sha256=record.endpoint_scope_sha256,
        service=ServiceEvidence(1234, "start-1", "epoch-1"),
    )
    assert read_lock_record(manifest.lock_path).service_epoch == "epoch-1"


def test_cross_process_duplicate_and_release(valid_manifest, acquire_in_child, release_child):
    first = acquire_in_child(valid_manifest, "child-first")
    second = acquire_in_child(valid_manifest, "child-second")
    try:
        assert first["status"] == "acquired"
        assert second == {"status": "rejected", "reason": "lock_exists"}
        assert read_lock_record(valid_manifest.lock_path).attempt_id == "child-first"
    finally:
        release_child(first)


def test_lock_path_must_be_derived_from_scope(valid_manifest):
    manifest = valid_manifest
    wrong_path = manifest.trusted_lock_root / ("0" * 64) / "commissioning.lock"
    tampered = replace(manifest, lock_path=wrong_path)

    with pytest.raises(CommissioningLockError, match="endpoint scope"):
        CommissioningLock.acquire(tampered, _record(manifest))
