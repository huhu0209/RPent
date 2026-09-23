"""Durable, endpoint-scoped lock for one-shot commissioning attempts."""

from __future__ import annotations

import json
import os
import re
import tempfile
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

from .commissioning_contract import (
    CommissioningManifest,
    LockRecord,
    ResourceDisposition,
    ServiceEvidence,
    endpoint_scope_digest,
)


class CommissioningLockError(RuntimeError):
    """A durable commissioning lock cannot safely be changed."""


class LockExistsError(CommissioningLockError):
    """An endpoint already has a lock, including a malformed lock."""


class LockRecordMismatch(CommissioningLockError):
    """The durable record no longer belongs to this lock handle."""


def _fsync_directory(path: Path) -> None:
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    descriptor = os.open(path, flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _record_json(record: LockRecord, *, reason: str | None = None) -> bytes:
    data = asdict(record)
    data["created_at"] = record.created_at.astimezone(timezone.utc).isoformat()
    if reason is not None:
        data["reason"] = reason
    return (json.dumps(data, sort_keys=True, separators=(",", ":"), ensure_ascii=True) + "\n").encode("utf-8")


def _matches(actual: LockRecord, expected: LockRecord) -> bool:
    return (
        actual.attempt_id == expected.attempt_id
        and actual.authorization_id == expected.authorization_id
        and actual.manifest_sha256 == expected.manifest_sha256
        and actual.endpoint_scope_sha256 == expected.endpoint_scope_sha256
        and actual.site_id == expected.site_id
        and actual.robot_id == expected.robot_id
        and actual.host_machine_id == expected.host_machine_id
    )


def read_lock_record(path: Path) -> LockRecord:
    record_path = path / "record.json"
    if record_path.is_symlink() or not record_path.is_file():
        raise CommissioningLockError(f"lock record is missing or unsafe: {record_path}")
    try:
        data = json.loads(record_path.read_text(encoding="utf-8"))
        data.pop("reason", None)
        data["created_at"] = datetime.fromisoformat(data["created_at"])
        return LockRecord(**data)
    except (OSError, ValueError, TypeError, KeyError) as exc:
        raise CommissioningLockError(f"malformed lock record: {record_path}") from exc


def _write_update(path: Path, record: LockRecord, *, reason: str | None = None) -> None:
    payload = _record_json(record, reason=reason)
    descriptor, temporary = tempfile.mkstemp(prefix=".record-", suffix=".tmp", dir=path)
    temp_path = Path(temporary)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_path, path / "record.json")
        _fsync_directory(path)
    finally:
        try:
            temp_path.unlink()
        except FileNotFoundError:
            pass


class CommissioningLockHandle:
    def __init__(self, path: Path, record: LockRecord):
        self.path = path
        self._record = record

    def _current_record(self) -> LockRecord:
        current = read_lock_record(self.path)
        if not _matches(current, self._record):
            raise LockRecordMismatch(
                "lock attempt, authorization, manifest digest, or endpoint scope changed"
            )
        return current

    def release_after_settled(self, disposition: ResourceDisposition) -> None:
        if disposition.status != "settled":
            raise CommissioningLockError("only a settled disposition may release a lock")
        current = self._current_record()
        if current.state != "held":
            raise LockRecordMismatch("only a held lock may be released by its original handle")
        entries = {entry.name for entry in self.path.iterdir()}
        if entries != {"record.json"}:
            raise CommissioningLockError("lock directory contains unexpected entries")
        (self.path / "record.json").unlink()
        self.path.rmdir()
        _fsync_directory(self.path.parent)

    def retain_unknown(self, reason: str) -> None:
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("unknown retention reason must be nonempty")
        current = self._current_record()
        if current.state not in {"held", "unknown"}:
            raise LockRecordMismatch("lock is not in a retainable state")
        updated = LockRecord(**{**asdict(current), "state": "unknown"})
        _write_update(self.path, updated, reason=reason)
        self._record = updated

    def record_service_identity(self, service: ServiceEvidence) -> None:
        if (
            service.pid is None
            or service.start_id is None
            or service.epoch is None
        ):
            raise LockRecordMismatch("service identity must include pid, start_id, and epoch")
        current = self._current_record()
        if current.state not in {"held", "unknown"}:
            raise LockRecordMismatch("lock is not in a recordable state")
        updated = LockRecord(
            **{
                **asdict(current),
                "service_pid": service.pid,
                "service_start_id": service.start_id,
                "service_epoch": service.epoch,
            }
        )
        _write_update(self.path, updated)
        self._record = updated


class CommissioningLock:
    @staticmethod
    def acquire(manifest: CommissioningManifest, record: LockRecord) -> CommissioningLockHandle:
        scope = endpoint_scope_digest(manifest)
        expected_path = manifest.trusted_lock_root / scope / "commissioning.lock"
        if manifest.lock_path != expected_path:
            raise CommissioningLockError("manifest lock path does not match endpoint scope")
        if (
            record.site_id != manifest.site_id
            or record.robot_id != manifest.robot_id
            or record.endpoint_scope_sha256 != scope
            or record.manifest_sha256 != manifest.manifest_sha256
            or record.authorization_id != manifest.authorization_id
            or record.host_machine_id != manifest.execution_host_machine_id
            or not record.attempt_id
            or record.state != "held"
        ):
            raise CommissioningLockError("lock record identity does not match manifest")
        manifest.trusted_lock_root.mkdir(parents=True, exist_ok=True)
        scope_directory = expected_path.parent
        scope_directory.mkdir(exist_ok=True)
        _fsync_directory(manifest.trusted_lock_root)
        try:
            expected_path.mkdir(exist_ok=False)
        except FileExistsError as exc:
            raise LockExistsError("lock_exists") from exc
        _fsync_directory(expected_path.parent)
        record_path = expected_path / "record.json"
        descriptor = os.open(record_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(_record_json(record))
            stream.flush()
            os.fsync(stream.fileno())
        _fsync_directory(expected_path)
        return CommissioningLockHandle(expected_path, record)

    @staticmethod
    def record_service_identity(
        path: Path,
        *,
        expected_attempt_id: str,
        expected_authorization_id: str,
        expected_manifest_sha256: str,
        expected_endpoint_scope_sha256: str,
        service: ServiceEvidence,
    ) -> None:
        """Persist late worker service identity on an unresolved lock."""
        record = read_lock_record(path)
        if record.state not in {"held", "unknown"}:
            raise LockRecordMismatch("lock is not in a recordable state")
        if (
            record.attempt_id != expected_attempt_id
            or record.authorization_id != expected_authorization_id
            or record.manifest_sha256 != expected_manifest_sha256
            or record.endpoint_scope_sha256 != expected_endpoint_scope_sha256
        ):
            raise LockRecordMismatch("late service identity does not match lock")
        CommissioningLockHandle(path, record).record_service_identity(service)


def consume_attempt(manifest: CommissioningManifest, record: LockRecord) -> Path:
    """Durably mark an exact authorization attempt before lock acquisition."""
    scope = endpoint_scope_digest(manifest)
    if (
        record.site_id != manifest.site_id
        or record.robot_id != manifest.robot_id
        or record.endpoint_scope_sha256 != scope
        or record.manifest_sha256 != manifest.manifest_sha256
        or record.authorization_id != manifest.authorization_id
        or record.host_machine_id != manifest.execution_host_machine_id
        or not re.fullmatch(r"[A-Za-z0-9_-]{1,128}", record.attempt_id)
        or record.state != "held"
    ):
        raise CommissioningLockError("consumption record does not match manifest")
    consumed_root = manifest.trusted_lock_root / "consumed"
    consumed_scope = consumed_root / scope
    consumed_root.mkdir(exist_ok=True)
    if consumed_root.is_symlink() or not consumed_root.is_dir():
        raise CommissioningLockError("attempt consumption root is unsafe")
    _fsync_directory(manifest.trusted_lock_root)
    consumed_scope.mkdir(exist_ok=True)
    if consumed_scope.is_symlink() or not consumed_scope.is_dir():
        raise CommissioningLockError("attempt consumption scope is unsafe")
    _fsync_directory(consumed_root)
    marker = consumed_scope / f"{record.attempt_id}.json"
    payload = _record_json(record, reason="authorization attempt consumed before lock acquisition")
    try:
        descriptor = os.open(marker, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError as exc:
        raise CommissioningLockError("authorization attempt already consumed") from exc
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    _fsync_directory(consumed_scope)
    return marker


def verify_consumed_attempt(manifest: CommissioningManifest, record: LockRecord) -> None:
    """Verify that a retained lock originated from a consumed run attempt."""
    marker = (
        manifest.trusted_lock_root / "consumed"
        / record.endpoint_scope_sha256 / f"{record.attempt_id}.json"
    )
    if marker.is_symlink() or not marker.is_file():
        raise CommissioningLockError("authorization attempt consumption record is missing")
    try:
        data = json.loads(marker.read_text(encoding="utf-8"))
        reason = data.pop("reason", None)
        data["created_at"] = datetime.fromisoformat(data["created_at"])
        consumed = LockRecord(**data)
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
        raise CommissioningLockError("authorization attempt consumption record is malformed") from exc
    if reason != "authorization attempt consumed before lock acquisition" or not _matches(consumed, record):
        raise CommissioningLockError("authorization attempt consumption record does not match lock")


def reconcile_lock(
    path: Path,
    *,
    operator: str,
    authorization_id: str,
    expected_attempt_id: str,
    expected_manifest_sha256: str,
    expected_endpoint_scope_sha256: str,
    disposition: ResourceDisposition,
) -> None:
    if not operator.strip() or not authorization_id.strip():
        raise ValueError("operator and authorization_id are required for reconciliation")
    if disposition.status != "settled":
        raise CommissioningLockError("reconciliation requires a settled disposition")
    if disposition.operator != operator:
        raise LockRecordMismatch("reconciliation operator does not match disposition")
    if (
        disposition.service_pid is None
        or disposition.service_start_id is None
        or disposition.service_epoch is None
    ):
        raise LockRecordMismatch("reconciliation requires verified service identity")
    record = read_lock_record(path)
    if record.state != "unknown":
        raise LockRecordMismatch("only an unresolved unknown lock may be reconciled")
    if record.attempt_id != expected_attempt_id:
        raise LockRecordMismatch("reconciliation attempt does not match lock")
    if record.authorization_id != authorization_id:
        raise LockRecordMismatch("reconciliation authorization does not match lock")
    if record.manifest_sha256 != expected_manifest_sha256:
        raise LockRecordMismatch("reconciliation manifest digest does not match lock")
    if record.endpoint_scope_sha256 != expected_endpoint_scope_sha256:
        raise LockRecordMismatch("reconciliation endpoint scope does not match lock")
    if (
        record.service_pid != disposition.service_pid
        or record.service_start_id != disposition.service_start_id
        or record.service_epoch != disposition.service_epoch
    ):
        raise LockRecordMismatch("reconciliation service identity does not match lock")
    if path.name != "commissioning.lock" or path.parent.name != record.endpoint_scope_sha256:
        raise LockRecordMismatch("lock path does not match endpoint scope")
    entries = {entry.name for entry in path.iterdir()}
    if entries != {"record.json"}:
        raise CommissioningLockError("lock directory contains unexpected entries")
    (path / "record.json").unlink()
    path.rmdir()
    _fsync_directory(path.parent)
