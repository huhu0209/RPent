"""Durable, credential-filtered evidence for one commissioning attempt."""

from __future__ import annotations

import json
import math
import os
import re
import tempfile
from collections.abc import Mapping, Sequence
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .commissioning_contract import CommissioningManifest


class CommissioningEvidenceError(ValueError):
    """Evidence is unsafe, incomplete, or cannot be durably recorded."""


_FORBIDDEN = frozenset({
    "password", "token", "api_key", "apikey", "access_token", "auth_token",
    "authorization", "secret", "private_key", "ssh_config",
})
_CREDENTIAL_KEY = re.compile(r"([a-z0-9])([A-Z])")


def _safe(value: Any, label: str = "evidence") -> Any:
    if is_dataclass(value):
        value = asdict(value)
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, datetime):
        if value.tzinfo is None or value.utcoffset() is None:
            raise CommissioningEvidenceError(f"{label} datetime must be timezone-aware")
        return value.astimezone(timezone.utc).isoformat().replace("+00:00", "Z")
    if isinstance(value, Mapping):
        result = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise CommissioningEvidenceError(f"{label} keys must be strings")
            normalized = re.sub(r"[\s-]+", "_", _CREDENTIAL_KEY.sub(r"\1_\2", key).lower())
            if normalized in _FORBIDDEN:
                raise CommissioningEvidenceError(f"{label}.{key} is a credential-like key")
            result[key] = _safe(item, f"{label}.{key}")
        return result
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return [_safe(item, f"{label}[{index}]") for index, item in enumerate(value)]
    if isinstance(value, float) and not math.isfinite(value):
        raise CommissioningEvidenceError(f"{label} contains a non-finite number")
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    raise CommissioningEvidenceError(f"{label} is not JSON-serializable")


def _canonical(value: Any) -> bytes:
    return (json.dumps(_safe(value), ensure_ascii=False, allow_nan=False,
                       sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")


class CommissioningEvidenceWriter:
    def __init__(self, run_id: str, manifest: CommissioningManifest, path: Path):
        if not run_id or not isinstance(path, Path):
            raise CommissioningEvidenceError("run_id and Path evidence destination are required")
        self.run_id, self.manifest, self.path = run_id, manifest, path
        self.report_path = path.with_suffix(".report.json")
        path.parent.mkdir(parents=True, exist_ok=True)
        for target in (path, self.report_path):
            if target.is_symlink():
                raise CommissioningEvidenceError(f"refusing symlink evidence target: {target}")
        if path.exists():
            raw = path.read_bytes()
            if raw and not raw.endswith(b"\n"):
                raise CommissioningEvidenceError("existing evidence ends with a truncated line")
            for number, line in enumerate(raw.decode("utf-8").splitlines(), 1):
                try:
                    event = json.loads(line)
                except (ValueError, json.JSONDecodeError) as exc:
                    raise CommissioningEvidenceError(f"invalid existing evidence line {number}") from exc
                if not isinstance(event, dict) or event.get("run_id") != run_id or event.get("manifest_sha256") != manifest.manifest_sha256:
                    raise CommissioningEvidenceError(f"existing evidence line {number} has a different run/manifest")
                if line.encode("utf-8") + b"\n" != _canonical(event):
                    raise CommissioningEvidenceError(f"existing evidence line {number} is not canonical")
        was_existing = path.exists()
        self._stream = path.open("ab")
        if not was_existing:
            directory = os.open(path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
            try:
                os.fsync(directory)
            finally:
                os.close(directory)

    def record(self, event: str, **fields: Any) -> None:
        if not event or "run_id" in fields or "manifest_sha256" in fields:
            raise CommissioningEvidenceError("event name is required and linkage fields are reserved")
        item = {
            "schema_version": 1,
            "event": event,
            "created_at": datetime.now(timezone.utc),
            "run_id": self.run_id,
            "manifest_sha256": self.manifest.manifest_sha256,
            **fields,
        }
        payload = _canonical(item)
        self._stream.write(payload)
        self._stream.flush()
        os.fsync(self._stream.fileno())

    def write_report(self, report: Mapping[str, Any]) -> Path:
        if self._stream.closed:
            raise CommissioningEvidenceError("evidence writer is closed")
        reserved = {"schema_version", "run_id", "manifest_sha256"}
        if reserved.intersection(report):
            raise CommissioningEvidenceError("report cannot override schema or linkage fields")
        payload = _canonical({
            "schema_version": 1,
            "run_id": self.run_id,
            "manifest_sha256": self.manifest.manifest_sha256,
            **dict(report),
        })
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{self.report_path.name}.", suffix=".tmp", dir=self.report_path.parent
        )
        temporary = Path(temporary_name)
        try:
            with os.fdopen(descriptor, "wb") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.report_path)
            directory = os.open(self.report_path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            temporary.unlink(missing_ok=True)
        return self.report_path

    def close(self) -> None:
        if not self._stream.closed:
            self._stream.close()

    def __enter__(self) -> "CommissioningEvidenceWriter":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
