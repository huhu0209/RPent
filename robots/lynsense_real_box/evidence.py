from __future__ import annotations

import json
import math
import os
import re
from collections.abc import Mapping, Sequence
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from robots.lynsense_real_box.atomic_profile import (
    AtomicCapabilityProfile,
    atomic_profile_hash,
)
from robots.lynsense_real_box.site_profile import SiteProfile, site_profile_hash


class EvidenceRecorderError(ValueError):
    """Raised when evidence cannot be represented or the recorder is closed."""


_FORBIDDEN_KEYS = frozenset(
    {
        "password",
        "token",
        "api_key",
        "apikey",
        "access_token",
        "auth_token",
        "authorization",
        "secret",
        "private_key",
        "ssh_config",
    }
)
_SCHEMA_VERSION = 1
_EVENT_ID_PATTERN = re.compile(r"evt-(\d{6,})\Z")


def _normalized_key(key: str) -> str:
    key = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", key.strip())
    return re.sub(r"[\s-]+", "_", key.strip().lower())


def _json_value(value: Any, label: str) -> Any:
    if isinstance(value, Mapping):
        result: dict[str, Any] = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise EvidenceRecorderError(f"{label} keys must be strings")
            if _normalized_key(key) in _FORBIDDEN_KEYS:
                raise EvidenceRecorderError(
                    f"{label}.{key} has a credential-like key; rejecting the event"
                )
            result[key] = _json_value(item, f"{label}.{key}")
        return result
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return [
            _json_value(item, f"{label}[{index}]")
            for index, item in enumerate(value)
        ]
    if isinstance(value, float) and not math.isfinite(value):
        raise EvidenceRecorderError(f"{label} must be a finite JSON number")
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    raise EvidenceRecorderError(f"{label} is not JSON-serializable")


def _text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise EvidenceRecorderError(f"{label} must be a nonempty string")
    return value


def _existing_event_high_water(
    path: Path,
    profile_id: str,
    profile_sha256: str,
) -> int:
    if not path.exists():
        return 0

    try:
        raw = path.read_bytes()
        if not raw:
            return 0
        if not raw.endswith(b"\n"):
            raise ValueError("evidence file ends with a truncated line")
        lines = raw.decode("utf-8").splitlines()
        high_water = 0
        seen_ids: set[int] = set()
        for line_number, line in enumerate(lines, start=1):
            try:
                event = json.loads(line)
            except (json.JSONDecodeError, ValueError) as error:
                raise ValueError(f"line {line_number} is not valid JSON") from error
            if not isinstance(event, dict):
                raise ValueError(f"line {line_number} is not a JSON object")
            if (
                event.get("profile_id") != profile_id
                or event.get("profile_sha256") != profile_sha256
            ):
                raise ValueError(
                    f"line {line_number} belongs to a different site profile"
                )
            match = _EVENT_ID_PATTERN.fullmatch(str(event.get("evidence_id", "")))
            if match is None:
                raise ValueError(
                    f"line {line_number} has no valid evidence_id"
                )
            event_number = int(match.group(1))
            if event_number in seen_ids:
                raise ValueError(
                    f"line {line_number} duplicates evidence_id {event['evidence_id']}"
                )
            canonical = json.dumps(
                event,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            )
            if line != canonical:
                raise ValueError(
                    f"line {line_number} is not canonical JSONL evidence"
                )
            seen_ids.add(event_number)
            high_water = max(high_water, event_number)
        return high_water
    except (OSError, UnicodeError, ValueError) as error:
        raise EvidenceRecorderError(f"invalid existing evidence: {error}") from error


class EvidenceRecorder:
    """Append accepted tool and operator events to a durable JSONL file."""

    def __init__(
        self,
        path: Path,
        profile: SiteProfile | AtomicCapabilityProfile,
    ) -> None:
        if not isinstance(path, Path):
            raise EvidenceRecorderError("path must be a Path")
        if not isinstance(profile, (SiteProfile, AtomicCapabilityProfile)):
            raise EvidenceRecorderError("profile type is not supported")

        path.parent.mkdir(parents=True, exist_ok=True)
        self._path = path
        self._profile = profile
        self._profile_sha256 = (
            site_profile_hash(profile)
            if isinstance(profile, SiteProfile)
            else atomic_profile_hash(profile)
        )
        self._event_count = _existing_event_high_water(
            path,
            profile.profile_id,
            self._profile_sha256,
        )
        self._closed = False
        self._file = path.open("a", encoding="utf-8", newline="\n")

    def record_tool_call(
        self,
        tool: str,
        arguments: dict,
        result: dict,
        state_before: str,
        state_after: str,
        intent_evidence_id: str | None = None,
    ) -> str:
        if self._closed:
            raise EvidenceRecorderError("evidence recorder is closed")

        tool_name = _text(tool, "tool")
        _text(state_before, "state_before")
        _text(state_after, "state_after")
        if not isinstance(arguments, dict):
            raise EvidenceRecorderError("arguments must be a dict")
        if not isinstance(result, dict):
            raise EvidenceRecorderError("result must be a dict")

        encoded_arguments = _json_value(arguments, "arguments")
        encoded_result = _json_value(result, "result")
        intent_id = (
            _text(intent_evidence_id, "intent_evidence_id")
            if intent_evidence_id is not None
            else None
        )
        evidence_id = self._next_event_id()
        result["evidence_id"] = evidence_id
        encoded_result["evidence_id"] = evidence_id
        if intent_id is not None:
            result["intent_evidence_id"] = intent_id
            encoded_result["intent_evidence_id"] = intent_id

        event = {
            "schema_version": _SCHEMA_VERSION,
            "event": "tool_call",
            "profile_id": self._profile.profile_id,
            "profile_sha256": self._profile_sha256,
            "created_at": _utc_timestamp(),
            "evidence_id": evidence_id,
            "tool": tool_name,
            "arguments": encoded_arguments,
            "result": encoded_result,
            "state_before": state_before,
            "state_after": state_after,
        }
        if intent_id is not None:
            event["intent_evidence_id"] = intent_id
        self._write(event)
        return evidence_id

    def record_tool_intent(
        self,
        tool: str,
        arguments: dict,
        state_before: str,
    ) -> str:
        """Persist a pre-dispatch intent record for an effectful call."""

        if self._closed:
            raise EvidenceRecorderError("evidence recorder is closed")

        tool_name = _text(tool, "tool")
        _text(state_before, "state_before")
        if not isinstance(arguments, dict):
            raise EvidenceRecorderError("arguments must be a dict")

        encoded_arguments = _json_value(arguments, "arguments")
        evidence_id = self._next_event_id()
        event = {
            "schema_version": _SCHEMA_VERSION,
            "event": "tool_intent",
            "profile_id": self._profile.profile_id,
            "profile_sha256": self._profile_sha256,
            "created_at": _utc_timestamp(),
            "evidence_id": evidence_id,
            "tool": tool_name,
            "arguments": encoded_arguments,
            "state_before": state_before,
        }
        self._write(event)
        return evidence_id

    def record_tool_completion_failure(
        self,
        tool: str,
        intent_evidence_id: str,
        state_before: str,
        state_after: str,
        reason: str,
        stop_status: str,
        stop_reason: str | None,
    ) -> str:
        """Record the minimal terminal outcome when normal completion fails."""

        if self._closed:
            raise EvidenceRecorderError("evidence recorder is closed")

        tool_name = _text(tool, "tool")
        intent_id = _text(intent_evidence_id, "intent_evidence_id")
        _text(state_before, "state_before")
        _text(state_after, "state_after")
        failure_reason = _text(reason, "reason")
        durable_stop_status = _text(stop_status, "stop_status")
        durable_stop_reason = (
            _text(stop_reason, "stop_reason")
            if stop_reason is not None
            else None
        )
        evidence_id = self._next_event_id()
        event = {
            "schema_version": _SCHEMA_VERSION,
            "event": "tool_completion_failure",
            "profile_id": self._profile.profile_id,
            "profile_sha256": self._profile_sha256,
            "created_at": _utc_timestamp(),
            "evidence_id": evidence_id,
            "tool": tool_name,
            "intent_evidence_id": intent_id,
            "state_before": state_before,
            "state_after": state_after,
            "reason": failure_reason,
            "stop_status": durable_stop_status,
            "stop_reason": durable_stop_reason,
        }
        self._write(event)
        return evidence_id

    def record_operator_acknowledgement(self, operator: str) -> None:
        if self._closed:
            raise EvidenceRecorderError("evidence recorder is closed")

        operator_name = _text(operator, "operator")
        event = {
            "schema_version": _SCHEMA_VERSION,
            "event": "operator_acknowledgement",
            "profile_id": self._profile.profile_id,
            "profile_sha256": self._profile_sha256,
            "created_at": _utc_timestamp(),
            "evidence_id": self._next_event_id(),
            "operator": operator_name,
        }
        self._write(event)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._file.close()

    @property
    def profile_sha256(self) -> str:
        """Return the profile hash bound to this evidence stream."""

        return self._profile_sha256

    def _next_event_id(self) -> str:
        self._event_count += 1
        return f"evt-{self._event_count:06d}"

    def _write(self, event: dict[str, Any]) -> None:
        payload = json.dumps(
            event,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        self._file.write(f"{payload}\n")
        self._file.flush()
        os.fsync(self._file.fileno())


def _utc_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace(
        "+00:00",
        "Z",
    )
