from __future__ import annotations

import json
import math
import os
import threading
import unicodedata
from pathlib import Path
from typing import IO, Any


_REQUIRED_FIELDS = ("kind", "sim_time_s", "action", "run_id")


def _string_is_utf8(value: str) -> bool:
    try:
        value.encode("utf-8")
    except UnicodeEncodeError:
        return False
    return True


def _run_id_is_valid(run_id: str) -> bool:
    if not isinstance(run_id, str) or not run_id:
        return False
    try:
        encoded = run_id.encode("utf-8")
    except UnicodeEncodeError:
        return False
    return len(encoded) <= 128 and not any(
        unicodedata.category(character) == "Cc" for character in run_id
    )


def _values_are_finite(value: Any) -> bool:
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, int):
        return True
    if isinstance(value, str):
        return _string_is_utf8(value)
    if isinstance(value, dict):
        return all(_values_are_finite(item) for item in value.values())
    if isinstance(value, (list, tuple)):
        return all(_values_are_finite(item) for item in value)
    return True


def _keys_are_utf8(value: Any) -> bool:
    if isinstance(value, dict):
        return all(
            _string_is_utf8(key) and _keys_are_utf8(item)
            for key, item in value.items()
        )
    if isinstance(value, (list, tuple)):
        return all(_keys_are_utf8(item) for item in value)
    return True


class EventRecorder:
    def __init__(self, path: Path | IO[str], run_id: str) -> None:
        if not _run_id_is_valid(run_id):
            raise ValueError("run_id must be a non-empty UTF-8 string")
        self._run_id = run_id
        self._owns_file = not hasattr(path, "write")
        if self._owns_file:
            mode = "a"
            try:
                with open(path, "r", encoding="utf-8") as existing:
                    first_line = existing.readline()
                if first_line:
                    existing_run_id = json.loads(first_line).get("run_id")
                    if existing_run_id != run_id:
                        mode = "w"
            except (OSError, json.JSONDecodeError, UnicodeDecodeError):
                mode = "w"
            self._file = open(
                os.fspath(path), mode, encoding="utf-8", newline="\n"
            ).__enter__()
        else:
            self._file = path
        self._lock = threading.Lock()

    def record(self, event: dict) -> None:
        with self._lock:
            if not isinstance(event, dict):
                raise ValueError("event must be a dict")
            if "run_id" in event and event["run_id"] != self._run_id:
                raise ValueError("run_id does not match recorder")
            event = {**event, "run_id": self._run_id}
            for field in _REQUIRED_FIELDS:
                if field not in event:
                    raise ValueError(f"event is missing required field: {field}")
            if not isinstance(event["kind"], str) or not event["kind"]:
                raise ValueError("kind must be a non-empty string")
            if not isinstance(event["action"], str) or not event["action"]:
                raise ValueError("action must be a non-empty string")
            if not math.isfinite(event["sim_time_s"]):
                raise ValueError("sim_time_s must be finite")
            if not _keys_are_utf8(event):
                raise ValueError("event keys must be valid UTF-8")
            if not _values_are_finite(event):
                raise ValueError("event values must be finite")

            line = json.dumps(
                event,
                sort_keys=True,
                ensure_ascii=False,
                allow_nan=False,
                separators=(",", ":"),
            )
            self._write_line(line + "\n")
            self._file.flush()
            os.fsync(self._file.fileno())

    def _write_line(self, line: str) -> None:
        rollback_position = self._file.tell() if hasattr(self._file, "tell") else None
        try:
            written = self._file.write(line)
        except Exception:
            self._rollback(rollback_position)
            raise
        if written is not None and written != len(line):
            self._rollback(rollback_position)
            raise OSError("short write")

    def _rollback(self, position: int | None) -> None:
        if position is None or not all(
            hasattr(self._file, method) for method in ("seek", "truncate")
        ):
            return
        try:
            self._file.seek(position)
            self._file.truncate(position)
        except Exception:
            # A failed rollback must not replace the original write failure.
            return

    def close(self) -> None:
        with self._lock:
            if self._owns_file and not self._file.closed:
                self._file.close()

    def __enter__(self) -> "EventRecorder":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()
