from __future__ import annotations

import copy
import json
import math
import threading
from pathlib import Path
from typing import Any


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON constant: {value}")


def _parse_finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError("non-finite JSON number")
    return parsed


class ViewerRunStore:
    def __init__(
        self,
        phase: str,
        run_id: str,
        event_path: Path,
        artifact_dir: Path,
        video_dir: Path,
        video_start_mono: float | None = None,
    ) -> None:
        self.phase = phase
        self.run_id = run_id
        self.event_path = event_path
        self.artifact_dir = artifact_dir
        self.video_dir = video_dir
        self.video_start_mono = video_start_mono
        self._lock = threading.RLock()
        self._byte_offset = 0
        self._events: list[dict[str, Any]] = []
        self._status = "starting"
        self._reason = ""
        self._current_event: dict[str, Any] | None = None
        self._terminal_event: dict[str, Any] | None = None
        self._pose: dict[str, float] | None = None
        self._motor_rates: dict[str, float] | None = None
        self._manipulation: dict[str, Any] = {
            "waist_position_mm": None,
            "arm_positions_rad": None,
            "gripper_positions": None,
            "max_arm_joint_error_rad": None,
        }
        self._box: dict[str, Any] | None = None
        self._video_available = False
        self._replay_available = False
        self._process_statuses: dict[str, dict[str, Any]] = {}
        self._map: dict[str, Any] = {"arena": {}, "goals": {}}
        self._started_at: str | None = None
        self._ended_at: str | None = None

    def ingest_new_events(self, now_mono: float) -> list[dict[str, Any]]:
        with self._lock:
            if self._status == "failed":
                return []

            with self.event_path.open("rb") as stream:
                stream.seek(self._byte_offset)
                raw_events = stream.read()
            if not raw_events or not raw_events.endswith(b"\n"):
                return []

            try:
                text_events = raw_events.decode("utf-8")
            except UnicodeDecodeError as exc:
                self._fail(f"invalid JSONL event: invalid UTF-8 ({exc})")
                return []

            complete_lines = text_events.splitlines()
            parsed_events: list[dict[str, Any]] = []
            for line_number, line in enumerate(complete_lines, start=1):
                try:
                    event = json.loads(
                        line,
                        parse_constant=_reject_json_constant,
                        parse_float=_parse_finite_float,
                    )
                except ValueError as exc:
                    self._fail(
                        f"invalid JSONL event at line {line_number}: {exc}"
                    )
                    return []
                if not isinstance(event, dict):
                    self._fail(
                        f"invalid JSONL event at line {line_number}: expected object"
                    )
                    return []
                parsed_events.append(event)

            first_index = len(self._events)
            accepted_events = [
                {
                    "index": first_index + offset,
                    "event": copy.deepcopy(event),
                    "video_offset_s": (
                        now_mono - self.video_start_mono
                        if self.video_start_mono is not None
                        else None
                    ),
                }
                for offset, event in enumerate(parsed_events)
            ]

            self._events.extend(accepted_events)
            self._byte_offset += len(raw_events)
            for record in accepted_events:
                self._record_event(record)

            return copy.deepcopy(accepted_events)

    def events_after(self, index: int, limit: int = 512) -> list[dict[str, Any]]:
        with self._lock:
            start = index + 1
            return copy.deepcopy(self._events[start : start + limit])

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            total_index = self._events[-1]["index"] if self._events else -1
            return {
                "phase": self.phase,
                "run_id": self.run_id,
                "status": self._status,
                "reason": self._reason,
                "event_count": len(self._events),
                "events_total_index": total_index,
                "current_event": copy.deepcopy(self._current_event),
                "terminal_event": copy.deepcopy(self._terminal_event),
                "pose": copy.deepcopy(self._pose),
                "motor_rates": copy.deepcopy(self._motor_rates),
                "manipulation": copy.deepcopy(self._manipulation),
                "box": copy.deepcopy(self._box),
                "video_available": self._video_available,
                "replay_available": self._replay_available,
                "video_start_mono": self.video_start_mono,
                "process_statuses": copy.deepcopy(self._process_statuses),
                "map": copy.deepcopy(self._map),
                "started_at": self._started_at,
                "ended_at": self._ended_at,
                "event_path": str(self.event_path),
                "artifact_dir": str(self.artifact_dir),
                "video_dir": str(self.video_dir),
            }

    def set_status(self, status: str, reason: str = "") -> None:
        with self._lock:
            self._status = status
            self._reason = reason

    def set_video_available(self, available: bool) -> None:
        with self._lock:
            self._video_available = available

    def set_replay_available(self, available: bool) -> None:
        with self._lock:
            self._replay_available = available

    def set_video_start_mono(self, now_mono: float) -> None:
        with self._lock:
            self.video_start_mono = now_mono

    def record_process_status(
        self, name: str, exit_code: int | None, reason: str = ""
    ) -> None:
        with self._lock:
            self._process_statuses[name] = {
                "exit_code": exit_code,
                "reason": reason,
            }

    def set_map_metadata(
        self,
        arena: dict[str, float],
        goals: dict[str, dict[str, float]],
    ) -> None:
        with self._lock:
            self._map = {
                "arena": copy.deepcopy(arena),
                "goals": copy.deepcopy(goals),
            }

    def set_time_range(self, started_at: str, ended_at: str | None) -> None:
        with self._lock:
            self._started_at = started_at
            self._ended_at = ended_at

    def write_summary(self, generated_at: str) -> None:
        with self._lock:
            summary = self.snapshot()
            summary["generated_at"] = generated_at
            summary["real_robot_connected"] = False
            path = self.artifact_dir / "viewer-summary.json"
            path.write_text(
                json.dumps(summary, sort_keys=True, allow_nan=False) + "\n",
                encoding="utf-8",
            )

    def _record_event(self, record: dict[str, Any]) -> None:
        event = record["event"]
        self._current_event = copy.deepcopy(event)
        if "pose" in event:
            self._pose = copy.deepcopy(event["pose"])
        if "motor_rates" in event:
            self._motor_rates = copy.deepcopy(event["motor_rates"])
        if "waist_position_m" in event:
            self._manipulation["waist_position_mm"] = (
                float(event["waist_position_m"]) * 1000.0
            )
        if "arm_positions_rad" in event:
            self._manipulation["arm_positions_rad"] = copy.deepcopy(
                event["arm_positions_rad"]
            )
        if "gripper_positions" in event:
            self._manipulation["gripper_positions"] = copy.deepcopy(
                event["gripper_positions"]
            )
        if "max_joint_error_rad" in event:
            self._manipulation["max_arm_joint_error_rad"] = copy.deepcopy(
                event["max_joint_error_rad"]
            )
        if "box" in event:
            self._box = copy.deepcopy(event["box"])
        if event.get("kind") == "action_result":
            self._terminal_event = copy.deepcopy(record)

    def _fail(self, reason: str) -> None:
        self._status = "failed"
        self._reason = reason
