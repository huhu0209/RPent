from __future__ import annotations

import json
from pathlib import Path

import pytest

from lynsense_webots_sim.viewer_state import ViewerRunStore

def make_store(tmp_path: Path) -> ViewerRunStore:
    artifact = tmp_path / "run-id"
    video = artifact / "video"
    video.mkdir(parents=True)
    event_path = artifact / "events.jsonl"
    event_path.touch()
    return ViewerRunStore(
        phase="match",
        run_id="run-id",
        event_path=event_path,
        artifact_dir=artifact,
        video_dir=video,
        video_start_mono=100.0,
    )

def append_events(path: Path, events: list[dict]) -> None:
    with path.open("a", encoding="utf-8") as stream:
        for event in events:
            stream.write(json.dumps(event, separators=(",", ":")) + "\n")

def test_initial_snapshot_is_starting_and_empty(tmp_path):
    snapshot = make_store(tmp_path).snapshot()
    assert snapshot["phase"] == "match"
    assert snapshot["run_id"] == "run-id"
    assert snapshot["status"] == "starting"
    assert snapshot["event_count"] == 0
    assert snapshot["events_total_index"] == -1
    assert snapshot["video_available"] is False
    assert snapshot["replay_available"] is False

def test_partial_json_line_is_not_ingested(tmp_path):
    store = make_store(tmp_path)
    store.event_path.write_bytes(b'{"kind":')
    assert store.ingest_new_events(101.0) == []
    assert store.snapshot()["event_count"] == 0

def test_events_have_monotonic_indexes_and_video_offsets(tmp_path):
    store = make_store(tmp_path)
    append_events(store.event_path, [
        {"kind": "run_started"},
        {"kind": "controller_ready"},
    ])
    events = store.ingest_new_events(102.5)
    assert [event["index"] for event in events] == [0, 1]
    assert events[1]["video_offset_s"] == pytest.approx(2.5)

def test_events_after_is_exclusive_and_can_resume(tmp_path):
    store = make_store(tmp_path)
    append_events(store.event_path, [{"kind": "run_started"}])
    store.ingest_new_events(101.0)
    append_events(store.event_path, [{"kind": "controller_ready"}])
    first = store.events_after(-1)
    assert store.events_after(first[-1]["index"]) == []
    ingested = store.ingest_new_events(102.0)
    assert [record["event"]["kind"] for record in ingested] == ["controller_ready"]
    second = store.events_after(first[-1]["index"])
    assert [event["event"]["kind"] for event in first] == ["run_started"]
    assert [event["event"]["kind"] for event in second] == ["controller_ready"]


def test_http_reads_do_not_consume_runtime_validation_events(tmp_path):
    store = make_store(tmp_path)
    append_events(store.event_path, [{"kind": "goal_accepted"}, {"kind": "action_result"}])
    assert store.events_after(-1) == []
    assert store.snapshot()["event_count"] == 0
    batch = store.ingest_new_events(102.0)
    assert [record["event"]["kind"] for record in batch] == ["goal_accepted", "action_result"]
    assert store.events_after(-1) == batch


def test_malformed_complete_line_fails_without_losing_offset(tmp_path):
    store = make_store(tmp_path)
    append_events(store.event_path, [{"kind": "run_started"}])
    store.ingest_new_events(101.0)
    with store.event_path.open("ab") as stream:
        stream.write(b"not-json\n")
    assert store.ingest_new_events(102.0) == []
    snapshot = store.snapshot()
    assert snapshot["status"] == "failed"
    assert "invalid JSONL event" in snapshot["reason"]
    assert snapshot["event_count"] == 1

@pytest.mark.parametrize("json_constant", ["NaN", "Infinity", "-Infinity", "1e999"])
def test_non_finite_json_constant_fails_without_losing_offset(
    tmp_path, json_constant
):
    store = make_store(tmp_path)
    append_events(store.event_path, [{"kind": "run_started"}])
    store.ingest_new_events(101.0)
    offset = store._byte_offset
    with store.event_path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps({"kind": "action_result", "value": 0.0}) + "\n")
        stream.write(f'{{"kind":"action_result","value":{json_constant}}}\n')

    assert store.ingest_new_events(102.0) == []
    snapshot = store.snapshot()
    assert snapshot["status"] == "failed"
    assert "invalid JSONL event" in snapshot["reason"]
    assert snapshot["event_count"] == 1
    assert store._byte_offset == offset
    store.write_summary("2026-09-21T00:00:00Z")


def test_terminal_evidence_keeps_its_own_index_in_a_batch(tmp_path):
    store = make_store(tmp_path)
    append_events(store.event_path, [
        {"kind": "action_result", "reason": "finished"},
        {"kind": "action_feedback"},
    ])
    records = store.ingest_new_events(102.0)
    assert store.snapshot()["terminal_event"] == records[0]

def test_process_statuses_and_video_start_can_be_recorded(tmp_path):
    store = make_store(tmp_path)
    store.set_video_start_mono(105.0)
    store.record_process_status("ffmpeg", 0)
    store.record_process_status("tree", 1, "interrupted")
    append_events(store.event_path, [{"kind": "controller_ready"}])
    store.ingest_new_events(106.0)

    snapshot = store.snapshot()
    assert snapshot["process_statuses"] == {
        "ffmpeg": {"exit_code": 0, "reason": ""},
        "tree": {"exit_code": 1, "reason": "interrupted"},
    }
    assert snapshot["events_total_index"] == 0

def test_map_metadata_and_wall_clock_range_are_snapshot_fields(tmp_path):
    store = make_store(tmp_path)
    store.set_map_metadata(
        {"length_m": 8.0, "width_m": 6.0, "boundary_x_m": 4.0, "boundary_y_m": 3.0},
        {"搬箱子1": {"x": 2.0249, "y": -2.4938, "yaw_deg": 178.9276}},
    )
    store.set_time_range("2026-09-21T00:00:00Z", None)

    snapshot = store.snapshot()
    assert snapshot["map"]["arena"]["width_m"] == 6.0
    assert snapshot["map"]["goals"]["搬箱子1"]["x"] == 2.0249
    assert snapshot["started_at"] == "2026-09-21T00:00:00Z"
    assert snapshot["ended_at"] is None

def test_terminal_result_updates_telemetry_and_terminal_evidence(tmp_path):
    store = make_store(tmp_path)
    terminal = {
        "kind": "action_result",
        "goal_name": "放箱子1_1",
        "pose": {"x": 2.46, "y": -2.63, "yaw": 0.006},
        "motor_rates": {
            "left_wheel_rate_radps": 0.0,
            "right_wheel_rate_radps": 0.0,
        },
    }
    append_events(store.event_path, [terminal])
    store.ingest_new_events(110.0)
    snapshot = store.snapshot()
    assert snapshot["terminal_event"]["event"]["goal_name"] == "放箱子1_1"
    assert snapshot["pose"] == terminal["pose"]
    assert snapshot["motor_rates"] == terminal["motor_rates"]

def test_summary_is_finite_and_not_real_robot_connected(tmp_path):
    store = make_store(tmp_path)
    store.set_video_available(True)
    store.write_summary("2026-09-21T00:00:00Z")
    summary = json.loads(
        (store.artifact_dir / "viewer-summary.json").read_text(encoding="utf-8")
    )
    assert summary["real_robot_connected"] is False
    assert summary["video_available"] is True
    assert summary["event_count"] == 0
    json.dumps(summary, allow_nan=False)
