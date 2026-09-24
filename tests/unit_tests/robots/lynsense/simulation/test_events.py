import builtins
import json
import math
import threading

import pytest

from lynsense_webots_sim.events import EventRecorder


def test_event_recorder_writes_one_json_object_per_line(tmp_path):
    path = tmp_path / "events.jsonl"
    recorder = EventRecorder(path, "run-1")
    recorder.record({"kind": "run_started", "action": "controller", "sim_time_s": 0.0})
    lines = path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 1
    assert json.loads(lines[0])["run_id"] == "run-1"


def test_event_recorder_serializes_concurrent_unique_records(tmp_path):
    path = tmp_path / "events.jsonl"
    recorder = EventRecorder(path, "concurrent-run")
    event_count = 32
    barrier = threading.Barrier(event_count)
    failures = []

    def record_event(index):
        try:
            barrier.wait(timeout=2.0)
            recorder.record(
                {
                    "kind": f"event-{index}",
                    "action": f"action-{index}",
                    "sim_time_s": float(index),
                    "value": index,
                }
            )
        except BaseException as exc:
            failures.append(exc)

    threads = [
        threading.Thread(target=record_event, args=(index,), daemon=True)
        for index in range(event_count)
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=2.0)
    recorder.close()

    assert failures == []
    content = path.read_text(encoding="utf-8")
    assert content.endswith("\n")
    events = [json.loads(line) for line in content.splitlines()]
    assert len(events) == event_count
    assert {(event["kind"], event["action"], event["value"]) for event in events} == {
        (f"event-{index}", f"action-{index}", index)
        for index in range(event_count)
    }


def test_event_recorder_appends_records_deterministically_in_utf8(tmp_path):
    path = tmp_path / "events.jsonl"
    recorder = EventRecorder(path, "run-2")
    first = {"kind": "a", "action": "controller", "sim_time_s": 1.0}
    second = {"sim_time_s": 2.0, "action": "controller", "kind": "b", "text": "目标"}
    recorder.record(first)
    recorder.record(second)
    lines = path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 2
    assert json.loads(lines[0])["run_id"] == "run-2"
    assert json.loads(lines[1])["text"] == "目标"
    assert lines[0] == json.dumps(
        {**first, "run_id": "run-2"},
        sort_keys=True,
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
    )
    recorder.close()


def test_new_recorder_starts_a_new_file_generation(tmp_path):
    path = tmp_path / "events.jsonl"
    first = EventRecorder(path, "run-1")
    first.record({"kind": "old", "action": "controller", "sim_time_s": 0.0})
    first.close()
    second = EventRecorder(path, "run-2")
    second.record({"kind": "new", "action": "controller", "sim_time_s": 0.1})
    second.close()
    lines = path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 1
    assert json.loads(lines[0])["run_id"] == "run-2"


def test_same_run_recorder_appends_across_controller_lifetimes(tmp_path):
    path = tmp_path / "events.jsonl"
    first = EventRecorder(path, "same-run")
    first.record({"kind": "first", "action": "controller", "sim_time_s": 0.0})
    first.close()

    second = EventRecorder(path, "same-run")
    second.record({"kind": "second", "action": "controller", "sim_time_s": 1.0})
    second.close()

    lines = path.read_text(encoding="utf-8").splitlines()
    assert [json.loads(line)["kind"] for line in lines] == ["first", "second"]


def test_event_recorder_rejects_invalid_events(tmp_path):
    recorder = EventRecorder(tmp_path / "events.jsonl", "run-3")
    with pytest.raises(ValueError, match="kind"):
        recorder.record({"action": "controller", "sim_time_s": 0.0})
    with pytest.raises(ValueError, match="sim_time_s"):
        recorder.record({"kind": "event", "action": "controller"})
    with pytest.raises(ValueError, match="action"):
        recorder.record({"kind": "event", "sim_time_s": 0.0})
    with pytest.raises(ValueError, match="run_id"):
        recorder.record(
            {"kind": "event", "action": "controller", "sim_time_s": 0.0, "run_id": "other"}
        )
    with pytest.raises(ValueError, match="finite"):
        recorder.record(
            {
                "kind": "event",
                "action": "controller",
                "sim_time_s": math.nan,
                "nested": {"value": math.inf},
            }
        )
    with pytest.raises(ValueError, match="UTF-8"):
        recorder.record(
            {
                "kind": "event",
                "action": "controller",
                "sim_time_s": 0.0,
                chr(0xD800): "value",
            }
        )
    recorder.close()


def test_event_recorder_flushes_and_fsynchronizes_each_record(monkeypatch, tmp_path):
    class RecordingFile:
        def __init__(self):
            self.lines = []
            self.flush_count = 0
            self.sync_count = 0

        def write(self, line):
            self.lines.append(line)

        def flush(self):
            self.flush_count += 1

        def fileno(self):
            return 123

    file = RecordingFile()
    recorder = EventRecorder(file, "run-4")
    fsync_calls = []
    def fake_fsync(fd):
        fsync_calls.append(fd)
        file.sync_count += 1

    monkeypatch.setattr("lynsense_webots_sim.events.os.fsync", fake_fsync)
    event = {"kind": "event", "action": "controller", "sim_time_s": 0.0}
    recorder.record(event)
    recorder.record(event)
    recorder.close()
    assert file.lines == [
        '{"action":"controller","kind":"event","run_id":"run-4","sim_time_s":0.0}\n',
        '{"action":"controller","kind":"event","run_id":"run-4","sim_time_s":0.0}\n',
    ]
    assert file.flush_count == 2
    assert len(fsync_calls) == 2
    assert file.sync_count == 2


def test_event_recorder_close_waits_for_in_progress_record(monkeypatch, tmp_path):
    path = tmp_path / "events.jsonl"
    recorder = EventRecorder(path, "close-mutex")
    fsync_entered = threading.Event()
    release_fsync = threading.Event()

    def blocking_fsync(_fd):
        fsync_entered.set()
        assert release_fsync.wait(timeout=2.0)

    monkeypatch.setattr("lynsense_webots_sim.events.os.fsync", blocking_fsync)
    record_thread = threading.Thread(
        target=recorder.record,
        args=({"kind": "event", "action": "controller", "sim_time_s": 0.0},),
        daemon=True,
    )
    record_thread.start()
    assert fsync_entered.wait(timeout=1.0)

    close_thread = threading.Thread(target=recorder.close, daemon=True)
    close_thread.start()
    close_thread.join(timeout=0.05)
    assert close_thread.is_alive()

    release_fsync.set()
    record_thread.join(timeout=1.0)
    close_thread.join(timeout=1.0)
    assert not record_thread.is_alive()
    assert not close_thread.is_alive()
    assert json.loads(path.read_text(encoding="utf-8"))["run_id"] == "close-mutex"


def test_open_failure_propagates_before_any_record(monkeypatch, tmp_path):
    class FailingOpen:
        def __enter__(self):
            raise OSError("open failed")

        def __exit__(self, exc_type, exc, traceback):
            return False

    monkeypatch.setattr(builtins, "open", lambda *args, **kwargs: FailingOpen())
    with pytest.raises(OSError, match="open failed"):
        EventRecorder(tmp_path / "events.jsonl", "run-5")


def test_write_failure_propagates_and_claims_no_success(tmp_path):
    class FailingFile:
        def __init__(self):
            self.flushed = False
            self.synced = False

        def write(self, line):
            raise OSError("write failed")

        def flush(self):
            self.flushed = True

        def fileno(self):
            return 123

    file = FailingFile()
    recorder = EventRecorder(file, "run-6")
    with pytest.raises(OSError, match="write failed"):
        recorder.record({"kind": "event", "action": "controller", "sim_time_s": 0.0})
    assert file.flushed is False
    assert file.synced is False


def test_partial_write_failure_leaves_no_unnewline_fragment():
    class PartialWritingFile:
        def __init__(self):
            self.content = ""

        def tell(self):
            return len(self.content)

        def write(self, line):
            self.content += line[:7]
            raise OSError("write failed")

        def seek(self, position):
            assert position == 0
            self.content = ""

        def truncate(self, size=None):
            assert size == 0
            self.content = ""

        def flush(self):
            pass

        def fileno(self):
            return 123

    file = PartialWritingFile()
    recorder = EventRecorder(file, "run-7")
    with pytest.raises(OSError, match="write failed"):
        recorder.record({"kind": "event", "action": "controller", "sim_time_s": 0.0})
    assert file.content == ""


def test_rollback_failure_does_not_mask_original_write_failure():
    class FailingRollbackFile:
        def __init__(self):
            self.content = ""

        def tell(self):
            return len(self.content)

        def write(self, line):
            self.content += line[:3]
            raise OSError("original write failed")

        def seek(self, position):
            raise OSError("rollback seek failed")

        def truncate(self, size=None):
            raise OSError("rollback truncate failed")

        def flush(self):
            pass

        def fileno(self):
            return 123

    file = FailingRollbackFile()
    recorder = EventRecorder(file, "run-10")
    with pytest.raises(OSError, match="original write failed"):
        recorder.record({"kind": "event", "action": "controller", "sim_time_s": 0.0})
    assert file.content == '{"a'


def test_flush_failure_propagates_without_fsync():
    class FlushFailureFile:
        def __init__(self):
            self.content = ""
            self.synced = False

        def write(self, line):
            self.content += line

        def flush(self):
            raise OSError("flush failed")

        def fileno(self):
            return 123

    file = FlushFailureFile()
    recorder = EventRecorder(file, "run-8")
    with pytest.raises(OSError, match="flush failed"):
        recorder.record({"kind": "event", "action": "controller", "sim_time_s": 0.0})
    assert file.content.endswith("\n")
    assert file.synced is False


def test_fsync_failure_propagates_after_one_write_and_flush(monkeypatch):
    class SyncFailureFile:
        def __init__(self):
            self.content = ""
            self.flush_count = 0

        def write(self, line):
            self.content += line

        def flush(self):
            self.flush_count += 1

        def fileno(self):
            return 123

    file = SyncFailureFile()
    recorder = EventRecorder(file, "run-9")

    def failing_fsync(fd):
        assert fd == 123
        raise OSError("fsync failed")

    monkeypatch.setattr("lynsense_webots_sim.events.os.fsync", failing_fsync)
    with pytest.raises(OSError, match="fsync failed"):
        recorder.record({"kind": "event", "action": "controller", "sim_time_s": 0.0})
    assert file.content.endswith("\n")
    assert file.flush_count == 1
