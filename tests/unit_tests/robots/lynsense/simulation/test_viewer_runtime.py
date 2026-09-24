from __future__ import annotations

import json
import configparser
import os
from pathlib import Path
import subprocess
import sys
import threading

import pytest

from lynsense_webots_sim.scripts import run_viewer


class _FakeHTTPServer:
    def __init__(self, alive: bool):
        self.alive = alive

    def is_alive(self):
        return self.alive


def test_server_liveness_failure_records_reason_and_requests_shutdown():
    from lynsense_webots_sim.viewer_state import ViewerRunStore

    store = ViewerRunStore("match", "run", Path(os.devnull), Path("."), Path("."))
    shutdown = threading.Event()

    with pytest.raises(run_viewer.ViewerError, match="HTTP server thread exited"):
        run_viewer._require_server_alive(_FakeHTTPServer(alive=False), store, shutdown)

    assert shutdown.is_set()
    assert store.snapshot()["status"] == "failed"
    assert store.snapshot()["process_statuses"]["http"]["reason"] == (
        "HTTP server thread exited unexpectedly"
    )


def test_server_liveness_check_allows_active_viewer():
    from lynsense_webots_sim.viewer_state import ViewerRunStore

    store = ViewerRunStore("match", "run", Path(os.devnull), Path("."), Path("."))
    shutdown = threading.Event()
    run_viewer._require_server_alive(_FakeHTTPServer(alive=True), store, shutdown)
    assert not shutdown.is_set()


def test_cli_accepts_navigation_and_box_viewer_phases():
    parser = run_viewer._build_parser()
    choices = [action.choices for action in parser._actions if action.dest == "phase"]
    assert choices == [{"smoke", "blocked", "match", "box"}]


def test_realtime_webots_command_has_rendering_enabled():
    command = run_viewer._webots_command("match")
    assert "--mode=realtime" in command
    assert "--no-rendering" not in command
    assert command[-1].endswith(".wbt")


def test_xvfb_is_fixed_local_and_sized_for_capture():
    assert run_viewer._xvfb_command(":99") == [
        "Xvfb", ":99", "-screen", "0", "1280x1024x24", "-nolisten", "tcp"
    ]


def test_webots_readiness_gates_controller_start(tmp_path, monkeypatch):
    class LiveProcess:
        returncode = None

        def poll(self):
            return None

    class Server(_FakeHTTPServer):
        def __init__(self, *args):
            super().__init__(True)

        def start(self):
            pass

    started = []
    def start(command, **kwargs):
        started.append((command, kwargs))
        return LiveProcess()

    def wait(process, log_path):
        assert started[-1][0][0] == "webots"
        assert log_path.name == "webots.log"
        assert started[-1][1]["env"]["LIBGL_ALWAYS_SOFTWARE"] == "1"
        assert started[-1][1]["env"]["QT_OPENGL"] == "software"
        raise run_viewer.ViewerError("world not ready")

    monkeypatch.setattr(run_viewer, "ARTIFACT_ROOT", tmp_path)
    monkeypatch.setattr(run_viewer, "_load_map_metadata", lambda phase: ({}, {}))
    monkeypatch.setattr(run_viewer.viewer_server, "ViewerHTTPServer", Server)
    monkeypatch.setattr(run_viewer.subprocess, "Popen", start)
    monkeypatch.setattr(run_viewer.run_smoke, "_wait_for_webots_ready", wait)
    monkeypatch.setattr(
        run_viewer.run_smoke, "_start_controller",
        lambda *args, **kwargs: pytest.fail("controller started before world was ready"),
    )
    monkeypatch.setattr(run_viewer, "_cleanup", lambda *args: None)

    assert run_viewer._run_viewer("match") == 1
    summary = json.loads(next(tmp_path.glob("*/viewer-summary.json")).read_text())
    assert summary["reason"] == "ViewerError: world not ready"
    assert len(started) == 2


def test_ffmpeg_command_contains_capture_and_hls_contract(tmp_path: Path):
    command = run_viewer._ffmpeg_command(tmp_path / "frames", tmp_path / "video")
    joined = " ".join(command)
    assert "x11grab" not in command
    assert "10" in command
    assert command.count("-i") == 2
    assert "libx264" in command
    assert "-hls_time 1" in joined
    for view in ("scene", "robot"):
        assert str(tmp_path / "frames" / f"{view}.jpg") in command
        assert str(tmp_path / "video" / view / "index.m3u8") in command
        assert str(tmp_path / "video" / view / "segment-%06d.ts") in command


def test_rendering_preferences_are_run_local_and_hide_camera_overlay(tmp_path):
    environment = {}
    run_viewer._configure_rendering(tmp_path, environment)
    root = Path(environment["XDG_CONFIG_HOME"])
    assert root.parent == tmp_path
    config = configparser.ConfigParser(interpolation=None)
    config.read(root / "Cyberbotics" / "Webots-R2025a.conf")
    assert config.getboolean("View3d", "hideAllCameraOverlays")
    assert not config.getboolean("%General", "checkWebotsUpdateOnStartup")


def test_smoke_environment_does_not_inherit_viewer_camera_activation(tmp_path, monkeypatch):
    monkeypatch.setenv("LYNSENSE_VIEWER_CAPTURE_DIR", "/unintended/capture")
    environment = run_viewer.run_smoke._command_environment("smoke", tmp_path / "events.jsonl", "run")
    assert "LYNSENSE_VIEWER_CAPTURE_DIR" not in environment


def test_camera_frame_wait_requires_both_views(tmp_path):
    store = run_viewer.viewer_state.ViewerRunStore("match", "run", Path(os.devnull), tmp_path, tmp_path)
    (tmp_path / "scene.jpg").write_bytes(b"scene")
    with pytest.raises(run_viewer.ViewerError, match="camera frames did not arrive"):
        run_viewer._wait_for_camera_frames(run_viewer.ViewerProcesses(), store, tmp_path, timeout=0)
    (tmp_path / "robot.jpg").write_bytes(b"robot")
    run_viewer._wait_for_camera_frames(run_viewer.ViewerProcesses(), store, tmp_path)


def test_replay_command_is_stream_copy(tmp_path: Path):
    command = run_viewer._replay_command(tmp_path / "index.m3u8", tmp_path / "replay.mp4")
    assert command == [
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-i",
        str(tmp_path / "index.m3u8"), "-c", "copy", "-movflags", "+faststart",
        str(tmp_path / "replay.mp4"),
    ]


def test_wait_for_hls_playlist_fails_when_ffmpeg_exits(tmp_path: Path):
    process = subprocess.Popen(["/bin/sh", "-c", "exit 4"])
    with pytest.raises(run_viewer.ViewerError, match="ffmpeg exited"):
        run_viewer._wait_for_hls_playlist(process, tmp_path / "index.m3u8", timeout=1)


def test_viewer_endpoint_validates_environment(monkeypatch):
    monkeypatch.setenv("LYNSENSE_VIEWER_HOST", "127.0.0.1")
    monkeypatch.setenv("LYNSENSE_VIEWER_PORT", "17047")
    assert run_viewer._viewer_endpoint() == ("127.0.0.1", 17047)
    monkeypatch.setenv("LYNSENSE_VIEWER_PORT", "not-a-port")
    with pytest.raises(run_viewer.ViewerError, match="integer"):
        run_viewer._viewer_endpoint()
    monkeypatch.setenv("LYNSENSE_VIEWER_PORT", "65536")
    with pytest.raises(run_viewer.ViewerError, match="between"):
        run_viewer._viewer_endpoint()


def test_process_supervision_records_unexpected_exit():
    class FakeProcess:
        pid = 123
        returncode = 17

        def poll(self):
            return self.returncode

    from lynsense_webots_sim.viewer_state import ViewerRunStore

    store = ViewerRunStore("match", "run", Path(os.devnull), Path("."), Path("."))
    processes = run_viewer.ViewerProcesses()
    processes.webots = FakeProcess()
    assert run_viewer._unexpected_exit(processes, store) == "webots exited with code 17"
    assert store.snapshot()["process_statuses"]["webots"] == {
        "exit_code": 17,
        "reason": "",
    }


def test_successful_terminal_run_waits_for_operator_shutdown():
    shutdown = threading.Event()
    calls: list[str] = []

    class LiveProcess:
        returncode = None

        def poll(self):
            return None

    from lynsense_webots_sim.viewer_state import ViewerRunStore

    store = ViewerRunStore("match", "run", Path(os.devnull), Path("."), Path("."))
    processes = run_viewer.ViewerProcesses()
    for name in ("xvfb", "webots", "controller", "ffmpeg", "tree"):
        setattr(processes, name, LiveProcess())

    def release():
        calls.append("waited")
        shutdown.set()

    thread = threading.Thread(target=lambda: (release(),), daemon=True)
    thread.start()
    assert run_viewer._wait_for_shutdown(
        shutdown, processes, _FakeHTTPServer(alive=True), store
    ) is None
    thread.join(timeout=1)
    assert calls == ["waited"]


def test_post_terminal_wait_fails_when_child_exits():
    shutdown = threading.Event()

    class FakeProcess:
        returncode = 9

        def poll(self):
            return self.returncode

    from lynsense_webots_sim.viewer_state import ViewerRunStore

    store = ViewerRunStore("match", "run", Path(os.devnull), Path("."), Path("."))
    processes = run_viewer.ViewerProcesses()
    processes.webots = FakeProcess()

    reason = run_viewer._wait_for_shutdown(
        shutdown, processes, _FakeHTTPServer(alive=True), store
    )

    assert reason == "webots exited with code 9"
    assert shutdown.is_set()
    assert store.snapshot()["status"] == "failed"
    assert store.snapshot()["process_statuses"]["webots"]["exit_code"] == 9


def test_post_terminal_wait_fails_when_http_thread_exits():
    shutdown = threading.Event()

    class LiveProcess:
        returncode = None

        def poll(self):
            return None

    from lynsense_webots_sim.viewer_state import ViewerRunStore

    store = ViewerRunStore("match", "run", Path(os.devnull), Path("."), Path("."))
    processes = run_viewer.ViewerProcesses()
    for name in ("xvfb", "webots", "controller", "ffmpeg", "tree"):
        setattr(processes, name, LiveProcess())

    reason = run_viewer._wait_for_shutdown(
        shutdown, processes, _FakeHTTPServer(alive=False), store
    )

    assert reason == "HTTP server thread exited unexpectedly"
    assert shutdown.is_set()
    assert store.snapshot()["status"] == "failed"
    assert store.snapshot()["process_statuses"]["http"]["reason"] == reason


def test_run_uses_per_run_logs_and_keeps_replay_server_alive():
    source = Path(
        "robots/lynsense/simulation/ros/lynsense_webots_sim/"
        "lynsense_webots_sim/scripts/run_viewer.py"
    ).read_text(encoding="utf-8")
    assert 'FILE_LOG_NAMES = ("webots", "controller", "tree", "ffmpeg")' in source
    assert 'f"{name}.log"' in source
    assert "_wait_for_shutdown(shutdown, processes, server, store)" in source
    assert "store.set_video_start_mono(time.monotonic())" in source
    assert "store.set_video_available(True)" in source
    assert source.count("_require_server_alive(server, store, shutdown)") >= 3


def test_finalize_video_adds_endlist_and_marks_replay(tmp_path: Path, monkeypatch):
    for view in ("scene", "robot"):
        (tmp_path / view).mkdir()
        (tmp_path / view / "index.m3u8").write_text("#EXTM3U\n", encoding="utf-8")
    replay = tmp_path / "replay.mp4"
    process = subprocess.Popen(["/bin/sh", "-c", "exit 0"])
    monkeypatch.setattr(
        run_viewer.subprocess,
        "run",
        lambda *args, **kwargs: (
            Path(args[0][-1]).write_bytes(b"mp4"),
            subprocess.CompletedProcess(args, 0),
        )[1],
    )
    from lynsense_webots_sim.viewer_state import ViewerRunStore

    event_path = tmp_path / "events.jsonl"
    event_path.touch()
    store = ViewerRunStore("match", "run", event_path, tmp_path, tmp_path)
    run_viewer._finalize_video(process, tmp_path, replay, store)
    for view in ("scene", "robot"):
        assert (tmp_path / view / "index.m3u8").read_text().endswith("#EXT-X-ENDLIST\n")
    assert (tmp_path / "replay-robot.mp4").is_file()
    assert store.snapshot()["replay_available"] is True


def test_setup_declares_viewer_entry_point():
    source = Path("robots/lynsense/simulation/ros/lynsense_webots_sim/setup.py").read_text()
    assert "lynsense_run_viewer = lynsense_webots_sim.scripts.run_viewer:main" in source


@pytest.mark.parametrize("failure", [None, "video", "summary"])
def test_first_tree_batch_is_validated_and_finalization_always_cleans_up(
    tmp_path, monkeypatch, failure
):
    class Process:
        returncode = None

        def poll(self):
            return None

    class Server(_FakeHTTPServer):
        def __init__(self, *args):
            super().__init__(True)

        def start(self):
            pass

    logs, cleaned = [], []
    def start(command, **kwargs):
        logs.append(kwargs["stdout"])
        return Process()

    def start_tree(environment, phase, run_id, **kwargs):
        events = []
        for action, goal_name, movement in run_viewer.run_smoke.MATCH_EXPECTED_SCRIPT:
            base = {"action": action, "goal_name": goal_name, "run_id": run_id}
            if movement:
                base.update(distance_m=movement[0], angle_deg=movement[1])
            events.append({**base, "kind": "goal_accepted"})
            events.append({
                **base, "kind": "action_result", "success": True,
                "pose": dict(zip(("x", "y", "yaw"), run_viewer.run_smoke.MATCH_FINAL_POSE)),
                "motor_rates": {"left_wheel_rate_radps": 0.0, "right_wheel_rate_radps": 0.0},
            })
        Path(environment["LYNSENSE_SIM_EVENT_FILE"]).write_text(
            "".join(json.dumps(event) + "\n" for event in events), encoding="utf-8"
        )
        return Process()

    def finalize(*args):
        if failure == "video":
            raise OSError("disk full")

    def fail_summary(*args):
        raise OSError("summary unavailable")

    monkeypatch.setattr(run_viewer, "ARTIFACT_ROOT", tmp_path)
    monkeypatch.setattr(run_viewer, "_load_map_metadata", lambda phase: ({}, {}))
    monkeypatch.setattr(run_viewer.viewer_server, "ViewerHTTPServer", Server)
    monkeypatch.setattr(run_viewer.subprocess, "Popen", start)
    for name in ("_wait_for_webots_ready", "_wait_for_ready", "_wait_for_action_graph", "_signal_process_group"):
        monkeypatch.setattr(run_viewer.run_smoke, name, lambda *args, **kwargs: None)
    monkeypatch.setattr(run_viewer.run_smoke, "_start_controller", lambda *args, **kwargs: Process())
    monkeypatch.setattr(run_viewer.run_smoke, "_start_tree", start_tree)
    monkeypatch.setattr(run_viewer, "_wait_for_hls_playlist", lambda *args: None)
    monkeypatch.setattr(run_viewer, "_wait_for_camera_frames", lambda *args: None)
    monkeypatch.setattr(run_viewer, "_finalize_video", finalize)
    monkeypatch.setattr(run_viewer, "_wait_for_shutdown", lambda *args: None)
    monkeypatch.setattr(run_viewer, "_cleanup", lambda *args: cleaned.append(True))
    if failure == "summary":
        monkeypatch.setattr(run_viewer.viewer_state.ViewerRunStore, "write_summary", fail_summary)
        with pytest.raises(OSError, match="summary unavailable"):
            run_viewer._run_viewer("match")
    else:
        assert run_viewer._run_viewer("match") == (1 if failure else 0)
        summary = json.loads(next(tmp_path.glob("*/viewer-summary.json")).read_text())
        assert summary["event_count"] == 8
        assert summary["status"] == ("failed" if failure else "completed")
        if failure:
            assert "disk full" in summary["reason"]
    assert cleaned == [True]
    assert all(log.closed for log in logs)


def test_replay_conversion_timeout_is_bounded(tmp_path, monkeypatch):
    (tmp_path / "scene").mkdir()
    playlist = tmp_path / "scene" / "index.m3u8"
    playlist.write_text("#EXTM3U\n")
    store = run_viewer.viewer_state.ViewerRunStore("match", "run", playlist, tmp_path, tmp_path)
    monkeypatch.setattr(run_viewer.run_smoke, "_terminate", lambda *args: None)
    def timeout(command, *, check, timeout):
        assert timeout == 30.0
        raise subprocess.TimeoutExpired(command, timeout)
    monkeypatch.setattr(run_viewer.subprocess, "run", timeout)
    with pytest.raises(run_viewer.ViewerError, match="replay conversion exceeded"):
        run_viewer._finalize_video(object(), tmp_path, tmp_path / "replay.mp4", store)
    assert not store.snapshot()["replay_available"]


def test_cleanup_attempts_remaining_children_after_termination_error(monkeypatch):
    processes = run_viewer.ViewerProcesses()
    for name in ("tree", "ffmpeg", "controller", "webots", "xvfb"):
        setattr(processes, name, name)
    stopped = []
    def terminate(process, timeout):
        stopped.append(process)
        if process == "tree":
            raise OSError("cannot stop tree")
    class Server:
        def stop(self):
            stopped.append("http")
    monkeypatch.setattr(run_viewer.run_smoke, "_terminate", terminate)
    with pytest.raises(OSError, match="cannot stop tree"):
        run_viewer._cleanup(processes, Server())
    assert stopped == ["tree", "ffmpeg", "controller", "webots", "xvfb", "http"]


def test_bootstrap_viewer_builds_before_delegate(monkeypatch):
    docker_dir = Path("robots/lynsense/simulation/docker").resolve()
    monkeypatch.syspath_prepend(str(docker_dir))
    import bootstrap_viewer as module

    order: list[str] = []
    monkeypatch.setattr(module.bootstrap, "_build", lambda: order.append("build"))
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda command, check: order.append("delegate") or subprocess.CompletedProcess(command, 0),
    )
    monkeypatch.setattr(sys, "argv", ["bootstrap_viewer", "--phase", "match"])
    assert module.main() == 0
    assert module._build_delegate_command("match").endswith("exec lynsense_run_viewer --phase match")
    assert order == ["build", "delegate"]
