from __future__ import annotations

import argparse
import configparser
import datetime as dt
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from typing import Any
import uuid

from lynsense_webots_sim import viewer_server, viewer_state
from lynsense_webots_sim.scripts import run_smoke


VIEWER_PHASES = ("smoke", "blocked", "match", "box")
CAMERA_VIEWS = ("scene", "robot")
DISPLAY = ":99"
CAPTURE_SIZE = "1280x1024"
ARTIFACT_ROOT = Path("/workspace/artifacts/viewer")
EVENT_ROOT = Path("/workspace/artifacts")
DEFAULT_VIEWER_HOST = "0.0.0.0"
DEFAULT_VIEWER_PORT = 7047
FILE_LOG_NAMES = ("webots", "controller", "tree", "ffmpeg")


class ViewerError(RuntimeError):
    pass


class ViewerProcesses:
    def __init__(self) -> None:
        self.xvfb: subprocess.Popen[Any] | None = None
        self.webots: subprocess.Popen[Any] | None = None
        self.controller: subprocess.Popen[Any] | None = None
        self.ffmpeg: subprocess.Popen[Any] | None = None
        self.tree: subprocess.Popen[Any] | None = None


def _viewer_endpoint() -> tuple[str, int]:
    host = os.environ.get("LYNSENSE_VIEWER_HOST", DEFAULT_VIEWER_HOST).strip()
    if not host:
        raise ViewerError("LYNSENSE_VIEWER_HOST must not be empty")
    raw_port = os.environ.get("LYNSENSE_VIEWER_PORT", str(DEFAULT_VIEWER_PORT))
    try:
        port = int(raw_port)
    except ValueError as exc:
        raise ViewerError("LYNSENSE_VIEWER_PORT must be an integer") from exc
    if not 1 <= port <= 65535:
        raise ViewerError("LYNSENSE_VIEWER_PORT must be between 1 and 65535")
    return host, port


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the Lynsense LAN viewer")
    parser.add_argument("--phase", required=True, choices=set(VIEWER_PHASES))
    return parser


def _webots_command(phase: str) -> list[str]:
    if phase not in VIEWER_PHASES:
        raise ValueError(f"unsupported viewer phase: {phase}")
    return [
        "webots",
        "--batch",
        "--mode=realtime",
        "--stdout",
        "--stderr",
        os.fspath(run_smoke.WORLD_PATHS[phase]),
    ]


def _xvfb_command(display: str) -> list[str]:
    return [
        "Xvfb",
        display,
        "-screen",
        "0",
        "1280x1024x24",
        "-nolisten",
        "tcp",
    ]


def _ffmpeg_command(frame_dir: Path, video_dir: Path) -> list[str]:
    command = ["ffmpeg", "-hide_banner", "-loglevel", "warning"]
    for view in CAMERA_VIEWS:
        command.extend([
            "-re", "-loop", "1", "-framerate", "10", "-i",
            os.fspath(frame_dir / f"{view}.jpg"),
        ])
    for index, view in enumerate(CAMERA_VIEWS):
        command.extend([
            "-map", f"{index}:v", "-an", "-vf",
            "scale=960:540:force_original_aspect_ratio=decrease:force_divisible_by=2,"
            "pad=960:540:(ow-iw)/2:(oh-ih)/2",
            "-pix_fmt", "yuv420p", "-c:v", "libx264", "-threads", "2",
            "-preset", "veryfast", "-tune", "zerolatency", "-profile:v", "baseline",
            "-g", "10", "-keyint_min", "10", "-sc_threshold", "0",
            "-f", "hls", "-hls_time", "1", "-hls_list_size", "0",
            "-hls_flags", "append_list+omit_endlist+independent_segments",
            "-hls_segment_filename", os.fspath(video_dir / view / "segment-%06d.ts"),
            os.fspath(video_dir / view / "index.m3u8"),
        ])
    return command


def _replay_command(playlist: Path, replay: Path) -> list[str]:
    return [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-i",
        os.fspath(playlist),
        "-c",
        "copy",
        "-movflags",
        "+faststart",
        os.fspath(replay),
    ]


def _load_map_metadata(phase: str) -> tuple[dict[str, float], dict[str, dict[str, float]]]:
    import yaml

    config_name = "box_navigation" if phase == "box" else phase
    config = yaml.safe_load(
        run_smoke.CONFIG_PATHS[config_name].read_text(encoding="utf-8")
    )
    world = config["world"]["arena"]
    return dict(world), {name: dict(goal) for name, goal in config["goals"].items()}


def _configure_rendering(run_dir: Path, environment: dict[str, str]) -> None:
    config_root = run_dir / "webots-config"
    directory = config_root / "Cyberbotics"
    directory.mkdir(parents=True, exist_ok=True)
    preferences = configparser.ConfigParser(interpolation=None)
    preferences.optionxform = str
    preferences["View3d"] = {
        "hideAllCameraOverlays": "true",
        "hideAllDisplayOverlays": "true",
        "hideAllRangeFinderOverlays": "true",
    }
    preferences["%General"] = {"checkWebotsUpdateOnStartup": "false"}
    with (directory / "Webots-R2025a.conf").open("w", encoding="utf-8") as stream:
        preferences.write(stream, space_around_delimiters=False)
    environment["XDG_CONFIG_HOME"] = str(config_root)


def _wait_for_hls_playlist(
    process: subprocess.Popen[Any], playlist: Path, timeout: float = 15.0
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise ViewerError(f"ffmpeg exited before HLS playlist: {process.returncode}")
        if playlist.is_file() and playlist.stat().st_size > 0:
            return
        time.sleep(0.1)
    raise ViewerError(f"HLS playlist did not arrive within {timeout:g} seconds")


def _finalize_video(
    ffmpeg: subprocess.Popen[Any], video_dir: Path, replay: Path, store: viewer_state.ViewerRunStore
) -> None:
    run_smoke._terminate(ffmpeg, 5.0)
    available = []
    for view in CAMERA_VIEWS:
        playlist = video_dir / view / "index.m3u8"
        target = replay if view == "scene" else replay.with_name("replay-robot.mp4")
        if not playlist.is_file():
            available.append(False)
            continue
        content = playlist.read_text(encoding="utf-8")
        if "#EXT-X-ENDLIST" not in content:
            temporary = playlist.with_name(".final-index.m3u8")
            temporary.write_text(content.rstrip() + "\n#EXT-X-ENDLIST\n", encoding="utf-8")
            os.replace(temporary, playlist)
        try:
            result = subprocess.run(
                _replay_command(playlist, target), check=False, timeout=30.0
            )
        except subprocess.TimeoutExpired as exc:
            store.set_replay_available(False)
            raise ViewerError(f"{view} replay conversion exceeded 30 seconds") from exc
        except OSError:
            available.append(False)
        else:
            available.append(result.returncode == 0 and target.is_file())
    store.set_replay_available(all(available))


def _wait_for_camera_frames(
    processes: ViewerProcesses, store: viewer_state.ViewerRunStore, frame_dir: Path,
    timeout: float = 15.0,
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        _require_children_alive(processes, store)
        if all((frame_dir / f"{view}.jpg").is_file() for view in CAMERA_VIEWS):
            return
        time.sleep(0.1)
    raise ViewerError("scene and robot camera frames did not arrive")


def _record_process_statuses(
    processes: ViewerProcesses, store: viewer_state.ViewerRunStore
) -> None:
    for name in ("xvfb", "webots", "controller", "ffmpeg", "tree"):
        process = getattr(processes, name)
        if process is not None and process.poll() is not None:
            store.record_process_status(name, process.returncode)


def _unexpected_exit(
    processes: ViewerProcesses,
    store: viewer_state.ViewerRunStore,
    expected_exits: set[str] | None = None,
) -> str | None:
    _record_process_statuses(processes, store)
    expected_exits = expected_exits or set()
    for name in ("xvfb", "webots", "controller", "ffmpeg", "tree"):
        process = getattr(processes, name)
        if name not in expected_exits and process is not None and process.poll() is not None:
            return f"{name} exited with code {process.returncode}"
    return None


def _require_children_alive(
    processes: ViewerProcesses, store: viewer_state.ViewerRunStore
) -> None:
    unexpected = _unexpected_exit(processes, store)
    if unexpected is not None:
        raise ViewerError(unexpected)


def _require_server_alive(
    server: viewer_server.ViewerHTTPServer,
    store: viewer_state.ViewerRunStore,
    shutdown: threading.Event,
) -> None:
    if server.is_alive():
        return
    reason = "HTTP server thread exited unexpectedly"
    store.record_process_status("http", None, reason)
    store.set_status("failed", reason)
    shutdown.set()
    raise ViewerError(reason)


def _cleanup(processes: ViewerProcesses, server: viewer_server.ViewerHTTPServer) -> None:
    first_error: Exception | None = None
    for process in (
        processes.tree,
        processes.ffmpeg,
        processes.controller,
        processes.webots,
        processes.xvfb,
    ):
        if process is not None:
            try:
                run_smoke._terminate(process, 5.0)
            except Exception as exc:
                if first_error is None:
                    first_error = exc
    try:
        server.stop()
    finally:
        if first_error is not None:
            raise first_error


def _wait_for_shutdown(
    shutdown: threading.Event,
    processes: ViewerProcesses | None = None,
    server: viewer_server.ViewerHTTPServer | None = None,
    store: viewer_state.ViewerRunStore | None = None,
) -> str | None:
    while not shutdown.is_set():
        if processes is not None and store is not None:
            unexpected = _unexpected_exit(processes, store, expected_exits={"ffmpeg", "tree"})
            if unexpected is not None:
                store.set_status("failed", unexpected)
                shutdown.set()
                return unexpected
        if server is not None and store is not None and not server.is_alive():
            reason = "HTTP server thread exited unexpectedly"
            store.record_process_status("http", None, reason)
            store.set_status("failed", reason)
            shutdown.set()
            return reason
        time.sleep(0.1)
    return None


def _run_viewer(phase: str) -> int:
    run_id = f"{dt.datetime.now(dt.timezone.utc):%Y%m%dT%H%M%SZ}-{uuid.uuid4()}"
    run_dir = ARTIFACT_ROOT / run_id
    video_dir = run_dir / "video"
    for view in CAMERA_VIEWS:
        (video_dir / view).mkdir(parents=True, exist_ok=True)
    frame_dir = run_dir / "frames"
    frame_dir.mkdir()
    event_path = run_dir / "events.jsonl"
    event_path.touch(mode=0o600)
    replay = run_dir / "replay.mp4"
    environment = run_smoke._command_environment(phase, event_path, run_id)
    environment.update({"DISPLAY": DISPLAY, "ROS_DOMAIN_ID": "42", "ROS_LOCALHOST_ONLY": "1"})
    environment["LYNSENSE_VIEWER_CAPTURE_DIR"] = str(frame_dir)
    _configure_rendering(run_dir, environment)
    store = viewer_state.ViewerRunStore(phase, run_id, event_path, run_dir, video_dir)
    arena, goals = _load_map_metadata(phase)
    store.set_map_metadata(arena, goals)
    store.set_time_range(dt.datetime.now(dt.timezone.utc).isoformat(), None)
    host, port = _viewer_endpoint()
    logs = {name: (run_dir / f"{name}.log").open("ab") for name in FILE_LOG_NAMES}
    server = viewer_server.ViewerHTTPServer(
        host, port, store, Path("/workspace/simulation/viewer"), video_dir, replay
    )
    processes = ViewerProcesses()
    shutdown = threading.Event()
    old_handlers: dict[int, Any] = {}
    result = 1
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, lambda _signum, _frame: shutdown.set())
        server.start()
        _require_server_alive(server, store, shutdown)
        processes.xvfb = subprocess.Popen(
            _xvfb_command(DISPLAY), env=environment, stdout=logs["webots"],
            stderr=logs["webots"], start_new_session=True
        )
        _require_children_alive(processes, store)
        processes.webots = subprocess.Popen(
            _webots_command(phase),
            env=run_smoke._software_rendering_environment(environment),
            stdout=logs["webots"],
            stderr=logs["webots"], start_new_session=True
        )
        run_smoke._wait_for_webots_ready(processes.webots, run_dir / "webots.log")
        _require_children_alive(processes, store)
        processes.controller = run_smoke._start_controller(
            environment, phase, run_id, log_path=run_dir / "controller.log"
        )
        phase_processes = run_smoke.PhaseProcesses(processes.webots, processes.controller)
        run_smoke._wait_for_ready(phase_processes, event_path, run_id, phase=phase)
        _require_children_alive(processes, store)
        _wait_for_camera_frames(processes, store, frame_dir)
        store.set_video_start_mono(time.monotonic())
        processes.ffmpeg = subprocess.Popen(
            _ffmpeg_command(frame_dir, video_dir), env=environment,
            stdout=logs["ffmpeg"], stderr=logs["ffmpeg"], start_new_session=True
        )
        for view in CAMERA_VIEWS:
            _wait_for_hls_playlist(processes.ffmpeg, video_dir / view / "index.m3u8")
        store.set_video_available(True)
        run_smoke._wait_for_action_graph(phase_processes, environment)
        _require_children_alive(processes, store)
        processes.tree = run_smoke._start_tree(
            environment, phase, run_id, log_path=run_dir / "tree.log"
        )
        store.set_status("running")
        validator = run_smoke.TreeEventValidator(
            blocked=phase == "blocked",
            expected_script=run_smoke.EXPECTED_SCRIPTS[phase],
            final_pose=run_smoke.FINAL_POSES[phase],
            phase=phase,
        )
        deadline = time.monotonic() + (
            180.0 if phase == "box" else (30.0 if phase == "blocked" else 120.0)
        )
        while time.monotonic() < deadline and not validator.terminal:
            if shutdown.is_set():
                raise ViewerError("viewer interrupted before terminal action result")
            _require_server_alive(server, store, shutdown)
            unexpected = _unexpected_exit(processes, store)
            if unexpected is not None:
                raise ViewerError(unexpected)
            for record in store.ingest_new_events(time.monotonic()):
                event = record["event"]
                if event.get("kind") in {"run_started", "controller_ready"}:
                    continue
                if validator.feed(event):
                    run_smoke._signal_process_group(processes.tree, signal.SIGINT)
            if processes.tree.poll() is not None and not validator.terminal:
                raise ViewerError("tree exited before terminal event")
            time.sleep(0.1)
        if not validator.terminal:
            raise ViewerError("viewer action deadline expired")
        _require_server_alive(server, store, shutdown)
        store.set_status("completed")
        result = 0
    except BaseException as exc:
        store.set_status("failed", f"{type(exc).__name__}: {exc}")
    finally:
        try:
            try:
                (frame_dir / "stopped").touch()
                if processes.ffmpeg is not None:
                    _finalize_video(processes.ffmpeg, video_dir, replay, store)
            except Exception as exc:
                result = 1
                store.set_replay_available(False)
                store.set_status("failed", f"video finalization failed: {exc}")
            _record_process_statuses(processes, store)
            store.set_time_range(store.snapshot()["started_at"] or "", dt.datetime.now(dt.timezone.utc).isoformat())
            store.write_summary(dt.datetime.now(dt.timezone.utc).isoformat())
            if result == 0 and not shutdown.is_set():
                if _wait_for_shutdown(shutdown, processes, server, store) is not None:
                    result = 1
        finally:
            try:
                _cleanup(processes, server)
            finally:
                try:
                    _record_process_statuses(processes, store)
                    store.write_summary(dt.datetime.now(dt.timezone.utc).isoformat())
                finally:
                    for signum, handler in old_handlers.items():
                        signal.signal(signum, handler)
                    for log in logs.values():
                        log.close()
    return result


def main() -> int:
    args = _build_parser().parse_args()
    return _run_viewer(args.phase)


if __name__ == "__main__":
    raise SystemExit(main())
