# Lynsense Webots Navigation Viewer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a simulation-only, read-only LAN viewer for the existing `smoke`, `blocked`, and `match` Lynsense Webots navigation gates, with realtime HLS video, trajectory/action timelines, and replayable MP4 evidence.

**Architecture:** Keep the existing no-network gate untouched. Add a separate Compose service that builds the same ephemeral ROS overlay, renders Webots in realtime on a fixed Xvfb display, captures that display with ffmpeg, serves a Python HTTP viewer on port `7047`, tails controller JSONL events, and finalizes HLS plus MP4 evidence under the existing artifact root.

**Tech Stack:** Python 3.10 standard-library HTTP server, pytest, Webots R2025a realtime rendering, Xvfb, ffmpeg x11grab/HLS/H.264, Docker Compose, vanilla HTML/CSS/JavaScript, and local `hls.js@1.5.13` as the non-native HLS fallback.

**Spec:** [Lynsense Webots Navigation Viewer Design](../../superpowers/specs/2026-09-21-lynsense-webots-viewer-design.md)

**Delivery checkpoint (2026-09-21):** Tasks 1-6 are implemented. The final
[acceptance record](2026-09-21-lynsense-viewer-acceptance.md) contains verified
results and remaining validation limits; the steps below preserve the original
implementation recipe. Commit steps were intentionally not executed.

## Global Constraints

- Only run simulation on this workstation.
- Do not connect robot one, use `ROS_DOMAIN_ID=3`, or source robot workspaces.
- Do not publish ROS DDS ports; publish only viewer HTTP TCP `7047`.
- Keep `robots/lynsense/simulation/docker/compose.yaml` on `network_mode: none` with no ports.
- Viewer frontend resources must be local; do not load CDN assets at runtime.
- Do not mount the repository root, `.env.lynsense`, SSH configuration, robot paths, or company URDF assets.
- Keep `lynsense_pytrees` and the simulation subtree read-only.
- Do not add start, stop, retry, or parameter-control endpoints.
- Viewer execution is read-only observation; passing video does not authorize real-robot motion.
- Preserve the currently passing `interface`, `smoke`, `blocked`, and `match` gate behavior.
- Manual source edits use `apply_patch`.
- Do not auto-commit, push, or create a PR.

## Baseline

- Branch: `feat/lynsense-readonly-state`, tracking `fork/feat/lynsense-readonly-state`.
- The worktree already contains the uncommitted navigation simulation work. Preserve it; this plan adds viewer files and narrowly updates shared simulation documentation and tests.
- Latest local baseline: `143` simulation tests passed, `253` Lynsense tests passed, and container `--phase all` passed all four gates.
- Webots R2025a accepts the exact realtime CLI value `--mode=realtime`.
- The current image already contains ffmpeg 4.4.2, but the Dockerfile will name `ffmpeg` explicitly so the viewer does not depend on a transitive base-image package.
- The outer project's `writing-plans` skill is located at `/home/huhu/work/RPent_lynsense/.agents/skills/writing-plans/SKILL.md`.

## File Structure

| File | Responsibility |
| --- | --- |
| `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/viewer_state.py` | Thread-safe event ingestion, pagination, snapshot, and summary state |
| `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/viewer_server.py` | Restricted read-only HTTP API and static media routes |
| `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/run_viewer.py` | Viewer process orchestration, event observation, and cleanup |
| `robots/lynsense/simulation/docker/bootstrap_viewer.py` | Build the ephemeral overlay and delegate to the installed viewer command |
| `robots/lynsense/simulation/docker/compose.viewer.yaml` | LAN viewer service, port `7047`, and approved mounts |
| `robots/lynsense/simulation/docker/Dockerfile` | Existing ROS/Webots image plus an explicit ffmpeg dependency |
| `robots/lynsense/simulation/viewer/index.html` | Viewer page structure |
| `robots/lynsense/simulation/viewer/static/app.css` | Responsive viewer layout |
| `robots/lynsense/simulation/viewer/static/app.js` | Polling, map, timeline, HLS, and replay behavior |
| `robots/lynsense/simulation/viewer/static/vendor/hls.min.js` | Local minimized `hls.js@1.5.13` fallback |
| `robots/lynsense/simulation/viewer/static/vendor/hls.min.js.sha256` | SHA256 provenance for the vendored file |
| `robots/lynsense/simulation/viewer/static/vendor/README.md` | Version, source URL, checksum, and Apache-2.0 provenance |
| `robots/lynsense/simulation/ros/lynsense_webots_sim/setup.py` | Installs the `lynsense_run_viewer` entry point |
| `robots/lynsense/simulation/README.md` | Viewer launch, LAN URL, evidence, limits, and shutdown |
| `tests/unit_tests/robots/lynsense/simulation/test_viewer_state.py` | Event store and summary contracts |
| `tests/unit_tests/robots/lynsense/simulation/test_viewer_server.py` | HTTP route, method, media-type, traversal, and pagination contracts |
| `tests/unit_tests/robots/lynsense/simulation/test_viewer_runtime.py` | Process ordering, lifecycle, video finalization, and cleanup contracts |
| `tests/unit_tests/robots/lynsense/simulation/test_viewer_assets.py` | Compose, Docker, bootstrap, frontend, and vendored asset contracts |

---

### Task 1: Viewer Event Store

**Files:**

- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/viewer_state.py`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_viewer_state.py`

**Interfaces:**

- Consumes controller JSONL event objects from `events.jsonl`.
- Produces `ViewerRunStore(phase: str, run_id: str, event_path: Path, artifact_dir: Path, video_dir: Path, video_start_mono: float | None = None)`.
- Produces `ingest_new_events(now_mono: float) -> list[dict[str, Any]]`.
- Produces `events_after(index: int, limit: int = 512) -> list[dict[str, Any]]`.
- Produces `snapshot() -> dict[str, Any]`.
- Produces `set_status(status: str, reason: str = "") -> None`.
- Produces `set_video_available(available: bool) -> None`.
- Produces `set_replay_available(available: bool) -> None`.
- Produces `set_video_start_mono(now_mono: float) -> None`.
- Produces `record_process_status(name: str, exit_code: int | None, reason: str = "") -> None`.
- Produces `set_map_metadata(arena: dict[str, float], goals: dict[str, dict[str, float]]) -> None`.
- Produces `set_time_range(started_at: str, ended_at: str | None) -> None`.
- Produces `write_summary(generated_at: str) -> None`.

- [ ] **Step 1: Write failing state tests**

Create `tests/unit_tests/robots/lynsense/simulation/test_viewer_state.py`:

```python
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
    second = store.events_after(first[-1]["index"])
    assert [event["event"]["kind"] for event in first] == ["run_started"]
    assert [event["event"]["kind"] for event in second] == ["controller_ready"]


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
```

- [ ] **Step 2: Run state tests and verify RED**

Run:

```bash
.venv/bin/python -m pytest tests/unit_tests/robots/lynsense/simulation/test_viewer_state.py -q
```

Expected: `ModuleNotFoundError: No module named 'lynsense_webots_sim.viewer_state'`.

- [ ] **Step 3: Implement the state store**

Implement the constructor with a `threading.RLock`, byte offset, event list, status, current/terminal event, telemetry, and video/replay flags. Use these exact behavior rules:

- Read only newly appended bytes and require a trailing newline.
- Parse each complete line as a JSON object.
- Advance the byte offset only after every line in that read has been handled.
- Wrap accepted records as `{index, event, video_offset_s}`.
- On malformed complete JSON, set `status=failed`, record a reason containing `invalid JSONL event`, retain accepted events, and return `[]`.
- Update pose and motor rates whenever present.
- Set terminal evidence only for `kind == "action_result"`.
- Record process statuses separately from viewer completion status.
- Expose map arena/goal metadata and start/end wall-clock range.
- Return deep-copied snapshots.
- Write `viewer-summary.json` with sorted keys and `allow_nan=False`; include
  phase, run ID, generated time, status/reason, event count, terminal evidence,
  final pose/rates, video/replay availability, process statuses, artifact paths,
  and `real_robot_connected: false`.

- [ ] **Step 4: Run state tests and verify GREEN**

Run the Step 2 command. Expected: all tests pass.

- [ ] **Step 5: Review the diff**

Confirm no controller event semantics changed. Do not commit unless explicitly authorized.

---

### Task 2: Restricted Viewer HTTP Server

**Files:**

- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/viewer_server.py`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_viewer_server.py`

**Interfaces:**

- Consumes `ViewerRunStore` from Task 1.
- Produces `ViewerHTTPServer(host: str, port: int, store: ViewerRunStore, static_root: Path, video_dir: Path, replay_path: Path)`.
- Produces `start() -> str` and `stop() -> None`.
- Produces exactly these GET routes:
  `/`, `/healthz`, `/api/run`, `/api/events`, `/static/app.css`,
  `/static/app.js`, `/static/vendor/hls.min.js`, `/video/index.m3u8`,
  `/video/segment-*.ts`, and `/replay.mp4`.

- [ ] **Step 1: Write failing server tests**

Create a temporary static/video tree and start the server on `127.0.0.1:0`. Test through `urllib.request`:

```python
@pytest.fixture
def viewer(tmp_path):
    artifact = tmp_path / "run"
    video = artifact / "video"
    static = tmp_path / "static"
    vendor = static / "vendor"
    video.mkdir(parents=True)
    vendor.mkdir(parents=True)
    (static / "app.css").write_text("body{}", encoding="utf-8")
    (static / "app.js").write_text("export {}", encoding="utf-8")
    (vendor / "hls.min.js").write_text("/*! hls.js v1.5.13 */", encoding="utf-8")
    (video / "index.m3u8").write_text("#EXTM3U\n", encoding="utf-8")
    (video / "segment-000001.ts").write_bytes(b"segment")
    replay = artifact / "replay.mp4"
    replay.write_bytes(b"mp4")
    event_path = artifact / "events.jsonl"
    event_path.touch()
    store = ViewerRunStore(
        phase="match", run_id="server-run", event_path=event_path,
        artifact_dir=artifact, video_dir=video, video_start_mono=0.0,
    )
    server = ViewerHTTPServer(
        host="127.0.0.1", port=0, store=store, static_root=static,
        video_dir=video, replay_path=replay,
    )
    base = server.start()
    yield base, store
    server.stop()
```

Assert:

- `/`, `/healthz`, `/api/run`, and `/api/events?after=-1` return 200.
- `/healthz` body is exactly `{"ok": true}`.
- Event pagination is exclusive: after two events, `after=-1` returns indexes 0 and 1, while `after=0` returns only 1.
- `/api/events?after=bad` returns 400.
- CSS, JavaScript, M3U8, TS, and MP4 return media types `text/css; charset=utf-8`, `text/javascript; charset=utf-8`, `application/vnd.apple.mpegurl`, `video/mp2t`, and `video/mp4`.
- API and playlist responses have `Cache-Control: no-store`.
- `POST /api/run`, `PUT /`, `DELETE /healthz`, and `PATCH /api/events` return 405 with `Allow: GET`.
- `/../README.md`, `/static/../viewer_state.py`, `/video/../events.jsonl`, `/video/segment-bad.ts`, and `/control` return 404.
- Replay returns 404 until `set_replay_available(True)`.

- [ ] **Step 2: Run server tests and verify RED**

Run:

```bash
.venv/bin/python -m pytest tests/unit_tests/robots/lynsense/simulation/test_viewer_server.py -q
```

Expected: `ModuleNotFoundError: No module named 'lynsense_webots_sim.viewer_server'`.

- [ ] **Step 3: Implement the HTTP server**

Use `ThreadingHTTPServer` with an explicit handler dispatch table. Enforce:

```python
_STATIC_ROUTES = {
    "/static/app.css": (Path("app.css"), "text/css; charset=utf-8"),
    "/static/app.js": (Path("app.js"), "text/javascript; charset=utf-8"),
    "/static/vendor/hls.min.js": (
        Path("vendor") / "hls.min.js", "text/javascript; charset=utf-8"
    ),
}
_SEGMENT_PATTERN = re.compile(r"^/video/segment-[0-9]{6,}\.ts$")
```

Resolve allowlisted files, require containment, reject unknown routes, return 405 before reading bodies, and serve replay only when available. `stop()` must shut down and join the daemon thread.

- [ ] **Step 4: Run Tasks 1-2 tests**

Run:

```bash
.venv/bin/python -m pytest \
  tests/unit_tests/robots/lynsense/simulation/test_viewer_state.py \
  tests/unit_tests/robots/lynsense/simulation/test_viewer_server.py -q
```

Expected: all tests pass.

---

### Task 3: Viewer Process Orchestration

**Files:**

- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/run_viewer.py`
- Create: `robots/lynsense/simulation/docker/bootstrap_viewer.py`
- Modify: `robots/lynsense/simulation/ros/lynsense_webots_sim/setup.py`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_viewer_runtime.py`

**Interfaces:**

- Consumes `ViewerRunStore` and `ViewerHTTPServer`.
- Consumes existing `run_smoke` paths, phase scripts, action discovery, event validator, and cleanup helpers.
- Produces console command `lynsense_run_viewer --phase smoke|blocked|match`.
- Produces command builders `_webots_command(phase)`, `_xvfb_command(display)`, `_ffmpeg_command(display, video_dir)`, and `_replay_command(playlist, replay)`.

- [ ] **Step 1: Write failing runtime tests**

Use AST/source assertions and monkeypatched fake processes. Do not start real Webots in unit tests. Cover:

```python
def test_cli_accepts_only_navigation_viewer_phases():
    parser = run_viewer._build_parser()
    choices = [action.choices for action in parser._actions if action.dest == "phase"]
    assert choices == [{"smoke", "blocked", "match"}]


def test_realtime_webots_command_has_rendering_enabled():
    command = run_viewer._webots_command("match")
    assert "--mode=realtime" in command
    assert "--no-rendering" not in command
    assert command[-1].endswith(".wbt")


def test_xvfb_is_fixed_local_and_sized_for_capture():
    assert run_viewer._xvfb_command(":99") == [
        "Xvfb", ":99", "-screen", "0", "1280x1024x24", "-nolisten", "tcp"
    ]
```

Also assert:

- ffmpeg command contains `x11grab`, `20`, `1280x1024`, `libx264`, `hls_time`, `1`, exact playlist, and exact segment pattern.
- observed startup order is HTTP, Xvfb, Webots, controller readiness, ffmpeg, playlist readiness, action graph, tree.
- startup loads arena and goals from the selected phase YAML and passes them to
  `ViewerRunStore.set_map_metadata`.
- start/end wall-clock range is recorded before summary serialization.
- `_wait_for_hls_playlist` fails if ffmpeg dies or playlist does not arrive.
- tree uses existing phase XML and validator.
- expected `blocked` terminal failure completes the viewer; unexpected failure marks it failed.
- finalization interrupts ffmpeg, appends `#EXT-X-ENDLIST` if absent, converts to MP4, and marks replay available.
- MP4 failure retains HLS and marks replay unavailable.
- cleanup order is tree, ffmpeg, controller, Webots, Xvfb, HTTP.
- bootstrap calls `bootstrap._build()` before installed viewer delegation.
- setup declares the `lynsense_run_viewer` entry point.

- [ ] **Step 2: Run runtime tests and verify RED**

Run:

```bash
.venv/bin/python -m pytest tests/unit_tests/robots/lynsense/simulation/test_viewer_runtime.py -q
```

Expected: `run_viewer` cannot be imported.

- [ ] **Step 3: Implement exact command builders**

Webots:

```python
["webots", "--batch", "--mode=realtime", "--stdout", "--stderr", str(world)]
```

Xvfb:

```python
["Xvfb", display, "-screen", "0", "1280x1024x24", "-nolisten", "tcp"]
```

ffmpeg capture flags:

```text
-f x11grab -framerate 20 -video_size 1280x1024 -i <display>.0
-pix_fmt yuv420p -c:v libx264 -preset veryfast -tune zerolatency
-profile:v baseline -g 20 -keyint_min 20 -sc_threshold 0
-f hls -hls_time 1 -hls_list_size 0
-hls_flags append_list+omit_endlist+independent_segments
-hls_segment_filename <video-dir>/segment-%06d.ts <video-dir>/index.m3u8
```

Replay conversion:

```bash
ffmpeg -hide_banner -loglevel error -y -i <playlist> -c copy -movflags +faststart <replay>
```

- [ ] **Step 4: Implement lifecycle**

Use this order:

1. validate phase and create run directory;
2. create mode-`0o600` `events.jsonl`;
3. construct store and start HTTP;
4. start Xvfb;
5. start rendered Webots and wait for ready marker;
6. start controller and wait for `controller_ready`;
7. start ffmpeg and wait for playlist;
8. wait for both action endpoints;
9. start tree;
10. feed only new events to `TreeEventValidator`;
11. on terminal result, interrupt tree and finalize video;
12. write summary and serve replay until Ctrl-C.

Use process groups, bounded SIGINT/SIGTERM/SIGKILL cleanup, and signal-safe shutdown events.

- [ ] **Step 5: Add bootstrap and entry point**

Create `bootstrap_viewer.py` so it parses the three phases, calls the existing `_build()`, then delegates through:

```bash
source /opt/ros/humble/setup.bash &&
source /workspace/ws/install/setup.bash &&
exec lynsense_run_viewer --phase <phase>
```

Add `lynsense_run_viewer = lynsense_webots_sim.scripts.run_viewer:main` to `setup.py`.

- [ ] **Step 6: Run runtime tests**

Run:

```bash
.venv/bin/python -m pytest \
  tests/unit_tests/robots/lynsense/simulation/test_viewer_runtime.py -q
```

Expected: all tests pass.

---

### Task 4: LAN Compose And Image Contract

**Files:**

- Modify: `robots/lynsense/simulation/docker/Dockerfile`
- Create: `robots/lynsense/simulation/docker/compose.viewer.yaml`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_viewer_assets.py`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_static_assets.py`

**Interfaces:**

- Consumes `bootstrap_viewer.py`.
- Produces service `lynsense-webots-viewer` on LAN port `7047`.

- [ ] **Step 1: Write failing Compose/Docker tests**

Parse YAML and assert:

```python
def test_original_gate_remains_isolated():
    gate = compose_services("compose.yaml")["lynsense-webots-smoke"]
    assert gate["network_mode"] == "none"
    assert "ports" not in gate


def test_viewer_publishes_only_read_only_http_port():
    viewer = compose_services("compose.viewer.yaml")["lynsense-webots-viewer"]
    assert viewer["network_mode"] == "bridge"
    assert viewer["ports"] == ["7047:7047"]
    assert viewer["init"] is True
```

Also assert:

- viewer build context, Dockerfile, and image match the gate;
- environment has Domain 42, localhost-only mode, host `0.0.0.0`, port `7047`;
- command phase is `${LYNSENSE_VIEWER_PHASE:-match}`;
- entrypoint is `bootstrap_viewer.py`;
- mounts exactly match the approved three gate mounts;
- no host X11 socket or fourth mount exists;
- Dockerfile names `ffmpeg`;
- digest-pinned base image remains unchanged.

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
.venv/bin/python -m pytest \
  tests/unit_tests/robots/lynsense/simulation/test_viewer_assets.py -q
```

Expected: viewer service or explicit ffmpeg dependency is missing.

- [ ] **Step 3: Implement Compose/Docker contract**

Create a separate `lynsense-webots-viewer` service with bridge networking, `init: true`, published `7047:7047`, the same three mounts as the gate, and the viewer entrypoint/command. Add `ffmpeg` to the Dockerfile package list. Do not alter the gate service.

Use these exact mounts and no others:

```yaml
volumes:
  - type: bind
    source: ..
    target: /workspace/simulation
    read_only: true
  - type: bind
    source: ../../../../../lynsense_pytrees
    target: /workspace/ws/src/lynsense_pytrees
    read_only: true
  - type: bind
    source: ../../../../.artifacts/lynsense-webots
    target: /workspace/artifacts
    read_only: false
```

- [ ] **Step 4: Run asset/static regressions**

Run:

```bash
.venv/bin/python -m pytest \
  tests/unit_tests/robots/lynsense/simulation/test_viewer_assets.py \
  tests/unit_tests/robots/lynsense/simulation/test_static_assets.py -q
```

Expected: all tests pass.

---

### Task 5: Viewer Frontend And Local HLS Fallback

**Files:**

- Create: `robots/lynsense/simulation/viewer/index.html`
- Create: `robots/lynsense/simulation/viewer/static/app.css`
- Create: `robots/lynsense/simulation/viewer/static/app.js`
- Create: `robots/lynsense/simulation/viewer/static/vendor/hls.min.js`
- Create: `robots/lynsense/simulation/viewer/static/vendor/hls.min.js.sha256`
- Create: `robots/lynsense/simulation/viewer/static/vendor/README.md`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_viewer_assets.py`

**Interfaces:**

- Consumes `/api/run`, `/api/events`, `/video/index.m3u8`, and `/replay.mp4`.
- Produces page IDs `video`, `run-state`, `pose`, `wheel-rates`, `map`, `action-timeline`, `terminal-summary`, `replay-link`, and `video-error`.

- [ ] **Step 1: Write failing frontend tests**

Assert:

- HTML references only local CSS, JS, and media routes.
- No remote `http://`, `https://`, `ws://`, or `wss://` resource is loaded.
- Labels include “Navigation baseline viewer” and “Delayed live video”.
- All required IDs exist.
- JS polls `/api/run` and `/api/events` every 250-500 ms.
- Polling has bounded failures and continues after video errors.
- Native HLS is tried first, then local `Hls`.
- Video initialization failure reveals `video-error`.
- Replay stays hidden until `replay_available` is true.
- When `replay_available` becomes true, `switchToReplay()` stops and detaches
  live HLS, changes the existing `#video` source to `/replay.mp4`, loads it,
  and leaves native controls available for playback and seeking.
- `switchToReplay()` runs only once; subsequent polls must not reset playback
  position.
- Map transforms only pixels, not robot coordinates.
- CSS has narrow-screen media rules and stable dimensions.
- No viewport-unit font sizes.
- Vendored SHA256 matches `hls.min.js`.
- Vendor README records version `1.5.13`, source
  `https://cdn.jsdelivr.net/npm/hls.js@1.5.13/dist/hls.min.js`, checksum, and Apache-2.0.

- [ ] **Step 2: Run frontend tests and verify RED**

Run:

```bash
.venv/bin/python -m pytest \
  tests/unit_tests/robots/lynsense/simulation/test_viewer_assets.py -q
```

Expected: frontend and vendor assertions fail.

- [ ] **Step 3: Vendor exact HLS fallback**

At implementation time fetch once:

```bash
curl -fsSL \
  https://cdn.jsdelivr.net/npm/hls.js@1.5.13/dist/hls.min.js \
  -o robots/lynsense/simulation/viewer/static/vendor/hls.min.js
sha256sum robots/lynsense/simulation/viewer/static/vendor/hls.min.js \
  > robots/lynsense/simulation/viewer/static/vendor/hls.min.js.sha256
```

Write provenance README. Runtime never accesses this URL.

- [ ] **Step 4: Implement frontend**

Use local HTML/CSS/JS with functions:

```javascript
const state = {
  afterIndex: -1,
  consecutiveFailures: 0,
  events: [],
  latestRun: null,
  hls: null,
  replayLoaded: false,
  videoInitialized: false,
};

async function fetchJSON(path) {
  const response = await fetch(path, {cache: "no-store"});
  if (!response.ok) throw new Error(`HTTP ${response.status}`);
  return response.json();
}

async function fetchRun() {
  return fetchJSON("/api/run");
}

async function fetchEvents(afterIndex) {
  return fetchJSON(`/api/events?after=${afterIndex}`);
}

function renderRun(run) {
  state.latestRun = run;
  document.querySelector("#run-state").textContent = run.status;
  renderTelemetry(run);
  document.querySelector("#replay-link").hidden = !run.replay_available;
  drawMap(run.map, state.events);
}

function renderEvents(events) {
  state.events.push(...events.filter((event) => event.event.pose));
  if (state.events.length > 2000) state.events.splice(0, state.events.length - 2000);
  appendTimelineEntries(events.slice(-200));
  drawMap(state.latestRun.map, state.events);
}

function drawMap(map, events) {
  const canvas = document.querySelector("#map");
  const context = canvas.getContext("2d");
  const arena = map.arena;
  const goalValues = Object.values(map.goals).flatMap((goal) => [goal.x, goal.y]);
  const trajectory = events.flatMap((event) => [
    event.event.pose.x, event.event.pose.y,
  ]);
  const values = [...goalValues, ...trajectory];
  const minX = Math.min(-arena.boundary_x_m, ...values);
  const maxX = Math.max(arena.boundary_x_m, ...values);
  const minY = Math.min(-arena.boundary_y_m, ...values);
  const maxY = Math.max(arena.boundary_y_m, ...values);
  const scale = Math.min(
    canvas.width / (maxX - minX),
    canvas.height / (maxY - minY),
  );
  const toX = (x) => (x - minX) * scale;
  const toY = (y) => canvas.height - (y - minY) * scale;
  context.clearRect(0, 0, canvas.width, canvas.height);
  context.strokeRect(
    toX(-arena.boundary_x_m), toY(arena.boundary_y_m),
    2 * arena.boundary_x_m * scale, 2 * arena.boundary_y_m * scale,
  );
  for (const [name, goal] of Object.entries(map.goals)) {
    context.fillText(name, toX(goal.x), toY(goal.y));
  }
  context.beginPath();
  events.forEach((event, index) => {
    const x = toX(event.event.pose.x);
    const y = toY(event.event.pose.y);
    if (index === 0) context.moveTo(x, y);
    else context.lineTo(x, y);
  });
  context.stroke();
}

function showVideoError(message) {
  const element = document.querySelector("#video-error");
  element.textContent = message;
  element.hidden = false;
}

function initializeVideo() {
  const video = document.querySelector("#video");
  if (video.canPlayType("application/vnd.apple.mpegurl")) {
    video.src = "/video/index.m3u8";
  } else if (window.Hls && Hls.isSupported()) {
    const hls = new Hls();
    state.hls = hls;
    hls.on(Hls.Events.ERROR, (_event, data) => {
      if (data.fatal) showVideoError("Live video stream failed");
    });
    hls.loadSource("/video/index.m3u8");
    hls.attachMedia(video);
  } else {
    showVideoError("HLS playback is unavailable in this browser");
  }
}

function switchToReplay() {
  if (state.replayLoaded) return;
  state.replayLoaded = true;
  const video = document.querySelector("#video");
  video.pause();
  if (state.hls) {
    state.hls.destroy();
    state.hls = null;
  }
  video.src = "/replay.mp4";
  video.load();
}

async function poll() {
  try {
    const [run, eventPage] = await Promise.all([
      fetchRun(),
      fetchEvents(state.afterIndex),
    ]);
    state.consecutiveFailures = 0;
    renderRun(run);
    if (run.replay_available) switchToReplay();
    if (eventPage.events.length) {
      state.afterIndex = eventPage.events.at(-1).index;
      renderEvents(eventPage.events);
    }
    if (!state.videoInitialized && run.video_available) {
      state.videoInitialized = true;
      initializeVideo();
    }
  } catch (error) {
    state.consecutiveFailures += 1;
    document.querySelector("#run-state").textContent = "connection retrying";
  }
}

setInterval(() => poll(), 400);
```

Behavior:

- retain at most 2,000 trajectory samples and 200 timeline entries;
- draw arena boundary, goals, heading, and trail;
- show current action, phase, reason, and remaining values;
- show terminal pose/rates;
- use a quiet operational layout with no decorative gradients;
- remain readable and non-overlapping at desktop and narrow widths.

- [ ] **Step 5: Run frontend tests**

Run:

```bash
.venv/bin/python -m pytest \
  tests/unit_tests/robots/lynsense/simulation/test_viewer_assets.py -q
```

Expected: all tests pass.

---

### Task 6: Documentation, Container, Browser, And Regression Evidence

**Files:**

- Modify: `robots/lynsense/simulation/README.md`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_static_assets.py`

**Interfaces:**

- Produces documented launch, shutdown, and verified browser/media evidence.
- Produces `.artifacts/lynsense-webots/viewer/<run-id>/` artifacts.

- [ ] **Step 1: Write failing README contract**

Require the README to contain:

- `LYNSENSE_VIEWER_PHASE=match`;
- `docker compose -f compose.yaml -f compose.viewer.yaml up lynsense-webots-viewer`;
- `http://<workstation-ip>:7047/`;
- `docker compose -f compose.yaml -f compose.viewer.yaml down`;
- `Navigation baseline viewer`;
- `2-5 seconds`;
- viewer artifact directory;
- “not a complete box-moving task”;
- “does not authorize real-robot motion”.

- [ ] **Step 2: Run README test and verify RED**

Run:

```bash
.venv/bin/python -m pytest \
  tests/unit_tests/robots/lynsense/simulation/test_static_assets.py -q
```

Expected: new README assertions fail.

- [ ] **Step 3: Document viewer launch and evidence**

Add launch, LAN URL, phase choices, shutdown, artifacts, delay, and safety boundary to `README.md`.

- [ ] **Step 4: Build combined image**

Run:

```bash
cd robots/lynsense/simulation/docker
docker compose -f compose.yaml -f compose.viewer.yaml build
```

Record local image digest and confirm the pinned base image.

- [ ] **Step 5: Run and inspect `match`**

Start with the documented `LYNSENSE_VIEWER_PHASE=match` Compose command. From a LAN browser verify page load, HLS ready state and advancing playback, live map/timeline updates, terminal pose/rates, and MP4 play/seek.

Capture Playwright or browser screenshots at `1440x900` and `390x844`; verify no overlap, clipping, or unreadable text.

Stop with Ctrl-C and run `docker compose -f compose.yaml -f compose.viewer.yaml down`.

- [ ] **Step 6: Validate media artifact**

Run:

```bash
ffprobe -v error -show_entries stream=codec_name,width,height \
  -show_entries format=duration -of json replay.mp4
```

Expect H.264, finite positive duration, and no audio track. Confirm playlist ends with `#EXT-X-ENDLIST`.

- [ ] **Step 7: Repeat for `smoke` and `blocked`**

Repeat Step 5 with both phases. For `blocked`, show expected `translate_stalled` as a completed viewer outcome rather than HTTP failure.

- [ ] **Step 8: Run local regressions**

```bash
.venv/bin/python -m pytest tests/unit_tests/robots/lynsense/simulation -q
.venv/bin/python -m pytest tests/unit_tests/robots/lynsense -q
git diff --check
```

Expected: all tests pass and diff check is clean.

- [ ] **Step 9: Run original no-network gate**

```bash
cd robots/lynsense/simulation/docker
docker compose run --rm lynsense-webots-smoke --phase all
```

Expected: all four phases pass and summary `passed` is true.

- [ ] **Step 10: Audit cleanup**

Run:

```bash
docker compose -f robots/lynsense/simulation/docker/compose.yaml ps
docker compose -f robots/lynsense/simulation/docker/compose.viewer.yaml ps
pgrep -af 'webots|lynsense_pytrees_node|lynsense_webots_controller|ffmpeg|Xvfb|lynsense_run_viewer' || true
find robots/lynsense/simulation tests/unit_tests/robots/lynsense/simulation \
  -type d -name __pycache__ -print
```

Expected: no service/process remains and no generated `__pycache__` remains.

---

## Completion Criteria

- `smoke`, `blocked`, and `match` can each be observed live from LAN port 7047.
- HLS playback advances in a real browser, not merely in ffprobe.
- Event API, map, and action timeline update during execution.
- Completed runs retain HLS, MP4, JSONL, logs, and viewer summary by run ID.
- Browser evidence covers desktop and narrow viewports without overlap.
- Original no-network `--phase all` gate remains unchanged and passes.
- Frontend uses only local assets and exposes no mutation endpoint.
- No real robot connection, RPent motion Toolkit, or complete box-moving task is introduced.

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-09-21-lynsense-webots-viewer.md`.

**1. Subagent-Driven (recommended)** - dispatch a fresh subagent per task, review between tasks, and keep implementation narrowly scoped.

**2. Inline Execution** - execute tasks in this session using `superpowers:executing-plans`, with checkpoints after each task.
