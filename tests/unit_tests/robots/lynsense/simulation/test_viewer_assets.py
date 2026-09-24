from __future__ import annotations

from pathlib import Path
import json
import shutil
import subprocess

import pytest
import yaml


DOCKER_DIR = Path("robots/lynsense/simulation/docker")
VIEWER_DIR = Path("robots/lynsense/simulation/viewer")


def test_first_poll_of_completed_run_uses_scene_mp4_source():
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node is required for the viewer JavaScript behavior check")
    result = subprocess.run(
        [node, "-e", r'''
const fs = require("fs"), vm = require("vm");
const context = new Proxy({}, {get: () => () => {}});
const video = {pause() {}, load() {}, canPlayType() { return "probably"; }, setAttribute() {}};
const elements = new Map([["#video", video]]);
const document = {querySelector(id) {
  if (!elements.has(id)) elements.set(id, {
    width: 900, height: 500, children: [], getContext: () => context,
    classList: {toggle() {}}, setAttribute() {}, addEventListener() {},
  });
  return elements.get(id);
}};
const run = {status: "completed", video_available: true, replay_available: true};
const sandbox = {
  document,
  window: {setInterval() {}},
  fetch: async path => ({ok: true, json: async () => path === "/api/run" ? run : {events: []}}),
};
vm.runInNewContext(fs.readFileSync(process.argv[1], "utf8"), sandbox);
setImmediate(() => console.log(JSON.stringify({
  src: video.src, replayLoaded: sandbox.window.viewerState.replayLoaded,
  failures: sandbox.window.viewerState.consecutiveFailures,
})));
''', str(VIEWER_DIR / "static" / "app.js")],
        text=True, capture_output=True, check=True, timeout=10,
    )
    assert json.loads(result.stdout) == {
        "src": "/replay/scene.mp4", "replayLoaded": True, "failures": 0,
    }

def test_view_switching_changes_live_source_and_replay_source():
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node is required for the viewer JavaScript behavior check")
    result = subprocess.run(
        [node, "-e", r'''
const fs = require("fs"), vm = require("vm");
const context = new Proxy({}, {get: () => () => {}});
const listeners = {};
const video = {
  paused: true, currentTime: 12, canPlayType() { return ""; },
  pause() { this.paused = true; }, load() { this.currentTime = 0; }, play() { this.paused = false; },
  setAttribute() {}, addEventListener(name, callback) { listeners[name] = callback; },
};
const elements = new Map([["#video", video]]);
const document = {querySelector(id) {
  if (!elements.has(id)) elements.set(id, {
    width: 900, height: 500, children: [], getContext: () => context,
    classList: {toggle() {}}, setAttribute() {}, addEventListener(name, callback) { this.listener = callback; },
  });
  return elements.get(id);
}};
const hlsInstances = [];
class Hls {
  static isSupported() { return true; }
  static Events = {ERROR: "error"};
  constructor() { this.sources = []; this.destroyed = false; hlsInstances.push(this); }
  on() {}
  loadSource(source) { this.sources.push(source); }
  attachMedia() {}
  destroy() { this.destroyed = true; }
}
let run = {status: "running", video_available: true, replay_available: false};
const sandbox = {
  document,
  window: {Hls, setInterval() {}},
  fetch: async path => ({ok: true, json: async () => path === "/api/run" ? run : {events: []}}),
};
vm.runInNewContext(fs.readFileSync(process.argv[1], "utf8"), sandbox);
setImmediate(async () => {
  await sandbox.window.viewerPoll();
  video.currentTime = 12;
  elements.get("#view-robot").listener();
  const liveSource = hlsInstances.at(-1).sources[0];
  const liveTime = video.currentTime;
  run = {status: "completed", video_available: true, replay_available: true};
  await sandbox.window.viewerPoll();
  video.currentTime = 18;
  video.paused = false;
  elements.get("#view-scene").listener();
  const replaySource = video.src;
  console.log(JSON.stringify({
    liveSource, liveTime, replaySource, currentTime: video.currentTime, paused: video.paused,
    oldHlsDestroyed: hlsInstances[0].destroyed,
  }));
});
''', str(VIEWER_DIR / "static" / "app.js")],
        text=True, capture_output=True, check=True, timeout=10,
    )
    assert json.loads(result.stdout) == {
        "liveSource": "/video/robot/index.m3u8",
        "liveTime": 12,
        "replaySource": "/replay/scene.mp4",
        "currentTime": 18,
        "paused": False,
        "oldHlsDestroyed": True,
    }


def compose_services(name: str) -> dict:
    return yaml.safe_load((DOCKER_DIR / name).read_text(encoding="utf-8"))["services"]


def test_original_gate_remains_isolated():
    gate = compose_services("compose.yaml")["lynsense-webots-smoke"]
    assert gate["network_mode"] == "none"
    assert "ports" not in gate


def test_viewer_publishes_only_read_only_http_port():
    viewer = compose_services("compose.viewer.yaml")["lynsense-webots-viewer"]
    assert viewer["network_mode"] == "bridge"
    assert viewer["ports"] == ["7047:7047"]
    assert viewer["init"] is True
    assert viewer["environment"]["ROS_DOMAIN_ID"] == "42"
    assert viewer["environment"]["ROS_LOCALHOST_ONLY"] == "1"
    assert viewer["environment"]["LYNSENSE_VIEWER_HOST"] == "0.0.0.0"
    assert viewer["environment"]["LYNSENSE_VIEWER_PORT"] == "7047"
    assert viewer["command"] == ["--phase", "${LYNSENSE_VIEWER_PHASE:-match}"]
    assert viewer["entrypoint"] == [
        "python3",
        "/workspace/simulation/docker/bootstrap_viewer.py",
    ]


def test_viewer_uses_exactly_the_three_approved_mounts():
    viewer = compose_services("compose.viewer.yaml")["lynsense-webots-viewer"]
    assert viewer["volumes"] == [
        {
            "type": "bind",
            "source": "..",
            "target": "/workspace/simulation",
            "read_only": True,
        },
        {
            "type": "bind",
            "source": "../../../../../lynsense_pytrees",
            "target": "/workspace/ws/src/lynsense_pytrees",
            "read_only": True,
        },
        {
            "type": "bind",
            "source": "../../../../.artifacts/lynsense-webots",
            "target": "/workspace/artifacts",
            "read_only": False,
        },
    ]


def test_viewer_docker_contract_is_explicit():
    source = (DOCKER_DIR / "Dockerfile").read_text(encoding="utf-8")
    assert "cyberbotics/webots@sha256:f0023e30daf38b172e4e6ad24ed345909bcd9551df34d63d824e121a7cebf099" in source
    assert "ffmpeg" in source
    compose = compose_services("compose.viewer.yaml")["lynsense-webots-viewer"]
    assert compose["build"] == {"context": ".", "dockerfile": "Dockerfile"}
    assert compose["image"] == "rpent-lynsense-webots:r2025a-humble"


def test_viewer_html_uses_local_resources_and_required_ids():
    source = (VIEWER_DIR / "index.html").read_text(encoding="utf-8")
    assert "Dual camera viewer" in source
    assert "Simulation video" in source
    for element_id in (
        "video", "run-state", "pose", "wheel-rates", "map",
        "action-timeline", "terminal-summary", "replay-link", "video-error",
        "view-scene", "view-robot",
    ):
        assert f'id="{element_id}"' in source
    assert "http://" not in source
    assert "https://" not in source
    assert "ws://" not in source
    assert "wss://" not in source


def test_viewer_javascript_contracts_are_local_and_replay_safe():
    source = (VIEWER_DIR / "static" / "app.js").read_text(encoding="utf-8")
    assert 'fetchJSON("/api/run")' in source
    assert 'fetchEvents(state.afterIndex)' in source
    assert "setInterval(poll, 400)" in source
    assert 'canPlayType("application/vnd.apple.mpegurl")' in source
    assert "new HlsConstructor()" in source
    assert 'showVideoError("HLS playback is unavailable in this browser")' in source
    assert 'live: "/video/scene/index.m3u8"' in source
    assert 'live: "/video/robot/index.m3u8"' in source
    assert 'replay: "/replay/scene.mp4"' in source
    assert 'replay: "/replay/robot.mp4"' in source
    assert "if (state.mediaMode === \"replay\" && state.mediaView === state.view) return" in source
    assert "video.pause()" in source
    assert "state.hls.destroy()" in source
    assert "canvas.width" in source
    assert "record.event.pose.x" in source
    assert "window.setInterval(poll, 400)" in source


def test_viewer_css_is_responsive_without_viewport_font_units():
    source = (VIEWER_DIR / "static" / "app.css").read_text(encoding="utf-8")
    assert "@media (max-width: 820px)" in source
    assert "@media (max-width: 420px)" in source
    assert "aspect-ratio: 16 / 9" in source
    assert "vw" not in source
    assert "vh" not in source
    assert "gradient" not in source


def test_vendored_hls_checksum_and_provenance():
    vendor = VIEWER_DIR / "static" / "vendor"
    checksum = (vendor / "hls.min.js.sha256").read_text(encoding="utf-8").split()[0]
    import hashlib

    actual = hashlib.sha256((vendor / "hls.min.js").read_bytes()).hexdigest()
    assert checksum == actual
    readme = (vendor / "README.md").read_text(encoding="utf-8")
    assert "1.5.13" in readme
    assert "https://cdn.jsdelivr.net/npm/hls.js@1.5.13/dist/hls.min.js" in readme
    assert "Apache-2.0" in readme
