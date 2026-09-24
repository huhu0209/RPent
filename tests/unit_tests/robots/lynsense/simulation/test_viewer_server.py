from __future__ import annotations

import json
import socket
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

import pytest

from lynsense_webots_sim.viewer_server import ViewerHTTPServer
from lynsense_webots_sim.viewer_state import ViewerRunStore


ORIGINAL_CONNECT = socket.socket.connect
ORIGINAL_GETADDRINFO = socket.getaddrinfo
LOCAL_OPENER = build_opener(ProxyHandler({}))


def loopback_connect(connection, address):
    if address[0] != "127.0.0.1":
        raise AssertionError("Viewer tests only allow IPv4 loopback")
    return ORIGINAL_CONNECT(connection, address)


def loopback_getaddrinfo(host, port, *args, **kwargs):
    if host != "127.0.0.1":
        raise AssertionError("Viewer tests only allow IPv4 loopback")
    return ORIGINAL_GETADDRINFO(host, port, *args, **kwargs)


@pytest.fixture
def viewer(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setattr(socket.socket, "connect", loopback_connect)
    monkeypatch.setattr(socket, "getaddrinfo", loopback_getaddrinfo)
    artifact = tmp_path / "run"
    video = artifact / "video"
    scene_video = video / "scene"
    robot_video = video / "robot"
    static = tmp_path / "static"
    vendor = static / "vendor"
    scene_video.mkdir(parents=True)
    robot_video.mkdir(parents=True)
    vendor.mkdir(parents=True)
    (static / "index.html").write_text("<html>viewer</html>", encoding="utf-8")
    (static / "app.css").write_text("body{}", encoding="utf-8")
    (static / "app.js").write_text("export {}", encoding="utf-8")
    (vendor / "hls.min.js").write_text(
        "/*! hls.js v1.5.13 */", encoding="utf-8"
    )
    (scene_video / "index.m3u8").write_text("#EXTM3U scene\n", encoding="utf-8")
    (scene_video / "segment-000001.ts").write_bytes(b"scene segment")
    (robot_video / "index.m3u8").write_text("#EXTM3U robot\n", encoding="utf-8")
    (robot_video / "segment-000001.ts").write_bytes(b"robot segment")
    replay = artifact / "replay.mp4"
    replay.write_bytes(b"0123456789")
    (artifact / "replay-robot.mp4").write_bytes(b"abcdefghij")
    event_path = artifact / "events.jsonl"
    event_path.touch()
    store = ViewerRunStore(
        phase="match",
        run_id="server-run",
        event_path=event_path,
        artifact_dir=artifact,
        video_dir=video,
        video_start_mono=0.0,
    )
    server = ViewerHTTPServer(
        host="127.0.0.1",
        port=0,
        store=store,
        static_root=static,
        video_dir=video,
        replay_path=replay,
    )
    base = server.start()
    try:
        yield base, store
    finally:
        server.stop()


def get(base: str, path: str):
    return LOCAL_OPENER.open(base + path, timeout=2)


def status(base: str, method: str, path: str) -> int:
    request = Request(base + path, method=method)
    try:
        get_response = LOCAL_OPENER.open(request, timeout=2)
    except HTTPError as exc:
        return exc.code
    return get_response.status


def test_read_only_routes_and_pagination(viewer):
    base, store = viewer
    store.event_path.write_text(
        '{"kind":"one"}\n{"kind":"two"}\n', encoding="utf-8"
    )
    store.ingest_new_events(1.0)

    assert get(base, "/").status == 200
    assert get(base, "/healthz").read() == b'{"ok": true}'
    assert json.loads(get(base, "/api/run").read())["run_id"] == "server-run"
    assert [event["index"] for event in json.loads(get(base, "/api/events?after=-1").read())["events"]] == [0, 1]
    assert [event["index"] for event in json.loads(get(base, "/api/events?after=0").read())["events"]] == [1]
    assert status(base, "GET", "/api/events?after=bad") == 400


def test_server_reports_thread_liveness(tmp_path: Path):
    artifact = tmp_path / "run"
    video = artifact / "video"
    static = tmp_path / "static"
    video.mkdir(parents=True)
    static.mkdir()
    event_path = artifact / "events.jsonl"
    event_path.touch()
    store = ViewerRunStore("match", "server-run", event_path, artifact, video)
    server = ViewerHTTPServer(
        "127.0.0.1", 0, store, static, video, artifact / "replay.mp4"
    )

    server.start()
    try:
        assert server.is_alive()
    finally:
        server.stop()
    assert not server.is_alive()


@pytest.mark.parametrize(
    ("path", "media_type"),
    [
        ("/static/app.css", "text/css; charset=utf-8"),
        ("/static/app.js", "text/javascript; charset=utf-8"),
        ("/static/vendor/hls.min.js", "text/javascript; charset=utf-8"),
        ("/video/index.m3u8", "application/vnd.apple.mpegurl"),
        ("/video/segment-000001.ts", "video/mp2t"),
        ("/video/scene/index.m3u8", "application/vnd.apple.mpegurl"),
        ("/video/robot/index.m3u8", "application/vnd.apple.mpegurl"),
        ("/video/scene/segment-000001.ts", "video/mp2t"),
        ("/video/robot/segment-000001.ts", "video/mp2t"),
    ],
)
def test_media_types_and_no_store(viewer, path, media_type):
    base, _ = viewer
    response = get(base, path)
    assert response.headers["Content-Type"] == media_type
    if path.startswith("/video/index"):
        assert response.headers["Cache-Control"] == "no-store"


def test_file_routes_stream_in_bounded_chunks():
    source = Path(
        "robots/lynsense/simulation/ros/lynsense_webots_sim/"
        "lynsense_webots_sim/viewer_server.py"
    ).read_text(encoding="utf-8")
    assert "_FILE_CHUNK_SIZE = 64 * 1024" in source
    assert "resolved.read_bytes()" not in source


def test_replay_is_gated_by_store(viewer):
    base, store = viewer
    with pytest.raises(HTTPError) as missing:
        get(base, "/replay.mp4")
    assert missing.value.code == 404
    store.set_replay_available(True)
    response = get(base, "/replay.mp4")
    assert response.status == 200
    assert response.headers["Content-Type"] == "video/mp4"
    assert response.headers["Content-Length"] == "10"
    assert response.headers["Accept-Ranges"] == "bytes"
    assert "Content-Range" not in response.headers
    assert response.read() == b"0123456789"


def test_dual_camera_routes_serve_distinct_streams(viewer):
    base, store = viewer
    store.set_replay_available(True)

    assert get(base, "/video/scene/index.m3u8").read() == b"#EXTM3U scene\n"
    assert get(base, "/video/robot/index.m3u8").read() == b"#EXTM3U robot\n"
    assert get(base, "/video/scene/segment-000001.ts").read() == b"scene segment"
    assert get(base, "/video/robot/segment-000001.ts").read() == b"robot segment"
    assert get(base, "/video/index.m3u8").read() == b"#EXTM3U scene\n"
    assert get(base, "/video/segment-000001.ts").read() == b"scene segment"
    assert get(base, "/replay/scene.mp4").read() == b"0123456789"
    assert get(base, "/replay/robot.mp4").read() == b"abcdefghij"


def get_with_range(base: str, range_value: str):
    request = Request(
        base + "/replay/scene.mp4",
        headers={"Range": range_value},
    )
    return LOCAL_OPENER.open(request, timeout=2)


@pytest.mark.parametrize(
    ("range_value", "content_range", "payload"),
    [
        ("bytes=2-5", "bytes 2-5/10", b"2345"),
        ("bytes=6-", "bytes 6-9/10", b"6789"),
        ("bytes=-3", "bytes 7-9/10", b"789"),
    ],
)
def test_replay_supports_single_byte_ranges(viewer, range_value, content_range, payload):
    base, store = viewer
    store.set_replay_available(True)

    response = get_with_range(base, range_value)

    assert response.status == 206
    assert response.headers["Content-Type"] == "video/mp4"
    assert response.headers["Accept-Ranges"] == "bytes"
    assert response.headers["Content-Range"] == content_range
    assert response.headers["Content-Length"] == str(len(payload))
    assert response.read() == payload


def test_robot_replay_supports_single_byte_ranges(viewer):
    base, store = viewer
    store.set_replay_available(True)

    request = Request(base + "/replay/robot.mp4", headers={"Range": "bytes=2-5"})
    response = LOCAL_OPENER.open(request, timeout=2)

    assert response.status == 206
    assert response.headers["Content-Range"] == "bytes 2-5/10"
    assert response.headers["Content-Length"] == "4"
    assert response.read() == b"cdef"


@pytest.mark.parametrize(
    "range_value",
    [
        "bytes=10-",
        "bytes=5-4",
        "bytes=-0",
        "bytes=0-1,2-3",
        "bytes=bad",
        "bytes=" + "9" * 5000 + "-",
    ],
)
def test_replay_rejects_invalid_byte_ranges(viewer, range_value):
    base, store = viewer
    store.set_replay_available(True)

    with pytest.raises(HTTPError) as error:
        get_with_range(base, range_value)

    assert error.value.code == 416
    assert error.value.headers["Content-Range"] == "bytes */10"
    assert error.value.headers["Content-Length"] == "0"


@pytest.mark.parametrize(
    ("method", "path"),
    [("POST", "/api/run"), ("PUT", "/"), ("DELETE", "/healthz"), ("PATCH", "/api/events")],
)
def test_mutation_methods_are_rejected(viewer, method, path):
    base, _ = viewer
    request = Request(base + path, method=method)
    with pytest.raises(HTTPError) as error:
        LOCAL_OPENER.open(request, timeout=2)
    assert error.value.code == 405
    assert error.value.headers["Allow"] == "GET"


@pytest.mark.parametrize(
    "path",
    [
        "/../README.md",
        "/static/../viewer_state.py",
        "/video/../events.jsonl",
        "/video/segment-bad.ts",
        "/video/unknown/index.m3u8",
        "/video/scene/nested/index.m3u8",
        "/replay/unknown.mp4",
        "/replay/scene/nested.mp4",
        "/control",
    ],
)
def test_unknown_and_traversal_routes_are_404(viewer, path):
    base, _ = viewer
    with pytest.raises(HTTPError) as error:
        get(base, path)
    assert error.value.code == 404


def test_loopback_fixture_still_blocks_external_connections(viewer):
    with pytest.raises(AssertionError, match="loopback"):
        socket.getaddrinfo("example.com", 80)
    with socket.socket() as connection:
        with pytest.raises(AssertionError, match="loopback"):
            connection.connect(("192.0.2.1", 80))


@pytest.mark.parametrize("route", ["/api/run", "/api/events?after=-1"])
def test_api_is_not_cached(viewer, route):
    base, _ = viewer
    with get(base, route) as response:
        assert response.headers["Cache-Control"] == "no-store"


def test_vendor_directory_symlink_cannot_escape_static_root(viewer, tmp_path):
    base, _ = viewer
    vendor = tmp_path / "static" / "vendor"
    vendor.rename(tmp_path / "outside-vendor")
    vendor.symlink_to(tmp_path / "outside-vendor", target_is_directory=True)
    assert status(base, "GET", "/static/vendor/hls.min.js") == 404


def test_media_symlink_cannot_escape_video_root(viewer, tmp_path):
    base, _ = viewer
    segment = tmp_path / "run" / "video" / "scene" / "segment-000001.ts"
    segment.rename(tmp_path / "private-data")
    segment.symlink_to(tmp_path / "private-data")
    assert status(base, "GET", "/video/segment-000001.ts") == 404


def test_view_symlink_cannot_escape_video_root(viewer, tmp_path):
    base, _ = viewer
    view = tmp_path / "run" / "video" / "robot"
    view.rename(tmp_path / "private-robot")
    view.symlink_to(tmp_path / "private-robot", target_is_directory=True)
    assert status(base, "GET", "/video/robot/index.m3u8") == 404


def test_viewer_root_can_serve_nested_static_directory(viewer, tmp_path):
    base, store = viewer
    nested_root = tmp_path / "viewer-root"
    (nested_root / "static" / "vendor").mkdir(parents=True)
    (nested_root / "index.html").write_text("<html>root</html>", encoding="utf-8")
    (nested_root / "static" / "app.css").write_text("body{}", encoding="utf-8")
    (nested_root / "static" / "app.js").write_text("export {}", encoding="utf-8")
    (nested_root / "static" / "vendor" / "hls.min.js").write_text("hls", encoding="utf-8")
    server = ViewerHTTPServer(
        host="127.0.0.1",
        port=0,
        store=store,
        static_root=nested_root,
        video_dir=tmp_path / "run" / "video",
        replay_path=tmp_path / "run" / "replay.mp4",
    )
    server.start()
    try:
        assert get(server._base_url(), "/static/app.css").read() == b"body{}"
    finally:
        server.stop()
