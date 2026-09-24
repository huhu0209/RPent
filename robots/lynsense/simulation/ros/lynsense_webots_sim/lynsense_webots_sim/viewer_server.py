from __future__ import annotations

import json
import re
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from lynsense_webots_sim.viewer_state import ViewerRunStore


_STATIC_ROUTES = {
    "/static/app.css": (Path("app.css"), "text/css; charset=utf-8"),
    "/static/app.js": (Path("app.js"), "text/javascript; charset=utf-8"),
    "/static/vendor/hls.min.js": (
        Path("vendor") / "hls.min.js",
        "text/javascript; charset=utf-8",
    ),
}
_SEGMENT_PATTERN = re.compile(r"^/video/segment-[0-9]{6,}\.ts$")
_VIEW_SEGMENT_PATTERN = re.compile(
    r"^/video/(?P<view>scene|robot)/segment-[0-9]{6,}\.ts$"
)
_FILE_CHUNK_SIZE = 64 * 1024


class ViewerHTTPServer:
    def __init__(
        self,
        host: str,
        port: int,
        store: ViewerRunStore,
        static_root: Path,
        video_dir: Path,
        replay_path: Path,
    ) -> None:
        self.host = host
        self.port = port
        self.store = store
        self.static_root = static_root
        self.video_dir = video_dir
        self.replay_path = replay_path
        self._server: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> str:
        if self._server is not None:
            return self._base_url()

        owner = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "LynsenseViewer/1"

            def do_GET(self) -> None:
                owner._handle_get(self)

            def do_POST(self) -> None:
                self._method_not_allowed()

            def do_PUT(self) -> None:
                self._method_not_allowed()

            def do_PATCH(self) -> None:
                self._method_not_allowed()

            def do_DELETE(self) -> None:
                self._method_not_allowed()

            def _method_not_allowed(self) -> None:
                self.send_response(HTTPStatus.METHOD_NOT_ALLOWED)
                self.send_header("Allow", "GET")
                self.send_header("Content-Length", "0")
                self.end_headers()

            def log_message(self, format: str, *args: object) -> None:
                return

        self._server = ThreadingHTTPServer((self.host, self.port), Handler)
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="lynsense-viewer-http",
            daemon=True,
        )
        self._thread.start()
        return self._base_url()

    def stop(self) -> None:
        server = self._server
        thread = self._thread
        self._server = None
        self._thread = None
        if server is None:
            return
        server.shutdown()
        server.server_close()
        if thread is not None:
            thread.join(timeout=2.0)

    def is_alive(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    def _base_url(self) -> str:
        if self._server is None:
            raise RuntimeError("viewer server is not running")
        host, port = self._server.server_address[:2]
        return f"http://{host}:{port}"

    def _handle_get(self, handler: BaseHTTPRequestHandler) -> None:
        request = urlsplit(handler.path)
        path = request.path
        asset_root = self._asset_root()
        if path == "/":
            self._send_file(
                handler,
                self.static_root / "index.html",
                "text/html; charset=utf-8",
                root=self.static_root,
                cache_control="no-store",
                fallback=b"<!doctype html><title>Lynsense Viewer</title>\n",
            )
            return
        if path == "/healthz":
            self._send_bytes(
                handler,
                b'{"ok": true}',
                "application/json; charset=utf-8",
                cache_control="no-store",
            )
            return
        if path == "/api/run":
            self._send_json(handler, self.store.snapshot())
            return
        if path == "/api/events":
            self._send_events(handler, request.query)
            return
        if path in _STATIC_ROUTES:
            relative, media_type = _STATIC_ROUTES[path]
            self._send_file(
                handler,
                asset_root / relative,
                media_type,
                root=asset_root,
            )
            return
        if path == "/video/index.m3u8":
            self._send_file(
                handler,
                self.video_dir / "scene" / "index.m3u8",
                "application/vnd.apple.mpegurl",
                root=self.video_dir,
                cache_control="no-store",
            )
            return
        if _SEGMENT_PATTERN.fullmatch(path):
            self._send_file(
                handler,
                self.video_dir / "scene" / Path(path).name,
                "video/mp2t",
                root=self.video_dir,
                cache_control="no-store",
            )
            return
        if path in {
            "/video/scene/index.m3u8",
            "/video/robot/index.m3u8",
        }:
            view = path.split("/")[2]
            self._send_file(
                handler,
                self.video_dir / view / "index.m3u8",
                "application/vnd.apple.mpegurl",
                root=self.video_dir,
                cache_control="no-store",
            )
            return
        segment_match = _VIEW_SEGMENT_PATTERN.fullmatch(path)
        if segment_match is not None:
            view = segment_match.group("view")
            self._send_file(
                handler,
                self.video_dir / view / Path(path).name,
                "video/mp2t",
                root=self.video_dir,
                cache_control="no-store",
            )
            return
        if path in {"/replay.mp4", "/replay/scene.mp4", "/replay/robot.mp4"}:
            if not self.store.snapshot()["replay_available"]:
                self._send_error(handler, HTTPStatus.NOT_FOUND)
                return
            replay_path = self.replay_path
            if path == "/replay/robot.mp4":
                replay_path = self.replay_path.parent / "replay-robot.mp4"
            self._send_file(
                handler,
                replay_path,
                "video/mp4",
                root=self.replay_path.parent,
                cache_control="no-store",
                allow_range=True,
            )
            return
        self._send_error(handler, HTTPStatus.NOT_FOUND)

    def _asset_root(self) -> Path:
        nested = self.static_root / "static"
        return nested if nested.is_dir() else self.static_root

    def _send_events(self, handler: BaseHTTPRequestHandler, query: str) -> None:
        values = parse_qs(query, keep_blank_values=True)
        try:
            after = int(values.get("after", ["-1"])[0])
            limit = int(values.get("limit", ["512"])[0])
        except (TypeError, ValueError):
            self._send_error(handler, HTTPStatus.BAD_REQUEST)
            return
        if after < -1 or limit <= 0:
            self._send_error(handler, HTTPStatus.BAD_REQUEST)
            return
        self._send_json(handler, {"events": self.store.events_after(after, limit)})

    def _send_json(self, handler: BaseHTTPRequestHandler, value: object) -> None:
        payload = json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
        self._send_bytes(
            handler,
            payload,
            "application/json; charset=utf-8",
            cache_control="no-store",
        )

    def _send_file(
        self,
        handler: BaseHTTPRequestHandler,
        path: Path,
        media_type: str,
        root: Path,
        cache_control: str | None = None,
        fallback: bytes | None = None,
        allow_range: bool = False,
    ) -> None:
        try:
            resolved = path.resolve(strict=True)
        except FileNotFoundError:
            if fallback is not None:
                self._send_bytes(handler, fallback, media_type, cache_control)
            else:
                self._send_error(handler, HTTPStatus.NOT_FOUND)
            return
        resolved_root = root.resolve()
        try:
            resolved.relative_to(resolved_root)
        except ValueError:
            self._send_error(handler, HTTPStatus.NOT_FOUND)
            return
        try:
            size = resolved.stat().st_size
            stream = resolved.open("rb")
        except OSError:
            self._send_error(handler, HTTPStatus.NOT_FOUND)
            return
        try:
            byte_range = None
            if allow_range and "Range" in handler.headers:
                byte_range = self._parse_byte_range(handler.headers["Range"], size)
                if byte_range is None:
                    self._send_range_error(handler, size)
                    return

            if byte_range is None:
                status = HTTPStatus.OK
                content_length = size
            else:
                status = HTTPStatus.PARTIAL_CONTENT
                start, end = byte_range
                content_length = end - start + 1

            handler.send_response(status)
            handler.send_header("Content-Type", media_type)
            handler.send_header("Content-Length", str(content_length))
            if allow_range:
                handler.send_header("Accept-Ranges", "bytes")
            if byte_range is not None:
                handler.send_header("Content-Range", f"bytes {start}-{end}/{size}")
            if cache_control is not None:
                handler.send_header("Cache-Control", cache_control)
            handler.end_headers()

            if byte_range is not None:
                stream.seek(start)
            remaining = content_length
            while remaining > 0 and (chunk := stream.read(min(_FILE_CHUNK_SIZE, remaining))):
                handler.wfile.write(chunk)
                remaining -= len(chunk)
        finally:
            stream.close()

    @staticmethod
    def _parse_byte_range(range_header: str, size: int) -> tuple[int, int] | None:
        if not range_header.startswith("bytes=") or "," in range_header:
            return None
        match = re.fullmatch(r"bytes=([0-9]*)-([0-9]*)", range_header)
        if match is None:
            return None
        start_text, end_text = match.groups()
        if not start_text and not end_text:
            return None

        try:
            if not start_text:
                suffix_length = int(end_text)
                if suffix_length <= 0 or size == 0:
                    return None
                return max(0, size - suffix_length), size - 1

            start = int(start_text)
            if start >= size:
                return None
            end = size - 1 if not end_text else min(int(end_text), size - 1)
            if start > end:
                return None
            return start, end
        except ValueError:
            return None

    @staticmethod
    def _send_range_error(handler: BaseHTTPRequestHandler, size: int) -> None:
        handler.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
        handler.send_header("Content-Range", f"bytes */{size}")
        handler.send_header("Content-Length", "0")
        handler.end_headers()

    @staticmethod
    def _send_bytes(
        handler: BaseHTTPRequestHandler,
        payload: bytes,
        media_type: str,
        cache_control: str | None = None,
    ) -> None:
        handler.send_response(HTTPStatus.OK)
        handler.send_header("Content-Type", media_type)
        handler.send_header("Content-Length", str(len(payload)))
        if cache_control is not None:
            handler.send_header("Cache-Control", cache_control)
        handler.end_headers()
        handler.wfile.write(payload)

    @staticmethod
    def _send_error(handler: BaseHTTPRequestHandler, status: HTTPStatus) -> None:
        handler.send_response(status)
        handler.send_header("Content-Length", "0")
        handler.end_headers()
