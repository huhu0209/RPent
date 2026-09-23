from __future__ import annotations

import ast
import builtins
import json
import socket
import sys
from contextlib import contextmanager
from pathlib import Path
from typing import Any

from pydantic_ai.models.function import FunctionModel

from tests.unit_tests.robots.lynsense_real_box.test_api_planner import (
    MAX_TURNS,
    REAL_BOX_CALLS,
    offline_backend_arguments,
    read_evidence,
    scripted_real_box_model,
)


def _cli_module() -> Any:
    from rpent.cli import main as cli

    return cli


def test_real_box_production_modules_have_no_ros_or_network_imports() -> None:
    forbidden = {"socket", "rclpy", "httpx", "http", "urllib", "lynsense_pytrees"}
    production_dir = Path("robots/lynsense_real_box")
    for path in production_dir.glob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        imported_roots = {
            node.names[0].name.split(".", 1)[0]
            for node in ast.walk(tree)
            if isinstance(node, ast.Import)
        }
        imported_roots |= {
            (node.module or "").split(".", 1)[0]
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom)
        }
        assert not forbidden & imported_roots, (path, forbidden & imported_roots)


@contextmanager
def _offline_transport_guard():
    forbidden_roots = {"rclpy", "lynsense_pytrees"}
    original_import = builtins.__import__

    def guarded_import(name: str, *args: Any, **kwargs: Any):
        if name.split(".", 1)[0] in forbidden_roots:
            raise AssertionError(f"offline CLI imported forbidden module: {name}")
        return original_import(name, *args, **kwargs)

    original_connect = socket.socket.connect
    original_connect_ex = socket.socket.connect_ex
    original_getaddrinfo = socket.getaddrinfo

    def reject_network(*args: Any, **kwargs: Any):
        raise AssertionError("offline CLI attempted a network operation")

    builtins.__import__ = guarded_import
    socket.socket.connect = reject_network
    socket.socket.connect_ex = reject_network
    socket.getaddrinfo = reject_network
    try:
        yield set(sys.modules)
    finally:
        builtins.__import__ = original_import
        socket.socket.connect = original_connect
        socket.socket.connect_ex = original_connect_ex
        socket.getaddrinfo = original_getaddrinfo


def test_cli_lifecycle_completes_offline_single_box_workflow(
    tmp_path: Path,
    monkeypatch: Any,
) -> None:
    cli = _cli_module()
    args = offline_backend_arguments(tmp_path)
    output_dir = args.output_dir
    monkeypatch.setattr(
        "pydantic_ai.models.infer_model",
        lambda *_args, **_kwargs: FunctionModel(scripted_real_box_model),
    )
    monkeypatch.setattr(
        sys,
        "stdin",
        type("OfflineTty", (), {"isatty": lambda _self: True})(),
    )
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "rpent",
            "--robot",
            "lynsense_real_box",
            "--planner",
            "api",
            "--model",
            "offline:function-model",
            "--max-turns",
            str(MAX_TURNS),
            "--planner-timeout-s",
            "10",
            "--output-dir",
            str(output_dir),
            "--memory-profile",
            "local",
            "--memory-dir",
            str(args.memory_dir),
            "--site-profile",
            str(args.site_profile),
        ],
    )

    before_modules: set[str]
    with _offline_transport_guard() as before_modules:
        exit_code = cli.main()

    forbidden_new_roots = {"rclpy", "socket", "httpx", "lynsense_pytrees"}
    assert exit_code == 0
    assert not forbidden_new_roots & (set(sys.modules) - before_modules)

    evidence_path = output_dir / "events-lynsense_real_box.jsonl"
    transcript_path = output_dir / "transcript_lynsense_real_box_phase1.json"
    evidence = read_evidence(evidence_path)
    transcript = json.loads(transcript_path.read_text(encoding="utf-8"))
    assert transcript_path.exists()
    assert evidence_path.exists()
    assert [(event["tool"], event["arguments"]) for event in evidence] == list(
        REAL_BOX_CALLS
    )
    assert transcript["stats"]["tool_calls"] == len(evidence)
    assert transcript["stats"]["tool_calls"] > 10
    assert evidence[-1]["tool"] == "finish"
    assert evidence[-1]["result"]["task_state"] == "safe_complete"
    assert transcript["finish"] == {"_finish": True, "status": "success"}
    assert evidence[0]["profile_sha256"] in transcript_path.read_text(
        encoding="utf-8"
    )
