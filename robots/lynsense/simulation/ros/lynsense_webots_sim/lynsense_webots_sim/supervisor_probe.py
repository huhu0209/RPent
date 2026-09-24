"""Shared launcher for isolated Supervisor-only Webots regressions."""

from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from contextlib import suppress
from pathlib import Path
from typing import Any


WEBOTS_READY_MARKER = "Waiting for local or remote connection"


def launch_probe(controller_path: Path, world: Path, output: Path) -> int:
    if not world.is_file():
        raise RuntimeError(f"Webots world is missing: {world}")
    output.mkdir(parents=True, exist_ok=True)
    if (output / "summary.json").exists():
        raise RuntimeError("use a fresh output directory to avoid stale evidence")

    environment = {
        **os.environ,
        "LIBGL_ALWAYS_SOFTWARE": "1",
        "QT_OPENGL": "software",
        "PYTHONPATH": "/usr/local/webots/lib/controller/python",
    }
    processes: list[subprocess.Popen[Any]] = []
    try:
        with (output / "webots.log").open("w") as log:
            webots = subprocess.Popen(
                [
                    "xvfb-run",
                    "-a",
                    "--server-args=-screen 0 1280x1024x24 +extension GLX +render",
                    "webots",
                    "--batch",
                    "--no-rendering",
                    "--mode=fast",
                    "--stdout",
                    "--stderr",
                    str(world.resolve()),
                ],
                env=environment,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            processes.append(webots)
            deadline = time.monotonic() + 30.0
            while WEBOTS_READY_MARKER not in (output / "webots.log").read_text():
                if webots.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("Webots failed to become ready; inspect webots.log")
                time.sleep(0.1)

            with (output / "controller.log").open("w") as controller_log:
                controller = subprocess.Popen(
                    [
                        "/usr/local/webots/webots-controller",
                        str(controller_path.resolve()),
                        "--controller",
                        "--output",
                        str(output.resolve()),
                    ],
                    env=environment,
                    stdout=controller_log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                processes.append(controller)
                return probe_exit_code(
                    controller.wait(timeout=150), output / "summary.json"
                )
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                with suppress(ProcessLookupError):
                    os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    with suppress(ProcessLookupError):
                        os.killpg(process.pid, signal.SIGKILL)
                    process.wait()


def probe_exit_code(controller_code: int, summary_path: Path) -> int:
    if not summary_path.exists():
        return controller_code
    text = summary_path.read_text(encoding="utf-8")
    summary = json.loads(text)
    print(text, end="")
    return 0 if summary.get("passed") is True else 1
