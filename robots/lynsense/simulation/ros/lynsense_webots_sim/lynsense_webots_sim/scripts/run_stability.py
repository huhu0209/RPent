"""Isolated wheel-physics regression probe; no ROS or real-robot connection."""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from contextlib import suppress
from pathlib import Path


STAGES = (
    ("settle", 5.0, 0.0, 0.0),
    ("stationary", 4.0, 0.0, 0.0),
    ("straight", 5.0, 0.3, 0.0),
    ("stop_straight", 2.0, 0.0, 0.0),
    ("turn_left", 6.0, 0.0, 0.5),
    ("stop_left", 2.0, 0.0, 0.0),
    ("turn_right", 6.0, 0.0, -0.5),
    ("stop_right", 2.0, 0.0, 0.0),
)


def summarize(samples: list[dict]) -> dict:
    """Check attitude and actual motion, excluding only initial settling."""
    expected_phases = [
        name for name, duration, _, _ in STAGES for _ in range(round(duration / 0.032))
    ]
    if [row["phase"] for row in samples] != expected_phases:
        raise ValueError("incomplete or out-of-order stability phases")
    for index, row in enumerate(samples):
        if not all(math.isfinite(row[key]) for key in (
            "time_s", "x", "y", "z", "roll_deg", "pitch_deg", "yaw"
        )):
            raise ValueError("non-finite stability sample")
        if index and not math.isclose(
            row["time_s"] - samples[index - 1]["time_s"], 0.032, abs_tol=1e-8
        ):
            raise ValueError("invalid stability sampling interval")
    phases = {}
    for name, _, _, _ in STAGES[1:]:
        rows = [row for row in samples if row["phase"] == name]
        phases[name] = {
            "samples": len(rows),
            "max_abs_roll_deg": max(abs(row["roll_deg"]) for row in rows),
            "max_abs_pitch_deg": max(abs(row["pitch_deg"]) for row in rows),
            "height_range_m": max(row["z"] for row in rows)
            - min(row["z"] for row in rows),
            "distance_m": math.hypot(
                rows[-1]["x"] - rows[0]["x"], rows[-1]["y"] - rows[0]["y"]
            ),
            "yaw_change_rad": sum(
                math.atan2(math.sin(b["yaw"] - a["yaw"]), math.cos(b["yaw"] - a["yaw"]))
                for a, b in zip(rows, rows[1:])
            ),
        }
    stable = all(
        row["max_abs_roll_deg"] < 1.0
        and row["max_abs_pitch_deg"] < 1.0
        and row["height_range_m"] < 0.003
        for row in phases.values()
    )
    moving = (
        phases["straight"]["distance_m"] > 1.0
        and phases["turn_left"]["yaw_change_rad"] > 2.0
        and phases["turn_right"]["yaw_change_rad"] < -2.0
    )
    return {"passed": stable and moving, "stable": stable, "moving": moving, "phases": phases}


def run_controller(output: Path) -> int:
    from controller import Supervisor

    robot = Supervisor()
    body = robot.getSelf()
    motors = [robot.getDevice(name) for name in ("left_wheel_motor", "right_wheel_motor")]
    for motor in motors:
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
    samples = []
    linear = angular = 0.0
    try:
        for name, duration, target_linear, target_angular in STAGES:
            for _ in range(round(duration / 0.032)):
                linear += max(-0.5 * 0.032, min(0.5 * 0.032, target_linear - linear))
                angular += max(-0.032, min(0.032, target_angular - angular))
                motors[0].setVelocity((linear - angular * 0.461 / 2) / 0.08)
                motors[1].setVelocity((linear + angular * 0.461 / 2) / 0.08)
                if robot.step(32) == -1:
                    raise RuntimeError("Webots stopped before stability probe completed")
                x, y, z = body.getPosition()
                rotation = body.getOrientation()
                samples.append(
                    {
                        "phase": name,
                        "time_s": robot.getTime(),
                        "x": x, "y": y, "z": z,
                        "roll_deg": math.degrees(math.atan2(rotation[7], rotation[8])),
                        "pitch_deg": math.degrees(
                            math.atan2(-rotation[6], math.hypot(rotation[7], rotation[8]))
                        ),
                        "yaw": math.atan2(rotation[3], rotation[0]),
                    }
                )
    finally:
        for motor in motors:
            motor.setVelocity(0.0)
    output.mkdir(parents=True, exist_ok=True)
    (output / "samples.json").write_text(json.dumps(samples), encoding="utf-8")
    summary = summarize(samples)
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    robot.simulationQuit(0)
    return 0 if summary["passed"] else 1


def orchestrator_exit_code(summary_path: Path) -> int:
    if not summary_path.exists():
        raise RuntimeError("stability probe produced no summary; inspect controller.log")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    print(summary_path.read_text(encoding="utf-8"), end="")
    return 0 if summary.get("passed") is True else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--controller", action="store_true")
    args = parser.parse_args()
    if args.controller:
        return run_controller(args.output)
    if args.world is None or not args.world.is_file():
        parser.error("--world must be an existing Webots world")
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / "summary.json").exists():
        parser.error("use a fresh output directory to avoid stale evidence")
    env = {**os.environ, "LIBGL_ALWAYS_SOFTWARE": "1", "QT_OPENGL": "software"}
    env["PYTHONPATH"] = "/usr/local/webots/lib/controller/python"
    processes = []
    try:
        with (args.output / "webots.log").open("w") as log:
            webots = subprocess.Popen(
                [
                    "xvfb-run", "-a",
                    "--server-args=-screen 0 1280x1024x24 +extension GLX +render",
                    "webots", "--batch", "--no-rendering", "--mode=fast",
                    "--stdout", "--stderr", str(args.world.resolve()),
                ],
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            processes.append(webots)
            deadline = time.monotonic() + 30
            while "Waiting for local or remote connection" not in (
                args.output / "webots.log"
            ).read_text():
                if webots.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("Webots failed to become ready; inspect webots.log")
                time.sleep(0.1)
            with (args.output / "controller.log").open("w") as controller_log:
                controller = subprocess.Popen(
                    [
                        "/usr/local/webots/webots-controller", str(Path(__file__).resolve()),
                        "--controller", "--output", str(args.output.resolve()),
                    ],
                    env=env,
                    stdout=controller_log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                processes.append(controller)
                code = controller.wait(timeout=120)
            summary_path = args.output / "summary.json"
            return orchestrator_exit_code(summary_path)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                with suppress(ProcessLookupError):
                    os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    with suppress(ProcessLookupError):
                        os.killpg(process.pid, signal.SIGKILL)
                    process.wait()


if __name__ == "__main__":
    sys.exit(main())
