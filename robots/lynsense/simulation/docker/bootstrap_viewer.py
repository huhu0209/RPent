from __future__ import annotations

import argparse
import subprocess

import bootstrap


def _build_delegate_command(phase: str) -> str:
    if phase not in {"smoke", "blocked", "match", "box"}:
        raise ValueError(f"unsupported viewer phase: {phase}")
    return (
        "source /opt/ros/humble/setup.bash && "
        "source /workspace/ws/install/setup.bash && "
        f"exec lynsense_run_viewer --phase {phase}"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", required=True, choices=("smoke", "blocked", "match", "box"))
    args = parser.parse_args()
    bootstrap._build()
    return subprocess.run(["/bin/bash", "-lc", _build_delegate_command(args.phase)], check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
