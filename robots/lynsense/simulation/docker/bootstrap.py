from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
from typing import Any


WS_ROOT = Path("/workspace/ws")
OVERLAY_ROOT = WS_ROOT / "install"
SOURCE_ROOT = Path("/workspace/simulation/ros")
PYTREES_ROOT = Path("/workspace/ws/src/lynsense_pytrees")
CONTROLLER_PATH = (
    OVERLAY_ROOT
    / "lynsense_webots_sim"
    / "bin"
    / "lynsense_webots_controller"
)
PYTREES_PREFIX = OVERLAY_ROOT / "lynsense_pytrees"
PYTREES_SHARE = PYTREES_PREFIX / "share" / "lynsense_pytrees"
PYTREES_CONFIG = PYTREES_SHARE / "config" / "move_points.yaml"
PYTREES_TREE = PYTREES_SHARE / "trees" / "lynsense_webots_tree.xml"
PYTREES_MATCH_TREE = PYTREES_SHARE / "trees" / "lynsense_match_min_tree.xml"
MATCH_TREE_SOURCE = (
    SOURCE_ROOT
    / "lynsense_webots_sim"
    / "trees"
    / "lynsense_match_min_tree.xml"
)
PYTREES_NODE = PYTREES_PREFIX / "lib" / "lynsense_pytrees" / "lynsense_pytrees_node.py"
SINGLE_BOX_TREE_EXECUTABLE = (
    OVERLAY_ROOT
    / "lynsense_webots_sim"
    / "bin"
    / "lynsense_single_box_tree"
)
RPENT_PLAN_RUNNER_EXECUTABLE = (
    OVERLAY_ROOT
    / "lynsense_webots_sim"
    / "bin"
    / "lynsense_run_rpent_plan"
)
SINGLE_BOX_TREE = (
    OVERLAY_ROOT
    / "lynsense_webots_sim"
    / "share"
    / "lynsense_webots_sim"
    / "trees"
    / "lynsense_single_box_tree.xml"
)
ROS_SOURCE = "source /opt/ros/humble/setup.bash"
PHASES = (
    "build",
    "interface",
    "smoke",
    "blocked",
    "match",
    "box",
    "rpent-box",
    "all",
    "box-all",
)
ROS_ENVIRONMENT_KEYS = (
    "ROS_DISTRO",
    "ROS_VERSION",
    "ROS_PYTHON_VERSION",
    "ROS_DOMAIN_ID",
    "ROS_LOCALHOST_ONLY",
    "AMENT_PREFIX_PATH",
    "ROS_PACKAGE_PATH",
)
_ENVIRONMENT_EVIDENCE_COMMAND = """python3 -c 'import json, os; print(json.dumps({
    "ROS_DISTRO": os.environ.get("ROS_DISTRO", ""),
    "ROS_VERSION": os.environ.get("ROS_VERSION", ""),
    "ROS_PYTHON_VERSION": os.environ.get("ROS_PYTHON_VERSION", ""),
    "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", ""),
    "ROS_LOCALHOST_ONLY": os.environ.get("ROS_LOCALHOST_ONLY", ""),
    "AMENT_PREFIX_PATH": os.environ.get("AMENT_PREFIX_PATH", ""),
    "ROS_PACKAGE_PATH": os.environ.get("ROS_PACKAGE_PATH", "")
}))'"""


def _bash(command: str) -> tuple[int, str]:
    result = subprocess.run(
        ["/bin/bash", "-lc", command],
        env=os.environ,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    if result.stdout:
        print(result.stdout, end="", flush=True)
    return result.returncode, result.stdout


def _check_bash(command: str, failure: str) -> None:
    return_code, _output = _bash(command)
    if return_code != 0:
        raise RuntimeError(f"{failure} failed with exit code {return_code}")


def _required_file(path: Path, description: str) -> None:
    if not path.is_file():
        raise RuntimeError(f"{description} is missing: {path}")


def _verify_controller() -> None:
    if not CONTROLLER_PATH.is_file() or not os.access(CONTROLLER_PATH, os.X_OK):
        raise RuntimeError(
            f"installed Webots controller entry point is not executable: {CONTROLLER_PATH}"
        )


def _ros_environment(source_command: str = "") -> dict[str, str]:
    command = (
        f"{source_command} && {_ENVIRONMENT_EVIDENCE_COMMAND}"
        if source_command
        else _ENVIRONMENT_EVIDENCE_COMMAND
    )
    return_code, output = _bash(command)
    if return_code != 0:
        raise RuntimeError(
            f"ROS environment evidence failed with exit code {return_code}"
        )
    try:
        environment = json.loads(output.strip())
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"ROS environment evidence was not JSON: {exc}") from exc
    if not isinstance(environment, dict) or any(
        not isinstance(environment.get(key), str) for key in ROS_ENVIRONMENT_KEYS
    ):
        raise RuntimeError("ROS environment evidence is incomplete")
    return environment


def _build_evidence(
    pre_source: dict[str, str], post_source: dict[str, str]
) -> dict[str, Any]:
    return {
        "controller": str(CONTROLLER_PATH),
        "controller_executable": True,
        "overlay": str(OVERLAY_ROOT),
        "packages": ["lynsense_utils", "lynsense_webots_sim", "lynsense_pytrees"],
        "result": "passed",
        "ros_environment": {
            key: post_source.get(key, "") for key in ROS_ENVIRONMENT_KEYS
        },
        "pre_source": {
            "AMENT_PREFIX_PATH": pre_source.get("AMENT_PREFIX_PATH", ""),
            "ROS_PACKAGE_PATH": pre_source.get("ROS_PACKAGE_PATH", ""),
        },
        "post_source": {
            "AMENT_PREFIX_PATH": post_source.get("AMENT_PREFIX_PATH", ""),
            "ROS_PACKAGE_PATH": post_source.get("ROS_PACKAGE_PATH", ""),
        },
    }


def _overlay_source_command(phase: str, plan_path: Path | None = None) -> str:
    if phase not in PHASES or phase == "build":
        raise ValueError(f"phase does not delegate: {phase}")
    if phase == "rpent-box" and plan_path is None:
        raise ValueError("rpent-box requires a plan path")
    if phase != "rpent-box" and plan_path is not None:
        raise ValueError("only rpent-box accepts a plan path")
    command = (
        "source /opt/ros/humble/setup.bash && "
        "source /workspace/ws/install/setup.bash && "
        f"exec lynsense_run_smoke --phase {shlex.quote(phase)}"
    )
    if plan_path is not None:
        command += f" --plan {shlex.quote(str(plan_path))}"
    return command


def _overlay_verification_command() -> str:
    return (
        f"{ROS_SOURCE} && source {OVERLAY_ROOT / 'setup.bash'} && "
        "cd /workspace/ws && python3 -c '"
        "import importlib, inspect, json, os\n"
        "from pathlib import Path\n"
        "print(json.dumps({\n"
        "\"lynsense_utils_file\": inspect.getfile(importlib.import_module(\"lynsense_utils\")),\n"
        "\"lynsense_webots_sim_file\": inspect.getfile(importlib.import_module(\"lynsense_webots_sim\")),\n"
        f"\"pytrees_share\": \"{PYTREES_SHARE}\",\n"
        f"\"pytrees_config\": \"{PYTREES_CONFIG}\",\n"
        f"\"pytrees_tree\": \"{PYTREES_TREE}\",\n"
        f"\"pytrees_match_tree\": \"{PYTREES_MATCH_TREE}\",\n"
        f"\"pytrees_node\": \"{PYTREES_NODE}\",\n"
        f"\"single_box_tree_executable\": \"{SINGLE_BOX_TREE_EXECUTABLE}\",\n"
        f"\"single_box_tree\": \"{SINGLE_BOX_TREE}\",\n"
        f"\"rpent_plan_runner_executable\": \"{RPENT_PLAN_RUNNER_EXECUTABLE}\",\n"
        f"\"pytrees_share_exists\": Path(\"{PYTREES_SHARE}\").is_dir(),\n"
        f"\"pytrees_config_exists\": Path(\"{PYTREES_CONFIG}\").is_file(),\n"
        f"\"pytrees_tree_exists\": Path(\"{PYTREES_TREE}\").is_file(),\n"
        f"\"pytrees_match_tree_exists\": Path(\"{PYTREES_MATCH_TREE}\").is_file(),\n"
        f"\"pytrees_node_exists\": Path(\"{PYTREES_NODE}\").is_file(),\n"
        f"\"pytrees_node_executable\": os.access(\"{PYTREES_NODE}\", os.X_OK)\n"
        f",\"single_box_tree_executable_exists\": Path(\"{SINGLE_BOX_TREE_EXECUTABLE}\").is_file()\n"
        f",\"single_box_tree_executable_is_executable\": os.access(\"{SINGLE_BOX_TREE_EXECUTABLE}\", os.X_OK)\n"
        f",\"single_box_tree_exists\": Path(\"{SINGLE_BOX_TREE}\").is_file()\n"
        f",\"rpent_plan_runner_executable_exists\": Path(\"{RPENT_PLAN_RUNNER_EXECUTABLE}\").is_file()\n"
        f",\"rpent_plan_runner_executable_is_executable\": os.access(\"{RPENT_PLAN_RUNNER_EXECUTABLE}\", os.X_OK)\n"
        "}))'"
    )


def _verify_overlay_with(evidence: dict[str, Any]) -> dict[str, Any]:
    expected = {
        "pytrees_share": str(PYTREES_SHARE),
        "pytrees_config": str(PYTREES_CONFIG),
        "pytrees_tree": str(PYTREES_TREE),
        "pytrees_match_tree": str(PYTREES_MATCH_TREE),
        "pytrees_node": str(PYTREES_NODE),
        "single_box_tree_executable": str(SINGLE_BOX_TREE_EXECUTABLE),
        "single_box_tree": str(SINGLE_BOX_TREE),
        "rpent_plan_runner_executable": str(RPENT_PLAN_RUNNER_EXECUTABLE),
    }
    required_boolean_fields = (
        "pytrees_share_exists",
        "pytrees_config_exists",
        "pytrees_tree_exists",
        "pytrees_match_tree_exists",
        "pytrees_node_exists",
        "pytrees_node_executable",
        "single_box_tree_executable_exists",
        "single_box_tree_executable_is_executable",
        "single_box_tree_exists",
        "rpent_plan_runner_executable_exists",
        "rpent_plan_runner_executable_is_executable",
    )
    valid = (
        isinstance(evidence.get("lynsense_utils_file"), str)
        and evidence["lynsense_utils_file"].startswith(
            f"{OVERLAY_ROOT / 'lynsense_utils'}/"
        )
        and isinstance(evidence.get("lynsense_webots_sim_file"), str)
        and evidence["lynsense_webots_sim_file"].startswith(
            f"{OVERLAY_ROOT / 'lynsense_webots_sim'}/"
        )
        and all(evidence.get(key) == value for key, value in expected.items())
        and all(evidence.get(key) is True for key in required_boolean_fields)
    )
    if not valid:
        raise RuntimeError("overlay verification failed")
    return evidence


def _verify_overlay() -> dict[str, Any]:
    return_code, output = _bash(_overlay_verification_command())
    if return_code != 0:
        raise RuntimeError(f"overlay verification failed with exit code {return_code}")
    try:
        evidence = json.loads(output.strip())
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"overlay verification was not JSON: {exc}") from exc
    return _verify_overlay_with(evidence)


def _build() -> None:
    pre_source = _ros_environment()
    _required_file(SOURCE_ROOT / "lynsense_utils_compat" / "package.xml", "simulation package")
    _required_file(SOURCE_ROOT / "lynsense_webots_sim" / "package.xml", "simulation package")
    _required_file(PYTREES_ROOT / "package.xml", "read-only lynsense_pytrees package")
    _required_file(MATCH_TREE_SOURCE, "match navigation tree")

    reset_command = (
        f"{ROS_SOURCE} && rm -rf "
        "/workspace/ws/build /workspace/ws/install /workspace/ws/log "
        "/workspace/ws/src/lynsense_utils /workspace/ws/src/lynsense_webots_sim && "
        "mkdir -p /workspace/ws/build /workspace/ws/install /workspace/ws/log /workspace/ws/src"
    )
    _check_bash(reset_command, "ephemeral workspace preparation")

    shutil.copytree(
        SOURCE_ROOT / "lynsense_utils_compat",
        WS_ROOT / "src" / "lynsense_utils",
        dirs_exist_ok=True,
    )
    shutil.copytree(
        SOURCE_ROOT / "lynsense_webots_sim",
        WS_ROOT / "src" / "lynsense_webots_sim",
        dirs_exist_ok=True,
    )

    build_command = (
        f"{ROS_SOURCE} && cd /workspace/ws && "
        "colcon build --packages-select lynsense_utils lynsense_webots_sim lynsense_pytrees"
    )
    _check_bash(build_command, "colcon build")

    shutil.copy2(
        MATCH_TREE_SOURCE,
        PYTREES_MATCH_TREE,
    )

    overlay_verification = _verify_overlay()

    _verify_controller()

    post_source = _ros_environment(
        f"{ROS_SOURCE} && source {OVERLAY_ROOT / 'setup.bash'}"
    )
    evidence = _build_evidence(pre_source, post_source)
    evidence["overlay_verification"] = overlay_verification
    artifact_root = Path("/workspace/artifacts")
    artifact_root.mkdir(parents=True, exist_ok=True)
    (artifact_root / "build-summary.json").write_text(
        json.dumps(evidence, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", required=True, choices=PHASES)
    parser.add_argument("--plan", type=Path)
    args = parser.parse_args()

    if args.phase == "rpent-box" and args.plan is None:
        parser.error("--phase rpent-box requires --plan")
    if args.phase != "rpent-box" and args.plan is not None:
        parser.error("--plan is only valid with --phase rpent-box")

    try:
        if args.phase == "build":
            _build()
            return 0
        _build()
        return _bash(_overlay_source_command(args.phase, args.plan))[0]
    except BaseException as exc:
        print(f"bootstrap failed: {type(exc).__name__}: {exc}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
