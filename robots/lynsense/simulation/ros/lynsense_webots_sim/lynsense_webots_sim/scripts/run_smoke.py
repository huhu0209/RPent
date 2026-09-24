from __future__ import annotations

import argparse
import datetime as dt
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Any
import uuid


OVERLAY_ROOT = Path("/workspace/ws/install")
CONTROLLER_PATH = (
    OVERLAY_ROOT / "lynsense_webots_sim" / "bin" / "lynsense_webots_controller"
)
DEFAULT_WEBOTS_HOME = Path("/usr/local/webots")
WEBOTS_PYTHON_API_PATH = Path("/usr/local/webots/lib/controller/python")
PYTREES_NODE_PATH = (
    OVERLAY_ROOT
    / "lynsense_pytrees"
    / "lib"
    / "lynsense_pytrees"
    / "lynsense_pytrees_node.py"
)
SINGLE_BOX_TREE_EXECUTABLE = (
    OVERLAY_ROOT
    / "lynsense_webots_sim"
    / "bin"
    / "lynsense_single_box_tree"
)
SINGLE_BOX_TREE_PATH = (
    OVERLAY_ROOT
    / "lynsense_webots_sim"
    / "share"
    / "lynsense_webots_sim"
    / "trees"
    / "lynsense_single_box_tree.xml"
)
RPENT_PLAN_RUNNER_EXECUTABLE = (
    OVERLAY_ROOT
    / "lynsense_webots_sim"
    / "bin"
    / "lynsense_run_rpent_plan"
)
CONFIG_ROOT = Path("/workspace/simulation/ros/lynsense_webots_sim/config")
CONFIG_PATHS = {
    "interface": CONFIG_ROOT / "smoke.yaml",
    "smoke": CONFIG_ROOT / "smoke.yaml",
    "blocked": CONFIG_ROOT / "smoke.yaml",
    "match": CONFIG_ROOT / "match.yaml",
    "box": CONFIG_ROOT / "box.yaml",
    "rpent-box": CONFIG_ROOT / "box.yaml",
    "box_navigation": CONFIG_ROOT / "box_navigation.yaml",
    "rpent-box_navigation": CONFIG_ROOT / "box_navigation.yaml",
}
SMOKE_WORLD = Path(
    "/workspace/simulation/ros/lynsense_webots_sim/worlds/lynsense_smoke.wbt"
)
BLOCKED_WORLD = Path(
    "/workspace/simulation/ros/lynsense_webots_sim/worlds/lynsense_blocked.wbt"
)
WORLD_PATHS = {
    "interface": SMOKE_WORLD,
    "smoke": SMOKE_WORLD,
    "blocked": BLOCKED_WORLD,
    "match": Path(
        "/workspace/simulation/ros/lynsense_webots_sim/worlds/lynsense_match.wbt"
    ),
    "box": Path(
        "/workspace/simulation/ros/lynsense_webots_sim/worlds/lynsense_box.wbt"
    ),
    "rpent-box": Path(
        "/workspace/simulation/ros/lynsense_webots_sim/worlds/lynsense_box.wbt"
    ),
}
TREE_NAMES = {
    "smoke": "lynsense_webots_tree.xml",
    "blocked": "lynsense_webots_tree.xml",
    "match": "lynsense_match_min_tree.xml",
    "box": "lynsense_single_box_tree.xml",
}
ROBOT_NAMES = {
    "interface": "EA200_SMOKE",
    "smoke": "EA200_SMOKE",
    "blocked": "EA200_SMOKE",
    "match": "EA200_SMOKE",
    "box": "EA200_BOX",
    "rpent-box": "EA200_BOX",
}
ARTIFACT_ROOT = Path("/workspace/artifacts")
EVENT_NAMES = {
    "interface": "events-interface.jsonl",
    "smoke": "events-smoke.jsonl",
    "blocked": "events-blocked.jsonl",
    "match": "events-match.jsonl",
    "box": "events-box.jsonl",
    "rpent-box": "events-rpent-box.jsonl",
}
RPENT_BOX_PHASE = "rpent-box"
BOX_VALIDATED_PHASES = {"box", RPENT_BOX_PHASE}
PHASE_SEQUENCE = {
    "all": ("interface", "smoke", "blocked", "match"),
    "box-all": ("interface", "smoke", "blocked", "match", "box"),
}
SMOKE_FINAL_POSE = (0.5, 0.0, 0.0)
MATCH_FINAL_POSE = (
    2.460558853149414,
    -2.6341702938073354,
    0.006126105582217833,
)
BOX_FINAL_POSE = (
    1.8606561495236394,
    -2.742422444761675,
    0.006126105582217833,
)
SMOKE_EXPECTED_SCRIPT = (
    ("nav_to_pose", "sim_goal1", None),
    ("move_distance", None, (-1.5, 0.0)),
    ("move_distance", None, (1.2, 0.0)),
    ("move_distance", None, (-1.2, 0.0)),
    ("nav_to_pose", "sim_goal2", None),
)
MATCH_EXPECTED_SCRIPT = (
    ("nav_to_pose", "搬箱子1", None),
    ("move_distance", None, (-0.6, 0.0)),
    ("move_distance", None, (0.0, 90.0)),
    ("nav_to_pose", "放箱子1_1", None),
)
BOX_EXPECTED_SCRIPT = (
    ("nav_to_pose", {"goal_name": "搬箱子1"}),
    ("move_waist", {"height_mm": 200.0}),
    ("set_gripper", {"position": 0.0}),
    ("move_named_config", {"target": "dualjo:joints_br"}),
    ("box_phase", {"action": "pick", "flow": "flow", "config": "box1"}),
    ("move_distance", {"distance": -0.6, "angle": 0.0}),
    ("move_distance", {"distance": 0.0, "angle": 90.0}),
    ("move_named_config", {"target": "dualposi_armbase_abso:pt_1f1_ready"}),
    ("nav_to_pose", {"goal_name": "放箱子1_1"}),
    ("box_phase", {"action": "place", "flow": "flow", "config": "box1"}),
    ("move_distance", {"distance": -0.6, "angle": 0.0}),
    ("move_named_config", {"target": "dualposi_armbase_abso:pt_up"}),
    ("move_waist", {"height_mm": 200.0}),
    ("set_gripper", {"position": 0.0}),
    ("move_named_config", {"target": "dualjo:joints_s"}),
)
EXPECTED_SCRIPTS = {
    "smoke": SMOKE_EXPECTED_SCRIPT,
    "blocked": SMOKE_EXPECTED_SCRIPT,
    "match": MATCH_EXPECTED_SCRIPT,
    "box": BOX_EXPECTED_SCRIPT,
    "rpent-box": BOX_EXPECTED_SCRIPT,
}
FINAL_POSES = {
    "smoke": SMOKE_FINAL_POSE,
    "blocked": SMOKE_FINAL_POSE,
    "match": MATCH_FINAL_POSE,
    "box": BOX_FINAL_POSE,
    "rpent-box": BOX_FINAL_POSE,
}
EXPECTED_ACTION_TYPES = {
    "nav_to_pose": "lynsense_utils/action/NavToPose",
    "move_distance": "lynsense_utils/action/MoveDistance",
}
MANIPULATION_ACTION_TYPES = {
    "move_waist": "lynsense_utils/action/MoveWaist",
    "move_named_config": "lynsense_utils/action/MoveNamedConfig",
    "set_gripper": "lynsense_utils/action/SetGripper",
    "box_phase": "lynsense_utils/action/BoxPhase",
}
PHASE_ACTION_TYPES = {
    "interface": EXPECTED_ACTION_TYPES,
    "smoke": EXPECTED_ACTION_TYPES,
    "blocked": EXPECTED_ACTION_TYPES,
    "match": EXPECTED_ACTION_TYPES,
    "box": {
        **EXPECTED_ACTION_TYPES,
        **MANIPULATION_ACTION_TYPES,
    },
    "rpent-box": {
        **EXPECTED_ACTION_TYPES,
        **MANIPULATION_ACTION_TYPES,
    },
}
EXPECTED_ACTION_ENDPOINTS = tuple(
    f"/lynsense/{name}" for name in EXPECTED_ACTION_TYPES
)
ROS_SOURCE = "source /opt/ros/humble/setup.bash"
OVERLAY_SOURCE = f"{ROS_SOURCE} && source {OVERLAY_ROOT / 'setup.bash'}"
RUNTIME_IMPORTS = ("rclpy", "py_trees", "py_trees_ros", "yaml")
FORBIDDEN_RUNTIME_PATHS = (
    "/home/rpp",
    "rpp_ws",
    "robot_ws",
    "robot_workspace",
)
WEBOTS_READY_MARKER = "Waiting for local or remote connection"
WEBOTS_READY_BUDGET_S = 15.0
ACTION_GRAPH_READY_BUDGET_S = 15.0
ExpectedActionSpec = tuple[str, str | None, tuple[float, float] | None]
ActionScript = tuple[ExpectedActionSpec, ...]
BoxActionSpec = tuple[str, dict[str, Any]]
BoxActionScript = tuple[BoxActionSpec, ...]
ExpectedScript = ActionScript | BoxActionScript
FinalPose = tuple[float, float, float]


@dataclass
class PhaseProcesses:
    webots: subprocess.Popen[Any] | None = None
    controller: subprocess.Popen[Any] | None = None
    tree: subprocess.Popen[Any] | None = None

    def all_alive(self) -> bool:
        return (
            self.webots is not None
            and self.controller is not None
            and self.webots.poll() is None
            and self.controller.poll() is None
            and (self.tree is None or self.tree.poll() is None)
        )


@dataclass(frozen=True)
class EventCursor:
    path: Path
    offset: int
    run_id: str


class SmokeError(RuntimeError):
    pass


class TreeEventValidator:
    def __init__(
        self,
        *,
        blocked: bool,
        expected_script: ExpectedScript | None = None,
        final_pose: FinalPose | None = None,
        phase: str = "smoke",
    ) -> None:
        self.blocked = blocked
        self.expected_script = expected_script or SMOKE_EXPECTED_SCRIPT
        self.final_pose = final_pose or SMOKE_FINAL_POSE
        self.phase = phase
        self.sequence_index = 0
        self.accepted_index: int | None = None
        self.release_time_s: float | None = None
        self.terminal = False

    def feed(self, event: dict[str, Any]) -> bool:
        kind = event.get("kind")
        if self.terminal:
            if kind == "goal_rejected":
                if event.get("reason") != "terminal_lockout":
                    raise SmokeError("post-terminal rejection was not terminal_lockout")
                return True
            if kind == "action_feedback":
                if not _zero_rates(event):
                    raise SmokeError("new motor command occurred after terminal result")
                return True
            if kind in {"goal_accepted", "action_result"}:
                raise SmokeError("post-terminal action sequence was not allowed")
            raise SmokeError(f"unexpected post-terminal event: {kind}")

        if kind == "action_feedback":
            _validate_feedback(event)
            return False
        if kind in {"run_started", "controller_ready"}:
            raise SmokeError(f"readiness event restarted during tree execution: {kind}")
        if kind == "goal_accepted":
            if self.sequence_index >= len(self.expected_script):
                raise SmokeError("goal_accepted arrived after the exact script finished")
            if not _action_matches(event, self.expected_script[self.sequence_index]):
                raise SmokeError("goal_accepted is out of order or unexpected")
            self.accepted_index = self.sequence_index
            self.sequence_index += 1
            return False
        if kind == "goal_rejected":
            raise SmokeError("goal was rejected before the terminal result")
        if kind != "action_result":
            raise SmokeError(f"unexpected event during tree execution: {kind}")

        if self.accepted_index is None:
            raise SmokeError("action_result arrived without an accepted goal")
        expected_index = self.accepted_index
        self.accepted_index = None
        if not _action_matches(event, self.expected_script[expected_index]):
            raise SmokeError(
                "action_result is out of order or unexpected: "
                f"expected index {expected_index} "
                f"{self.expected_script[expected_index]!r}, got "
                f"{(event.get('action'), event.get('goal_name'), event.get('distance_m'), event.get('angle_deg'))!r}"
            )
        success = event.get("success") is True
        if self.blocked and success:
            raise SmokeError("blocked phase did not fail as required")
        if self.blocked and event.get("reason") != "translate_stalled":
            raise SmokeError("blocked phase failed without the expected translate stall")
        if not self.blocked and not success:
            raise SmokeError("action_result is out of order or failed")
        if not _zero_rates(event):
            raise SmokeError("terminal result did not have zero motor rates")

        if self.phase in BOX_VALIDATED_PHASES:
            box = event.get("box")
            if event.get("action") == "box_phase" and event.get("box_action") == "pick":
                if not isinstance(box, dict) or box.get("attached") is not True:
                    raise SmokeError("pick did not attach the box")
            if event.get("action") == "box_phase" and event.get("box_action") == "place":
                if not isinstance(box, dict) or box.get("attached") is not False:
                    raise SmokeError("place did not release the box")
                self.release_time_s = float(event["sim_time_s"])

        if self.blocked:
            self.terminal = True
            return True
        if self.sequence_index != len(self.expected_script):
            return False
        pose = event.get("pose")
        if not _finite_pose(pose):
            raise SmokeError("terminal result pose is invalid")
        if (
            not math.isclose(float(pose["x"]), self.final_pose[0], abs_tol=0.051)
            or abs(float(pose["y"]) - self.final_pose[1]) > 0.051
            or abs(float(pose["yaw"]) - self.final_pose[2]) > 0.03590658503988659
        ):
            raise SmokeError(
                "terminal pose is not the expected final phase pose "
                f"{self.final_pose}"
            )
        if self.phase in BOX_VALIDATED_PHASES:
            if self.release_time_s is None:
                raise SmokeError("box terminal result has no release time")
            if float(event["sim_time_s"]) - self.release_time_s < 0.5:
                raise SmokeError("post-release evidence is shorter than 0.5 seconds")
            if not _box_at_place_target(event.get("box")):
                raise SmokeError("final box pose is outside place tolerances")
        self.terminal = True
        return True


def _run_id() -> str:
    timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"{timestamp}-{uuid.uuid4()}"


def _load_rpent_plan(plan_path: Path) -> tuple[dict[str, Any], ...]:
    from lynsense_webots_sim.scripts.run_rpent_plan import _plan_contract

    load_plan = _plan_contract("load_plan")
    plan_error = _plan_contract("PlanError")
    try:
        return load_plan(plan_path)
    except plan_error as exc:
        raise SmokeError(f"invalid Lynsense RPent plan: {exc}") from exc


def _prepare_event(phase: str) -> tuple[Path, str]:
    phase_file = ARTIFACT_ROOT / EVENT_NAMES[phase]
    phase_file.parent.mkdir(parents=True, exist_ok=True)
    if phase_file.exists():
        old_mtime = max(0, int(phase_file.stat().st_mtime))
        archive = ARTIFACT_ROOT / "archive"
        archive.mkdir(parents=True, exist_ok=True)
        phase_file.rename(
            archive / f"{phase}-{old_mtime}-{uuid.uuid4()}.jsonl"
        )
    phase_file.touch(mode=0o600)
    return phase_file, _run_id()


def _command_environment(
    phase: str,
    event_path: Path,
    run_id: str,
    *,
    plan_path: Path | None = None,
) -> dict[str, str]:
    environment = os.environ.copy()
    environment.pop("LYNSENSE_VIEWER_CAPTURE_DIR", None)
    environment.update(
        {
            "LYNSENSE_SIM_CONFIG": str(CONFIG_PATHS[phase]),
            "LYNSENSE_SIM_EVENT_FILE": str(event_path),
            "LYNSENSE_SIM_SCRIPT": (
                "box" if phase == RPENT_BOX_PHASE else phase
            ),
            "LYNSENSE_SIM_RUN_ID": run_id,
            "WEBOTS_WORLD": str(WORLD_PATHS[phase]),
        }
    )
    if phase in {"box", RPENT_BOX_PHASE}:
        environment["LYNSENSE_SIM_NAV_CONFIG"] = str(
            CONFIG_PATHS["box_navigation"]
        )
    if phase == RPENT_BOX_PHASE:
        if plan_path is None:
            raise SmokeError("rpent-box requires --plan")
        environment["LYNSENSE_RPENT_PLAN"] = str(plan_path)
    return environment


def _check_file(path: Path, description: str) -> None:
    if not path.is_file():
        raise SmokeError(f"{description} is missing: {path}")


def _verify_controller() -> None:
    _check_file(CONTROLLER_PATH, "installed Webots controller")
    if not os.access(CONTROLLER_PATH, os.X_OK):
        raise SmokeError("installed Webots controller is not executable")


def _software_rendering_environment(environment: dict[str, str]) -> dict[str, str]:
    return {
        **environment,
        "LIBGL_ALWAYS_SOFTWARE": "1",
        "QT_OPENGL": "software",
    }


def _wait_for_webots_ready(
    process: subprocess.Popen[Any], log_path: Path
) -> None:
    deadline = time.monotonic() + WEBOTS_READY_BUDGET_S
    while True:
        exit_code = process.poll()
        if exit_code is not None:
            raise SmokeError(
                f"Webots exited before the ready marker with code {exit_code}"
            )
        try:
            if WEBOTS_READY_MARKER in log_path.read_text(
                encoding="utf-8", errors="replace"
            ):
                return
        except FileNotFoundError:
            pass
        if time.monotonic() >= deadline:
            raise SmokeError("Webots ready marker exceeded 15 seconds")
        time.sleep(0.1)


def _start_webots(phase: str, environment: dict[str, str], run_id: str) -> subprocess.Popen[Any]:
    world = Path(environment["WEBOTS_WORLD"])
    _check_file(world, f"{phase} Webots world")
    ARTIFACT_ROOT.mkdir(parents=True, exist_ok=True)
    stdout_path = ARTIFACT_ROOT / f"webots-{phase}-{run_id}.log"
    stderr_path = ARTIFACT_ROOT / f"webots-{phase}-{run_id}.err"
    webots_environment = _software_rendering_environment(environment)
    with stdout_path.open("ab") as stdout, stderr_path.open("ab") as stderr:
        process = subprocess.Popen(
            [
                "xvfb-run",
                "-a",
                "--server-args=-screen 0 1280x1024x24 +extension GLX +render",
                "webots",
                "--batch",
                "--no-rendering",
                "--mode=run",
                "--stdout",
                "--stderr",
                str(world),
            ],
            env=webots_environment,
            stdin=subprocess.DEVNULL,
            stdout=stdout,
            stderr=stderr,
            start_new_session=True,
        )
    try:
        if process.poll() is not None:
            raise SmokeError(f"Webots exited immediately with code {process.returncode}")
        _wait_for_webots_ready(process, stdout_path)
    except BaseException:
        _terminate(process, 8.0)
        raise
    return process


def _start_controller(
    environment: dict[str, str], phase: str, run_id: str, log_path: Path | None = None
) -> subprocess.Popen[Any]:
    _verify_controller()
    webots_home = Path(environment.get("WEBOTS_HOME") or DEFAULT_WEBOTS_HOME)
    launcher = webots_home / "webots-controller"
    if not launcher.is_file() or not os.access(launcher, os.X_OK):
        raise SmokeError(f"Webots controller launcher is not executable: {launcher}")
    stdout_path = log_path or ARTIFACT_ROOT / f"controller-{phase}-{run_id}.log"
    pythonpath_entries = [str(WEBOTS_PYTHON_API_PATH)]
    if environment.get("PYTHONPATH"):
        pythonpath_entries.append(environment["PYTHONPATH"])
    controller_environment = _software_rendering_environment(
        {
            **environment,
            "PYTHONPATH": os.pathsep.join(pythonpath_entries),
        }
    )
    with stdout_path.open("ab") as stdout:
        process = subprocess.Popen(
            [
                os.fspath(launcher),
                f"--robot-name={ROBOT_NAMES[phase]}",
                os.fspath(CONTROLLER_PATH),
            ],
            env=controller_environment,
            stdin=subprocess.DEVNULL,
            stdout=stdout,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    try:
        if process.poll() is not None:
            raise SmokeError(f"controller exited immediately with code {process.returncode}")
    except BaseException:
        _terminate(process, 8.0)
        raise
    return process


def _validate_event(event: Any, run_id: str) -> dict[str, Any]:
    if not isinstance(event, dict):
        raise SmokeError("JSONL event is not an object")
    if event.get("run_id") != run_id:
        raise SmokeError("JSONL event has a missing or different run_id")
    if not isinstance(event.get("kind"), str) or not event["kind"]:
        raise SmokeError("JSONL event has no kind")
    return event


def _finite_pose(pose: Any) -> bool:
    return (
        isinstance(pose, dict)
        and all(name in pose for name in ("x", "y", "yaw"))
        and all(
            isinstance(pose[name], (int, float))
            and math.isfinite(float(pose[name]))
            for name in ("x", "y", "yaw")
        )
    )


def _zero_rates(event: dict[str, Any]) -> bool:
    rates = event.get("motor_rates")
    wheels_zero = (
        isinstance(rates, dict)
        and float(rates.get("left_wheel_rate_radps", math.nan)) == 0.0
        and float(rates.get("right_wheel_rate_radps", math.nan)) == 0.0
    )
    manipulation = event.get("manipulation_rates")
    if manipulation is None:
        return wheels_zero
    return (
        wheels_zero
        and isinstance(manipulation, dict)
        and float(manipulation.get("waist_velocity_mps", math.nan)) == 0.0
        and _all_zero(manipulation.get("arm_velocities_radps"))
        and _all_zero(manipulation.get("gripper_velocities"))
    )


def _all_zero(values: Any) -> bool:
    return (
        isinstance(values, (list, tuple))
        and bool(values)
        and all(isinstance(value, (int, float)) and float(value) == 0.0 for value in values)
    )


def _box_at_place_target(box: Any) -> bool:
    if not isinstance(box, dict):
        return False
    position = box.get("position_m")
    orientation = box.get("orientation_rad")
    if not isinstance(position, (list, tuple)) or len(position) != 3:
        return False
    if not isinstance(orientation, (list, tuple)) or len(orientation) != 3:
        return False
    target = (2.810559, -2.734170, 0.1085)
    return (
        all(math.isfinite(float(value)) for value in (*position, *orientation))
        and abs(float(position[0]) - target[0]) <= 0.051
        and abs(float(position[1]) - target[1]) <= 0.051
        and abs(float(position[2]) - target[2]) <= 0.051
        and abs(_angle_delta(float(orientation[2]), 0.006126)) <= 0.174533
    )


def _angle_delta(value: float, reference: float) -> float:
    return (value - reference + math.pi) % (2.0 * math.pi) - math.pi


def _read_new_events(cursor: EventCursor) -> tuple[list[dict[str, Any]], EventCursor]:
    data = cursor.path.read_bytes()
    if len(data) < cursor.offset:
        raise SmokeError("phase event file was truncated")
    new_data = data[cursor.offset:]
    if not new_data:
        return [], cursor
    if not new_data.endswith(b"\n"):
        raise SmokeError("JSONL event has a missing newline")
    try:
        events = [json.loads(line) for line in new_data.decode("utf-8").splitlines()]
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise SmokeError(f"invalid JSONL event: {exc}") from exc
    return events, EventCursor(cursor.path, len(data), cursor.run_id)


def _wait_for_ready(
    processes: PhaseProcesses, event_path: Path, run_id: str, phase: str = "smoke"
) -> EventCursor:
    deadline = time.monotonic() + 15.0
    cursor = EventCursor(event_path, 0, run_id)
    seen_run_started = False
    ready = False
    while not ready:
        if not processes.webots.poll() is None:
            raise SmokeError("Webots exited before controller readiness")
        if processes.controller.poll() is not None:
            raise SmokeError("controller exited before controller readiness")
        if not event_path.is_file():
            raise SmokeError("phase event file disappeared before controller readiness")
        if time.monotonic() >= deadline:
            raise SmokeError("controller readiness exceeded 15 seconds")

        raw_events, cursor = _read_new_events(cursor)
        for raw_event in raw_events:
            event = _validate_event(raw_event, run_id)
            kind = event["kind"]
            if not seen_run_started:
                if kind != "run_started":
                    raise SmokeError("first controller event is not run_started")
                seen_run_started = True
                continue
            if not ready and kind != "controller_ready":
                raise SmokeError("controller_ready did not follow run_started")
            if not ready:
                action_names = event.get("action_names")
                expected_action_types = PHASE_ACTION_TYPES[phase]
                if action_names != list(expected_action_types):
                    raise SmokeError("controller_ready does not contain the expected endpoints")
                if event.get("action_types") != expected_action_types:
                    raise SmokeError("controller_ready has incorrect action types")
                if not _finite_pose(event.get("pose")) or not _zero_rates(event):
                    raise SmokeError("controller_ready pose or initial motor evidence is invalid")
                ready = True
        if not ready:
            time.sleep(0.1)

    time.sleep(1.0)
    if not processes.all_alive():
        raise SmokeError("Webots or controller exited during readiness settle")
    raw_events, cursor = _read_new_events(cursor)
    for raw_event in raw_events:
        _validate_event(raw_event, run_id)
    return cursor


def _wait_for_action_graph(
    processes: PhaseProcesses, environment: dict[str, str], phase: str = "smoke"
) -> None:
    deadline = time.monotonic() + ACTION_GRAPH_READY_BUDGET_S
    while True:
        if not processes.all_alive():
            raise SmokeError(
                "Webots or controller exited before ROS action discovery"
            )
        try:
            completed = subprocess.run(
                ["ros2", "action", "list"],
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=2.0,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            completed = None
        if completed is not None and completed.returncode == 0:
            actions = set(completed.stdout.split())
            expected_endpoints = set(
                f"/lynsense/{name}" for name in PHASE_ACTION_TYPES[phase]
            )
            if expected_endpoints <= actions:
                return
        if time.monotonic() >= deadline:
            raise SmokeError("ROS action endpoints did not become discoverable")
        time.sleep(0.25)


def _action_matches(event: dict[str, Any], expected: ExpectedActionSpec | BoxActionSpec) -> bool:
    if len(expected) == 2:
        action, fields = expected
        if event.get("action") != action:
            return False
        event_fields = {
            "action": event.get("box_action"),
            "goal_name": event.get("goal_name"),
            "height_mm": event.get("height_mm"),
            "target": event.get("target"),
            "gripper_position": event.get("gripper_position"),
            "box_action": event.get("box_action"),
            "box_flow": event.get("box_flow"),
            "box_config": event.get("box_config"),
            "distance": event.get("distance_m"),
            "angle": event.get("angle_deg"),
            "flow": event.get("box_flow"),
            "config": event.get("box_config"),
            "position": event.get("gripper_position"),
        }
        return all(event_fields.get(name) == value for name, value in fields.items())
    if event.get("action") != expected[0]:
        return False
    if expected[0] == "nav_to_pose":
        return event.get("goal_name") == expected[1]
    distance, angle = expected[2] or (math.nan, math.nan)
    return (
        float(event.get("distance_m", math.nan)) == distance
        and float(event.get("angle_deg", math.nan)) == angle
    )


def _validate_feedback(event: dict[str, Any]) -> None:
    if event["kind"] != "action_feedback":
        return
    required = {"sim_time_s", "action", "pose", "motor_rates"}
    if not required <= event.keys():
        raise SmokeError("action_feedback is missing required fields")
    if not _finite_pose(event.get("pose")):
        raise SmokeError("action_feedback has an invalid pose")


def _watch_tree(
    tree: subprocess.Popen[Any],
    processes: PhaseProcesses,
    cursor: EventCursor,
    *,
    blocked: bool,
    expected_script: ExpectedScript | None = None,
    final_pose: FinalPose | None = None,
    phase: str = "smoke",
) -> None:
    deadline_s = (
        180.0 if phase in BOX_VALIDATED_PHASES else (30.0 if blocked else 120.0)
    )
    deadline = time.monotonic() + deadline_s
    terminal = False
    validator = TreeEventValidator(
        blocked=blocked,
        expected_script=expected_script,
        final_pose=final_pose,
        phase=phase,
    )
    while True:
        raw_events, cursor = _read_new_events(cursor)
        for raw_event in raw_events:
            event = _validate_event(raw_event, cursor.run_id)
            if validator.feed(event):
                terminal = True
                if tree.poll() is None:
                    _signal_process_group(tree, signal.SIGINT)
        if terminal:
            if tree.poll() is not None:
                return
            if time.monotonic() >= deadline:
                raise SmokeError("tree did not exit after terminal event deadline")
            time.sleep(0.1)
            continue

        if not processes.all_alive():
            raise SmokeError("Webots, controller, or tree exited before the terminal event")
        if time.monotonic() >= deadline:
            raise SmokeError("tree deadline expired")
        time.sleep(0.1)


def _signal_process_group(process: subprocess.Popen[Any], signal_value: int) -> None:
    try:
        os.killpg(process.pid, signal_value)
    except ProcessLookupError:
        pass


def _terminate(process: subprocess.Popen[Any], timeout: float) -> None:
    _signal_process_group(process, signal.SIGINT)
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        _signal_process_group(process, signal.SIGTERM)
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            _signal_process_group(process, signal.SIGKILL)
            process.wait(timeout=2.0)


def _cleanup(processes: PhaseProcesses) -> None:
    ordered = [
        process for process in (processes.tree, processes.controller, processes.webots)
        if process is not None
    ]
    for process in ordered:
        _terminate(process, 5.0 if process is processes.tree else 8.0)


def _run_probe(
    environment: dict[str, str], phase: str, run_id: str, mode: str
) -> None:
    log_path = ARTIFACT_ROOT / f"probe-{mode}-{run_id}.log"
    with log_path.open("wb") as log:
        probe = subprocess.Popen(
            ["lynsense_action_probe", "--mode", mode],
            env=environment,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            if probe.wait(timeout=60.0) != 0:
                raise SmokeError(f"action probe failed with exit code {probe.returncode}")
        except subprocess.TimeoutExpired as exc:
            _signal_process_group(probe, signal.SIGKILL)
            probe.wait()
            raise SmokeError("action probe exceeded 60 seconds") from exc


def _runtime_isolation_evidence(
    environment: dict[str, str], package_prefix: str, imports: dict[str, bool]
) -> dict[str, Any]:
    values = [
        environment.get("AMENT_PREFIX_PATH", ""),
        environment.get("ROS_PACKAGE_PATH", ""),
        environment.get("PATH", ""),
        environment.get("PYTHONPATH", ""),
        environment.get("LD_LIBRARY_PATH", ""),
        environment.get("CMAKE_PREFIX_PATH", ""),
        package_prefix,
    ]
    return {
        "ROS_DOMAIN_ID": environment.get("ROS_DOMAIN_ID", ""),
        "ROS_LOCALHOST_ONLY": environment.get("ROS_LOCALHOST_ONLY", ""),
        "overlay_prefix": str(OVERLAY_ROOT),
        "ament_prefix_path": environment.get("AMENT_PREFIX_PATH", ""),
        "ros_package_path": environment.get("ROS_PACKAGE_PATH", ""),
        "ros2_pkg_prefix_lynsense_utils": package_prefix,
        "imports": {name: bool(imports.get(name, False)) for name in RUNTIME_IMPORTS},
        "forbidden_paths": [
            path for path in FORBIDDEN_RUNTIME_PATHS
            if any(path in value for value in values)
        ],
    }


def _validate_runtime_isolation(evidence: dict[str, Any]) -> None:
    if evidence.get("forbidden_paths"):
        raise SmokeError(
            "robot workspace path appeared in runtime isolation evidence: "
            + ", ".join(evidence["forbidden_paths"])
        )
    if evidence.get("ROS_DOMAIN_ID") != "42":
        raise SmokeError("runtime isolation requires ROS_DOMAIN_ID=42")
    if evidence.get("ROS_LOCALHOST_ONLY") != "1":
        raise SmokeError("runtime isolation requires ROS_LOCALHOST_ONLY=1")
    if str(OVERLAY_ROOT) not in evidence.get("ament_prefix_path", ""):
        raise SmokeError("runtime isolation is not using the ephemeral overlay")
    if evidence.get("ros2_pkg_prefix_lynsense_utils") != str(
        OVERLAY_ROOT / "lynsense_utils"
    ):
        raise SmokeError("lynsense_utils package prefix is not from the ephemeral overlay")
    if evidence.get("imports") != {name: True for name in RUNTIME_IMPORTS}:
        raise SmokeError("required runtime Python packages are unavailable")


def _bash(command: str) -> tuple[int, str]:
    try:
        result = subprocess.run(
            ["/bin/bash", "-lc", command],
            env=os.environ,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
            timeout=10.0,
        )
    except subprocess.TimeoutExpired:
        return 124, ""
    return result.returncode, result.stdout


def _collect_runtime_isolation_evidence() -> dict[str, Any]:
    selected_keys = (
        "ROS_DOMAIN_ID",
        "ROS_LOCALHOST_ONLY",
        "AMENT_PREFIX_PATH",
        "ROS_PACKAGE_PATH",
        "PATH",
        "PYTHONPATH",
        "LD_LIBRARY_PATH",
        "CMAKE_PREFIX_PATH",
    )
    environment = {key: os.environ.get(key, "") for key in selected_keys}
    prefix_command = (
        f"{OVERLAY_SOURCE} && ros2 pkg prefix lynsense_utils"
    )
    return_code, package_prefix = _bash(prefix_command)
    if return_code != 0:
        raise SmokeError(
            f"ros2 pkg prefix lynsense_utils failed with exit code {return_code}"
        )

    import_names = ", ".join(f'"{name}"' for name in RUNTIME_IMPORTS)
    import_command = (
        f"{OVERLAY_SOURCE} && python3 -c "
        f"'import importlib, json; print(json.dumps({{"
        f"name: bool(importlib.import_module(name)) "
        f"for name in ({import_names})}}))'"
    )
    return_code, import_output = _bash(import_command)
    if return_code != 0:
        raise SmokeError(f"runtime import evidence failed with exit code {return_code}")
    try:
        imports = json.loads(import_output.strip())
    except json.JSONDecodeError as exc:
        raise SmokeError(f"runtime import evidence was not JSON: {exc}") from exc
    return _runtime_isolation_evidence(
        environment, package_prefix.strip(), imports
    )


def _start_tree(
    environment: dict[str, str], phase: str, run_id: str, log_path: Path | None = None
) -> subprocess.Popen[Any]:
    if phase == RPENT_BOX_PHASE:
        _check_file(RPENT_PLAN_RUNNER_EXECUTABLE, "installed RPent plan runner")
        if not os.access(RPENT_PLAN_RUNNER_EXECUTABLE, os.X_OK):
            raise SmokeError("installed RPent plan runner is not executable")
        stdout_path = log_path or ARTIFACT_ROOT / f"tree-{phase}-{run_id}.log"
        with stdout_path.open("wb") as stdout:
            return subprocess.Popen(
                [os.fspath(RPENT_PLAN_RUNNER_EXECUTABLE)],
                env=environment,
                stdin=subprocess.DEVNULL,
                stdout=stdout,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
    if phase == "box":
        _check_file(SINGLE_BOX_TREE_EXECUTABLE, "installed single-box tree executable")
        _check_file(SINGLE_BOX_TREE_PATH, "installed single-box tree XML")
        if not os.access(SINGLE_BOX_TREE_EXECUTABLE, os.X_OK):
            raise SmokeError("installed single-box tree executable is not executable")
        stdout_path = log_path or ARTIFACT_ROOT / f"tree-{phase}-{run_id}.log"
        with stdout_path.open("wb") as stdout:
            return subprocess.Popen(
                [os.fspath(SINGLE_BOX_TREE_EXECUTABLE)],
                env={
                    **environment,
                    "LYNSENSE_SINGLE_BOX_TREE": str(SINGLE_BOX_TREE_PATH),
                },
                stdin=subprocess.DEVNULL,
                stdout=stdout,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
    _check_file(PYTREES_NODE_PATH, "installed lynsense_pytrees executable")
    if not os.access(PYTREES_NODE_PATH, os.X_OK):
        raise SmokeError("installed lynsense_pytrees executable is not executable")
    stdout_path = log_path or ARTIFACT_ROOT / f"tree-{phase}-{run_id}.log"
    with stdout_path.open("wb") as stdout:
        return subprocess.Popen(
            [
                os.fspath(PYTREES_NODE_PATH),
                "--ros-args",
                "-p",
                f"xml:={TREE_NAMES[phase]}",
            ],
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=stdout,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )


def _start_phase_processes(
    phase: str, environment: dict[str, str], run_id: str
) -> PhaseProcesses:
    processes = PhaseProcesses()
    try:
        processes.webots = _start_webots(phase, environment, run_id)
        processes.controller = _start_controller(environment, phase, run_id)
        return processes
    except BaseException:
        _cleanup(processes)
        raise


def _run_phase(phase: str, *, plan_path: Path | None = None) -> dict[str, Any]:
    if phase == RPENT_BOX_PHASE:
        if plan_path is None:
            raise SmokeError("rpent-box requires --plan")
        _load_rpent_plan(plan_path)
    _check_file(CONFIG_PATHS[phase], "simulation configuration")
    event_path, run_id = _prepare_event(phase)
    environment = _command_environment(
        phase, event_path, run_id, plan_path=plan_path
    )
    processes: PhaseProcesses | None = None
    result = {"phase": phase, "run_id": run_id, "event_file": event_path.name}
    try:
        if phase == "interface":
            runtime_isolation = _collect_runtime_isolation_evidence()
            result["runtime_isolation"] = runtime_isolation
            _validate_runtime_isolation(runtime_isolation)
        processes = _start_phase_processes(phase, environment, run_id)
        cursor = _wait_for_ready(processes, event_path, run_id, phase)

        if phase == "interface":
            _run_probe(environment, phase, run_id, "interface")
            _cleanup(processes)
            processes = _start_phase_processes(phase, environment, run_id)
            _run_probe(environment, phase, run_id, "cancel")
        else:
            _wait_for_action_graph(processes, environment, phase)
            tree = _start_tree(environment, phase, run_id)
            processes.tree = tree
            if tree.poll() is not None:
                raise SmokeError("pytrees node exited immediately")
            _watch_tree(
                tree,
                processes,
                cursor,
                blocked=phase == "blocked",
                expected_script=EXPECTED_SCRIPTS[phase],
                final_pose=FINAL_POSES[phase],
                phase=phase,
            )
        result["outcome"] = "passed"
        return result
    except BaseException as exc:
        result["outcome"] = "failed"
        result["reason"] = f"{type(exc).__name__}: {exc}"
        return result
    finally:
        if processes is not None:
            _cleanup(processes)


def _write_summary(results: list[dict[str, Any]]) -> None:
    ARTIFACT_ROOT.mkdir(parents=True, exist_ok=True)
    passed = all(result.get("outcome") == "passed" for result in results)
    summary = {
        "passed": passed,
        "phases": results,
        "generated_at": dt.datetime.now(dt.timezone.utc).isoformat(),
    }
    (ARTIFACT_ROOT / "summary.json").write_text(
        json.dumps(summary, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    interface_results = [result for result in results if result.get("phase") == "interface"]
    if interface_results:
        (ARTIFACT_ROOT / "interface-summary.json").write_text(
            json.dumps(interface_results[-1], sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )


def _run_all(phases: tuple[str, ...]) -> int:
    results: list[dict[str, Any]] = []
    try:
        for phase in phases:
            results.append(_run_phase(phase))
        return 0 if all(result.get("outcome") == "passed" for result in results) else 1
    finally:
        _write_summary(results)


def main(args: Any = None) -> int:
    parser = argparse.ArgumentParser(description="Run Lynsense Webots smoke gates.")
    parser.add_argument(
        "--phase",
        required=True,
        choices=(
            "interface", "smoke", "blocked", "match", "box", "rpent-box",
            "all", "box-all",
        ),
    )
    parser.add_argument("--plan", type=Path)
    args = parser.parse_args(args)
    if args.plan is not None and args.phase != RPENT_BOX_PHASE:
        parser.error("--plan is only valid with --phase rpent-box")
    if args.phase == RPENT_BOX_PHASE and args.plan is None:
        parser.error("--phase rpent-box requires --plan")
    if args.phase in {"all", "box-all"}:
        return _run_all(PHASE_SEQUENCE[args.phase])
    result = _run_phase(args.phase, plan_path=args.plan)
    _write_summary([result])
    return 0 if result.get("outcome") == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
