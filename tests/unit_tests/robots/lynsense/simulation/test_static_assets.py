from __future__ import annotations

import ast
import importlib.util
import json
import math
import os
import runpy
from pathlib import Path


def test_viewer_compose_does_not_mount_host_x11_or_extra_paths():
    compose = Path("robots/lynsense/simulation/docker/compose.viewer.yaml").read_text(
        encoding="utf-8"
    )
    assert "/tmp/.X11-unix" not in compose
    assert compose.count("type: bind") == 3
import sys
import threading
import types

import pytest
import yaml
from xml.etree import ElementTree

from lynsense_webots_sim.action_runtime import (
    Acceptance,
    Outcome as RuntimeOutcome,
    RuntimeObservation,
    TerminalResult,
)
from lynsense_webots_sim.events import EventRecorder
from lynsense_webots_sim.geometry import Pose2
from lynsense_webots_sim.state_machine import (
    ActionFeedback,
    MotionCommand,
    MotionPhase,
)


REPO_ROOT = Path(__file__).resolve().parents[5]
SIMULATION_ROOT = REPO_ROOT / "robots" / "lynsense" / "simulation" / "ros"
SIMULATION_TREE = REPO_ROOT / "robots" / "lynsense" / "simulation"
DOCKER_PATH = SIMULATION_TREE / "docker"
DOCKERFILE_PATH = DOCKER_PATH / "Dockerfile"
BOOTSTRAP_PATH = DOCKER_PATH / "bootstrap.py"
COMPOSE_PATH = DOCKER_PATH / "compose.yaml"
COMPOSE_GUI_PATH = DOCKER_PATH / "compose.gui.yml"
DOCKERIGNORE_PATH = DOCKER_PATH / ".dockerignore"
ORCHESTRATOR_PATH = (
    SIMULATION_ROOT
    / "lynsense_webots_sim"
    / "lynsense_webots_sim"
    / "scripts"
    / "run_smoke.py"
)
PROBE_PATH = (
    SIMULATION_ROOT
    / "lynsense_webots_sim"
    / "lynsense_webots_sim"
    / "scripts"
    / "action_probe.py"
)
LAUNCH_PATH = SIMULATION_ROOT / "lynsense_webots_sim" / "launch" / "minimal_smoke.launch.py"
SIMULATION_README_PATH = SIMULATION_TREE / "README.md"
ACTION_ROOT = SIMULATION_ROOT / "lynsense_utils_compat" / "action"
CONFIG_PATH = SIMULATION_ROOT / "lynsense_webots_sim" / "config" / "smoke.yaml"
MATCH_CONFIG_PATH = SIMULATION_ROOT / "lynsense_webots_sim" / "config" / "match.yaml"
MATCH_TREE_PATH = (
    SIMULATION_ROOT
    / "lynsense_webots_sim"
    / "trees"
    / "lynsense_match_min_tree.xml"
)
WORLD_ROOT = SIMULATION_ROOT / "lynsense_webots_sim" / "worlds"
SMOKE_WORLD_PATH = WORLD_ROOT / "lynsense_smoke.wbt"
BLOCKED_WORLD_PATH = WORLD_ROOT / "lynsense_blocked.wbt"
MATCH_WORLD_PATH = WORLD_ROOT / "lynsense_match.wbt"
CONTROLLER_PATH = (
    SIMULATION_ROOT
    / "lynsense_webots_sim"
    / "lynsense_webots_sim"
    / "webots_controller.py"
)


def _controller_source() -> str:
    return CONTROLLER_PATH.read_text(encoding="utf-8")


def _controller_tree() -> ast.Module:
    return ast.parse(_controller_source(), filename=str(CONTROLLER_PATH))


def test_controller_entry_point_is_declared():
    setup = (SIMULATION_ROOT / "lynsense_webots_sim" / "setup.py").read_text()
    assert (
        "lynsense_webots_controller = "
        "lynsense_webots_sim.webots_controller:main" in setup
    )


def test_launch_packaging_excludes_bytecode_directories(tmp_path, monkeypatch):
    launch = tmp_path / "launch"
    launch.mkdir()
    (launch / "minimal_smoke.launch.py").touch()
    (launch / "__pycache__").mkdir()
    captured = {}
    setuptools = types.ModuleType("setuptools")
    setuptools.setup = lambda **kwargs: captured.update(kwargs)
    monkeypatch.setitem(sys.modules, "setuptools", setuptools)
    monkeypatch.chdir(tmp_path)
    runpy.run_path(str(SIMULATION_ROOT / "lynsense_webots_sim" / "setup.py"))

    files = dict(captured["data_files"])["share/lynsense_webots_sim/launch"]
    assert files == ["launch/minimal_smoke.launch.py"]
    assert all(Path(path).is_file() for path in files)


@pytest.mark.parametrize("world", [SMOKE_WORLD_PATH, BLOCKED_WORLD_PATH, MATCH_WORLD_PATH])
def test_worlds_disable_default_online_audio_assets(world):
    source = world.read_text(encoding="utf-8")
    contacts = source.count("ContactProperties {")
    motors = source.count("RotationalMotor {")
    for field in ("bumpSound", "rollSound", "slideSound"):
        assert source.count(f'{field} ""') == contacts
    assert source.count('sound ""') == motors


def test_ros_and_webots_imports_are_lazy():
    tree = _controller_tree()
    module_imports = {
        alias.name.split(".")[0]
        for node in tree.body
        if isinstance(node, (ast.Import, ast.ImportFrom))
        for alias in node.names
    }
    assert "rclpy" not in module_imports
    assert "controller" not in module_imports
    assert "lynsense_utils" not in module_imports

    main = next(
        node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main"
    )
    main_names = {
        alias.name.split(".")[0]
        for node in ast.walk(main)
        if isinstance(node, (ast.Import, ast.ImportFrom))
        for alias in node.names
    }
    main_names |= {
        node.module.split(".")[0]
        for node in ast.walk(main)
        if isinstance(node, ast.ImportFrom) and node.module
    }
    assert {"rclpy", "controller", "lynsense_utils"} <= main_names


def test_controller_targets_exactly_the_two_wheel_motors():
    source = _controller_source()
    assert (
        'MOTOR_NAMES = ("left_wheel_motor", "right_wheel_motor")' in source
    )
    assert 'motor.setPosition(float("inf"))' in source
    assert source.count("left_wheel_motor") >= 1
    assert source.count("right_wheel_motor") >= 1
    assert "getDevice(left" not in source
    assert "getDevice(right" not in source


def test_simulation_loop_uses_supervisor_time_and_identity_wheel_rates():
    source = _controller_source()
    assert "self._robot.step(32) != -1" in source
    assert "sim_time_s = self._robot.getTime()" in source
    assert "self._runtime.observe(pose, sim_time_s)" in source
    assert "from lynsense_webots_sim.geometry import Pose2, webots_motor_rate" in source
    assert "webots_motor_rate(observation.command.left_wheel_rate_radps)" in source
    assert "webots_motor_rate(observation.command.right_wheel_rate_radps)" in source

    from lynsense_webots_sim.webots_controller import webots_motor_rate

    assert webots_motor_rate(-3.25) == -3.25
    assert webots_motor_rate(0.0) == 0.0
    assert webots_motor_rate(9.5) == 9.5


def test_executor_thread_and_signal_contracts_are_explicit():
    tree = _controller_tree()
    executor = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "_start_executor"
    )
    assert any(
        isinstance(node, ast.ImportFrom)
        and node.module == "rclpy.executors"
        and any(alias.name == "MultiThreadedExecutor" for alias in node.names)
        for node in executor.body
    )
    assert any(
        isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Attribute)
            and ast.unparse(target) == "self._executor"
            for target in node.targets
        )
        and isinstance(node.value, ast.Call)
        and ast.unparse(node.value) == "MultiThreadedExecutor(num_threads=2)"
        for node in executor.body
    )
    source = _controller_source()
    assert "SingleThreadedExecutor" not in source
    assert "target=self._executor_loop" in source
    assert "daemon=True" in source
    assert "spin_once(timeout_sec=0.01)" in source
    assert "signal.SIGINT" in source
    assert "signal.SIGTERM" in source
    assert "executor_failed:" in source


def test_ros_action_servers_share_reentrant_callback_group():
    tree = _controller_tree()
    main = next(
        node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main"
    )
    assert any(
        isinstance(node, ast.ImportFrom)
        and node.module == "rclpy.callback_groups"
        and any(alias.name == "ReentrantCallbackGroup" for alias in node.names)
        for node in ast.walk(main)
    )
    assignments = [
        node
        for node in ast.walk(main)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "callback_group"
            for target in node.targets
        )
        and isinstance(node.value, ast.Call)
        and ast.unparse(node.value) == "ReentrantCallbackGroup()"
    ]
    assert len(assignments) == 1

    action_server_calls = [
        node
        for node in ast.walk(main)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "ActionServer"
    ]
    assert len(action_server_calls) == 1
    for call in action_server_calls:
        keyword = next(keyword for keyword in call.keywords if keyword.arg == "callback_group")
        assert ast.unparse(keyword.value) == "callback_group"


def test_reentrant_ros_callbacks_do_not_bypass_controller_lock_or_motor_writer():
    tree = _controller_tree()
    methods = {
        node.name: node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef)
        and node.name in {"_goal_callback", "_execute_callback", "_cancel_callback", "run"}
    }

    def contains_controller_lock(body):
        return any(
            any(
                isinstance(item.context_expr, ast.Attribute)
                and ast.unparse(item.context_expr) == "self._lock"
                for item in node.items
            )
            for statement in body
            for node in ast.walk(statement)
            if isinstance(node, ast.With)
        )

    for name in ("_goal_callback", "_execute_callback", "_cancel_callback"):
        assert contains_controller_lock(methods[name].body)
        assert not _body_contains_call(methods[name].body, "self", "_write_motors")

    run_motor_calls = [
        node
        for node in ast.walk(methods["run"])
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "_write_motors"
        and ast.unparse(node.func.value) == "self"
    ]
    assert len(run_motor_calls) == 2


def test_shutdown_claims_reserved_goal_when_runtime_remains_active():
    recorder = _RecorderFake(run_id="reserved-shutdown")
    motors = _motors()
    runtime = _RuntimeFake()
    controller = _controller_with_fakes(recorder, motors, runtime=runtime)
    assert controller._record_controller_ready() is True
    descriptor = _GoalDescriptorFake()
    runtime.active = True
    controller._reserved_goal = descriptor

    assert controller._shutdown() == 1
    assert controller._reserved_goal is None
    assert controller._active_goal is None
    assert controller._active_goal_done.is_set()
    assert [motor.velocities[-1] for motor in motors.values()] == [0.0, 0.0]
    event = _events(recorder)[-1]
    assert event["kind"] == "webots_terminated"
    assert event["action"] == "nav_to_pose"
    assert event["reason"] == "webots_terminated_with_active_goal"


def test_delayed_execute_callback_returns_after_shutdown_without_waiting():
    recorder = _RecorderFake(run_id="delayed-execute-shutdown")
    motors = _motors()
    controller = _controller_with_fakes(recorder, motors)
    assert controller._record_controller_ready() is True
    descriptor = _GoalDescriptorFake()
    controller._reserved_goal = descriptor
    controller._shutdown_requested.set()
    controller._executor_stop_requested.set()
    result = object()
    execute = controller._execute_callback("nav", lambda: result)
    returned = []

    thread = threading.Thread(
        target=lambda: returned.append(execute(_GoalHandleFake())),
        daemon=True,
    )
    thread.start()
    thread.join(timeout=0.2)

    assert not thread.is_alive()
    assert returned == [result]
    assert controller._reserved_goal is descriptor
    assert controller._active_goal is None
    assert not controller._active_goal_done.is_set()


def test_execute_callback_returns_default_when_reservation_is_missing():
    recorder = _RecorderFake(run_id="missing-reservation")
    motors = _motors()
    controller = _controller_with_fakes(recorder, motors)
    assert controller._record_controller_ready() is True
    result = object()
    execute = controller._execute_callback("nav", lambda: result)

    assert execute(_GoalHandleFake()) is result
    assert controller._reserved_goal is None
    assert controller._active_goal is None
    assert not controller._active_goal_done.is_set()


def test_started_execute_callback_is_released_by_terminal_completion():
    recorder = _RecorderFake(run_id="started-execute")
    motors = _motors()
    controller = _controller_with_fakes(recorder, motors)
    assert controller._record_controller_ready() is True
    controller._reserved_goal = _GoalDescriptorFake()
    result = object()
    execute = controller._execute_callback("nav", lambda: result)
    returned = []

    thread = threading.Thread(
        target=lambda returned=returned: returned.append(execute(_GoalHandleFake())),
        daemon=True,
    )
    thread.start()
    for _ in range(100):
        if controller._active_goal is not None:
            break
        time.sleep(0.001)
    assert controller._active_goal is not None

    controller._finish_active_goal(RuntimeOutcome.SUCCEEDED)
    thread.join(timeout=1.0)
    assert not thread.is_alive()
    assert returned == [result]
    assert controller._active_goal_done.is_set()


def test_finally_cleanup_is_in_required_order():
    tree = _controller_tree()
    run = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "run"
    )
    final_body = None
    for node in ast.walk(run):
        if isinstance(node, ast.Try) and node.finalbody:
            final_body = node.finalbody
            break
    assert final_body is not None

    calls = [
        expression.value.func
        for expression in final_body
        if isinstance(expression, ast.Expr) and isinstance(expression.value, ast.Call)
    ]
    call_names = []
    for call in calls:
        if isinstance(call, ast.Attribute):
            call_names.append(call.attr)
        elif isinstance(call, ast.Name):
            call_names.append(call.id)
    assert call_names == [
        "_safe_stop_motors",
        "_destroy_action_servers",
        "_destroy_node",
        "_shutdown_executor",
        "_shutdown_rclpy",
    ]


def test_controller_does_not_create_real_robot_interfaces():
    source = _controller_source()
    tree = _controller_tree()
    forbidden_calls = {"create_publisher", "create_client", "create_service"}
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr in forbidden_calls
        ):
            pytest.fail(f"forbidden ROS interface call: {node.func.attr}")
    assert "action_client" not in source.lower()
    for forbidden in ("xarm", "chassis"):
        assert forbidden not in source.lower()


def test_fault_signal_stops_motors_and_aborts_the_active_goal():
    recorder = _RecorderFake(run_id="fault-run")
    motors = {
        "left_wheel_motor": _MotorFake(),
        "right_wheel_motor": _MotorFake(),
    }
    controller = _controller_with_fakes(recorder, motors)
    handle = _GoalHandleFake()
    controller._reserved_goal = _GoalDescriptorFake()
    execute = controller._execute_callback("nav", lambda: object())
    execute_thread = _execute_goal(controller, execute, handle)
    controller._faults.request("executor_failed:callback boom")
    reason = controller._faults.take()

    assert reason == "executor_failed:callback boom"
    assert controller._abort_for_fault(reason) == 1
    execute_thread.join(timeout=1.0)
    assert not execute_thread.is_alive()
    assert handle.abort_count == 1
    assert controller._active_goal_done.is_set()
    assert [motor.velocities for motor in motors.values()] == [[0.0], [0.0]]
    assert controller._faults.take() is None
    recorder.close()


def test_controller_loop_holds_controller_lock_during_observation():
    tree = _controller_tree()
    run = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "run"
    )
    locked_sections = [
        node
        for node in ast.walk(run)
        if isinstance(node, ast.With)
        and any(
            isinstance(item.context_expr, ast.Attribute)
            and item.context_expr.attr == "_lock"
            for item in node.items
        )
    ]
    assert any(
        _body_contains_attribute(section.body, "self._runtime", "active")
        and _body_contains_attribute(section.body, "self", "_active_goal")
        and _body_contains_call(section.body, "self._runtime", "observe")
        and _body_contains_assignment(section.body, "self._latest_telemetry")
        for section in locked_sections
    )


def test_controller_idle_loop_skips_observe_and_writes_zero_rates():
    recorder = _RecorderFake(run_id="idle-run")
    motors = _motors()
    robot = _IdleRobotFake(motors)
    runtime = _RuntimeFake()
    controller = _controller_with_fakes(recorder, motors, robot=robot, runtime=runtime)
    controller._record_controller_ready = lambda: True
    controller._start_executor = lambda: None
    controller._shutdown = lambda: 0
    robot.step_results = [0, -1]

    assert controller.run() == 0
    assert runtime.observed == []
    assert runtime.active_lock_held is True
    assert [motor.velocities for motor in motors.values()] == [
        [0.0, 0.0],
        [0.0, 0.0],
    ]
    assert robot.step_index == 2
    assert controller._latest_telemetry.pose == Pose2(1.25, -0.5, math.pi / 6)
    assert controller._latest_telemetry.sim_time_s == 1.1
    assert controller._latest_telemetry.left_wheel_rate_radps == 0.0
    assert controller._latest_telemetry.right_wheel_rate_radps == 0.0
    assert recorder.events == []
    recorder.close()


def test_pending_reserved_goal_is_idle_until_execute_binds_it():
    recorder = _RecorderFake(run_id="pending-reserved-run")
    motors = _motors()
    robot = _IdleRobotFake(motors)
    runtime = _RuntimeFake()
    controller = _controller_with_fakes(recorder, motors, robot=robot, runtime=runtime)
    descriptor = _GoalDescriptorFake()
    runtime.active = True
    controller._reserved_goal = descriptor
    controller._record_controller_ready = lambda: True
    controller._start_executor = lambda: None
    controller._shutdown = lambda: 0
    robot.step_results = [0, -1]

    assert controller.run() == 0
    assert runtime.observed == []
    assert controller._reserved_goal is descriptor
    assert controller._active_goal is None
    assert [motor.velocities for motor in motors.values()] == [
        [0.0, 0.0],
        [0.0, 0.0],
    ]
    assert controller._latest_telemetry.pose == Pose2(1.25, -0.5, math.pi / 6)
    assert controller._latest_telemetry.sim_time_s == 1.1
    assert controller._latest_telemetry.left_wheel_rate_radps == 0.0
    assert controller._latest_telemetry.right_wheel_rate_radps == 0.0
    assert recorder.events == []


def test_bound_active_goal_resumes_runtime_observation():
    recorder = _RecorderFake(run_id="bound-active-run")
    motors = _motors()
    runtime = _RuntimeFake(
        observations=[
            _observation(
                RuntimeOutcome.RUNNING,
                terminal=False,
                left_rate=2.0,
                right_rate=-1.0,
            )
        ]
    )
    controller = _controller_with_fakes(recorder, motors, runtime=runtime)
    runtime.active = True
    controller._active_goal = _GoalDescriptorFake()
    controller._record_controller_ready = lambda: True
    controller._start_executor = lambda: None
    controller._robot.step_results = [0, -1]

    assert controller.run() == 1
    assert len(runtime.observed) == 1
    assert runtime.lock_held_during_observe is True
    assert motors["left_wheel_motor"].velocities == [2.0, 0.0, 0.0]
    assert motors["right_wheel_motor"].velocities == [-1.0, 0.0, 0.0]


def test_webots_step_exception_requests_a_fault_and_returns_nonzero():
    recorder = _RecorderFake(run_id="webots-run")
    motors = {
        "left_wheel_motor": _MotorFake(),
        "right_wheel_motor": _MotorFake(),
    }
    robot = _RobotFake(motors)
    robot.step_results = [0, RuntimeError("simulation stopped")]
    controller = _controller_with_fakes(recorder, motors, robot=robot)
    controller._record_controller_ready = lambda: True
    controller._start_executor = lambda: None
    controller._shutdown = lambda: 1

    assert controller.run() == 1
    assert [motor.velocities for motor in motors.values()] == [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ]
    recorder.close()


def test_controller_ready_builds_servers_reads_pose_stops_motors_then_records():
    recorder = _RecorderFake(run_id="ready-run")
    motors = {
        "left_wheel_motor": _MotorFake(),
        "right_wheel_motor": _MotorFake(),
    }
    robot = _RobotFake(
        motors,
        position=[1.25, -0.5, 0.08],
        orientation=[
            math.cos(math.pi / 6),
            -math.sin(math.pi / 6),
            0.0,
            math.sin(math.pi / 6),
            math.cos(math.pi / 6),
            0.0,
            0.0,
            0.0,
            1.0,
        ],
    )
    controller = _controller_with_fakes(recorder, motors, robot=robot)

    assert controller._record_controller_ready() is True
    assert robot.log[:5] == [
        "action_server:nav",
        "action_server:move",
        "get_time",
        "get_position",
        "get_orientation",
    ]
    assert robot.log[5:] == [
        "motor:left_wheel_motor:0.0",
        "motor:right_wheel_motor:0.0",
    ]
    assert [motor.positions for motor in motors.values()] == [
        [float("inf")],
        [float("inf")],
    ]
    assert all(binding.callbacks is not None for binding in controller._action_server_bindings)

    events = _events(recorder)
    assert len(events) == 1
    assert events[0] == {
        "kind": "controller_ready",
        "action": "controller",
        "sim_time_s": 0.1,
        "action_names": ["nav_to_pose", "move_distance"],
        "action_types": {
            "nav_to_pose": "lynsense_utils/action/NavToPose",
            "move_distance": "lynsense_utils/action/MoveDistance",
        },
        "motor_names": ["left_wheel_motor", "right_wheel_motor"],
        "pose": {"x": 1.25, "y": -0.5, "yaw": pytest.approx(math.pi / 6)},
        "motor_rates": {
            "left_wheel_rate_radps": 0.0,
            "right_wheel_rate_radps": 0.0,
        },
        "phase": "ready",
        "outcome": "ready",
        "reason": "",
        "run_id": "ready-run",
    }


def test_goal_callbacks_accept_then_reject_concurrently_and_record_both(monkeypatch):
    recorder = _RecorderFake(run_id="goal-run")
    motors = _motors()
    controller = _controller_with_fakes(
        recorder,
        motors,
        runtime=_RuntimeFake(
            acceptances=[
                Acceptance(True, ""),
                Acceptance(False, "action_active"),
            ]
        ),
    )
    assert controller._record_controller_ready() is True
    goal_response = _install_fake_goal_response(monkeypatch)
    nav_goal = controller._action_server_bindings[0].callbacks["goal"]
    move_goal = controller._action_server_bindings[1].callbacks["goal"]

    assert (
        nav_goal(_GoalRequestFake(goal_name="sim_goal1"))
        is goal_response.ACCEPT
    )
    assert (
        move_goal(_GoalRequestFake(distance=-1.5, angle=0.0))
        is goal_response.REJECT
    )

    events = _events(recorder)
    assert [(event["kind"], event["action"], event["reason"]) for event in events[1:]] == [
        ("goal_accepted", "nav_to_pose", ""),
        ("goal_rejected", "move_distance", "action_active"),
    ]
    assert controller._reserved_goal is not None
    assert controller._reserved_goal.action_name == "nav_to_pose"
    assert controller._runtime.submit_arguments == [
        ("nav", "sim_goal1"),
        ("move", -1.5, 0.0),
    ]


def test_cancel_callback_sets_runtime_cancel_and_publishes_canceled_result(monkeypatch):
    recorder = _RecorderFake(run_id="cancel-run")
    motors = _motors()
    runtime = _RuntimeFake(
        acceptances=[Acceptance(True, "")],
        observations=[
            _observation(RuntimeOutcome.CANCELED, terminal=False),
            _observation(RuntimeOutcome.CANCELED, terminal=True, success=False),
        ],
    )
    controller = _controller_with_fakes(recorder, motors, runtime=runtime)
    assert controller._record_controller_ready() is True
    callbacks = controller._action_server_bindings[0].callbacks
    goal_response = _install_fake_goal_response(monkeypatch)
    assert (
        callbacks["goal"](_GoalRequestFake(goal_name="sim_goal1"))
        is goal_response.ACCEPT
    )
    handle = _GoalHandleFake()
    execute_thread = _execute_goal(controller, callbacks["execute"], handle)

    assert callbacks["cancel"](_CancelRequestFake()) is goal_response.ACCEPT
    assert runtime.cancel_requested is True
    handle.is_cancel_requested = True
    controller._start_executor = lambda: None
    controller._robot.step_results = [0, 0, -1]
    assert controller.run() == 1
    execute_thread.join(timeout=1.0)
    assert not execute_thread.is_alive()

    assert handle.canceled_count == 1
    events = _events(recorder)
    assert events[-2]["kind"] == "action_feedback"
    assert events[-1]["kind"] == "action_result"
    assert events[-1]["outcome"] == "canceled"
    assert events[-1]["success"] is False
    assert controller._active_goal_response.success is False


def test_normal_terminal_goal_records_complete_feedback_and_result(monkeypatch):
    recorder = _RecorderFake(run_id="terminal-run")
    motors = _motors()
    runtime = _RuntimeFake(
        acceptances=[Acceptance(True, "")],
        observations=[
            _observation(
                RuntimeOutcome.SUCCEEDED,
                terminal=True,
                success=True,
                left_rate=1.25,
                right_rate=-0.5,
                distance_remaining=0.25,
                angle_remaining=12.5,
                reason="goal_reached",
            )
        ],
    )
    controller = _controller_with_fakes(recorder, motors, runtime=runtime)
    assert controller._record_controller_ready() is True
    callbacks = controller._action_server_bindings[0].callbacks
    goal_response = _install_fake_goal_response(monkeypatch)
    assert (
        callbacks["goal"](_GoalRequestFake(goal_name="sim_goal2"))
        is goal_response.ACCEPT
    )
    handle = _GoalHandleFake()
    execute_thread = _execute_goal(controller, callbacks["execute"], handle)
    controller._start_executor = lambda: None
    controller._robot.step_results = [0, -1]

    assert controller.run() == 0
    execute_thread.join(timeout=1.0)
    assert not execute_thread.is_alive()
    assert runtime.lock_held_during_observe is True
    assert handle.succeeded_count == 1
    assert len(handle.feedback) == 1
    assert handle.feedback[0].distance_remaining == 0.25
    assert not hasattr(handle.feedback[0], "angle_remaining")
    assert controller._active_goal_response.success is True
    assert controller._active_goal_response.message == "goal_reached"
    assert not hasattr(controller._active_goal_response, "distance_remaining")

    feedback, result = _events(recorder)[-2:]
    required = {
        "action",
        "goal_name",
        "distance_m",
        "angle_deg",
        "distance_remaining_m",
        "angle_remaining_deg",
        "phase",
        "outcome",
        "reason",
        "pose",
        "sim_time_s",
        "motor_rates",
    }
    assert required <= feedback.keys()
    assert required <= result.keys()
    assert feedback["motor_rates"] == {
        "left_wheel_rate_radps": 1.25,
        "right_wheel_rate_radps": -0.5,
    }
    assert result["success"] is True
    assert result["reason"] == "goal_reached"


def test_move_feedback_uses_both_generated_ros_fields(monkeypatch):
    recorder = _RecorderFake(run_id="move-feedback-run")
    runtime = _RuntimeFake(
        acceptances=[Acceptance(True, "")],
        observations=[
            _observation(
                RuntimeOutcome.SUCCEEDED,
                terminal=True,
                success=True,
                distance_remaining=0.4,
                angle_remaining=8.0,
            )
        ],
    )
    controller = _controller_with_fakes(recorder, _motors(), runtime=runtime)
    assert controller._record_controller_ready() is True
    callbacks = controller._action_server_bindings[1].callbacks
    goal_response = _install_fake_goal_response(monkeypatch)
    assert (
        callbacks["goal"](_GoalRequestFake(distance=0.5, angle=10.0))
        is goal_response.ACCEPT
    )
    handle = _GoalHandleFake()
    execute_thread = _execute_goal(controller, callbacks["execute"], handle)
    controller._start_executor = lambda: None
    controller._robot.step_results = [0, -1]

    assert controller.run() == 0
    execute_thread.join(timeout=1.0)
    assert not execute_thread.is_alive()
    assert handle.feedback[0].distance_remaining == 0.4
    assert handle.feedback[0].angle_remaining == 8.0
    assert not hasattr(controller._active_goal_response, "distance_remaining")
    assert not hasattr(controller._active_goal_response, "angle_remaining")


def test_normal_webots_termination_with_active_goal_aborts_nonzero():
    recorder = _RecorderFake(run_id="active-end-run")
    motors = _motors()
    runtime = _RuntimeFake(
        observations=[_observation(RuntimeOutcome.RUNNING, terminal=False)]
    )
    runtime.active = True
    controller = _controller_with_fakes(recorder, motors, runtime=runtime)
    assert controller._record_controller_ready() is True
    handle = _GoalHandleFake()
    controller._reserved_goal = _GoalDescriptorFake()
    execute = controller._action_server_bindings[0].callbacks["execute"]
    execute_thread = _execute_goal(controller, execute, handle)
    controller._start_executor = lambda: None
    controller._robot.step_results = [0, -1]

    assert controller.run() == 1
    execute_thread.join(timeout=1.0)
    assert not execute_thread.is_alive()
    assert handle.abort_count == 1
    events = _events(recorder)
    assert events[-1]["kind"] == "webots_terminated"
    assert events[-1]["reason"] == "webots_terminated_with_active_goal"


def test_shutdown_reads_active_runtime_state_under_controller_lock():
    recorder = _RecorderFake(run_id="shutdown-active-lock")
    motors = _motors()
    runtime = _RuntimeFake()
    controller = _controller_with_fakes(recorder, motors, runtime=runtime)
    assert controller._record_controller_ready() is True
    handle = _GoalHandleFake()
    controller._reserved_goal = _GoalDescriptorFake()
    execute = controller._action_server_bindings[0].callbacks["execute"]
    execute_thread = _execute_goal(controller, execute, handle)
    runtime.active = True
    runtime.active_lock_held = None

    assert controller._shutdown() == 1
    execute_thread.join(timeout=1.0)
    assert not execute_thread.is_alive()
    assert runtime.active_lock_held is True
    assert handle.abort_count == 1
    assert recorder.events[-1]["kind"] == "webots_terminated"


@pytest.mark.parametrize(
    ("success", "expected_exit"), [(True, 0), (False, 1)]
)
def test_shutdown_selects_normal_result_under_controller_lock(success, expected_exit):
    recorder = _RecorderFake(run_id=f"shutdown-result-{success}")
    motors = _motors()
    runtime = _RuntimeFake()
    controller = _controller_with_fakes(recorder, motors, runtime=runtime)
    assert controller._record_controller_ready() is True
    runtime.terminal_result = TerminalResult(
        success=success,
        message="goal_reached" if success else "action_failed",
        outcome=RuntimeOutcome.SUCCEEDED if success else RuntimeOutcome.FAILED,
    )
    runtime.active_lock_held = None
    runtime.terminal_result_lock_held = None

    assert controller._shutdown() == expected_exit
    assert runtime.active_lock_held is True
    assert runtime.terminal_result_lock_held is True


def test_pose_read_failure_requests_webots_fault_and_stops_motors():
    recorder = _RecorderFake(run_id="pose-failure-run")
    motors = _motors()
    robot = _RobotFake(motors)
    controller = _controller_with_fakes(recorder, motors, robot=robot)
    assert controller._record_controller_ready() is True
    robot.self.position_error = RuntimeError("supervisor position unavailable")
    controller._record_controller_ready = lambda: True
    controller._start_executor = lambda: None
    controller._shutdown = lambda: 1
    robot.step_results = [0]

    assert controller.run() == 1
    events = _events(recorder)
    assert events[-1]["kind"] == "controller_failed"
    assert events[-1]["reason"].startswith("webots_failed:RuntimeError")
    assert [motor.velocities[-1] for motor in motors.values()] == [0.0, 0.0]


def test_motor_write_failure_requests_webots_fault_and_restores_zero():
    recorder = _RecorderFake(run_id="motor-failure-run")
    motors = _motors()
    motors["right_wheel_motor"].fail_once_value = -1.25
    runtime = _RuntimeFake(
        observations=[
            _observation(
                RuntimeOutcome.RUNNING,
                terminal=False,
                left_rate=2.5,
                right_rate=-1.25,
            )
        ]
    )
    runtime.active = True
    controller = _controller_with_fakes(recorder, motors, runtime=runtime)
    controller._active_goal = _GoalDescriptorFake()
    assert controller._record_controller_ready() is True
    controller._record_controller_ready = lambda: True
    controller._start_executor = lambda: None
    controller._shutdown = lambda: 1
    controller._robot.step_results = [0, -1]

    assert controller.run() == 1
    assert motors["left_wheel_motor"].velocities == [0.0, 2.5, 0.0, 0.0]
    assert motors["right_wheel_motor"].velocities == [0.0, 0.0, 0.0]
    events = _events(recorder)
    assert events[-1]["kind"] == "controller_failed"
    assert events[-1]["reason"].startswith("webots_failed:RuntimeError")
    assert controller._faults.take() is None


def test_feedback_event_write_failure_safe_stops_and_retains_internal_fault():
    recorder = _RecorderFake(fail_kinds={"action_feedback", "controller_failed"})
    motors = _motors()
    runtime = _RuntimeFake()
    controller = _controller_with_fakes(recorder, motors, runtime=runtime)
    assert controller._record_controller_ready() is True
    handle = _GoalHandleFake()
    controller._reserved_goal = _GoalDescriptorFake()
    execute = controller._action_server_bindings[0].callbacks["execute"]
    execute_thread = _execute_goal(controller, execute, handle)

    controller._publish_feedback_and_result(
        _observation(RuntimeOutcome.RUNNING, terminal=False)
    )

    execute_thread.join(timeout=1.0)
    assert not execute_thread.is_alive()
    assert handle.abort_count == 1
    assert [motor.velocities[-1] for motor in motors.values()] == [0.0, 0.0]
    assert controller._faults.take() == "event_recorder_failed:OSError: event sink failed"


class _MotorFake:
    def __init__(self, log=None):
        self.velocities = []
        self.positions = []
        self.log = log
        self.fail_once_value = None

    def setPosition(self, position):
        self.positions.append(float(position))

    def setVelocity(self, velocity):
        velocity = float(velocity)
        if velocity == self.fail_once_value:
            self.fail_once_value = None
            raise RuntimeError("motor rejected velocity")
        self.velocities.append(velocity)
        if self.log is not None:
            self.log.append(f"motor:{self.name}:{velocity}")


class _SelfFake:
    def __init__(self, log=None, position=(0.0, 0.0, 0.08), orientation=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)):
        self.log = log
        self.position = list(position)
        self.orientation = list(orientation)
        self.position_error = None

    def getPosition(self):
        if self.position_error is not None:
            raise self.position_error
        if self.log is not None:
            self.log.append("get_position")
        return self.position

    def getOrientation(self):
        if self.log is not None:
            self.log.append("get_orientation")
        return self.orientation


class _RobotFake:
    def __init__(self, motors, position=(0.0, 0.0, 0.08), orientation=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)):
        self.motors = motors
        self.log = []
        self.self = _SelfFake(self.log, position, orientation)
        self.step_results = []
        self.step_index = 0

    def getDevice(self, name):
        return self.motors[name]

    def getTime(self):
        self.log.append("get_time")
        return 0.1

    def getSelf(self):
        return self.self

    def step(self, _duration_ms):
        if not self.step_results:
            self.step_results = [0, -1]
        result = self.step_results[min(self.step_index, len(self.step_results) - 1)]
        self.step_index += 1
        if isinstance(result, Exception):
            raise result
        return result


class _IdleRobotFake(_RobotFake):
    def __init__(self, motors):
        super().__init__(motors)
        self.sim_time_s = 0.1
        self.self.position = [0.0, 0.0, 0.08]
        self.self.orientation = [
            math.cos(math.pi / 6),
            -math.sin(math.pi / 6),
            0.0,
            math.sin(math.pi / 6),
            math.cos(math.pi / 6),
            0.0,
            0.0,
            0.0,
            1.0,
        ]

    def getTime(self):
        return self.sim_time_s

    def step(self, duration_ms):
        result = super().step(duration_ms)
        if result != -1:
            self.sim_time_s += 1.0
            self.self.position[0] += 1.25
            self.self.position[1] -= 0.5
        return result


class _RuntimeFake:
    def __init__(self, acceptances=None, observations=None):
        self.acceptances = list(acceptances or [])
        self.observations = list(observations or [])
        self.submit_arguments = []
        self.observed = []
        self.cancel_requested = False
        self._active = False
        self._terminal_result = None
        self.active_lock_held = None
        self.terminal_result_lock_held = None
        self.lock_held_during_observe = None
        self._controller = None

    def _accept(self, kind, *values):
        self.submit_arguments.append((kind, *values))
        acceptance = self.acceptances.pop(0)
        if acceptance.accepted:
            self.active = True
        return acceptance

    def submit_nav(self, goal_name):
        return self._accept("nav", goal_name)

    def submit_move(self, distance, angle):
        return self._accept("move", distance, angle)

    @property
    def active(self):
        self.active_lock_held = (
            self._controller is None or self._controller._lock.locked()
        )
        return self._active

    @active.setter
    def active(self, value):
        self._active = value

    @property
    def terminal_result(self):
        self.terminal_result_lock_held = (
            self._controller is None or self._controller._lock.locked()
        )
        return self._terminal_result

    @terminal_result.setter
    def terminal_result(self, value):
        self._terminal_result = value

    def observe(self, _pose, _sim_time_s):
        self.lock_held_during_observe = (
            self._controller is None or self._controller._lock.locked()
        )
        observation = self.observations.pop(0)
        self.observed.append(observation)
        if observation.terminal:
            self._active = False
            self._terminal_result = observation.terminal_result
        else:
            self.active = True
        return observation

    def request_cancel(self):
        self.cancel_requested = True
        return self.active


class _GoalHandleFake:
    def __init__(self):
        self.abort_count = 0
        self.canceled_count = 0
        self.succeeded_count = 0
        self.feedback = []
        self.is_cancel_requested = False

    def abort(self):
        self.abort_count += 1

    def canceled(self):
        self.canceled_count += 1

    def succeed(self):
        self.succeeded_count += 1

    def publish_feedback(self, feedback):
        self.feedback.append(feedback)


class _GoalRequestFake:
    def __init__(self, goal_name=None, distance=0.0, angle=0.0):
        self.goal_name = goal_name
        self.distance = distance
        self.angle = angle


class _CancelRequestFake:
    pass


class _FeedbackFake:
    __slots__ = ("distance_remaining",)


class _MoveFeedbackFake(_FeedbackFake):
    __slots__ = ("angle_remaining",)



class _ResultFake:
    __slots__ = ("success", "message")

class _GoalDescriptorFake:
    kind = "nav"
    action_name = "nav_to_pose"
    goal_name = "sim_goal"
    distance_m = None
    angle_deg = None
    height_mm = None
    target = None
    gripper_position = None
    box_action = None
    box_flow = None
    box_config = None


class _NodeFake:
    pass


class _ActionServerBindingFake:
    def __init__(self, kind, action_name, log):
        self.kind = kind
        self.action_name = action_name
        self.log = log
        self.callbacks = None

    def build(self, _node, callbacks):
        self.callbacks = callbacks
        self.log.append(f"action_server:{self.kind}")
        return object()

    def new_feedback(self):
        if self.kind == "move":
            return _MoveFeedbackFake()
        return _FeedbackFake()

    def new_result(self):
        return _ResultFake()


class _RecorderFake:
    def __init__(self, run_id=None, fail_kinds=()):
        self.run_id = run_id
        self.events = []
        self.fail_kinds = set(fail_kinds)

    def record(self, event):
        if event["kind"] in self.fail_kinds:
            raise OSError("event sink failed")
        self.events.append({**event, "run_id": self.run_id})

    def close(self):
        pass


class _OrchestratorProcessFake:
    def __init__(self, exit_code=None):
        self.exit_code = exit_code
        self.pid = 424242
        self.interrupt_signals = []
        self.terminate_signals = []
        self.kill_signals = []

    def poll(self):
        return self.exit_code

    def send_signal(self, signal_value):
        self.interrupt_signals.append(signal_value)
        self.exit_code = 0

    def terminate(self):
        self.terminate_signals.append(True)
        self.exit_code = 1

    def kill(self):
        self.kill_signals.append(True)
        self.exit_code = 9

    def wait(self, timeout=None):
        return self.exit_code


def _motors(log=None):
    return {
        "left_wheel_motor": _MotorFake(log),
        "right_wheel_motor": _MotorFake(log),
    }


def _controller_with_fakes(recorder, motors, robot=None, runtime=None):
    from lynsense_webots_sim.webots_controller import LynsenseWebotsController

    operations = []
    for name, motor in motors.items():
        motor.name = name
        motor.log = operations
    robot = robot or _RobotFake(motors)
    robot.log = operations
    robot.self.log = operations
    runtime = runtime or _RuntimeFake()
    controller = LynsenseWebotsController(
        robot=robot,
        node=_NodeFake(),
        action_servers=(
            _ActionServerBindingFake("nav", "nav_to_pose", operations),
            _ActionServerBindingFake("move", "move_distance", operations),
        ),
        event_recorder=recorder,
        runtime=runtime,
        config={
            "goals": {},
            "motion": {},
            "physics": {},
            "world": {"controller_step_ms": 32},
            "timeouts": {},
            "actuators": {name: {} for name in motors},
            "scripts": {},
        },
    )
    runtime._controller = controller
    return controller


def _install_fake_goal_response(monkeypatch):
    class GoalResponse:
        ACCEPT = object()
        REJECT = object()

    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy_action = types.ModuleType("rclpy.action")
    fake_rclpy_action.GoalResponse = GoalResponse
    class CancelResponse:
        ACCEPT = GoalResponse.ACCEPT
        REJECT = GoalResponse.REJECT

    fake_rclpy_action.CancelResponse = CancelResponse
    fake_rclpy.action = fake_rclpy_action
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.action", fake_rclpy_action)
    return GoalResponse


def _events(recorder):
    return list(recorder.events)


def _observation(
    outcome,
    *,
    terminal,
    success=True,
    left_rate=0.0,
    right_rate=0.0,
    distance_remaining=0.0,
    angle_remaining=0.0,
    reason="",
):
    terminal_result = (
        TerminalResult(success=success, message=reason, outcome=outcome)
        if terminal
        else None
    )
    return RuntimeObservation(
        command=MotionCommand(left_rate, right_rate),
        feedback=ActionFeedback(distance_remaining, angle_remaining),
        phase=MotionPhase.TERMINAL if terminal else MotionPhase.NAV_APPROACH,
        outcome=outcome,
        reason=reason,
        terminal=terminal,
        terminal_result=terminal_result,
    )


def _execute_goal(controller, execute_callback, handle):
    thread = threading.Thread(target=execute_callback, args=(handle,), daemon=True)
    thread.start()
    ready = threading.Event()
    ready.wait(0.01)
    assert not thread.is_alive() or controller._active_goal_handle is handle
    return thread


def _body_contains_call(body, object_name, attribute):
    return any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == attribute
        and ast.unparse(node.func.value) == object_name
        for statement in body
        for node in ast.walk(statement)
    )


def _body_contains_attribute(body, object_name, attribute):
    return any(
        isinstance(node, ast.Attribute)
        and node.attr == attribute
        and ast.unparse(node.value) == object_name
        for statement in body
        for node in ast.walk(statement)
    )


def _body_contains_assignment(body, target_name):
    return any(
        isinstance(node, ast.Assign)
        and any(
            ast.unparse(target) == target_name
            for target in node.targets
        )
        for statement in body
        for node in ast.walk(statement)
    )


def test_action_definitions_match_simulation_contract():
    nav = ACTION_ROOT.joinpath("NavToPose.action").read_text()
    move = ACTION_ROOT.joinpath("MoveDistance.action").read_text()
    assert nav.splitlines()[:5] == [
        "string goal_name",
        "---",
        "bool success",
        "string message",
        "---",
    ]
    assert nav.splitlines()[5:] == ["float64 distance_remaining"]
    assert move.splitlines() == [
        "float64 distance",
        "float64 angle",
        "---",
        "bool success",
        "string message",
        "---",
        "float64 distance_remaining",
        "float64 angle_remaining",
    ]


def test_action_package_is_reconstructed_ros_interface_package():
    package_root = SIMULATION_ROOT / "lynsense_utils_compat"
    package = package_root.joinpath("package.xml").read_text()
    cmake = package_root.joinpath("CMakeLists.txt").read_text()
    readme = package_root.joinpath("README.md").read_text()

    assert "<name>lynsense_utils</name>" in package
    assert "<buildtool_depend>ament_cmake</buildtool_depend>" in package
    assert "<depend>rosidl_default_generators</depend>" in package
    assert "<exec_depend>rosidl_default_runtime</exec_depend>" in package
    assert (
        "<member_of_group>rosidl_interface_packages</member_of_group>"
        in package
    )
    assert "rosidl_generate_interfaces(${PROJECT_NAME}" in cmake
    assert '"action/NavToPose.action"' in cmake
    assert '"action/MoveDistance.action"' in cmake
    assert "simulation-only reconstruction" in readme
    assert "must be replaced or reconciled" in readme
    assert "supplied URDF does not provide a reviewed wheel torque limit" in readme


def test_goals_and_world_are_deterministic():
    config = yaml.safe_load(CONFIG_PATH.read_text())
    assert config["goals"] == {
        "home": {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
        "sim_goal1": {"x": 2.0, "y": 0.0, "yaw_deg": 0.0},
        "sim_goal2": {"x": 0.5, "y": 0.0, "yaw_deg": 0.0},
    }
    assert config["physics"]["wheel_separation_m"] == 0.461
    assert config["physics"]["wheel_radius_m"] == 0.08
    assert config["actuators"] == {
        "left_wheel_motor": {"max_velocity_radps": 25.0, "max_torque_nm": 50.0},
        "right_wheel_motor": {"max_velocity_radps": 25.0, "max_torque_nm": 50.0},
    }
    assert config["scripts"]["smoke"] == [
        {"action": "nav_to_pose", "goal_name": "sim_goal1"},
        {"action": "move_distance", "distance": -1.5, "angle": 0.0},
        {"action": "move_distance", "distance": 1.2, "angle": 0.0},
        {"action": "move_distance", "distance": -1.2, "angle": 0.0},
        {"action": "nav_to_pose", "goal_name": "sim_goal2"},
    ]
    assert config["scripts"]["blocked"] == config["scripts"]["smoke"]


def test_match_fragment_preserves_wahrg2026_navigation_coordinates():
    config = yaml.safe_load(MATCH_CONFIG_PATH.read_text(encoding="utf-8"))
    tree = ElementTree.fromstring(MATCH_TREE_PATH.read_text(encoding="utf-8"))
    actions = list(tree)

    assert config["goals"] == {
        "home": {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
        "搬箱子1": {
            "x": 2.024931001163208,
            "y": -2.4938457012176514,
            "yaw_deg": 178.927596408134,
        },
        "放箱子1_1": {
            "x": 2.460558853149414,
            "y": -2.6341702938073354,
            "yaw_deg": 0.35089784218538095,
        },
    }
    assert config["scripts"]["match"] == [
        {"action": "nav_to_pose", "goal_name": "搬箱子1"},
        {"action": "move_distance", "distance": -0.6, "angle": 0.0},
        {"action": "move_distance", "distance": 0.0, "angle": 90.0},
        {"action": "nav_to_pose", "goal_name": "放箱子1_1"},
    ]
    assert [action.tag.rsplit(".", 1)[-1] for action in actions] == [
        "NavToPose",
        "MoveDistance",
        "MoveDistance",
        "NavToPose",
    ]
    assert actions[0].attrib["goal_name"] == "搬箱子1"
    assert (actions[1].attrib["distance"], actions[1].attrib["angle"]) == ("-0.6", "0")
    assert (actions[2].attrib["distance"], actions[2].attrib["angle"]) == ("0.0", "90")
    assert actions[3].attrib["goal_name"] == "放箱子1_1"


def test_config_contains_exact_smoke_timing_and_motion_values():
    config = yaml.safe_load(CONFIG_PATH.read_text())

    assert config["motion"] == {
        "max_linear_velocity_mps": 2.0,
        "max_angular_velocity_radps": 1.5,
        "max_linear_acceleration_mps2": 1.0,
        "max_angular_acceleration_radps2": 1.0,
        "smoke_linear_velocity_mps": 0.3,
        "smoke_angular_velocity_radps": 0.5,
        "smoke_linear_acceleration_mps2": 0.5,
        "smoke_angular_acceleration_radps2": 1.0,
        "position_tolerance_m": 0.05,
        "yaw_tolerance_rad": 0.03490658503988659,
        "stall_window_s": 1.0,
        "yaw_stall_threshold_rad": 0.008726646259971648,
        "distance_stall_threshold_m": 0.005,
        "total_budget_s": 60.0,
        "timeout_factor": 3.0,
        "timeout_margin_s": 2.0,
        "minimum_stage_timeout_s": 5.0,
    }
    assert config["world"] == {
        "arena": {
            "length_m": 8.0,
            "width_m": 5.0,
            "boundary_x_m": 4.0,
            "boundary_y_m": 2.5,
        },
        "basic_time_step_ms": 16,
        "controller_step_ms": 32,
    }
    assert config["timeouts"] == {
        "action_budget_s": 60.0,
        "readiness_budget_s": 15.0,
        "readiness_settle_s": 1.0,
    }


def world_common_contract(world: str) -> None:
    assert world.startswith("#VRML_SIM R2025a utf8\n")
    assert "basicTimeStep 16" in world
    assert world.count("maxVelocity 25") == 2
    assert world.count("maxTorque 50") == 2
    assert 'name "left_wheel_motor"' in world
    assert 'name "right_wheel_motor"' in world
    assert 'name "left_wheel"' in world
    assert 'name "right_wheel"' in world

    for name in ("arm", "gripper", "lift"):
        assert name not in world.lower()

    assert "DEF EA200_SMOKE Robot {" in world
    assert "supervisor TRUE" in world
    assert 'controller "<extern>"' in world
    assert "synchronization TRUE" in world
    assert "translation 0 0 0.08" in world
    assert "rotation 0 0 1 0" in world
    assert "anchor 0 0.2305 0" in world
    assert "anchor 0 -0.2305 0" in world
    assert "axis 0 1 0" in world
    assert "anchor -0.436 0.18 -0.0425" in world
    assert "anchor -0.436 -0.18 -0.0425" in world
    assert world.count("HingeJoint {") == 2
    assert world.count("RotationalMotor {") == 2
    assert world.count("BallJoint {") == 2
    assert "DEF FLOOR Solid {" in world
    assert 'contactMaterial "floor"' in world
    assert world.count('contactMaterial "wheel"') == 2
    assert world.count('contactMaterial "caster"') == 2
    assert 'material1 "wheel"' in world
    assert 'material2 "floor"' in world
    assert 'coulombFriction [ 100.0 ]' in world
    assert 'material1 "caster"' in world
    assert 'coulombFriction [ 0.05 ]' in world
    assert "EXTERNPROTO" not in world
    assert "TexturedBackground" not in world
    assert "RectangleArena" not in world
    assert "size 0.70 0.60 0.35" in world
    assert "size 0.50 0.34 0.97" in world
    assert "ROS_DOMAIN_ID=3" not in world


def test_smoke_world_has_basic_geometry_and_only_wheel_motors():
    world_common_contract(SMOKE_WORLD_PATH.read_text())
    world = SMOKE_WORLD_PATH.read_text()
    assert "Mesh {" not in world
    assert "url" not in world.lower()
    assert "SMOKE_OBSTACLE" not in world
    assert "size 8 5" in world


def test_blocked_world_adds_only_the_specified_obstacle():
    world_common_contract(BLOCKED_WORLD_PATH.read_text())
    world = BLOCKED_WORLD_PATH.read_text()
    assert "Mesh {" not in world
    assert "url" not in world.lower()
    assert "DEF SMOKE_OBSTACLE Solid {" in world
    assert "translation 1.0 0.0 0.25" in world
    assert "size 0.10 1.00 0.50" in world
    assert "size 8 5" in world


def test_match_world_extends_width_without_adding_obstacles():
    world = MATCH_WORLD_PATH.read_text(encoding="utf-8")
    world_common_contract(world)
    assert "Mesh {" not in world
    assert "url" not in world.lower()
    assert "SMOKE_OBSTACLE" not in world
    assert "size 8 6" in world


def _service():
    compose = yaml.safe_load(COMPOSE_PATH.read_text())
    return compose["services"]["lynsense-webots-smoke"]


def test_compose_is_isolated_and_read_only():
    service = _service()
    assert service["network_mode"] == "none"
    assert service["environment"]["ROS_DOMAIN_ID"] == "42"
    assert service["environment"]["ROS_LOCALHOST_ONLY"] == "1"
    volumes = service["volumes"]
    source_mounts = [
        volume for volume in volumes
        if volume["target"] in {"/workspace/simulation", "/workspace/ws/src/lynsense_pytrees"}
    ]
    assert len(source_mounts) == 2
    assert all(volume["read_only"] is True for volume in source_mounts)
    assert "ports" not in service

    by_target = {volume["target"]: volume for volume in volumes}
    assert by_target["/workspace/simulation"]["source"] == ".."
    assert by_target["/workspace/ws/src/lynsense_pytrees"]["source"] == "../../../../../lynsense_pytrees"
    assert by_target["/workspace/artifacts"]["read_only"] is False


def test_compose_mounts_only_permitted_paths_and_writes_to_artifacts():
    service = _service()
    forbidden = {".env.lynsense", ".ssh", "/home/rpp", "robot"}
    for volume in service["volumes"]:
        assert not any(value in volume["source"] for value in forbidden)
        assert volume["target"] in {
            "/workspace/simulation",
            "/workspace/ws/src/lynsense_pytrees",
            "/workspace/artifacts",
        }


def test_compose_starts_mounted_bootstrap_in_docker_context():
    service = _service()
    assert service["build"]["context"] == "."
    assert service["build"]["dockerfile"] == "Dockerfile"
    assert service["entrypoint"] == [
        "python3",
        "/workspace/simulation/docker/bootstrap.py",
    ]
    assert service["command"] == ["--phase", "all"]
    assert service["environment"]["WEBOTS_HOME"]
    assert (
        service["environment"]["LYNSENSE_SIM_CONFIG"]
        == "/workspace/simulation/ros/lynsense_webots_sim/config/smoke.yaml"
    )


def test_gui_override_only_adds_x11_read_only():
    override = yaml.safe_load(COMPOSE_GUI_PATH.read_text())
    service = override["services"]["lynsense-webots-smoke"]
    assert service["network_mode"] == "none"
    assert service["volumes"] == [
        {
            "source": "/tmp/.X11-unix",
            "target": "/tmp/.X11-unix",
            "type": "bind",
            "read_only": True,
        }
    ]


def test_dockerfile_pins_verified_image_and_installs_humble():
    dockerfile = DOCKERFILE_PATH.read_text()
    assert dockerfile.startswith(
        "FROM cyberbotics/webots@sha256:f0023e30daf38b172e4e6ad24ed345909bcd9551df34d63d824e121a7cebf099"
    )
    assert "packages.ros.org/ros2/ubuntu jammy main" in dockerfile
    for package in (
        "ros-humble-ros-base",
        "ros-humble-py-trees",
        "ros-humble-py-trees-ros",
        "python3-colcon-common-extensions",
        "xvfb",
    ):
        assert package in dockerfile
    assert "apt-get upgrade" not in dockerfile
    assert "USER lynsense" in dockerfile
    assert "--uid 1000" in dockerfile
    assert "COPY" not in dockerfile
    assert "chown -R lynsense:lynsense /workspace" in dockerfile
    assert "/workspace/artifacts" in dockerfile


def test_dockerignore_exposes_only_dockerfile():
    entries = [
        line for line in DOCKERIGNORE_PATH.read_text().splitlines()
        if line.strip() and not line.startswith("#")
    ]
    assert entries == ["**", "!Dockerfile"]


def test_bootstrap_builds_ephemeral_overlay_without_touching_read_only_source():
    source = BOOTSTRAP_PATH.read_text()
    assert "def _bash" in source
    assert '"/bin/bash", "-lc"' in source
    assert "/opt/ros/humble/setup.bash" in source
    for name in ("lynsense_utils_compat", "lynsense_webots_sim"):
        assert name in source
        copied_name = name if name != "lynsense_utils_compat" else "lynsense_utils"
        assert f"/workspace/ws/src/{copied_name}" in source
    assert "/workspace/ws/src/lynsense_pytrees" in source
    assert "colcon build --packages-select lynsense_utils lynsense_webots_sim lynsense_pytrees" in source
    assert "lynsense_webots_controller" in source
    assert "os.access" in source
    assert "import_module" in source
    assert "lynsense_utils" in source
    assert "lynsense_webots_sim" in source
    assert "lynsense_pytrees" in source
    assert "OVERLAY_ROOT / 'lynsense_utils'" in source
    assert "build-summary.json" in source
    assert "build.json" not in source
    assert "\"rpent-box\"" in source
    assert "import shlex" in source
    assert "--plan {shlex.quote(str(plan_path))}" in source
    assert "lynsense_match_min_tree.xml" in source
    assert "shutil.copy2" in source


def test_bootstrap_sources_overlay_before_delegating_installed_command():
    source = BOOTSTRAP_PATH.read_text()
    assert "exec lynsense_run_smoke --phase" in source
    assert "source /workspace/ws/install/setup.bash" in source
    assert "shlex.quote" in source


def test_bootstrap_delegates_rpent_box_plan_path_safely():
    bootstrap = _load_bootstrap()
    command = bootstrap._overlay_source_command(
        "rpent-box", Path("/workspace/artifacts/rpent single-box plan.json")
    )
    assert command.endswith(
        "exec lynsense_run_smoke --phase rpent-box "
        "--plan '/workspace/artifacts/rpent single-box plan.json'"
    )
    with pytest.raises(ValueError, match="requires a plan path"):
        bootstrap._overlay_source_command("rpent-box")
    with pytest.raises(ValueError, match="only rpent-box"):
        bootstrap._overlay_source_command("box", Path("/workspace/artifacts/plan.json"))


def test_readme_documents_rpent_plan_gate_and_its_boundaries():
    readme = SIMULATION_README_PATH.read_text(encoding="utf-8")
    assert "## RPent Single-Box Plan Gate" in readme
    assert "--robot lynsense_simulation" in readme
    assert "--phase rpent-box" in readme
    assert "read_simulation_task" in readme
    assert "submit_simulation_plan" in readme
    assert "real `lynsense` backend remains read-only" in readme
    assert "without contacting an external model" in readme
    assert "not prove autonomous perception" in readme


def test_action_probe_checks_invalid_and_concurrent_goals():
    source = PROBE_PATH.read_text()
    assert "45.0" in source
    assert '"/lynsense/nav_to_pose"' in source
    assert '"/lynsense/move_distance"' in source
    assert "math.isnan" in source
    assert "math.isinf" in source
    assert "terminal_lockout" not in source
    assert "cancel" in source
    assert "node.spin_once" not in source
    assert "SingleThreadedExecutor()" in source
    assert "executor.add_node(node)" in source
    assert "executor.spin_once(timeout_sec=0)" in source


def test_interface_gate_runs_both_probe_modes_and_cancel_uses_long_goal():
    probe_tree = ast.parse(PROBE_PATH.read_text(), filename=str(PROBE_PATH))
    cancel_function = next(
        node
        for node in probe_tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_run_cancel_check"
    )
    cancel_angles = [
        ast.literal_eval(node.value)
        for node in ast.walk(cancel_function)
        if isinstance(node, ast.Assign)
        and isinstance(node.targets[0], ast.Attribute)
        and node.targets[0].attr == "angle"
    ]
    assert cancel_angles == [180.0]
    assert 'feedback.get("angle_remaining_deg", 0.0) < 90.0' in PROBE_PATH.read_text()
    assert "get_status_async" not in PROBE_PATH.read_text()
    assert "cancel_response.return_code != 0" in PROBE_PATH.read_text()
    assert "result.status != GoalStatus.STATUS_CANCELED" in PROBE_PATH.read_text()
    cancel_source = ast.unparse(cancel_function)
    assert cancel_source.index("get_result_async()") < cancel_source.index(
        "cancel_goal_async()"
    )

    orchestrator_tree = ast.parse(
        ORCHESTRATOR_PATH.read_text(), filename=str(ORCHESTRATOR_PATH)
    )
    run_phase = next(
        node
        for node in orchestrator_tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_run_phase"
    )
    probe_modes = [
        ast.literal_eval(call.args[-1])
        for call in ast.walk(run_phase)
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
        and call.func.id == "_run_probe"
    ]
    assert probe_modes == ["interface", "cancel"]
    process_starts = [
        call
        for call in ast.walk(run_phase)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "_start_phase_processes"
    ]
    assert len(process_starts) == 2
    assert ORCHESTRATOR_PATH.read_text().count('.open("ab")') == 3


def test_controller_ros_endpoints_use_lynsense_namespace_and_events_stay_logical():
    source = _controller_source()
    for endpoint in (
        "/lynsense/nav_to_pose",
        "/lynsense/move_distance",
        "/lynsense/move_waist",
        "/lynsense/move_named_config",
        "/lynsense/set_gripper",
        "/lynsense/box_phase",
    ):
        assert endpoint in source
    assert 'ACTION_ENDPOINTS = {' in source
    assert '"/lynsense/nav_to_pose"' in source
    assert '"/lynsense/move_distance"' in source
    assert source.count('ACTION_ENDPOINTS[kind]') == 1
    assert '[binding.action_name for binding in self._action_server_bindings]' in source


def test_action_probe_submits_concurrent_goals_before_waiting_for_responses():
    tree = ast.parse(PROBE_PATH.read_text(), filename=str(PROBE_PATH))
    interface = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_run_interface_checks"
    )
    send_calls = [
        node
        for node in ast.walk(interface)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "send_goal_async"
        and ast.unparse(node.func.value) == "move_client"
    ]
    spin_calls = [
        node
        for node in ast.walk(interface)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "_spin"
    ]

    assert len(send_calls) == 2
    assert len(spin_calls) == 3
    assert max(node.lineno for node in send_calls) < min(
        node.lineno for node in spin_calls
    )
    assert [ast.unparse(node.args[0]) for node in send_calls] == [
        "concurrent",
        "concurrent",
    ]


@pytest.mark.parametrize("mode", ["interface", "cancel"])
def test_action_probe_parser_dispatches_without_rclpy(mode, monkeypatch):
    from lynsense_webots_sim.scripts import action_probe

    calls = []
    monkeypatch.setattr("sys.argv", ["lynsense_action_probe", "--mode", mode])
    monkeypatch.setattr(
        action_probe,
        "_run_interface_checks",
        lambda: calls.append("interface"),
    )
    monkeypatch.setattr(
        action_probe,
        "_run_cancel_check",
        lambda: calls.append("cancel"),
    )

    assert action_probe.main() == 0
    assert calls == [mode]


def test_orchestrator_uses_exact_startup_and_deadlines():
    source = ORCHESTRATOR_PATH.read_text()
    ast.parse(source, filename=str(ORCHESTRATOR_PATH))
    assert "xvfb-run" in source
    assert "--batch" in source and "--mode=run" in source
    assert "$WEBOTS_HOME/webots-controller" not in source
    assert "webots-controller" in source
    assert "120.0" in source
    assert "30.0" in source
    assert "terminal_lockout" in source
    assert "run_started" in source
    assert "controller_ready" in source
    assert "0.1" in source
    assert "Popen" in source


def test_orchestrator_records_expected_smoke_sequence():
    source = ORCHESTRATOR_PATH.read_text()
    expected = [
        ("nav_to_pose", "sim_goal1"),
        ("move_distance", "-1.5"),
        ("move_distance", "1.2"),
        ("move_distance", "-1.2"),
        ("nav_to_pose", "sim_goal2"),
    ]
    lines = source.splitlines()
    positions = []
    for action, goal in expected:
        matches = [index for index, line in enumerate(lines) if action in line and goal in line]
        assert matches
        positions.append(matches[-1])
    assert positions == sorted(positions)
    assert "archive" in source
    assert "uuid.uuid4" in source
    assert "summary.json" in source


def test_launch_file_uses_installer_controller_path():
    launch = LAUNCH_PATH.read_text()
    assert "lynsense_webots_sim" in launch
    assert "lynsense_webots_controller" in launch


def test_simulation_readme_documents_isolation_and_boundaries():
    readme = SIMULATION_README_PATH.read_text()
    for required in (
        "--phase build",
        "--phase interface",
        "--phase smoke",
        "--phase blocked",
        "--phase match",
        "compose.gui.yml",
        "network_mode: none",
        "summary.json",
        "build-summary.json",
        "interface-summary.json",
        "read-only",
        "must not be interpreted as authorization",
        "--mode=run",
    ):
        assert required in readme


def test_repository_ignores_simulation_artifacts():
    assert ".artifacts/" in (REPO_ROOT / ".gitignore").read_text().splitlines()


def _write_events(path, events):
    path.write_text(
        "".join(json.dumps(event, separators=(",", ":")) + "\n" for event in events),
        encoding="utf-8",
    )


def _readiness_events(**overrides):
    ready = {
        "kind": "controller_ready",
        "action": "controller",
        "sim_time_s": 0.0,
        "action_names": ["nav_to_pose", "move_distance"],
        "action_types": {
            "nav_to_pose": "lynsense_utils/action/NavToPose",
            "move_distance": "lynsense_utils/action/MoveDistance",
        },
        "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "motor_rates": {
            "left_wheel_rate_radps": 0.0,
            "right_wheel_rate_radps": 0.0,
        },
        "run_id": "ready-run",
    }
    ready.update(overrides)
    return [
        {
            "kind": "run_started",
            "action": "controller",
            "sim_time_s": 0.0,
            "run_id": "ready-run",
        },
        ready,
    ]


def test_readiness_returns_durable_cursor_and_validates_action_types(tmp_path, monkeypatch):
    from lynsense_webots_sim.scripts import run_smoke

    event_path = tmp_path / "events.jsonl"
    _write_events(event_path, _readiness_events())
    processes = run_smoke.PhaseProcesses(_OrchestratorProcessFake(), _OrchestratorProcessFake())
    monkeypatch.setattr(run_smoke.time, "sleep", lambda _seconds: None)

    cursor = run_smoke._wait_for_ready(processes, event_path, "ready-run")
    assert cursor.path == event_path
    assert cursor.offset == event_path.stat().st_size

    wrong_types = _readiness_events(
        action_types={
            "nav_to_pose": "lynsense_utils/action/Wrong",
            "move_distance": "lynsense_utils/action/MoveDistance",
        }
    )
    rejection_path = tmp_path / "wrong-types.jsonl"
    _write_events(rejection_path, wrong_types)
    with pytest.raises(run_smoke.SmokeError, match="action types"):
        run_smoke._wait_for_ready(
            processes, rejection_path, "ready-run"
        )


def test_readiness_rejects_a_different_run_id(tmp_path, monkeypatch):
    from lynsense_webots_sim.scripts import run_smoke

    event_path = tmp_path / "wrong-run-id.jsonl"
    _write_events(event_path, _readiness_events())
    processes = run_smoke.PhaseProcesses(_OrchestratorProcessFake(), _OrchestratorProcessFake())
    monkeypatch.setattr(run_smoke.time, "sleep", lambda _seconds: None)

    with pytest.raises(run_smoke.SmokeError, match="missing or different run_id"):
        run_smoke._wait_for_ready(processes, event_path, "new-run")


def test_readiness_validates_lines_after_controller_ready_in_same_read(tmp_path, monkeypatch):
    from lynsense_webots_sim.scripts import run_smoke

    event_path = tmp_path / "trailing-bad-run-id.jsonl"
    _write_events(
        event_path,
        _readiness_events()
        + [
            {
                "kind": "action_feedback",
                "action": "controller",
                "sim_time_s": 0.1,
                "run_id": "different-run",
            }
        ],
    )
    processes = run_smoke.PhaseProcesses(_OrchestratorProcessFake(), _OrchestratorProcessFake())
    monkeypatch.setattr(run_smoke.time, "sleep", lambda _seconds: None)

    with pytest.raises(run_smoke.SmokeError, match="missing or different run_id"):
        run_smoke._wait_for_ready(processes, event_path, "ready-run")


def test_readiness_cursor_hands_off_only_new_events_to_watchdog(tmp_path, monkeypatch):
    from lynsense_webots_sim.scripts import run_smoke

    event_path = tmp_path / "handoff.jsonl"
    _write_events(event_path, _readiness_events())
    processes = run_smoke.PhaseProcesses(_OrchestratorProcessFake(), _OrchestratorProcessFake())
    monkeypatch.setattr(run_smoke.time, "sleep", lambda _seconds: None)
    cursor = run_smoke._wait_for_ready(processes, event_path, "ready-run")

    tree = _OrchestratorProcessFake()
    blocked_processes = run_smoke.PhaseProcesses(
        _OrchestratorProcessFake(),
        _OrchestratorProcessFake(),
        tree,
    )
    _write_events(
        event_path,
        _readiness_events()
        + [
            {
                "kind": "goal_accepted",
                "action": "nav_to_pose",
                "goal_name": "sim_goal1",
                "run_id": "ready-run",
            },
            {
                "kind": "action_result",
                "action": "nav_to_pose",
                    "goal_name": "sim_goal1",
                    "success": False,
                    "reason": "translate_stalled",
                    "pose": {"x": 0.9, "y": 0.0, "yaw": 0.0},
                "motor_rates": {
                    "left_wheel_rate_radps": 0.0,
                    "right_wheel_rate_radps": 0.0,
                },
                "run_id": "ready-run",
            },
        ],
    )
    monkeypatch.setattr(
        run_smoke,
        "_signal_process_group",
        lambda process, signal_value: (
            tree.interrupt_signals.append(signal_value),
            setattr(tree, "exit_code", 0),
        )[0],
    )
    run_smoke._watch_tree(
        tree,
        blocked_processes,
        cursor,
        blocked=True,
    )
    assert tree.interrupt_signals == [run_smoke.signal.SIGINT]


def test_tree_validator_filters_feedback_and_enforces_terminal_lockout():
    from lynsense_webots_sim.scripts import run_smoke

    validator = run_smoke.TreeEventValidator(blocked=False)
    script = [
        ("goal_accepted", "nav_to_pose", "sim_goal1"),
        ("action_result", "nav_to_pose", "sim_goal1"),
        ("goal_accepted", "move_distance", "-1.5"),
        ("action_result", "move_distance", "-1.5"),
        ("goal_accepted", "move_distance", "1.2"),
        ("action_result", "move_distance", "1.2"),
        ("goal_accepted", "move_distance", "-1.2"),
        ("action_result", "move_distance", "-1.2"),
        ("goal_accepted", "nav_to_pose", "sim_goal2"),
        ("action_result", "nav_to_pose", "sim_goal2"),
    ]
    for index, (kind, action, goal) in enumerate(script):
        if action == "nav_to_pose":
            event = {"kind": kind, "action": action, "goal_name": goal}
        else:
            event = {
                "kind": kind,
                "action": action,
                "distance_m": float(goal),
                "angle_deg": 0.0,
            }
        if kind == "action_result":
            event.update(
                {
                    "success": True,
                    "pose": {"x": 0.5, "y": 0.0, "yaw": 0.0},
                    "motor_rates": {
                        "left_wheel_rate_radps": 0.0,
                        "right_wheel_rate_radps": 0.0,
                    },
                }
            )
        else:
            event["pose"] = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        if index == 1:
            feedback = {
                "kind": "action_feedback",
                "action": "nav_to_pose",
                "sim_time_s": 1.0,
                "pose": {"x": 0.1, "y": 0.0, "yaw": 0.0},
                "motor_rates": {
                    "left_wheel_rate_radps": 1.0,
                    "right_wheel_rate_radps": 1.0,
                },
            }
            assert validator.feed(feedback) is False
        assert validator.feed(event) is (index == len(script) - 1)

    assert validator.terminal
    assert validator.feed(
        {"kind": "goal_rejected", "reason": "terminal_lockout"}
    ) is True
    with pytest.raises(run_smoke.SmokeError, match="post-terminal rejection"):
        validator.feed({"kind": "goal_rejected", "reason": "other"})
    with pytest.raises(run_smoke.SmokeError, match="post-terminal action"):
        validator.feed({"kind": "goal_accepted", "action": "move_distance"})
    with pytest.raises(run_smoke.SmokeError, match="new motor command"):
        validator.feed(
            {
                "kind": "action_feedback",
                "pose": {"x": 0.5, "y": 0.0, "yaw": 0.0},
                "motor_rates": {
                    "left_wheel_rate_radps": 0.1,
                    "right_wheel_rate_radps": 0.0,
                },
            }
        )


def _expected_event_fields(expected):
    action, goal_name, move = expected
    fields = {"action": action}
    if goal_name is not None:
        fields["goal_name"] = goal_name
    if move is not None:
        fields["distance_m"] = move[0]
        fields["angle_deg"] = move[1]
    return fields


def test_match_validator_accepts_exact_wahrg2026_fragment_and_final_pose():
    from lynsense_webots_sim.scripts import run_smoke

    validator = run_smoke.TreeEventValidator(
        blocked=False,
        expected_script=run_smoke.MATCH_EXPECTED_SCRIPT,
        final_pose=run_smoke.MATCH_FINAL_POSE,
    )

    for expected in run_smoke.MATCH_EXPECTED_SCRIPT[:3]:
        accepted = {
            "kind": "goal_accepted",
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            **_expected_event_fields(expected),
        }
        result = {
            "kind": "action_result",
            "success": True,
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "motor_rates": {
                "left_wheel_rate_radps": 0.0,
                "right_wheel_rate_radps": 0.0,
            },
            **_expected_event_fields(expected),
        }
        assert validator.feed(accepted) is False
        assert validator.feed(result) is False

    final = run_smoke.MATCH_EXPECTED_SCRIPT[3]
    assert validator.feed(
        {
            "kind": "goal_accepted",
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            **_expected_event_fields(final),
        }
    ) is False
    assert validator.feed(
        {
            "kind": "action_result",
            "success": True,
            "pose": {
                "x": run_smoke.MATCH_FINAL_POSE[0] + 0.04,
                "y": run_smoke.MATCH_FINAL_POSE[1] - 0.04,
                "yaw": run_smoke.MATCH_FINAL_POSE[2] + 0.03,
            },
            "motor_rates": {
                "left_wheel_rate_radps": 0.0,
                "right_wheel_rate_radps": 0.0,
            },
            **_expected_event_fields(final),
        }
    ) is True


def test_watch_tree_checks_child_liveness_before_waiting(tmp_path):
    from lynsense_webots_sim.scripts import run_smoke

    dead_tree = _OrchestratorProcessFake(exit_code=1)
    processes = run_smoke.PhaseProcesses(
        _OrchestratorProcessFake(),
        _OrchestratorProcessFake(),
        dead_tree,
    )
    event_path = tmp_path / "events.jsonl"
    event_path.touch()
    with pytest.raises(run_smoke.SmokeError, match="exited before the terminal event"):
        run_smoke._watch_tree(
            dead_tree,
            processes,
            run_smoke.EventCursor(event_path, 0, "ready-run"),
            blocked=False,
        )


def test_partial_controller_startup_cleans_webots(monkeypatch):
    from lynsense_webots_sim.scripts import run_smoke

    webots = _OrchestratorProcessFake()
    monkeypatch.setattr(
        run_smoke,
        "_start_webots",
        lambda _phase, _environment, _run_id: webots,
    )

    def fail_controller(_environment, _phase, _run_id):
        raise run_smoke.SmokeError("controller launcher missing")

    monkeypatch.setattr(run_smoke, "_start_controller", fail_controller)
    monkeypatch.setattr(
        run_smoke.os,
        "killpg",
        lambda _process_group, signal_value: (
            webots.interrupt_signals.append(signal_value),
            setattr(webots, "exit_code", 0),
        )[0],
    )
    with pytest.raises(run_smoke.SmokeError, match="controller launcher missing"):
        run_smoke._start_phase_processes("smoke", {}, "startup-run")
    assert webots.interrupt_signals == [run_smoke.signal.SIGINT]
    assert webots.terminate_signals == []


def test_tree_validator_accepts_exact_first_blocked_failure():
    from lynsense_webots_sim.scripts import run_smoke

    validator = run_smoke.TreeEventValidator(blocked=True)
    accepted = {
        "kind": "goal_accepted",
        "action": "nav_to_pose",
        "goal_name": "sim_goal1",
    }
    failed = {
        "kind": "action_result",
        "action": "nav_to_pose",
        "goal_name": "sim_goal1",
        "success": False,
        "reason": "translate_stalled",
        "pose": {"x": 0.9, "y": 0.0, "yaw": 0.0},
        "motor_rates": {
            "left_wheel_rate_radps": 0.0,
            "right_wheel_rate_radps": 0.0,
        },
    }
    assert validator.feed(accepted) is False
    assert validator.feed(failed) is True

    unexpected = dict(failed, reason="invalid_pose")
    unexpected.pop("pose")
    with pytest.raises(run_smoke.SmokeError, match="expected translate stall"):
        unexpected_validator = run_smoke.TreeEventValidator(blocked=True)
        unexpected_validator.feed(accepted)
        unexpected_validator.feed(unexpected)


def test_orchestrator_uses_process_groups():
    source = ORCHESTRATOR_PATH.read_text()
    assert source.count("start_new_session=True") >= 3
    assert "os.killpg" in source


def _load_bootstrap():
    spec = importlib.util.spec_from_file_location(
        "task5_bootstrap_test", BOOTSTRAP_PATH
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_bootstrap_build_phase_only_builds(monkeypatch):
    bootstrap = _load_bootstrap()
    built = []
    monkeypatch.setattr(bootstrap, "_build", lambda: built.append(True))
    monkeypatch.setattr(
        "sys.argv", ["bootstrap.py", "--phase", "build"]
    )

    assert bootstrap.main() == 0
    assert built == [True]


def test_bootstrap_rebuilds_and_delegates_each_non_build_phase(monkeypatch):
    bootstrap = _load_bootstrap()
    built = []
    commands = []
    monkeypatch.setattr(bootstrap, "_build", lambda: built.append(True))
    monkeypatch.setattr(
        bootstrap,
        "_bash",
        lambda command: (commands.append(command), (0, ""))[1],
    )

    for phase in ("interface", "smoke", "blocked", "all"):
        monkeypatch.setattr("sys.argv", ["bootstrap.py", "--phase", phase])
        assert bootstrap.main() == 0

    assert len(built) == 4
    assert commands == [
        "source /opt/ros/humble/setup.bash && "
        "source /workspace/ws/install/setup.bash && "
        f"exec lynsense_run_smoke --phase {phase}"
        for phase in ("interface", "smoke", "blocked", "all")
    ]


def test_bootstrap_build_evidence_serializes_source_boundaries():
    bootstrap = _load_bootstrap()
    pre = {
        "ROS_DISTRO": "",
        "ROS_VERSION": "",
        "ROS_PYTHON_VERSION": "",
        "ROS_DOMAIN_ID": "42",
        "ROS_LOCALHOST_ONLY": "1",
        "AMENT_PREFIX_PATH": "",
        "ROS_PACKAGE_PATH": "",
    }
    post = {
        "ROS_DISTRO": "humble",
        "ROS_VERSION": "2",
        "ROS_PYTHON_VERSION": "3",
        "ROS_DOMAIN_ID": "42",
        "ROS_LOCALHOST_ONLY": "1",
        "AMENT_PREFIX_PATH": "/opt/ros/humble:/workspace/ws/install",
        "ROS_PACKAGE_PATH": "/workspace/ws/src",
    }
    evidence = bootstrap._build_evidence(pre, post)

    assert evidence["result"] == "passed"
    assert evidence["ros_environment"] == post
    assert evidence["pre_source"] == {
        "AMENT_PREFIX_PATH": "",
        "ROS_PACKAGE_PATH": "",
    }
    assert evidence["post_source"] == {
        "AMENT_PREFIX_PATH": post["AMENT_PREFIX_PATH"],
        "ROS_PACKAGE_PATH": post["ROS_PACKAGE_PATH"],
    }
    assert evidence["controller"] == str(bootstrap.CONTROLLER_PATH)
    assert json.dumps(evidence, sort_keys=True, allow_nan=False)


def test_runtime_isolation_evidence_serializes_and_rejects_robot_paths():
    from lynsense_webots_sim.scripts import run_smoke

    environment = {
        "ROS_DOMAIN_ID": "42",
        "ROS_LOCALHOST_ONLY": "1",
        "AMENT_PREFIX_PATH": "/workspace/ws/install",
        "ROS_PACKAGE_PATH": "/workspace/ws/src",
    }
    imports = {
        "rclpy": True,
        "py_trees": True,
        "py_trees_ros": True,
        "yaml": True,
    }
    evidence = run_smoke._runtime_isolation_evidence(
        environment, "/workspace/ws/install/lynsense_utils", imports
    )
    assert evidence["ROS_DOMAIN_ID"] == "42"
    assert evidence["ROS_LOCALHOST_ONLY"] == "1"
    assert evidence["overlay_prefix"] == "/workspace/ws/install"
    assert evidence["ament_prefix_path"] == environment["AMENT_PREFIX_PATH"]
    assert evidence["ros_package_path"] == environment["ROS_PACKAGE_PATH"]
    assert evidence["ros2_pkg_prefix_lynsense_utils"] == (
        "/workspace/ws/install/lynsense_utils"
    )
    assert evidence["imports"] == imports
    assert evidence["forbidden_paths"] == []
    assert json.dumps(evidence, sort_keys=True, allow_nan=False)

    contaminated_environment = {
        **environment,
        "AMENT_PREFIX_PATH": "/home/rpp/rpp_ws/install",
    }
    contaminated = run_smoke._runtime_isolation_evidence(
        contaminated_environment,
        "/home/rpp/rpp_ws/install/lynsense_utils",
        imports,
    )
    assert contaminated["forbidden_paths"] == ["/home/rpp", "rpp_ws"]
    with pytest.raises(run_smoke.SmokeError, match="robot workspace path"):
        run_smoke._validate_runtime_isolation(contaminated)


def test_interface_summary_serializes_runtime_evidence(tmp_path, monkeypatch):
    from lynsense_webots_sim.scripts import run_smoke

    monkeypatch.setattr(run_smoke, "ARTIFACT_ROOT", tmp_path)
    runtime_isolation = run_smoke._runtime_isolation_evidence(
        {
            "ROS_DOMAIN_ID": "42",
            "ROS_LOCALHOST_ONLY": "1",
            "AMENT_PREFIX_PATH": "/workspace/ws/install",
            "ROS_PACKAGE_PATH": "/workspace/ws/src",
        },
        "/workspace/ws/install/lynsense_utils",
        {name: True for name in run_smoke.RUNTIME_IMPORTS},
    )
    result = {
        "phase": "interface",
        "run_id": "interface-run",
        "event_file": "events-interface.jsonl",
        "outcome": "passed",
        "runtime_isolation": runtime_isolation,
    }

    run_smoke._write_summary([result])
    interface_summary = json.loads(
        (tmp_path / "interface-summary.json").read_text(encoding="utf-8")
    )
    assert interface_summary == result
    assert json.dumps(interface_summary, sort_keys=True, allow_nan=False)


def test_bootstrap_verifies_python_packages_and_pytrees_artifacts(monkeypatch):
    bootstrap = _load_bootstrap()
    commands = []
    good_evidence = {
        "lynsense_utils_file": "/workspace/ws/install/lynsense_utils/lib/python3.10/site-packages/lynsense_utils/__init__.py",
        "lynsense_webots_sim_file": "/workspace/ws/install/lynsense_webots_sim/lib/python3.10/site-packages/lynsense_webots_sim/__init__.py",
        "pytrees_share": "/workspace/ws/install/lynsense_pytrees/share/lynsense_pytrees",
        "pytrees_config": "/workspace/ws/install/lynsense_pytrees/share/lynsense_pytrees/config/move_points.yaml",
        "pytrees_tree": "/workspace/ws/install/lynsense_pytrees/share/lynsense_pytrees/trees/lynsense_webots_tree.xml",
        "pytrees_match_tree": "/workspace/ws/install/lynsense_pytrees/share/lynsense_pytrees/trees/lynsense_match_min_tree.xml",
            "pytrees_node": "/workspace/ws/install/lynsense_pytrees/lib/lynsense_pytrees/lynsense_pytrees_node.py",
            "single_box_tree_executable": "/workspace/ws/install/lynsense_webots_sim/bin/lynsense_single_box_tree",
            "single_box_tree": "/workspace/ws/install/lynsense_webots_sim/share/lynsense_webots_sim/trees/lynsense_single_box_tree.xml",
            "rpent_plan_runner_executable": "/workspace/ws/install/lynsense_webots_sim/bin/lynsense_run_rpent_plan",
        "pytrees_share_exists": True,
        "pytrees_config_exists": True,
        "pytrees_tree_exists": True,
        "pytrees_match_tree_exists": True,
        "pytrees_node_exists": True,
            "pytrees_node_executable": True,
            "single_box_tree_executable_exists": True,
            "single_box_tree_executable_is_executable": True,
            "single_box_tree_exists": True,
            "rpent_plan_runner_executable_exists": True,
            "rpent_plan_runner_executable_is_executable": True,
    }

    def fake_bash(command):
        commands.append(command)
        return 0, json.dumps(good_evidence) + "\n"

    monkeypatch.setattr(bootstrap, "_bash", fake_bash)
    verification = bootstrap._verify_overlay()
    assert verification == good_evidence
    assert len(commands) == 1
    assert 'import_module("lynsense_utils")' in commands[0]
    assert 'import_module("lynsense_webots_sim")' in commands[0]
    assert "import lynsense_pytrees" not in commands[0]
    embedded_script = commands[0].split("python3 -c '", 1)[1].rsplit("'", 1)[0]
    compile(embedded_script, "overlay-verification", "exec")
    for required in (
        "/workspace/ws/install/lynsense_pytrees/share/lynsense_pytrees",
        "/workspace/ws/install/lynsense_pytrees/share/lynsense_pytrees/config/move_points.yaml",
        "/workspace/ws/install/lynsense_pytrees/share/lynsense_pytrees/trees/lynsense_webots_tree.xml",
        "/workspace/ws/install/lynsense_pytrees/share/lynsense_pytrees/trees/lynsense_match_min_tree.xml",
        "/workspace/ws/install/lynsense_pytrees/lib/lynsense_pytrees/lynsense_pytrees_node.py",
    ):
        assert required in commands[0]

    for field in (
        "pytrees_share_exists",
        "pytrees_config_exists",
        "pytrees_tree_exists",
        "pytrees_match_tree_exists",
        "pytrees_node_exists",
        "pytrees_node_executable",
        "rpent_plan_runner_executable_exists",
        "rpent_plan_runner_executable_is_executable",
    ):
        with pytest.raises(RuntimeError, match="overlay verification"):
            bootstrap._verify_overlay_with({**good_evidence, field: False})


def test_tree_startup_invokes_installed_pytrees_executable(tmp_path, monkeypatch):
    from lynsense_webots_sim.scripts import run_smoke

    executable = tmp_path / "lynsense_pytrees_node.py"
    executable.write_text("#!/usr/bin/env python3\n", encoding="utf-8")
    executable.chmod(0o755)
    monkeypatch.setattr(run_smoke, "PYTREES_NODE_PATH", executable)
    monkeypatch.setattr(run_smoke, "ARTIFACT_ROOT", tmp_path)
    invocations = []

    class _PopenFake:
        pass

    def fake_popen(argv, **_kwargs):
        invocations.append((argv, _kwargs))
        return _PopenFake()

    monkeypatch.setattr(run_smoke.subprocess, "Popen", fake_popen)
    run_smoke._start_tree({}, "match", "tree-run")

    assert len(invocations) == 1
    assert invocations[0][0] == [
        str(executable),
        "--ros-args",
        "-p",
        "xml:=lynsense_match_min_tree.xml",
    ]


def test_controller_console_script_path_and_executable_checks(tmp_path, monkeypatch):
    bootstrap = _load_bootstrap()
    from lynsense_webots_sim.scripts import run_smoke

    expected = (
        "/workspace/ws/install/lynsense_webots_sim/bin/"
        "lynsense_webots_controller"
    )
    assert str(bootstrap.CONTROLLER_PATH) == expected
    assert str(run_smoke.CONTROLLER_PATH) == expected

    bootstrap_controller = tmp_path / "bootstrap-controller"
    bootstrap_controller.write_text("#!/bin/sh\n", encoding="utf-8")
    bootstrap_controller.chmod(0o755)
    monkeypatch.setattr(bootstrap, "CONTROLLER_PATH", bootstrap_controller)
    bootstrap._verify_controller()
    bootstrap_controller.chmod(0o644)
    with pytest.raises(RuntimeError, match="not executable"):
        bootstrap._verify_controller()

    orchestrator_controller = tmp_path / "orchestrator-controller"
    orchestrator_controller.write_text("#!/bin/sh\n", encoding="utf-8")
    orchestrator_controller.chmod(0o755)
    monkeypatch.setattr(run_smoke, "CONTROLLER_PATH", orchestrator_controller)
    run_smoke._verify_controller()
    orchestrator_controller.chmod(0o644)
    with pytest.raises(run_smoke.SmokeError, match="not executable"):
        run_smoke._verify_controller()

    launch = LAUNCH_PATH.read_text(encoding="utf-8")
    assert expected in launch
    assert "ExecuteProcess" in launch
    assert 'executable="lynsense_webots_controller"' not in launch


def test_webots_startup_enables_glx_render_and_software_rendering(tmp_path, monkeypatch):
    from lynsense_webots_sim.scripts import run_smoke

    world = tmp_path / "world.wbt"
    world.touch()
    monkeypatch.setattr(run_smoke, "ARTIFACT_ROOT", tmp_path)
    invocations = []

    def fake_popen(argv, **kwargs):
        invocations.append((argv, kwargs))
        return _OrchestratorProcessFake()

    monkeypatch.setattr(run_smoke.subprocess, "Popen", fake_popen)
    readiness_calls = []
    monkeypatch.setattr(
        run_smoke,
        "_wait_for_webots_ready",
        lambda process, log_path: readiness_calls.append((process, log_path)),
    )
    run_smoke._start_webots(
        "interface",
        {"WEBOTS_WORLD": str(world)},
        "render-run",
    )

    assert len(invocations) == 1
    argv, kwargs = invocations[0]
    assert argv == [
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
    ]
    assert kwargs["env"]["LIBGL_ALWAYS_SOFTWARE"] == "1"
    assert kwargs["env"]["QT_OPENGL"] == "software"
    assert len(readiness_calls) == 1
    assert readiness_calls[0][0] is invocations[0][1] or readiness_calls[0][0] is not None
    assert readiness_calls[0][1] == tmp_path / "webots-interface-render-run.log"


def test_webots_readiness_helper_accepts_marker_and_does_not_sleep(tmp_path, monkeypatch):
    from lynsense_webots_sim.scripts import run_smoke

    log_path = tmp_path / "webots.log"
    log_path.write_text(
        "Webots startup\nWaiting for local or remote connection\n",
        encoding="utf-8",
    )
    process = _OrchestratorProcessFake()
    sleeps = []
    monkeypatch.setattr(run_smoke.time, "sleep", sleeps.append)

    run_smoke._wait_for_webots_ready(process, log_path)
    assert sleeps == []


def test_webots_readiness_checks_exit_before_marker(tmp_path):
    from lynsense_webots_sim.scripts import run_smoke

    log_path = tmp_path / "webots.log"
    log_path.write_text("Waiting for local or remote connection\n", encoding="utf-8")
    process = _OrchestratorProcessFake(exit_code=1)

    with pytest.raises(
        run_smoke.SmokeError,
        match="Webots exited before the ready marker with code 1",
    ):
        run_smoke._wait_for_webots_ready(process, log_path)


def test_webots_readiness_times_out_without_real_sleep(tmp_path, monkeypatch):
    from lynsense_webots_sim.scripts import run_smoke

    log_path = tmp_path / "webots.log"
    log_path.write_text("Webots has not accepted a controller yet\n", encoding="utf-8")
    process = _OrchestratorProcessFake()
    sleeps = []
    monotonic_values = iter((0.0, 1.0, 15.0))
    monkeypatch.setattr(run_smoke.time, "monotonic", lambda: next(monotonic_values))
    monkeypatch.setattr(run_smoke.time, "sleep", sleeps.append)

    with pytest.raises(
        run_smoke.SmokeError,
        match="Webots ready marker exceeded 15 seconds",
    ):
        run_smoke._wait_for_webots_ready(process, log_path)
    assert sleeps == [0.1]


def test_action_graph_readiness_polls_until_both_endpoints_are_listed(monkeypatch):
    from lynsense_webots_sim.scripts import run_smoke

    processes = run_smoke.PhaseProcesses(
        webots=_OrchestratorProcessFake(),
        controller=_OrchestratorProcessFake(),
    )
    outputs = [
        "/lynsense/nav_to_pose\n",
        "/unrelated/action\n",
        "/lynsense/move_distance\n/lynsense/nav_to_pose\n",
    ]
    invocations = []

    class _CompletedFake:
        returncode = 0

        def __init__(self, output):
            self.stdout = output
            self.stderr = ""

    def fake_run(argv, **kwargs):
        invocations.append((argv, kwargs))
        return _CompletedFake(outputs[len(invocations) - 1])

    sleeps = []
    monkeypatch.setattr(run_smoke.subprocess, "run", fake_run)
    monkeypatch.setattr(run_smoke.time, "sleep", sleeps.append)

    run_smoke._wait_for_action_graph(processes, {"ROS_DOMAIN_ID": "42"})

    assert len(invocations) == 3
    assert all(invocation[0] == ["ros2", "action", "list"] for invocation in invocations)
    assert invocations[0][1]["env"] == {"ROS_DOMAIN_ID": "42"}
    assert invocations[0][1]["timeout"] == 2.0
    assert invocations[0][1]["check"] is False
    assert sleeps == [0.25, 0.25]


def test_action_graph_readiness_times_out_when_endpoint_is_missing(monkeypatch):
    from lynsense_webots_sim.scripts import run_smoke

    processes = run_smoke.PhaseProcesses(
        webots=_OrchestratorProcessFake(),
        controller=_OrchestratorProcessFake(),
    )

    class _CompletedFake:
        returncode = 0
        stdout = "/lynsense/nav_to_pose\n"
        stderr = ""

    monkeypatch.setattr(
        run_smoke.subprocess,
        "run",
        lambda *_args, **_kwargs: _CompletedFake(),
    )
    monotonic_values = iter((100.0, 200.0))
    monkeypatch.setattr(run_smoke.time, "monotonic", lambda: next(monotonic_values))

    with pytest.raises(
        run_smoke.SmokeError,
        match="ROS action endpoints did not become discoverable",
    ):
        run_smoke._wait_for_action_graph(processes, {})


def test_controller_pythonpath_puts_webots_api_first_and_preserves_existing(monkeypatch, tmp_path):
    from lynsense_webots_sim.scripts import run_smoke

    launcher = tmp_path / "webots-controller"
    controller = tmp_path / "lynsense_webots_controller"
    for executable in (launcher, controller):
        executable.write_text("#!/bin/sh\n", encoding="utf-8")
        executable.chmod(0o755)
    monkeypatch.setattr(run_smoke, "CONTROLLER_PATH", controller)
    monkeypatch.setattr(run_smoke, "ARTIFACT_ROOT", tmp_path)
    invocations = []

    def fake_popen(argv, **kwargs):
        invocations.append((argv, kwargs))
        return _OrchestratorProcessFake()

    monkeypatch.setattr(run_smoke.subprocess, "Popen", fake_popen)
    run_smoke._start_controller(
        {
            "WEBOTS_HOME": str(tmp_path),
            "PYTHONPATH": "/overlay/python:/opt/ros/humble/lib/python3.10/site-packages",
        },
        "interface",
        "python-api-run",
    )

    expected_pythonpath = os.pathsep.join(
        [
            "/usr/local/webots/lib/controller/python",
            "/overlay/python",
            "/opt/ros/humble/lib/python3.10/site-packages",
        ]
    )
    assert len(invocations) == 1
    assert invocations[0][0] == [
        str(launcher),
        "--robot-name=EA200_SMOKE",
        str(controller),
    ]
    assert invocations[0][1]["env"]["PYTHONPATH"] == expected_pythonpath
    assert invocations[0][1]["env"]["LIBGL_ALWAYS_SOFTWARE"] == "1"
    assert invocations[0][1]["env"]["QT_OPENGL"] == "software"


def test_controller_selects_script_only_for_smoke_and_blocked():
    tree = _controller_tree()
    main = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "main"
    )
    script_assignments = [
        ast.unparse(node)
        for node in ast.walk(main)
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "script" for target in node.targets)
    ]
    assert script_assignments == [
        "script = None if script_name in {'interface', 'box'} "
        "else config['scripts'][script_name]"
    ]


def test_viewer_readme_documents_launch_evidence_and_safety_boundary():
    readme = Path("robots/lynsense/simulation/README.md").read_text(encoding="utf-8")
    normalized = " ".join(readme.split())
    for phrase in (
        "Simulation Viewer",
        "LYNSENSE_VIEWER_PHASE=box",
        "LYNSENSE_VIEWER_PHASE=match",
        "docker compose -f compose.yaml -f compose.viewer.yaml up lynsense-webots-viewer",
        "http://<workstation-ip>:7047/",
        "2-5 seconds",
        ".artifacts/lynsense-webots/viewer/<run-id>/",
        "It is an observer only",
        "does not authorize real-robot motion",
        "docker compose -f compose.yaml -f compose.viewer.yaml down",
    ):
        assert phrase in normalized
