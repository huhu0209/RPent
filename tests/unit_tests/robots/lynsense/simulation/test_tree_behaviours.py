from __future__ import annotations

import importlib.util
import enum
import sys
import types
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path("robots") / "lynsense" / "simulation" / "ros" / "lynsense_webots_sim"
TREE_PATH = ROOT / "trees" / "lynsense_single_box_tree.xml"


class PendingFuture:
    def __init__(self, value=None):
        self.value = value
        self.done_value = False

    def done(self):
        return self.done_value

    def result(self):
        if not self.done_value:
            raise RuntimeError("future is not done")
        if isinstance(self.value, BaseException):
            raise self.value
        return self.value


class FakeGoal:
    pass


class FakeActionType:
    Goal = FakeGoal


class FakeGoalHandle:
    def __init__(self, accepted=True, result=None):
        self.accepted = accepted
        self.cancel_future = PendingFuture(None)
        self.result_future = PendingFuture(
            SimpleNamespace(result=result or SimpleNamespace(success=True))
        )

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        return self.cancel_future


class FakeActionClient:
    def __init__(self, *, available=True):
        self.action_type = FakeActionType
        self.available = available
        self.wait_calls = 0
        self.goals = []
        self.feedback_callbacks = []
        self.cancel_calls = 0
        self.cancel_future = PendingFuture(None)
        self.goal_future = PendingFuture(FakeGoalHandle())
        self.goal_future.value.cancel_future = self.cancel_future
        self.goal_future.value.cancel_goal_async = self.cancel_goal_async

    def wait_for_server(self, timeout_sec=None):
        self.wait_calls += 1
        self.last_timeout_sec = timeout_sec
        return self.available

    def send_goal_async(self, goal, feedback_callback=None):
        self.goals.append(goal)
        self.feedback_callbacks.append(feedback_callback)
        return self.goal_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future

    def destroy(self):
        self.destroyed = True


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(("info", message))

    def error(self, message):
        self.messages.append(("error", message))


class FakeNode:
    def __init__(self):
        self.logger = FakeLogger()


def complete(future):
    future.done_value = True
    return future


def test_single_box_xml_is_serial_and_exact() -> None:
    from lynsense_webots_sim.tree_behaviours import Status

    root = ET.parse(TREE_PATH).getroot()
    assert root.tag == "py_trees.composites.Sequence"
    assert root.attrib["name"] == "SingleBoxSerialTask"
    assert root.attrib["memory"] == "True"
    assert not root.findall(".//Parallel")
    assert [node.attrib["name"] for node in root] == [
        "EnsureSimulationServices",
        "NavigateToBox1",
        "PrepareWaist",
        "CloseGrippersForPrepare",
        "PrepareDualArms",
        "PickBox1",
        "BackOffFromBox1",
        "TurnAfterBox1",
        "PreparePlacement",
        "NavigateToBox1Drop",
        "PlaceBox1",
        "RetreatAfterPlacement",
        "RaiseArmsAfterPlacement",
        "RestoreWaist",
        "RestoreGrippers",
        "RestoreTravelPosture",
    ]
    text = TREE_PATH.read_text(encoding="utf-8")
    for forbidden in ("plan_arm_simple", "subprocess", "ros2 run"):
        assert forbidden not in text
    assert Status.RUNNING.value == "RUNNING"


@pytest.mark.parametrize(
    ("behaviour_factory", "action_name", "expected"),
    [
        (
            lambda client_factory: __import__(
                "lynsense_webots_sim.tree_behaviours", fromlist=["SimulationNavToPose"]
            ).SimulationNavToPose(goal_name="搬箱子1", client_factory=client_factory),
            "nav_to_pose",
            {"goal_name": "搬箱子1"},
        ),
        (
            lambda client_factory: __import__(
                "lynsense_webots_sim.tree_behaviours", fromlist=["SimulationMoveDistance"]
            ).SimulationMoveDistance(distance=-0.6, angle=90.0, client_factory=client_factory),
            "move_distance",
            {"distance": -0.6, "angle": 90.0},
        ),
        (
            lambda client_factory: __import__(
                "lynsense_webots_sim.tree_behaviours", fromlist=["SimulationPlanMoveWaist"]
            ).SimulationPlanMoveWaist(height_mm=200.0, client_factory=client_factory),
            "move_waist",
            {"height_mm": 200.0},
        ),
        (
            lambda client_factory: __import__(
                "lynsense_webots_sim.tree_behaviours", fromlist=["SimulationPlanMoveNamedConfig"]
            ).SimulationPlanMoveNamedConfig(target="dualjo:joints_br", client_factory=client_factory),
            "move_named_config",
            {"target": "dualjo:joints_br"},
        ),
        (
            lambda client_factory: __import__(
                "lynsense_webots_sim.tree_behaviours", fromlist=["SimulationPlanSetGripper"]
            ).SimulationPlanSetGripper(position=0.0, client_factory=client_factory),
            "set_gripper",
            {"position": 0.0},
        ),
        (
            lambda client_factory: __import__(
                "lynsense_webots_sim.tree_behaviours", fromlist=["SimulationPlanBoxPhase"]
            ).SimulationPlanBoxPhase(
                action="pick", flow="flow", config="box1", client_factory=client_factory
            ),
            "box_phase",
            {"action": "pick", "flow": "flow", "config": "box1"},
        ),
    ],
)
def test_action_behaviour_success_result_and_goal_fields(
    behaviour_factory, action_name, expected
) -> None:
    from lynsense_webots_sim.tree_behaviours import Status

    client = FakeActionClient()
    node = FakeNode()
    behaviour = behaviour_factory(lambda _node, _action: client)
    assert behaviour.action_name == action_name
    assert behaviour.setup(node) is True
    assert behaviour.initialise() is Status.RUNNING
    assert behaviour.update() is Status.RUNNING
    assert len(client.goals) == 1
    assert {key: getattr(client.goals[0], key) for key in expected} == expected

    complete(client.goal_future)
    assert behaviour.update() is Status.RUNNING
    assert ("info", f"{action_name} goal accepted") in node.logger.messages

    feedback = SimpleNamespace(feedback=SimpleNamespace(phase="running"))
    client.feedback_callbacks[0](feedback)
    assert behaviour.last_feedback is feedback
    assert ("info", f"{action_name} feedback: running") in node.logger.messages

    complete(client.goal_future.value.result_future)
    assert behaviour.update() is Status.SUCCESS


def test_action_behaviour_rejects_and_fails_results() -> None:
    from lynsense_webots_sim.tree_behaviours import SimulationNavToPose, Status

    node = FakeNode()
    client = FakeActionClient()
    behaviour = SimulationNavToPose(goal_name="home", client_factory=lambda _n, _a: client)
    behaviour.setup(node)
    behaviour.initialise()
    complete(client.goal_future)
    client.goal_future.value.accepted = False
    assert behaviour.update() is Status.FAILURE

    client = FakeActionClient()
    node = FakeNode()
    behaviour = SimulationNavToPose(goal_name="home", client_factory=lambda _n, _a: client)
    behaviour.setup(node)
    behaviour.initialise()
    complete(client.goal_future)
    behaviour.update()
    client.goal_future.value.result_future.value.result.success = False
    complete(client.goal_future.value.result_future)
    assert behaviour.update() is Status.FAILURE


def test_action_behaviour_halt_cancels_and_waits_for_cancel() -> None:
    from lynsense_webots_sim.tree_behaviours import SimulationPlanBoxPhase, Status

    client = FakeActionClient()
    behaviour = SimulationPlanBoxPhase(
        action="place", flow="flow", config="box1", client_factory=lambda _n, _a: client
    )
    behaviour.setup(FakeNode())
    behaviour.initialise()
    complete(client.goal_future)
    behaviour.update()

    behaviour.halt()
    assert client.cancel_calls == 1
    assert behaviour.update() is Status.RUNNING
    complete(client.cancel_future)
    assert behaviour.update() is Status.FAILURE
    assert client.cancel_calls == 1


def test_service_wait_behaviour_waits_for_all_six_actions() -> None:
    from lynsense_webots_sim.tree_behaviours import (
        EnsureSimulationManipulationServices,
        Status,
    )

    clients = {
        name: FakeActionClient(available=False)
        for name in (
            "nav_to_pose", "move_distance", "move_waist",
            "move_named_config", "set_gripper", "box_phase",
        )
    }
    node = FakeNode()
    behaviour = EnsureSimulationManipulationServices(
        client_factory=lambda _node, action: clients[action]
    )
    behaviour.setup(node)
    assert behaviour.initialise() is Status.RUNNING
    assert behaviour.update() is Status.RUNNING
    for client in clients.values():
        client.available = True
    assert behaviour.update() is Status.SUCCESS
    assert all(client.wait_calls >= 2 for client in clients.values())


def test_runner_and_setup_declare_owned_entry_point_with_lazy_runtime_imports() -> None:
    setup = (ROOT / "setup.py").read_text(encoding="utf-8")
    runner_path = (
        ROOT
        / "lynsense_webots_sim"
        / "scripts"
        / "run_single_box_tree.py"
    )
    assert "lynsense_single_box_tree = lynsense_webots_sim.scripts.run_single_box_tree:main" in setup
    assert runner_path.is_file()
    runner = runner_path.read_text(encoding="utf-8")
    for forbidden in ("subprocess", "ros2 run", "plan_arm_simple"):
        assert forbidden not in runner
    import ast

    parsed = ast.parse(runner)
    allowed = {
        "__future__", "os", "pathlib", "typing",
        "xml.etree", "xml.etree.ElementTree",
    }
    top_level_runtime_imports = []
    for node in parsed.body:
        if isinstance(node, ast.Import):
            if any(alias.name not in allowed for alias in node.names):
                top_level_runtime_imports.append(node)
        elif isinstance(node, ast.ImportFrom) and node.module not in allowed:
            top_level_runtime_imports.append(node)
    assert top_level_runtime_imports == []
    spec = importlib.util.spec_from_file_location("single_box_runner", runner_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert callable(module.main)


def test_runner_builds_sequence_with_current_py_trees_api(monkeypatch) -> None:
    class FakeSequence:
        def __init__(self, name, memory):
            self.name = name
            self.memory = memory
            self.children = []

        def add_child(self, child):
            self.children.append(child)
            return child

    py_trees = types.ModuleType("py_trees")
    py_trees.composites = SimpleNamespace(Sequence=FakeSequence)
    monkeypatch.setitem(sys.modules, "py_trees", py_trees)

    spec = importlib.util.spec_from_file_location(
        "single_box_runner_current_api", ROOT / "lynsense_webots_sim" / "scripts" / "run_single_box_tree.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    root = module.create_root(TREE_PATH)
    assert root.name == "SingleBoxSerialTask"
    assert root.memory is True
    assert len(root.children) == 16


def test_single_box_runner_stops_after_success() -> None:
    source = (
        ROOT
        / "lynsense_webots_sim"
        / "scripts"
        / "run_single_box_tree.py"
    ).read_text(encoding="utf-8")
    assert "add_post_tick_handler(stop_after_success)" in source
    assert (
        "while rclpy.ok() and root.status != py_trees.common.Status.SUCCESS:"
        in source
    )


def test_behaviour_setup_accepts_py_trees_ros_node_keyword(monkeypatch) -> None:
    class RuntimeStatus(enum.Enum):
        INVALID = "INVALID"
        RUNNING = "RUNNING"
        SUCCESS = "SUCCESS"
        FAILURE = "FAILURE"

    class RuntimeBehaviour:
        def __init__(self, name=""):
            self.name = name

        def setup(self, **kwargs):
            self.setup_kwargs = kwargs
            return True

    py_trees = types.ModuleType("py_trees")
    common = types.ModuleType("py_trees.common")
    common.Status = RuntimeStatus
    behaviour = types.ModuleType("py_trees.behaviour")
    behaviour.Behaviour = RuntimeBehaviour
    py_trees.common = common
    py_trees.behaviour = behaviour
    monkeypatch.setitem(sys.modules, "py_trees", py_trees)
    monkeypatch.setitem(sys.modules, "py_trees.common", common)
    monkeypatch.setitem(sys.modules, "py_trees.behaviour", behaviour)

    path = ROOT / "lynsense_webots_sim" / "tree_behaviours.py"
    spec = importlib.util.spec_from_file_location("tree_behaviours_runtime_api", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = FakeNode()
    client = FakeActionClient()
    behaviour = module.SimulationNavToPose(
        goal_name="搬箱子1", client_factory=lambda _node, _action: client
    )
    assert behaviour.setup(node=node) is True
    assert behaviour.node is node
    assert behaviour.setup_kwargs == {}


def test_action_behaviour_resolves_rclpy_private_action_type() -> None:
    from lynsense_webots_sim.tree_behaviours import SimulationNavToPose, Status

    client = FakeActionClient()
    private_type = client.action_type
    del client.action_type
    client._action_type = private_type
    behaviour = SimulationNavToPose(
        goal_name="搬箱子1", client_factory=lambda _node, _action: client
    )
    assert behaviour.setup(FakeNode()) is True
    assert behaviour._goal_type is private_type
    assert behaviour.initialise() is Status.RUNNING
    assert client.goals[0].goal_name == "搬箱子1"
