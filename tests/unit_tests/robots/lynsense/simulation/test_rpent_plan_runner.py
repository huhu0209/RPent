from __future__ import annotations

import json
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from robots.lynsense.simulation.rpent_plan import (
    PLAN_VERSION,
    TASK_ID,
    PlanError,
    canonical_single_box_plan,
)
from lynsense_webots_sim.scripts import run_rpent_plan


def _plan_document(**changes: Any) -> dict[str, Any]:
    document = {
        "task_id": TASK_ID,
        "version": PLAN_VERSION,
        "actions": [dict(action) for action in canonical_single_box_plan()],
    }
    document.update(changes)
    return document


def _write_plan(tmp_path: Path, document: Any) -> Path:
    path = tmp_path / "plan.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


def test_plan_path_is_required_and_loaded_before_ros_startup(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("LYNSENSE_RPENT_PLAN", raising=False)
    initialized = False

    def _fail_init(*args: Any, **kwargs: Any) -> None:
        nonlocal initialized
        initialized = True

    monkeypatch.setattr(run_rpent_plan, "_init_rclpy", _fail_init)
    with pytest.raises(RuntimeError, match="LYNSENSE_RPENT_PLAN"):
        run_rpent_plan.main()
    assert initialized is False


def test_invalid_plan_is_rejected_before_ros_startup(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    document = _plan_document()
    document["actions"][0]["goal_name"] = "wrong"
    plan_path = _write_plan(tmp_path, document)
    monkeypatch.setenv("LYNSENSE_RPENT_PLAN", str(plan_path))
    monkeypatch.setattr(
        run_rpent_plan,
        "_init_rclpy",
        lambda *args, **kwargs: pytest.fail("ROS startup must follow plan validation"),
    )

    with pytest.raises(Exception, match="actions do not match"):
        run_rpent_plan.main()


def test_plan_action_fields_use_existing_action_goal_schema() -> None:
    actions = canonical_single_box_plan()
    assert run_rpent_plan._goal_fields(actions[0]) == {"goal_name": "搬箱子1"}
    assert run_rpent_plan._goal_fields(actions[4]) == {
        "action": "pick",
        "flow": "flow",
        "config": "box1",
    }
    assert run_rpent_plan._goal_fields(actions[5]) == {
        "distance": -0.6,
        "angle": 0.0,
    }


def test_runner_uses_the_real_rclpy_node_api() -> None:
    source = Path(run_rpent_plan.__file__).read_text(encoding="utf-8")
    assert "from rclpy.node import Node" in source


class _Future:
    def __init__(self, value: Any, *, done: bool = True) -> None:
        self.value = value
        self._done = done

    def done(self) -> bool:
        return self._done

    def complete(self, value: Any) -> None:
        self.value = value
        self._done = True

    def result(self) -> Any:
        return self.value


class _GoalHandle:
    def __init__(self, accepted: bool = True, *, result_pending: bool = False) -> None:
        self.accepted = accepted
        self.cancel_requested = False
        self.result_pending = result_pending

    def get_result_async(self) -> _Future:
        return _Future(
            SimpleNamespace(result=SimpleNamespace(success=True)),
            done=not self.result_pending,
        )

    def cancel_goal_async(self) -> _Future:
        self.cancel_requested = True
        return _Future(None)


class _ActionType:
    class Goal:
        pass


class _ActionClient:
    action_type = _ActionType

    def __init__(
        self,
        accepted: bool = True,
        *,
        send_goal_pending: bool = False,
        result_pending: bool = False,
    ) -> None:
        self.handle = _GoalHandle(
            accepted=accepted,
            result_pending=result_pending,
        )
        self.waited = False
        self.send_goal_pending = send_goal_pending
        self.send_goal_future: _Future | None = None

    def wait_for_server(self, *, timeout_sec: float) -> bool:
        self.waited = True
        return True

    def send_goal_async(self, goal: Any, *, feedback_callback: Any) -> _Future:
        self.goal = goal
        future = _Future(self.handle, done=not self.send_goal_pending)
        self.send_goal_future = future
        return future

    def destroy(self) -> None:
        pass


class _Rclpy:
    def __init__(self) -> None:
        self.started = False
        self.stopped = False

    def init(self, *args: Any) -> None:
        self.started = True

    def ok(self) -> bool:
        return self.started and not self.stopped

    def spin_once(self, node: Any = None, *, timeout_sec: float) -> None:
        pass

    def shutdown(self) -> None:
        self.stopped = True


class _Node:
    def __init__(self, name: str) -> None:
        self.name = name
        self.destroyed = False

    def destroy_node(self) -> None:
        self.destroyed = True


def _run_validated_plan(accepted: bool = True):
    actions = canonical_single_box_plan()
    rclpy = _Rclpy()
    rclpy.started = True
    node = _Node("lynsense_rpent_plan_runner")
    clients: list[_ActionClient] = []

    def client_factory(_node: Any, _action_name: str) -> _ActionClient:
        client = _ActionClient(accepted=accepted)
        clients.append(client)
        return client

    exit_code = run_rpent_plan._run_plan(actions, rclpy, client_factory, node=node)
    return exit_code, clients, rclpy, node


def test_run_plan_executes_the_validated_actions_serially() -> None:
    exit_code, clients, rclpy, node = _run_validated_plan()
    assert exit_code == 0
    assert len(clients) == 15
    assert all(client.waited for client in clients)
    assert clients[0].goal.goal_name == "搬箱子1"
    assert clients[4].goal.action == "pick"
    assert clients[5].goal.distance == -0.6
    assert clients[5].goal.angle == 0.0
    assert rclpy.stopped is True
    assert node.destroyed is True


def test_run_plan_returns_nonzero_when_a_goal_is_rejected() -> None:
    exit_code, clients, rclpy, node = _run_validated_plan(accepted=False)
    assert exit_code == 1
    assert len(clients) == 1
    assert rclpy.stopped is True
    assert node.destroyed is True


@pytest.mark.parametrize("interrupt_type", [KeyboardInterrupt, RuntimeError])
def test_action_exception_waits_for_pending_goal_and_cancels_it(
    monkeypatch: pytest.MonkeyPatch,
    interrupt_type: type[BaseException],
) -> None:
    from lynsense_webots_sim.tree_behaviours import SimulationActionBehaviour

    actions = canonical_single_box_plan()
    rclpy = _Rclpy()
    rclpy.started = True
    node = _Node("lynsense_rpent_plan_runner")
    client = _ActionClient(send_goal_pending=True, result_pending=True)

    def client_factory(_node: Any, _action_name: str) -> _ActionClient:
        return client

    def spin_once(_node: Any = None, *, timeout_sec: float) -> None:
        if client.send_goal_future is not None and not client.send_goal_future.done():
            client.send_goal_future.complete(client.handle)

    rclpy.spin_once = spin_once  # type: ignore[method-assign]
    original_initialise = SimulationActionBehaviour.initialise

    def initialise_then_interrupt(self: SimulationActionBehaviour) -> Any:
        result = original_initialise(self)
        raise interrupt_type("interrupt after goal request")

    monkeypatch.setattr(
        SimulationActionBehaviour, "initialise", initialise_then_interrupt
    )

    if interrupt_type is KeyboardInterrupt:
        exit_code = run_rpent_plan._run_plan(
            actions, rclpy, client_factory, node=node
        )
        assert exit_code == 130
    else:
        with pytest.raises(RuntimeError, match="interrupt after goal request"):
            run_rpent_plan._run_plan(actions, rclpy, client_factory, node=node)

    assert client.send_goal_future is not None
    assert client.send_goal_future.done()
    assert client.handle.cancel_requested is True
    assert rclpy.stopped is True
    assert node.destroyed is True


def test_runner_shutdown_accepts_an_absent_behaviour(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(run_rpent_plan, "_shutdown_behaviour", lambda behaviour: None)
    run_rpent_plan._shutdown_behaviour(None)
