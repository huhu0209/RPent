from __future__ import annotations

import os
from pathlib import Path
import sys
import time
from typing import Any


PLAN_ENVIRONMENT_VARIABLE = "LYNSENSE_RPENT_PLAN"
PLAN_SOURCE_ROOT = Path("/workspace/simulation")
HOST_PLAN_SOURCE_ROOT = Path(__file__).resolve().parents[4]
NODE_NAME = "lynsense_rpent_plan_runner"
CANCELLATION_SHUTDOWN_BUDGET_S = 5.0


def main(args: Any = None) -> int:
    plan_path = os.environ.get(PLAN_ENVIRONMENT_VARIABLE)
    if not plan_path:
        raise RuntimeError(
            f"{PLAN_ENVIRONMENT_VARIABLE} must identify the validated plan JSON file"
        )
    actions = _load_plan(Path(plan_path))
    rclpy = _init_rclpy(args)
    try:
        return _run_plan(actions, rclpy)
    except KeyboardInterrupt:
        return 130
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def _load_plan(path: Path) -> tuple[dict[str, Any], ...]:
    load_plan = _plan_contract("load_plan")
    return load_plan(path)


def _plan_contract(name: str) -> Any:
    try:
        import rpent_plan
    except ModuleNotFoundError:
        for source_root in (HOST_PLAN_SOURCE_ROOT, PLAN_SOURCE_ROOT):
            if not (source_root / "rpent_plan.py").is_file():
                continue
            if str(source_root) not in sys.path:
                sys.path.insert(0, str(source_root))
            import rpent_plan
            break
        else:
            raise
    return getattr(rpent_plan, name)


def _init_rclpy(args: Any = None) -> Any:
    import rclpy

    rclpy.init(args=args)
    return rclpy


def _run_plan(
    actions: tuple[dict[str, Any], ...],
    rclpy: Any,
    client_factory: Any = None,
    *,
    node: Any = None,
) -> int:
    from lynsense_webots_sim.tree_behaviours import (
        SimulationActionBehaviour,
        Status,
    )

    if client_factory is None:
        from lynsense_webots_sim.tree_behaviours import _create_action_client

        client_factory = lambda behaviour_node, action_name: _create_action_client(  # noqa: E731
            behaviour_node, action_name
        )

    if node is None:
        from rclpy.node import Node

        node = Node(NODE_NAME)
    try:
        for index, action in enumerate(actions):
            behaviour = SimulationActionBehaviour(
                name=f"RpentPlanAction{index:02d}",
                action_name=action["action"],
                goal_fields=_goal_fields(action),
                client_factory=client_factory,
            )
            try:
                if not behaviour.setup(node):
                    raise RuntimeError(f"plan action {index} setup failed")
                status = behaviour.initialise()
                while status is Status.RUNNING:
                    rclpy.spin_once(node, timeout_sec=0.1)
                    status = behaviour.update()
                if status is not Status.SUCCESS:
                    return 1
            except KeyboardInterrupt:
                _cancel_behaviour(behaviour, rclpy, node)
                return 130
            except BaseException:
                _cancel_behaviour(behaviour, rclpy, node)
                raise
            finally:
                behaviour.shutdown()
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _goal_fields(action: dict[str, Any]) -> dict[str, Any]:
    name = action["action"]
    if name == "nav_to_pose":
        return {"goal_name": action["goal_name"]}
    if name == "move_distance":
        return {"distance": action["distance"], "angle": action["angle"]}
    if name == "move_waist":
        return {"height_mm": action["height_mm"]}
    if name == "move_named_config":
        return {"target": action["target"]}
    if name == "set_gripper":
        return {"position": action["position"]}
    if name == "box_phase":
        return {
            "action": action["box_action"],
            "flow": action["flow"],
            "config": action["config"],
        }
    raise ValueError(f"unsupported plan action: {name!r}")


def _cancel_behaviour(behaviour: Any, rclpy: Any, node: Any) -> None:
    deadline = time.monotonic() + CANCELLATION_SHUTDOWN_BUDGET_S
    while rclpy.ok() and time.monotonic() < deadline:
        if not getattr(behaviour, "_cancel_requested", False):
            send_goal_future = getattr(behaviour, "_send_goal_future", None)
            if send_goal_future is not None and not send_goal_future.done():
                rclpy.spin_once(node, timeout_sec=0.1)
                continue
            if send_goal_future is not None:
                behaviour.update()
            behaviour.halt()
            if not getattr(behaviour, "_cancel_requested", False):
                return

        cancel_future = getattr(behaviour, "_cancel_future", None)
        if cancel_future is not None and cancel_future.done():
            behaviour.update()
            return
        rclpy.spin_once(node, timeout_sec=0.1)


def _shutdown_behaviour(behaviour: Any) -> None:
    if behaviour is not None:
        behaviour.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
