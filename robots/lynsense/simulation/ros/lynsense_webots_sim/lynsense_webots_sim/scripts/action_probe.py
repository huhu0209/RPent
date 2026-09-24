from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import time
from typing import Any


ACTION_ENDPOINTS = ("/lynsense/nav_to_pose", "/lynsense/move_distance")
PROBE_BUDGET_S = 45.0


def _invalid_move_goal(distance: float, angle: float) -> bool:
    return (
        math.isnan(distance)
        or math.isinf(distance)
        or not -10.0 <= distance <= 10.0
        or math.isnan(angle)
        or math.isinf(angle)
        or not -360.0 <= angle <= 360.0
    )


class ProbeError(RuntimeError):
    pass


def _deadline() -> float:
    return time.monotonic() + PROBE_BUDGET_S


def _assert_deadline(deadline: float) -> None:
    if time.monotonic() >= deadline:
        raise ProbeError("action probe exceeded its 45 second budget")


def _spin(executor: Any, future: Any, deadline: float) -> Any:
    while not future.done():
        _assert_deadline(deadline)
        executor.spin_once(timeout_sec=0.05)
    return future.result()


def _event_path() -> Path:
    return Path(os.environ["LYNSENSE_SIM_EVENT_FILE"])


def _run_id() -> str:
    return os.environ["LYNSENSE_SIM_RUN_ID"]


def _events() -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for line in _event_path().read_text(encoding="utf-8").splitlines():
        event = json.loads(line)
        if event.get("run_id") != _run_id():
            raise ProbeError("event has a missing or different run_id")
        events.append(event)
    return events


def _feedback_events() -> list[dict[str, Any]]:
    return [event for event in _events() if event.get("kind") == "action_feedback"]


def _wait_for_clients(clients: tuple[Any, ...], executor: Any, deadline: float) -> None:
    while not all(client.server_is_ready() for client in clients):
        _assert_deadline(deadline)
        time.sleep(0.05)
        executor.spin_once(timeout_sec=0)


def _submit(executor: Any, client: Any, goal: Any, deadline: float) -> tuple[bool, Any]:
    goal_future = client.send_goal_async(goal)
    goal_handle = _spin(executor, goal_future, deadline)
    return bool(goal_handle.accepted), goal_handle


def _run_interface_checks() -> None:
    import rclpy
    from rclpy.action import ActionClient

    from lynsense_utils.action import MoveDistance, NavToPose

    rclpy.init()
    node = rclpy.create_node("lynsense_action_probe")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    deadline = _deadline()
    try:
        nav_client = ActionClient(node, NavToPose, ACTION_ENDPOINTS[0])
        move_client = ActionClient(node, MoveDistance, ACTION_ENDPOINTS[1])
        _wait_for_clients((nav_client, move_client), executor, deadline)

        unknown = NavToPose.Goal()
        unknown.goal_name = "unknown_goal"
        accepted, _handle = _submit(executor, nav_client, unknown, deadline)
        if accepted:
            raise ProbeError("unknown NavToPose goal was accepted")

        invalid_values = (
            (float("nan"), 0.0),
            (float("inf"), 0.0),
            (float("-inf"), 0.0),
            (0.0, 360.1),
        )
        invalid_goals = []
        for distance, angle in invalid_values:
            if not _invalid_move_goal(distance, angle):
                raise AssertionError("probe did not recognize an invalid MoveDistance goal")
            goal = MoveDistance.Goal()
            goal.distance = distance
            goal.angle = angle
            invalid_goals.append(goal)
        for goal in invalid_goals:
            accepted, _handle = _submit(executor, move_client, goal, deadline)
            if accepted:
                raise ProbeError("invalid MoveDistance goal was accepted")

        concurrent = MoveDistance.Goal()
        concurrent.distance = 0.0
        concurrent.angle = 360.0
        first_goal_future = move_client.send_goal_async(concurrent)
        second_goal_future = move_client.send_goal_async(concurrent)
        first_handle = _spin(executor, first_goal_future, deadline)
        second_handle = _spin(executor, second_goal_future, deadline)
        first_accepted = bool(first_handle.accepted)
        if not first_accepted:
            raise ProbeError("boundary MoveDistance goal was rejected before concurrency check")
        second_accepted = bool(second_handle.accepted)
        if second_accepted:
            raise ProbeError("second concurrent MoveDistance goal was accepted")
        result_future = first_handle.get_result_async()
        result = _spin(executor, result_future, deadline)
        if not result.result.success:
            raise ProbeError("boundary MoveDistance goal did not succeed")
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def _run_cancel_check() -> None:
    import rclpy
    from rclpy.action import ActionClient

    from lynsense_utils.action import MoveDistance

    rclpy.init()
    node = rclpy.create_node("lynsense_action_probe_cancel")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    deadline = time.monotonic() + PROBE_BUDGET_S
    try:
        client = ActionClient(node, MoveDistance, ACTION_ENDPOINTS[1])
        while not client.server_is_ready():
            _assert_deadline(deadline)
            time.sleep(0.05)

        goal = MoveDistance.Goal()
        goal.distance = 0.0
        goal.angle = 180.0
        goal_handle = _spin(executor, client.send_goal_async(goal), deadline)
        if not goal_handle.accepted:
            raise ProbeError("cancel probe goal was rejected")

        while len(_feedback_events()) < 2:
            _assert_deadline(deadline)
            time.sleep(0.05)
            executor.spin_once(timeout_sec=0)
        feedback = _feedback_events()[-1]
        if feedback.get("angle_remaining_deg", 0.0) < 90.0:
            raise ProbeError("cancel probe did not retain at least half the rotation")

        result_future = goal_handle.get_result_async()
        cancel_future = goal_handle.cancel_goal_async()
        cancel_response = _spin(executor, cancel_future, deadline)
        result = _spin(executor, result_future, deadline)
        from action_msgs.msg import GoalStatus

        if cancel_response.return_code != 0:
            raise ProbeError("cancel request was not accepted")
        if result.status != GoalStatus.STATUS_CANCELED:
            raise ProbeError("cancel probe did not end in CANCELED status")
        if result.result.success:
            raise ProbeError("cancel probe returned success=true")

        final_feedback = _feedback_events()[-1]
        rates = final_feedback["motor_rates"]
        if any(rates[name] != 0.0 for name in (
            "left_wheel_rate_radps",
            "right_wheel_rate_radps",
        )):
            raise ProbeError("cancel probe recorded nonzero motor rates")
        feedback_count = len(_feedback_events())
        time.sleep(0.2)
        if len(_feedback_events()) != feedback_count:
            raise ProbeError("cancel probe observed progress after cancellation")
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate simulation action boundaries.")
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--mode", choices=("interface", "cancel"))
    args = parser.parse_args()
    if args.mode == "interface":
        _run_interface_checks()
    else:
        _run_cancel_check()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
