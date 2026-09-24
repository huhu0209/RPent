from __future__ import annotations

import enum
import math
from collections.abc import Callable, Mapping
from typing import Any


try:  # Runtime container; host unit tests intentionally do not install ROS/py_trees.
    import py_trees

    Status = py_trees.common.Status
    BehaviourBase = py_trees.behaviour.Behaviour
except ModuleNotFoundError:  # pragma: no cover - exercised by the host suite.
    class Status(enum.Enum):
        INVALID = "INVALID"
        RUNNING = "RUNNING"
        SUCCESS = "SUCCESS"
        FAILURE = "FAILURE"

    class BehaviourBase:
        def __init__(self, name: str = "") -> None:
            self.name = name
            self.status = Status.INVALID
            self.feedback_message = ""
            self.node: Any = None

        def setup(self, node: Any = None) -> bool:
            if node is not None:
                self.node = node
            return True

        def initialise(self) -> Any:
            self.status = Status.INVALID
            return self.status

        def update(self) -> Any:
            return self.status


SINGLE_BOX_ACTIONS = (
    "nav_to_pose",
    "move_distance",
    "move_waist",
    "move_named_config",
    "set_gripper",
    "box_phase",
)
ACTION_ENDPOINTS = {
    "nav_to_pose": "/lynsense/nav_to_pose",
    "move_distance": "/lynsense/move_distance",
    "move_waist": "/lynsense/move_waist",
    "move_named_config": "/lynsense/move_named_config",
    "set_gripper": "/lynsense/set_gripper",
    "box_phase": "/lynsense/box_phase",
}
ClientFactory = Callable[[Any, str], Any]


class SimulationActionBehaviour(BehaviourBase):
    def __init__(
        self,
        name: str,
        action_name: str,
        goal_fields: Mapping[str, Any],
        *,
        client_factory: ClientFactory | None = None,
        wait_timeout_s: float = 1.0,
    ) -> None:
        super().__init__(name)
        if action_name not in SINGLE_BOX_ACTIONS:
            raise ValueError(f"unsupported action: {action_name}")
        self.action_name = action_name
        self.goal_fields = dict(goal_fields)
        self.client_factory = client_factory or _create_action_client
        self.wait_timeout_s = wait_timeout_s
        self.action_client: Any = None
        self._goal_type: Any = None
        self.last_feedback: Any = None
        self._send_goal_future: Any = None
        self._goal_handle: Any = None
        self._result_future: Any = None
        self._cancel_future: Any = None
        self._cancel_requested = False

    def setup(self, node: Any = None) -> bool:
        if node is not None:
            self.node = node
        super().setup()
        if self.action_client is None:
            self.action_client = self.client_factory(self.node, self.action_name)
            self._goal_type = getattr(self.action_client, "action_type", None)
            if self._goal_type is None:
                self._goal_type = getattr(self.action_client, "_action_type", None)
            if self._goal_type is None:
                raise RuntimeError(
                    f"{self.action_name} action client does not expose a goal type"
                )
        return True

    def initialise(self) -> Status:
        self.status = Status.RUNNING
        self.last_feedback = None
        self._send_goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._cancel_future = None
        self._cancel_requested = False
        if self.action_client is None or not self.action_client.wait_for_server(
            timeout_sec=self.wait_timeout_s
        ):
            self._log(
                "error",
                f"{self.action_name} action server unavailable: "
                f"{ACTION_ENDPOINTS[self.action_name]}",
            )
            self.status = Status.FAILURE
            return self.status

        goal = self._goal_type.Goal()
        for field, value in self.goal_fields.items():
            setattr(goal, field, value)
        self._send_goal_future = self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        return self.status

    def feedback_callback(self, feedback_message: Any) -> None:
        self.last_feedback = feedback_message
        feedback = getattr(feedback_message, "feedback", feedback_message)
        phase = getattr(feedback, "phase", "unknown")
        self._log("info", f"{self.action_name} feedback: {phase}")

    def update(self) -> Status:
        if self._cancel_requested:
            if self._cancel_future is None or not self._cancel_future.done():
                return self._set_status(Status.RUNNING)
            self._log("info", f"{self.action_name} cancellation completed")
            return self._set_status(Status.FAILURE)

        if self._send_goal_future is not None:
            if not self._send_goal_future.done():
                return self._set_status(Status.RUNNING)
            try:
                self._goal_handle = self._send_goal_future.result()
            except BaseException as exc:
                self._send_goal_future = None
                self._log("error", f"{self.action_name} send goal failed: {exc}")
                return self._set_status(Status.FAILURE)
            self._send_goal_future = None
            if not self._goal_handle.accepted:
                self._log("error", f"{self.action_name} goal rejected")
                self._goal_handle = None
                return self._set_status(Status.FAILURE)
            self._log("info", f"{self.action_name} goal accepted")
            self._result_future = self._goal_handle.get_result_async()
            return self._set_status(Status.RUNNING)

        if self._result_future is not None:
            if not self._result_future.done():
                return self._set_status(Status.RUNNING)
            try:
                result_wrapper = self._result_future.result()
                result = getattr(result_wrapper, "result", result_wrapper)
            except BaseException as exc:
                self._log("error", f"{self.action_name} result failed: {exc}")
                return self._set_status(Status.FAILURE)
            if not result.success:
                self._log("error", f"{self.action_name} action failed")
                return self._set_status(Status.FAILURE)
            self._log("info", f"{self.action_name} action succeeded")
            return self._set_status(Status.SUCCESS)

        return self._set_status(Status.FAILURE)

    def halt(self) -> None:
        if (
            not self._cancel_requested
            and self._goal_handle is not None
            and self._result_future is not None
            and not self._result_future.done()
        ):
            self._cancel_requested = True
            self._cancel_future = self._goal_handle.cancel_goal_async()
            self._log("info", f"{self.action_name} cancellation requested")
            return
        self._send_goal_future = None
        self._goal_handle = None
        self._result_future = None

    def shutdown(self) -> None:
        self.halt()
        if self.action_client is not None:
            destroy = getattr(self.action_client, "destroy", None)
            if destroy is not None:
                destroy()

    def _set_status(self, status: Status) -> Status:
        self.status = status
        return status

    def _log(self, level: str, message: str) -> None:
        logger = getattr(self, "logger", None)
        if logger is None and self.node is not None:
            logger = getattr(self.node, "logger", None)
        if logger is not None:
            getattr(logger, level)(message)


class EnsureSimulationManipulationServices(BehaviourBase):
    def __init__(
        self,
        name: str = "EnsureSimulationServices",
        *,
        client_factory: ClientFactory | None = None,
        wait_timeout_s: float = 1.0,
    ) -> None:
        super().__init__(name)
        self.client_factory = client_factory or _create_action_client
        self.wait_timeout_s = wait_timeout_s
        self.action_clients: dict[str, Any] = {}

    def setup(self, node: Any = None) -> bool:
        if node is not None:
            self.node = node
        super().setup()
        if not self.action_clients:
            self.action_clients = {
                action: self.client_factory(self.node, action)
                for action in SINGLE_BOX_ACTIONS
            }
        return True

    def initialise(self) -> Status:
        return self._set_status(Status.RUNNING)

    def update(self) -> Status:
        available = {
            action: bool(
                client.wait_for_server(timeout_sec=self.wait_timeout_s)
            )
            for action, client in self.action_clients.items()
        }
        if not all(available.values()):
            missing = [action for action, ready in available.items() if not ready]
            self._log("error", f"simulation action servers unavailable: {missing}")
            return self._set_status(Status.RUNNING)
        self._log("info", "all simulation action servers available")
        return self._set_status(Status.SUCCESS)

    def shutdown(self) -> None:
        for client in self.action_clients.values():
            destroy = getattr(client, "destroy", None)
            if destroy is not None:
                destroy()

    def _set_status(self, status: Status) -> Status:
        self.status = status
        return status

    def _log(self, level: str, message: str) -> None:
        logger = getattr(self, "logger", None)
        if logger is None and self.node is not None:
            logger = getattr(self.node, "logger", None)
        if logger is not None:
            getattr(logger, level)(message)


class SimulationNavToPose(SimulationActionBehaviour):
    def __init__(self, goal_name: str, name: str = "SimulationNavToPose", **kwargs: Any) -> None:
        super().__init__(name, "nav_to_pose", {"goal_name": goal_name}, **kwargs)


class SimulationMoveDistance(SimulationActionBehaviour):
    def __init__(
        self, distance: float, angle: float = 0.0, name: str = "SimulationMoveDistance", **kwargs: Any
    ) -> None:
        values = _finite_values(distance=distance, angle=angle)
        super().__init__(name, "move_distance", values, **kwargs)


class SimulationPlanMoveWaist(SimulationActionBehaviour):
    def __init__(self, height_mm: float, name: str = "SimulationPlanMoveWaist", **kwargs: Any) -> None:
        super().__init__(
            name, "move_waist", _finite_values(height_mm=height_mm), **kwargs
        )


class SimulationPlanMoveNamedConfig(SimulationActionBehaviour):
    def __init__(self, target: str, name: str = "SimulationPlanMoveNamedConfig", **kwargs: Any) -> None:
        if not target:
            raise ValueError("target must not be empty")
        super().__init__(name, "move_named_config", {"target": target}, **kwargs)


class SimulationPlanSetGripper(SimulationActionBehaviour):
    def __init__(self, position: float, name: str = "SimulationPlanSetGripper", **kwargs: Any) -> None:
        super().__init__(
            name, "set_gripper", _finite_values(position=position), **kwargs
        )


class SimulationPlanBoxPhase(SimulationActionBehaviour):
    def __init__(
        self,
        action: str,
        flow: str,
        config: str,
        name: str = "SimulationPlanBoxPhase",
        **kwargs: Any,
    ) -> None:
        if not action or not flow or not config:
            raise ValueError("box phase action, flow, and config are required")
        super().__init__(
            name,
            "box_phase",
            {"action": action, "flow": flow, "config": config},
            **kwargs,
        )


def _create_action_client(node: Any, action_name: str) -> Any:
    from rclpy.action import ActionClient

    action_type = _action_type(action_name)
    return ActionClient(node, action_type, ACTION_ENDPOINTS[action_name])


def _action_type(action_name: str) -> Any:
    from lynsense_utils.action import (
        BoxPhase,
        MoveDistance,
        MoveNamedConfig,
        MoveWaist,
        NavToPose,
        SetGripper,
    )

    return {
        "nav_to_pose": NavToPose,
        "move_distance": MoveDistance,
        "move_waist": MoveWaist,
        "move_named_config": MoveNamedConfig,
        "set_gripper": SetGripper,
        "box_phase": BoxPhase,
    }[action_name]


def _finite_values(**values: float) -> dict[str, float]:
    for name, value in values.items():
        if not math.isfinite(float(value)):
            raise ValueError(f"{name} must be finite")
    return {name: float(value) for name, value in values.items()}
