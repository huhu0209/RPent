"""Guarded, evidence-sealed tools for the Phase 1 real single-box task."""

from __future__ import annotations

import copy
from collections.abc import Mapping
from typing import Any

from robots.lynsense_real_box.dry_run_adapter import OfflineDryRunAdapter
from robots.lynsense_real_box.evidence import EvidenceRecorder
from robots.lynsense_real_box.site_profile import SiteProfile, site_profile_hash
from robots.lynsense_real_box.task_state import (
    TaskState,
    TaskStateMachine,
    ToolName,
    TransitionError,
)
from rpent.dashboard.events import DashboardEventSink
from rpent.memory import MemoryManager
from rpent.tools.toolkit import ToolResult, Toolkit, readonly


_PREPARATION_PROFILES = {
    TaskState.AT_PICK_APPROACH: {
        ToolName.MOVE_WAIST: "pick_ready",
        ToolName.SET_DUAL_GRIPPERS: "close",
        ToolName.MOVE_DUAL_ARMS: "pick_ready",
    },
    TaskState.AT_PLACE_APPROACH: {
        ToolName.MOVE_DUAL_ARMS: "place_ready",
    },
    TaskState.WITHDRAWN: {
        ToolName.MOVE_WAIST: "safe",
        ToolName.SET_DUAL_GRIPPERS: "open",
        ToolName.MOVE_DUAL_ARMS: "safe",
    },
}


class LynsenseRealBoxToolkit(Toolkit):
    """The exactly-12-tool model boundary for the single-box workflow."""

    include_image_reader = False

    class _NoStepStore:
        def latest_record(self) -> None:
            return None

    def __init__(
        self,
        *,
        adapter: OfflineDryRunAdapter,
        state_machine: TaskStateMachine,
        evidence: EvidenceRecorder,
        dashboard_events: DashboardEventSink,
        memory: MemoryManager,
        profile: SiteProfile,
    ) -> None:
        super().__init__(
            dashboard_events=dashboard_events,
            memory=memory,
            state=self._NoStepStore(),
        )
        self._adapter = adapter
        self._machine = state_machine
        self._evidence = evidence
        self._profile = profile
        self._profile_sha256 = site_profile_hash(profile)
        self._navigation_goals = tuple(profile.navigation_goals)
        self._move_transitions = tuple(profile.move_transitions)
        self._waist_profiles = tuple(profile.waist_profiles)
        self._dual_arm_configs = tuple(profile.dual_arm_configs)
        self._gripper_commands = tuple(profile.gripper_commands)
        self._closed = False

        self.add_tool("read_task_state", self._schema("read_task_state"), self.read_task_state)
        self.add_tool(
            "read_robot_state", self._schema("read_robot_state"), self.read_robot_state
        )
        self.add_tool("detect_box", self._schema("detect_box"), self.detect_box)
        self.add_tool("nav_to_pose", self._schema("nav_to_pose"), self.nav_to_pose)
        self.add_tool(
            "move_distance", self._schema("move_distance"), self.move_distance
        )
        self.add_tool("move_waist", self._schema("move_waist"), self.move_waist)
        self.add_tool(
            "move_dual_arms", self._schema("move_dual_arms"), self.move_dual_arms
        )
        self.add_tool(
            "set_dual_grippers",
            self._schema("set_dual_grippers"),
            self.set_dual_grippers,
        )
        self.add_tool("pick_box", self._schema("pick_box"), self.pick_box)
        self.add_tool("place_box", self._schema("place_box"), self.place_box)
        self.add_tool("stop_task", self._schema("stop_task"), self.stop_task)
        self.add_tool("finish", self._schema("finish"), self.finish)

    def _register_common_tools(self) -> None:
        """The real-box boundary intentionally exposes no common tools."""

    def get_tools_spec(self) -> list[dict[str, Any]]:
        """Return stable local schemas; this table has no output-dir templates."""

        return [copy.deepcopy(spec) for spec, _ in self._tools.values()]

    def get_env_state(
        self,
        *,
        command: dict[str, Any],
        result: dict[str, Any],
        elapsed_s: float,
    ) -> dict[str, Any]:
        """Keep base-class post-dispatch capture inside the guarded boundary."""

        return result

    def _schema(self, name: str) -> dict[str, Any]:
        properties: dict[str, Any] = {}
        description = {
            "read_task_state": "Read the guarded task state and profile hash.",
            "read_robot_state": "Read the adapter's guarded robot state snapshot.",
            "detect_box": "Trigger guarded box detection.",
            "nav_to_pose": "Navigate to a profile-approved goal.",
            "move_distance": "Execute a state-matched profiled chassis move.",
            "move_waist": "Execute a profiled waist motion.",
            "move_dual_arms": "Execute a profiled dual-arm configuration.",
            "set_dual_grippers": "Command both grippers together.",
            "pick_box": "Execute the reviewed pick sequence.",
            "place_box": "Execute the reviewed place sequence.",
            "stop_task": "Stop motion best-effort and confirm the stop.",
            "finish": "Finish only from an explicitly safe terminal state.",
        }[name]
        if name == "nav_to_pose":
            properties["goal"] = _enum_property(self._navigation_goals)
        elif name == "move_distance":
            properties["profile_name"] = _enum_property(
                [item.name for item in self._move_transitions]
            )
        elif name == "move_waist":
            properties["profile_name"] = _enum_property(
                [item.name for item in self._waist_profiles]
            )
        elif name == "move_dual_arms":
            properties["profile_name"] = _enum_property(
                [item.name for item in self._dual_arm_configs]
            )
        elif name == "set_dual_grippers":
            properties["command"] = _enum_property(self._gripper_commands)
        elif name == "finish":
            properties["status"] = {
                "type": "string",
                "enum": ["success", "failure"],
            }
        return {
            "name": name,
            "description": description,
            "input_schema": {
                "type": "object",
                "properties": properties,
                "required": list(properties),
                "additionalProperties": False,
            },
        }

    @readonly
    def read_task_state(self) -> dict[str, Any]:
        return self._record(
            ToolName.READ_TASK_STATE,
            {},
            {
                "status": "ok",
                "reason": None,
                "task_state": self._machine.state.value,
                "profile_id": self._profile.profile_id,
                "profile_sha256": self._profile_sha256,
            },
        )

    @readonly
    def read_robot_state(self) -> dict[str, Any]:
        return self._read_tool(ToolName.READ_ROBOT_STATE, lambda: self._adapter.read_state())

    @readonly
    def detect_box(self) -> dict[str, Any]:
        tool = ToolName.DETECT_BOX
        if tool not in self._machine.allowed_tools():
            return self._guard_rejection(tool, {}, "illegal_tool")
        state_before = self._machine.state.value
        try:
            adapter_result = self._adapter.detect_box()
        except Exception:
            return self._record(
                tool,
                {},
                {
                    "status": "failed",
                    "reason": "adapter_exception",
                    "task_state": self._machine.state.value,
                },
                state_before,
            )
        if adapter_result.get("status") in {"ok", "dry_run"}:
            self._machine.apply(tool, success=True)
            return self._successful(tool, adapter_result, state_before, {})
        if adapter_result.get("status") == "failed":
            self._machine.apply(tool, success=False)
        return self._guard_rejection(
            tool,
            {},
            str(adapter_result.get("reason", "adapter_rejected")),
            state_before=state_before,
        )

    def nav_to_pose(self, *, goal: str) -> dict[str, Any]:
        arguments = {"goal": goal}
        tool = ToolName.NAV_TO_POSE
        rejection = self._profile_argument(tool, arguments, goal, self._navigation_goals, "goal")
        if rejection is not None:
            return rejection
        state = self._machine.state.value
        if state == "box_localized" and goal != "pickup":
            return self._guard_rejection(tool, arguments, "wrong_goal_for_state")
        if state == "carrying" and goal != "placement":
            return self._guard_rejection(tool, arguments, "wrong_goal_for_state")
        return self._motion_tool(tool, arguments, lambda: self._adapter.navigate(goal))

    def move_distance(self, *, profile_name: str) -> dict[str, Any]:
        arguments = {"profile_name": profile_name}
        tool = ToolName.MOVE_DISTANCE
        names = [item.name for item in self._move_transitions]
        rejection = self._profile_argument(
            tool, arguments, profile_name, names, "profile_name"
        )
        if rejection is not None:
            return rejection
        transition = next(
            item for item in self._move_transitions if item.name == profile_name
        )
        if transition.from_state != self._machine.state.value:
            return self._guard_rejection(tool, arguments, "wrong_from_state")
        return self._motion_tool(
            tool, arguments, lambda: self._adapter.move_distance(profile_name)
        )

    def move_waist(self, *, profile_name: str) -> dict[str, Any]:
        arguments = {"profile_name": profile_name}
        tool = ToolName.MOVE_WAIST
        names = [item.name for item in self._waist_profiles]
        rejection = self._profile_argument(
            tool, arguments, profile_name, names, "profile_name"
        )
        if rejection is not None:
            return rejection
        rejection = self._preparation_profile_rejection(tool, profile_name, arguments)
        if rejection is not None:
            return rejection
        return self._motion_tool(
            tool, arguments, lambda: self._adapter.move_waist(profile_name)
        )

    def move_dual_arms(self, *, profile_name: str) -> dict[str, Any]:
        arguments = {"profile_name": profile_name}
        tool = ToolName.MOVE_DUAL_ARMS
        names = [item.name for item in self._dual_arm_configs]
        rejection = self._profile_argument(
            tool, arguments, profile_name, names, "profile_name"
        )
        if rejection is not None:
            return rejection
        rejection = self._preparation_profile_rejection(tool, profile_name, arguments)
        if rejection is not None:
            return rejection
        return self._motion_tool(
            tool, arguments, lambda: self._adapter.move_dual_arms(profile_name)
        )

    def set_dual_grippers(self, *, command: str) -> dict[str, Any]:
        arguments = {"command": command}
        tool = ToolName.SET_DUAL_GRIPPERS
        rejection = self._profile_argument(
            tool, arguments, command, self._gripper_commands, "command"
        )
        if rejection is not None:
            return rejection
        rejection = self._preparation_profile_rejection(tool, command, arguments)
        if rejection is not None:
            return rejection
        return self._motion_tool(
            tool, arguments, lambda: self._adapter.set_dual_grippers(command)
        )

    def pick_box(self) -> dict[str, Any]:
        arguments: dict[str, Any] = {}
        if (
            self._machine.state.value != "dual_pick_prepared"
            or self._machine.pick_preparations
            != frozenset(
                {"waist", "grippers", "arms"}
            )
        ):
            return self._guard_rejection(ToolName.PICK_BOX, arguments, "missing_preparations")
        return self._motion_tool(ToolName.PICK_BOX, arguments, self._adapter.pick_box)

    def place_box(self) -> dict[str, Any]:
        return self._motion_tool(ToolName.PLACE_BOX, {}, self._adapter.place_box)

    def stop_task(self) -> dict[str, Any]:
        tool = ToolName.STOP_TASK
        if tool not in self._machine.allowed_tools():
            return self._guard_rejection(tool, {}, "illegal_tool")
        state_before = self._machine.state.value
        held_box = self._machine.state in (
            TaskState.BOX_GRASPED,
            TaskState.CARRYING,
        )
        try:
            adapter_result = self._adapter.stop_motion()
        except Exception:
            self._machine.apply(tool, success=False)
            return self._record(
                tool,
                {},
                {
                    "status": "failed",
                    "reason": "adapter_exception",
                    "task_state": self._machine.state.value,
                },
                state_before,
            )
        self._machine.apply(
            tool,
            success=adapter_result.get("status") in {"best_effort", "ok", "dry_run"},
            failure_kind=adapter_result.get("failure_kind"),
            box_hazard=held_box,
        )
        success = adapter_result.get("status") in {"best_effort", "ok", "dry_run"}
        result = {
            "status": "ok" if success else adapter_result.get("status", "failed"),
            "reason": None if success else adapter_result.get("reason", "stop_failed"),
            "task_state": self._machine.state.value,
            "estop": False,
            "stop_timeout_s": adapter_result.get("stop_timeout_s"),
        }
        return self._record(tool, {}, result, state_before)

    def finish(self, *, status: str) -> dict[str, Any]:
        tool = ToolName.FINISH
        arguments = {"status": status}
        if status not in {"success", "failure"}:
            return self._guard_rejection(tool, arguments, "invalid_status")
        try:
            self._machine.assert_may_finish(status)
        except TransitionError:
            return self._guard_rejection(tool, arguments, "unsafe_finish")
        return self._record(
            tool,
            arguments,
            {
                "status": "ok",
                "reason": None,
                "task_state": self._machine.state.value,
                "_finish": True,
            },
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._adapter.close()
        self._evidence.close()

    def execute_tool(
        self,
        name: str,
        input_dict: dict[str, Any],
    ) -> ToolResult:
        entry = self._tools.get(name)
        if entry is None:
            result = self._dispatch_rejection(
                name, {}, "unknown_tool", f"unknown tool: {name}"
            )
            return self._tool_result(name, result)
        if not isinstance(input_dict, Mapping):
            result = self._dispatch_rejection(
                name, {}, "schema_invalid", "input must be an object"
            )
            return self._tool_result(name, result)

        schema = entry[0]["input_schema"]
        expected = set(schema.get("properties", {}))
        actual = set(input_dict)
        if actual - expected or not set(schema.get("required", [])) <= actual:
            result = self._dispatch_rejection(
                name,
                dict(input_dict),
                "schema_invalid",
                "input does not match the tool schema",
            )
            return self._tool_result(name, result)

        result = super().execute_tool(name, input_dict).result
        if not isinstance(result, dict) or "evidence_id" not in result:
            reason = "handler_error" if "error" in result else "missing_evidence"
            result = self._dispatch_rejection(
                name,
                dict(input_dict),
                reason,
                result.get("error") if isinstance(result, dict) else None,
            )
        return self._tool_result(name, result)

    def _tool_result(self, name: str, result: dict[str, Any]) -> ToolResult:
        return ToolResult(name=name, result=result)

    def _read_tool(self, tool: ToolName, request) -> dict[str, Any]:
        if tool not in self._machine.allowed_tools():
            return self._guard_rejection(tool, {}, "illegal_tool")
        state_before = self._machine.state.value
        try:
            adapter_result = request()
        except Exception:
            return self._record(
                tool,
                {},
                {
                    "status": "failed",
                    "reason": "adapter_exception",
                    "task_state": self._machine.state.value,
                },
                state_before,
            )
        if adapter_result.get("status") in {"ok", "dry_run"}:
            return self._successful(
                tool,
                adapter_result,
                state_before,
                {},
            )
        return self._guard_rejection(
            tool,
            {},
            str(adapter_result.get("reason", "adapter_rejected")),
            state_before=state_before,
        )

    def _motion_tool(
        self,
        tool: ToolName,
        arguments: dict[str, Any],
        request,
    ) -> dict[str, Any]:
        if tool not in self._machine.allowed_tools():
            return self._guard_rejection(tool, arguments, "illegal_tool")
        state_before = self._machine.state.value
        try:
            adapter_result = request()
        except Exception:
            self._machine.apply(
                tool,
                success=False,
                failure_kind="adapter_exception",
                box_hazard=False,
            )
            return self._record(
                tool,
                arguments,
                {
                    "status": "failed",
                    "reason": "adapter_exception",
                    "task_state": self._machine.state.value,
                },
                state_before,
            )

        success = adapter_result.get("status") in {"ok", "dry_run"}
        if success:
            try:
                self._machine.apply(
                    tool,
                    profile_name=arguments.get(
                        "profile_name",
                        arguments.get("command", arguments.get("goal")),
                    ),
                    success=True,
                )
            except TransitionError:
                return self._guard_rejection(
                    tool,
                    arguments,
                    "illegal_transition",
                    state_before=state_before,
                )
            return self._successful(
                tool, adapter_result, state_before, arguments
            )

        if adapter_result.get("status") == "failed":
            self._machine.apply(
                tool,
                success=False,
                failure_kind=adapter_result.get("failure_kind"),
                box_hazard=bool(adapter_result.get("box_hazard", False)),
            )
            result = {
                "status": "failed",
                "reason": adapter_result.get("failure_kind", "adapter_failed"),
                "task_state": self._machine.state.value,
                "failure_kind": adapter_result.get("failure_kind"),
                "box_hazard": bool(adapter_result.get("box_hazard", False)),
            }
        else:
            result = {
                "status": "rejected",
                "reason": adapter_result.get("reason", "adapter_rejected"),
                "task_state": self._machine.state.value,
            }
        return self._record(tool, arguments, result, state_before)

    def _successful(
        self,
        tool: ToolName,
        adapter_result: dict[str, Any],
        state_before: str,
        arguments: dict[str, Any],
    ) -> dict[str, Any]:
        result: dict[str, Any] = {
            "status": "ok",
            "reason": None,
            "task_state": self._machine.state.value,
        }
        if "robot_state" in adapter_result:
            result["robot_state"] = adapter_result["robot_state"]
        if tool is ToolName.DETECT_BOX:
            for key in ("frame", "received_at", "pose"):
                result[key] = adapter_result.get(key)
        return self._record(tool, arguments, result, state_before)

    def _profile_argument(
        self,
        tool: ToolName,
        arguments: dict[str, Any],
        value: str,
        values: list[str] | tuple[str, ...],
        field: str,
    ) -> dict[str, Any] | None:
        if value not in values:
            return self._guard_rejection(tool, arguments, f"unknown_{field}")
        return None

    def _preparation_profile_rejection(
        self,
        tool: ToolName,
        value: str,
        arguments: dict[str, Any],
    ) -> dict[str, Any] | None:
        required_profile = _PREPARATION_PROFILES.get(self._machine.state, {}).get(tool)
        if required_profile is not None and value != required_profile:
            return self._guard_rejection(
                tool,
                arguments,
                "wrong_profile_for_state",
            )
        return None

    def _guard_rejection(
        self,
        tool: ToolName,
        arguments: dict[str, Any],
        reason: str,
        *,
        state_before: str | None = None,
    ) -> dict[str, Any]:
        before = state_before or self._machine.state.value
        result = {
            "status": "rejected",
            "reason": reason,
            "task_state": self._machine.state.value,
        }
        return self._record(tool, arguments, result, before)

    def _dispatch_rejection(
        self,
        name: str,
        arguments: dict[str, Any],
        reason: str,
        error: str | None,
    ) -> dict[str, Any]:
        result: dict[str, Any] = {
            "status": "rejected",
            "reason": reason,
            "task_state": self._machine.state.value,
        }
        if error is not None:
            result["error"] = error
        return self._record_name(name, arguments, result)

    def _record(
        self,
        tool: ToolName,
        arguments: dict[str, Any],
        result: dict[str, Any],
        state_before: str | None = None,
    ) -> dict[str, Any]:
        return self._record_name(
            tool.value,
            arguments,
            result,
            state_before or self._machine.state.value,
        )

    def _record_name(
        self,
        tool: str,
        arguments: dict[str, Any],
        result: dict[str, Any],
        state_before: str | None = None,
    ) -> dict[str, Any]:
        sealed = dict(result)
        evidence_id = self._evidence.record_tool_call(
            tool,
            arguments,
            sealed,
            state_before or self._machine.state.value,
            self._machine.state.value,
        )
        sealed["evidence_id"] = evidence_id
        return sealed


def _enum_property(values: list[str] | tuple[str, ...]) -> dict[str, Any]:
    return {
        "type": "string",
        "enum": list(values),
    }
