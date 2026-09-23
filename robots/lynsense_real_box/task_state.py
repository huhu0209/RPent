from __future__ import annotations

from collections.abc import Mapping
from enum import Enum
from types import MappingProxyType

from robots.lynsense_real_box.site_profile import MoveTransition


class TaskState(str, Enum):
    INITIALIZED = "initialized"
    BOX_LOCALIZED = "box_localized"
    AT_PICK_APPROACH = "at_pick_approach"
    DUAL_PICK_PREPARED = "dual_pick_prepared"
    BOX_GRASPED = "box_grasped"
    CARRYING = "carrying"
    AT_PLACE_APPROACH = "at_place_approach"
    DUAL_PLACE_PREPARED = "dual_place_prepared"
    BOX_RELEASED = "box_released"
    WITHDRAWN = "withdrawn"
    SAFE_COMPLETE = "safe_complete"
    FAILED_STOPPING = "failed_stopping"
    FAILED_STOPPED = "failed_stopped"
    OPERATOR_REVIEW = "operator_review"
    OPERATOR_ACKNOWLEDGED_STOPPED = "operator_acknowledged_stopped"


class ToolName(str, Enum):
    READ_TASK_STATE = "read_task_state"
    READ_ROBOT_STATE = "read_robot_state"
    DETECT_BOX = "detect_box"
    NAV_TO_POSE = "nav_to_pose"
    MOVE_DISTANCE = "move_distance"
    MOVE_WAIST = "move_waist"
    MOVE_DUAL_ARMS = "move_dual_arms"
    SET_DUAL_GRIPPERS = "set_dual_grippers"
    PICK_BOX = "pick_box"
    PLACE_BOX = "place_box"
    STOP_TASK = "stop_task"
    FINISH = "finish"


class TransitionError(ValueError):
    """Raised when a tool call cannot legally advance the task."""


_SUCCESS_TRANSITIONS: dict[tuple[TaskState, ToolName], TaskState] = {
    (TaskState.INITIALIZED, ToolName.DETECT_BOX): TaskState.BOX_LOCALIZED,
    (TaskState.BOX_LOCALIZED, ToolName.NAV_TO_POSE): TaskState.AT_PICK_APPROACH,
    (TaskState.CARRYING, ToolName.NAV_TO_POSE): TaskState.AT_PLACE_APPROACH,
    (TaskState.AT_PLACE_APPROACH, ToolName.MOVE_DUAL_ARMS): TaskState.DUAL_PLACE_PREPARED,
    (TaskState.DUAL_PICK_PREPARED, ToolName.PICK_BOX): TaskState.BOX_GRASPED,
    (TaskState.DUAL_PLACE_PREPARED, ToolName.PLACE_BOX): TaskState.BOX_RELEASED,
}

_REQUIRED_PICK_PREPARATIONS = frozenset({"waist", "grippers", "arms"})
_MOTION_TOOLS = frozenset(
    {
        ToolName.NAV_TO_POSE,
        ToolName.MOVE_DISTANCE,
        ToolName.MOVE_WAIST,
        ToolName.MOVE_DUAL_ARMS,
        ToolName.SET_DUAL_GRIPPERS,
        ToolName.PICK_BOX,
        ToolName.PLACE_BOX,
    }
)
_ORDINARY_FAILURE_KINDS = frozenset(
    {
        "action_failed",
        "timeout",
        "canceled",
    }
)
_TERMINAL_STATES = frozenset(
    {TaskState.SAFE_COMPLETE, TaskState.OPERATOR_ACKNOWLEDGED_STOPPED}
)
_PICK_PREPARATION_TOOLS = {
    ToolName.MOVE_WAIST: "waist",
    ToolName.SET_DUAL_GRIPPERS: "grippers",
    ToolName.MOVE_DUAL_ARMS: "arms",
}
_PICK_PREPARATION_PROFILES = {
    ToolName.MOVE_WAIST: "pick_ready",
    ToolName.SET_DUAL_GRIPPERS: "close",
    ToolName.MOVE_DUAL_ARMS: "pick_ready",
}
_SAFE_PREPARATION_PROFILES = {
    ToolName.MOVE_WAIST: "safe",
    ToolName.SET_DUAL_GRIPPERS: "open",
    ToolName.MOVE_DUAL_ARMS: "safe",
}
_HELD_BOX_STATES = frozenset(
    {
        TaskState.BOX_GRASPED,
        TaskState.CARRYING,
    }
)


class TaskStateMachine:
    def __init__(
        self,
        initial_state: TaskState = TaskState.INITIALIZED,
        move_transitions: Mapping[str, MoveTransition] = MappingProxyType({}),
    ) -> None:
        if not isinstance(initial_state, TaskState):
            raise TransitionError("initial_state must be a TaskState")
        self._state = initial_state
        self._move_transitions = MappingProxyType(dict(move_transitions))
        self._pick_preparations: set[str] = set()
        self._safe_preparations: set[str] = set()
        self._failure_kind: str | None = None
        self._requires_operator_review = False

    @property
    def state(self) -> TaskState:
        return self._state

    @property
    def pick_preparations(self) -> frozenset[str]:
        return frozenset(self._pick_preparations)

    @property
    def safe_preparations(self) -> frozenset[str]:
        return frozenset(self._safe_preparations)

    @property
    def failure_kind(self) -> str | None:
        return self._failure_kind

    @property
    def requires_operator_review(self) -> bool:
        return self._requires_operator_review

    def allowed_tools(self) -> frozenset[ToolName]:
        tools = {
            ToolName.READ_TASK_STATE,
            ToolName.READ_ROBOT_STATE,
            ToolName.DETECT_BOX,
            ToolName.STOP_TASK,
        }
        state = self._state
        if state in (TaskState.BOX_LOCALIZED, TaskState.CARRYING):
            tools.add(ToolName.NAV_TO_POSE)
        if state == TaskState.AT_PICK_APPROACH:
            tools.update(_PICK_PREPARATION_TOOLS)
            tools.add(ToolName.PICK_BOX)
        if state == TaskState.DUAL_PICK_PREPARED:
            tools.add(ToolName.PICK_BOX)
        if state in (TaskState.BOX_GRASPED, TaskState.BOX_RELEASED, TaskState.CARRYING):
            tools.add(ToolName.MOVE_DISTANCE)
        if state == TaskState.AT_PLACE_APPROACH:
            tools.update((ToolName.MOVE_DUAL_ARMS, ToolName.PLACE_BOX))
        if state == TaskState.DUAL_PLACE_PREPARED:
            tools.add(ToolName.PLACE_BOX)
        if state == TaskState.WITHDRAWN:
            tools.update(_PICK_PREPARATION_TOOLS)
        if state == TaskState.SAFE_COMPLETE:
            tools = {ToolName.READ_TASK_STATE, ToolName.FINISH}
        if state == TaskState.OPERATOR_ACKNOWLEDGED_STOPPED:
            tools = {
                ToolName.READ_TASK_STATE,
                ToolName.READ_ROBOT_STATE,
                ToolName.FINISH,
            }
        return frozenset(tools)

    def apply(
        self,
        tool: ToolName,
        *,
        profile_name: str | None = None,
        success: bool = True,
        failure_kind: str | None = None,
        box_hazard: bool = False,
    ) -> TaskState:
        if not isinstance(tool, ToolName):
            raise TransitionError("tool must be a ToolName")
        if tool is ToolName.FINISH:
            raise TransitionError("finish must use assert_may_finish")
        if self._state in _TERMINAL_STATES and tool not in (
            ToolName.READ_TASK_STATE,
            ToolName.READ_ROBOT_STATE,
        ):
            raise TransitionError("tool is illegal in a terminal state")

        if tool is ToolName.STOP_TASK:
            return self._apply_stop(success, box_hazard)
        if tool in (ToolName.READ_TASK_STATE, ToolName.READ_ROBOT_STATE):
            return self._state
        if tool is ToolName.DETECT_BOX:
            if success and self._state is TaskState.INITIALIZED:
                self._apply_success(tool, profile_name)
            return self._state

        if tool is ToolName.PICK_BOX and self._pick_preparations != (
            _REQUIRED_PICK_PREPARATIONS
        ):
            raise TransitionError("pick_box requires all three pick preparations")

        if success:
            self._validate_successful_motion(tool, profile_name)
            self._apply_success(tool, profile_name)
            return self._state

        if tool not in _MOTION_TOOLS:
            raise TransitionError("only motion tools can fail into stopping")
        if tool is ToolName.MOVE_DISTANCE:
            self._move_transition(profile_name)
        self._failure_kind = failure_kind
        self._requires_operator_review = (
            box_hazard
            or (failure_kind is not None and failure_kind not in _ORDINARY_FAILURE_KINDS)
        )
        self._state = TaskState.FAILED_STOPPING
        return self._state

    def mark_external_estop(self) -> TaskState:
        self._validate_nonterminal("mark_external_estop")
        self._requires_operator_review = True
        self._state = TaskState.OPERATOR_REVIEW
        return self._state

    def mark_operator_review(self, reason: str) -> TaskState:
        self._validate_nonterminal("mark_operator_review")
        if not isinstance(reason, str) or not reason:
            raise TransitionError("operator review requires a reason")
        self._requires_operator_review = True
        self._state = TaskState.OPERATOR_REVIEW
        return self._state

    def mark_operator_stopped(self) -> TaskState:
        if self._state not in (
            TaskState.FAILED_STOPPED,
            TaskState.OPERATOR_REVIEW,
        ):
            raise TransitionError(
                "operator acknowledgement requires failed_stopped or operator_review"
            )
        self._requires_operator_review = False
        self._state = TaskState.OPERATOR_ACKNOWLEDGED_STOPPED
        return self._state

    def assert_may_finish(self, status: str) -> None:
        legal = (
            status == "success" and self._state is TaskState.SAFE_COMPLETE
        ) or (
            status == "failure"
            and self._state is TaskState.OPERATOR_ACKNOWLEDGED_STOPPED
        )
        if not legal:
            raise TransitionError("finish is not legal for this state and status")

    def _validate_successful_motion(
        self,
        tool: ToolName,
        profile_name: str | None,
    ) -> None:
        if tool is ToolName.PICK_BOX:
            if self._state is not TaskState.DUAL_PICK_PREPARED:
                raise TransitionError("pick_box is illegal in the current state")
            return

        if (self._state, tool) in _SUCCESS_TRANSITIONS:
            if tool is ToolName.NAV_TO_POSE:
                required_goal = (
                    "pickup"
                    if self._state is TaskState.BOX_LOCALIZED
                    else "placement"
                )
                if profile_name != required_goal:
                    raise TransitionError(f"nav_to_pose requires {required_goal}")
            if tool is ToolName.MOVE_DUAL_ARMS and profile_name != "place_ready":
                raise TransitionError("move_dual_arms requires place_ready")
            return

        preparation_kind = _PICK_PREPARATION_TOOLS.get(tool)
        if preparation_kind is not None and self._state in (
            TaskState.AT_PICK_APPROACH,
            TaskState.WITHDRAWN,
        ):
            if profile_name is None:
                raise TransitionError("preparation motion requires a profile name")
            required_profile = (
                _PICK_PREPARATION_PROFILES[tool]
                if self._state is TaskState.AT_PICK_APPROACH
                else _SAFE_PREPARATION_PROFILES[tool]
            )
            if profile_name != required_profile:
                raise TransitionError(
                    "preparation motion requires the context-specific profile"
                )
            if tool is ToolName.SET_DUAL_GRIPPERS and profile_name not in {
                "open",
                "close",
            }:
                raise TransitionError("set_dual_grippers requires open or close")
            return

        if tool is ToolName.MOVE_DISTANCE and self._state in (
            TaskState.BOX_GRASPED,
            TaskState.CARRYING,
            TaskState.BOX_RELEASED,
        ):
            self._move_transition(profile_name)
            return

        raise TransitionError("tool is illegal in the current state")

    def _apply_success(self, tool: ToolName, profile_name: str | None) -> None:
        preparation_kind = _PICK_PREPARATION_TOOLS.get(tool)
        if preparation_kind is not None and self._state is TaskState.AT_PICK_APPROACH:
            self._pick_preparations.add(preparation_kind)
            if self._pick_preparations == _REQUIRED_PICK_PREPARATIONS:
                self._state = TaskState.DUAL_PICK_PREPARED
            return
        if preparation_kind is not None and self._state is TaskState.WITHDRAWN:
            self._safe_preparations.add(preparation_kind)
            if self._safe_preparations == _REQUIRED_PICK_PREPARATIONS:
                self._state = TaskState.SAFE_COMPLETE
            return
        if tool is ToolName.MOVE_DISTANCE:
            transition = self._move_transition(profile_name)
            self._state = TaskState(transition.to_state)
            return

        self._state = _SUCCESS_TRANSITIONS[(self._state, tool)]

    def _move_transition(
        self,
        profile_name: str | None,
    ) -> MoveTransition:
        if not profile_name:
            raise TransitionError("move_distance requires profile_name")
        if profile_name not in self._move_transitions:
            raise TransitionError(f"unknown move_distance profile: {profile_name}")
        transition = self._move_transitions[profile_name]
        if transition.from_state != self._state.value:
            raise TransitionError("move_distance profile does not match current state")
        try:
            TaskState(transition.to_state)
        except ValueError as error:
            raise TransitionError(
                f"move_distance profile has invalid target state: {transition.to_state}"
            ) from error
        return transition

    def _apply_stop(self, success: bool, box_hazard: bool) -> TaskState:
        self._validate_nonterminal("stop_task")
        if not success:
            self._requires_operator_review = True
            self._state = TaskState.OPERATOR_REVIEW
            return self._state
        review_required = (
            self._requires_operator_review
            or box_hazard
            or self._state in _HELD_BOX_STATES
        )
        if self._state is TaskState.FAILED_STOPPING:
            self._requires_operator_review = review_required
            self._state = (
                TaskState.OPERATOR_REVIEW
                if review_required
                else TaskState.FAILED_STOPPED
            )
        elif self._state is not TaskState.OPERATOR_REVIEW:
            self._requires_operator_review = review_required
            self._failure_kind = None
            self._state = (
                TaskState.OPERATOR_REVIEW
                if review_required
                else TaskState.FAILED_STOPPED
            )
        return self._state

    def _validate_nonterminal(self, operation: str) -> None:
        if self._state in _TERMINAL_STATES:
            raise TransitionError(f"{operation} is illegal in a terminal state")
