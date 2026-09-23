from __future__ import annotations

import pytest

from collections.abc import Mapping

from robots.lynsense_real_box.site_profile import MoveTransition
from robots.lynsense_real_box.task_state import (
    TaskState,
    TaskStateMachine,
    ToolName,
    TransitionError,
)


def move(name: str, from_state: str, to_state: str,
         distance_m: float, angle_deg: float) -> MoveTransition:
    return MoveTransition(
        name=name,
        from_state=from_state,
        to_state=to_state,
        distance_m=distance_m,
        angle_deg=angle_deg,
    )


MOVE_TRANSITIONS: Mapping[str, MoveTransition] = {
    item.name: item
    for item in (
        move("retreat_after_pick", "box_grasped", "carrying", -0.6, 0.0),
        move("turn_after_pick", "carrying", "carrying", 0.0, 90.0),
        move("retreat_after_place", "box_released", "withdrawn", -0.6, 0.0),
    )
}


def prepared_pick_machine() -> TaskStateMachine:
    machine = TaskStateMachine(TaskState.AT_PICK_APPROACH)
    machine.apply(ToolName.MOVE_WAIST, profile_name="pick_ready")
    machine.apply(ToolName.SET_DUAL_GRIPPERS, profile_name="close")
    machine.apply(ToolName.MOVE_DUAL_ARMS, profile_name="pick_ready")
    return machine


def withdrawn_machine() -> TaskStateMachine:
    machine = TaskStateMachine(
        TaskState.BOX_RELEASED,
        move_transitions=MOVE_TRANSITIONS,
    )
    machine.apply(ToolName.MOVE_DISTANCE, profile_name="retreat_after_place")
    return machine


def test_normal_success_path_is_serial():
    machine = TaskStateMachine(move_transitions=MOVE_TRANSITIONS)
    steps = [
        (ToolName.DETECT_BOX, None),
        (ToolName.NAV_TO_POSE, "pickup"),
        (ToolName.MOVE_WAIST, "pick_ready"),
        (ToolName.SET_DUAL_GRIPPERS, "close"),
        (ToolName.MOVE_DUAL_ARMS, "pick_ready"),
        (ToolName.PICK_BOX, None),
        (ToolName.MOVE_DISTANCE, "retreat_after_pick"),
        (ToolName.MOVE_DISTANCE, "turn_after_pick"),
        (ToolName.NAV_TO_POSE, "placement"),
        (ToolName.MOVE_DUAL_ARMS, "place_ready"),
        (ToolName.PLACE_BOX, None),
        (ToolName.MOVE_DISTANCE, "retreat_after_place"),
        (ToolName.MOVE_DUAL_ARMS, "safe"),
        (ToolName.SET_DUAL_GRIPPERS, "open"),
        (ToolName.MOVE_WAIST, "safe"),
    ]
    expected = [
        TaskState.BOX_LOCALIZED,
        TaskState.AT_PICK_APPROACH,
        TaskState.AT_PICK_APPROACH,
        TaskState.AT_PICK_APPROACH,
        TaskState.DUAL_PICK_PREPARED,
        TaskState.BOX_GRASPED,
        TaskState.CARRYING,
        TaskState.CARRYING,
        TaskState.AT_PLACE_APPROACH,
        TaskState.DUAL_PLACE_PREPARED,
        TaskState.BOX_RELEASED,
        TaskState.WITHDRAWN,
        TaskState.WITHDRAWN,
        TaskState.WITHDRAWN,
        TaskState.SAFE_COMPLETE,
    ]
    assert len(steps) == len(expected)
    for index, (tool, profile_name) in enumerate(steps):
        machine.apply(tool, profile_name=profile_name)
        assert machine.state is expected[index]
    assert machine.state is TaskState.SAFE_COMPLETE


def test_read_tools_do_not_advance_state():
    machine = TaskStateMachine(TaskState.AT_PICK_APPROACH)
    for tool in (ToolName.READ_TASK_STATE, ToolName.READ_ROBOT_STATE,
                 ToolName.DETECT_BOX):
        machine.apply(tool)
    assert machine.state is TaskState.AT_PICK_APPROACH


def test_failed_detect_from_initialized_retains_state():
    machine = TaskStateMachine()
    machine.apply(ToolName.DETECT_BOX, success=False)
    assert machine.state is TaskState.INITIALIZED


def test_stop_time_hazard_requires_review_after_ordinary_failure():
    machine = TaskStateMachine(TaskState.AT_PICK_APPROACH)
    machine.apply(ToolName.MOVE_WAIST, profile_name="pick_ready")
    machine.apply(ToolName.SET_DUAL_GRIPPERS, profile_name="close")
    machine.apply(ToolName.MOVE_DUAL_ARMS, profile_name="pick_ready")
    machine.apply(
        ToolName.PICK_BOX,
        success=False,
        failure_kind="action_failed",
    )
    machine.apply(ToolName.STOP_TASK, success=True, box_hazard=True)
    assert machine.state is TaskState.OPERATOR_REVIEW


def test_unknown_failure_kind_requires_review():
    machine = TaskStateMachine(TaskState.AT_PICK_APPROACH)
    machine.apply(ToolName.MOVE_WAIST, profile_name="pick_ready")
    machine.apply(ToolName.SET_DUAL_GRIPPERS, profile_name="close")
    machine.apply(ToolName.MOVE_DUAL_ARMS, profile_name="pick_ready")
    machine.apply(
        ToolName.PICK_BOX,
        success=False,
        failure_kind="new_unrecognized_fault",
    )
    assert machine.requires_operator_review is True


def test_directly_prepared_state_requires_preparation_flags():
    machine = TaskStateMachine(TaskState.DUAL_PICK_PREPARED)
    with pytest.raises(TransitionError):
        machine.apply(ToolName.PICK_BOX)
    assert machine.state is TaskState.DUAL_PICK_PREPARED


def test_motion_failure_enters_stopping_then_stopped():
    machine = prepared_pick_machine()
    machine.apply(ToolName.PICK_BOX, success=False)
    assert machine.state is TaskState.FAILED_STOPPING
    machine.apply(ToolName.STOP_TASK, success=True)
    assert machine.state is TaskState.FAILED_STOPPED


def test_held_box_hazard_requires_operator_review():
    machine = prepared_pick_machine()
    machine.apply(ToolName.PICK_BOX, success=False, box_hazard=True)
    assert machine.state is TaskState.FAILED_STOPPING
    machine.apply(ToolName.STOP_TASK, success=True, box_hazard=True)
    assert machine.state is TaskState.OPERATOR_REVIEW


@pytest.mark.parametrize(
    ("initial_state", "tool", "failure_kind"),
    [
        (TaskState.DUAL_PICK_PREPARED, ToolName.PICK_BOX, "cancel_unknown"),
        (TaskState.DUAL_PICK_PREPARED, ToolName.PICK_BOX, "dual_arm_disagreement"),
        (TaskState.DUAL_PICK_PREPARED, ToolName.PICK_BOX, "gripper_disagreement"),
        (TaskState.DUAL_PICK_PREPARED, ToolName.PICK_BOX, "missing_grasp_evidence"),
        (TaskState.DUAL_PLACE_PREPARED, ToolName.PLACE_BOX, "release_verification_failed"),
        (TaskState.CARRYING, ToolName.NAV_TO_POSE, "localization_loss"),
    ],
)
def test_structured_physical_failures_require_operator_review(
    initial_state, tool, failure_kind
):
    if tool is ToolName.PICK_BOX:
        machine = prepared_pick_machine()
    else:
        machine = TaskStateMachine(
            initial_state,
            move_transitions=MOVE_TRANSITIONS,
        )
    machine.apply(
        tool,
        success=False,
        failure_kind=failure_kind,
    )
    assert machine.failure_kind == failure_kind
    assert machine.requires_operator_review is True
    machine.apply(ToolName.STOP_TASK, success=True)
    assert machine.state is TaskState.OPERATOR_REVIEW


@pytest.mark.parametrize("failure_kind", ["action_failed", "timeout", "canceled"])
def test_confirmed_stop_is_sufficient_for_ordinary_failures(failure_kind):
    machine = prepared_pick_machine()
    machine.apply(
        ToolName.PICK_BOX,
        success=False,
        failure_kind=failure_kind,
    )
    assert machine.requires_operator_review is False
    machine.apply(ToolName.STOP_TASK, success=True)
    assert machine.state is TaskState.FAILED_STOPPED


def test_stop_from_ordinary_state_is_idempotent_after_confirmation():
    machine = TaskStateMachine(TaskState.AT_PICK_APPROACH)
    machine.apply(ToolName.STOP_TASK, success=True)
    assert machine.state is TaskState.FAILED_STOPPED
    machine.apply(ToolName.STOP_TASK, success=True)
    assert machine.state is TaskState.FAILED_STOPPED
    machine.mark_operator_stopped()
    assert machine.state is TaskState.OPERATOR_ACKNOWLEDGED_STOPPED


def test_stop_failure_requires_operator_review():
    machine = TaskStateMachine(
        TaskState.CARRYING,
        move_transitions=MOVE_TRANSITIONS,
    )
    machine.apply(
        ToolName.MOVE_DISTANCE,
        profile_name="turn_after_pick",
        success=False,
    )
    machine.apply(ToolName.STOP_TASK, success=False)
    assert machine.state is TaskState.OPERATOR_REVIEW


def test_external_estop_and_asynchronous_fault_enter_operator_review():
    estop_machine = TaskStateMachine(TaskState.CARRYING)
    assert estop_machine.mark_external_estop() is TaskState.OPERATOR_REVIEW

    fault_machine = TaskStateMachine(TaskState.AT_PICK_APPROACH)
    assert fault_machine.mark_operator_review(
        "asynchronous_driver_fault"
    ) is TaskState.OPERATOR_REVIEW


def test_failure_states_allow_reads_and_stop_but_no_motion():
    machine = TaskStateMachine(TaskState.FAILED_STOPPED)
    allowed = machine.allowed_tools()
    assert ToolName.READ_ROBOT_STATE in allowed
    assert ToolName.STOP_TASK in allowed
    assert ToolName.MOVE_DUAL_ARMS not in allowed
    assert ToolName.PICK_BOX not in allowed


def test_pick_requires_all_three_pick_preparations():
    machine = TaskStateMachine(TaskState.AT_PICK_APPROACH)
    machine.apply(ToolName.MOVE_WAIST, profile_name="pick_ready")
    with pytest.raises(TransitionError):
        machine.apply(ToolName.PICK_BOX)
    machine.apply(ToolName.SET_DUAL_GRIPPERS, profile_name="close")
    with pytest.raises(TransitionError):
        machine.apply(ToolName.PICK_BOX)
    machine.apply(ToolName.MOVE_DUAL_ARMS, profile_name="pick_ready")
    assert machine.state is TaskState.DUAL_PICK_PREPARED
    assert machine.apply(ToolName.PICK_BOX) is TaskState.BOX_GRASPED


@pytest.mark.parametrize(
    ("waist_profile", "gripper_command", "arms_profile"),
    [
        ("safe", "open", "pick_ready"),
        ("safe", "open", "place_ready"),
        ("safe", "close", "place_ready"),
        ("safe", "close", "safe"),
        ("pick_ready", "open", "pick_ready"),
        ("pick_ready", "open", "place_ready"),
        ("pick_ready", "open", "safe"),
        ("pick_ready", "close", "place_ready"),
        ("pick_ready", "close", "safe"),
        ("safe", "close", "pick_ready"),
    ],
)
def test_non_safe_preparation_combinations_cannot_finish_success(
    waist_profile: str,
    gripper_command: str,
    arms_profile: str,
) -> None:
    machine = withdrawn_machine()

    with pytest.raises(TransitionError):
        machine.apply(ToolName.MOVE_WAIST, profile_name=waist_profile)
        machine.apply(ToolName.SET_DUAL_GRIPPERS, profile_name=gripper_command)
        machine.apply(ToolName.MOVE_DUAL_ARMS, profile_name=arms_profile)

    assert machine.state is TaskState.WITHDRAWN
    with pytest.raises(TransitionError):
        machine.assert_may_finish("success")


def test_exact_safe_profiles_are_required_for_success() -> None:
    machine = withdrawn_machine()
    machine.apply(ToolName.MOVE_DUAL_ARMS, profile_name="safe")
    machine.apply(ToolName.SET_DUAL_GRIPPERS, profile_name="open")
    machine.apply(ToolName.MOVE_WAIST, profile_name="safe")

    assert machine.state is TaskState.SAFE_COMPLETE
    machine.assert_may_finish("success")


@pytest.mark.parametrize(
    ("waist_profile", "gripper_command", "arms_profile"),
    [
        ("safe", "close", "pick_ready"),
        ("pick_ready", "open", "pick_ready"),
        ("pick_ready", "close", "place_ready"),
        ("safe", "close", "place_ready"),
    ],
)
def test_pick_preparations_use_context_specific_profiles(
    waist_profile: str,
    gripper_command: str,
    arms_profile: str,
) -> None:
    machine = TaskStateMachine(TaskState.AT_PICK_APPROACH)

    with pytest.raises(TransitionError):
        machine.apply(ToolName.MOVE_WAIST, profile_name=waist_profile)
        machine.apply(ToolName.SET_DUAL_GRIPPERS, profile_name=gripper_command)
        machine.apply(ToolName.MOVE_DUAL_ARMS, profile_name=arms_profile)

    assert machine.state is TaskState.AT_PICK_APPROACH
    with pytest.raises(TransitionError):
        machine.apply(ToolName.PICK_BOX)


def test_place_preparation_requires_the_place_profile() -> None:
    machine = TaskStateMachine(TaskState.AT_PLACE_APPROACH)

    with pytest.raises(TransitionError):
        machine.apply(ToolName.MOVE_DUAL_ARMS, profile_name="pick_ready")

    assert machine.state is TaskState.AT_PLACE_APPROACH


def test_move_transition_profile_must_match_current_state():
    machine = TaskStateMachine(
        TaskState.BOX_GRASPED,
        move_transitions=MOVE_TRANSITIONS,
    )
    with pytest.raises(TransitionError):
        machine.apply(
            ToolName.MOVE_DISTANCE,
            profile_name="turn_after_pick",
        )


@pytest.mark.parametrize(
    ("state", "tool"),
    [
        (TaskState.INITIALIZED, ToolName.PICK_BOX),
        (TaskState.BOX_GRASPED, ToolName.PLACE_BOX),
        (TaskState.SAFE_COMPLETE, ToolName.MOVE_DUAL_ARMS),
        (TaskState.OPERATOR_ACKNOWLEDGED_STOPPED, ToolName.MOVE_DUAL_ARMS),
    ],
)
def test_illegal_motion_is_rejected(state, tool):
    machine = TaskStateMachine(state)
    with pytest.raises(TransitionError):
        machine.apply(tool, profile_name="pick_ready")


def test_operator_acknowledgement_has_source_restrictions():
    machine = TaskStateMachine(TaskState.SAFE_COMPLETE)
    with pytest.raises(TransitionError):
        machine.mark_operator_stopped()


def test_finish_legality_is_explicit():
    machine = TaskStateMachine()
    with pytest.raises(TransitionError):
        machine.assert_may_finish("success")
    machine.apply(ToolName.DETECT_BOX)
    with pytest.raises(TransitionError):
        machine.assert_may_finish("failure")
    safe_machine = TaskStateMachine(TaskState.SAFE_COMPLETE)
    safe_machine.assert_may_finish("success")
    stopped_machine = TaskStateMachine(TaskState.OPERATOR_ACKNOWLEDGED_STOPPED)
    stopped_machine.assert_may_finish("failure")


@pytest.mark.parametrize("initial_state", [TaskState.BOX_GRASPED, TaskState.CARRYING])
def test_stop_from_a_held_box_requires_operator_confirmation(initial_state):
    machine = TaskStateMachine(
        initial_state,
        move_transitions=MOVE_TRANSITIONS,
    )

    assert machine.apply(ToolName.STOP_TASK, success=True) is TaskState.OPERATOR_REVIEW
    assert machine.requires_operator_review is True
    with pytest.raises(TransitionError):
        machine.assert_may_finish("success")

    machine.mark_operator_stopped()
    machine.assert_may_finish("failure")
