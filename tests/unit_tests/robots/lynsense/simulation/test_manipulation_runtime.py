from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path

import pytest

from lynsense_webots_sim.action_runtime import Acceptance, Outcome
from lynsense_webots_sim.box_config import load_box_config
from lynsense_webots_sim.geometry import Pose2
from lynsense_webots_sim.manipulation_runtime import (
    ManipulationGoal,
    ManipulationKind,
    ManipulationRuntime,
    ManipulationState,
)


CONFIG = load_box_config(
    Path("robots")
    / "lynsense"
    / "simulation"
    / "ros"
    / "lynsense_webots_sim"
    / "config"
    / "box.yaml"
)
TARGET = CONFIG.named_targets["dualjo:joints_br"]
LEFT = TARGET.left_joints_rad
RIGHT = TARGET.right_joints_rad


def goal_waist(height_mm: float = 200.0) -> ManipulationGoal:
    return ManipulationGoal(ManipulationKind.WAIST, height_mm=height_mm)


def goal_named(target: str = "dualjo:joints_br") -> ManipulationGoal:
    return ManipulationGoal(ManipulationKind.NAMED_CONFIG, target=target)


def goal_gripper(position: float = 0.0) -> ManipulationGoal:
    return ManipulationGoal(ManipulationKind.GRIPPER, gripper_position=position)


def goal_pick() -> ManipulationGoal:
    return ManipulationGoal(
        ManipulationKind.BOX_PHASE,
        box_action="pick",
        flow="flow",
        config_name="box1",
    )


def goal_place() -> ManipulationGoal:
    return ManipulationGoal(
        ManipulationKind.BOX_PHASE,
        box_action="place",
        flow="flow",
        config_name="box1",
    )


def runtime_with_successful_pick() -> ManipulationRuntime:
    runtime = ManipulationRuntime(CONFIG, initial_time_s=0.0)
    assert runtime.submit(goal_pick(), state_factory()).accepted
    runtime.observe(state_factory())
    assert runtime.observe(state_factory(box_attached=True)).success
    runtime.reset(10.0)
    return runtime


def state_factory(**overrides: object) -> ManipulationState:
    values = {
        "sim_time_s": 10.0,
        "waist_position_m": -0.278,
        "arm_positions_rad": LEFT + RIGHT,
        "gripper_positions": (0.0, 0.0),
        "box_position_m": CONFIG.box.initial_pose_m,
        "box_orientation_rad": (0.0, 0.0, CONFIG.box.initial_yaw_rad),
        "robot_pose": Pose2(2.024931, -2.493846, 3.124139),
        "box_attached": False,
    }
    values.update(overrides)
    return ManipulationState(**values)


def test_waist_moves_and_settles() -> None:
    runtime = ManipulationRuntime(CONFIG, initial_time_s=0.0)
    assert runtime.submit(goal_waist(), state_factory(waist_position_m=-0.478)).accepted

    first = runtime.observe(state_factory(waist_position_m=-0.478))
    assert first.outcome is Outcome.RUNNING
    assert first.phase == "moving"
    assert first.remaining_mm > 0.0

    done = runtime.observe(state_factory(waist_position_m=-0.2781))
    assert done.terminal and done.success and done.phase == "settling"
    assert runtime.commands().waist_velocity_mps == 0.0


def test_named_config_moves_both_arms_and_rejects_unknown_target() -> None:
    runtime = ManipulationRuntime(CONFIG, initial_time_s=0.0)
    rejected = runtime.submit(goal_named("dualjo:not_allowed"), state_factory())
    assert not rejected.accepted
    assert rejected.reason == "unknown_target"

    assert runtime.submit(
        goal_named(),
        state_factory(arm_positions_rad=(0.0,) * 12, gripper_positions=(0.0, 0.0)),
    ).accepted
    running = runtime.observe(
        state_factory(arm_positions_rad=(0.0,) * 12, gripper_positions=(0.0, 0.0))
    )
    assert running.phase == "interpolating"
    assert any(value != 0.0 for value in runtime.commands().arm_velocities_radps)

    done = runtime.observe(state_factory())
    assert done.terminal and done.success
    assert runtime.commands().arm_velocities_radps == (0.0,) * 12


def test_gripper_opens_and_closes() -> None:
    runtime = ManipulationRuntime(CONFIG, initial_time_s=0.0)
    assert runtime.submit(goal_gripper(0.0), state_factory(gripper_positions=(0.6, 0.6))).accepted
    running = runtime.observe(state_factory(gripper_positions=(0.6, 0.6)))
    assert running.phase == "closing_or_opening"
    done = runtime.observe(state_factory())
    assert done.terminal and done.success


def test_pick_requires_alignment_and_closed_grippers_before_attachment() -> None:
    runtime = ManipulationRuntime(CONFIG, initial_time_s=0.0)
    assert runtime.submit(goal_pick(), state_factory()).accepted
    verifying = runtime.observe(state_factory())
    assert verifying.phase == "verifying"
    assert runtime.commands().attach_box is True

    attached = runtime.observe(state_factory(box_attached=True))
    assert attached.terminal and attached.success


def test_pick_accepts_pi_equivalent_box_orientation() -> None:
    runtime = ManipulationRuntime(CONFIG, initial_time_s=0.0)
    state = state_factory(
        box_orientation_rad=(0.0, 0.0, math.pi),
        box_attached=True,
    )
    assert runtime.submit(goal_pick(), state).accepted
    result = runtime.observe(state)
    assert result.terminal and result.success


def test_pick_fails_for_alignment_and_grasp_preconditions() -> None:
    runtime = ManipulationRuntime(CONFIG, initial_time_s=0.0)
    misaligned = state_factory(
        box_position_m=(
            CONFIG.box.initial_pose_m[0] + 0.1,
            *CONFIG.box.initial_pose_m[1:],
        )
    )
    assert runtime.submit(
        goal_pick(),
        misaligned,
    ).accepted
    result = runtime.observe(misaligned)
    assert result.terminal and not result.success
    assert result.reason == "box_alignment_failed"

    runtime = ManipulationRuntime(CONFIG, initial_time_s=0.0)
    assert runtime.submit(goal_pick(), state_factory(gripper_positions=(0.6, 0.6))).accepted
    result = runtime.observe(state_factory(gripper_positions=(0.6, 0.6)))
    assert result.reason == "grasp_failed"


def test_place_releases_only_after_opening_and_settle() -> None:
    runtime = runtime_with_successful_pick()
    attached = state_factory(
        box_attached=True,
        arm_positions_rad=CONFIG.named_targets["dualposi_armbase_abso:pt_1f1_ready"].left_joints_rad
        + CONFIG.named_targets["dualposi_armbase_abso:pt_1f1_ready"].right_joints_rad,
        box_position_m=CONFIG.box.place_target_m,
        box_orientation_rad=(0.0, 0.0, CONFIG.box.place_target_yaw_rad),
        robot_pose=Pose2(2.460559, -2.734170, 0.006126),
    )
    assert runtime.submit(goal_place(), attached).accepted
    opening = runtime.observe(attached)
    assert opening.phase == "opening"
    assert runtime.commands().release_box is False
    assert runtime.commands().gripper_velocities == (1.0, 1.0)

    opened = replace(attached, gripper_positions=(0.6, 0.6), sim_time_s=10.1)
    releasing = runtime.observe(opened)
    assert releasing.phase == "releasing"
    assert runtime.commands().release_box is True
    assert runtime.commands().gripper_velocities == (0.0, 0.0)

    settled = replace(
        opened,
        box_attached=False,
        sim_time_s=10.7,
    )
    first_settling = runtime.observe(settled)
    assert first_settling.phase == "settling"
    assert first_settling.terminal is False
    done = runtime.observe(replace(settled, sim_time_s=11.2))
    assert done.terminal and done.success


def test_place_requires_successful_pick_in_same_runtime() -> None:
    runtime = ManipulationRuntime(CONFIG, initial_time_s=0.0)
    attached = state_factory(
        box_attached=True,
        arm_positions_rad=CONFIG.named_targets["dualposi_armbase_abso:pt_1f1_ready"].left_joints_rad
        + CONFIG.named_targets["dualposi_armbase_abso:pt_1f1_ready"].right_joints_rad,
        box_position_m=CONFIG.box.place_target_m,
        box_orientation_rad=(0.0, 0.0, CONFIG.box.place_target_yaw_rad),
        robot_pose=Pose2(2.460559, -2.734170, 0.006126),
    )
    rejected = runtime.submit(goal_place(), attached)
    assert not rejected.accepted
    assert rejected.reason == "place_requires_pick"

    runtime = runtime_with_successful_pick()
    detached = replace(attached, box_attached=False)
    rejected = runtime.submit(goal_place(), detached)
    assert not rejected.accepted
    assert rejected.reason == "place_requires_pick"


def test_place_fails_when_target_is_misaligned() -> None:
    runtime = runtime_with_successful_pick()
    attached = state_factory(
        box_attached=True,
        box_position_m=(CONFIG.box.place_target_m[0] + 0.2, *CONFIG.box.place_target_m[1:]),
        robot_pose=Pose2(2.660559, -2.734170, 0.006126),
    )
    assert runtime.submit(goal_place(), attached).accepted
    result = runtime.observe(attached)
    assert result.terminal and not result.success
    assert result.reason == "place_alignment_failed"


def test_release_settle_rejects_place_target_drift() -> None:
    runtime = runtime_with_successful_pick()
    attached = state_factory(
        box_attached=True,
        arm_positions_rad=CONFIG.named_targets["dualposi_armbase_abso:pt_1f1_ready"].left_joints_rad
        + CONFIG.named_targets["dualposi_armbase_abso:pt_1f1_ready"].right_joints_rad,
        box_position_m=CONFIG.box.place_target_m,
        box_orientation_rad=(0.0, 0.0, CONFIG.box.place_target_yaw_rad),
        robot_pose=Pose2(2.460559, -2.734170, 0.006126),
        gripper_positions=(0.6, 0.6),
    )
    runtime.submit(goal_place(), attached)
    runtime.observe(attached)
    released = replace(attached, box_attached=False, sim_time_s=10.2)
    assert runtime.observe(released).phase == "settling"

    drifted = replace(
        released,
        box_position_m=(CONFIG.box.place_target_m[0] + 0.2, *CONFIG.box.place_target_m[1:]),
        sim_time_s=10.7,
    )
    result = runtime.observe(drifted)
    assert result.terminal and not result.success
    assert result.reason == "place_alignment_failed"


def test_release_settle_rejects_bad_box_attitude() -> None:
    runtime = runtime_with_successful_pick()
    attached = state_factory(
        box_attached=True,
        arm_positions_rad=CONFIG.named_targets["dualposi_armbase_abso:pt_1f1_ready"].left_joints_rad
        + CONFIG.named_targets["dualposi_armbase_abso:pt_1f1_ready"].right_joints_rad,
        box_position_m=CONFIG.box.place_target_m,
        box_orientation_rad=(0.0, 0.0, CONFIG.box.place_target_yaw_rad),
        robot_pose=Pose2(2.460559, -2.734170, 0.006126),
        gripper_positions=(0.6, 0.6),
    )
    runtime.submit(goal_place(), attached)
    runtime.observe(attached)
    settling = runtime.observe(replace(attached, box_attached=False, sim_time_s=10.2))
    assert settling.phase == "settling"

    runtime = runtime_with_successful_pick()
    runtime.submit(goal_place(), attached)
    runtime.observe(attached)
    runtime.observe(attached)
    result = runtime.observe(
        replace(
            attached,
            box_attached=False,
            box_orientation_rad=(0.2, 0.0, 0.0),
            sim_time_s=10.7,
        )
    )
    assert result.reason == "release_settle_failed"


def test_timeout_stall_cancel_and_second_goal() -> None:
    runtime = ManipulationRuntime(CONFIG, initial_time_s=0.0)
    assert runtime.submit(goal_waist(), state_factory(waist_position_m=-0.478)).accepted
    assert not runtime.submit(goal_gripper(), state_factory(waist_position_m=-0.478)).accepted
    result = runtime.observe(
        state_factory(waist_position_m=-0.478, sim_time_s=CONFIG.timeouts.waist_s + 11.0)
    )
    assert result.reason == "motion_timeout"

    runtime = ManipulationRuntime(CONFIG, initial_time_s=0.0)
    runtime.submit(goal_named(), state_factory(arm_positions_rad=(0.0,) * 12))
    result = runtime.observe(
        state_factory(arm_positions_rad=(0.0,) * 12, sim_time_s=CONFIG.timeouts.stall_window_s + 12.0)
    )
    assert result.reason == "joint_stall"

    runtime = ManipulationRuntime(CONFIG, initial_time_s=0.0)
    runtime.submit(goal_gripper(), state_factory(gripper_positions=(0.6, 0.6)))
    assert runtime.request_cancel()
    result = runtime.observe(state_factory(gripper_positions=(0.6, 0.6)))
    assert result.outcome is Outcome.CANCELED
    assert result.reason == "cancel_requested"


def test_attachment_monitor_detects_carry_drift_and_rejects_invalid_state() -> None:
    runtime = ManipulationRuntime(CONFIG, initial_time_s=0.0)
    runtime.submit(goal_pick(), state_factory())
    runtime.observe(state_factory())
    attached = state_factory(box_attached=True)
    runtime.observe(attached)
    assert runtime.attachment_fault(attached) == ""
    moved_robot = replace(attached, robot_pose=Pose2(2.124931, -2.493846, 3.124139))
    assert runtime.attachment_fault(moved_robot) == "carry_lost"


def test_attachment_monitor_accepts_pi_symmetric_box_orientation() -> None:
    runtime = ManipulationRuntime(CONFIG, initial_time_s=0.0)
    runtime.submit(goal_pick(), state_factory())
    runtime.observe(state_factory())
    attached = state_factory(box_attached=True)
    assert runtime.observe(attached).success

    carried = replace(
        attached,
        robot_pose=Pose2(2.024931, -2.493846, 0.0),
        box_position_m=(2.374931, -2.493846, 0.1085),
        box_orientation_rad=(0.0, 0.0, 0.0),
    )
    assert runtime.attachment_fault(carried) == ""

    with pytest.raises(ValueError, match="finite"):
        runtime.observe(
            replace(
                attached,
                arm_positions_rad=(float("nan"), *(attached.arm_positions_rad[1:])),
            )
        )


def test_reset_accepts_next_goal_and_preserves_carry_monitor() -> None:
    runtime = ManipulationRuntime(CONFIG, initial_time_s=0.0)
    runtime.submit(goal_pick(), state_factory())
    runtime.observe(state_factory())
    attached = state_factory(box_attached=True)
    assert runtime.observe(attached).success
    assert runtime.submit(goal_waist(), attached).reason == "terminal_lockout"

    runtime.reset(11.0)
    assert runtime.submit(goal_waist(), attached).accepted
    runtime.request_cancel()
    runtime.observe(attached)

    runtime.reset(12.0)
    moved_robot = replace(attached, robot_pose=Pose2(2.124931, -2.493846, 3.124139))
    assert runtime.attachment_fault(moved_robot) == "carry_lost"
