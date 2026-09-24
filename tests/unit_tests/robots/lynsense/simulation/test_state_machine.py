import math

import pytest

from lynsense_webots_sim.geometry import Pose2
from lynsense_webots_sim.state_machine import (
    MotionPhase,
    MotionProfile,
    NavigationStateMachine,
    Outcome,
)


def test_move_distance_turns_before_negative_translation():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=-1.2, angle_deg=90, sim_time_s=0)
    turning = machine.update(Pose2(0, 0, 0), 0.032)
    assert turning.phase is MotionPhase.MOVE_TURN
    assert turning.feedback.distance_remaining == pytest.approx(1.2)
    assert turning.feedback.angle_remaining > 0

    after_turn = machine.update(Pose2(0, 0, math.pi / 2), 2.0)
    assert machine.phase is MotionPhase.MOVE_TRANSLATE
    assert after_turn.feedback.angle_remaining == pytest.approx(0.0)


def test_negative_move_distance_commands_negative_wheel_rates():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=-1.5, angle_deg=0, sim_time_s=0)

    result = machine.update(Pose2(0, 0, 0), 0.032)

    assert result.phase is MotionPhase.MOVE_TRANSLATE
    assert result.command.left_wheel_rate_radps < 0.0
    assert result.command.right_wheel_rate_radps < 0.0


def test_move_turn_distance_remaining_ignores_translation_drift():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=1.0, angle_deg=90, sim_time_s=0)

    drifting = machine.update(Pose2(0.01, 0.01, 0.2), 0.1)

    assert drifting.phase is MotionPhase.MOVE_TURN
    assert drifting.feedback.distance_remaining == pytest.approx(1.0)
    assert drifting.feedback.angle_remaining < 90.0


def test_turn_stall_ignores_translation_progress():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=1.0, angle_deg=90, sim_time_s=0)
    machine.update(Pose2(0, 0, 0), 0.032)
    result = machine.update(Pose2(0, 0, 0), 1.1)
    assert result.outcome is Outcome.FAILED
    assert result.reason == "turn_stalled"


def test_translate_stall_ignores_yaw_progress():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=1.0, angle_deg=0, sim_time_s=0)
    machine.update(Pose2(0, 0, 0), 0.032)
    result = machine.update(Pose2(0.001, 0, 0), 1.1)
    assert result.outcome is Outcome.FAILED
    assert result.reason == "translate_stalled"


def test_turn_stall_is_detected_with_periodic_updates():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=1.0, angle_deg=90, sim_time_s=0)

    result = None
    for step in range(1, 56):
        result = machine.update(Pose2(0, 0, 0), step * 0.02)
        if result.outcome is not Outcome.RUNNING:
            break

    assert result is not None
    assert result.outcome is Outcome.FAILED
    assert result.reason == "turn_stalled"


def test_translate_stall_is_detected_with_periodic_updates():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=1.0, angle_deg=0, sim_time_s=0)

    result = None
    for step in range(1, 56):
        result = machine.update(Pose2(0, 0, 0), step * 0.02)
        if result.outcome is not Outcome.RUNNING:
            break

    assert result is not None
    assert result.outcome is Outcome.FAILED
    assert result.reason == "translate_stalled"


def test_sufficient_turn_progress_renews_stall_window():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=1.0, angle_deg=180, sim_time_s=0)

    machine.update(Pose2(0, 0, 0.6), 0.2)
    renewed = machine.update(Pose2(0, 0, 0.6), 1.1)
    assert renewed.outcome is Outcome.RUNNING

    result = machine.update(Pose2(0, 0, 0.6), 1.3)
    assert result.outcome is Outcome.FAILED
    assert result.reason == "turn_stalled"


def test_reverse_turn_does_not_renew_stall_window():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=1.0, angle_deg=90, sim_time_s=0)

    machine.update(Pose2(0, 0, -0.6), 0.2)
    result = machine.update(Pose2(0, 0, -0.6), 1.1)

    assert result.outcome is Outcome.FAILED
    assert result.reason == "turn_stalled"


def test_reverse_translation_does_not_renew_stall_window():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=1.0, angle_deg=0, sim_time_s=0)

    machine.update(Pose2(-0.01, 0, 0), 0.2)
    result = machine.update(Pose2(-0.01, 0, 0), 1.3)

    assert result.outcome is Outcome.FAILED
    assert result.reason == "translate_stalled"


def test_lateral_translation_does_not_renew_stall_window():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=1.0, angle_deg=0, sim_time_s=0)

    machine.update(Pose2(0, 0.01, 0), 0.2)
    result = machine.update(Pose2(0, 0.01, 0), 1.3)

    assert result.outcome is Outcome.FAILED
    assert result.reason == "translate_stalled"


def test_terminal_failure_retains_last_remaining_distances():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=1.0, angle_deg=0, sim_time_s=0)

    machine.update(Pose2(0, 0, 0), 0.032)
    result = machine.update(Pose2(0, 0, 0), 1.1)

    assert result.outcome is Outcome.FAILED
    assert result.feedback.distance_remaining == pytest.approx(1.0)


def test_nav_final_yaw_stall_uses_yaw_progress_with_periodic_updates():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_nav(
        Pose2(0, 0, math.pi / 2),
        Pose2(2, 0, math.pi / 2),
        sim_time_s=0,
    )
    machine.update(Pose2(0, 0, math.pi / 2), 0.02)
    machine.update(Pose2(0, 0, 0), 0.04)
    entered_final_yaw = machine.update(Pose2(2, 0, 0), 0.06)
    assert entered_final_yaw.phase is MotionPhase.NAV_FINAL_YAW

    result = None
    for step in range(4, 56):
        result = machine.update(Pose2(2, 0, 0), step * 0.02)
        if result.outcome is not Outcome.RUNNING:
            break

    assert result is not None
    assert result.outcome is Outcome.FAILED
    assert result.reason == "turn_stalled"


def test_nav_to_pose_moves_through_turn_approach_and_final_yaw():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_nav(Pose2(0, 0, math.pi / 2), Pose2(2, 0, math.pi / 2), 0)

    turning = machine.update(Pose2(0, 0, math.pi / 2), 0.1)
    assert turning.phase is MotionPhase.NAV_TURN

    approaching = machine.update(Pose2(0, 0, 0), 0.2)
    assert machine.phase is MotionPhase.NAV_APPROACH
    assert approaching.feedback.distance_remaining == pytest.approx(2.0)

    final_yaw = machine.update(Pose2(2, 0, 0), 0.3)
    assert machine.phase is MotionPhase.NAV_FINAL_YAW
    assert final_yaw.feedback.distance_remaining == pytest.approx(0.0)
    assert final_yaw.feedback.angle_remaining == pytest.approx(math.degrees(math.pi / 2))


def test_phase_transition_resets_stage_timeout():
    profile = MotionProfile(total_budget_s=100.0, stall_window_s=100.0)
    machine = NavigationStateMachine(profile)
    machine.start_nav(Pose2(0, 0, 0), Pose2(2, 0, 0), 0)
    machine.update(Pose2(0, 0, 0), 0.1)
    machine.update(Pose2(0, 0, 0), 0.7)
    assert machine.phase is MotionPhase.NAV_APPROACH

    result = machine.update(Pose2(0, 0, 0), 1.2)
    assert result.phase is MotionPhase.NAV_APPROACH
    assert result.outcome is Outcome.RUNNING
    assert result.reason == ""


def test_nav_turn_carries_goal_distance_into_approach_timeout():
    profile = MotionProfile(total_budget_s=100.0, stall_window_s=100.0)
    machine = NavigationStateMachine(profile)
    machine.start_nav(Pose2(0, 0, math.pi / 2), Pose2(2, 0, 0), 0)

    turning = machine.update(Pose2(0, 0, math.pi / 2), 0.1)
    assert turning.phase is MotionPhase.NAV_TURN
    assert turning.feedback.distance_remaining == pytest.approx(2.0)

    machine.update(Pose2(0, 0, 0), 0.2)
    assert machine.phase is MotionPhase.NAV_APPROACH
    assert machine._stage_timeout_s == pytest.approx(22.0)

    result = machine.update(Pose2(0, 0, 0), 6.0)
    assert result.phase is MotionPhase.NAV_APPROACH
    assert result.outcome is Outcome.RUNNING


def test_stage_timeout_uses_exact_phase_reason():
    profile = MotionProfile(total_budget_s=100.0, stall_window_s=100.0)
    machine = NavigationStateMachine(profile)
    machine.start_nav(Pose2(0, 0, math.pi / 2), Pose2(2, 0, math.pi / 2), 0)
    machine.update(Pose2(0, 0, math.pi / 2), 0.1)
    result = machine.update(Pose2(0, 0, 0), 20.2)
    assert result.outcome is Outcome.FAILED
    assert result.reason == "nav_turn_timeout"


def test_total_budget_times_out():
    profile = MotionProfile(
        total_budget_s=1.0,
        stall_window_s=5.0,
        minimum_stage_timeout_s=5.0,
    )
    machine = NavigationStateMachine(profile)
    machine.start_move(Pose2(0, 0, 0), distance_m=1.0, angle_deg=0, sim_time_s=0)
    machine.update(Pose2(0, 0, 0), 0.2)
    result = machine.update(Pose2(0, 0, 0), 1.3)
    assert result.outcome is Outcome.FAILED
    assert result.reason == "total_timeout"


def test_nav_to_pose_success_requires_position_and_yaw_tolerance():
    machine = NavigationStateMachine(MotionProfile())
    goal = Pose2(0.04, 0.0, 0.2)
    machine.start_nav(Pose2(0, 0, 0), goal, 0)
    machine.update(Pose2(0, 0, 0), 0.1)
    machine.update(Pose2(0.04, 0, 0), 0.2)
    position_ready = machine.update(Pose2(0.04, 0, 0), 0.3)
    assert machine.phase is MotionPhase.NAV_FINAL_YAW
    assert position_ready.outcome is Outcome.RUNNING

    result = machine.update(goal, 0.4)
    assert result.phase is MotionPhase.TERMINAL
    assert result.outcome is Outcome.SUCCEEDED
    assert result.reason == ""
    assert result.command.left_wheel_rate_radps == 0.0
    assert result.command.right_wheel_rate_radps == 0.0


def test_nav_final_yaw_returns_to_approach_when_position_drifts_out():
    machine = NavigationStateMachine(MotionProfile())
    goal = Pose2(0.04, 0.0, 0.2)
    machine.start_nav(Pose2(0, 0, 0), goal, 0)
    machine.update(Pose2(0, 0, 0), 0.1)
    machine.update(Pose2(0.04, 0, 0), 0.2)
    assert machine.update(Pose2(0.04, 0, 0), 0.3).phase is MotionPhase.NAV_FINAL_YAW

    result = machine.update(Pose2(0.11, 0, 0), 0.4)

    assert result.phase is MotionPhase.NAV_APPROACH
    assert result.outcome is Outcome.RUNNING
    assert result.feedback.distance_remaining == pytest.approx(0.07)


def test_zero_distance_performs_turn_only():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=0.0, angle_deg=90, sim_time_s=0)
    machine.update(Pose2(0, 0, 0.4), 0.1)
    result = machine.update(Pose2(0, 0, math.pi / 2), 0.2)
    assert result.phase is MotionPhase.TERMINAL
    assert result.outcome is Outcome.SUCCEEDED
    assert result.feedback.angle_remaining == pytest.approx(0.0)


def test_zero_angle_performs_translate_only():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=1.0, angle_deg=0, sim_time_s=0)
    machine.update(Pose2(0, 0, 0), 0.1)
    machine.update(Pose2(0.5, 0, 0), 0.2)
    result = machine.update(Pose2(1.0, 0, 0), 0.3)
    assert result.phase is MotionPhase.TERMINAL
    assert result.outcome is Outcome.SUCCEEDED
    assert result.feedback.distance_remaining == pytest.approx(0.0)


def test_nonfinite_pose_fails_with_zero_command():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=1.0, angle_deg=0, sim_time_s=0)
    machine.update(Pose2(0, 0, 0), 0.1)
    result = machine.update(Pose2(math.inf, 0, 0), 0.2)
    assert result.phase is MotionPhase.TERMINAL
    assert result.outcome is Outcome.FAILED
    assert result.reason == "invalid_pose"
    assert result.command.left_wheel_rate_radps == 0.0
    assert result.command.right_wheel_rate_radps == 0.0


def test_commands_use_differential_geometry_and_smoke_limits():
    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=1.0, angle_deg=90, sim_time_s=0)
    turning = machine.update(Pose2(0, 0, 0), 0.1)
    assert turning.command.right_wheel_rate_radps > turning.command.left_wheel_rate_radps

    machine = NavigationStateMachine(MotionProfile())
    machine.start_move(Pose2(0, 0, 0), distance_m=1.0, angle_deg=0, sim_time_s=0)
    forward = machine.update(Pose2(0, 0, 0), 0.3)
    assert forward.command.left_wheel_rate_radps == pytest.approx(
        forward.command.right_wheel_rate_radps
    )
    assert forward.command.left_wheel_rate_radps > 0.0


def test_motion_profile_rejects_invalid_limits():
    with pytest.raises(ValueError, match="wheel_radius_m"):
        MotionProfile(wheel_radius_m=0.0)
    with pytest.raises(ValueError, match="max_linear_velocity_mps"):
        MotionProfile(max_linear_velocity_mps=math.nan)
    with pytest.raises(ValueError, match="yaw_tolerance_rad"):
        MotionProfile(yaw_tolerance_rad=math.pi)
    with pytest.raises(ValueError, match="smoke linear velocity"):
        MotionProfile(smoke_linear_velocity_mps=2.1)
