import math
import threading

import pytest

from lynsense_webots_sim.action_runtime import (
    ActionRuntime,
    ExpectedAction,
    FaultSignal,
    Outcome,
)
from lynsense_webots_sim.geometry import Pose2
from lynsense_webots_sim.state_machine import MotionProfile


GOALS = {
    "sim_goal1": Pose2(0.0, 0.0, 0.0),
    "sim_goal2": Pose2(1.0, 0.0, 0.0),
}


def test_unknown_and_invalid_goals_are_rejected_without_state_change():
    runtime = ActionRuntime(MotionProfile(), {"home": Pose2(0, 0, 0)})
    assert runtime.submit_nav("missing").accepted is False
    assert runtime.submit_move(float("nan"), 0).accepted is False
    assert runtime.submit_move(1.0, float("inf")).accepted is False
    assert runtime.submit_nav("").accepted is False
    assert runtime.submit_nav("x" * 129).accepted is False
    assert runtime.active is False


def test_terminal_result_property_reports_only_final_runtime_state():
    runtime = ActionRuntime(MotionProfile(), GOALS)
    assert runtime.terminal_result is None

    assert runtime.submit_move(1.0, 0.0).accepted is True
    assert runtime.observe(Pose2(0.0, 0.0, 0.0), 0.1).outcome is Outcome.RUNNING
    assert runtime.terminal_result is None

    failed = runtime.observe(Pose2(0.0, 0.0, 0.0), 1.1)
    terminal_result = runtime.terminal_result
    assert terminal_result is not None
    assert terminal_result is failed.terminal_result
    assert terminal_result.success is False
    assert terminal_result.outcome is Outcome.FAILED


def test_nonfinal_scripted_success_has_no_terminal_result_until_final_action():
    script = [
        ExpectedAction.move(0.0, 360.0),
        ExpectedAction.move(0.0, 0.0),
    ]
    runtime = ActionRuntime(MotionProfile(), GOALS, allowed_actions=script)
    assert runtime.submit_move(0.0, 360.0).accepted is True
    assert runtime.observe(Pose2(0.0, 0.0, 0.0), 0.1).outcome is Outcome.SUCCEEDED
    assert runtime.terminal_result is None

    assert runtime.submit_move(0.0, 0.0).accepted is True
    final = runtime.observe(Pose2(0.0, 0.0, 0.0), 0.2)
    terminal_result = runtime.terminal_result
    assert terminal_result is final.terminal_result
    assert terminal_result.success is True
    assert terminal_result.outcome is Outcome.SUCCEEDED


def test_second_goal_is_rejected_and_cancel_stops():
    runtime = ActionRuntime(MotionProfile(), {"home": Pose2(0, 0, 0)})
    assert runtime.submit_move(1, 360).accepted is True
    assert runtime.submit_nav("home").accepted is False
    assert runtime.request_cancel() is True
    observation = runtime.observe(Pose2(0, 0, 0), 0.1)
    assert observation.outcome is Outcome.CANCELED
    assert observation.command.left_wheel_rate_radps == 0
    assert observation.command.right_wheel_rate_radps == 0


def test_goal_name_rejects_control_characters_and_invalid_utf8_bytes():
    runtime = ActionRuntime(MotionProfile(), GOALS)
    assert runtime.submit_nav("bad\x07name").accepted is False
    assert runtime.submit_nav("bad\rname").accepted is False
    surrogate = chr(0xD800)
    assert runtime.submit_nav(surrogate).accepted is False
    assert runtime.active is False


def test_move_and_goal_boundaries_are_accepted():
    negative = ActionRuntime(MotionProfile(), {"x" * 128: Pose2(0, 0, 0)})
    assert negative.submit_move(-10.0, -360.0).accepted is True
    negative.request_cancel()
    negative.observe(Pose2(0, 0, 0), 0.0)

    positive = ActionRuntime(MotionProfile(), {"x" * 128: Pose2(0, 0, 0)})
    assert positive.submit_move(10.0, 360.0).accepted is True
    positive.request_cancel()
    positive.observe(Pose2(0, 0, 0), 0.1)

    goal = ActionRuntime(MotionProfile(), {"x" * 128: Pose2(0, 0, 0)})
    assert goal.submit_nav("x" * 128).accepted is True


def test_runtime_uses_one_machine_and_preserves_terminal_failure():
    runtime = ActionRuntime(MotionProfile(), GOALS)
    assert runtime.request_cancel() is False
    assert runtime.submit_move(1.0, 0.0).accepted is True
    first = runtime.observe(Pose2(0, 0, 0), 0.02)
    assert first.outcome is Outcome.RUNNING

    failed = runtime.observe(Pose2(0, 0, 0), 1.1)
    assert failed.outcome is Outcome.FAILED
    assert failed.terminal is True
    assert failed.terminal_result is not None
    assert failed.terminal_result.success is False
    assert failed.feedback.distance_remaining == pytest.approx(1.0)
    assert failed.command.left_wheel_rate_radps == 0.0
    assert failed.command.right_wheel_rate_radps == 0.0

    repeat = runtime.observe(Pose2(1.0, 0.0, 0.0), 2.0)
    assert repeat.outcome is Outcome.FAILED
    assert repeat.command.left_wheel_rate_radps == 0.0
    assert repeat.command.right_wheel_rate_radps == 0.0
    assert runtime.submit_move(1.0, 0.0).reason == "terminal_lockout"
    assert runtime.submit_nav("sim_goal1").reason == "terminal_lockout"


def test_cancel_preserves_last_remaining_distances():
    runtime = ActionRuntime(MotionProfile(), GOALS)
    assert runtime.submit_move(1.0, 0.0).accepted is True
    assert runtime.observe(Pose2(0, 0, 0), 0.032).outcome is Outcome.RUNNING

    assert runtime.request_cancel() is True
    canceled = runtime.observe(Pose2(0, 0, 0), 0.1)

    assert canceled.outcome is Outcome.CANCELED
    assert canceled.feedback.distance_remaining == pytest.approx(1.0)
    assert canceled.command.left_wheel_rate_radps == 0.0
    assert canceled.command.right_wheel_rate_radps == 0.0


def test_unconstrained_success_locks_terminal_state():
    runtime = ActionRuntime(MotionProfile(), {"home": Pose2(0.05, 0.0, 0.0)})
    assert runtime.submit_nav("home").accepted is True
    succeeded = runtime.observe(Pose2(0.05, 0.0, 0.0), 0.1)
    assert succeeded.outcome is Outcome.SUCCEEDED
    assert succeeded.terminal_result.success is True
    assert succeeded.command.left_wheel_rate_radps == 0.0
    assert succeeded.command.right_wheel_rate_radps == 0.0
    assert runtime.observe(Pose2(0.05, 0.0, 0.0), 0.2).outcome is Outcome.SUCCEEDED
    assert runtime.submit_nav("home").reason == "terminal_lockout"
    assert runtime.request_cancel() is False


def test_script_accepts_only_strict_next_action_then_locks():
    script = [
        ExpectedAction.nav("sim_goal1"),
        ExpectedAction.move(-1.5, 0.0),
    ]
    runtime = ActionRuntime(MotionProfile(), GOALS, allowed_actions=script)
    assert runtime.submit_move(-1.5, 0).accepted is False
    assert runtime.submit_move(-1.5, 0).reason == "script_out_of_order"
    assert runtime.submit_nav("sim_goal1").accepted is True
    assert runtime.submit_nav("sim_goal1").accepted is False
    assert runtime.submit_nav("sim_goal1").reason == "action_active"
    assert runtime.request_cancel() is True
    assert runtime.observe(Pose2(0, 0, 0), 0.1).outcome is Outcome.CANCELED
    assert runtime.submit_move(-1.5, 0).accepted is False
    assert runtime.submit_move(-1.5, 0).reason == "terminal_lockout"


def test_scripted_final_success_locks_after_every_nonfinal_success():
    script = [
        ExpectedAction.move(0.0, 360.0),
        ExpectedAction.nav("sim_goal1"),
        ExpectedAction.move(0.0, 0.0),
    ]
    runtime = ActionRuntime(MotionProfile(), GOALS, allowed_actions=script)
    assert runtime.submit_move(0.0, 360.0).accepted is True
    first = runtime.observe(Pose2(0.0, 0.0, 0.0), 0.1)
    assert first.outcome is Outcome.SUCCEEDED

    assert runtime.submit_move(0.0, 360.0).reason == "script_out_of_order"
    assert runtime.submit_nav("sim_goal1").accepted is True
    second = runtime.observe(Pose2(0.0, 0.0, 0.0), 0.2)
    assert second.outcome is Outcome.SUCCEEDED

    assert runtime.submit_move(0.0, 360.0).reason == "script_out_of_order"
    assert runtime.submit_move(0.0, 0.0).accepted is True
    third = runtime.observe(Pose2(0.0, 0.0, 0.0), 0.3)
    assert third.outcome is Outcome.SUCCEEDED
    assert runtime.submit_move(0.0, 0.0).reason == "terminal_lockout"
    assert runtime.submit_nav("sim_goal1").reason == "terminal_lockout"


def test_scripted_first_failure_enters_terminal_lockout():
    script = [ExpectedAction.move(1.0, 0.0)]
    runtime = ActionRuntime(MotionProfile(), GOALS, allowed_actions=script)
    assert runtime.submit_move(1.0, 0.0).accepted is True
    assert runtime.observe(Pose2(0.0, 0.0, 0.0), 0.0).outcome is Outcome.RUNNING
    failed = runtime.observe(Pose2(math.inf, 0.0, 0.0), 0.1)
    assert failed.outcome is Outcome.FAILED
    assert runtime.submit_move(1.0, 0.0).reason == "terminal_lockout"


def test_scripted_mismatch_does_not_lock_before_a_terminal_result():
    script = [ExpectedAction.move(0.0, 0.0)]
    runtime = ActionRuntime(MotionProfile(), GOALS, allowed_actions=script)
    assert runtime.submit_nav("sim_goal1").reason == "script_out_of_order"
    assert runtime.submit_move(0.0, 0.0).accepted is True


def test_fault_signal_is_single_shot_and_thread_safe():
    signal = FaultSignal()
    assert signal.take() is None

    started = threading.Event()
    released = threading.Event()

    def request():
        started.wait()
        signal.request("thread_fault")
        released.set()

    thread = threading.Thread(target=request)
    thread.start()
    started.set()
    released.wait()
    assert signal.take() == "thread_fault"
    assert signal.take() is None

    signal.request("ignored")
    signal.request("later")
    assert signal.take() == "ignored"
    thread.join()


def test_runtime_rejects_invalid_observation_inputs_without_locking():
    runtime = ActionRuntime(MotionProfile(), GOALS)
    assert runtime.submit_move(1.0, 0.0).accepted is True
    with pytest.raises(ValueError):
        runtime.observe(Pose2(0.0, 0.0, 0.0), math.inf)
    assert runtime.active is True
