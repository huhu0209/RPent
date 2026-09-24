from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from robots.lynsense.simulation.rpent_plan import (
    PLAN_VERSION,
    TASK_ID,
    canonical_single_box_plan,
)
from lynsense_webots_sim.scripts import run_smoke


def test_box_phase_uses_dedicated_world_config_tree_and_events() -> None:
    assert run_smoke.CONFIG_PATHS["box"].name == "box.yaml"
    assert run_smoke.WORLD_PATHS["box"].name == "lynsense_box.wbt"
    assert run_smoke.TREE_NAMES["box"] == "lynsense_single_box_tree.xml"
    assert run_smoke.EVENT_NAMES["box"] == "events-box.jsonl"
    assert run_smoke.PHASE_SEQUENCE["all"] == (
        "interface", "smoke", "blocked", "match"
    )
    assert run_smoke.PHASE_SEQUENCE["box-all"] == (
        *run_smoke.PHASE_SEQUENCE["all"],
        "box",
    )


def test_box_phase_selects_match_navigation_goals(monkeypatch, tmp_path) -> None:
    monkeypatch.delenv("LYNSENSE_SIM_NAV_CONFIG", raising=False)
    event_path = tmp_path / "events-box.jsonl"
    environment = run_smoke._command_environment("box", event_path, "run-id")
    assert environment["LYNSENSE_SIM_NAV_CONFIG"] == str(
        run_smoke.CONFIG_PATHS["box_navigation"]
    )
    assert environment["LYNSENSE_SIM_CONFIG"] == str(run_smoke.CONFIG_PATHS["box"])


def test_phase_group_runner_preserves_original_and_extended_sequences(
    monkeypatch,
) -> None:
    executed: list[str] = []
    summaries: list[list[dict[str, object]]] = []
    monkeypatch.setattr(
        run_smoke,
        "_run_phase",
        lambda phase: executed.append(phase) or {"outcome": "passed"},
    )
    monkeypatch.setattr(
        run_smoke,
        "_write_summary",
        lambda results: summaries.append(results),
    )

    assert run_smoke._run_all(run_smoke.PHASE_SEQUENCE["all"]) == 0
    assert executed == ["interface", "smoke", "blocked", "match"]
    assert run_smoke._run_all(run_smoke.PHASE_SEQUENCE["box-all"]) == 0
    assert executed == [
        "interface",
        "smoke",
        "blocked",
        "match",
        "interface",
        "smoke",
        "blocked",
        "match",
        "box",
    ]
    assert len(summaries) == 2


def test_box_navigation_uses_carry_aligned_place_station() -> None:
    import yaml

    path = (
        Path("robots")
        / "lynsense"
        / "simulation"
        / "ros"
        / "lynsense_webots_sim"
        / "config"
        / "box_navigation.yaml"
    )
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    station = config["goals"]["放箱子1_1"]
    assert station == {
        "x": 2.4606561495236394,
        "y": -2.742422444761675,
        "yaw_deg": 0.35089784218538095,
    }
    assert config["motion"]["position_tolerance_m"] == 0.01
    assert config["motion"]["yaw_tolerance_rad"] == 0.01


def test_box_expected_script_is_exact_fifteen_serial_actions() -> None:
    actions = [entry[0] for entry in run_smoke.BOX_EXPECTED_SCRIPT]
    assert actions == [
        "nav_to_pose",
        "move_waist",
        "set_gripper",
        "move_named_config",
        "box_phase",
        "move_distance",
        "move_distance",
        "move_named_config",
        "nav_to_pose",
        "box_phase",
        "move_distance",
        "move_named_config",
        "move_waist",
        "set_gripper",
        "move_named_config",
    ]
    assert run_smoke.BOX_EXPECTED_SCRIPT[4] == (
        "box_phase",
        {"action": "pick", "flow": "flow", "config": "box1"},
    )
    assert run_smoke.BOX_EXPECTED_SCRIPT[9] == (
        "box_phase",
        {"action": "place", "flow": "flow", "config": "box1"},
    )
    assert run_smoke.PHASE_ACTION_TYPES["interface"] == run_smoke.EXPECTED_ACTION_TYPES
    assert run_smoke.PHASE_ACTION_TYPES["box"]["move_waist"] == (
        "lynsense_utils/action/MoveWaist"
    )


def _event(action: str, fields: dict[str, Any], **overrides: Any) -> dict[str, Any]:
    event: dict[str, Any] = {
        "kind": "action_result",
        "action": action,
        "pose": {"x": 1.0, "y": -2.0, "yaw": 0.0},
        "motor_rates": {
            "left_wheel_rate_radps": 0.0,
            "right_wheel_rate_radps": 0.0,
        },
        "box": {
            "position_m": (2.810559, -2.734170, 0.1085),
            "orientation_rad": (0.0, 0.0, 0.006126),
            "attached": False,
        },
        "manipulation_rates": {
            "waist_velocity_mps": 0.0,
            "arm_velocities_radps": (0.0,) * 12,
            "gripper_velocities": (0.0, 0.0),
        },
        "success": True,
        "reason": "",
    }
    translated = {
        "goal_name": fields.get("goal_name"),
        "height_mm": fields.get("height_mm"),
        "target": fields.get("target"),
        "gripper_position": fields.get("position"),
        "box_action": fields.get("action"),
        "box_flow": fields.get("flow"),
        "box_config": fields.get("config"),
        "distance_m": fields.get("distance"),
        "angle_deg": fields.get("angle"),
    }
    event.update({key: value for key, value in translated.items() if value is not None})
    event.update(overrides)
    return event


def test_box_validator_accepts_complete_terminal_evidence() -> None:
    validator = run_smoke.TreeEventValidator(
        blocked=False,
        expected_script=run_smoke.BOX_EXPECTED_SCRIPT,
        final_pose=run_smoke.BOX_FINAL_POSE,
        phase="box",
    )
    for index, (action, fields) in enumerate(run_smoke.BOX_EXPECTED_SCRIPT):
        accepted = _event(action, fields, kind="goal_accepted")
        assert validator.feed(accepted) is False
        result = _event(action, fields, sim_time_s=float(index))
        if action == "box_phase" and fields["action"] == "pick":
            result["box"]["attached"] = True
        if action == "box_phase" and fields["action"] == "place":
            result["sim_time_s"] = 9.0
        if index == len(run_smoke.BOX_EXPECTED_SCRIPT) - 1:
            result["sim_time_s"] = 9.6
            result["pose"] = {
                "x": run_smoke.BOX_FINAL_POSE[0],
                "y": run_smoke.BOX_FINAL_POSE[1],
                "yaw": run_smoke.BOX_FINAL_POSE[2],
            }
            assert validator.feed(result) is True
        else:
            assert validator.feed(result) is False


def test_box_validator_rejects_incomplete_box_evidence() -> None:
    validator = run_smoke.TreeEventValidator(
        blocked=False,
        expected_script=run_smoke.BOX_EXPECTED_SCRIPT,
        final_pose=run_smoke.BOX_FINAL_POSE,
        phase="box",
    )
    action, fields = run_smoke.BOX_EXPECTED_SCRIPT[4]
    validator.sequence_index = 4
    validator.feed(_event(action, fields, kind="goal_accepted"))
    pick = _event(action, fields, sim_time_s=4.0)
    pick["box"]["attached"] = False
    with pytest.raises(run_smoke.SmokeError, match="pick did not attach"):
        validator.feed(pick)

    validator = run_smoke.TreeEventValidator(
        blocked=False,
        expected_script=run_smoke.BOX_EXPECTED_SCRIPT,
        final_pose=run_smoke.BOX_FINAL_POSE,
        phase="box",
    )
    action, fields = run_smoke.BOX_EXPECTED_SCRIPT[9]
    validator.sequence_index = 9
    validator.feed(_event(action, fields, kind="goal_accepted"))
    place = _event(action, fields, sim_time_s=9.0)
    place["box"]["attached"] = True
    with pytest.raises(run_smoke.SmokeError, match="place did not release"):
        validator.feed(place)


def test_box_validator_requires_post_release_settle_and_zero_manipulation_rates() -> None:
    validator = run_smoke.TreeEventValidator(
        blocked=False,
        expected_script=run_smoke.BOX_EXPECTED_SCRIPT,
        final_pose=run_smoke.BOX_FINAL_POSE,
        phase="box",
    )
    action, fields = run_smoke.BOX_EXPECTED_SCRIPT[9]
    validator.sequence_index = 9
    validator.feed(_event(action, fields, kind="goal_accepted"))
    place = _event(action, fields, sim_time_s=9.0)
    assert validator.feed(place) is False

    for index in range(10, len(run_smoke.BOX_EXPECTED_SCRIPT) - 1):
        next_action, next_fields = run_smoke.BOX_EXPECTED_SCRIPT[index]
        validator.feed(_event(next_action, next_fields, kind="goal_accepted"))
        validator.feed(_event(next_action, next_fields, sim_time_s=9.2))

    terminal_action, terminal_fields = run_smoke.BOX_EXPECTED_SCRIPT[-1]
    validator.feed(_event(terminal_action, terminal_fields, kind="goal_accepted"))
    terminal = _event(
        terminal_action,
        terminal_fields,
        sim_time_s=9.4,
        pose={
            "x": run_smoke.BOX_FINAL_POSE[0],
            "y": run_smoke.BOX_FINAL_POSE[1],
            "yaw": run_smoke.BOX_FINAL_POSE[2],
        },
    )
    with pytest.raises(run_smoke.SmokeError, match="post-release"):
        validator.feed(terminal)


def test_box_tree_runner_is_selected_from_ephemeral_overlay() -> None:
    assert run_smoke.SINGLE_BOX_TREE_EXECUTABLE.name == "lynsense_single_box_tree"
    assert str(run_smoke.SINGLE_BOX_TREE_EXECUTABLE).startswith("/workspace/ws/install/")
    assert run_smoke.SINGLE_BOX_TREE_PATH.name == "lynsense_single_box_tree.xml"
    bootstrap = Path("robots/lynsense/simulation/docker/bootstrap.py").read_text(
        encoding="utf-8"
    )
    assert "box-all" in bootstrap


def test_rpent_box_phase_maps_to_box_isolation_and_own_events() -> None:
    assert run_smoke.CONFIG_PATHS["rpent-box"] == run_smoke.CONFIG_PATHS["box"]
    assert run_smoke.CONFIG_PATHS["rpent-box_navigation"].name == "box_navigation.yaml"
    assert run_smoke.WORLD_PATHS["rpent-box"] == run_smoke.WORLD_PATHS["box"]
    assert run_smoke.ROBOT_NAMES["rpent-box"] == run_smoke.ROBOT_NAMES["box"]
    assert run_smoke.EVENT_NAMES["rpent-box"] == "events-rpent-box.jsonl"
    assert run_smoke.PHASE_ACTION_TYPES["rpent-box"] == (
        run_smoke.PHASE_ACTION_TYPES["box"]
    )
    assert run_smoke.EXPECTED_SCRIPTS["rpent-box"] == run_smoke.BOX_EXPECTED_SCRIPT
    assert run_smoke.FINAL_POSES["rpent-box"] == run_smoke.BOX_FINAL_POSE
    assert run_smoke.RPENT_PLAN_RUNNER_EXECUTABLE.name == "lynsense_run_rpent_plan"


def test_rpent_box_environment_keeps_box_motion_and_injects_plan(monkeypatch, tmp_path) -> None:
    plan_path = tmp_path / "plan.json"
    monkeypatch.setenv("LYNSENSE_SIM_NAV_CONFIG", "must-be-overridden")
    environment = run_smoke._command_environment(
        "rpent-box",
        tmp_path / "events-rpent-box.jsonl",
        "run-id",
        plan_path=plan_path,
    )
    assert environment["LYNSENSE_SIM_CONFIG"] == str(run_smoke.CONFIG_PATHS["box"])
    assert environment["LYNSENSE_SIM_NAV_CONFIG"] == str(
        run_smoke.CONFIG_PATHS["box_navigation"]
    )
    assert environment["LYNSENSE_SIM_SCRIPT"] == "box"
    assert environment["WEBOTS_WORLD"] == str(run_smoke.WORLD_PATHS["box"])
    assert environment["LYNSENSE_RPENT_PLAN"] == str(plan_path)


def test_rpent_box_validator_enforces_box_attachment_evidence() -> None:
    validator = run_smoke.TreeEventValidator(
        blocked=False,
        expected_script=run_smoke.BOX_EXPECTED_SCRIPT,
        final_pose=run_smoke.BOX_FINAL_POSE,
        phase="rpent-box",
    )
    action, fields = run_smoke.BOX_EXPECTED_SCRIPT[4]
    validator.sequence_index = 4
    validator.feed(_event(action, fields, kind="goal_accepted"))
    pick = _event(action, fields, sim_time_s=4.0)
    pick["box"]["attached"] = False
    with pytest.raises(run_smoke.SmokeError, match="pick did not attach"):
        validator.feed(pick)


def test_rpent_box_rejects_invalid_plan_before_any_process_or_event_startup(
    monkeypatch, tmp_path
) -> None:
    document = {
        "task_id": TASK_ID,
        "version": PLAN_VERSION,
        "actions": [dict(action) for action in canonical_single_box_plan()],
    }
    document["actions"].pop()
    plan_path = tmp_path / "invalid-plan.json"
    plan_path.write_text(json.dumps(document), encoding="utf-8")
    started: list[str] = []
    monkeypatch.setattr(
        run_smoke,
        "_prepare_event",
        lambda phase: started.append(f"event:{phase}") or (tmp_path / "events", "run"),
    )
    monkeypatch.setattr(
        run_smoke,
        "_start_phase_processes",
        lambda *args, **kwargs: started.append("process") or pytest.fail(
            "process startup followed an invalid plan"
        ),
    )

    with pytest.raises(run_smoke.SmokeError, match="invalid Lynsense RPent plan"):
        run_smoke._run_phase("rpent-box", plan_path=plan_path)
    assert started == []


def test_main_accepts_plan_only_for_rpent_box(monkeypatch, tmp_path) -> None:
    plan_path = tmp_path / "plan.json"
    calls: list[tuple[str, Path | None]] = []
    summaries: list[list[dict[str, object]]] = []
    monkeypatch.setattr(
        run_smoke,
        "_run_phase",
        lambda phase, plan_path=None: calls.append((phase, plan_path))
        or {"outcome": "passed"},
    )
    monkeypatch.setattr(
        run_smoke,
        "_write_summary",
        lambda results: summaries.append(results),
    )

    assert run_smoke.main(["--phase", "rpent-box", "--plan", str(plan_path)]) == 0
    assert calls == [("rpent-box", plan_path)]
    with pytest.raises(SystemExit) as exit_info:
        run_smoke.main(["--phase", "box", "--plan", str(plan_path)])
    assert exit_info.value.code == 2
    assert len(summaries) == 1
