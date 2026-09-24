from __future__ import annotations

import json
from pathlib import Path

import pytest

from robots.lynsense.simulation.rpent_plan import (
    PLAN_VERSION,
    TASK_ID,
    PlanError,
    canonical_single_box_plan,
    load_plan,
    task_description,
    validate_plan,
)


def _document(**changes: object) -> dict[str, object]:
    document = {
        "task_id": TASK_ID,
        "version": PLAN_VERSION,
        "actions": [dict(action) for action in canonical_single_box_plan()],
    }
    document.update(changes)
    return document


def test_canonical_plan_is_the_accepted_fifteen_action_serial_task() -> None:
    actions = canonical_single_box_plan()
    assert len(actions) == 15
    assert [action["action"] for action in actions] == [
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
    assert actions[4] == {
        "action": "box_phase",
        "box_action": "pick",
        "flow": "flow",
        "config": "box1",
    }
    assert actions[9] == {
        "action": "box_phase",
        "box_action": "place",
        "flow": "flow",
        "config": "box1",
    }


def test_task_description_is_finite_and_describes_the_strict_contract() -> None:
    description = task_description()
    assert description["task_id"] == TASK_ID
    assert description["version"] == PLAN_VERSION
    assert description["execution"] == "serial"
    assert description["required_action_count"] == 15
    assert description["allowed_actions"]["nav_to_pose"]["fields"] == ("goal_name",)
    assert description["known_goal_names"] == ("搬箱子1", "放箱子1_1")
    assert description["required_action_order"] == tuple(
        action["action"] for action in canonical_single_box_plan()
    )
    assert description["known_action_values"]["box_phase"]["config"] == ("box1",)
    assert set(
        description["known_action_values"]["move_named_config"]["target"]
    ) == {
        "dualjo:joints_br",
        "dualposi_armbase_abso:pt_1f1_ready",
        "dualposi_armbase_abso:pt_up",
        "dualjo:joints_s",
    }
    json.dumps(description, allow_nan=False)


def test_validate_plan_accepts_and_normalizes_numeric_values() -> None:
    document = _document()
    document["actions"][1]["height_mm"] = 200
    assert validate_plan(document) == canonical_single_box_plan()


@pytest.mark.parametrize(
    "mutate",
    [
        lambda document: document.update(task_id="other-task"),
        lambda document: document.update(version=PLAN_VERSION + 1),
        lambda document: document.pop("actions"),
        lambda document: document.update(extra=True),
        lambda document: document.update(actions=[]),
        lambda document: document.update(actions=list(document["actions"])[:-1]),
        lambda document: document["actions"].reverse(),
        lambda document: document["actions"].__setitem__(0, {"action": "move_distance"}),
        lambda document: document["actions"][0].update(goal_name="wrong"),
        lambda document: document["actions"][0].update(extra="wrong"),
        lambda document: document["actions"][1].update(height_mm=True),
        lambda document: document["actions"][1].update(height_mm="200"),
        lambda document: document["actions"][5].update(distance=float("nan")),
        lambda document: document["actions"][3].update(target=""),
    ],
)
def test_validate_plan_rejects_metadata_shape_and_sequence_errors(mutate) -> None:
    document = _document()
    mutate(document)
    with pytest.raises(PlanError):
        validate_plan(document)


def test_validate_plan_rejects_non_object_input() -> None:
    with pytest.raises(PlanError, match="object"):
        validate_plan([dict(action) for action in canonical_single_box_plan()])


def test_load_plan_reads_json_and_rejects_invalid_files(tmp_path: Path) -> None:
    valid = tmp_path / "plan.json"
    valid.write_text(json.dumps(_document()), encoding="utf-8")
    assert load_plan(valid) == canonical_single_box_plan()

    missing = tmp_path / "missing.json"
    with pytest.raises(PlanError, match="read"):
        load_plan(missing)

    malformed = tmp_path / "malformed.json"
    malformed.write_text("{", encoding="utf-8")
    with pytest.raises(PlanError, match="JSON"):
        load_plan(malformed)
