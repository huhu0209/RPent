from __future__ import annotations

import json
from pathlib import Path

from lynsense_webots_sim.scripts import run_viewer
from lynsense_webots_sim.viewer_state import ViewerRunStore


VIEWER_DIR = Path("robots/lynsense/simulation/viewer")


def make_store(tmp_path: Path) -> ViewerRunStore:
    artifact = tmp_path / "run-id"
    video = artifact / "video"
    video.mkdir(parents=True)
    event_path = artifact / "events.jsonl"
    event_path.touch()
    return ViewerRunStore(
        phase="box",
        run_id="run-id",
        event_path=event_path,
        artifact_dir=artifact,
        video_dir=video,
        video_start_mono=10.0,
    )


def append_event(path: Path, event: dict) -> None:
    path.write_text(json.dumps(event) + "\n", encoding="utf-8")


def test_box_phase_is_allowed_and_uses_navigation_map_config(
    monkeypatch,
) -> None:
    assert "box" in run_viewer.VIEWER_PHASES
    parser_choices = [
        action.choices for action in run_viewer._build_parser()._actions
        if action.dest == "phase"
    ]
    assert parser_choices == [{"smoke", "blocked", "match", "box"}]

    monkeypatch.setitem(
        run_viewer.run_smoke.CONFIG_PATHS,
        "box_navigation",
        Path(
            "robots/lynsense/simulation/ros/lynsense_webots_sim/"
            "config/box_navigation.yaml"
        ),
    )
    arena, goals = run_viewer._load_map_metadata("box")
    assert arena == {
        "length_m": 8.0,
        "width_m": 6.0,
        "boundary_x_m": 4.0,
        "boundary_y_m": 3.0,
    }
    assert goals["放箱子1_1"] == {
        "x": 2.4606561495236394,
        "y": -2.742422444761675,
        "yaw_deg": 0.35089784218538095,
    }


def test_box_events_update_normalized_manipulation_and_box_telemetry(
    tmp_path: Path,
) -> None:
    store = make_store(tmp_path)
    append_event(
        store.event_path,
        {
            "kind": "action_feedback",
            "action": "box_phase",
            "pose": {"x": 2.4, "y": -2.7, "yaw": 0.0},
            "motor_rates": {
                "left_wheel_rate_radps": 0.0,
                "right_wheel_rate_radps": 0.0,
            },
            "waist_position_m": -0.278,
            "arm_positions_rad": [0.1, -0.1] * 6,
            "gripper_positions": [0.2, 0.3],
            "max_joint_error_rad": 0.025,
            "box": {
                "attached": True,
                "position_m": [2.8, -2.73, 0.108],
                "orientation_rad": [0.0, 0.0, 0.006],
            },
        },
    )

    store.ingest_new_events(11.0)
    snapshot = store.snapshot()
    assert snapshot["manipulation"] == {
        "waist_position_mm": -278.0,
        "arm_positions_rad": [0.1, -0.1] * 6,
        "gripper_positions": [0.2, 0.3],
        "max_arm_joint_error_rad": 0.025,
    }
    assert snapshot["box"] == {
        "attached": True,
        "position_m": [2.8, -2.73, 0.108],
        "orientation_rad": [0.0, 0.0, 0.006],
    }


def test_navigation_events_leave_box_telemetry_empty(tmp_path: Path) -> None:
    store = make_store(tmp_path)
    append_event(
        store.event_path,
        {
            "kind": "action_feedback",
            "action": "nav_to_pose",
            "pose": {"x": 1.0, "y": 2.0, "yaw": 0.0},
            "motor_rates": {
                "left_wheel_rate_radps": 0.1,
                "right_wheel_rate_radps": 0.1,
            },
        },
    )

    store.ingest_new_events(11.0)
    snapshot = store.snapshot()
    assert snapshot["manipulation"] == {
        "waist_position_mm": None,
        "arm_positions_rad": None,
        "gripper_positions": None,
        "max_arm_joint_error_rad": None,
    }
    assert snapshot["box"] is None


def test_viewer_assets_expose_compact_box_telemetry() -> None:
    html = (VIEWER_DIR / "index.html").read_text(encoding="utf-8")
    for element_id in (
        "waist-position",
        "gripper-positions",
        "arm-error",
        "box-pose",
        "box-state",
    ):
        assert f'id="{element_id}"' in html

    javascript = (VIEWER_DIR / "static" / "app.js").read_text(encoding="utf-8")
    assert "run.manipulation" in javascript
    assert "run.box" in javascript
    assert "formatNumber(manipulation.waist_position_mm)" in javascript
    assert "box.attached" in javascript
    assert "fetch(" in javascript
    assert "/control" not in javascript


def test_box_viewer_uses_box_validation_contract() -> None:
    source = (
        Path(
            "robots/lynsense/simulation/ros/lynsense_webots_sim/"
            "lynsense_webots_sim/scripts/run_viewer.py"
        )
    ).read_text(encoding="utf-8")
    assert "_wait_for_ready(phase_processes, event_path, run_id, phase=phase)" in source
    assert "phase=phase" in source
    assert "180.0 if phase == \"box\"" in source
