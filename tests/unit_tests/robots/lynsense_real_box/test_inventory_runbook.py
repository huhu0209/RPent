from __future__ import annotations

import json
import re
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]
RUNBOOK_PATH = (
    REPO_ROOT
    / "docs"
    / "superpowers"
    / "plans"
    / "2026-09-22-robot-one-readonly-inventory.md"
)
COMMISSIONING_RUNBOOK_PATH = (
    REPO_ROOT
    / "docs"
    / "superpowers"
    / "plans"
    / "2026-09-23-robot-one-lynrotcontrol-first-connection-runbook.md"
)


def runbook() -> str:
    return RUNBOOK_PATH.read_text(encoding="utf-8")


def runbook_prose() -> str:
    """Collapse Markdown wrapping so prose assertions stay readable."""
    return " ".join(runbook().split()).lower()


def commissioning_runbook() -> str:
    return COMMISSIONING_RUNBOOK_PATH.read_text(encoding="utf-8")


def position(text: str, pattern: str) -> int:
    match = re.search(pattern, text, flags=re.DOTALL)
    assert match is not None, pattern
    return match.start()


def test_runbook_exists_and_requires_fresh_authorization() -> None:
    text = runbook()

    assert "Do not run this inventory without new explicit user authorization." in text
    assert "Task 8 does not execute this procedure." in text


def test_commissioning_runbook_pins_ownership_gate_and_remaining_approvals() -> None:
    text = commissioning_runbook()
    prose = " ".join(text.lower().split())

    assert "lynrotcontrol.service-ownership" in text
    assert "78c45f6796cd90fdf63f51ac407ac261c04e42ab454bb6d0897749d81904c84a" in text
    assert "/tmp/lynrotcontrol-authority-1000" in text
    assert "runtime protocol `15`" in text
    assert "do not authorize live operation" in text.lower()
    assert "old dependency remains fail-closed with exit code `5`" in prose
    assert "dependency_reconciliation_unavailable" in text
    assert "no tokenless dependency cleanup" in prose
    assert "Deployment and Robot One operation require separate approvals." in " ".join(text.split())


def test_shanghai_time_is_checked_before_ssh() -> None:
    text = runbook()

    assert position(text, re.escape("TZ=Asia/Shanghai date -Is")) < position(text, r"\bssh robot1\b")


def test_read_only_allowlist_is_explicit() -> None:
    text = runbook().lower()

    for command in [
        "date -is",
        "readlink -f /home/rpp/rpp_ws",
        "stat -c",
        "source /opt/ros/humble/setup.bash",
        "source /home/rpp/rpp_ws/install/setup.bash",
        "printenv ros_domain_id ros_localhost_only",
        "timeout 10s ros2 node list",
        "timeout 10s ros2 topic list -v",
        "timeout 10s ros2 service list -v",
        "timeout 10s ros2 action list -v",
        "timeout 10s ros2 service list",
        "timeout 10s ros2 action list",
        'timeout 10s ros2 service type "$service"',
        'timeout 10s ros2 action info "$action"',
        "timeout 10s ros2 topic info -v",
        "timeout 10s ros2 topic echo --once",
        "timeout 10s ros2 interface show",
    ]:
        assert command in text


def test_mutation_and_control_commands_are_forbidden() -> None:
    text = runbook()
    forbidden = [
        "ros2 topic pub",
        "ros2 service call",
        "ros2 action send_goal",
        "ros2 run",
        "ros2 launch",
        "ros2 param set",
        "sudo",
        "chmod",
        "chown",
        "rm -",
        "mv ",
        "cp ",
        "touch ",
        "mkdir ",
        "tee ",
        "systemctl start",
        "systemctl stop",
        "systemctl restart",
    ]

    assert "## Robot One SSH forbidden actions" in text
    for command in forbidden:
        assert f"`{command.strip()}`" in text


def test_ros_domain_selection_is_inline_and_recorded() -> None:
    text = runbook()
    prose = runbook_prose()

    assert "Selected ROS domain: `3`" in text
    assert "inline `ROS_DOMAIN_ID=3` environment assignment" in text
    assert "do not modify remote shell configuration" in prose
    assert 'ROS_DOMAIN_ID=3 timeout 10s ros2 node list' in text
    assert 'ROS_DOMAIN_ID=3 timeout 10s ros2 topic list -v' in text
    assert 'ROS_DOMAIN_ID=3 timeout 10s ros2 service list' in text
    assert 'ROS_DOMAIN_ID=3 timeout 10s ros2 action list' in text


def test_required_subsystem_sections_are_present() -> None:
    text = runbook()
    sections = [
        "Chassis",
        "Localization",
        "Waist",
        "Left arm",
        "Right arm",
        "Dual-arm controller",
        "Left gripper",
        "Right gripper",
        "Box perception",
    ]

    for section in sections:
        assert f"### {section}" in text


def test_each_subsystem_has_four_explicit_topic_type_observations() -> None:
    text = runbook()
    labels = [
        "chassis state",
        "localization state",
        "waist state",
        "left-arm state",
        "right-arm state",
        "dual-arm controller feedback",
        "left-gripper feedback",
        "right-gripper feedback",
        "box perception pose",
        "box perception status",
    ]

    for label in labels:
        pattern = (
            rf"topic=<OBSERVED {re.escape(label.upper())} TOPIC>\n"
            r"ROS_DOMAIN_ID=3 timeout 10s ros2 topic info -v \"\$topic\"\n"
            r"ROS_DOMAIN_ID=3 timeout 10s ros2 topic echo --once \"\$topic\"\n"
            rf"type=<OBSERVED {re.escape(label.upper())} TYPE>\n"
            r"ROS_DOMAIN_ID=3 timeout 10s ros2 interface show \"\$type\""
        )
        assert re.search(pattern, text), label


def test_artifact_directory_and_manifest_schema_are_defined() -> None:
    text = runbook()

    assert "`inventory-artifacts/<UTC timestamp>/`" in text
    json_blocks = re.findall(r"```json\n(.*?)```", text, flags=re.DOTALL)
    assert len(json_blocks) == 1
    manifest = json.loads(json_blocks[0])

    assert manifest["manifest_version"] == 1
    assert manifest["created_at_utc"]
    assert manifest["robot"] == "robot1"
    assert manifest["runbook_sha256"]
    assert manifest["authorization_reference"]
    assert manifest["operator_notes"]
    assert manifest["sha256_algorithm"] == "sha256"
    artifacts = manifest["artifacts"]
    assert isinstance(artifacts, list)
    assert len(artifacts) == 1
    artifact = artifacts[0]
    assert set(artifact) == {
        "path",
        "sha256",
        "bytes",
        "command",
        "captured_at_utc",
    }
    assert re.fullmatch(r"[0-9a-f]{64}", artifact["sha256"])
    assert isinstance(artifact["bytes"], int)
    assert not isinstance(artifact["bytes"], bool)
    assert artifact["bytes"] >= 0
    assert artifact["command"]
    assert artifact["captured_at_utc"]


def test_workstation_artifact_preparation_is_separate_from_robot_allowlist() -> None:
    text = runbook()
    workstation = text[
        text.index("## Workstation-only artifact preparation") :
        text.index("## Robot One SSH read-only allowlist")
    ]
    block = re.search(r"```bash\n(.*?)```", workstation, flags=re.DOTALL)
    assert block is not None
    commands = block.group(1)

    expected = [
        'utc_stamp="$(date -u +%Y%m%dT%H%M%SZ)"',
        'artifact_dir="inventory-artifacts/${utc_stamp}"',
        'mkdir -p -- "$artifact_dir"',
        'find "$artifact_dir" -mindepth 1 -print -quit',
    ]
    positions = [commands.index(command) for command in expected]
    assert positions == sorted(positions)
    assert "These commands never target Robot One" in workstation
    assert "must not be used inside the Robot One SSH session" in workstation


def test_missing_controller_or_safety_review_stops_before_inventory() -> None:
    text = runbook()
    prose = runbook_prose()

    assert "STOP" in text
    assert "reviewed coordination implementation and safety evidence" in prose


def test_coordination_modes_and_perception_reference_candidates_are_defined() -> None:
    text = runbook()
    prose = runbook_prose()

    assert "atomic composite dual-arm grasp/place controller" in text
    assert "independent synchronized left/right control" in text
    assert "Do not infer coordination safety from the existence of left and right drivers alone." in text
    assert "Missing atomic coordination does not stop the read-only inventory" in text
    assert "the later live adapter plan must stop for a fresh approval" in prose
    assert "`/industrial_box/pose_base`" in text
    assert "`/industrial_box/status`" in text
    assert "`/object_approach_right/object_pose_base`" in text
    assert "`/object_approach_right/status`" in text
    assert "source-reference candidates, not proof of live presence" in text


def test_verbose_service_and_action_failure_has_bounded_fallback() -> None:
    text = runbook()

    assert "unrecognized arguments: -v" in text
    assert "run the corresponding plain list command once" in text
    assert "Do not retry in a loop" in text
    assert 'ROS_DOMAIN_ID=3 timeout 10s ros2 service type "$service"' in text
    assert 'ROS_DOMAIN_ID=3 timeout 10s ros2 action info "$action"' in text


def test_workspace_source_requires_path_ownership_mode_and_review() -> None:
    text = runbook()
    section = text[
        text.index("## Shared workspace review and graph observations") :
        text.index("## Per-subsystem topic/type observations")
    ]
    blocks = re.findall(r"```bash\n(.*?)```", section, flags=re.DOTALL)
    assert len(blocks) == 1
    commands = blocks[0]

    expected = [
        "date -Is",
        "readlink -f /home/rpp/rpp_ws",
        "stat -c '%U:%G %a %n' /opt/ros/humble/setup.bash /home/rpp/rpp_ws/install/setup.bash",
        "# Review decision: reviewed expected owner and mode; otherwise STOP.",
        "source /opt/ros/humble/setup.bash",
        "source /home/rpp/rpp_ws/install/setup.bash",
        "printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY",
        "timeout 10s ros2 node list",
        "timeout 10s ros2 topic list -v",
        "timeout 10s ros2 service list -v",
        "timeout 10s ros2 action list -v",
    ]
    positions = [commands.index(command) for command in expected]
    assert positions == sorted(positions)

    assert "readlink -f /home/rpp/rpp_ws" in text
    assert "stat -c '%U:%G %a %n' /opt/ros/humble/setup.bash /home/rpp/rpp_ws/install/setup.bash" in text
    assert "reviewed expected owner and mode" in commands
    assert "stop; do not source it" in text
    assert re.search(r"do not\s+change permissions", text, flags=re.IGNORECASE)


def test_storage_is_limited_and_absent_interfaces_are_not_guessed() -> None:
    text = runbook()
    prose = runbook_prose()

    assert "Store only command output, timestamps, selected environment flags, interface names/types, and operator notes." in text
    assert "present: false" in text
    assert "evidence" in text.lower()
    assert "do not substitute a guessed alternate interface." in prose
