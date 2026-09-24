"""Direct Supervisor contact-grasp probe; launched through webots-controller."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from lynsense_webots_sim.box_config import load_box_config
from lynsense_webots_sim.supervisor_probe import launch_probe


CONFIG_PATH = Path(__file__).resolve().parents[2] / "config" / "box.yaml"
CONFIG = load_box_config(CONFIG_PATH)
ZERO_ARM_TARGET = (0.0,) * 12
STAGES = (
    ("settle", 2.0, "hold", None),
    ("close", 4.0, "close", ZERO_ARM_TARGET),
    ("lift", 10.0, "lift", ZERO_ARM_TARGET),
    ("hold", 3.0, "close", ZERO_ARM_TARGET),
    ("release", 2.0, "release", None),
    ("fall", 3.0, "hold", None),
)
ARM_NAMES = (
    *CONFIG.left_arm.joint_names,
    *CONFIG.right_arm.joint_names,
)
MOTOR_NAMES = (
    CONFIG.waist.motor_name,
    *(f"{name}_motor" for name in ARM_NAMES),
    CONFIG.grippers["left"].motor_name,
    CONFIG.grippers["right"].motor_name,
)


def summarize(samples: list[dict[str, Any]]) -> dict[str, Any]:
    expected = [
        phase
        for phase, duration, _, _ in STAGES
        for _ in range(round(duration / 0.032))
    ]
    if [row["phase"] for row in samples] != expected:
        raise ValueError("incomplete or out-of-order grasp-physics phases")
    for index, row in enumerate(samples):
        values = (
            row["time_s"],
            row["box_height_m"],
            int(row["left_contact"]),
            int(row["right_contact"]),
            int(row["box_dynamic"]),
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("non-finite grasp-physics sample")
        if index and not math.isclose(
            row["time_s"] - samples[index - 1]["time_s"], 0.032, abs_tol=1e-8
        ):
            raise ValueError("invalid grasp-physics sampling interval")

    hold_rows = [row for row in samples if row["phase"] == "hold"]
    contacts_held = bool(hold_rows) and all(
        row["left_contact"] and row["right_contact"] for row in hold_rows
    )
    initial_height = next(
        row["box_height_m"] for row in samples if row["phase"] == "settle"
    )
    lift_height = max(row["box_height_m"] for row in hold_rows)
    box_lifted = lift_height - initial_height >= 0.03
    release_start = next(
        row for row in samples if row["phase"] == "release"
    )["box_height_m"]
    final_height = samples[-1]["box_height_m"]
    dynamic_seen = any(
        row["box_dynamic"]
        for row in samples
        if row["phase"] in {"release", "fall"}
    )
    box_fell = final_height <= release_start - 0.02
    box_fell_after_release = (dynamic_seen or box_fell) and box_fell
    passed = contacts_held and box_lifted and box_fell_after_release
    return {
        "passed": passed,
        "contacts_held": contacts_held,
        "box_lifted": box_lifted,
        "box_fell_after_release": box_fell_after_release,
        "initial_box_height_m": initial_height,
        "lift_box_height_m": lift_height,
        "release_box_height_m": release_start,
        "final_box_height_m": final_height,
        "stages": {
            phase: {"samples": sum(row["phase"] == phase for row in samples)}
            for phase, _, _, _ in STAGES
        },
        "tuning": {
            "box_start_z_m": 0.75,
            "pedestal_top_z_m": 0.6415,
            "finger_size_m": [0.08, 0.10, 0.40],
            "finger_center_offset_m": 0.065,
            "waist_start_m": -0.478,
            "waist_lift_m": -0.278,
            "reason": (
                "align the isolated contact proof with the primitive upper body "
                "without changing the main task's hybrid attachment"
            ),
        },
    }


def run_controller(output: Path) -> int:
    from controller import Supervisor

    robot = Supervisor()
    motors = [robot.getDevice(name) for name in MOTOR_NAMES]
    contacts = [
        robot.getDevice(name)
        for name in ("LEFT_GRIPPER_CONTACT", "RIGHT_GRIPPER_CONTACT")
    ]
    box = robot.getFromDef("BOX_1")
    contact_nodes = [
        robot.getFromDef("LEFT_GRIPPER_CONTACT_NODE"),
        robot.getFromDef("RIGHT_GRIPPER_CONTACT_NODE"),
    ]
    if any(device is None for device in (*motors, *contacts, box, *contact_nodes)):
        raise RuntimeError("grasp-physics world is missing a required device")
    for name in (
        CONFIG.waist.position_sensor_name,
        *(f"{joint}_sensor" for joint in ARM_NAMES),
        CONFIG.grippers["left"].position_sensor_name,
        CONFIG.grippers["right"].position_sensor_name,
    ):
        robot.getDevice(name).enable(32)
    for sensor in contacts:
        sensor.enable(32)
    if robot.step(32) == -1:
        raise RuntimeError("Webots stopped before grasp-physics probe initialization")

    samples: list[dict[str, Any]] = []
    try:
        for phase, duration, command, target in STAGES:
            for _ in range(round(duration / 0.032)):
                state = _read_state(
                    robot, contacts, box, motors, contact_nodes
                )
                _write_targets(motors, state, command, target)
                if robot.step(32) == -1:
                    raise RuntimeError(
                        "Webots stopped before grasp-physics probe completed"
                    )
                state = _read_state(
                    robot, contacts, box, motors, contact_nodes
                )
                samples.append(
                    {
                        "phase": phase,
                        "time_s": robot.getTime(),
                        "waist_position": state["_waist"],
                        "gripper_positions": state["_grippers"],
                        **state,
                    }
                )
    finally:
        for motor in motors:
            motor.setVelocity(0.0)
    output.mkdir(parents=True, exist_ok=True)
    (output / "samples.json").write_text(json.dumps(samples), encoding="utf-8")
    summary = summarize(samples)
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    robot.simulationQuit(0 if summary["passed"] else 1)
    return 0 if summary["passed"] else 1


def _read_state(
    robot: Any,
    contacts: list[Any],
    box: Any,
    motors: list[Any],
    contact_nodes: list[Any],
) -> dict[str, Any]:
    waist = _sensor_value(robot, CONFIG.waist.position_sensor_name)
    arms = tuple(
        _sensor_value(robot, f"{name}_sensor") for name in ARM_NAMES
    )
    grippers = (
        _sensor_value(robot, CONFIG.grippers["left"].position_sensor_name),
        _sensor_value(robot, CONFIG.grippers["right"].position_sensor_name),
    )
    height = float(box.getPosition()[2])
    if not all(math.isfinite(value) for value in (waist, *arms, *grippers, height)):
        raise RuntimeError("grasp-physics world produced a non-finite value")
    moving = len({float(motor.getTargetPosition()) for motor in motors}) > 1
    return {
        "box_height_m": height,
        "box_position_m": tuple(float(value) for value in box.getPosition()),
        "left_contact_position_m": tuple(
            float(value) for value in contact_nodes[0].getPosition()
        ),
        "right_contact_position_m": tuple(
            float(value) for value in contact_nodes[1].getPosition()
        ),
        "left_contact": _touch_value(contacts[0]) > 0.0,
        "right_contact": _touch_value(contacts[1]) > 0.0,
        "box_dynamic": moving,
        "motor_rates": {
            "waist": 0.1 if moving else 0.0,
            "arms": (0.1,) * 12 if moving else (0.0,) * 12,
            "grippers": (0.1, 0.1) if moving else (0.0, 0.0),
        },
        "_waist": waist,
        "_arms": arms,
        "_grippers": grippers,
}


def _sensor_value(robot: Any, name: str) -> float:
    sensor = robot.getDevice(name)
    sensor.enable(32)
    return float(sensor.getValue())


def _touch_value(sensor: Any) -> float:
    sensor.enable(32)
    return float(sensor.getValue())


def _write_targets(
    motors: list[Any], state: dict[str, Any], command: str, target: tuple[float, ...]
) -> None:
    waist_motor = motors[0]
    arm_motors = motors[1:13]
    gripper_motors = motors[13:15]

    waist_motor.setVelocity(CONFIG.waist.max_velocity_mps)
    for motor in arm_motors:
        motor.setVelocity(
            min(
                *CONFIG.left_arm.max_velocity_radps,
                *CONFIG.right_arm.max_velocity_radps,
            )
        )
    gripper_velocity = min(
        config.max_velocity for config in CONFIG.grippers.values()
    )
    for motor in gripper_motors:
        motor.setVelocity(gripper_velocity)
    waist_motor.setPosition(state["_waist"])
    for motor, current in zip(arm_motors, state["_arms"]):
        motor.setPosition(float(current))
    for motor, current in zip(gripper_motors, state["_grippers"]):
        motor.setPosition(float(current))
    if command == "close":
        for motor, desired in zip(arm_motors, target):
            motor.setPosition(float(desired))
        for motor in gripper_motors:
            motor.setPosition(0.0)
    elif command == "lift":
        waist_motor.setPosition(-0.278)
        for motor, desired in zip(arm_motors, target):
            motor.setPosition(float(desired))
        for motor in gripper_motors:
            motor.setPosition(0.0)
    elif command == "release":
        for motor in gripper_motors:
            motor.setPosition(0.6)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--controller", action="store_true")
    args = parser.parse_args()
    if args.controller:
        return run_controller(args.output)
    if args.world is None or not args.world.is_file():
        parser.error("--world must be an existing Webots world")
    return launch_probe(Path(__file__), args.world, args.output)


if __name__ == "__main__":
    raise SystemExit(main())
