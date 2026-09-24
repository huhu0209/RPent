"""Direct Supervisor mechanical probe; launched through webots-controller."""

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
ARM_BR_TARGET = (
    CONFIG.named_targets["dualjo:joints_br"].left_joints_rad
    + CONFIG.named_targets["dualjo:joints_br"].right_joints_rad
)
ARM_UP_TARGET = (
    CONFIG.named_targets["dualposi_armbase_abso:pt_up"].left_joints_rad
    + CONFIG.named_targets["dualposi_armbase_abso:pt_up"].right_joints_rad
)
STAGES = (
    ("settle", 1.0, "hold", None),
    ("waist", 12.0, "waist", -0.278),
    ("arms_br", 8.0, "arms", ARM_BR_TARGET),
    ("gripper_open", 2.0, "grippers", (0.6, 0.6)),
    ("arms_up", 7.0, "arms", ARM_UP_TARGET),
    ("gripper_close", 2.0, "grippers", (0.0, 0.0)),
    ("terminal", 1.0, "hold", None),
)
ARM_NAMES = (
    *CONFIG.left_arm.joint_names,
    *CONFIG.right_arm.joint_names,
)
ARM_MOTOR_NAMES = tuple(f"{name}_motor" for name in ARM_NAMES)
ARM_SENSOR_NAMES = tuple(f"{name}_sensor" for name in ARM_NAMES)
MOTOR_NAMES = (
    CONFIG.waist.motor_name,
    *ARM_MOTOR_NAMES,
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
        raise ValueError("incomplete or out-of-order mechanical phases")
    for index, row in enumerate(samples):
        values = (
            row["time_s"],
            row["waist_position_m"],
            row["waist_velocity_mps"],
            *row["arm_positions_rad"],
            *row["arm_velocities_radps"],
            *row["gripper_positions"],
            *row["gripper_velocities"],
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("non-finite mechanical sample")
        if index and not math.isclose(
            row["time_s"] - samples[index - 1]["time_s"], 0.032, abs_tol=1e-8
        ):
            raise ValueError("invalid mechanical sampling interval")

    rows = _last_rows(samples)
    targets_reached = (
        abs(rows["waist"]["waist_position_m"] + 0.278)
        <= CONFIG.thresholds.waist_arrival_m
        and _arm_error(rows["arms_br"]["arm_positions_rad"], ARM_BR_TARGET)
        <= CONFIG.thresholds.joint_arrival_rad
        and _arm_error(rows["arms_up"]["arm_positions_rad"], ARM_UP_TARGET)
        <= CONFIG.thresholds.joint_arrival_rad
        and rows["arms_br"]["max_joint_error_rad"]
        <= CONFIG.thresholds.joint_arrival_rad
        and rows["arms_up"]["max_joint_error_rad"]
        <= CONFIG.thresholds.joint_arrival_rad
        and max(
            abs(value - target)
            for value, target in zip(
                rows["gripper_open"]["gripper_positions"], (0.6, 0.6)
            )
        )
        <= CONFIG.thresholds.gripper_arrival
        and max(
            abs(value - target)
            for value, target in zip(
                rows["gripper_close"]["gripper_positions"], (0.0, 0.0)
            )
        )
        <= CONFIG.thresholds.gripper_arrival
    )
    terminal = rows["terminal"]
    terminal_rates_zero = (
        terminal["waist_velocity_mps"] == 0.0
        and all(value == 0.0 for value in terminal["arm_velocities_radps"])
        and all(value == 0.0 for value in terminal["gripper_velocities"])
    )
    moved_without_stall = all(
        _phase_motion(samples, phase) > 0.02
        for phase in ("waist", "arms_br", "gripper_open", "arms_up", "gripper_close")
    )
    passed = targets_reached and terminal_rates_zero and moved_without_stall
    return {
        "passed": passed,
        "targets_reached": targets_reached,
        "terminal_rates_zero": terminal_rates_zero,
        "moved_without_stall": moved_without_stall,
        "stages": {
            phase: {"samples": sum(row["phase"] == phase for row in samples)}
            for phase, _, _, _ in STAGES
        },
    }


def run_controller(output: Path) -> int:
    from controller import Supervisor

    robot = Supervisor()
    motors = [robot.getDevice(name) for name in MOTOR_NAMES]
    if any(motor is None for motor in motors):
        raise RuntimeError("mechanical world is missing a required motor")
    for motor in motors:
        motor.setPosition(math.inf)
        motor.setVelocity(0.0)
    for name in (
        CONFIG.waist.position_sensor_name,
        *ARM_SENSOR_NAMES,
        CONFIG.grippers["left"].position_sensor_name,
        CONFIG.grippers["right"].position_sensor_name,
    ):
        robot.getDevice(name).enable(32)
    if robot.step(32) == -1:
        raise RuntimeError("Webots stopped before mechanical probe initialization")

    samples: list[dict[str, Any]] = []
    try:
        for phase, duration, target_kind, target in STAGES:
            for _ in range(round(duration / 0.032)):
                state = _read_state(robot)
                _write_stage(motors, state, target_kind, target)
                if robot.step(32) == -1:
                    raise RuntimeError("Webots stopped before mechanical probe completed")
                state = _read_state(robot)
                max_error = (
                    _arm_error(state["arm_positions_rad"], target)
                    if target_kind == "arms"
                    else 0.0
                )
                samples.append(
                    {
                        "phase": phase,
                        "time_s": robot.getTime(),
                        **state,
                        "max_joint_error_rad": max_error,
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


def _read_state(robot: Any) -> dict[str, Any]:
    waist = _sensor_value(robot, CONFIG.waist.position_sensor_name)
    arms = tuple(
        _sensor_value(robot, name) for name in ARM_SENSOR_NAMES
    )
    grippers = (
        _sensor_value(robot, CONFIG.grippers["left"].position_sensor_name),
        _sensor_value(robot, CONFIG.grippers["right"].position_sensor_name),
    )
    if not all(
        math.isfinite(value) for value in (waist, *arms, *grippers)
    ):
        raise RuntimeError("mechanical world produced a non-finite sensor value")
    return {
        "waist_position_m": waist,
        "waist_velocity_mps": 0.0,
        "arm_positions_rad": arms,
        "arm_velocities_radps": (0.0,) * 12,
        "gripper_positions": grippers,
        "gripper_velocities": (0.0, 0.0),
    }


def _sensor_value(robot: Any, name: str) -> float:
    sensor = robot.getDevice(name)
    sensor.enable(32)
    return float(sensor.getValue())


def _write_stage(
    motors: list[Any], state: dict[str, Any], target_kind: str, target: Any
) -> None:
    waist_motor = motors[0]
    arm_motors = motors[1:13]
    gripper_motors = motors[13:15]
    if target_kind == "hold":
        for motor in motors:
            motor.setVelocity(0.0)
        return
    if target_kind == "waist":
        error = float(target) - state["waist_position_m"]
        speed = min(
            CONFIG.waist.max_velocity_mps,
            math.sqrt(2.0 * 0.05 * abs(error)),
        )
        waist_motor.setVelocity(
            0.0
            if abs(error) <= CONFIG.thresholds.waist_arrival_m
            else math.copysign(speed, error)
        )
        for motor in (*arm_motors, *gripper_motors):
            motor.setVelocity(0.0)
        return
    if target_kind == "arms":
        for motor, current, desired, limit in zip(
            arm_motors,
            state["arm_positions_rad"],
            target,
            CONFIG.left_arm.max_velocity_radps + CONFIG.right_arm.max_velocity_radps,
        ):
            error = desired - current
            motor.setVelocity(
                0.0 if abs(error) <= CONFIG.thresholds.joint_arrival_rad else math.copysign(limit, error)
            )
        waist_motor.setVelocity(0.0)
        for motor in gripper_motors:
            motor.setVelocity(0.0)
        return
    for motor, current, desired in zip(
        gripper_motors, state["gripper_positions"], target
    ):
        error = desired - current
        motor.setVelocity(
            0.0
            if abs(error) <= CONFIG.thresholds.gripper_arrival
            else math.copysign(min(config.max_velocity for config in CONFIG.grippers.values()), error)
        )
    waist_motor.setVelocity(0.0)
    for motor in arm_motors:
        motor.setVelocity(0.0)


def _last_rows(samples: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    rows: dict[str, dict[str, Any]] = {}
    for row in samples:
        rows[row["phase"]] = row
    return rows


def _phase_motion(samples: list[dict[str, Any]], phase: str) -> float:
    rows = [row for row in samples if row["phase"] == phase]
    if phase == "waist":
        return abs(rows[-1]["waist_position_m"] - rows[0]["waist_position_m"])
    if phase in {"arms_br", "arms_up"}:
        return max(
            abs(after - before)
            for before, after in zip(
                rows[0]["arm_positions_rad"], rows[-1]["arm_positions_rad"]
            )
        )
    return max(
        abs(after - before)
        for before, after in zip(
            rows[0]["gripper_positions"], rows[-1]["gripper_positions"]
        )
    )


def _arm_error(actual: tuple[float, ...], target: tuple[float, ...]) -> float:
    return max(abs(value - desired) for value, desired in zip(actual, target))


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
