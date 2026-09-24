# Lynsense RPent Single-Box Plan Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the real RPent API Planner produce a validated 15-action plan and execute it through the existing isolated Lynsense Webots gate.

**Architecture:** Add a shared simulation plan contract, a separate simulation-only RPent robot backend, and a ROS plan runner. Keep the real `lynsense` backend read-only and preserve the deterministic `box` and `box-all` gates.

**Tech Stack:** Python 3.10, RPent API Planner/Toolkit, pydantic-ai FunctionModel tests, ROS 2 Humble actions, Webots, pytest, Docker Compose.

**Spec:** `docs/superpowers/specs/2026-09-22-lynsense-rpent-single-box-design.md`

## Global Constraints

- Do not modify the real `robots/lynsense` tool surface or safety gates.
- Do not connect to robot one or use `ROS_DOMAIN_ID=3`.
- Keep simulation execution at `ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=1`, and container `network_mode: none`.
- Do not mount or copy `.env.lynsense`, SSH state, home directories, robot workspaces, company URDF, or STL assets.
- Preserve `all` as `interface -> smoke -> blocked -> match`; preserve `box-all` as that sequence plus `box`.
- Reject every malformed plan before starting Webots.
- Use only existing action primitives; add no new motion primitive.
- No commit, push, PR, or real-robot operation unless separately authorized.

---

### Task 1: Shared Plan Contract

**Files:**

- Create: `robots/lynsense/simulation/__init__.py`
- Create: `robots/lynsense/simulation/rpent_plan.py`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_rpent_plan.py`

**Interfaces:**

- Produces `TASK_ID`, `PLAN_VERSION`, `PlanError`, `task_description()`,
  `canonical_single_box_plan()`, `validate_plan(source)`, and
  `load_plan(path)` for Tasks 2 and 3.

- [x] Write failing tests for canonical shape, exact task metadata, all invalid top-level/action forms, JSON loading, and canonical task description.
- [x] Implement the contract without importing ROS, `rclpy`, `py_trees`, RPent, or Webots.
- [x] Run `.venv/bin/pytest -q tests/unit_tests/robots/lynsense/simulation/test_rpent_plan.py`.

### Task 2: Simulation-Only RPent Backend

**Files:**

- Create: `robots/lynsense_simulation/__init__.py`
- Create: `robots/lynsense_simulation/robot_spec.py`
- Create: `robots/lynsense_simulation/toolkit.py`
- Modify: `tests/unit_tests/rpent/robots/test_registry_contracts.py`
- Test: `tests/unit_tests/robots/lynsense_simulation/test_extension.py`

**Interfaces:**

- Consumes the Task 1 plan contract.
- Produces `get_robot_spec()` and `get_toolkit(...)` for the native RPent CLI.
- Produces a JSON plan at `RunConfig.prompt_vars["plan_path"]`.

- [x] Add contract tests for lazy registration, simulation-only flags, planner/memory/mode rejection, exact tool table, schema strictness, invalid-plan rejection, atomic accepted-plan output, actual `ApiAgentLoop` execution, and real CLI execution with `FunctionModel`.
- [x] Implement the backend with no ROS import and no network access.
- [x] Update the registry contract's expected robot and prompt-variable tables.
- [x] Run focused backend and registry tests.

### Task 3: ROS Plan Runner And `rpent-box` Gate

**Files:**

- Create: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/run_rpent_plan.py`
- Modify: `robots/lynsense/simulation/ros/lynsense_webots_sim/setup.py`
- Modify: `robots/lynsense/simulation/ros/lynsense_webots_sim/lynsense_webots_sim/scripts/run_smoke.py`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_rpent_plan_runner.py`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_box_orchestrator.py`

**Interfaces:**

- Consumes the Task 1 plan file and existing `SimulationActionBehaviour`.
- Produces executable `lynsense_run_rpent_plan`.
- Produces `run_smoke --phase rpent-box --plan PATH`.

- [x] Add host tests proving `rpent-box` maps to the box world/controller configuration, uses its own event file, preserves the box validator, and rejects a missing or invalid plan before process startup.
- [x] Implement a serial ROS action runner with ordered startup, cancellation-safe shutdown, and nonzero exit on any rejected/failed action.
- [x] Extend `run_smoke` and the package entry point while leaving `box`, `all`, and `box-all` behavior unchanged.
- [x] Run focused runner/orchestrator tests and the full simulation host suite.

### Task 4: Container Gate And Documentation

**Files:**

- Modify: `robots/lynsense/simulation/docker/bootstrap.py`
- Modify: `robots/lynsense/simulation/README.md`
- Test: `tests/unit_tests/robots/lynsense/simulation/test_static_assets.py`

**Interfaces:**

- Consumes Task 3's CLI.
- Produces documented host RPent plan-generation and container execution commands.

- [x] Add static tests for phase delegation and documentation boundaries.
- [x] Document the two-step workflow, artifact path, offline test meaning, and no-perception/no-real-robot limits.
- [x] Generate an offline FunctionModel plan and run Docker `rpent-box`.
- [x] Run Python compilation, diff checks, host suites, and inspect the generated summary/events before reporting completion.
