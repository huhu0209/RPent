# Lynsense RPent Single-Box Plan Acceptance

Date: 2026-09-22 Asia/Shanghai

Status: implementation, host regression, offline planner-path generation, and
isolated Docker `rpent-box` gate complete. Independent implementation review
found no blocker and two Important issues; both were fixed and covered by
focused regressions before the final Docker rerun.

## Scope And Limits

This accepts a plan-level RPent-to-Webots integration. The RPent API Planner
reads a deterministic simulation task description and submits one strict JSON
plan through a dedicated `lynsense_simulation` Toolkit. The plan is validated
before any Webots process starts and is then executed serially by the existing
ROS action controller.

It does not connect to robot one, use `ROS_DOMAIN_ID=3`, source a robot
workspace, mount company URDF/STL assets, access `.env.lynsense` inside the
container, or expose motion tools through the real `lynsense` backend. It does
not claim perception, localization, collision avoidance, closed-loop model
control, calibrated contact physics, real-robot readiness, or authorization to
move a physical robot.

## Architecture Evidence

- The real `robots/lynsense` backend remains unchanged as a read-only,
  real-robot backend with only `read_robot_state`.
- The new `robots/lynsense_simulation` backend is simulation-only,
  `is_real_robot=False`, and exposes exactly:
  `read_simulation_task`, `submit_simulation_plan`, and `finish`.
- Host tests exercise the actual `ApiAgentLoop` and actual RPent CLI lifecycle
  with `FunctionModel`; no external model request is made.
- The accepted plan is written atomically to
  `.artifacts/lynsense-webots/rpent-plan-run/plan.json`.
- The Toolkit session accepts only one successful plan submission.
  `finish(status="success")` is rejected until that atomic submission has
  completed.
- Docker execution remains Domain 42, localhost-only DDS, `network_mode: none`,
  read-only source mounts, and a writable artifacts mount only.
- `rpent-box` uses the same box world, controller, action graph, manipulation
  preconditions, event validator, final-pose check, and 180-second budget as
  the deterministic `box` gate. It writes separate `events-rpent-box.jsonl`
  and process logs. The original `all` and `box-all` phase groups remain
  unchanged.

## Offline Planner Path

A local `FunctionModel` run through the actual `ApiAgentLoop` and
`LynsenseSimulationToolkit` completed:

- model-visible tool table: exactly three tools;
- tool calls: `read_simulation_task -> submit_simulation_plan -> finish`;
- total tool calls: 3;
- accepted actions: 15;
- output: `.artifacts/lynsense-webots/rpent-plan-run/plan.json`.

This offline run and the offline CLI regression prove the RPent planning path,
not real-model quality or autonomous task understanding.

## Host Regression

Command:

```bash
.venv/bin/pytest -q \
  tests/unit_tests/robots/lynsense/simulation \
  tests/unit_tests/robots/lynsense_simulation \
  tests/unit_tests/rpent/robots/test_registry_contracts.py
```

Result: 423 tests passed in 23.85 seconds after review fixes. Python
compilation, `git diff --check`, and a trailing-whitespace scan of the new
simulation and test files also exited zero.

## Independent Review

An independent read-only review confirmed the real-backend isolation, strict
15-action contract, Docker isolation, and passing run evidence. It reported two
Important findings, both fixed:

- successful plan submission is now session-enforced and required for a
  successful finish;
- runner interruption or an action exception now waits for a pending goal
  request, requests cancellation, and waits for cancellation within the bounded
  shutdown budget before destroying the action client and node.

The reviewer's pre-fix broad host suite passed 529 tests. Its review was
read-only and did not run Docker/Webots.

## Docker `rpent-box`

Command, run from `robots/lynsense/simulation/docker`:

```bash
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 docker compose run --rm \
  lynsense-webots-smoke \
  --phase rpent-box \
  --plan /workspace/artifacts/rpent-plan-run/plan.json
```

Unified evidence: `.artifacts/lynsense-webots/summary.json`.

The final post-fix passing run ID was
`20260922T011327Z-7aaf2df0-4802-4f35-84cc-9cb46138c571`. Build evidence
confirmed the ephemeral `lynsense_run_rpent_plan` entry point and Domain 42 /
localhost-only environment.

`events-rpent-box.jsonl` records exactly 15 successful action results. Pick
ended attached and place ended detached. The final box position error was
0.007954583238962244 m, absolute yaw error was 0.009527404245537531 rad, and
post-release evidence spanned 7.072 seconds. The terminal reason was empty and
wheel, waist, arm, and gripper rates were all zero.
