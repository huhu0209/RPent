# Lynsense RPent Single-Box Plan Design

## Goal

Connect the RPent API Planner to the accepted Lynsense single-box Webots task
without weakening the existing real-robot read-only backend.

The first integration is plan-level, not perception or low-level control. A
planner observes a deterministic task description, submits a validated 15-action
plan, and the existing isolated Webots controller executes that plan serially.
The planner does not receive camera images, contact feedback, localization, or
competition perception.

## Boundaries

- Add a separate `lynsense_simulation` robot backend; do not add motion tools to
  `robots.lynsense`.
- The simulation toolkit exposes only `read_simulation_task`,
  `submit_simulation_plan`, and `finish`.
- Plan files are JSON and contain exactly the accepted single-box task/action
  contract; malformed, extra-field, out-of-order, duplicate, or partial plans are
  rejected before any Webots process starts.
- Execution remains isolated at ROS Domain 42 with localhost-only DDS and no
  container network.
- `lynsense_simulation` is not a real robot and must never use ROS Domain 3,
  robot workspace paths, SSH configuration, company URDF/STL assets, or
  `.env.lynsense`.
- The deterministic `box` and `box-all` gates remain unchanged.
- The new gate is named `rpent-box` and reuses the same Webots world, controller,
  event validator, final-pose checks, manipulation preconditions, and 180-second
  budget as `box`.

## Interfaces

The shared plan module lives at
`robots/lynsense/simulation/rpent_plan.py` so both the host RPent backend and
container plan runner can use the same contract:

- `TASK_ID = "lynsense_single_box_v1"`
- `PLAN_VERSION = 1`
- `task_description() -> dict[str, Any]`
- `canonical_single_box_plan() -> tuple[dict[str, Any], ...]`
- `validate_plan(source: Any) -> tuple[dict[str, Any], ...]`
- `load_plan(path: Path) -> tuple[dict[str, Any], ...]`
- `PlanError(ValueError)`

The top-level document has exactly `task_id`, `version`, and `actions`. Each
action has exactly the fields required for one existing ROS action goal. The
canonical sequence is the already accepted sequence from
`run_smoke.BOX_EXPECTED_SCRIPT`.

The host backend writes one accepted plan atomically to the configured output
path and permits a successful finish only after that submission. The container
runner reads the path from `LYNSENSE_RPENT_PLAN` and sends each action to the
existing `/lynsense/*` action servers in order. Interruption waits for a pending
goal request and bounded cancellation before teardown.
