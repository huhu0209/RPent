# Working on RPent

Follow [CONTRIBUTING.md](CONTRIBUTING.md) for development, testing, and pull
requests. Use the implementation and its callers to verify behavior; report
disagreements with documentation instead of assuming either is correct.

## Repository map

- `rpent/planner/`, `rpent/session/`, and `rpent/prompt/`: planner adapters,
  execution sessions, and shared prompt construction.
- `rpent/tools/` and `rpent/memory/`: tool execution, observations, and memory.
- `rpent/robots/`: robot discovery, descriptors, shared runtime, and Env/VLA
  components. `robots/<robot>/` owns each concrete integration.
- `rpent/cli/`, `rpent/dashboard/`, and `rpent/evaluation/`: entry points,
  interactive execution, and run results.
- `tests/`: offline tests and integration checks; see [tests/README.md](tests/README.md).
- `docs/source-en/` and `docs/source-zh/`: paired user and developer documentation.

## Robot One integration boundary

Robot One has three distinct RPent paths. Do not merge their evidence or
authorization boundaries:

- `robots/lynsense/` is the historical ROS read-only right-arm backend. It is
  not the current whole-robot integration.
- `robots/lynsense_real_box/` contains the offline dry-run workflow, guarded
  atomic-capability contracts, commissioning subprocess, and fail-closed live
  composition helpers. Its offline tests do not authorize motion.
- `robots/robot_one_readonly/` is the current registered whole-robot read-only
  backend. Its exact model surface is `read_robot_state`; it reads both arms,
  both grippers, waist lift, and both force sensors through an owned
  LynrotControl runtime, then releases the claim. It has no motion, gripper,
  chassis, perception, file, image, or finish capability.

`robot_one_readonly` is the only real-robot backend currently allowed to use
Dashboard through `RobotSpec.supports_dashboard`. Other real-robot backends
remain ordinary-terminal-only. Dashboard conversation access is not motion
authorization.

The Robot One host is `zxh@rpp-PC` (`192.168.77.249`), deployed under
`/home/zxh/zxh/RPent` and `/home/zxh/zxh/lynrotcontrol`. The runtime requires
`ROS_DOMAIN_ID=3` and the pinned `ea200.yaml` configuration digest. Model
credentials live in `/home/zxh/.config/rpent/lynsense.env`; never copy its API
key into source, logs, evidence, tests, or documentation.

Before any Robot One change:

1. Read `TODOS.md` for the current integration state and next gate.
2. Keep RPent as the strategy/composition layer and LynrotControl as the
   bounded robot capability layer. Do not import company pytree task flow or
   use `plan_arm_simple` as the runtime orchestrator.
3. Treat `stop_all`, Toolkit `close()`, resource cleanup, and a physical
   E-stop as different operations. None of them is a substitute for a
   confirmed software stop API required by an actuator capability.
4. CTAG grippers currently report no supported software cancel. Live gripper
   dispatch remains blocked until a reviewed stop/cancel interface exists.
5. Live motion requires a site-confirmed v2 atomic profile plus separate
   operator authorization naming the exact component, profile, and stop
   condition. Passing state reads or Dashboard conversation grants none of
   these.

For changes to `robot_one_readonly`, run at minimum:

```sh
pytest -q \
  tests/unit_tests/robots/robot_one_readonly \
  tests/unit_tests/rpent/robots/test_registry_contracts.py \
  tests/unit_tests/rpent/cli/test_main_contracts.py \
  tests/unit_tests/rpent/dashboard/test_state_contracts.py
```

Real-host checks are separate from these offline tests and must record the
host, `ROS_DOMAIN_ID`, service identity, state groups, release result, and
final claim disposition.

## Development principles

- Confirm the working branch and relevant upstream state before editing. Read
  the closest current implementation and both sides of changed interfaces.
  Preserve unrelated working-tree changes.
- Reuse existing configuration, registration, RPC, runtime, and artifact
  helpers. Prefer RLinf's supported Env/VLA implementations with thin RPent
  adapters; keep robot-specific behavior in its owning package.
- Preserve existing contracts. Base compatibility paths and new abstractions
  on actual consumers or supported data, and explain intentional behavior
  changes and migrations in the PR.
- Validate external inputs at the boundary that owns the operation. Keep
  internal control flow direct, avoid repeated checks of established state,
  and preserve useful errors. Make resource ownership and cleanup explicit.
- Keep optional simulator/model imports at their use sites so basic imports,
  robot discovery, and CLI help work without every robot extra installed.
- Follow neighboring code for naming and structure, use the project logger,
  and follow CONTRIBUTING's type annotation and docstring conventions.
- Write prose about current behavior and non-obvious constraints. Keep review
  exchanges and implementation history in PR discussions or migration notes.
  Translate meaning into natural Chinese and preserve technical identifiers.
- Keep prompts and tool descriptions focused on the model's task. Remove
  repetition without dropping required behavior or operational constraints.
- Test observable behavior and concrete regressions, reusing existing cases
  and fixtures where possible. Avoid tests that merely restate implementation
  details. Never weaken failing tests to make a change pass.

## Lynsense Isaac Sim constraints

- The company-standard runtime is Isaac Sim `2023.1.*`. Do not silently target
  Isaac Sim 4 or 5 or treat another local installation as compatible.
- Real Isaac runs must pass an explicit `python.sh` executable and the version
  contract in `robots/lynsense/isaac/`; offline/archive inspection alone is not
  runtime acceptance.
- Generated URDF, USD, logs, screenshots, and summaries belong under the
  ignored `.artifacts/lynsense-isaac/` tree. Do not commit company assets or
  probe evidence.
- Isaac 3's `SimulationApp.close()` can terminate the Python process before
  later statements run. Runtime evidence must be persisted before closing the
  application, and a child exit code alone is not sufficient proof.
- Desktop rendering requires a usable `DISPLAY` and `XAUTHORITY`. Keep real
  runtime evidence separate from unit-test results, and do not claim the
  Lynsense Isaac milestone until both the technical probe and recorded human
  visual review pass `--verify-milestone`.

## Task skills

Read the relevant skill when the task calls for it; ordinary edits do not
require loading every skill.

| Task | Skill |
| --- | --- |
| Explain and review a PR, or review an explicitly selected local diff | [review-pr](.agents/skills/review-pr/SKILL.md) |
| Check documentation, comments, translations, or user/model-facing prose | [docs-check](.agents/skills/docs-check/SKILL.md) |
| Select and run checks for a change, or prepare validation evidence | [verify-change](.agents/skills/verify-change/SKILL.md) |
| Add a robot integration or extend its Env/VLA/tool/runtime wiring | [add-robot](.agents/skills/add-robot/SKILL.md) |

Reuse repository tools and the commands documented in CONTRIBUTING and the
test guide. Report what ran, its result, and what remains unverified; keep
integration evidence distinct from benchmark task success.

Skill sources live in `.agents/skills/`. `.claude/skills` links to that whole
directory, and `CLAUDE.md` links to this file. Edit the canonical files so both
clients share the same instructions. Keep each skill self-contained in its
`SKILL.md` and link to existing repository resources.
