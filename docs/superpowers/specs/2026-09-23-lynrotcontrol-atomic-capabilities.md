# LynrotControl-based RPent Atomic Capability Layer

Date: 2026-09-23

Status: user-direction confirmed. The pure-Python tool/mode contract, strict
`AtomicCapabilityProfile`, transport-independent execution coordinator, atomic
Toolkit, and offline API Planner proof are implemented. Tests use a fake
LynrotControl-shaped runtime and scripted `FunctionModel`; they do not connect
to Robot One. The profile contains exact paired joint paths, joint limits,
gripper targets, waist targets, chassis profiles, force gates, and budgets. The
coordinator owns paired submissions, both-side arrival checks, force feedback
gates, failure interlocks, and reviewed chassis stop dispatch. The Toolkit seals
every accepted or rejected model call to atomic-profile evidence. It does not
import or instantiate `lynrotcontrol`; live motion remains unimplemented and
separately gated.

## Goal

RPent owns strategy and task composition. Robot One exposes a small set of
guarded atomic capabilities through `lynrotcontrol` and the already reviewed
ROS interfaces. RPent must not run the company pytree task flow and must not
treat `plan_arm_simple` as a required task-flow or grasp/place orchestrator.

The model chooses capabilities such as perception, chassis, waist, paired
dual-arm motion, paired gripper motion, and stop. It never chooses a raw
single-arm joint target, a free Cartesian target, a free `cmd_vel`, a speed, an
acceleration, or a payload.

## Evidence

The local `lynrotcontrol` V0.0.3 public API provides independent left/right arm
objects, paired grippers, waist lift/pitch, and left/right force sources:

- arm state, errors, joints, TCP, limits, speeds, and payload;
- bounded `movejoint` and `moveline` tasks with feedback and cancellation;
- caller-paced joint/Cartesian streaming with range and tracking checks;
- `cancel()` and `recover()`;
- dual-arm composition example that submits both sides, monitors force, and
  cancels both on any error or threshold trigger;
- paired gripper move/read with feedback-based completion;
- waist move/read/cancel/recover with position tolerances;
- six-axis force readback.

Its documentation explicitly states that left and right arms are independent and
that synchronization and fault interlock belong to the upper layer. It does not
provide chassis motion. The existing Robot One graph supplies chassis feedback
and a reviewed chassis control interface must therefore remain a separate
ROS-backed atomic capability.

## Model-Facing Capability Table

The target table is:

- `read_capability_state`
- `detect_box`
- `move_chassis(profile_name)`
- `move_waist(profile_name)`
- `move_dual_arms(profile_name)`
- `set_dual_grippers(command)`
- `stop_all`
- `finish(status)`

No `pick_box` or `place_box` tool is exposed. Grasping and placement are RPent
strategies composed from atomic capabilities plus state evidence, not opaque
task-flow methods.

`atomic_toolkit.py` implements this exact surface without common file/image
tools. Its offline constructor rejects `mode="live"`. A real `ApiAgentLoop` test
drives the fake runtime through perception, chassis, waist, paired arms, paired
grippers, and finish, while recording each call in JSONL evidence. The test also
verifies that the planner-visible list contains no `read_image`, `pick_box`,
`place_box`, or single-arm tool.

Every motion argument is a site-reviewed profile name:

- chassis profile: named navigation goal or bounded displacement/turn;
- waist profile: exact lift target and bounds;
- dual-arm profile: exact paired left/right joint paths, tolerance, and budget;
- gripper command: exact open/close target derived from site calibration.

The model cannot invent numeric values. The profile catalog is local
configuration and its identifier/hash is recorded in every evidence event.

## Capability Safety Modes

The replacement for a task-flow state machine is a resource-safety mode, not a
task sequence:

- `idle`
- `perception_running`
- `chassis_moving`
- `waist_moving`
- `dual_arms_moving`
- `grippers_moving`
- `stopping`
- `operator_review`
- `operator_review_acknowledged`
- `closed`

Read tools are always available except after close. `stop_all` is available in
every non-closed mode. Motion capabilities are denied while another motion
capability is active, robot state is stale/unhealthy, an arm has an error, the
waist or chassis state is unknown, an operator review is open, or a required
precondition profile is not current.

The mode tracks resource safety, not business progress. It does not encode
\"pick then place\"; RPent decides that order.

`operator_review` is not cleared by reconnecting or by the model. The non-model
path is `acknowledge_operator_review(operator)`, which moves the adapter to
`operator_review_acknowledged`. Only that acknowledged mode may finish with
`failure`; `resume_from_acknowledged_review()` performs a fresh preflight before
returning to `idle`. A model-facing rejected `finish` result does not authorize
`ApiAgentLoop` to end the run.

## LynrotControl Mapping

- `read_capability_state`
  - read both arms, both grippers, waist, force sources, and chassis feedback;
  - normalize timestamps and validity; a missing subsystem is explicit.
- `move_dual_arms(profile_name)`
  - resolve one reviewed paired joint path;
  - precheck both arms, both force sources, and configured limits;
  - require fresh current joints to match `path[0]` within
    `start_tolerance_deg` before either submission;
  - submit left and right through their public arm APIs;
  - collect both submission results; if either rejects, request both cancels;
  - monitor both tasks and both force sources while either is active;
  - succeed only when both report final arrival within tolerance and budget;
  - route any timeout, failure, disagreement, or stop uncertainty to `stopping`.
- `set_dual_grippers(command)`
  - submit both reviewed targets;
  - await fresh feedback from both;
  - succeed only when both positions are inside tolerance;
  - preserve the reviewed held-box policy when one side fails because current
    gripper adapters expose no hardware cancel.
- `move_waist(profile_name)`
  - use the reviewed waist axis API, bounded wait, and final feedback check.
- `stop_all`
  - request both arm cancellations and waist cancellation in parallel;
  - use the reviewed chassis stop interface;
  - do not assume gripper stop is supported;
  - share one monotonic stop budget for requests and stability confirmation;
  - require recognized requests plus fresh arm, waist, and chassis stability, or
    route to operator review with explicit request/stability sub-results.

The first implementation uses paired bounded `movejoint` profiles. Caller-paced
dual streaming is a later capability only after the bounded-path coordinator and
its fake-ROS failure tests pass.

## Configuration Additions

The current Phase 1 profile names profiles but does not contain executable
paired joint paths or chassis motion targets. Before any live adapter:

1. add an atomic-capability profile schema;
2. include exact paired left/right joint paths and per-side axes;
3. include gripper open/close values and tolerances;
4. include waist targets and bounds;
5. include named chassis goals or bounded moves;
6. include budgets, force gates, tracking tolerances, and stop budgets;
7. include the reviewed chassis control interface outside `lynrotcontrol`;
8. hash the expanded profile into all evidence.

An empty or placeholder numeric profile must reject live execution.

The follow-up live worksheet and transport composition plan is
`2026-09-23-robot-one-atomic-live-profile-and-transport.md`. It records the
confirmed perception evidence, blocks unmeasured motion values, and requires a
separately reviewed bounded chassis bridge rather than exposing `/cmd_vel`.

## Implementation Gates

1. Pure Python capability contract and profile validation; no ROS import.
2. Complete: fake `lynrotcontrol` robot tests cover state normalization, exact
   paired paths, one-side submission failure, task failure/timeout, force
   threshold and invalid/stale force, joint disagreement, gripper disagreement,
   waist timeout/tolerance, chassis dispatch, and separate chassis stop.
3. Complete: `atomic_toolkit.py` exposes only the exact table above, validates
   schema/profile arguments, records evidence, follows adapter capability modes,
   and closes owned perception/adapter/evidence resources.
4. Complete: a real `ApiAgentLoop` with a scripted `FunctionModel` composes the
   capabilities stepwise and finishes only after the fake runtime is idle.
5. Dry-run evidence showing normalized paired requests and zero live dispatch.
6. Independent review of the dual-arm coordinator and stop semantics.
7. Separate live low-speed authorization and supervised component gates.

No implementation step in this document connects to Robot One or sends motion.

Implementation note: `atomic_contract.py` implements the exact model-facing
tool table and resource-safety rejection rules. `atomic_profile.py` implements
and hashes the strict numeric capability profile. `atomic_adapter.py` consumes
only an injected LynrotControl-shaped object and a separately injected chassis
runtime; it does not import or construct either transport. Test values are
offline fixtures only; they are not Robot One site values and must not be copied
into a live profile. The adapter's live dispatch flag is disabled by default.
