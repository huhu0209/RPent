# Lynsense real box, Phase 1

This backend is offline dry-run only in Phase 1. It creates no ROS client and
dispatches nothing to Robot One. A site profile can represent `mode: "live"`,
but `get_robot_spec().init_runtime(...)` rejects live mode; only validated
`mode: "dry_run"` profiles can construct the offline backend.

Atomic capability profiles now support strict `version: 2` dry-run/live
validation. Version 1 remains an offline fixture schema and cannot be marked
live. Version 2 hashes the LynrotControl runtime binding, bounded chassis
limits, gripper stop requirements, perception interfaces, subsystem-specific
freshness, and carry guards. A v2 connection also requires a fresh runtime
identity receipt matching the profile hash, LynrotControl binding, execution
host, ROS domain, and chassis transport interfaces. The adapter enforces carry
guards before, during, and after bounded chassis motion, uses monotonic sample
sequences in stop/close confirmation, and requires confirmed gripper stop
behavior in the v2 failure path. These are offline contracts only; no live
transport is imported or constructed.

The Robot One inventory runbook at
`docs/superpowers/plans/2026-09-22-robot-one-readonly-inventory.md` is a future
procedure, not permission to run it. The inventory and every live component
require separate authorization that names the scope and operator. This phase did
not execute the inventory.

Subsequent separately authorized read-only checks on 2026-09-23 confirmed the
live perception interfaces `/industrial_box/pose_base`
(`geometry_msgs/msg/PoseStamped`), `/industrial_box/status`
(`std_msgs/msg/String`), and `/industrial_box/trigger`
(`std_srvs/srv/Trigger`). One authorized retry produced a fresh `base_link`
pose. `perception_client.py` captures that transport-independent sequence, but
still creates no ROS client and dispatches no request by itself.

The follow-up direction is documented in
`docs/superpowers/specs/2026-09-23-lynrotcontrol-atomic-capabilities.md`:
RPent composes guarded atomic capabilities over `lynrotcontrol`; it does not
expose arbitrary single-arm or chassis commands and does not run the company
pytree task flow. `atomic_contract.py` defines that offline tool/mode boundary;
and `atomic_profile.py` validates the reviewed numeric profiles behind those
names. `atomic_adapter.py` adds the transport-independent coordinator tested
against a fake LynrotControl-shaped runtime: paired bounded `movejoint`
submissions, both-side arrival checks, force gating, paired gripper feedback,
waist tolerance, and separate reviewed chassis dispatch/stop. None of these
modules imports or instantiates `lynrotcontrol`, ROS, or the company pytree
package, and the atomic test values are offline fixtures rather than Robot One
site values. The adapter also rejects live dispatch unless a future integration
explicitly enables that separately reviewed path.

`atomic_toolkit.py` wraps that coordinator in the exact eight-tool atomic model
surface, with no common file/image tools and no task-flow tools. Every accepted
or rejected call is sealed to JSONL evidence using the atomic profile hash. An
offline test drives the real `ApiAgentLoop` with a scripted `FunctionModel` and
a fake runtime, proving stepwise composition across perception, chassis, waist,
paired dual arms, and paired grippers. This Toolkit is not yet registered as a
live robot backend, and its offline constructor rejects `mode="live"`.

After independent review, the coordinator also requires a fresh
current-joints-to-first-waypoint match, treats unknown runtime results as
failures, binds adapter/evidence/Toolkit profile hashes, confirms software stops
from both request results and physical stability, and requires an explicit
non-model operator acknowledgement before a failure finish. `ApiAgentLoop`
continues after a rejected `finish` tool result and does not promote the model's
claimed finish arguments.

Effectful calls are also audited with a durable pre-dispatch `tool_intent`
record linked to their completion record. Arm start checks require advancing
joint feedback sequences, and connected resources cannot close until fresh
subsystem stability is confirmed. If completion evidence fails after dispatch,
the Toolkit requests a software stop and records a linked minimal terminal
failure event; a second evidence failure fails the run rather than claiming
success.

The v2 Toolkit additionally requires a perception identity receipt matching the
profile hash, pose/status/trigger interfaces, and expected frame before it
registers tools. Perception runs in the dedicated `perception_running` mode and
returns to idle even when the perception transport reports failure. This closes
the offline protocol shape, but it does not provide or authorize the live ROS
transport implementation.

The live profile worksheet and transport composition plan is documented in
`docs/superpowers/specs/2026-09-23-robot-one-atomic-live-profile-and-transport.md`.
It separates confirmed read-only evidence from missing site measurements and
keeps `/cmd_vel` behind a future reviewed bounded chassis bridge.

`stop_task` is a best-effort software stop; it is not an E-stop. A physical
operator remains responsible for the E-stop and work envelope. Webots and Isaac
values are simulation references, not validated real trajectories.

From the repository root, run the focused offline suite with the approved
project Python environment:

```sh
pytest -q tests/unit_tests/robots/lynsense_real_box \
  tests/unit_tests/rpent/robots/test_registry_contracts.py
```

The focused suite includes the offline import/network audit
`test_cli_lifecycle_completes_offline_single_box_workflow`.

## LynrotControl Commissioning

`commissioning_cli` is an operator-only entry point, separate from the planner,
Toolkit, and atomic capability surface. `validate` performs local manifest,
host-enrollment, and trusted-policy checks only. `run` is the sole effectful
command and requires a matching authorization record; it records evidence,
acquires the endpoint-scoped durable lock, and delegates one attempt to the
fixed subprocess worker. Initialization and observation have separately reviewed
time budgets. The worker request and ownership-gate contract carry
`ROS_DOMAIN_ID=3`; the future real gate must pin and verify that domain before
entering LynrotControl. No motion, recovery, calibration, trajectory,
gripper/waist command, force zeroing, or Dashboard operation is permitted.
Company pytree and `plan_arm_simple` flows are not part of this runtime.

The authorization record is bound to a trusted policy-approved self-digest and
an exact one-shot `attempt_id`; a caller-constructed matching JSON is not
effectful authorization. A durable consumed-attempt record under the trusted
lock root prevents reuse even if local evidence is moved. Acceptance remains
three separate facts: expected device identity and operator verification, all
required state groups observed with fresh distinct advancing samples, and the
reviewed owned ROS resource disposition. A timeout or uncertain worker outcome
retains the lock indefinitely;
there is no automatic retry or lock expiry. A late result does not release it:
an operator must use `reconcile` with the matching manifest, retained lock,
authorization, attempt result, endpoint scope, and service identity. Reconcile
is local-only, writes a durable reconcile report, and never initializes or
contacts the robot.

The worker now binds `build_effectful_gate` to a pre-import ownership-protocol
artifact. Protocol v1 pins
`lynrotcontrol.service-ownership`, version `1`, runtime protocol `15`,
authority root `/tmp/lynrotcontrol-authority-1008`, read-allowlist SHA-256
`38015e139b4b7ae2c0ca7f2261be2a90cb4d40ee12d35420875c6d150962ceee`, and artifact
SHA-256 `53a6ea5b7787ae2926f2ee951cd2de630c1af7b03e2bf4bca8cc622729225062`.
The approved effects are initialization/configuration, creation of the fixed
read-only ROS resources, and fixed read-only ROS parameter queries. The worker
calls dependency release only after identity, required reads, service identity,
and final resource inventory all agree; release uncertainty remains unknown and
retained.

Passing the offline suite proves no ROS, LynrotControl, or Robot One contact and
authorizes no live operation. Deployment and Robot One operation require
separate approvals. Until the reviewed dependency-side gate is deployed, an old
dependency remains fail-closed at exit `5`. Exit codes are fixed: `0` accepted,
`2` pre-contact rejection, `3` known failure, `4` unknown/retained outcome, and
`5` ownership gate unavailable.
