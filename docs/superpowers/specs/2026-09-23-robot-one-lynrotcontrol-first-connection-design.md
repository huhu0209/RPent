# Robot One LynrotControl First Connection and State Acceptance

Date: 2026-09-23

Status: approved design; scope and design sections confirmed by huhu, with
independent review approved. This document authorizes neither implementation
nor a hardware connection.

## 1. Objective and Boundaries

Establish the first verifiable LynrotControl connection to Robot One using an
operator-driven terminal acceptance entry, not an LLM or Dashboard session.
The output is identity, state, initialization-effect, and resource-disposition
evidence for the later RPent atomic-capability integration.

The user believes LynrotControl has not yet been instantiated on Robot One.
Treat that as a working hypothesis, not proof that no service or other control
client exists. Replace the earlier goal of finding an already selected instance
with selecting and reviewing the configuration for first initialization.

In scope:

- Static hardware/configuration matching and initialization call-path review.
- A separately authorized, single initialization attempt with enumerated effects.
- Repeated reads of both arms, both grippers, waist lift, and both force sources.
- Identity provenance, timeout/partial-failure reporting, and resource accounting.
- Offline tests and independent review of the eventual implementation.

Not in scope:

- Trajectories, single-arm or paired motion tools, gripper motion, waist motion,
  drag, recovery, force zeroing, manual enabling, or experimental stop/cancel.
- Chassis transport, perception triggering, full box profiles, or box handling.
- Dashboard access, model calls, Toolkit registration, or relaxing existing live
  rejections. The terminal entry is a commissioning tool, not a new planner.
- A new ROS-only multi-subsystem backend or a new connect-only LynrotControl API.
- Dependency upgrades, global configuration changes, or unapproved deployment.

Initialization may itself stop a controller, change settings, and issue READY.
Those effects are NOT classified as read-only and require their own approval.
No approval of this design waives that requirement or existing motion blockers.

## 2. Evidence and Existing Behavior

Repository-relative references below describe RPent. Paths prefixed with
`/home/huhu/work/RPent_lynsense/lynrotcontrol/` are local dependency reference
source, not proof of what is installed on Robot One.

| Evidence | What it establishes | What it does not establish |
| --- | --- | --- |
| `inventory-artifacts/20260923T100606Z/manifest.json` and `run-notes.md` | Domain 3 ROS graph, single state samples, xArm candidate service types; 38 captured command outputs | Runtime selection, sustained freshness, functioning cancel, or permission to initialize |
| `robots/lynsense_real_box/atomic_toolkit.py` and `robot_spec.py` | Live operation is rejected; existing factory uses the offline adapter | A ready live commissioning entry |
| `lynrotcontrol/implementation/initialization.py` | Top-level initialization processes left then right arm and creates peripheral wrappers | Transactional initialization or full rollback on later failure |
| `lynrotcontrol/lynarmcontrol/interfaces/initialization.py` and `implementation/client.py` | Initialization can start/reuse a persistent local RPC service | A harmless read-only constructor or caller-owned service |
| `lynrotcontrol/lynarmcontrol/adapters/xarm_1_18_5/implementation/arm.py` and `recovery.py` | Arm construction invokes `initialize_configuration`; a mismatch can write controller presets | No-write initialization on the actual robot |
| `lynrotcontrol/lynarmcontrol/implementation/service.py` | SDK sessions persist across callers; shutdown may cancel active tasks | Client exit closes the robot or a service can be killed safely |

The recorded left-arm sample was `state=2`, `mode=1`, `err=0`. This is a dated
sample, not a current permission or operating-mode guarantee. In local xArm
source, `prepare_motion()` uses servo mode 1 but also enables and issues READY;
that method is prohibited in this phase. Mode 1 is neither automatically a
configuration mismatch nor sufficient evidence of motion readiness.

## 3. Chosen Approach

Reuse the existing LynrotControl initialization mechanism only after reviewing
and approving its complete effects. Do not conceal writes behind a label such
as read-only connection, and do not simply delete the RPent live rejection.

An alternative connect-only dependency mode would reduce initialization effects
but needs dependency API changes and separate testing. A ROS-only reader would
observe the graph but would not validate the intended execution dependency.
Neither is part of this design; switch only after a scope revision and approval.

```text
Reviewed code/configuration inventory + site authorization
  -> operator terminal commissioning entry
  -> static validation and resource/conflict checks
  -> one LynrotControl initialize(instance) attempt
  -> identity checks + bounded repeated state reads
  -> local evidence and explicit resource disposition
```

## 4. Module Responsibilities

Keep a small commissioning package within `robots/lynsense_real_box/`, separate
from the planner-facing Toolkit and transport-free atomic coordinator.
Implementation planning will choose file names, following existing repo style.

1. **Pure commissioning contract** validates reviewed input and classifies
   evidence. Importing it performs no networking, ROS imports, initialization,
   process creation, or configuration mutation. It does not use an invented
   `AtomicCapabilityProfile` populated with dummy motion values.
2. **Terminal commissioning runner** owns one attempt, approval validation,
   deadlines, pre-effect logging, and the result report. The default operation
   validates local documents only. A distinct explicitly authorized operation
   performs initialization and observation; it cannot be selected by a model.
3. **LynrotControl binding** is the only dependency-import boundary. It resolves
   the reviewed installation and exposes the fixed initialization/read calls to
   the runner. No arbitrary method name, code expression, or shell command comes
   from input. Offline tests inject a fake binding.

Do not change `api_loop.py`, existing live gates, or the eight-tool atomic
contract to implement this phase. Reuse serialization/logging helpers where
compatible, but do not make commissioning evidence masquerade as a full live
profile-v2 identity receipt including chassis and perception.

## 5. Reviewed Input and Authority

Before any effectful attempt, require a complete commissioning manifest with:

- Robot/site identity, operator, execution host/account, interpreter/dependency
  identity, explicit ROS domain, run identifier and expiration of authorization.
- Resolved library path and digest of executable source including the bundled
  SDK; instance configuration and every transitively loaded configuration,
  including model endpoints, brand, tool, controller, force and runtime settings.
- Expected arm sides, models, axes and endpoints, peripheral identities, units,
  force frames/transforms, and evidence for mapping each endpoint to the robot.
- Finite reviewed connection, initialization, per-read, observation and cleanup
  budgets; polling interval, per-source freshness requirements and minimum
  distinct samples (at least two per required source). No fixture-derived values.
- Exact approved startup effects, read methods, read-service requests, endpoint
  creation, artifact/runtime paths, and service disposition policy.
- Reviewer, supporting evidence references, approval bound to the manifest
  digest, and the exact attempt. Missing or inconsistent fields reject the run.

Pure validation may parse reviewed local files without calling initialize or
loading drivers. At execution, validate resolved import paths and effective
configuration on the selected host against the approved inventory before any
device contact; do not assume the workstation checkout matches the robot.
Require a controlled, unchanged installation throughout the run. A path/hash
change invalidates authorization. Do not silently install or patch dependencies.

Pin this first-connection attempt to one authorized execution host. The
manifest records a canonical stable machine identifier, reviewed hostname,
account, interpreter and dependency digests. The canonical identifier is
required; missing, unreadable, changed, or unregistered identity rejects before
acquiring the commissioning lock or contacting the device. Uniqueness comes
from a reviewed site host-enrollment record or other explicitly named trusted
authority; a local self-report cannot establish global uniqueness. If that
authority is unavailable or reports a duplicate/conflict, the attempt rejects.
Hostname is only a human-readable cross-check, never the authority. Do not rely
on independent local locks to coordinate multiple hosts. A future multi-host
design would require a shared lock authority and a new review. The selected
host must have the reviewed SDK/ROS dependencies and allowed connectivity.

The old ROS list/type/info/echo runbook does not cover initialization, RPC startup,
publisher creation, or parameter service calls. A new operational runbook must
enumerate these effects and obtain separate hardware authorization after the
implementation and its tests are reviewed. Check Asia/Shanghai time before any
future remote execution. Planning never counts as that authorization.

## 6. Effect Inventory

| Boundary | Source-observed effects | Approval rule |
| --- | --- | --- |
| RPC client/service | Runtime directory, locks, socket and log creation; service spawn; possible existing-service reuse | Exact paths and ownership reviewed; do not call startup during static validation |
| xArm construction | SDK connection, device reads, configuration comparison | Approved endpoints and installation only |
| xArm preset mismatch | `set_state(4)`, gravity/collision/TCP/payload setters, `set_state(0)` after writes; failure path may send `set_state(4)` | Review effective values and all branch effects before entry; prohibit initialization if any possible branch exceeds approval |
| CTAG gripper read | Lazy ROS context/node/subscription and command-service client creation | Endpoint construction must be listed; no gripper command service call |
| Waist01 read | Lazy context/node, command publisher, clients and subscription; GetParameters for low/high limit and mechanical zero | Parameter query and inert command endpoint creation need explicit approval; never publish or enable |
| Force01 read | Lazy subscription and configured sensor-to-flange transform | Review actual frame/geometry; unsupported or unverified transform cannot pass |
| Exit | Peripheral atexit hooks and potentially persistent arm service | Review exact ownership and disposition; no unowned service shutdown |

Creating an endpoint is not sending a motion request, but it is also not covered
by the previous subscription-only gate. Fixed effect allowlists and fake-driver
call traces must show no movement, recovery, calibration, or unlisted write.
If local source does not match the selected installation, repeat the inventory.

## 7. Ownership, Sequencing and Failure

Use explicit states:

`prepared -> preflight_passed -> initializing -> observing -> accepted`

Terminal alternatives are `rejected_before_initialization`, `failed`, and
`initialization_outcome_unknown`. A terminal outcome never automatically retries.
An accepted observation result still carries a separate resource disposition.

### Preflight

- Persist the validated intent and initialize a writable evidence destination
  before hardware contact; a logging failure prevents initialization.
- Check for existing runtime service/process/socket ownership and protocol using
  the reviewed procedure. A nonresponsive socket is not proof of no service.
- This first-connection flow does not adopt an existing service. Any pre-existing
  service is reported and handled under a separately reviewed reuse procedure;
  do not overwrite sockets, kill services, or change runtime directories to
  bypass the conflict.
- Require an operator-controlled exclusive commissioning window: known motion
  producers are inactive, no payload is being carried, the work envelope and
  hardware E-stop are supervised. ROS driver presence alone does not establish
  or violate exclusivity. Do not automatically stop other producers.
- The startup path must recheck service ownership at the actual reuse/spawn
  decision, not just earlier in preflight. Existing `ensure_service()` can adopt
  a service that appeared meanwhile. A check before `initialize()` alone is not
  sufficient. If the binding cannot enforce this, live execution stays blocked;
  a narrowly reviewed dependency lifecycle change needs separate approval.

### Initialization and Observation

- Invoke top-level initialization once. It may initialize both arms even though
  the eventual first movement experiment might concern only the left arm.
- No retry, recover, stop-all fallback or manual settings call is permitted.
  Only the initialization-internal effects enumerated in the approval are allowed.
- On a returned error or exception, terminate normal progression immediately.
  The dependency may already have opened one arm before failing on the other.
  Record confirmed partial resources, mark unobservable state unknown, and do
  not claim rollback. No subsequent peripheral reads follow an initialization
  failure.
- Before any initialization request, verify the pinned execution host and acquire
  a durable, atomic commissioning lock keyed by the site and physical endpoint
  set, not merely by the local
  runtime directory or socket path. The lock record contains the attempt,
  manifest digest, host/account, endpoint identities and state. A second
  process, alternate runtime directory, or racing runner must be rejected
  before it can issue an initialization request.
- A timeout is not cancellation. If an RPC/SDK operation can finish late, report
  `initialization_outcome_unknown`, retain the lock and ownership evidence for
  operator reconciliation, and keep that lock across runner exit. It is not
  cleared by time expiration. Only an explicit operator reconciliation record
  matching the attempt, runtime identity, service epoch, and observed disposition
  may release an unresolved lock. A clean accepted attempt releases the lock
  only after all approved operations settle and resource disposition is recorded.
  Do not terminate an arbitrary process and describe the robot as disconnected
  or unchanged.
- A runtime identity or epoch change during observation fails the attempt. Do
  not reconnect or reinitialize implicitly.

### Resource Disposition

- The runner records service PID/start identity, RPC epoch where available,
  initialized endpoints, ownership evidence, and owned ROS resources.
- Normal client exit must be observed; it is not evidence that the arm service
  exited. The present public Robot wrapper has no unified close method.
- Default: do not shut down the persistent arm service automatically. A normal
  run may finish with `retained_for_operator` only when the approved manifest
  explicitly selects retention, records an accountable operator and exact
  surviving resources, and all operations have settled. A later run must not
  adopt it through this first-connection flow.
- If the site requires full teardown instead, its exact supported cleanup and
  ownership verification must be reviewed before first connection. Do not invent
  a `Robot.close()` call or default to process kill. Service shutdown may cancel
  active tasks and is not a harmless release.
- Unsettled work, unverifiable survivors or unconfirmed cleanup require a failed
  or unknown terminal report, never an accepted result. Retention is not physical
  stop confirmation or permission to leave equipment unattended.

## 8. Identity and State Evidence

Identity report separates `configured`, `device_reported`, and
`operator_verified` values. `get_model/get_ip/get_side/get_axes` and
`get_bindings/get_modules` must not all be relabeled as independent device
identification: some methods report configuration. Correlate approved endpoint
mapping with device-reported axis/type/version when the reviewed API provides
it. Missing required identity evidence blocks acceptance.

Required state groups: left/right arm joints and controller status, left/right
gripper feedback, waist lift position/reference/limits, left/right force values
and source frame/transform. Explicitly unsupported axes such as waist pitch are
not requested; chassis and perception remain excluded rather than faked.

For each group record the actual call, result code, finite payload, units,
source, local monotonic request/receipt times, adapter sequence, source timestamp
where available, and sample provenance. Values must match the approved shape,
identity and units. Do not silently substitute raw ROS values for failed
LynrotControl reads or convert joint radians/degrees without a declared rule.

At least two distinct observations per group must occur within the approved
observation budget and freshness bounds. Locally incremented request/callback
sequences prove only distinct requests/receipts, not distinct device acquisition.
Unchanged positions are valid when stationary. Record unavailable source stamps
and separate receipt freshness from sensor-time synchronization. Never invent
`moving: false`, a device sequence, or a hardware timestamp.

Use the reviewed actual transport semantics for freshness: request-response
reads must succeed within their budget; cached subscriptions must deliver new
samples, not just repeated reads of the same cache. Invalid later data prevents
an earlier good sample from being returned as current success.

Force01 embeds a sensor-to-flange transform based on an EA200 description.
It must match Robot One's installed geometry before transformed force readings
can pass. Waist conversion must be justified by the selected adapter and actual
controller parameters. Library numeric limits and fixture values are not site
calibration. Unverified mapping means the relevant required group fails.

## 9. Reports and Acceptance

Write a local structured report and an event log linked by run ID and manifest
digest. Include authorization reference, effective code/configuration hashes,
identity provenance, allowed/observed initialization effects and read outcomes,
sample freshness assessment, failures, outstanding work, and resource disposition.
Hashes provide integrity checks, not tamper-proof auditing. Do not log credentials,
complete environments or unrelated host/process details.

Track three independent acceptance fields: `identity_verified`,
`required_state_groups_observed`, and `resource_disposition_verified`. Overall
acceptance requires all three, matching approval, settled initialization and no
unexplained effects. A partial read or an unconfirmed cleanup is not overall
success. An unsupported requested group is a failure, not a silent scope change.

This report is not a runtime identity receipt for the motion Toolkit, a motion
profile, cancellation test, safety certificate, or permission to move. It expires
with the attempt; future motion requires fresh runtime checks and new approval.

If evidence writing fails after contact, mark the attempt unsuccessful and block
progression; do not issue a software stop merely because logging failed in this
non-motion workflow. Emit a minimal terminal error if possible. Missing logs
mean reconciliation is incomplete, not that no effects occurred.

## 10. Validation and Delivery Gates

Offline implementation tests must cover:

- Import/validation does not import a driver, contact a socket, start a process,
  initialize ROS or write controller settings.
- Wrong host/domain, expired/mismatched approval, changed code/configuration or
  unwritable evidence rejects before initialization.
- A host identity mismatch rejects before lock acquisition; the same physical
  endpoint cannot be approached through a second execution host under this
  design.
- Missing, unreadable, duplicated, or changed canonical machine identity also
  rejects before lock acquisition; hostname-only fallback is forbidden. The
  duplicate check must consult the approved host-enrollment authority; if that
  authority cannot be consulted, uniqueness is unknown and the run rejects.
- Existing or racing service, stale socket and wrong epoch do not get adopted
  or deleted. A failed service probe does not silently authorize a new service.
- One initialization invocation only; all conditional preset writes match the
  approved inventory; no motion/recover/cancel/publish calls escape the allowlist.
- Left-success/right-failure, force binding failure, lazy peripheral failure and
  delayed RPC completion do not retry, claim rollback, or allow continued reads.
- Duplicate/stale/invalid samples, reversed arm mapping, unit mismatch, unknown
  force transforms and missing waist parameters fail the relevant gate.
- Identity change mid-run, Ctrl+C, logging failure and timeout preserve uncertainty
  and ownership evidence; a returning timed-out caller does not unlock a retry.
- The durable commissioning lock is acquired before initialization, is atomic
  across processes and alternate runtime directories, and survives runner
  exits. A second process is rejected while the first initialization is delayed
  and after its caller times out. A clean accepted attempt releases it only
  after settled disposition; unresolved outcomes retain it until explicit
  operator reconciliation, never automatic expiration.
- Explicit normal retention, unknown survivors, partial cleanup and shared-service
  scenarios yield the specified result without killing an unowned process.
- Existing RPent live rejection and model tool surfaces remain unchanged.

Use fakes at the dependency boundary, including transport delays and operation
traces. Reuse existing state-validation patterns where compatible; do not rerun
unrelated simulation as a prerequisite for this phase. Offline passing tests
never replace the later site acceptance run.

Delivery order:

1. Independently review this spec, resolve Important/blocker findings, then get
   explicit user approval of the written revision.
2. Write a focused implementation plan, independently review it, and obtain
   approval before writing runtime code. Resolve ownership enforcement and
   effect auditing feasibility in that plan; do not leave them to live trial.
3. Implement offline, run focused and affected regression tests, and obtain code
   review. Any required dependency modification has explicit scope approval.
4. Publish the concrete site manifest and new exact-command operational runbook,
   with no missing numeric, provenance or lifecycle fields, and obtain separate
   hardware authorization for their digests.
5. Run the single approved attempt; report observed success, partial failure or
   unknown outcome. Only then design the separate motion/cancel trial.

`AGENTS.md` is present in the worktree. The referenced
`docs/codex-workflow-guide.md` was not found; no instructions were fabricated in
its place. No SSH, ROS, SDK initialization or dependency changes occur while
producing this specification.

## Review Record

- Chat approval: scope, reuse of reviewed initialization effects, terminal entry,
  failure handling and acceptance boundaries confirmed by huhu.
- Spec self-review: complete; placeholder, scope, ordering, timeout, ownership,
  and no-motion authorization boundaries checked.
- Independent read-only review: APPROVED; no remaining Important/blocker findings.
- Approval of written spec: approved by huhu in chat.
- Implementation plan/code/site initialization: not started under this spec.
