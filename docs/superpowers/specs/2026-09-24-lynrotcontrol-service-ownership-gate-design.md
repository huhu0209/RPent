# LynrotControl Service Ownership Gate Design

## Status

Approved by independent review and the user; implementation plan pending.

This design covers the dependency lifecycle change required before Robot One
commissioning can leave its current fail-closed exit `5` state. It does not
authorize modifying the external repository, deploying a patch, connecting ROS,
initializing LynrotControl, or operating Robot One.

## Problem

RPent commissioning already validates authority, host identity, source/config
inventories, authorization purpose, endpoint scope, and the durable attempt.
The remaining unsafe boundary is inside `lynrotcontrol`.

The current `lynarmcontrol` client's `ensure_service()`:

1. acquires the runtime-directory `startup.lock`;
2. pings `control.sock`;
3. reuses a responsive service when its protocol matches;
4. otherwise spawns the persistent service and waits for a ping.

That lock serializes startup in one runtime directory. It does not represent
ownership of Robot One's physical endpoint set and does not prevent an unowned
caller from issuing initialization, settings, task, or motion requests after
commissioning starts. A preflight outside the dependency therefore cannot prove
that the actual reuse/spawn decision and following initialization are exclusive.
The first draft also left a spawn-to-claim window: the service could bind its
public socket before the commissioning claim existed, allowing a direct socket
caller to initialize an endpoint between startup and claim.

## Goal and Non-Goals

The dependency lifecycle API must:

- make the no-service/spawn/existing-service decision while `startup.lock` is held.
- reject a responsive pre-existing service for this first connection rather than adopt it.
- claim the approved endpoint scope before top-level Robot initialization.
- hold dependency-wide authority over every canonical physical endpoint,
  independent of the runtime directory selected for this process.
- activate the claim before the public `control.sock` is bound or listens.
- return a service receipt with PID, process start identity, epoch, protocol,
  runtime paths, endpoint scope, ROS domain, initialized endpoints, and owned resources.
- block unowned initialization and all settings, task, and motion operations
  while the commissioning claim is active.
- permit only fixed commissioning reads and lifecycle operations under the claim.
- preserve uncertainty for timeouts, late startup, partial initialization,
  epoch changes, and release failures.

Non-goals:

- No RPent planner, Dashboard, Toolkit, pytree, or model access to the gate.
- No motion, trajectory, recovery, calibration, stop-all fallback, force zeroing,
  gripper/waist/chassis command, or perception trigger.
- No automatic shutdown or kill of an existing or partially initialized service.
- No service reuse in the first-connection attempt.
- No software claim treated as an E-stop or a replacement for the operator's
  exclusive commissioning window.
- No deployment, dependency installation, remote write, SSH, ROS connection,
  or Robot One operation during implementation or tests.

## Architecture

Add the lifecycle boundary inside `lynrotcontrol`; do not build a second
launcher in RPent. The dependency exposes one fixed initialization operation:

```python
initialize_under_ownership(
    instance: str,
    *,
    site_id: str,
    robot_id: str,
    endpoint_scope_sha256: str,
    ros_domain_id: int,
    approved_effects: tuple[str, ...],
    expected_endpoints: tuple[CanonicalEndpoint, ...],
) -> OwnedRuntime
```

`CanonicalEndpoint` is a dependency-owned, immutable value with exactly
`subsystem`, `endpoint_id`, `transport`, and `locator`. The dependency derives
the same set from the reviewed instance configuration and rejects the request
before spawn or device contact if it differs from `expected_endpoints`. It does
not accept a caller-selected runtime directory, authority directory, module,
import path, executable, environment mutation, or callable.

The endpoint scope digest is normative and is computed exactly as RPent's
commissioning contract computes it: canonical JSON with sorted keys and no
spaces over:

```json
{
  "endpoints": [
    {
      "endpoint_id": "<endpoint_id>",
      "locator": "<locator>",
      "subsystem": "<subsystem>",
      "transport": "<transport>"
    }
  ],
  "robot_id": "<robot_id>",
  "site_id": "<site_id>"
}
```

The endpoint array is sorted by the four-field tuple
`(subsystem, endpoint_id, transport, locator)`. The digest is the lowercase
hex SHA-256 of those UTF-8 bytes. The dependency recomputes it from
`site_id`, `robot_id`, and `expected_endpoints`, and rejects a mismatch before
spawn or device contact.

`OwnedRuntime` exposes only the initialized Robot runtime used by commissioning
fixed reads, a receipt that excludes the claim token, fixed resource inventory
queries, and explicit claim release. The token remains in process memory and is
never placed in evidence, logs, environments, arguments, or persistent files.

The dependency also exposes one read-only inspection operation:

```python
inspect_claim(site_id: str, robot_id: str, expected_endpoints: tuple[CanonicalEndpoint, ...]) -> ClaimInspection
```

`inspect_claim` performs no release or cleanup. Protocol version 1 exposes no
tokenless reconciliation or cleanup API: active, orphaned, release-failed,
pending, and invalid records remain blocking. A future cleanup protocol must
separately define a dependency-verifiable authorization binding and an atomic
endpoint-lock transition before it can supersede this fail-closed behavior.

Rejected alternatives:

- **RPent-owned launcher:** duplicates private lifecycle behavior, drifts with
  dependency updates, and leaves `lynrotcontrol.initialize()` able to use the
  unguarded path.
- **Operational serialization only:** cannot close the code race and can only
  supplement the gate.
- **Adopt an existing service:** couples first connection to unknown prior state
  and cleanup obligations; reuse requires a separate future design.

## Dependency Protocol Artifact

RPent must detect an old dependency before importing it. The reviewed source
inventory therefore contains the exact file:

```text
lynrotcontrol/OWNERSHIP_PROTOCOL.json
```

RPent reads this fixed artifact with the standard library from the manifest's
already digest-verified source path. It validates strict JSON with exactly
these fields:

- `protocol_id`, which must equal `lynrotcontrol.service-ownership`;
- `protocol_version`, which must equal the integer `1`;
- `read_allowlist_sha256`, the SHA-256 of the canonical JSON allowlist table
  below;
- `endpoint_authority_root`, the absolute production authority root;
- `runtime_protocol`, which must equal the exact service protocol integer.

The artifact digest and every field value are bound in the site manifest. If
the artifact is absent, malformed, or differs by one byte from its manifest
digest, RPent returns `ServiceOwnershipUnavailable` without importing
`lynrotcontrol`, opening its socket, or spawning its service. The runtime
service must use the same protocol constants and allowlist digest; receipt or
metadata mismatch is an identity failure, not a fallback to legacy behavior.

The production endpoint authority root is dependency-wide for the executing
UID and is not selected from `LYNROTCONTROL_RUNTIME_DIR`, the request, the
environment, or a package installation path. Tests may replace the root only
through an explicit dependency test harness, never through the public API.

## Endpoint Authority

Before the runtime `startup.lock` is acquired, the initializer resolves the
endpoint set and acquires one exclusive nonblocking `authority.lock` for each
canonical physical endpoint:

```text
<endpoint_authority_root>/endpoints/<sha256(canonical_endpoint_json)>/authority.lock
```

The authority root and every component are validated as nonsymlink,
current-account-owned paths with no group or other access. Endpoint records and
locks use names derived only from the canonical endpoint descriptor, never from
an unhashed locator. Locks are attempted in the canonical descriptor's sorted
order. If any lock is already held, all locks acquired in this attempt are
released and the API returns an endpoint ownership conflict with the conflicting
scope when it can be reported without exposing credentials.

The complete lock order is:

1. all endpoint authority locks, sorted by canonical endpoint descriptor;
2. the selected runtime directory's `startup.lock`;
3. the selected runtime directory's `service.lock`;
4. the in-process service registry lock.

No code path may take these locks in another order. The service retains the
endpoint authority and `service.lock` descriptors for its process lifetime,
which is longer than an individual claim and prevents a second runtime from
owning the same physical endpoints while this retained service remains alive.

### Universal Legacy-Path Enforcement

Durable authority applies to every LynrotControl spawn and initialization path,
not only to `initialize_under_ownership`. After this protocol is installed:

- top-level `lynrotcontrol.initialize(instance)` derives the complete
  canonical endpoint set from its reviewed instance configuration before
  service spawn or device contact;
- `lynarmcontrol.initialize(...)` derives the canonical endpoint for its
  resolved arm specification before service spawn or device contact;
- top-level initialization acquires the complete set once and passes that
  authority handle to its internal arm initialization; standalone arm
  initialization acquires only its own endpoint, so no path acquires the same
  authority lock twice;
- every path that can spawn the service supplies that nonempty endpoint set to
  the common endpoint-authority acquisition routine;
- a compatibility `ensure_service()` call with no endpoint set may ping an
  existing service but must fail closed rather than spawn;
- direct execution of the service entrypoint without inherited endpoint
  authority descriptors fails before binding `control.sock`;
- a responsive existing service is usable only when its receipt proves that it
  holds authority for every requested canonical endpoint.

Before deciding that no service exists, every spawn path also performs a
read-only Linux process inventory and rejects any live process whose executable
or command line identifies `lynrotcontrol.lynarmcontrol.implementation.service`
unless that same PID/start identity is the authority-holding service in the
current receipt. This catches an old cross-runtime service that predates the
protocol and therefore never acquired endpoint authority. Once this protocol is
installed, its updated service entrypoint cannot start without authority, so a
new old-protocol process cannot appear after this scan.

All of these paths inspect every durable record and lock observation while
holding the relevant endpoint authority locks. The states `claim_active`,
`claim_pending`, `claim_orphaned`, `claim_release_failed`, and
`authority_record_invalid` reject initialization and service spawn. Only a complete
`claim_released` historical record is nonblocking, and even then a live retained
service's endpoint locks remain blocking. The spawned service rechecks its
inherited endpoint set and record state before public bind and again in the
registry initialization dispatch.

A token-bearing `initialize` request is additionally restricted to the exact
internal arm specifications derived from the reviewed instance configuration,
in fixed left-then-right order. It accepts no alternate model, IP, side, force
binding, or caller-selected endpoint key. An unclaimed legacy service may
continue to use its old public API only if it proved endpoint authority for the
requested endpoints and no blocking durable record exists; a service under a
commissioning claim or in `retained_no_claim` rejects unowned initialization
and all settings, task, and motion operations as specified below.

If a public socket responds while endpoint authority is held, the API returns
`ExistingServiceConflict` with the available no-token protocol and identity
receipt; it does not adopt, claim, initialize, stop, or shut down that service.
If `service.lock` is already held, the initializer reports
an unresponsive-or-locked service conflict. The spawned child must report this
condition over its private bootstrap channel and exit nonzero; correct behavior
must never depend on a second service silently exiting.

When the parent holds `startup.lock` and the child holds `service.lock`, a
stale `control.sock` may be removed only if it is a current-account-owned Unix
socket, it does not respond to ping, and no service lock is held. The child
removes it before binding and fsyncs the runtime directory. It must never
remove a regular file, directory, symlink, another account's socket, or a
socket that responded.

Each endpoint directory also contains a token-free `claims/` directory. On
claim activation the child writes the same canonical JSON record to
`claims/<endpoint_scope_sha256>.json` in every endpoint directory, fsyncs each
file and parent directory, and records the record digest in its public receipt.
The record contains endpoint scope, complete canonical endpoint set, approved
effects, ROS domain, service PID, `/proc` start identity, service epoch, claim
state, monotonic-independent creation timestamp, protocol constants, and record
digest. The digest is SHA-256 over the canonical JSON object with the
`record_sha256` field omitted. The record never contains a token or token
digest.

Before writing, the child verifies that no active, pending, orphaned,
release-failed, invalid, or otherwise blocking historical claim blocks any
endpoint. The writes are
individually atomic, but a crash between them is intentionally observable as
`authority_record_invalid`; it is never automatically repaired. Successful
release updates every copy to a token-free released state. A failed or partial
update yields `claim_release_failed` or `authority_record_invalid`, both of
which remain blocking.

## Lifecycle State Machine

The owned initializer uses:

`validated -> endpoint_authority_acquired -> startup_lock_acquired -> bootstrap_spawned -> pending_claim -> claimed -> initialized -> observed -> released`

Terminal alternatives are typed failures or unknown outcomes, never a generic
connection error.

### Local Validation

Before spawn or device contact, verify:

- the approved ROS domain is exactly the supplied integer, initially `3`;
- the current environment has that `ROS_DOMAIN_ID`; the API never sets it silently;
- instance, site identity, robot identity, endpoint scope, effects, and
  expected endpoints match fixed schemas;
- the runtime directory remains a nonsymlink, current-account-owned `0700` directory;
- the owned path ignores `LYNROTCONTROL_RUNTIME_DIR` and uses the fixed
  dependency-derived production runtime directory;
- socket and lock paths derive from that directory rather than caller input.

### Startup Decision

Under `startup.lock`, perform exactly one decision:

- **Responsive service:** return `ExistingServiceConflict` with its available
  no-token receipt; do not claim, modify, initialize, or shut it down.
- **No responsive service and no live service owner:** spawn the normal
  persistent service in bootstrap mode with a private inherited socketpair and
  the already-acquired endpoint authority descriptors. In bootstrap mode the
  child does not bind, unlink, replace, or listen on public `control.sock`.
  It first acquires the nonblocking `service.lock`, then enters
  `pending_claim`.
- **Unresponsive socket or held service lock:** report an existing or
  unresponsive service conflict. A failed ping is not proof that no service
  exists and must not cause a second owner.

In `pending_claim`, the parent sends one random 256-bit claim token, endpoint
scope, ROS domain, approved effects, endpoint descriptors, and protocol digest
over the private bootstrap socketpair. The child stores only a constant-time
comparison form such as the token's SHA-256 digest, creates a durable
token-free authority record, activates the claim, binds and listens on public
`control.sock`, and only then acknowledges the claim over the bootstrap
channel. The parent verifies the child PID, `/proc` start identity, service
epoch, protocol, endpoint record digest, and public ping before closing the
private channel.

Consequently, there is no public endpoint to which a direct initialize,
settings, task, or motion request can connect before the claim is active. Once
the public socket exists, registry dispatch already enforces the claim. A
bootstrap timeout, malformed handshake, child identity mismatch, authority
write failure, or public bind failure is an unknown startup outcome. The child
is not killed and no actor may unlink its socket to force a retry.

### Commissioning Claim

The bootstrap claim is accepted only once. It records only a token digest,
endpoint scope, ROS domain, approved effects, endpoint descriptors, creation
time, service identity, and service epoch in memory. The durable authority
record contains the same public fields, an atomic record digest, and no token
or token digest. RPent's attempt ID remains outside the dependency and is bound
by RPent authorization, lock, request, result, and evidence.

Only one active commissioning claim is allowed. While it is active, service
dispatch:

- allows ping, no-token `claim.inspect`, token-bearing `claim.release`, and
  token-bearing receipt refresh;
- allows initialization only with the matching claim token;
- allows only the exact read table in the next section, with the matching token;
- rejects unowned initialization;
- rejects settings, task, and motion operations.

The check occurs before dynamic import, endpoint construction, ROS context
creation, argument defaulting, or method lookup. Requests with omitted
arguments, extra arguments, extra fields, unknown endpoint IDs, unknown method
names, or noncanonical positional/keyword forms are rejected.

Enforcement occurs in the service registry dispatch path, not only in a client
wrapper, so a direct socket caller cannot bypass it.

### Exact Read Allowlist

The sole approved effect set for first connection is exactly, in this order:

```text
initialize_configuration
fixed_read_ros_resource_creation
fixed_read_ros_parameter_query
```

`initialize_configuration` covers reviewed configuration loading and
construction of fixed SDK and Robot objects. The two fixed-read effects permit
only:

- creating the dependency-owned `rclpy` contexts and nodes required by the
  table below;
- creating read-only subscriptions and read-only parameter-service clients;
- invoking the exact configured waist `GetParameters` service needed to convert
  feedback to site units.

They do not permit publishers, control service clients, topic publication,
action goals, calibration, recovery, motion, cancellation, or perception. If an
existing adapter would construct a command publisher or gripper/waist control
client merely to satisfy a read, the dependency must separate its read-only
connection path or fail the read. Resource creation is inventoried before the
first fixed read.

For direct socket dispatch, `call` is accepted only with the matching token and
one of these exact requests. The resource identifier is the logical identifier
in the first column (`left_arm` or `right_arm`); the service maps it to the
canonical endpoint key recorded during initialization and never accepts a
caller-selected arm key. Each request also carries the current protocol and
service epoch and no extra fields.

| Resource | Section and method | Positional args | Keyword args |
| --- | --- | --- | --- |
| left arm | `reading.get_model` | `()` | `{}` |
| left arm | `reading.get_ip` | `()` | `{}` |
| left arm | `reading.get_side` | `()` | `{}` |
| left arm | `reading.get_axes` | `()` | `{}` |
| left arm | `reading.get_version` | `()` | `{}` |
| left arm | `reading.get_state` | `()` | `{}` |
| left arm | `reading.get_joints` | `()` | `{}` |
| right arm | `reading.get_model` | `()` | `{}` |
| right arm | `reading.get_ip` | `()` | `{}` |
| right arm | `reading.get_side` | `()` | `{}` |
| right arm | `reading.get_axes` | `()` | `{}` |
| right arm | `reading.get_version` | `()` | `{}` |
| right arm | `reading.get_state` | `()` | `{}` |
| right arm | `reading.get_joints` | `()` | `{}` |

The normative allowlist object is exactly:

```json
{
  "direct_calls": [
    {"resource": "left_arm", "section": "reading", "method": "get_model", "args": [], "kwargs": {}},
    {"resource": "left_arm", "section": "reading", "method": "get_ip", "args": [], "kwargs": {}},
    {"resource": "left_arm", "section": "reading", "method": "get_side", "args": [], "kwargs": {}},
    {"resource": "left_arm", "section": "reading", "method": "get_axes", "args": [], "kwargs": {}},
    {"resource": "left_arm", "section": "reading", "method": "get_version", "args": [], "kwargs": {}},
    {"resource": "left_arm", "section": "reading", "method": "get_state", "args": [], "kwargs": {}},
    {"resource": "left_arm", "section": "reading", "method": "get_joints", "args": [], "kwargs": {}},
    {"resource": "right_arm", "section": "reading", "method": "get_model", "args": [], "kwargs": {}},
    {"resource": "right_arm", "section": "reading", "method": "get_ip", "args": [], "kwargs": {}},
    {"resource": "right_arm", "section": "reading", "method": "get_side", "args": [], "kwargs": {}},
    {"resource": "right_arm", "section": "reading", "method": "get_axes", "args": [], "kwargs": {}},
    {"resource": "right_arm", "section": "reading", "method": "get_version", "args": [], "kwargs": {}},
    {"resource": "right_arm", "section": "reading", "method": "get_state", "args": [], "kwargs": {}},
    {"resource": "right_arm", "section": "reading", "method": "get_joints", "args": [], "kwargs": {}}
  ],
  "runtime_methods": [
    {"resource": "robot", "method": "get_info", "args": [], "kwargs": {}},
    {"resource": "robot", "method": "get_bindings", "args": [], "kwargs": {}},
    {"resource": "robot", "method": "get_modules", "args": [], "kwargs": {}},
    {"resource": "left_gripper", "method": "read", "args": [], "kwargs": {}},
    {"resource": "right_gripper", "method": "read", "args": [], "kwargs": {}},
    {"resource": "waist_lift", "method": "read", "args": [], "kwargs": {}},
    {"resource": "left_force", "method": "read", "args": [], "kwargs": {"tcp_id": 0}},
    {"resource": "right_force", "method": "read", "args": [], "kwargs": {"tcp_id": 0}}
  ]
}
```

The artifact digest is SHA-256 of this object serialized as canonical JSON:
sorted keys, `(',', ':')` separators, UTF-8 encoding, and no trailing newline.
The implementation may not reorder, default, or normalize request arguments
before this comparison.

`OwnedRuntime` additionally exposes only these fixed-arity dependency methods:
`robot.get_info`, `robot.get_bindings`, `robot.get_modules`,
`left_gripper.read`, `right_gripper.read`, `waist_lift.read`,
`left_force.read(tcp_id=0)`, and `right_force.read(tcp_id=0)`. Their internal
ROS access remains subject to the two fixed-read effects and exact resource
inventory above. No arbitrary object, method, section, or argument dispatch is
part of the runtime protocol.

### Owned Initialization and Observation

The dependency invokes its own top-level Robot initialization while the claim
is active and attaches the claim token to initialization requests. It accepts
no caller initializer, import path, callable, shell command, or alternate
configuration source.

If initialization fails after one arm or peripheral is constructed, the claim
remains active. The API reports available partial resources and does not retry,
recover, close, stop, or infer rollback.

The receipt is refreshed after initialization and again after fixed reads. It
reports canonical dependency-owned service and ROS resource names. RPent must
not invent or infer these names.

### Claim Release

Only an explicit token-bearing lifecycle operation releases the claim.
Successful commissioning releases it after all required reads, final resource
inventory, and final service identity check. The persistent service may remain
alive under the reviewed site retention policy.

After a successful release the retained service enters `retained_no_claim`.
In that state it permits ping, no-token inspection, and separately designed
future claim acquisition only; it rejects initialization, settings, task,
motion, and fixed commissioning reads. A legacy `lynrotcontrol.initialize()`
caller can therefore reuse the responsive socket but cannot initialize or
operate the retained service without a future reviewed claim protocol.

Failure, worker crash, partial initialization, observation failure, or release
failure leaves the claim active. There is no time-based automatic release.
Normal service exit closes only process-local transports; it issues no stop,
recovery, rollback, or other device command. Exiting while a claim is active
produces the orphaned or invalid-record state, never an inferred release.

`inspect_claim` returns exactly one of:

- `no_claim`, only when no blocking record exists and no endpoint authority
  lock is held;
- `claim_released`, when every endpoint copy records the successful release;
- `claim_active`, when a service with the recorded PID/start identity/epoch is
  alive, holds the endpoint authority descriptors, and exposes the recorded
  public claim state;
- `claim_orphaned`, when the durable record exists but the recorded service
  identity is not live, the service no longer holds its authority, and no
  different live owner holds any endpoint lock;
- `claim_release_failed`, when a token-bearing release was attempted but its
  final disposition is unknown;
- `authority_record_invalid`, for missing, duplicate, malformed, or conflicting
  records, including authority locks held before a durable claim record exists;
  that condition may be a live bootstrap or crash residue, so its disposition
  is unknown and it remains blocking.

Inspection changes no state and never returns a token or token digest.
Tokenless release is impossible. `claim_active`, `claim_orphaned`,
`claim_release_failed`, and `authority_record_invalid` all block a new first
connection attempt. A released record is historical evidence, not service
reuse authority.

RPent's existing commissioning reconciliation may release only its own durable
attempt lock. It must report that dependency claim reconciliation is unavailable
when `inspect_claim` returns a blocking state. A live service with a lost token
and an orphaned or invalid record both remain blocked until a separately
designed, dependency-verifiable lifecycle protocol handles them; this
specification provides no backdoor.

## RPent Integration

RPent's fixed `build_effectful_gate(request)` remains the only bridge. It:

1. reads and digest-verifies `OWNERSHIP_PROTOCOL.json` before dependency import;
2. verifies that the inventoried dependency advertises the exact lifecycle
   protocol, allowlist digest, authority root, and runtime protocol;
3. calls only `initialize_under_ownership`;
4. returns `ServiceOwnershipUnavailable` for an old dependency before import or
   initialization, preserving current exit `5`;
5. validates receipt endpoint scope, canonical endpoints, ROS domain, protocol,
   service identity, authority record digest,
   expected endpoints, and resources against the manifest;
6. returns the fixed commissioning runtime adapter;
7. leaves claim release to the worker's explicit settled-success path.

No request field may choose an arbitrary module, function, executable,
environment mutation, or callable.

## Failure Semantics

| Outcome | Required behavior |
| --- | --- |
| Old dependency or unsupported protocol | ownership unavailable before import/call |
| Environment, runtime directory, or schema mismatch | typed preflight rejection before spawn |
| Responsive existing service | conflict; no claim, initialization, or shutdown |
| Unresponsive or locked service | existing/unresponsive conflict; no second owner |
| Endpoint authority conflict | typed conflict after releasing only locks acquired by this attempt |
| Public socket visible before claim | impossible by protocol; observed occurrence is a critical unknown startup failure |
| Bootstrap or authority record failure | unknown startup outcome; no kill, unlink, retry, or cleanup |
| Spawn timeout | unknown outcome with child identity when available; no kill |
| Child PID/start/epoch mismatch | unknown startup outcome; no adoption |
| Claim conflict | known failure; existing claim unchanged |
| Active, orphaned, release-failed, or invalid claim record | new attempt blocked; inspection only |
| Initialization failure | known failure with partial resources; claim remains |
| Observation failure | known failure; claim remains |
| Service identity or epoch change | failed attempt; no reconnect/retry |
| Release failure | unknown disposition; never accepted |

RPent maps these results to its existing commissioning statuses and retained
lock behavior. Arbitrary dependency exceptions must not become exit `5`.

## Evidence and Site Inputs

The dependency receipt supplements, but does not replace, RPent host enrollment,
trusted operator policy, manifest and authorization digests, attempt purpose and
expiry, endpoint scope, source/config hashes, and operator-verified identity.

The final site manifest must bind exact lifecycle protocol, runtime paths,
canonical resource names, and dependency source digests after the API is final.
No simulation value or unverified Robot One number is acceptable.

## Test Strategy

External dependency tests use fake Unix sockets, subprocesses, clocks,
filesystems, ROS clients, and service registries. They must cover:

- strict `OWNERSHIP_PROTOCOL.json` parsing and digest/protocol mismatch;
- exact endpoint-scope digest computation and mismatch rejection;
- import safety and startup-lock serialization with legacy `ensure_service()`;
- two different runtime directories conflicting on one canonical endpoint;
- partial endpoint overlap, sorted lock order, and release of only acquired
  authority locks;
- compatibility `ensure_service()` failing closed instead of spawning when no
  endpoint set is supplied;
- direct service entrypoint refusal without inherited endpoint authority;
- preexisting old-protocol service process refusal before a no-service decision;
- crash after claim followed by concurrent top-level and arm-level legacy
  initialization;
- every spawn and initialization path refusing each blocking record state;
- partial multi-endpoint claim corruption remaining blocking;
- responsive, locked, and safely removable stale socket cases;
- nonblocking `service.lock` conflict reporting, with no reliance on silent
  child exit;
- private bootstrap handshake, inherited authority descriptors, child
  PID/start/epoch/protocol verification, and no public socket before claim;
- direct initialize, settings, task, and motion attempts during bootstrap and
  after public bind;
- every positive row of the exact read table;
- unknown methods, extra fields, omitted arguments, extra arguments, wrong
  positional/keyword forms, wrong endpoint, and invalid token rejection;
- lazy ROS effects, including rejection of command publishers and control
  clients on read-only paths;
- exact waist parameter query and no other ROS service call or publication;
- partial initialization, token-bearing release, release failure, and service
  restart invalidation;
- no-token inspection for `claim_active`, `claim_orphaned`,
  `claim_release_failed`, and `authority_record_invalid`;
- absence of a public tokenless reconciliation API, RPent reporting dependency
  reconciliation unavailable, and blocked new attempts;
- resource inventory and receipt/log credential filtering.

RPent tests remain fully fake and must cover old-dependency exit `5`; fixed API
metadata consumption before `sys.modules` changes; exact endpoint descriptor
and effect-set validation; typed error mapping; receipt validation; claim
release only after final identity/resource checks; claim retention on
failure/crash; dependency reconciliation-unavailable mapping; unchanged
planner/Dashboard/Toolkit surfaces; and existing offline import/network audits.

## Delivery Gates

Before implementation:

1. this specification passes independent read-only review with no Blocker or
   Important finding;
2. the user approves the written specification;
3. a separate implementation plan defines the exact external dependency diff,
   RPent integration diff, tests, and validation commands.

Implementation remains offline. Dependency deployment requires separate approval
and exact diff review. A real Robot One run additionally requires a new site
manifest, policy record, authorization, and operational authorization. Passing
these tests authorizes neither motion nor box carrying.
