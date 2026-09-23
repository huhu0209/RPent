# Robot One LynrotControl First-Connection Runbook

**Status:** operator procedure template; not hardware authorization. Passing
tests and validation do not authorize live operation, deployment, SSH, or Robot
One contact. Until the reviewed dependency-side gate is deployed, an old
dependency remains fail-closed with exit code `5`. Do not bypass it, call
LynrotControl directly, or treat this document as permission to initialize
hardware.

## Scope

This command is operator-only. It is not reachable from RPent, a planner, model,
Dashboard, Toolkit, or pytree task flow. `pytree` and `plan_arm_simple` are not
part of commissioning runtime. This procedure authorizes no motion: no
trajectory, recovery, calibration, gripper/waist/chassis command, force
zeroing, stop fallback, or manual controller setting. Initialization may have
side effects and requires a separately reviewed effect inventory and explicit
operator authorization. Never fill missing site values with estimates.

The manifest must bind the approved site and robot identifiers, execution host
machine ID, interpreter, host-enrollment digest, `ROS_DOMAIN_ID=3`,
dependency/configuration artifact digests, endpoint identities, fixed approved
effects, expected arm identity, operator verification evidence, owned ROS
resources, initialization/read budgets, durable lock path, and local evidence
directory. The host must be exactly enrolled and enabled. The root-owned policy
is read from
`/etc/rpent/commissioning/operator-policy.json`; its trusted lock root and
operator allowlist and approved authorization-digest map must match the
reviewed authorization. The authorization file has a self-digest and exact
`attempt_id`; a plain self-constructed JSON record is rejected. Keep
authorization and evidence local and protect them according to site policy.

The ownership protocol is pinned before dependency import. Protocol v1 requires
the dependency artifact
`/home/huhu/work/RPent_lynsense/lynrotcontrol/OWNERSHIP_PROTOCOL.json` (or the
equivalent reviewed deployment path), artifact SHA-256
`78c45f6796cd90fdf63f51ac407ac261c04e42ab454bb6d0897749d81904c84a`, protocol ID
`lynrotcontrol.service-ownership`, protocol version `1`, runtime protocol `15`,
authority root `/tmp/lynrotcontrol-authority-1000`, and read-allowlist SHA-256
`38015e139b4b7ae2c0ca7f2261be2a90cb4d40ee12d35420875c6d150962ceee`. A manifest
that names a different value, a symlink, or an absent artifact is rejected.

## Preflight

1. Obtain a reviewed manifest and host-enrollment authority from the responsible
   site authority. Confirm their digests and the policy file before execution.
2. Obtain an authorization file matching the manifest digest, exact attempt ID,
   host, enrollment digest, policy digest, operator, Robot One identity, and
   domain 3. A run authorization has purpose `commissioning_run`; a reconcile
   authorization has purpose `commissioning_reconcile`. Its self-digest must be
   listed in the trusted policy. A run authorization expiry must be in the
   future. Authorization for this step is distinct from authorization for
   motion.
3. Confirm the execution host and operator-controlled commissioning window by
   the site's separately approved procedure. Do not infer service ownership or
   an empty endpoint from a ROS graph listing.
4. Run static validation from the repository root:

   ```sh
   python -m robots.lynsense_real_box.commissioning_cli validate \
     --manifest <approved-manifest.json> \
     --host-enrollment <approved-host-enrollment.json>
   ```

   This checks local contract, policy, enrollment, host, and artifact digests.
   It does not import a robot driver, open a socket, start a worker, or contact
   Robot One. A passing result is not permission to run.

## Effectful Command

Only after the separate effect inventory and authorization are approved, the
operator command shape is:

```sh
python -m robots.lynsense_real_box.commissioning_cli run \
  --manifest <approved-manifest.json> \
  --host-enrollment <approved-host-enrollment.json> \
  --authorization-file <approved-authorization.json>
```

The CLI records local evidence before starting its fixed subprocess worker and
consumes the exact attempt under the trusted lock root and acquires a durable
lock scoped to the manifest's physical endpoint set. The worker result is bound
to the attempt, manifest digest, and endpoint scope. There
is one attempt only: no retry, cancellation, recovery, lock timeout, or
automatic lock release after an uncertain result. The worker calls the fixed
`build_effectful_gate`, which verifies all reviewed source/config digests and the
protocol artifact before importing LynrotControl. The dependency claim is
released only after fixed reads and final service/resource checks settle. With
an old dependency that cannot enforce this gate, expected current outcome remains
fail-closed exit `5`. Do not alter the gate or invoke the dependency to get
around this result.

Exit codes are stable: `0` means identity, required state, and resource
disposition were all accepted; `2` means pre-contact rejection; `3` means a
known initialization/read failure; `4` means an unknown outcome with retained
lock; `5` means actual service-ownership enforcement is unavailable. Preserve
the emitted result and local evidence. Do not interpret exit `5` as a
successful connection.

## Retained Lock and Reconciliation

A timeout, interrupted supervisor, incomplete result, or uncertain disposition
retains the lock indefinitely. A late worker result does not release it. Never
delete or edit the lock manually and never start another attempt against the
same endpoint scope while it remains.

Reconciliation is a separate explicit operator action. First establish through
the approved local service-ownership procedure that the late worker is no
longer active and that the recorded service identity and initialized endpoint
set are understood. If there is no durable matching attempt result, complete
service identity, and known endpoint scope, do not reconcile. The command only
reads local files; it does not initialize or contact the robot:

```sh
python -m robots.lynsense_real_box.commissioning_cli reconcile \
  --manifest <same-approved-manifest.json> \
  --lock <manifest-derived-commissioning.lock> \
  --authorization-file <explicit-reconciliation-authorization.json>
```

The CLI verifies the unknown lock, attempt result, manifest digest, endpoint
scope, authorization, operator, and service identity, records durable reconcile
evidence and a pending report before lock release, then writes a final report
after verified release. It also verifies the trusted-root consumed-attempt
record. Where the lock lacks a
late service identity, it records the verified identity first; only then does
this explicit reconcile operation release the matching retained lock. Any
mismatch leaves the lock retained. The authorization must have been valid when
the retained lock was created and remain listed by the trusted policy; it may
have expired while the outcome was unresolved. Reconciliation does not
establish accepted device state and grants no motion or Toolkit live permission.

Dependency claim inspection runs through the same pre-import artifact gate. An
orphaned, active, release-failed, or invalid/pending authority record is recorded
as `dependency_reconciliation_unavailable`; the RPent lock stays retained and
dependency records are not deleted or edited. Protocol v1 has no tokenless
dependency cleanup. Deployment and Robot One operation require separate
approvals.

## Evidence and Boundaries

Evidence and reports are written beneath the manifest's local evidence
directory. Preserve the JSONL event stream, attempt result, final report,
manifest, host enrollment, policy digest, and authorization together for
review. Acceptance records identity, state-group observation, and resource
disposition independently; a result missing any one is not accepted.

No Robot One numeric configuration values are supplied here. Use only values
in an independently reviewed site manifest; simulation values, dated samples,
or guessed values are not substitutes. This runbook does not authorize ROS,
SSH, SDK, hardware, motion, or Dashboard operations by itself.
