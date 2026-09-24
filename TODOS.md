# Robot One Integration TODOs

Last updated: 2026-09-24

## Current State

### Completed

- Robot One commissioning and direct owned-state reads are working on
  `rpp-PC` with `ROS_DOMAIN_ID=3`.
- The commissioning worker now waits for source evidence to advance instead of
  counting two rapid reads of one cached subscription sample as two
  observations.
- `robot_one_readonly` is registered as a real Robot One backend.
- Its exact model surface is `read_robot_state`.
- Dashboard primitive dispatch to `read_robot_state` was successfully exercised
  against Robot One and returned all seven fixed state groups.
- The Toolkit closed and LynrotControl released the claim after the read.
- Final observed claim disposition was `12/12 claim_released`.
- No Robot One motion, gripper command, chassis command, or perception trigger
  has been executed from RPent.

### Validation Evidence

- Local focused Robot One/Dashboard tests: `62 passed`.
- Full `tests/unit_tests/robots/lynsense_real_box` suite most recently reported
  `502 passed, 2 failed`; both failures are the existing README/runbook SHA
  baseline mismatch and are unrelated to the read-only backend.
- Remote `robot_one_readonly` Dashboard primitive test passed:
  `DASHBOARD_PRIMITIVE read_robot_state ok`.
- Remote test service identity: PID `8008`, start ID `20904`, epoch
  `cf17f48eb2a64dfd997e6aabc7b79e2a`.
- Remote final claim state: `claim_released` for all 12 endpoint records.

## Blockers

1. **CTAG gripper software stop**
   - Both CTAG grippers currently report `supports_cancel=false`.
   - A physical E-stop is not a replacement for a confirmed software stop API.
   - Live gripper capability remains blocked until a reviewed stop/cancel
     interface is found or implemented.

2. **Real motion site profile**
   - No reviewed Robot One v2 live motion profile exists yet.
   - Required numeric evidence includes paired arm paths and limits, gripper
     open/close calibration, waist range, bounded chassis limits, force gates,
     stop budgets, freshness bounds, runtime identity, and execution host.

3. **Live chassis and perception transports**
   - The bounded `/odom` + `/cmd_vel` chassis transport has not been exercised
     against Robot One.
   - The `/industrial_box/*` perception transport has not been connected to the
     live atomic Toolkit.

4. **Box-control evidence**
   - Carrying-chassis profiles must fail closed without reviewed held-box
     evidence and force monitoring.

5. **Repository synchronization**
   - Local `main` is ahead of and behind `origin/main`.
   - Do not merge, rebase, reset, or push without explicit user approval.

## Next Work

1. **Preserve the read-only gate**
   - Commit the Robot One read-only backend, commissioning freshness fix, CLI
     Dashboard opt-in, tests, `AGENTS.md`, and this TODO file.
   - Keep the remote deployment hashes in sync after the local commit.
   - Capture the successful Dashboard transcript under the remote run output if
     it is not already retained.

2. **Read-only CTAG stop investigation**
   - Inspect the CTAG ROS graph, generated service definitions, installed
     driver source, and LynrotControl adapter APIs.
   - Allowed operations are list/type/info/source reads only.
   - Do not publish, call a service, send an action goal, move a gripper, or
     change a parameter.
   - Exit condition: identify a confirmed stop API, or record that no such API
     exists and keep live gripper dispatch blocked.

3. **Design the waist minimal-motion gate**
   - Prefer waist first because `waist01` reports cancel support.
   - First gate should be current-position or at most a 1 mm fixed delta.
   - Required behavior: fresh pre-read, reviewed in-range target, timeout,
     software cancel/stop confirmation, arrival readback, claim release, and
     operator-written stop condition.
   - This gate needs a separate implementation review and explicit live-motion
     authorization before dispatch.

4. **Build the real v2 site profile**
   - Use only measured Robot One evidence.
   - Do not import numerical values from Webots, Isaac, pytree, or dry-run
     fixtures.
   - Validate exact subsystem freshness, force limits, paired arm paths,
     gripper values, waist range, chassis bounds, stop budgets, runtime hashes,
     and execution identity.

5. **Exercise live transports without a box**
   - Validate perception topic identity and trigger behavior.
   - Validate bounded chassis `/odom` preflight, stop, and zero/small-displacement
     behavior.
   - Keep each component gate separate before composing them.

6. **Return to the eight-tool atomic surface**
   - Only after gripper stop, waist gate, profile review, and transports pass,
     connect the reviewed fixed eight-tool live Toolkit.
   - First composed hardware gates must use no box, then an approved surrogate
     payload. The production box is not the first live payload.

## Operational Notes

- Current Dashboard backend: `robot_one_readonly`.
- Current tool surface: `read_robot_state` only.
- Current Dashboard is conversational during a task, but it cannot command
  motion.
- LLM configuration on `rpp-PC` is `/home/zxh/.config/rpent/lynsense.env`.
  Source it before starting Dashboard; do not print or commit the API key.
- For LAN Dashboard access, restart with a bind host such as
  `--dashboard-host 192.168.77.249`; do not bind wider than necessary.
