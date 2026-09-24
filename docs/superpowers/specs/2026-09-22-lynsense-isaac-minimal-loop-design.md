# Lynsense Isaac Sim Minimal Loop Design

## Goal

Create a company-aligned Isaac Sim path for the Lynsense robot without weakening
the accepted Webots baseline.

The first milestone imports the supplied company URDF into USD, opens the
resulting model in the company's Isaac Sim 3-era desktop environment, inspects
the articulation joint tree and state, and commands one bounded simulated arm
motion. Successful completion validates model import, visibility, state access,
and actuation; it does not reproduce the competition box-moving task.

## Current State And Inputs

- The Webots smoke, navigation, box-task, viewer, and RPent single-box gates
  remain the verified baseline and continue to work independently.
- Company colleagues use Isaac Sim 3 as a desktop/Omniverse application. The
  passing environment allowlist for this milestone is Isaac Sim `2023.1.*`.
  Another 3-era version requires an explicit revision of this design rather
  than a runtime override.
- The exact installed patch version is not assumed. An environment probe records
  the executable path, semantic version, Python/Kit version, renderer, and
  relevant capabilities before an Isaac run starts.
- No Isaac Sim executable or container has been found on this workstation yet.
  The design therefore requires offline contract checks and an explicit,
  non-passing Isaac skip until the company environment is available.
- The supplied archive is `../URDF/URDF_robot_1.tar.gz` relative to the RPent
  repository. Its top-level robot file is `URDF_robot_1/robot.urdf`.
- The archive contains chassis, wheel, caster, waist/connector, dual six-joint
  arm, sensor, camera, and CTAG-style gripper subtrees.

Generated extractions, imported USD, meshes, logs, and screenshots stay below
`.artifacts/lynsense-isaac/`, which is already ignored by this repository.
Company assets are not committed or redistributed.

## Non-Goals

- Do not remove, bypass, or regress the Webots implementation.
- Do not connect to the real robot, robot SSH state, ROS Domain 3, the robot
  workspace, or `.env.lynsense`.
- Do not add ROS or `py_trees` to the first Isaac milestone.
- Do not add RPent policy, perception, localization, collision avoidance,
  grasping, navigation, or the full box task.
- Do not claim LAN viewer or recorded-video parity with Webots.
- Do not track or copy company URDF, STL, or generated USD assets into the
  repository.

## Selected Approach

Use the company Isaac Sim 3-era desktop environment as the primary target. This
prioritizes compatibility with the company's robot model workflow, importer
behavior, and human visual review over immediate workstation CI.

Two alternatives are intentionally deferred:

- A latest-Isaac container baseline would be easier to automate on this
  workstation, but it can differ from the company's importer and articulation
  behavior and does not answer the immediate fidelity question.
- A dual-version compatibility layer would broaden coverage, but it adds an
  API surface before the first model has been shown to import and actuate
  correctly.

Once the company-aligned path is stable, a container or dual-version CI gate
can be added as a separate milestone.

## Architecture

```text
company URDF archive
  -> verified extraction under ignored Isaac artifacts
  -> offline URDF contract inspection
  -> Isaac Sim 3-era URDF importer
  -> generated local USD
  -> articulation joint and state inspection
  -> one bounded dual-arm joint action
  -> screenshots, logs, and machine-readable summary
```

Implementation lives under `robots/lynsense/isaac/`, separate from the existing
Webots code under `robots/lynsense/simulation/`. Optional Isaac APIs remain
behind the probe entry point so ordinary imports, robot discovery, and offline
tests continue without Isaac installed.

## Components

### Environment Resolver

Requires an explicit operator-supplied Isaac executable path for a passing run.
It may report discovered executables as diagnostics, but never selects one
automatically. It writes `environment.json` containing the resolved path,
version information, run mode, renderer, and probe result. It never writes
credentials, environment secrets, license text, or other sensitive values.
A version outside the `2023.1.*` allowlist is an environment failure.

### Archive Stager

Verifies the archive exists, checks that `URDF_robot_1/robot.urdf` is present,
extracts into an ignored run directory, and records file digests for the URDF
and referenced meshes. It reports a missing, malformed, or partial archive as
an explicit failure.

Extraction accepts only relative directory and regular-file entries. It rejects
absolute paths, parent traversal, symbolic links, hard links, device nodes,
more than 100,000 entries, or more than 2 GiB of uncompressed content. Every
resolved output path must remain below the new run directory.

### URDF Contract Inspector

Parses the extracted URDF without Isaac and verifies the joint inventory,
limits, mesh references, and expected gripper subtree structure before an
Isaac process starts. It resolves `package://` references against the extracted
archive layout rather than requiring ROS package installation.

### Isaac Probe Runner

Launches the selected company Isaac environment, imports the URDF to USD,
loads the generated USD as an articulation, reads joint names and positions,
applies the bounded action, captures rendering evidence, and writes a summary.
The runner keeps Isaac dispatch on one fixed main thread and performs explicit
shutdown cleanup.

### Artifact Store

Creates one timestamped run directory below `.artifacts/lynsense-isaac/`.
All extracted inputs, imported USD, logs, screenshots, `environment.json`, and
the final summary are written there. The command output reports the absolute
run directory.

## URDF And USD Contract

The importer must expose the following independently controllable joints:

```text
connector_joint
left_joint1 through left_joint6
right_joint1 through right_joint6
```

The left and right gripper subtrees must remain structurally visible after
import. Their CTAG-style coupled and mimic joints are checked as a subtree, not
as independently drivable joints. Every importer warning, skipped link, fixed
link reduction, mimic-joint decision, and material or mesh substitution is
recorded in the run summary.

Warnings are classified before the probe continues:

- Blocking: a required joint or limit is missing or invalid, a referenced mesh
  is unreadable or substituted, a gripper link, coupling, camera, or other
  sensor path is dropped, or the importer reports an uncategorized severity.
- Recorded: material, color, texture, fixed-link reduction, or naming changes
  that do not remove a required joint, sensor path, or gripper subtree.

Each event is saved with severity, source, message, and the resulting decision.
The runner stops on a blocking event or an event whose severity it cannot map.

The URDF contract inspector resolves all `package://` mesh references to files
in the extracted archive. A missing mesh is a failure, even if Isaac can open
the model with that mesh omitted.

## Scene And Articulation Setup

The URDF root is `base_link`. For this model-fidelity probe, `base_link` is
fixed to the world so wheel contact and chassis locomotion cannot mask an arm
import problem. Gravity uses the Isaac default of `-9.81 m/s^2` along the world
Z axis. A ground plane may be added for visual context, but task collision and
navigation are out of scope.

The base transform may move no more than `0.001 m` in translation or `0.5 deg`
in rotation during the action. Exceeding either bound, or observing a non-finite
transform, fails the probe.

## Minimal Action

The first action is deliberately small and symmetric:

1. Read the initial positions of `left_joint1` and `right_joint1`.
2. Set each target to `initial + 0.1 rad`; fail before actuation if either
   target is outside the imported joint limits.
3. Configure both joints with the same recorded position-drive configuration,
   the URDF effort limit of `200 N*m`, and a probe speed limit of
   `0.314 rad/s`. Imported damping and friction are retained; gains are fixed
   before the run and recorded rather than tuned while it executes.
4. Apply both position targets simultaneously.
5. Step the simulator for at most 10 simulated seconds and check finite joint
   positions, velocities, and the base transform at every physics step.
6. Require each trajectory to progress at least `0.05 rad` toward its target.
7. Require the final `0.5` simulated second of both trajectories to remain
   within `0.02 rad` of target with joint speed no greater than `0.05 rad/s`.
8. Fail if either joint overshoots its target by more than `0.02 rad`, fails to
   settle, or cannot complete within the budget.

A non-finite value, base-motion violation, or shutdown exception fails the
probe. This action does not attempt to reproduce `dualjo:joints_br`, navigation,
or grasping.

## Rendering Evidence

Every probe attempt writes all screenshot stages that it reaches:

```text
scene-initial.png
scene-action.png
scene-final.png
```

Automated checks verify that the files exist, are readable, and are not blank.
Human review in the company Isaac desktop environment decides whether the
visible chassis, arms, hands, cameras, and grippers match the company robot.
Automated pixel checks cannot certify company-model fidelity.

The technical gate and visual gate are recorded separately in
`visual-review.json`. The file records the reviewed screenshots, visible robot
subsystems, accepted or rejected status, reviewer identifier, and timestamp.
Both gates must be accepted before the milestone is closed.

## Error Handling

The runner reports distinct failures for:

- Isaac is absent, cannot start, or reports an unsupported version.
- The archive is missing, malformed, or incomplete.
- A URDF file or referenced mesh is missing.
- URDF-to-USD import fails or emits a blocking warning.
- A required articulation joint is absent.
- A gripper subtree is dropped.
- State contains a non-finite number.
- The bounded action times out.
- Rendering is blank or a screenshot cannot be written.
- Isaac shutdown raises an exception.

Exit status and the machine-readable summary distinguish an explicit Isaac skip
because no runtime is available from a failed Isaac run.

## Testing And Acceptance

Offline tests cover archive extraction safety, URDF contract validation,
`package://` path handling, warning classification, action-target selection,
drive-limit evaluation, trajectory settling and overshoot evaluation, base
motion limits, technical summary generation, and the `visual-review.json`
schema without requiring Isaac.

The Isaac gate is skipped explicitly when no compatible runtime is present. A
skip is never counted as a pass.

The milestone is accepted only when all of the following hold:

- The selected environment is explicitly supplied, recorded, and reports an
  Isaac Sim `2023.1.*` version.
- The offline URDF inventory passes.
- USD generation succeeds.
- All required articulation joints are visible with finite state.
- The left and right gripper subtrees are preserved.
- Both selected first-arm joints move at least `0.05 rad` toward target, settle
  within `0.02 rad`, remain below `0.05 rad/s` for the final `0.5` simulated
  second, and do not overshoot by more than `0.02 rad` within 10 simulated
  seconds.
- The fixed `base_link` remains within its translation and rotation bounds.
- All three screenshots are nonblank.
- `visual-review.json` records human acceptance of company-model fidelity.
- Generated company assets remain untracked.
- Existing Webots suites still pass when shared repository code changes.

## Follow-Up Milestones

After both the technical and visual gates pass:

1. Migrate a bounded navigation slice.
2. Migrate the full single-box sequence.
3. Reintroduce the RPent backend and policy execution path.
4. Add viewer and video capture once model fidelity is stable.
5. Consider a company-approved Isaac container or dual-version CI gate.
