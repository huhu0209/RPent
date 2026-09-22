# Lynsense Isaac Sim Probe

## Scope

This probe targets company Isaac Sim `2023.1.*` only. The Webots baseline
remains the verified baseline for Lynsense. This documentation does not claim
support for ROS, `py_trees`, RPent policy execution, a real robot, ROS Domain
3, robot SSH state, the robot workspace, or `.env.lynsense`.

The probe is an offline archive inspection followed by an explicitly selected
company Isaac runtime. Generated company assets are local evidence and must
not be committed.

## Offline inspection and explicit skip

Run the host inspection without an Isaac executable when checking the archive
and URDF contracts:

```bash
PYTHONPATH=. .venv/bin/python robots/lynsense/isaac/probe.py \
  --archive ../URDF/URDF_robot_1.tar.gz
```

The host command creates a run below `.artifacts/lynsense-isaac/<run-id>` and
writes `technical-summary.json`. When `--isaac-python` is absent, the command
records `outcome="skipped"`, `is_pass=false`, and exits with `77`. Exit `77`
is an explicit skip, not a pass. A skipped run must not create
`visual-review.json` or claim milestone acceptance.

## Company Isaac run

The operator must supply the executable from the company Isaac Sim
installation. The supported runtime is Isaac Sim `2023.1.*`:

```bash
PYTHONPATH=. .venv/bin/python robots/lynsense/isaac/probe.py \
  --archive ../URDF/URDF_robot_1.tar.gz \
  --isaac-python /company/path/to/isaac-sim/python.sh
```

The archive is expected to be `URDF_robot_1.tar.gz` and must contain the
company robot URDF and its referenced meshes. The run must pass the importer,
USD structure, articulation, joint-limit, fixed-base action, and render
evidence contracts before human review.

## Evidence

A passing run contains these machine-produced artifacts in its run directory:

- `environment.json`
- `robot_1.usd`
- `technical-summary.json`
- `urdf-inventory.json`
- `isaac.log`
- `scene-initial.png`
- `scene-action.png`
- `scene-final.png`

## Human visual review

Automated nonblank checks establish only that screenshots contain decoded,
spatially varying pixels. They cannot prove company-model fidelity, correct
subsystem appearance, or acceptable camera framing. A company reviewer must
inspect all three screenshots and record specific acceptance notes:

```bash
PYTHONPATH=. .venv/bin/python robots/lynsense/isaac/probe.py \
  --visual-review .artifacts/lynsense-isaac/<run-id> \
  --status accepted \
  --reviewer "<company reviewer identity>" \
  --notes "<specific visual acceptance notes>"
```

This creates `visual-review.json` only after the required screenshots and
review fields validate.

## Failure meanings

- Unsupported version: the runtime is not Isaac Sim `2023.1.*`.
- Absent explicit executable: no company runtime was selected; this is the
  explicit skip with exit `77`.
- Archive, URDF, or mesh failure: the archive is missing, unsafe, incomplete,
  malformed, or exceeds its extraction limit.
- USD/import failure: conversion or importer output is missing or contains a
  blocking importer event.
- Missing joint/subtree: the generated robot lacks required articulation,
  links, or gripper structure.
- Non-finite state: runtime observations or artifact JSON contain invalid
  numeric values.
- Action timeout: the fixed-base joint or base-motion probe did not complete
  within its contract.
- Blank render: a required screenshot is missing, invalid, too small, or has
  insufficient pixel variation.
- Shutdown exception: runtime cleanup failed; it remains a failed technical
  run unless all required cleanup and evidence contracts pass.

## Final milestone gate

After the technical run passes and a company reviewer records an accepted
visual review, verify the complete milestone:

```bash
PYTHONPATH=. .venv/bin/python robots/lynsense/isaac/probe.py \
  --verify-milestone .artifacts/lynsense-isaac/<run-id>
```

Exit `0` from this command is the only machine-readable milestone acceptance.
An offline pass, an explicit skip, or a nonblank automated render is not
milestone acceptance. The company Isaac run and human visual review gates
remain open until this command returns `0`.

## Limits

This minimal loop does not provide navigation, the full box task, viewer/video
parity, calibrated physics, or real-robot readiness. It does not authorize
ROS Domain 3 operation, SSH access, robot workspace changes, or deployment.
Generated company assets, screenshots, logs, and run directories must not be
committed.
