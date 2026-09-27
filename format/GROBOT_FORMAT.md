# The robot formats: `.grobot.json` (spec), `.grobot` (compiled), `.gpolicy` (trained)

`status: v0 — 2026-09-27` (founder real-time: "we need to get goldenband rigged up with real robot
data from industrial data sheets" + "and the rl animations pipeline")

HQ-SPEC-SIM-100 §3 says a hardware-bound skeleton carries "actuator metadata: torque limits,
velocity limits", and SHANKPIT's `RAGDOLL_ORIENTATION_NORTHSTAR.md` says what that means in
practice: "the real NUMBERS CAD/motor-datasheet work produces (per-link mass, inertia,
torque/velocity curves)". These formats hold exactly those numbers, each one traceable to the
manufacturer document it came from.

Same split-by-consumer rule as `.gband`: a human/tooling-facing JSON spec, and a fixed binary the C
runtime reads with a bounded parser.

## `<name>.grobot.json` — the spec (source of truth, in git under `robots/`)

URDF-shaped on purpose (per-joint origin `xyz`/`rpy`, per-link `<inertial>`), because that is the
form manufacturers actually publish machine-readable data in. SI units throughout (m, kg, kg·m²,
rad, rad/s, N·m).

| Field | Meaning |
|---|---|
| `grobot_version` | `1` |
| `name` | short id, < 32 bytes (`ur5e`) |
| `manufacturer`, `model` | human-readable |
| `sources[]` | `{id, title, url, commit?, retrieved?, license?}` — every link/joint cites one `id` |
| `notes[]` | provenance caveats, verbatim-honest (e.g. data the manufacturer doesn't publish) |
| `base_link` | the link fixed to the world (never simulated) |
| `links[]` | `{name, mass, com, inertia_rpy, inertia{ixx,ixy,ixz,iyy,iyz,izz}, source, note?}` — `com` in the link frame; the tensor is about the COM, in the link frame rotated by `inertia_rpy` (URDF `<inertial><origin>` semantics) |
| `joints[]` | `{name, type: revolute\|continuous, parent, child, origin_xyz, origin_rpy, axis, lower, upper, velocity, effort, damping, source, note?}` — must be listed parent-before-child; `velocity` = datasheet max joint speed, `effort` = datasheet max joint torque |
| `tcp` | `{link, xyz}` — tool center point, for end-effector reward terms |

`gbtool robot compile` validates: every source id resolves, masses > 0, every inertia tensor is
positive definite **and physically realizable** (triangle inequality on principal moments), joint
limits ordered, speed/torque limits present, the tree is a tree.

### Where the committed specs come from

`robots/ur3e.grobot.json`, `ur5e`, `ur10e` are **generated, not hand-typed**:
`gbtool robot import-ur` reads Universal Robots' own description files
(`github.com/UniversalRobots/Universal_Robots_ROS2_Description`, `config/<model>/`
`default_kinematics.yaml` / `physical_parameters.yaml` / `joint_limits.yaml`, pinned at commit
`89bbe795f38a7ab00fb66fe8831dfff79dc99edf`, BSD-3-Clause — copies and the license are vendored in
`robots/sources/universal_robots/`). UR's own `joint_limits.yaml` cites its sources for the limits:
the e-Series User Manual (position/velocity) and UR's "Max. joint torques" support article
(effort). A Go test regenerates all three specs from the vendored YAML and fails if the committed
JSON has drifted.

Verified against independent references (tests, not claims):
- **Kinematics** — zero-pose flange position from our FK equals UR's published DH parameters
  (`a2+a3, -(d4+d6), d1-d5`) to 6·10⁻¹¹ m (Go `TestUR5eZeroPoseMatchesDH`, C `test_grobot`).
- **Dynamics** — gbtool's inverse dynamics (RNEA) passes both a power-balance test and a
  per-joint Euler–Lagrange test against numerically-differentiated energy, on all three arms;
  the grb physics sim's servos hold a loaded UR5e pose with the same torques RNEA predicts
  (40.4 vs 40.09 N·m shoulder, within 1%).

What the manufacturer does **not** publish, and is therefore not invented here: joint friction /
damping (0), acceleration limits (unconstrained), base inertia (unit placeholder; the base is fixed
and never simulated), gear backlash, motor torque–speed curves (the effort limit is flat up to the
speed limit — a v0 simplification). UR3e's `wrist_3_joint` is `continuous` because UR marks it
with no position limit (infinite rotation), not because a value is missing.

## `<name>.grobot` — compiled binary (read by `src/grobot.c`)

Little-endian. Header 104 bytes, then `joint_count` records of 280 bytes.

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | magic `"GRBT"` |
| 4 | 4 | `version` (u32) = 1 |
| 8 | 4 | `joint_count` (u32), ≤ 64 |
| 12 | 4 | `tcp_joint` (i32) — joint whose child link carries the TCP |
| 16 | 24 | `tcp_xyz` (3×f64) |
| 40 | 32 | `name` (ASCII, NUL-padded) |
| 72 | 32 | `spec_hash` — sha256 of the `.grobot.json` bytes it was compiled from |

Per-joint record:

| Offset | Size | Field |
|---|---|---|
| 0 | 32 | joint `name` |
| 32 | 32 | `child` link name |
| 64 | 4 | `parent` (i32) — index of the joint whose child is this joint's parent link, −1 = base; always < own index |
| 68 | 4 | `type` (u32) — 1 revolute, 2 continuous |
| 72 | 24 | `origin_xyz` (3×f64), parent link frame |
| 96 | 32 | `origin_quat` (4×f64, x,y,z,w) |
| 128 | 24 | `axis` (3×f64, unit, joint frame) |
| 152 | 48 | `lower, upper, velocity, effort, damping, mass` (6×f64) |
| 200 | 24 | `com` (3×f64), child link frame |
| 224 | 32 | `principal_quat` (4×f64) — child link frame → principal inertia frame |
| 256 | 24 | `principal_moments` (3×f64) |

f64 (not f32 like `.gband`) because wrist inertias are ~10⁻⁴ kg·m² and the physics is double
precision. The compiler diagonalizes each published inertia tensor (Jacobi) so the runtime rigid
body stores a diagonal inertia in its principal frame.

`gbtool robot compile` also writes `<name>.gskel` — the rest-pose kinematic tree as an ordinary
GOLDEN BAND skeleton (joint 0 = base link, joint i+1 = robot joint i, named after the joint), so
every existing consumer (gpose, SHANKPIT, NOCK's animation repository) can load a robot rig.

## Robot motion clips

Ordinary `.gband` clips whose channels are `<joint>.angle` (radians). `gbtool robot bake-motion`
authors them from keyframes with minimum-jerk interpolation (Flash & Hogan 1985: zero velocity and
acceleration at every keyframe); `skeleton_hash` is the sha256 of the robot's compiled `.gskel`.
`gbtool robot check` is the SIM-100 §3 feasibility pass: exact inverse dynamics on every tick,
position/speed/torque violations counted per joint, exit status 2 when infeasible, `--annotate`
writes the peak speed/torque into the manifest's `safety` block.

## `<name>.gpolicy` — trained policy (written by `tools/gbtrain`)

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | magic `"GPOL"` |
| 4 | 20 | `version, obs_dim, act_dim, harmonics, reserved` (5×u32) |
| 24 | 32 | robot `spec_hash` |
| 56 | 32 | reference clip `content_hash` |
| 88 | 32 | `reward_id` — sha256(robot spec hash ‖ clip content hash ‖ canonical reward profile ‖ feature set): the compiled reward function's identity |
| 120 | `act_dim·obs_dim·8` | linear policy weights (f64, row-major) |

The sidecar `<name>.gpolicy.json` carries the policy hash, the full reward profile text, the
trainer settings/seed, and the frozen-eval report (baseline vs trained, every reward term). Every
field that went into the artifact is hashed into it — SIM-100 §6's "the hash is what gets
approved" in miniature; the full deployment-bundle gate is not built.
