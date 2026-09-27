# GOLDEN BAND

`.gband` — the canonical motion-asset format from `HQ-SPEC-SIM-100`. This repo builds the
format itself, a tiny C runtime sampler, and the Go/Blender pipeline tools that get a real
skinned character + animations into it. See `CLAUDE.md` for the full technical map;
`GoblinFoxDragon/docs2/GOLDENBAND_INTEGRATION_NORTHSTAR.md` for the design rationale behind the
choices below.

## Available rigs

Two real armatures exist so far, both built the same way (a `create_*_armature.py` script that
constructs the Armature procedurally in Blender's own Python API, not hand-modeled) and both
export through the exact same `.gskel`/`.gmesh`/`.gband` pipeline below — the format has no
hardcoded bone count/hierarchy (`GSKEL_MAX_JOINTS` is 128, read from the file), so a new rig is
never a format change, just a new joint table:

| Rig | Bones | Body plan |
|---|---|---|
| `TylerRig` | 5 | Humanoid: `Hips → Spine → {Head, L_Arm, R_Arm}` |
| `AntRig` (S202-28) | 11 | Insect: `Abdomen → Thorax → Head`, 6 single-bone legs (front/mid/rear pairs), 2 antennae — horizontal/low-slung, not upright |

`AntRig` is the armature only (this repo's own stated scope) — whether it becomes a real
REDGARDEN roster hero is a separate, later decision, not assumed here. Build it the same way as
`TylerRig` below, swapping in `tools/blender_export/create_ant_armature.py` /
`.github/workflows/blender-tools.yml`'s `build-ant-armature` job.

## Attaching a model to the rig, in Blender

You need three things before you have something exportable: a rig (armature), a model (mesh),
and the model skinned to the rig. If you already have a rig-provided skeleton and a model and
just need to attach them:

1. **Open the rig.** `TylerRig.blend` (repo root, or the identical copy at
   `tools/blender_export/TylerRig.blend`) already has the canonical 5-bone armature
   (`Hips → Spine → {Head, L_Arm, R_Arm}`) that the engine's box-rig and mesh-rig both expect.
   If you'd rather build it fresh in your own file instead of opening this one: Scripting tab →
   open `tools/blender_export/create_tyler_armature.py` → Run Script. Either way you end up with
   one Armature object named `TylerRig`. (For `AntRig` instead, same steps with
   `create_ant_armature.py` — see "Available rigs" above.)
2. **Bring your model into the same scene** and position/scale it around the armature in
   Blender's normal (Z-up) view — the armature is already built in Blender's own coordinate
   space, so nothing needs pre-conversion at this step.
3. **Skin it.** Select your mesh, then shift-select the armature (armature must be the last/
   active selection), `Ctrl+P` → **"With Automatic Weights"**. Blender computes the vertex
   weights for you — no manual weight painting required for a first pass. (Manual touch-ups in
   Weight Paint mode later are fine if a joint looks wrong — same standard Blender workflow
   either way.) This is the actual "attach" step: it parents the mesh to the armature and creates
   an Armature modifier + vertex groups named to match the bone names.
4. **(Optional) Author or apply animations** — Idle, Walk, whatever you need — as normal
   keyframed Actions on the armature (Action Editor/NLA). Not required just to attach model to
   rig, but the export step below picks up every Action it finds.
5. **Save as an uncompressed `.blend`.** File → Save As, uncheck "Compress" in the file browser's
   options panel. The CI pipeline's portable Blender build can't open zstd-compressed saves
   (real gap found 2026-08-05, not a hypothetical) — this is the one save-time setting that
   actually matters for the next step to work.

## Getting `.gskel` / `.gmesh` / `.gband` out of that file

Two ways to run the actual export — pick whichever fits your setup:

- **No local Blender scripting needed (recommended if you're disk/tool-constrained):** upload
  your saved `.blend` to `incoming/` in this repo via GitHub's website (repo page → `incoming/` →
  "Add file" → "Upload files") and commit. That push automatically runs
  `.github/workflows/blender-tools.yml`'s `export-incoming` job — `export_gband_rig.py` against
  every `.blend`, `gbtool import` against every `.bvh` — and uploads the results
  (`.gskel`/`.gmesh`/one `.gband` per Action) as a downloadable Actions artifact. `incoming/`
  already has its own short README with the same instructions.
- **Local Blender + scripting:** with your mesh selected as the *active* object (its Armature
  modifier pointing at `TylerRig`), run `tools/blender_export/export_gband_rig.py` from the
  Scripting tab, or headless: `blender --background yourfile.blend --python
  tools/blender_export/export_gband_rig.py -- <out_dir>`. Writes `<mesh_name>.gskel` +
  `<mesh_name>.gmesh`, plus one `<action_name>.gband` per Action found on the armature.

`export_gband_rig.py` was verified 2026-08-05 against a real Blender 5.2 install (mesh, bone
rest-transform, and animation export all checked value-by-value against hand-computed
expectations, not assumed) — see the script's own header comment for the three real bugs that
verification pass found and fixed.

Don't need a full skinned mesh, just a mocap animation on the existing box-rig? `gbtool import
--bvh <file.bvh> --out <name>` (in `tools/gbtool/`) — no Blender needed at all for that path, see
`CLAUDE.md`'s `gbtool` usage.

## Rigid body physics, real industrial robots, and the RL animation pipeline

Added 2026-09-27 (founder real-time: "upgrade shankpit and nock to formal rigid body physics we
need to get goldenband rigged up with real robot data from industrial data sheets" + "and the rl
animations pipeline"). Three new pieces of kit, all C runtime + Go tooling, no dependencies:

- **`src/grb.{h,c}` — formal rigid body physics.** Real 6-DOF bodies (quaternion orientation,
  full Euler rigid-body equation with the gyroscopic term, diagonalized inertia tensors), XPBD
  joints (Müller et al. 2020: hinge with limits, ball with swing-cone + twist limits, fixed),
  effort- and speed-limited servo motors solved implicitly at 1024 Hz substep rate, and
  sphere/capsule/box ground contact with Coulomb friction + restitution. Deterministic
  (bit-identical reruns). `tests/test_grb.c` checks it against closed-form physics: pendulum
  period (0.02% off), free fall, angular-momentum conservation + the Dzhanibekov flip, Coulomb
  sliding distance, restitution rebound height, joint limits, motor effort/speed clamps.
  **Limits:** ground planes only (no body-vs-body collision yet), no broadphase/sleeping/CCD.
- **Datasheet robots — `robots/*.grobot.json` → `.grobot`.** Universal Robots **UR3e, UR5e,
  UR10e**, generated by `gbtool robot import-ur` from UR's own published description data
  (link masses, centers of mass, full inertia tensors, joint frames, and the joint torque/speed/
  position limits UR cites from its User Manual and "Max. joint torques" article), every number
  source-cited; see `format/GROBOT_FORMAT.md`. Verified: our kinematics reproduce UR's DH
  parameters to 6e-11 m; our inverse dynamics (RNEA) pass power-balance and Euler–Lagrange
  checks; the physics sim holds a loaded UR5e pose with the same joint torques RNEA predicts
  (within 1%). **Limits:** fixed-base arms only; no link collision geometry; manufacturers don't
  publish friction, acceleration limits or torque–speed curves, so those aren't modeled.
- **The RL animation pipeline — `src/grl.{h,c}` + `tools/gbtrain`.** HQ-SPEC-SIM-100 §4's
  reward compiler (pose / velocity / end-effector tracking + energy, smoothness and effort-limit
  regularizers, content-hashed reward id) and a v0 training backbone (cross-entropy method, linear
  policy, domain-randomized link masses, frozen held-out eval). It trains a policy that makes the
  physical robot honor an authored clip, and writes back the motion physics actually achieved as
  a new `.gband`. Committed example (`assets/robots/`, regenerate with
  `scripts/robot_assets.sh`): a UR5e on soft 5 Hz servos, frozen-eval return **0.28 → 0.90**,
  end-effector tracking **0.19 → 0.93**, zero effort-limit saturation, ~65 s on one CPU core.
  **Limits:** a linear policy on hand-built features, CPU single-thread, fixed-base robots only —
  not yet a humanoid/ragdoll get-up policy (that needs contact-rich balance, SIM-100 §8 step 5),
  and no GPU backbone (Isaac/MJX adapter not built).

```bash
cd tools/gbtool
GOWORK=off go run . robot compile --robot ../../robots/ur5e.grobot.json --out /tmp/ur5e
GOWORK=off go run . robot bake-motion --robot ../../robots/ur5e.grobot.json --out /tmp/wave \
  --keyframes "0:0,-90,90,-90,-90,0;1.5:60,-60,40,-70,-90,30;3:0,-90,90,-90,-90,0"
GOWORK=off go run . robot check --robot ../../robots/ur5e.grobot.json --clip /tmp/wave   # datasheet feasibility
cd ../.. && gcc -O2 -o /tmp/gbtrain tools/gbtrain/gbtrain.c src/grl.c src/grobot.c src/grb.c src/gband.c -lm
/tmp/gbtrain --robot /tmp/ur5e.grobot --clip /tmp/wave --out /tmp/wave_policy
```

## Repo map, build & test

See `CLAUDE.md`.
