# GOLDENBAND — CLAUDE.md

## What this is

GOLDEN BAND, the animation layer from `HQ-SPEC-SIM-100`: one canonical motion asset format
(`.gband`), consumed by a game runtime, a physics reward compiler, and (long-horizon, out of
scope here) hardware deployment. This repo builds only HQ-SPEC-SIM-100 §8 build step 1: the
`.gband` format itself, a C sampler, and Go pipeline tools. Steps 2-5 (SHANKPIT integration,
reward compiler, training backbone, dual deployment) are separate, larger, not-yet-started
backlog items — see `EMILY/BACKLOG.md` SECTION 144.

Standalone repo, not in the monorepo's `go.work` — mirrors SHANKPIT/PITVIPER/EmilyOS's own
convention. Deliberate: HQ-SPEC-SIM-100 §2 requires "no engine dependency in the asset" —
GOLDEN BAND assets know nothing about SHANKPIT, Unreal, or Godot.

## Repo map

```
format/GBAND_FORMAT.md   — the .gband binary layout + manifest schema, full spec
src/gband.h, gband.c     — the runtime C sampler (gb_init/gb_sample/gb_blend/gb_verify).
                            Deliberately tiny: HQ-SPEC-SIM-100 §8's own acceptance test is
                            "a hundred line parser" — this file is ~90.
src/sha256.h             — self-contained SHA-256 (content-addressing only, no HMAC needed
                            here), trimmed from REDGARDEN's packages/common/hmac_sha256.h
src/gseq.h, gseq.c       — real animation STITCHING (founder real-time: "make sure we can
                            stitch animations together like James Bond walk turn raise gun
                            shoot") -- chains several different .gband clips into one ordered
                            GSeq with a real nlerp crossfade at each transition, resolving
                            channel names against a GSkel so mismatched clips fall back to that
                            joint's own real rest pose. The one real, minimal channel-name-aware
                            layer above gband.c's own name-blind sampler -- see gseq.h's own
                            header comment for the full design and what it deliberately doesn't
                            cover yet (no authored .gseq asset format/NOCK UI, single blend
                            duration per whole sequence).
src/gpose.h, gpose.c     — real general forward kinematics + mesh skinning for an arbitrary
                            joint count (founder real-time, 2026-09-17: "we want animations in
                            game... build the affordances to start building the animations into
                            the games"). Closes a real gap: SHANKPIT's existing
                            gband_mesh_rig.c hardcodes a 5-joint armature with manually-indexed
                            channels (real, working for Tyler's own hand-authored rig, but not
                            reusable for any other skeleton) -- gpose.c is the general N-joint
                            replacement, self-contained column-major float[16] math with no
                            engine Mat4 dependency, consumed by taking a pose (e.g. from
                            gseq_player_sample_pose) and producing skin matrices, then a flat
                            pos+normal triangle list ready for a consuming engine's own render
                            bridge (vendored per-engine the same way gband_mesh_rig.c itself
                            was already ported from REDGARDEN -- see gpose.h's own header
                            comment).
tools/gbtool/            — Go CLI: BVH import, glTF import (quaternion anim + mesh + skeleton),
                            hash, validate (standalone Go module)
tests/test_sha256.c      — sha256.h against NIST FIPS 180-4 vectors
tests/test_gband.c       — gband.c against a synthetic in-test fixture
scripts/build_and_test.sh — builds + runs everything (C tests + Go tests)
```

## Build & test

```bash
bash scripts/build_and_test.sh
```

## `gbtool` usage

```bash
cd tools/gbtool
GOWORK=off go run . import --bvh <file.bvh> --out <name> [--kind mocap|human|generative] [--who "<text>"]
GOWORK=off go run . import --gltf <file.glb|.gltf> --out <name> [--tick-rate <n>] [--kind ...] [--who "<text>"]
GOWORK=off go run . bake-ease --out <name> --channel <name> --ticks <n> [--tick-rate <n>]
GOWORK=off go run . hash <name>
GOWORK=off go run . validate <name>
```

`import --gltf` (2026-09-16, founder real-time: "let's start iterating towards nock tools
modeler (blender) and golden band we need to be able to import quaternion animations into nock
golden band") reads a real glTF 2.0 file — Blender's own native "glTF Binary (.glb)" / "glTF
Separate" export format, quaternion-based — and writes out whichever of `<name>.gskel`
(skeleton, real quaternion rest poses), `<name>.gmesh` (skinned geometry), and `<name>.gband` +
`.gband.json` (animation, real `<joint>.qx/.qy/.qz/.qw` quaternion channels — see
`format/GBAND_FORMAT.md`'s own "Quaternion channels" section) the source file actually contains.
Hand-rolled stdlib-only glTF reader (`tools/gbtool/gltf.go`), no vendored dependency. v0 scope:
first skin/mesh/animation only, LINEAR sampler interpolation only, nlerp (not slerp) for
quaternion resampling — all documented in `import_gltf.go`'s own header comment and the format
doc, not silently assumed. This is the real, direct Blender unlock for GOLDEN BAND: no custom
Blender plugin needed for this pass, just glTF's own standard export.

`bake-ease` (2026-08-26) synthesizes a real single-channel smoothstep ease-in/ease-out curve
(0.0->1.0, monotonic, zero velocity at both ends) instead of importing one from a BVH — for
callers that need a baked timing/blend curve rather than a skeletal pose (e.g. REDGARDEN's
Abraham fireball windup, `assets/anim/rotation_ease.gband`, consumed as a facing-angle blend
weight, not a pose). `skeleton_hash` is the zeroed sentinel, same as `import`'s own "no skeleton
resolution yet" convention — flagged as a real, deliberate non-skeletal repurposing of the
sampler, not a skeleton retargeting pass.

`<name>` resolves to `<name>.gband` (binary, runtime-consumed) + `<name>.gband.json` (manifest,
tooling-consumed).

## What v0 (this pass) does not cover

- A Blender plugin / "NOCK tools modeler" UI (the founder's own longer-horizon direction this
  glTF importer is a first real step towards) — not started; glTF import is the pipeline half,
  a NOCK-hosted authoring/browsing surface is a separate, not-yet-scoped next phase.
- An "animation repository" (NOCK-hosted storage/browse UI for `.gband`/`.gskel`/`.gmesh`
  assets, mirroring NOCK's own texture library) — named as real, near-term direction, not built.
- Multi-skin/multi-mesh/multi-clip glTF files in one import run (first of each only, see
  `format/GBAND_FORMAT.md`'s own gap list).
- True slerp for quaternion channel blending (nlerp only, both at import and in `gb_blend`).
- Retargeting maps, hardware feasibility passes.
- The reward compiler, training backbone, SHANKPIT integration (build steps 2-5).
- `golden` promotion / Apples wiring (HQ-SPEC-SIM-100 §3's `ApplePublished` promotion events).

## SAGA claim

This repo's existence is `SIM-100.BEH-2`'s citation point (`reality_binding: running`) — see
`EMILY/docs/hq-specs/HQ-SPEC-SIM-100-springerton-seam-golden-band.md` and
`EMILY/docs/hq-specs/SAGA_SCHEMA.md`.

## Founder Real-Time Direction

Whenever the founder gives real-time direction — a new ask, a correction, a "can we also..." —
route it through `emily observe -s info "Founder real-time: <summary>"` first, even if it isn't
this repo's usual domain, then sprint-plan it into `EMILY/BACKLOG.md` (`emily backlog curate`,
scoped into a real SECTION/sub-item, not just a one-line log), and only then implement. See
`EMILY/docs/THE_EMILY_WAY.md` Principle 18 ("Pave the Cow Paths").

## Frame-Break Reframing

Founder-sourced prompting technique (REDGARDEN/NORTHSTAR.md §28, full origin in
REDGARDEN/docs2/MULTI_AGENT_RD_RESEARCH_NOTES.md §5): given a request, name the underlying
structural/systemic pattern it's one instance of — one level of abstraction up — as an added
lens during planning/triage/judgment calls. Use it to spot the general case behind a specific
ask. It augments judgment, it does not replace doing the work: direct, concrete execution of
the literal task asked for still happens every time.

## Commit Protocol (standing instruction)

Always commit and push completed work immediately — don't wait to be asked. This is the default
for every repo in this monorepo.

Every commit — human-written or produced by automated code paths (git-commit helpers in emily-agent, emily.cli, IDUNA handlers, etc.) — must carry the active `emily session` fingerprint as a `session: <tag>` trailer (blank line, then the trailer). This was silently missing from several independently-implemented automated commit helpers across the monorepo until an audit on 2026-08-10 (founder, real-time: "where in the fuck is my llm session id anywhere"). If you add a new automated git-commit code path anywhere, wire in the session tag the same way — don't assume an existing helper already does it.
