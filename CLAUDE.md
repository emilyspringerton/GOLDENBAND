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
tools/gbtool/            — Go CLI: BVH import, hash, validate (standalone Go module)
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
GOWORK=off go run . hash <name>
GOWORK=off go run . validate <name>
```

`<name>` resolves to `<name>.gband` (binary, runtime-consumed) + `<name>.gband.json` (manifest,
tooling-consumed).

## What v0 (this pass) does not cover

- glTF import (BVH only — glTF's skinning/animation extensions are a real, separate
  undertaking, not silently skipped, see `format/GBAND_FORMAT.md`'s own gap list).
- Skeleton assets, retargeting maps, hardware feasibility passes.
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
