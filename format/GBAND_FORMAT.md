# The `.gband` Format

`status: v0 — HQ-SPEC-SIM-100 §8 build step 1`

Content-addressed, two-part canonical motion asset (HQ-SPEC-SIM-100 §3). The two parts are
deliberately split by consumer, not by convenience:

- The **binary** (`<name>.gband`) is what the C sampler reads at runtime. Fixed layout, no
  parsing beyond reading a struct and an array of floats — that's the "fits in a hundred lines
  of C" acceptance test from the spec, and it only holds because this file is never JSON.
- The **manifest** (`<name>.gband.json`) is what tooling (import, hash, validate, the reward
  compiler, Back Office) reads. Richer, human-authored fields — authorship, intent tags, loop
  points, safety annotations — none of which the runtime sampler needs to do its one job.

## Binary layout (`<name>.gband`)

Little-endian throughout.

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 4 | `magic` | ASCII `"GBND"` |
| 4 | 4 | `version` (uint32) | format version, `1` today |
| 8 | 4 | `tick_rate` (uint32) | ticks/second this clip was authored at |
| 12 | 4 | `duration_ticks` (uint32) | number of sampled frames |
| 16 | 4 | `num_channels` (uint32) | channels per tick |
| 20 | 32 | `skeleton_hash` | sha256 of the referenced skeleton asset (raw bytes, not hex) |
| 52 | 32 | `content_hash` | sha256 of everything from offset 84 onward (the channel data) — the asset's own content address |
| 84 | `duration_ticks * num_channels * 4` | `channel_data` | row-major: for each tick, `num_channels` float32 values, in the manifest's `channels` order |

Total file size: `84 + duration_ticks * num_channels * 4` bytes.

No curve interpolation modes, no compression, no per-channel type variation. Everything is
resampled to uniform `tick_rate` float32 tracks at import time (HQ-SPEC-SIM-100 §3: "no
curves-with-twelve-interpolation-modes cleverness; resample at import"). Determinism
requirement: the same clip, same seed, same tick must sample to a bit-identical pose on every
platform — a fixed binary layout with no float-format ambiguity is what makes that true.

## Manifest (`<name>.gband.json`)

```json
{
  "gband_version": 1,
  "skeleton_hash": "<hex sha256, matches the binary's skeleton_hash field>",
  "content_hash": "<hex sha256, matches the binary's content_hash field>",
  "tick_rate": 64,
  "duration_ticks": 128,
  "channels": ["root.tx", "root.ty", "root.tz", "root.rx", "root.ry", "root.rz", "hip.rx", ...],
  "authorship": {
    "kind": "mocap | human | generative",
    "who": "free text — labeled honestly, per HQ-SPEC-SIM-100 §3"
  },
  "intent_tags": ["gait", "idle", "gesture", "showpiece"],
  "loop_points": { "start_tick": 0, "end_tick": 128 },
  "safety": { "max_joint_velocity": null, "max_joint_torque": null }
}
```

`channels` must have exactly `num_channels` entries and their order defines the binary's
channel-data column order. `tick_rate`/`duration_ticks` must match the binary exactly —
`gbtool validate` checks this (see below).

Skeleton assets themselves (joint hierarchy, limits, actuator metadata for hardware-bound
characters) are out of scope for this v0 pass — `skeleton_hash` is a forward-compatible
placeholder field today; nothing resolves it to a real skeleton asset yet.

## Content addressing

`content_hash` is computed the same way everywhere (`gbtool hash`, the C sampler's optional
verification path): sha256 over the raw channel-data bytes only (not the header, not the
manifest). Promotions to `golden` status are Apples-logged `ApplePublished` events per
HQ-SPEC-SIM-100 §3 — out of scope for this pass (no promotion pipeline wired yet), but the
hash is what a future promotion event would cite.

## Quaternion channels (S144-XX, glTF import)

BVH import only ever produces Euler channels (`<joint>.Xrotation` etc. — BVH's own native
rotation representation). `gbtool import --gltf` (glTF is Blender's own native, quaternion-based
export format) produces real quaternion channels instead, named `<joint>.qx`, `<joint>.qy`,
`<joint>.qz`, `<joint>.qw` — same "row-major float32 columns, no per-channel type variation"
binary layout as every other channel; the `q` prefix is a naming convention for
tooling/consumers to recognize a quaternion group, not a distinct wire format. Translation
channels from a glTF import are named `<joint>.tx/.ty/.tz`.

`gb_blend`'s runtime interpolation (src/gband.c) is a plain per-component lerp with no
channel-type awareness — correct for scalar/Euler channels, but for a quaternion group this is
an nlerp approximation (lerp then the CALLER must renormalize before using the result for
skinning), not a true slerp. Acceptable for blending between adjacent, already-uniformly-sampled
ticks of the same clip (the only case `gb_blend` is used for); not correct for blending between
two arbitrary, widely-separated orientations. `gbtool import --gltf`'s own resampling step
(collapsing a glTF clip's real, sparse keyframes down to `.gband`'s required uniform tick rate)
performs and renormalizes this same nlerp itself, at import time — the runtime sampler never
does curve interpolation of its own, per this doc's own "resample at import" rule above.

## What v0 does not cover (explicitly deferred)

- glTF import beyond a single skin/mesh/animation per file (`gbtool import --gltf` takes the
  FIRST skin, first mesh's first primitive, first animation only — a real, documented v0 scope
  cut, not silently wrong for a multi-clip file).
- True slerp for quaternion channel blending (see above — nlerp only, both at import-time
  resampling and in the runtime `gb_blend`).
- STEP/CUBICSPLINE glTF sampler interpolation (LINEAR only).
- Retargeting maps, hardware feasibility passes.
- The reward compiler, training backbone, SHANKPIT integration (build steps 2-5).
- `golden` promotion / Apples wiring.
