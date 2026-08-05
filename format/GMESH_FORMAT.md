# The `.gmesh` Format

`status: v0 — HQ-SPEC-SIM-100 §8 build step 1 follow-on (S144-07)`

Skinned geometry: positions, normals, UVs (reserved, unused until a texturing pass exists), and up
to 4 bone influences per vertex. Paired with a `.gskel` (bone indices are indices into that
skeleton's `joints[]`) and animated by a `.gband` clip sampled through the skeleton — see
`GSKEL_FORMAT.md`.

Fixed binary layout, same rationale as `.gband`/`.gskel`.

## Binary layout (`<name>.gmesh`)

Little-endian throughout.

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 4 | `magic` | ASCII `"GMSH"` |
| 4 | 4 | `version` (uint32) | format version, `1` today |
| 8 | 4 | `vertex_count` (uint32) | |
| 12 | 4 | `index_count` (uint32) | always a multiple of 3 (triangle list) |
| 16 | `vertex_count * 52` | `vertices[]` | see below |
| 16 + vertex_count*52 | `index_count * 4` | `indices[]` | uint32 triangle indices into `vertices[]` |

### Per-vertex record (52 bytes)

| Offset (within record) | Size | Field |
|---|---|---|
| 0 | 12 | `position` (3x float32), bind pose, in the skeleton's rest-pose world space (same space `.gskel`'s `inverse_bind_matrix` values assume) |
| 12 | 12 | `normal` (3x float32), bind pose |
| 24 | 8 | `uv` (2x float32) — reserved; no texturing pass consumes this yet |
| 32 | 4 | `bone_indices` (4x uint8) — indices into the paired `.gskel`'s `joints[]`; unused slots (weight 0) may repeat index 0, the loader never reads an index whose weight is 0 |
| 36 | 16 | `bone_weights` (4x float32) — normalized, sums to 1.0 per vertex; the exporter's responsibility, not validated at load time (same "trust the pipeline, no runtime cleverness" stance `.gband` already takes) |

## Runtime skinning (CPU, deliberately — see `GoblinFoxDragon/docs2/GOLDENBAND_INTEGRATION_NORTHSTAR.md` §2 Phase 2 for why)

For each vertex, each frame:

```
skinned_pos = Σ_k weight[k] * (joint_world[bone_indices[k]] * inverse_bind_matrix[bone_indices[k]] * bind_pos)
```

summed over the up to 4 influences, where `joint_world[j]` comes from the same forward-kinematics
pass `.gband`/`.gskel` already drive. Normals use the same blend with the upper 3×3 (no
translation), renormalized after.

Note: `gskel.h`/`gmesh.h` themselves are pure data loaders (no engine dependency, matching
`gband.h`'s own "no engine dependency in the asset" rule) — they know nothing about matrices or
forward kinematics. The FK + skinning math above lives in the *consuming* engine's own bridge code
(e.g. `packages/goldenband/gband_mesh_rig.c` in REDGARDEN/GFD), which already has a real `Mat4`
type to do it with. `gband_rig.c` (the Phase 1 box-rig) is the worked example for the FK half.

## What v0 does not cover

- Multiple UV sets, vertex colors, tangents (no texturing pass exists yet to need them).
- LOD / multiple meshes per character.
- GPU (shader-based) skinning — CPU is the deliberate v0 choice, see the northstar doc.
