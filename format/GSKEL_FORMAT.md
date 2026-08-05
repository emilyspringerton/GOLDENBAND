# The `.gskel` Format

`status: v0 — HQ-SPEC-SIM-100 §8 build step 1 follow-on (S144-07)`

A skeleton asset: joint hierarchy, rest pose, and precomputed inverse-bind matrices. Deliberately
separate from `.gband` (motion) and `.gmesh` (geometry) — the same clip can drive any skeleton
with matching joint names, and the same skeleton can be reused by multiple meshes, per
HQ-SPEC-SIM-100 §3's "split by consumer" rule `GBAND_FORMAT.md` already established.

Fixed binary layout, no JSON, same rationale as `.gband`: the runtime loader stays a simple,
bounded parser. Inverse-bind matrices are precomputed by the exporter (Blender's own `mathutils`,
or a synthetic test generator) — the C runtime never inverts a matrix.

## Binary layout (`<name>.gskel`)

Little-endian throughout.

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 4 | `magic` | ASCII `"GSKL"` |
| 4 | 4 | `version` (uint32) | format version, `1` today |
| 8 | 4 | `joint_count` (uint32) | number of joints |
| 12 | `joint_count * 128` | `joints[]` | see below |

Total file size: `12 + joint_count * 128` bytes.

### Per-joint record (128 bytes)

| Offset (within record) | Size | Field |
|---|---|---|
| 0 | 32 | `name` — ASCII, null-padded, null-terminated if shorter than 32 |
| 32 | 4 | `parent_index` (int32) — `-1` for the root, otherwise an index into this same array, always `< self index` (parent-before-child ordering is required, not just conventional — the runtime relies on it for single-pass forward kinematics) |
| 36 | 12 | `rest_local_translation` (3x float32) — offset from parent, BVH `OFFSET`-equivalent |
| 48 | 16 | `rest_local_rotation` (4x float32 quaternion, x/y/z/w order) |
| 64 | 64 | `inverse_bind_matrix` (16x float32, column-major — same layout `packages/common/mat4.h`'s `Mat4` already uses) |

`name` is how a `.gband` clip's channel names (`"<joint>.<channel>"`, see `GBAND_FORMAT.md` and
`gbtool`'s BVH importer) bind to a joint: match on the leaf joint name (the segment after the last
`/` and before the first `.`). A clip missing channels for a given joint simply leaves that joint
at its rest local transform for every tick.

## What v0 does not cover

- Multiple skeletons sharing joints/retargeting maps (HQ-SPEC-SIM-100 §3's hardware-bound
  actuator metadata is a separate, later concern, same as `GBAND_FORMAT.md`'s own deferral).
- Joint limits/constraints.
