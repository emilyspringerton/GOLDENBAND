"""export_gband_rig.py -- S144-07: exports the active Armature + a selected
Mesh (parented to it, with vertex groups named to match bone names -- e.g.
via Ctrl+P > "With Automatic Weights") to real .gskel + .gmesh files
matching GOLDENBAND/format/GSKEL_FORMAT.md and GMESH_FORMAT.md exactly.

*** UPDATE 2026-08-05: verified against real Blender 4.2.23, not blind. ***
Earlier versions of this script were written with no Blender available at
all. A real Blender install was obtained for real testing this same day,
and that testing found and fixed three real bugs:
  1. The mesh/skeleton axis-conversion MATH is correct (verified
     numerically), but bone rest transforms must be converted per-bone
     (parent and child each converted individually, THEN combined) rather
     than converting an already-parent-relative matrix -- both are
     algebraically equivalent for pure translations but not in general;
     fixed to convert-then-combine, the mathematically robust order.
  2. `mesh.calc_normals_split()` was removed in Blender 4.1+ (split normals
     are automatic now) -- removed the call, `loop.normal` already works.
  3. Blender's OWN built-in BVH exporter turned out to export in Blender's
     native Z-up space (not auto-converted to Y-up as originally assumed
     here) AND gives some bones 6 channels instead of 3 depending on
     connectivity -- rather than fight that inconsistency, animation export
     is now done directly by THIS script (export_animation, below),
     reading posed bone matrices per frame and converting axes the same
     principled way the skeleton/mesh already do. Blender's built-in BVH
     exporter is no longer part of this pipeline at all.

export_animation itself was then also verified with a real keyframed test
animation (translation + rotation), round-tripped through the real C
gband.c loader and checked value-by-value against hand-computed
expectations -- correct on the first real check, once the test itself
stopped confusing a bone-local axis for a world one.

Axis conversion (verified numerically, see the real Blender test run this
comment describes): Blender is Z-up, the engine (REDGARDEN/GoblinFoxDragon)
is Y-up. AXIS_CONVERT below is a proper 90-degree rotation about X
(engine = (blender.x, blender.z, -blender.y)), preserving right-handedness
-- confirmed correct by exporting a real cylinder mesh and checking its
bounding box came out at the expected engine-space height.

Usage (Blender's Scripting tab, or `blender --background <file.blend>
--python export_gband_rig.py -- <out_dir>`):
  1. Select your mesh (must have vertex groups named exactly like the
     armature's bone names -- Ctrl+P > With Automatic Weights sets this up
     for you). It must be the ACTIVE object, not the armature (click it in
     the outliner/viewport last).
  2. Make sure its Armature modifier points at your rigged armature.
  3. Run this script. Writes <out_dir>/<mesh_name>.gskel and .gmesh, plus
     one <out_dir>/<action_name>.gband per Action found on the armature
     (Idle, Walk, etc. -- however you named them).
"""
import bpy
import mathutils
import struct
import sys
import os
import math
import hashlib

MAGIC_GSKEL = b"GSKL"
MAGIC_GMESH = b"GMSH"
GSKEL_NAME_LEN = 32
MAX_INFLUENCES = 4

# See the axis-conversion warning in this file's own header. This is a
# proper rotation (X stays, Y<->Z with one sign flip), not a mirror.
AXIS_CONVERT = mathutils.Matrix((
    (1, 0, 0, 0),
    (0, 0, 1, 0),
    (0, -1, 0, 0),
    (0, 0, 0, 1),
))
AXIS_CONVERT_INV = AXIS_CONVERT.inverted()

# MODEL_SCALE (2026-08-05, founder: "the unit scale isnt going to match you
# are going to have to scale it"): the first real founder-modeled export
# measured ~6.82 engine units tall; every other hero in the roster runs
# ~1.3 units tall (Tyler's own old box was BOX(...,0.75,1.3,0.75)). This
# targets that same ballpark -- tune per-model if a different character
# comes in at a very different scale, this isn't meant to be universal.
MODEL_SCALE = 0.19


def convert_matrix(m):
    """Axis-converts a rigid (rotation + translation, no scale) matrix, then
    scales just the translation column by MODEL_SCALE -- keeps rotation
    exact while uniformly rescaling *where* things are positioned. Used for
    both bone rest transforms and (via .inverted() on its result elsewhere)
    inverse-bind matrices, so mesh positions (scaled the same way in
    convert_point) and skeleton positions stay mutually consistent."""
    result = AXIS_CONVERT @ m @ AXIS_CONVERT_INV
    result.translation = result.translation * MODEL_SCALE
    return result


def convert_point(v3):
    v4 = AXIS_CONVERT @ mathutils.Vector((v3[0], v3[1], v3[2], 1.0))
    return (v4[0] * MODEL_SCALE, v4[1] * MODEL_SCALE, v4[2] * MODEL_SCALE)


def convert_direction(v3):
    v4 = AXIS_CONVERT @ mathutils.Vector((v3[0], v3[1], v3[2], 0.0))
    return (v4[0], v4[1], v4[2])


def find_armature_and_mesh():
    # Real friction found by actually running this against real uploaded
    # files (2026-08-05): requiring the mesh to be the *active* object at
    # save time is brittle -- easy to leave the armature active after the
    # Ctrl+P/Pose Mode step, and CI runs this unattended anyway (there's no
    # "active object" to control at all in a scripted incoming/ upload).
    # Prefer scanning the whole file for a mesh with an Armature modifier;
    # only fall back to requiring an active-object mesh if that's ambiguous
    # (more than one candidate) or fails outright.
    candidates = [obj for obj in bpy.data.objects if obj.type == "MESH"
                  and any(m.type == "ARMATURE" and m.object is not None for m in obj.modifiers)]
    if len(candidates) == 1:
        mesh_obj = candidates[0]
    elif len(candidates) > 1:
        names = ", ".join(o.name for o in candidates)
        active = bpy.context.active_object
        if active is not None and active in candidates:
            mesh_obj = active
        else:
            raise RuntimeError(
                f"Multiple meshes with an Armature modifier found ({names}) and none is the "
                f"active object -- select the one you want to export before running this."
            )
    else:
        mesh_obj = bpy.context.active_object
        if mesh_obj is None or mesh_obj.type != "MESH":
            raise RuntimeError(
                "No mesh with an Armature modifier found anywhere in this file, and the active "
                "object isn't a mesh either. Make sure your mesh is parented to the armature "
                "(Ctrl+P > With Automatic Weights) before exporting."
            )

    arm_obj = None
    for mod in mesh_obj.modifiers:
        if mod.type == "ARMATURE" and mod.object is not None:
            arm_obj = mod.object
            break
    if arm_obj is None:
        raise RuntimeError(f"{mesh_obj.name} has no Armature modifier pointing at a rig.")
    return arm_obj, mesh_obj


def export_gskel(arm_obj, out_path):
    bones = list(arm_obj.data.bones)  # rest-pose bones, already parent-before-child order in Blender
    name_to_index = {b.name: i for i, b in enumerate(bones)}

    with open(out_path, "wb") as f:
        f.write(MAGIC_GSKEL)
        f.write(struct.pack("<I", 1))
        f.write(struct.pack("<I", len(bones)))
        for bone in bones:
            name_bytes = bone.name.encode("ascii")[:GSKEL_NAME_LEN - 1]
            f.write(name_bytes + b"\x00" * (GSKEL_NAME_LEN - len(name_bytes)))

            parent_index = name_to_index[bone.parent.name] if bone.parent else -1
            f.write(struct.pack("<i", parent_index))

            # Rest transform relative to parent (world/armature-space if root).
            # Bug found by actually running this against real Blender data
            # (2026-08-05): bone.matrix_local is in Blender's world/armature
            # space, so AXIS_CONVERT must be applied to it directly -- the
            # PARENT-relative "local" transform (parent^-1 @ child) is
            # already expressed in the parent bone's own rotated local frame,
            # and conjugating THAT by a world-space axis change is wrong
            # (verified: it silently produced a translation on the wrong
            # axis, (0,0,-0.35) instead of the expected (0,0.35,0), for a
            # bone offset that's purely vertical in both spaces). Converting
            # each bone's own world-space matrix first, then taking the
            # relative transform between two already-converted matrices, is
            # the correct order.
            bone_engine = convert_matrix(bone.matrix_local)
            if bone.parent:
                parent_engine = convert_matrix(bone.parent.matrix_local)
                local = parent_engine.inverted() @ bone_engine
            else:
                local = bone_engine
            t = local.to_translation()
            q = local.to_quaternion()  # Blender quaternions are (w,x,y,z); format wants (x,y,z,w)
            f.write(struct.pack("<3f", t.x, t.y, t.z))
            f.write(struct.pack("<4f", q.x, q.y, q.z, q.w))

            inv_bind = convert_matrix(bone.matrix_local.inverted())
            # mathutils.Matrix is row-major when indexed [row][col]; .gskel wants column-major
            # (matches packages/common/mat4.h) -- transpose on the way out.
            flat = []
            for col in range(4):
                for row in range(4):
                    flat.append(inv_bind[row][col])
            f.write(struct.pack("<16f", *flat))

    print(f"wrote {out_path} ({len(bones)} joints)")
    return name_to_index


def export_gmesh(mesh_obj, bone_name_to_index, out_path):
    mesh = mesh_obj.data
    mesh.calc_loop_triangles()
    # calc_normals_split() existed pre-4.1 to populate loop.normal; Blender
    # 4.1+ removed it because split normals are computed automatically now
    # (loop.normal is already valid without an explicit call). Real bug
    # found by actually running this against Blender 4.2.23, not assumed.

    vgroup_to_bone = {}
    for vg in mesh_obj.vertex_groups:
        if vg.name in bone_name_to_index:
            vgroup_to_bone[vg.index] = bone_name_to_index[vg.name]
    if not vgroup_to_bone:
        raise RuntimeError(
            "No vertex group names match any bone name -- did you skin with "
            "Ctrl+P > With Automatic Weights against the right armature?"
        )

    # Flatten per-loop (Blender's split normals are per-loop, not per-vertex,
    # and a shared vertex can have different normals per triangle at a hard
    # edge) -- simplest correct approach: one output vertex per loop, no
    # index sharing across triangles with different normals. This mirrors
    # what most engine exporters do, at the cost of not sharing vertices
    # across smooth-shaded triangles either; a real optimization (weld
    # identical loops) is a reasonable future pass, not needed for
    # correctness here.
    out_verts = []
    out_indices = []
    for tri in mesh.loop_triangles:
        for loop_index in tri.loops:
            loop = mesh.loops[loop_index]
            vert = mesh.vertices[loop.vertex_index]

            pos = convert_point(mesh_obj.matrix_world @ vert.co)
            nrm = convert_direction(mesh_obj.matrix_world.to_3x3() @ loop.normal)
            uv = (0.0, 0.0)
            if mesh.uv_layers.active:
                uv = tuple(mesh.uv_layers.active.data[loop_index].uv)

            influences = []  # (bone_index, weight)
            for ge in vert.groups:
                if ge.group in vgroup_to_bone and ge.weight > 0.0:
                    influences.append((vgroup_to_bone[ge.group], ge.weight))
            influences.sort(key=lambda p: -p[1])
            influences = influences[:MAX_INFLUENCES]
            total = sum(w for _, w in influences) or 1.0
            bone_indices = [bi for bi, _ in influences] + [0] * (MAX_INFLUENCES - len(influences))
            weights = [w / total for _, w in influences] + [0.0] * (MAX_INFLUENCES - len(influences))

            out_verts.append((pos, nrm, uv, bone_indices, weights))
            out_indices.append(len(out_verts) - 1)

    with open(out_path, "wb") as f:
        f.write(MAGIC_GMESH)
        f.write(struct.pack("<I", 1))
        f.write(struct.pack("<I", len(out_verts)))
        f.write(struct.pack("<I", len(out_indices)))
        for pos, nrm, uv, bone_indices, weights in out_verts:
            f.write(struct.pack("<3f", *pos))
            f.write(struct.pack("<3f", *nrm))
            f.write(struct.pack("<2f", *uv))
            f.write(bytes(bone_indices))
            f.write(struct.pack("<4f", *weights))
        for idx in out_indices:
            f.write(struct.pack("<I", idx))

    print(f"wrote {out_path} ({len(out_verts)} verts, {len(out_indices)//3} tris)")


def export_animation(arm_obj, action, out_path, tick_rate=30):
    """Writes one Action to a real .gband file. Channels are the FULL baked
    local rotation per frame (not a delta from rest) -- self-sufficient per
    tick, matching what gband_rig.c/gband_mesh_rig.c already hardcode: 6
    channels for the root (translation + rotation), 3 rotation-only
    channels for every other joint, in bones-list order. That order (Hips,
    Spine, Head, L_Arm, R_Arm for TylerRig) was verified empirically to
    match Blender's own bone storage order for this specific armature --
    not a general guarantee, but true for the one skeleton this pipeline
    targets today.
    """
    bones = list(arm_obj.data.bones)
    pose_bones = [arm_obj.pose.bones[b.name] for b in bones]
    parent_index_of = {}
    for i, b in enumerate(bones):
        parent_index_of[i] = bones.index(b.parent) if b.parent else -1

    frame_start = int(round(action.frame_range[0]))
    frame_end = int(round(action.frame_range[1]))
    duration_ticks = frame_end - frame_start + 1

    if arm_obj.animation_data is None:
        arm_obj.animation_data_create()
    prev_action = arm_obj.animation_data.action
    arm_obj.animation_data.action = action

    scene = bpy.context.scene
    num_channels = 6 + 3 * (len(bones) - 1)
    channel_data = []

    for frame in range(frame_start, frame_end + 1):
        scene.frame_set(frame)
        # Defensive: forces the dependency graph to re-evaluate pose bones
        # before reading them. Not confirmed to be strictly necessary in
        # this Blender version (frame_set appeared to already do this when
        # tested), but cheap insurance against relying on undocumented
        # background-mode behavior.
        bpy.context.view_layer.update()
        engine_mats = [convert_matrix(pb.matrix) for pb in pose_bones]

        for i in range(len(bones)):
            parent_i = parent_index_of[i]
            local = engine_mats[i] if parent_i == -1 else (engine_mats[parent_i].inverted() @ engine_mats[i])
            if parent_i == -1:
                t = local.to_translation()
                channel_data += [t.x, t.y, t.z]
            euler = local.to_euler("XYZ")
            channel_data += [math.degrees(euler.x), math.degrees(euler.y), math.degrees(euler.z)]

    arm_obj.animation_data.action = prev_action

    data_bytes = struct.pack(f"<{len(channel_data)}f", *channel_data)
    content_hash = hashlib.sha256(data_bytes).digest()
    skeleton_hash = b"\x00" * 32

    with open(out_path, "wb") as f:
        f.write(b"GBND")
        f.write(struct.pack("<I", 1))
        f.write(struct.pack("<I", tick_rate))
        f.write(struct.pack("<I", duration_ticks))
        f.write(struct.pack("<I", num_channels))
        f.write(skeleton_hash)
        f.write(content_hash)
        f.write(data_bytes)

    print(f"wrote {out_path} (tick_rate={tick_rate} duration_ticks={duration_ticks} num_channels={num_channels})")


def main():
    argv = sys.argv
    out_dir = "."
    if "--" in argv:
        rest = argv[argv.index("--") + 1:]
        if rest:
            out_dir = rest[0]
    os.makedirs(out_dir, exist_ok=True)

    arm_obj, mesh_obj = find_armature_and_mesh()
    name_to_index = export_gskel(arm_obj, os.path.join(out_dir, f"{mesh_obj.name}.gskel"))
    export_gmesh(mesh_obj, name_to_index, os.path.join(out_dir, f"{mesh_obj.name}.gmesh"))

    actions = [a for a in bpy.data.actions if a.users > 0]
    if not actions:
        print("no Actions found on this file -- skipping animation export "
              "(model/skeleton/mesh still exported above)")
    for action in actions:
        export_animation(arm_obj, action, os.path.join(out_dir, f"{action.name}.gband"))


if __name__ == "__main__":
    main()
