"""export_gband_rig.py -- S144-07: exports the active Armature + a selected
Mesh (parented to it, with vertex groups named to match bone names -- e.g.
via Ctrl+P > "With Automatic Weights") to real .gskel + .gmesh files
matching GOLDENBAND/format/GSKEL_FORMAT.md and GMESH_FORMAT.md exactly.

*** THE ONE THING IN THIS PIPELINE THAT IS UNVERIFIED, READ THIS FIRST ***
This script was written without Blender available to run it against (see
GoblinFoxDragon/docs2/GOLDENBAND_INTEGRATION_NORTHSTAR.md -- the whole
Phase 2 plan was scoped around not being able to test Blender-side code in
this environment). The one real unknown is axis conversion: Blender is
Z-up, the engine (REDGARDEN/GoblinFoxDragon) is Y-up. This script converts
via a proper 90-degree rotation about X (engine = (blender.x, blender.z,
-blender.y), preserving right-handedness -- not a naive axis swap, which
would mirror the mesh), applied as a matrix similarity transform via
mathutils so Blender's own bone-roll/parent-chain math does the hard part.
It's principled, but the FIRST thing to check once you have a real export:
does the mesh appear upright and correctly proportioned in-engine, not
sideways, mirrored, or tiny/huge? If not, the fix is almost certainly in
AXIS_CONVERT below, not elsewhere in this file.

For animation, this script does NOT export .gband -- reuse Blender's own
built-in File > Export > Motion Capture (.bvh) for each action, then run
that through gbtool import --bvh (already shipped, S144-01) exactly like
tyler_idle.bvh/tyler_walk.bvh were. Blender's BVH exporter has its own axis
handling that's separately unverified here -- same "check it's upright"
sanity test applies.

Usage (Blender's Scripting tab, or `blender --background <file.blend>
--python export_gband_rig.py -- <out_dir>`):
  1. Select your mesh (must have vertex groups named exactly like the
     armature's bone names -- Ctrl+P > With Automatic Weights sets this up
     for you).
  2. Make sure its Armature modifier points at your rigged armature.
  3. Run this script. Writes <out_dir>/<mesh_name>.gskel and .gmesh.
"""
import bpy
import mathutils
import struct
import sys
import os

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


def convert_matrix(m):
    return AXIS_CONVERT @ m @ AXIS_CONVERT_INV


def convert_point(v3):
    v4 = AXIS_CONVERT @ mathutils.Vector((v3[0], v3[1], v3[2], 1.0))
    return (v4[0], v4[1], v4[2])


def convert_direction(v3):
    v4 = AXIS_CONVERT @ mathutils.Vector((v3[0], v3[1], v3[2], 0.0))
    return (v4[0], v4[1], v4[2])


def find_armature_and_mesh():
    mesh_obj = bpy.context.active_object
    if mesh_obj is None or mesh_obj.type != "MESH":
        raise RuntimeError("Select the mesh to export first (the active object must be a Mesh).")
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
            if bone.parent:
                local = convert_matrix(bone.parent.matrix_local.inverted() @ bone.matrix_local)
            else:
                local = convert_matrix(bone.matrix_local)
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
    mesh.calc_normals_split()

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


if __name__ == "__main__":
    main()
