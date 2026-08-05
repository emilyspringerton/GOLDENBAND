"""create_tyler_armature.py -- S144-07: builds an Armature in the current
Blender scene that exactly matches the 5-joint skeleton already driving
Tyler's box-rig (REDGARDEN/GoblinFoxDragon packages/goldenband/gband_rig.c)
and the synthetic skinned-mesh proof rig (gband_mesh_rig.c). Model your mesh
around this armature, then Ctrl+P -> "With Automatic Weights" to skin it --
no manual weight painting required for a first pass.

Bone positions are written in BLENDER's own Z-up space (so the armature
looks right-side-up in Blender's viewport, matching how you'd expect to see
and model against it) -- (0, 0, height), not the engine's own Y-up space.
export_gband_rig.py converts axes at export time; see that script's own
header for the one real unknown this pipeline has (Blender's BVH exporter's
own axis handling has NOT been verified against the engine, since Blender
isn't available in the environment these scripts were written in).

Usage: open Blender -> Scripting tab -> open this file -> Run Script
(or: blender --background --python create_tyler_armature.py, if you already
have a .blend to run it against). Creates one Armature object, "TylerRig",
with 5 bones: Hips (root) -> Spine -> {Head, L_Arm, R_Arm}.
"""
import bpy

# Bind-pose joint positions in Blender's Z-up space (X, Y, Z=height) --
# matches gen_synthetic_body.py's bind_world_positions() exactly (that
# script works in the engine's own Y-up space, so its Y becomes this
# script's Z; see export_gband_rig.py for the reverse conversion).
JOINTS = [
    # name,    parent,   head_pos,              tail_pos
    ("Hips",   None,     (0.0, 0.0, 0.65),      (0.0, 0.0, 1.00)),
    ("Spine",  "Hips",   (0.0, 0.0, 1.00),      (0.0, 0.0, 1.30)),
    ("Head",   "Spine",  (0.0, 0.0, 1.30),      (0.0, 0.0, 1.55)),
    ("L_Arm",  "Spine",  (-0.45, 0.0, 1.00),    (-0.45, 0.0, 0.50)),
    ("R_Arm",  "Spine",  (0.45, 0.0, 1.00),     (0.45, 0.0, 0.50)),
]

# Bones that are a direct, contiguous extension of their parent (tail-to-
# head) use Blender's "connected" bone convention, which just affects
# editing convenience in Blender -- it has no effect on the exported rest
# pose either way, since the exporter reads each bone's own head/tail
# directly regardless of use_connect.
CONNECTED = {"Spine", "Head"}


def create_tyler_armature():
    bpy.ops.object.armature_add(enter_editmode=True, location=(0, 0, 0))
    arm_obj = bpy.context.object
    arm_obj.name = "TylerRig"
    arm_obj.data.name = "TylerSkeleton"

    eb = arm_obj.data.edit_bones
    for b in list(eb):
        eb.remove(b)  # armature_add's default single bone isn't part of our 5

    created = {}
    for name, parent, head, tail in JOINTS:
        b = eb.new(name)
        b.head = head
        b.tail = tail
        if parent is not None:
            b.parent = created[parent]
            b.use_connect = name in CONNECTED
        created[name] = b

    bpy.ops.object.mode_set(mode="OBJECT")
    print(f"Created {arm_obj.name} with {len(created)} bones: {', '.join(created.keys())}")
    return arm_obj


if __name__ == "__main__":
    create_tyler_armature()
