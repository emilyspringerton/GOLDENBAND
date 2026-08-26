"""create_ant_armature.py -- S202-28: builds an Armature in the current
Blender scene for a new "Ant" hero, following exactly the same pattern
create_tyler_armature.py already established (same JOINTS-table shape,
same headless --save invocation, same export path through
export_gband_rig.py -- nothing new needed on the format/sampler side:
gskel.h's own GSKEL_MAX_JOINTS is 64 and joint_count/hierarchy are read
from the file, not hardcoded to Tyler's own 5-bone shape, so a real,
different body plan is exactly as supported as Tyler's was).

Founder real-time: "dd a rig for an ant hero" (add a rig for an ant hero).
Scope note, flagged not silently assumed: this builds the ARMATURE only
(GOLDENBAND's own stated scope -- the .gband format, C sampler, and Go
pipeline tools; SHANKPIT/REDGARDEN integration is explicitly separate,
larger, not-yet-started backlog work per this repo's own CLAUDE.md). Real
open question this item's own BACKLOG.md entry already names: whether
"Ant" becomes a real REDGARDEN roster hero (needing a docs/HEROES_VS0.md
kit entry + arena_game.c wiring) is a separate, later decision -- not
assumed here.

Bone positions are written in BLENDER's own Z-up space, same convention
create_tyler_armature.py's own header comment already explains -- model
your mesh around this armature, then Ctrl+P -> "With Automatic Weights"
to skin it.

Body plan, deliberately real-but-minimal (an anatomically exhaustive ant
has 3 leg segments per leg plus separate head/thorax/abdomen sub-joints;
this keeps TylerRig's own "single bone per limb" simplicity instead of
overbuilding a first pass): 3 body segments (Abdomen root -> Thorax ->
Head, laid out horizontally/low to the ground rather than upright like
Tyler's humanoid shape), 6 single-bone legs off the Thorax (front/mid/
rear pairs, splayed outward and down -- the real, recognizable "ant
stance"), and 2 single-bone antennae off the Head.

Usage: open Blender -> Scripting tab -> open this file -> Run Script
(or: blender --background --python create_ant_armature.py, if you already
have a .blend to run it against). Creates one Armature object, "AntRig",
with 11 bones.

Headless save: `blender --background --python create_ant_armature.py --
--save <out.blend>` builds the armature in a fresh scene and saves it as a
standalone .blend -- same shape as create_tyler_armature.py's own headless
path, run by .github/workflows/blender-tools.yml's new build-ant-armature
job.
"""
import bpy
import sys

# Bind-pose joint positions in Blender's Z-up space (X, Y, Z=height).
# Horizontal/low-slung body (Z~0.3, not upright like TylerRig's Z~0.65-1.55)
# -- an ant's own real posture, not a humanoid one.
JOINTS = [
    # name,          parent,       head_pos,              tail_pos
    ("Abdomen",       None,        (0.0, -0.50, 0.30),    (0.0, -0.20, 0.30)),
    ("Thorax",        "Abdomen",   (0.0, -0.20, 0.30),    (0.0,  0.10, 0.30)),
    ("Head",          "Thorax",    (0.0,  0.10, 0.30),    (0.0,  0.35, 0.30)),
    # Legs -- 3 pairs off the Thorax, splayed outward and down (front pair
    # angled forward, rear pair angled back, same real ant leg-fan shape).
    ("L_Leg_Front",   "Thorax",    (0.0,  0.05, 0.30),    (-0.50, 0.20, 0.0)),
    ("L_Leg_Mid",     "Thorax",    (0.0, -0.05, 0.30),    (-0.55, -0.05, 0.0)),
    ("L_Leg_Rear",    "Thorax",    (0.0, -0.15, 0.30),    (-0.50, -0.30, 0.0)),
    ("R_Leg_Front",   "Thorax",    (0.0,  0.05, 0.30),    (0.50, 0.20, 0.0)),
    ("R_Leg_Mid",     "Thorax",    (0.0, -0.05, 0.30),    (0.55, -0.05, 0.0)),
    ("R_Leg_Rear",    "Thorax",    (0.0, -0.15, 0.30),    (0.50, -0.30, 0.0)),
    # Antennae off the Head, angled up and out -- the real, thematically
    # essential "it reads as an ant" feature.
    ("L_Antenna",     "Head",      (-0.05, 0.30, 0.30),   (-0.15, 0.55, 0.45)),
    ("R_Antenna",     "Head",      (0.05, 0.30, 0.30),    (0.15, 0.55, 0.45)),
]

# Same "connected is purely a Blender editing convenience, no effect on
# the exported rest pose" reasoning create_tyler_armature.py's own
# CONNECTED set uses -- only the direct spine-line segments qualify here.
CONNECTED = {"Thorax", "Head"}


def create_ant_armature():
    bpy.ops.object.armature_add(enter_editmode=True, location=(0, 0, 0))
    arm_obj = bpy.context.object
    arm_obj.name = "AntRig"
    arm_obj.data.name = "AntSkeleton"

    eb = arm_obj.data.edit_bones
    for b in list(eb):
        eb.remove(b)  # armature_add's default single bone isn't part of our set

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


def _save_path_from_argv():
    argv = sys.argv
    if "--" not in argv:
        return None
    rest = argv[argv.index("--") + 1:]
    if len(rest) >= 2 and rest[0] == "--save":
        return rest[1]
    return None


if __name__ == "__main__":
    create_ant_armature()
    save_path = _save_path_from_argv()
    if save_path:
        bpy.ops.wm.save_as_mainfile(filepath=save_path)
        print(f"saved {save_path}")
