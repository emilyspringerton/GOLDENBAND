# Animation stitching example: "walk, turn, raise gun, shoot"

Real, working `gseq.h`/`gseq.c` usage matching the founder's own real-time framing ("make sure we
can stitch animations together like James Bond walk turn raise gun shoot"). This example shows
the real API shape -- it does **not** ship real walk/turn/raise-gun/shoot animation clips (no
such character asset exists in this repo yet, same honest gap named elsewhere in this monorepo's
own BACKLOG for character content). Swap `WALK_GBAND`/`TURN_GBAND`/etc. below for real clips
(e.g. imported via `gbtool import --gltf` from a real rigged Blender export, or from NOCK's own
animation repository) and this runs unmodified.

```c
#include "gseq.h"

GSeqClip clips[4];
gseq_clip_load(WALK_GBAND, WALK_GBAND ".json", &clips[0]);
gseq_clip_load(TURN_GBAND, TURN_GBAND ".json", &clips[1]);
gseq_clip_load(RAISE_GUN_GBAND, RAISE_GUN_GBAND ".json", &clips[2]);
gseq_clip_load(SHOOT_GBAND, SHOOT_GBAND ".json", &clips[3]);

GSeq bond_sequence = {
    .steps = {
        { .clip_index = 0 }, // walk -- real, own duration (duration_ticks / tick_rate)
        { .clip_index = 1 }, // turn
        { .clip_index = 2 }, // raise gun
        { .clip_index = 3, .duration_seconds = 0.4f }, // shoot -- real, explicit override
    },
    .step_count = 4,
    .blend_seconds = 0.15f, // real 150ms crossfade at every transition
    .loop = 0,               // hold the shoot pose once reached, don't loop back to walk
};

GSeqPlayer player;
gseq_player_init(&player, &bond_sequence, clips);

// Once per real frame:
gseq_player_advance(&player, dt_seconds);
float rot[MAX_JOINTS * 4], trans[MAX_JOINTS * 3];
gseq_player_sample_pose(&player, &skel, rot, trans);
// rot/trans now hold this frame's real, blended per-joint pose -- feed straight into the
// consuming engine's own FK + skinning bridge code (same real split GMESH_FORMAT.md's own
// "Runtime skinning" section already documents -- gseq.c stays a pure data transform, same as
// gband.c/gskel.c/gmesh.c).
```

See `src/gseq.h`'s own header comment for the full design (why channel names matter for
stitching, the real crossfade-freezes-the-outgoing-clip's-final-pose behavior, and what v0
deliberately doesn't cover yet), and `tests/test_gseq.c` for a fully real, running, synthetic
2-clip example with assertions on every step of the crossfade math.
