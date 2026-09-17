// gband_skel_npc.h — general, arbitrary-joint-count character rendering, built on top of
// GOLDENBAND's new gpose.c (real N-joint forward kinematics + skinning) and gseq.c (real
// named-channel animation sampling). Real, deliberate SIBLING to gband_mesh_rig.c/.h, not a
// replacement: gband_mesh_rig.c stays exactly as-is (it draws the PLAYER's own body, tyler_body,
// a real hand-authored 5-joint rig with its own hardcoded channel convention -- still real and
// still working). This module is for any OTHER GOLDENBAND character asset -- in particular a
// real, founder-imported one (a mannequin mesh+skeleton paired with a UAL2_Standard_RM mocap
// clip, both a real 65-joint industry-standard rig, confirmed to share the exact same joint
// hierarchy) -- used to spawn NPCs. See EMILY/BACKLOG.md S459-97 and GOLDENBAND's own
// src/gpose.h header comment for the general-vs-hardcoded rationale.
//
// Real idle/walk clip switching (S466 follow-up, founder real-time: "continue fill in the gaps
// can we animate and model end to end?"): adopts gband_mesh_rig.c's own real, proven,
// movement-delta-driven switch (no crossfade -- an instant clip swap, matching that same real,
// already-shipped behavior exactly) rather than inventing a different scheme. Both the idle and
// walk GSeqPlayer for a given npc_slot are always advanced every frame (even the one not
// currently sampled), so switching never causes a visible time-jump -- only which track gets
// SAMPLED changes.
#ifndef GOLDENBAND_GBAND_SKEL_NPC_H
#define GOLDENBAND_GBAND_SKEL_NPC_H

#include "../common/mat4.h"

// gband_skel_npc_init loads <asset_dir>/<mesh_name>.gskel + .gmesh, plus two real animation
// clips (+ their own .gband.json manifests, for real channel names): <idle_clip_name> looped
// while stationary, <walk_clip_name> looped while moving. Returns 1 on success, 0 on failure
// (missing/malformed asset, or a skeleton with more joints than GSKEL_MAX_JOINTS). Only one
// character asset can be loaded at a time (same real, deliberate v0 scope as gband_mesh_rig.c's
// own single-mesh_name convention) -- callers spawn multiple independent NPC *instances* of that
// one loaded asset via distinct npc_slot values in gband_skel_npc_draw, each with its own real,
// independent animation clock pair (idle + walk, both always ticking).
int gband_skel_npc_init(const char *asset_dir, const char *mesh_name, const char *idle_clip_name, const char *walk_clip_name);

void gband_skel_npc_shutdown(void);

int gband_skel_npc_ready(void);

// gband_skel_npc_draw animates, skins, and draws one NPC instance (independent animation clocks
// per npc_slot). Movement is detected the same real way gband_mesh_rig_draw already does: a
// per-slot previous-position comparison against a small epsilon, not an explicit "is this NPC
// walking" flag from the caller -- so any caller that just updates npc_x/z each frame (AI-driven
// or otherwise) gets correct idle/walk switching for free. Same draw_skinned callback contract
// as gband_mesh_rig_draw -- pos+normal, world-space, non-indexed triangle list.
void gband_skel_npc_draw(int npc_slot, float npc_x, float npc_y, float npc_z, float facing_rad, float dt_ms,
                          const Mat4 *vp,
                          void (*draw_skinned)(const float *verts6, int vert_count,
                                                const Mat4 *mvp, const Mat4 *model));

#endif // GOLDENBAND_GBAND_SKEL_NPC_H
