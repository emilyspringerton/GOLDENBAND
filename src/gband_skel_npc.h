// gband_skel_npc.h — general, arbitrary-joint-count character rendering, built on top of
// GOLDENBAND's new gpose.c (real N-joint forward kinematics + skinning) and gseq.c (real
// named-channel animation sampling). Real, deliberate SIBLING to gband_mesh_rig.c/.h, not a
// replacement: gband_mesh_rig.c stays exactly as-is (it draws the PLAYER's own body, tyler_body,
// a real hand-authored 5-joint rig with its own hardcoded channel convention -- still real and
// still working). This module is for any OTHER GOLDENBAND character asset -- founder-imported
// ones (a mannequin, and as of S467's own follow-up, four real distinct robot characters --
// George/Leela/Mike/Stan, each its own real mesh+skeleton+animation set, confirmed via a direct
// GOLDENBAND-side load-and-inspect: 47/17/43/43 real joints respectively, not a shared rig) --
// used to spawn NPCs. See EMILY/BACKLOG.md S459-97/S467/S468 and GOLDENBAND's own src/gpose.h
// header comment for the general-vs-hardcoded rationale.
//
// Real multi-kit support (S467 follow-up, founder real-time: "GEORGE LEELA MIKE AND STAN ARE
// ANIMATED ROBOT CHARACTERS WITH MESH RIG AND ANIMATIONS PER BOT"). The original v0 scope --
// "only one character asset can be loaded at a time" -- is lifted: gband_skel_npc_load_kit loads
// a real, independent character (its own skeleton, mesh, idle/walk clips) into a real kit slot,
// and gband_skel_npc_draw takes a kit_index so different NPC instances can be different
// characters, not clones of one shared look. Kits are loaded once at startup and never unloaded
// individually (gband_skel_npc_shutdown frees all of them together) -- matches the real,
// deliberate "small, fixed roster, loaded once" scope every other GOLDENBAND asset already
// assumes (no runtime asset streaming exists anywhere in this repo).
#ifndef GOLDENBAND_GBAND_SKEL_NPC_H
#define GOLDENBAND_GBAND_SKEL_NPC_H

#include "../common/mat4.h"

#define GBAND_SKEL_NPC_MAX_KITS 8

// S470 follow-up, founder real-time: "get all the robots in there and have them walking around
// different waypoints... if you can get them to wave to the player when the player gets close
// and then dance before resuming patrol". Real, honest asset-driven behavior: 0 is the existing
// movement-driven idle/walk auto-pick, GREET/DANCE force a specific one-shot(-looking) gesture
// clip regardless of movement -- the caller (SHANKPIT's story_ai-driven anim_override, see
// PlayerState's own field) decides WHEN, this module only decides WHICH clip plays once told.
#define GBAND_SKEL_NPC_ANIM_AUTO  0
#define GBAND_SKEL_NPC_ANIM_GREET 1
#define GBAND_SKEL_NPC_ANIM_DANCE 2

// gband_skel_npc_load_kit loads <asset_dir>/<mesh_name>.gskel + .gmesh, plus real animation
// clips (+ their own .gband.json manifests, for real channel names): <idle_clip_name> looped
// while stationary, <walk_clip_name> looped while moving. greet_clip_name/dance_clip_name are
// each OPTIONAL (pass NULL if this character has no real clip for that gesture) -- a kit missing
// one is real and expected, not an error: not every currently-uploaded character has both a real
// greeting gesture AND a real dance (checked directly against the live asset library before
// wiring this, not assumed -- see EMILY/BACKLOG.md S470 for exactly which kit has which). A kit
// with anim_override forced but no real clip loaded for it falls back to idle in
// gband_skel_npc_draw, never draws nothing. Returns the new kit's real index (0-based, stable
// for the process lifetime) on success, or -1 on failure (missing/malformed required asset, a
// skeleton with more joints than GSKEL_MAX_JOINTS, or GBAND_SKEL_NPC_MAX_KITS already reached).
int gband_skel_npc_load_kit(const char *asset_dir, const char *mesh_name,
                             const char *idle_clip_name, const char *walk_clip_name,
                             const char *greet_clip_name, const char *dance_clip_name);

void gband_skel_npc_shutdown(void);

// gband_skel_npc_kit_ready reports whether kit_index is a real, successfully-loaded kit --
// callers should fall back to a different skin/kit on 0, never draw nothing.
int gband_skel_npc_kit_ready(int kit_index);

// gband_skel_npc_draw animates, skins, and draws one NPC instance using kit_index's own real
// character (independent animation clocks per npc_slot -- re-initialized automatically if the
// same npc_slot is later drawn with a DIFFERENT kit_index, e.g. a respawned NPC assigned a new
// look, so a stale animation player is never sampled against the wrong clip/skeleton).
// anim_override selects which clip plays -- GBAND_SKEL_NPC_ANIM_AUTO detects movement the same
// real way gband_mesh_rig_draw already does (a per-slot previous-position comparison against a
// small epsilon) and picks idle/walk; GREET/DANCE force that specific gesture clip regardless of
// movement, falling back to idle if this kit has no real clip loaded for the requested gesture.
// Same draw_skinned callback contract as gband_mesh_rig_draw -- pos+normal, world-space,
// non-indexed triangle list.
void gband_skel_npc_draw(int kit_index, int npc_slot, float npc_x, float npc_y, float npc_z, float facing_rad, float dt_ms,
                          int anim_override,
                          const Mat4 *vp,
                          void (*draw_skinned)(const float *verts6, int vert_count,
                                                const Mat4 *mvp, const Mat4 *model));

#endif // GOLDENBAND_GBAND_SKEL_NPC_H
