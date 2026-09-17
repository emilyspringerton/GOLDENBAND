// gband_skel_npc.c — see gband_skel_npc.h.
#include "gband_skel_npc.h"
#include "gband.h"
#include "gskel.h"
#include "gmesh.h"
#include "gseq.h"
#include "gpose.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct {
    int ready;
    GSkel skel;
    GMesh mesh;
    GSeqClip idle_clip, walk_clip;
    GSeq idle_seq, walk_seq;
    /* S470 -- both optional, see has_greet/has_dance. A kit that never loaded one keeps its own
       GSeqClip/GSeq zeroed and is never sampled -- gband_skel_npc_draw checks the flag first. */
    GSeqClip greet_clip, dance_clip;
    GSeq greet_seq, dance_seq;
    int has_greet, has_dance;
    float *out_buf; /* owned, 6 floats per output vertex (pos+normal), sized to THIS kit's own mesh */
    uint32_t out_capacity_verts;
} GbandSkelNpcKit;

static GbandSkelNpcKit g_kits[GBAND_SKEL_NPC_MAX_KITS];
static int g_kit_count = 0;

static void init_single_clip_seq(GSeq *seq) {
    memset(seq, 0, sizeof(*seq));
    seq->steps[0].clip_index = 0;
    seq->steps[0].duration_seconds = -1.0f; /* the clip's own real duration */
    seq->step_count = 1;
    seq->blend_seconds = 0.0f; /* single, looping clip -- nothing to cross-fade into */
    seq->loop = 1;
}

int gband_skel_npc_load_kit(const char *asset_dir, const char *mesh_name,
                             const char *idle_clip_name, const char *walk_clip_name,
                             const char *greet_clip_name, const char *dance_clip_name) {
    if (g_kit_count >= GBAND_SKEL_NPC_MAX_KITS) return -1;
    GbandSkelNpcKit *kit = &g_kits[g_kit_count];
    memset(kit, 0, sizeof(*kit));

    char path[512], gband_path[512], manifest_path[512];

    snprintf(path, sizeof(path), "%s/%s.gskel", asset_dir, mesh_name);
    if (!gskel_init(path, &kit->skel)) return -1;
    snprintf(path, sizeof(path), "%s/%s.gmesh", asset_dir, mesh_name);
    if (!gmesh_init(path, &kit->mesh)) return -1;

    snprintf(gband_path, sizeof(gband_path), "%s/%s.gband", asset_dir, idle_clip_name);
    snprintf(manifest_path, sizeof(manifest_path), "%s/%s.gband.json", asset_dir, idle_clip_name);
    if (!gseq_clip_load(gband_path, manifest_path, &kit->idle_clip)) { gmesh_free(&kit->mesh); return -1; }

    snprintf(gband_path, sizeof(gband_path), "%s/%s.gband", asset_dir, walk_clip_name);
    snprintf(manifest_path, sizeof(manifest_path), "%s/%s.gband.json", asset_dir, walk_clip_name);
    if (!gseq_clip_load(gband_path, manifest_path, &kit->walk_clip)) { gseq_clip_free(&kit->idle_clip); gmesh_free(&kit->mesh); return -1; }

    /* S470 -- both real, optional gestures. A NULL name means this character has no real clip
       for that gesture (checked directly against the live asset library, not silently assumed);
       a non-NULL name that fails to load is a real error, same as idle/walk above -- if the
       caller named a clip, it must actually be there. */
    if (greet_clip_name) {
        snprintf(gband_path, sizeof(gband_path), "%s/%s.gband", asset_dir, greet_clip_name);
        snprintf(manifest_path, sizeof(manifest_path), "%s/%s.gband.json", asset_dir, greet_clip_name);
        if (!gseq_clip_load(gband_path, manifest_path, &kit->greet_clip)) {
            gseq_clip_free(&kit->idle_clip); gseq_clip_free(&kit->walk_clip); gmesh_free(&kit->mesh); return -1;
        }
        kit->has_greet = 1;
    }
    if (dance_clip_name) {
        snprintf(gband_path, sizeof(gband_path), "%s/%s.gband", asset_dir, dance_clip_name);
        snprintf(manifest_path, sizeof(manifest_path), "%s/%s.gband.json", asset_dir, dance_clip_name);
        if (!gseq_clip_load(gband_path, manifest_path, &kit->dance_clip)) {
            if (kit->has_greet) gseq_clip_free(&kit->greet_clip);
            gseq_clip_free(&kit->idle_clip); gseq_clip_free(&kit->walk_clip); gmesh_free(&kit->mesh); return -1;
        }
        kit->has_dance = 1;
    }

    init_single_clip_seq(&kit->idle_seq);
    init_single_clip_seq(&kit->walk_seq);
    if (kit->has_greet) init_single_clip_seq(&kit->greet_seq);
    if (kit->has_dance) init_single_clip_seq(&kit->dance_seq);

    kit->out_capacity_verts = kit->mesh.index_count;
    kit->out_buf = (float *)malloc((size_t)kit->out_capacity_verts * 6 * sizeof(float));
    if (!kit->out_buf) {
        if (kit->has_greet) gseq_clip_free(&kit->greet_clip);
        if (kit->has_dance) gseq_clip_free(&kit->dance_clip);
        gseq_clip_free(&kit->idle_clip); gseq_clip_free(&kit->walk_clip); gmesh_free(&kit->mesh); return -1;
    }

    kit->ready = 1;
    return g_kit_count++;
}

void gband_skel_npc_shutdown(void) {
    for (int i = 0; i < g_kit_count; i++) {
        GbandSkelNpcKit *kit = &g_kits[i];
        if (!kit->ready) continue;
        gseq_clip_free(&kit->idle_clip);
        gseq_clip_free(&kit->walk_clip);
        if (kit->has_greet) gseq_clip_free(&kit->greet_clip);
        if (kit->has_dance) gseq_clip_free(&kit->dance_clip);
        gmesh_free(&kit->mesh);
        free(kit->out_buf);
        kit->out_buf = NULL;
        kit->ready = 0;
    }
    g_kit_count = 0;
}

int gband_skel_npc_kit_ready(int kit_index) {
    if (kit_index < 0 || kit_index >= g_kit_count) return 0;
    return g_kits[kit_index].ready;
}

#define MAX_NPC_SLOTS 32
static GSeqPlayer g_idle_players[MAX_NPC_SLOTS];
static GSeqPlayer g_walk_players[MAX_NPC_SLOTS];
static GSeqPlayer g_greet_players[MAX_NPC_SLOTS]; /* S470 -- only sampled when kit->has_greet */
static GSeqPlayer g_dance_players[MAX_NPC_SLOTS]; /* S470 -- only sampled when kit->has_dance */
static int g_players_init[MAX_NPC_SLOTS];
static int g_slot_kit[MAX_NPC_SLOTS]; /* which kit_index this slot's own players were last bound to */
static float g_prev_x[MAX_NPC_SLOTS];
static float g_prev_z[MAX_NPC_SLOTS];
static int g_has_prev[MAX_NPC_SLOTS];
/* Same real epsilon gband_mesh_rig.c's own MOVE_EPSILON already uses -- kept in sync by hand
   (no shared header between the two siblings), matching how SHANKPIT_GRID_CELL_SIZE is already
   hand-kept in sync across a different real boundary elsewhere in this monorepo. */
#define GBAND_SKEL_NPC_MOVE_EPSILON 0.02f

void gband_skel_npc_draw(int kit_index, int npc_slot, float npc_x, float npc_y, float npc_z, float facing_rad, float dt_ms,
                          int anim_override,
                          const Mat4 *vp,
                          void (*draw_skinned)(const float *verts6, int vert_count,
                                                const Mat4 *mvp, const Mat4 *model)) {
    if (kit_index < 0 || kit_index >= g_kit_count) return;
    GbandSkelNpcKit *kit = &g_kits[kit_index];
    if (!kit->ready) return;
    if (npc_slot < 0 || npc_slot >= MAX_NPC_SLOTS) return;

    /* Real, deliberate re-init on a kit change for this slot -- a respawned NPC assigned a
       different look (or, defensively, any stale slot reuse) must never sample its new kit's
       skeleton with an animation player still bound to the OLD kit's own clip data. Greet/dance
       players are only ever initialized (and later advanced/sampled) when this kit actually has
       a real clip loaded for that slot -- an uninitialized GSeqPlayer over a zeroed GSeqClip is
       never touched. */
    if (!g_players_init[npc_slot] || g_slot_kit[npc_slot] != kit_index) {
        gseq_player_init(&g_idle_players[npc_slot], &kit->idle_seq, &kit->idle_clip);
        gseq_player_init(&g_walk_players[npc_slot], &kit->walk_seq, &kit->walk_clip);
        if (kit->has_greet) gseq_player_init(&g_greet_players[npc_slot], &kit->greet_seq, &kit->greet_clip);
        if (kit->has_dance) gseq_player_init(&g_dance_players[npc_slot], &kit->dance_seq, &kit->dance_clip);
        g_players_init[npc_slot] = 1;
        g_slot_kit[npc_slot] = kit_index;
        g_has_prev[npc_slot] = 0; /* real fresh start -- a position delta across a kit swap isn't a real "walked" signal */
    }

    int walking = 0;
    if (g_has_prev[npc_slot]) {
        float mdx = npc_x - g_prev_x[npc_slot];
        float mdz = npc_z - g_prev_z[npc_slot];
        walking = (mdx * mdx + mdz * mdz) > (GBAND_SKEL_NPC_MOVE_EPSILON * GBAND_SKEL_NPC_MOVE_EPSILON);
    }
    g_prev_x[npc_slot] = npc_x;
    g_prev_z[npc_slot] = npc_z;
    g_has_prev[npc_slot] = 1;

    /* Every real track this kit has always advances, even ones not currently sampled, so
       switching back never causes a visible time-jump (same real reason this file's own header
       comment already names for idle/walk). */
    gseq_player_advance(&g_idle_players[npc_slot], dt_ms / 1000.0f);
    gseq_player_advance(&g_walk_players[npc_slot], dt_ms / 1000.0f);
    if (kit->has_greet) gseq_player_advance(&g_greet_players[npc_slot], dt_ms / 1000.0f);
    if (kit->has_dance) gseq_player_advance(&g_dance_players[npc_slot], dt_ms / 1000.0f);

    /* S470 -- anim_override forces a specific gesture regardless of movement, falling back to
       idle if this particular kit has no real clip loaded for the requested gesture (e.g. the
       founder's own mannequin has a real Dance_Loop but no real greet clip today) rather than
       drawing nothing or silently sampling an uninitialized player. */
    GSeqPlayer *active;
    if (anim_override == GBAND_SKEL_NPC_ANIM_GREET && kit->has_greet) active = &g_greet_players[npc_slot];
    else if (anim_override == GBAND_SKEL_NPC_ANIM_DANCE && kit->has_dance) active = &g_dance_players[npc_slot];
    else if (anim_override != GBAND_SKEL_NPC_ANIM_AUTO) active = &g_idle_players[npc_slot];
    else active = walking ? &g_walk_players[npc_slot] : &g_idle_players[npc_slot];

    float pose_rot[GSKEL_MAX_JOINTS * 4];
    float pose_trans[GSKEL_MAX_JOINTS * 3];
    gseq_player_sample_pose(active, &kit->skel, pose_rot, pose_trans);

    float skin[GSKEL_MAX_JOINTS][16];
    gpose_compute_skin_matrices(&kit->skel, pose_rot, pose_trans, skin);

    /* Bake the NPC's own world transform (position + facing) into every joint's skin matrix,
       the same "vertices already come out world-space" contract gband_mesh_rig_draw's own
       comment documents -- mvp collapses to just vp for the draw call below. */
    Mat4 npc_world_t = mat4_translate(npc_x, npc_y, npc_z);
    Mat4 npc_rot = mat4_rotate_y(facing_rad);
    Mat4 npc_world = mat4_multiply(&npc_world_t, &npc_rot);
    for (uint32_t j = 0; j < kit->skel.joint_count; j++) {
        Mat4 sj;
        memcpy(sj.m, skin[j], sizeof(sj.m));
        Mat4 baked = mat4_multiply(&npc_world, &sj);
        memcpy(skin[j], baked.m, sizeof(baked.m));
    }

    uint32_t vert_count = gpose_skin_mesh(&kit->mesh, skin, kit->out_buf);
    if (vert_count > kit->out_capacity_verts) vert_count = kit->out_capacity_verts; /* defensive */

    Mat4 identity = mat4_identity();
    Mat4 mvp = mat4_multiply(vp, &identity);
    draw_skinned(kit->out_buf, (int)vert_count, &mvp, &identity);
}
