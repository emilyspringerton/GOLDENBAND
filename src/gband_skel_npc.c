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

static GSkel g_skel;
static GMesh g_mesh;
static GSeqClip g_idle_clip, g_walk_clip;
static GSeq g_idle_seq, g_walk_seq;
static int g_ready = 0;

static float *g_out_buf; /* owned, 6 floats per output vertex (pos+normal) */
static uint32_t g_out_capacity_verts;

static void init_single_clip_seq(GSeq *seq) {
    memset(seq, 0, sizeof(*seq));
    seq->steps[0].clip_index = 0;
    seq->steps[0].duration_seconds = -1.0f; /* the clip's own real duration */
    seq->step_count = 1;
    seq->blend_seconds = 0.0f; /* single, looping clip -- nothing to cross-fade into */
    seq->loop = 1;
}

int gband_skel_npc_init(const char *asset_dir, const char *mesh_name, const char *idle_clip_name, const char *walk_clip_name) {
    char path[512], gband_path[512], manifest_path[512];

    snprintf(path, sizeof(path), "%s/%s.gskel", asset_dir, mesh_name);
    if (!gskel_init(path, &g_skel)) return 0;
    snprintf(path, sizeof(path), "%s/%s.gmesh", asset_dir, mesh_name);
    if (!gmesh_init(path, &g_mesh)) return 0;

    snprintf(gband_path, sizeof(gband_path), "%s/%s.gband", asset_dir, idle_clip_name);
    snprintf(manifest_path, sizeof(manifest_path), "%s/%s.gband.json", asset_dir, idle_clip_name);
    if (!gseq_clip_load(gband_path, manifest_path, &g_idle_clip)) { gmesh_free(&g_mesh); return 0; }

    snprintf(gband_path, sizeof(gband_path), "%s/%s.gband", asset_dir, walk_clip_name);
    snprintf(manifest_path, sizeof(manifest_path), "%s/%s.gband.json", asset_dir, walk_clip_name);
    if (!gseq_clip_load(gband_path, manifest_path, &g_walk_clip)) { gseq_clip_free(&g_idle_clip); gmesh_free(&g_mesh); return 0; }

    init_single_clip_seq(&g_idle_seq);
    init_single_clip_seq(&g_walk_seq);

    g_out_capacity_verts = g_mesh.index_count;
    g_out_buf = (float *)malloc((size_t)g_out_capacity_verts * 6 * sizeof(float));
    if (!g_out_buf) { gseq_clip_free(&g_idle_clip); gseq_clip_free(&g_walk_clip); gmesh_free(&g_mesh); return 0; }

    g_ready = 1;
    return 1;
}

void gband_skel_npc_shutdown(void) {
    if (!g_ready) return;
    gseq_clip_free(&g_idle_clip);
    gseq_clip_free(&g_walk_clip);
    gmesh_free(&g_mesh);
    free(g_out_buf);
    g_out_buf = NULL;
    g_ready = 0;
}

int gband_skel_npc_ready(void) { return g_ready; }

#define MAX_NPC_SLOTS 32
static GSeqPlayer g_idle_players[MAX_NPC_SLOTS];
static GSeqPlayer g_walk_players[MAX_NPC_SLOTS];
static int g_players_init[MAX_NPC_SLOTS];
static float g_prev_x[MAX_NPC_SLOTS];
static float g_prev_z[MAX_NPC_SLOTS];
static int g_has_prev[MAX_NPC_SLOTS];
/* Same real epsilon gband_mesh_rig.c's own MOVE_EPSILON already uses -- kept in sync by hand
   (no shared header between the two siblings), matching how SHANKPIT_GRID_CELL_SIZE is already
   hand-kept in sync across a different real boundary elsewhere in this monorepo. */
#define GBAND_SKEL_NPC_MOVE_EPSILON 0.02f

void gband_skel_npc_draw(int npc_slot, float npc_x, float npc_y, float npc_z, float facing_rad, float dt_ms,
                          const Mat4 *vp,
                          void (*draw_skinned)(const float *verts6, int vert_count,
                                                const Mat4 *mvp, const Mat4 *model)) {
    if (!g_ready) return;
    if (npc_slot < 0 || npc_slot >= MAX_NPC_SLOTS) return;

    if (!g_players_init[npc_slot]) {
        gseq_player_init(&g_idle_players[npc_slot], &g_idle_seq, &g_idle_clip);
        gseq_player_init(&g_walk_players[npc_slot], &g_walk_seq, &g_walk_clip);
        g_players_init[npc_slot] = 1;
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

    /* Both tracks always advance, even the one not currently sampled, so switching back never
       causes a visible time-jump (same real reason this file's own header comment names). */
    gseq_player_advance(&g_idle_players[npc_slot], dt_ms / 1000.0f);
    gseq_player_advance(&g_walk_players[npc_slot], dt_ms / 1000.0f);

    GSeqPlayer *active = walking ? &g_walk_players[npc_slot] : &g_idle_players[npc_slot];

    float pose_rot[GSKEL_MAX_JOINTS * 4];
    float pose_trans[GSKEL_MAX_JOINTS * 3];
    gseq_player_sample_pose(active, &g_skel, pose_rot, pose_trans);

    float skin[GSKEL_MAX_JOINTS][16];
    gpose_compute_skin_matrices(&g_skel, pose_rot, pose_trans, skin);

    /* Bake the NPC's own world transform (position + facing) into every joint's skin matrix,
       the same "vertices already come out world-space" contract gband_mesh_rig_draw's own
       comment documents -- mvp collapses to just vp for the draw call below. */
    Mat4 npc_world_t = mat4_translate(npc_x, npc_y, npc_z);
    Mat4 npc_rot = mat4_rotate_y(facing_rad);
    Mat4 npc_world = mat4_multiply(&npc_world_t, &npc_rot);
    for (uint32_t j = 0; j < g_skel.joint_count; j++) {
        Mat4 sj;
        memcpy(sj.m, skin[j], sizeof(sj.m));
        Mat4 baked = mat4_multiply(&npc_world, &sj);
        memcpy(skin[j], baked.m, sizeof(baked.m));
    }

    uint32_t vert_count = gpose_skin_mesh(&g_mesh, skin, g_out_buf);
    if (vert_count > g_out_capacity_verts) vert_count = g_out_capacity_verts; /* defensive */

    Mat4 identity = mat4_identity();
    Mat4 mvp = mat4_multiply(vp, &identity);
    draw_skinned(g_out_buf, (int)vert_count, &mvp, &identity);
}
