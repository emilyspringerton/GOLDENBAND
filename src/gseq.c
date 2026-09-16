// gseq.c — see gseq.h.
#include "gseq.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// gseq_read_channel_names -- a real, minimal scanner scoped to exactly one known shape:
// `"channels": [ "a.qx", "a.qy", ... ]` inside a real .gband.json manifest. Not a general JSON
// parser (same deliberate scope as SHANKPIT/packages/world/level_boxes.h's own scanner). Returns
// the real channel count found, or -1 on any structural failure.
static int gseq_read_channel_names(const char *json, char names[GSEQ_MAX_CHANNELS][GSEQ_CHANNEL_NAME_LEN]) {
    const char *key = strstr(json, "\"channels\"");
    if (!key) return -1;
    const char *arr = strchr(key, '[');
    if (!arr) return -1;
    const char *arr_end = strchr(arr, ']');
    if (!arr_end) return -1;

    int count = 0;
    const char *p = arr + 1;
    while (p < arr_end && count < GSEQ_MAX_CHANNELS) {
        while (p < arr_end && *p != '"') p++;
        if (p >= arr_end) break;
        p++; // skip opening quote
        const char *start = p;
        while (p < arr_end && *p != '"') p++;
        if (p >= arr_end) break;
        size_t len = (size_t)(p - start);
        if (len >= GSEQ_CHANNEL_NAME_LEN) len = GSEQ_CHANNEL_NAME_LEN - 1;
        memcpy(names[count], start, len);
        names[count][len] = '\0';
        count++;
        p++; // skip closing quote
    }
    return count;
}

int gseq_clip_load(const char *gband_path, const char *manifest_json_path, GSeqClip *out) {
    memset(out, 0, sizeof(*out));
    if (!gb_init(gband_path, &out->clip)) return 0;

    FILE *f = fopen(manifest_json_path, "rb");
    if (!f) { gb_free(&out->clip); return 0; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > 1 << 20) { fclose(f); gb_free(&out->clip); return 0; }
    char *buf = (char *)malloc((size_t)sz + 1);
    if (!buf) { fclose(f); gb_free(&out->clip); return 0; }
    size_t n = fread(buf, 1, (size_t)sz, f);
    fclose(f);
    buf[n] = '\0';

    int count = gseq_read_channel_names(buf, out->channel_names);
    free(buf);
    if (count < 0 || (uint32_t)count != out->clip.num_channels) {
        gb_free(&out->clip);
        return 0;
    }
    out->channel_count = (uint32_t)count;
    return 1;
}

void gseq_clip_free(GSeqClip *c) {
    gb_free(&c->clip);
    memset(c, 0, sizeof(*c));
}

// gseq_find_channel returns the channel index matching "<joint_name><suffix>" in c, or -1.
static int gseq_find_channel(const GSeqClip *c, const char *joint_name, const char *suffix) {
    char want[GSEQ_CHANNEL_NAME_LEN];
    snprintf(want, sizeof(want), "%s%s", joint_name, suffix);
    for (uint32_t i = 0; i < c->channel_count; i++) {
        if (strcmp(c->channel_names[i], want) == 0) return (int)i;
    }
    return -1;
}

// gseq_sample_joint resolves ONE joint's real quaternion + translation from clip at real
// `seconds` into the clip, via gb_blend's own real intra-clip lerp for a smooth sub-tick sample
// (renormalized here, same nlerp discipline GBAND_FORMAT.md's own "Quaternion channels" section
// already documents for gb_blend). Falls back to the skeleton's own real rest pose for any
// missing channel, matching GSKEL_FORMAT.md's documented convention exactly.
static void gseq_sample_joint(const GSeqClip *c, const GSkelJoint *joint, float seconds,
                               float *out_quat, float *out_trans) {
    int qx = gseq_find_channel(c, joint->name, ".qx");
    int qy = gseq_find_channel(c, joint->name, ".qy");
    int qz = gseq_find_channel(c, joint->name, ".qz");
    int qw = gseq_find_channel(c, joint->name, ".qw");
    if (qx < 0 || qy < 0 || qz < 0 || qw < 0) {
        memcpy(out_quat, joint->rest_rotation, sizeof(float) * 4);
    } else {
        float tick_f = seconds * (float)c->clip.tick_rate;
        if (tick_f < 0.0f) tick_f = 0.0f;
        uint32_t max_tick = c->clip.duration_ticks > 0 ? c->clip.duration_ticks - 1 : 0;
        uint32_t ta = (uint32_t)tick_f;
        if (ta > max_tick) ta = max_tick;
        uint32_t tb = ta + 1 > max_tick ? max_tick : ta + 1;
        float w = tick_f - (float)ta;
        if (w < 0.0f) w = 0.0f;
        if (w > 1.0f) w = 1.0f;
        float blended[GSEQ_MAX_CHANNELS];
        gb_blend(&c->clip, ta, tb, w, blended);
        float qxv = blended[qx], qyv = blended[qy], qzv = blended[qz], qwv = blended[qw];
        float mag = sqrtf(qxv * qxv + qyv * qyv + qzv * qzv + qwv * qwv);
        if (mag > 1e-9f) {
            out_quat[0] = qxv / mag; out_quat[1] = qyv / mag;
            out_quat[2] = qzv / mag; out_quat[3] = qwv / mag;
        } else {
            out_quat[0] = 0.0f; out_quat[1] = 0.0f; out_quat[2] = 0.0f; out_quat[3] = 1.0f;
        }
    }

    int tx = gseq_find_channel(c, joint->name, ".tx");
    int ty = gseq_find_channel(c, joint->name, ".ty");
    int tz = gseq_find_channel(c, joint->name, ".tz");
    if (tx < 0 || ty < 0 || tz < 0) {
        memcpy(out_trans, joint->rest_translation, sizeof(float) * 3);
    } else {
        float tick_f = seconds * (float)c->clip.tick_rate;
        if (tick_f < 0.0f) tick_f = 0.0f;
        uint32_t max_tick = c->clip.duration_ticks > 0 ? c->clip.duration_ticks - 1 : 0;
        uint32_t ta = (uint32_t)tick_f;
        if (ta > max_tick) ta = max_tick;
        uint32_t tb = ta + 1 > max_tick ? max_tick : ta + 1;
        float w = tick_f - (float)ta;
        if (w < 0.0f) w = 0.0f;
        if (w > 1.0f) w = 1.0f;
        float blended[GSEQ_MAX_CHANNELS];
        gb_blend(&c->clip, ta, tb, w, blended);
        out_trans[0] = blended[tx]; out_trans[1] = blended[ty]; out_trans[2] = blended[tz];
    }
}

static float gseq_step_duration(const GSeq *seq, const GSeqClip *clips, int step_idx) {
    const GSeqStep *step = &seq->steps[step_idx];
    if (step->duration_seconds > 0.0f) return step->duration_seconds;
    const GBClip *c = &clips[step->clip_index].clip;
    if (c->tick_rate == 0) return 0.0f;
    return (float)c->duration_ticks / (float)c->tick_rate;
}

void gseq_player_init(GSeqPlayer *p, const GSeq *seq, const GSeqClip *clips) {
    memset(p, 0, sizeof(*p));
    p->seq = seq;
    p->clips = clips;
}

void gseq_player_advance(GSeqPlayer *p, float dt_seconds) {
    if (!p->seq || p->seq->step_count <= 0) return;

    p->elapsed_in_step += dt_seconds;
    if (p->blending) {
        p->blend_elapsed += dt_seconds;
        if (p->blend_elapsed >= p->seq->blend_seconds) p->blending = 0;
    }

    float dur = gseq_step_duration(p->seq, p->clips, p->current_step);
    if (dur > 0.0f && p->elapsed_in_step >= dur) {
        int next = p->current_step + 1;
        if (next >= p->seq->step_count) {
            if (!p->seq->loop) {
                // Hold the final pose forever -- clamp, don't keep accumulating past it.
                p->elapsed_in_step = dur;
                return;
            }
            next = 0;
        }
        p->blending = p->seq->blend_seconds > 0.0f;
        p->blend_from_step = p->current_step;
        p->blend_elapsed = 0.0f;
        p->current_step = next;
        p->elapsed_in_step = 0.0f;
    }
}

void gseq_player_sample_pose(const GSeqPlayer *p, const GSkel *skel, float *out_rot, float *out_trans) {
    const GSeqClip *to_clip = &p->clips[p->seq->steps[p->current_step].clip_index];

    for (uint32_t j = 0; j < skel->joint_count; j++) {
        const GSkelJoint *joint = &skel->joints[j];
        float to_quat[4], to_trans[3];
        gseq_sample_joint(to_clip, joint, p->elapsed_in_step, to_quat, to_trans);

        if (p->blending) {
            const GSeqClip *from_clip = &p->clips[p->seq->steps[p->blend_from_step].clip_index];
            float from_dur = gseq_step_duration(p->seq, p->clips, p->blend_from_step);
            float from_quat[4], from_trans[3];
            // Freeze the outgoing clip at its own real final pose -- a crossfade blends INTO
            // the new step from where the old one ENDED, not from wherever it would have kept
            // sampling had it kept playing past its own real duration.
            gseq_sample_joint(from_clip, joint, from_dur, from_quat, from_trans);

            float w = p->seq->blend_seconds > 0.0f ? p->blend_elapsed / p->seq->blend_seconds : 1.0f;
            if (w < 0.0f) w = 0.0f;
            if (w > 1.0f) w = 1.0f;

            float qx = from_quat[0] + (to_quat[0] - from_quat[0]) * w;
            float qy = from_quat[1] + (to_quat[1] - from_quat[1]) * w;
            float qz = from_quat[2] + (to_quat[2] - from_quat[2]) * w;
            float qw = from_quat[3] + (to_quat[3] - from_quat[3]) * w;
            float mag = sqrtf(qx * qx + qy * qy + qz * qz + qw * qw);
            if (mag > 1e-9f) {
                out_rot[j * 4 + 0] = qx / mag; out_rot[j * 4 + 1] = qy / mag;
                out_rot[j * 4 + 2] = qz / mag; out_rot[j * 4 + 3] = qw / mag;
            } else {
                out_rot[j * 4 + 0] = 0.0f; out_rot[j * 4 + 1] = 0.0f;
                out_rot[j * 4 + 2] = 0.0f; out_rot[j * 4 + 3] = 1.0f;
            }
            out_trans[j * 3 + 0] = from_trans[0] + (to_trans[0] - from_trans[0]) * w;
            out_trans[j * 3 + 1] = from_trans[1] + (to_trans[1] - from_trans[1]) * w;
            out_trans[j * 3 + 2] = from_trans[2] + (to_trans[2] - from_trans[2]) * w;
        } else {
            out_rot[j * 4 + 0] = to_quat[0]; out_rot[j * 4 + 1] = to_quat[1];
            out_rot[j * 4 + 2] = to_quat[2]; out_rot[j * 4 + 3] = to_quat[3];
            out_trans[j * 3 + 0] = to_trans[0]; out_trans[j * 3 + 1] = to_trans[1]; out_trans[j * 3 + 2] = to_trans[2];
        }
    }
}
