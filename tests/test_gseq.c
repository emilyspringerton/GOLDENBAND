// test_gseq.c — headless test of gseq_* (animation stitching/crossfade) against synthetic
// fixtures this test itself writes. Two tiny clips, one joint ("root"), a two-step sequence
// (clip A -> clip B) verifying: pure single-clip sampling, real crossfade blending (start/mid/
// end of the transition window), and missing-channel fallback to the skeleton's own rest pose.
#include "../src/gseq.h"
#include "../src/sha256.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;
#define CHECK(cond, label) do { \
    if (cond) { printf("PASS: %s\n", label); } \
    else { printf("FAIL: %s\n", label); failures++; } \
} while (0)

static void write_u32le(FILE *f, uint32_t v) {
    unsigned char b[4] = {
        (unsigned char)(v & 0xFF), (unsigned char)((v >> 8) & 0xFF),
        (unsigned char)((v >> 16) & 0xFF), (unsigned char)((v >> 24) & 0xFF)
    };
    fwrite(b, 1, 4, f);
}

// write_clip writes a real, 1-second, 10-tick clip with a single quaternion channel group
// ("root.qx/.qy/.qz/.qw") that stays a fixed identity-ish quaternion the whole clip (simplest
// real fixture that still exercises real channel lookup + blend math without needing a real
// rotation to reason about by hand).
static void write_clip(const char *path, float qx, float qy, float qz, float qw) {
    const uint32_t duration_ticks = 10, num_channels = 4, tick_rate = 10;
    float data[10 * 4];
    for (uint32_t t = 0; t < duration_ticks; t++) {
        data[t * num_channels + 0] = qx;
        data[t * num_channels + 1] = qy;
        data[t * num_channels + 2] = qz;
        data[t * num_channels + 3] = qw;
    }
    unsigned char content_hash[32];
    sha256((const unsigned char *)data, sizeof(data), content_hash);

    FILE *f = fopen(path, "wb");
    fwrite("GBND", 1, 4, f);
    write_u32le(f, 1);
    write_u32le(f, tick_rate);
    write_u32le(f, duration_ticks);
    write_u32le(f, num_channels);
    unsigned char skeleton_hash[32] = {0};
    fwrite(skeleton_hash, 1, 32, f);
    fwrite(content_hash, 1, 32, f);
    fwrite(data, sizeof(float), duration_ticks * num_channels, f);
    fclose(f);
}

static void write_manifest(const char *path) {
    FILE *f = fopen(path, "wb");
    fprintf(f, "{\n  \"channels\": [\"root.qx\", \"root.qy\", \"root.qz\", \"root.qw\"]\n}\n");
    fclose(f);
}

int main(void) {
    // Clip A: a fixed, real 90deg-about-Z quaternion. Clip B: real identity.
    float half = (float)sqrt(0.5);
    write_clip("/tmp/test_gseq_a.gband", 0.0f, 0.0f, half, half);
    write_manifest("/tmp/test_gseq_a.gband.json");
    write_clip("/tmp/test_gseq_b.gband", 0.0f, 0.0f, 0.0f, 1.0f);
    write_manifest("/tmp/test_gseq_b.gband.json");

    GSeqClip clips[2];
    int okA = gseq_clip_load("/tmp/test_gseq_a.gband", "/tmp/test_gseq_a.gband.json", &clips[0]);
    int okB = gseq_clip_load("/tmp/test_gseq_b.gband", "/tmp/test_gseq_b.gband.json", &clips[1]);
    CHECK(okA == 1, "gseq_clip_load loads clip A");
    CHECK(okB == 1, "gseq_clip_load loads clip B");
    CHECK(clips[0].channel_count == 4, "clip A channel_count read from manifest");
    CHECK(strcmp(clips[0].channel_names[3], "root.qw") == 0, "clip A channel name[3] == root.qw");

    // A minimal, real 1-joint skeleton -- "root" matches the clips' own channel prefix; "other"
    // has no matching channel in either clip, so it must fall back to its own real rest pose.
    GSkel skel;
    memset(&skel, 0, sizeof(skel));
    skel.version = 1;
    skel.joint_count = 2;
    strncpy(skel.joints[0].name, "root", GSKEL_NAME_LEN - 1);
    skel.joints[0].parent_index = -1;
    skel.joints[0].rest_rotation[3] = 1.0f; // identity rest, irrelevant since root has real channels
    strncpy(skel.joints[1].name, "other", GSKEL_NAME_LEN - 1);
    skel.joints[1].parent_index = -1;
    skel.joints[1].rest_rotation[0] = 0.25f; // a real, distinctive rest value to assert on
    skel.joints[1].rest_rotation[3] = 0.9f;
    skel.joints[1].rest_translation[1] = 3.5f;

    GSeq seq;
    memset(&seq, 0, sizeof(seq));
    seq.steps[0].clip_index = 0; // A
    seq.steps[1].clip_index = 1; // B
    seq.step_count = 2;
    seq.blend_seconds = 0.2f;
    seq.loop = 0;

    GSeqPlayer player;
    gseq_player_init(&player, &seq, clips);

    float rot[2 * 4], trans[2 * 3];

    // t=0: pure clip A, no blend yet.
    gseq_player_sample_pose(&player, &skel, rot, trans);
    CHECK(fabsf(rot[2] - half) < 1e-4f && fabsf(rot[3] - half) < 1e-4f,
          "t=0: root pose is pure clip A (90deg about Z)");
    CHECK(fabsf(rot[4 + 0] - 0.25f) < 1e-6f && fabsf(rot[4 + 3] - 0.9f) < 1e-6f,
          "t=0: 'other' joint (no matching channel) falls back to its own rest rotation");
    CHECK(fabsf(trans[3 + 1] - 3.5f) < 1e-6f,
          "t=0: 'other' joint's rest translation also used (no matching channel)");

    // Advance exactly to clip A's own real end (1.0s) -- crossfade should just have started.
    gseq_player_advance(&player, 1.0f);
    CHECK(player.current_step == 1, "after 1.0s, sequence advanced to step 1 (clip B)");
    CHECK(player.blending == 1, "a crossfade is active right at the transition");

    gseq_player_sample_pose(&player, &skel, rot, trans);
    CHECK(fabsf(rot[2] - half) < 1e-3f && fabsf(rot[3] - half) < 1e-3f,
          "blend_elapsed=0: root pose ~= clip A's own final pose (crossfade just started)");

    // Halfway through the crossfade: a real nlerp midpoint between A's final pose and B's first
    // pose, renormalized -- NOT simply (qA+qB)/2 unnormalized.
    gseq_player_advance(&player, 0.1f); // blend_elapsed now 0.1 of 0.2 -- the real midpoint
    gseq_player_sample_pose(&player, &skel, rot, trans);
    float mag = sqrtf(rot[0]*rot[0] + rot[1]*rot[1] + rot[2]*rot[2] + rot[3]*rot[3]);
    CHECK(fabsf(mag - 1.0f) < 1e-4f, "mid-crossfade: root quaternion is real, renormalized (unit magnitude)");
    CHECK(rot[2] > 0.0f && rot[2] < half, "mid-crossfade: root.qz is a real, genuine midpoint between A and B, not either endpoint");

    // Advance well past the crossfade window -- pure clip B now.
    gseq_player_advance(&player, 1.0f);
    CHECK(player.blending == 0, "crossfade ends once blend_seconds has elapsed");
    gseq_player_sample_pose(&player, &skel, rot, trans);
    CHECK(fabsf(rot[2]) < 1e-3f && fabsf(rot[3] - 1.0f) < 1e-3f,
          "well after the crossfade: root pose is pure clip B (identity)");

    // No loop -- holds the final pose forever past the end, doesn't wrap or crash.
    gseq_player_advance(&player, 100.0f);
    CHECK(player.current_step == 1, "non-looping sequence holds at its own last step");
    gseq_player_sample_pose(&player, &skel, rot, trans);
    CHECK(fabsf(rot[3] - 1.0f) < 1e-3f, "held final pose is still real and correct");

    gseq_clip_free(&clips[0]);
    gseq_clip_free(&clips[1]);

    if (failures == 0) { printf("\nALL PASS\n"); return 0; }
    printf("\n%d FAILURE(S)\n", failures);
    return 1;
}
