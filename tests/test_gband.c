// test_gband.c — headless test of the gb_* sampler against a synthetic
// .gband fixture written by this test itself (no external asset needed).
#include "../src/gband.h"
#include "../src/sha256.h"
#include <stdio.h>
#include <stdlib.h>
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

// writeFixture builds a 2-channel, 4-tick clip: channel 0 ramps 0..3,
// channel 1 is always 10x channel 0. Content hash computed for real so
// gb_verify has something honest to check.
static void write_fixture(const char *path) {
    const uint32_t duration_ticks = 4, num_channels = 2;
    float data[8]; // duration_ticks * num_channels
    for (uint32_t t = 0; t < duration_ticks; t++) {
        data[t * num_channels + 0] = (float)t;
        data[t * num_channels + 1] = (float)t * 10.0f;
    }
    unsigned char content_hash[32];
    sha256((const unsigned char *)data, sizeof(data), content_hash);

    FILE *f = fopen(path, "wb");
    fwrite("GBND", 1, 4, f);
    write_u32le(f, 1);              // version
    write_u32le(f, 64);             // tick_rate
    write_u32le(f, duration_ticks);
    write_u32le(f, num_channels);
    unsigned char skeleton_hash[32] = {0};
    fwrite(skeleton_hash, 1, 32, f);
    fwrite(content_hash, 1, 32, f);
    fwrite(data, sizeof(float), duration_ticks * num_channels, f);
    fclose(f);
}

int main(void) {
    const char *path = "/tmp/test_fixture.gband";
    write_fixture(path);

    GBClip clip;
    int ok = gb_init(path, &clip);
    CHECK(ok == 1, "gb_init loads a well-formed fixture");
    CHECK(clip.tick_rate == 64, "tick_rate read correctly");
    CHECK(clip.duration_ticks == 4, "duration_ticks read correctly");
    CHECK(clip.num_channels == 2, "num_channels read correctly");

    const float *tick0 = gb_sample(&clip, 0);
    CHECK(tick0[0] == 0.0f && tick0[1] == 0.0f, "tick 0 sample correct");

    const float *tick2 = gb_sample(&clip, 2);
    CHECK(tick2[0] == 2.0f && tick2[1] == 20.0f, "tick 2 sample correct");

    // Out-of-range tick clamps to the last valid tick, doesn't read past the buffer.
    const float *tick_oob = gb_sample(&clip, 999);
    CHECK(tick_oob[0] == 3.0f && tick_oob[1] == 30.0f, "out-of-range tick clamps to last tick");

    float blended[2];
    gb_blend(&clip, 0, 2, 0.5f, blended);
    CHECK(blended[0] == 1.0f && blended[1] == 10.0f, "gb_blend linearly interpolates at w=0.5");

    gb_blend(&clip, 1, 3, 0.0f, blended);
    CHECK(blended[0] == 1.0f && blended[1] == 10.0f, "gb_blend at w=0 equals tick_a exactly");

    CHECK(gb_verify(&clip) == 1, "gb_verify accepts a correctly-hashed fixture");

    // Corrupt one float in memory (not on disk) and confirm verify now fails.
    clip.data[0] = 999.0f;
    CHECK(gb_verify(&clip) == 0, "gb_verify rejects tampered channel data");

    gb_free(&clip);
    CHECK(clip.data == NULL, "gb_free clears the data pointer");

    // A file with a bad magic must fail gb_init cleanly, not crash.
    FILE *bad = fopen("/tmp/test_fixture_bad.gband", "wb");
    fwrite("XXXX", 1, 4, bad);
    fclose(bad);
    GBClip bad_clip;
    int bad_ok = gb_init("/tmp/test_fixture_bad.gband", &bad_clip);
    CHECK(bad_ok == 0, "gb_init rejects a file with bad magic");

    int missing_ok = gb_init("/tmp/does_not_exist.gband", &bad_clip);
    CHECK(missing_ok == 0, "gb_init rejects a missing file");

    if (failures == 0) {
        printf("\nALL PASS\n");
    } else {
        printf("\n%d FAILURE(S)\n", failures);
    }
    return failures ? 1 : 0;
}
