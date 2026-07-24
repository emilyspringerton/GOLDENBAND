// gband.h — the GOLDEN BAND runtime sampler (HQ-SPEC-SIM-100 §3/§8 step 1).
//
// Deliberately tiny: this is the "fits in a hundred lines of C" acceptance
// test from the spec. No JSON, no curve interpolation, no engine
// dependency — see format/GBAND_FORMAT.md for the full binary layout this
// reads. Determinism is load-bearing: same clip + same tick = the same
// float32 bytes, every platform, every time.
#ifndef GOLDENBAND_GBAND_H
#define GOLDENBAND_GBAND_H

#include <stdint.h>

typedef struct {
    uint32_t version;
    uint32_t tick_rate;
    uint32_t duration_ticks;
    uint32_t num_channels;
    unsigned char skeleton_hash[32];
    unsigned char content_hash[32];
    float *data; // owned; duration_ticks * num_channels floats, row-major by tick
} GBClip;

// gb_init loads and structurally validates a .gband file. Returns 1 on
// success, 0 on any failure (bad magic, truncated file, alloc failure).
// On success, caller must eventually call gb_free.
int gb_init(const char *path, GBClip *clip);

// gb_free releases the clip's owned channel data. Safe to call on a
// zeroed/failed-init clip.
void gb_free(GBClip *clip);

// gb_sample returns a pointer to num_channels floats for the given tick.
// Out-of-range ticks clamp to the last valid tick rather than reading past
// the buffer -- callers that need looping should wrap `tick` themselves
// using clip->duration_ticks and the manifest's loop_points.
const float *gb_sample(const GBClip *clip, uint32_t tick);

// gb_blend linearly interpolates between two exact ticks into a
// caller-owned `out` buffer of at least num_channels floats. This is for
// authoring/reward-compiler use (e.g. sub-tick reward evaluation), not for
// render-rate smoothness -- per HQ-SPEC-SIM-100 §2, render smoothing comes
// from the consumer's own state-interpolation layer, never from
// re-sampling here.
void gb_blend(const GBClip *clip, uint32_t tick_a, uint32_t tick_b, float w, float *out);

// gb_verify recomputes the sha256 over the clip's channel data and
// compares it against the content_hash read from the file. Returns 1 if
// they match, 0 otherwise (corrupt file, or a hash that was never set).
int gb_verify(const GBClip *clip);

#endif // GOLDENBAND_GBAND_H
