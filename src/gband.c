// gband.c — see gband.h. This file is the acceptance test itself: it must
// stay small enough to read in one sitting (HQ-SPEC-SIM-100 §8: "a hundred
// line parser is the acceptance test").
#include "gband.h"
#include "sha256.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GBAND_HEADER_SIZE 84

static uint32_t read_u32le(const unsigned char *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

int gb_init(const char *path, GBClip *clip) {
    memset(clip, 0, sizeof(*clip));

    FILE *f = fopen(path, "rb");
    if (!f) return 0;

    unsigned char header[GBAND_HEADER_SIZE];
    if (fread(header, 1, GBAND_HEADER_SIZE, f) != GBAND_HEADER_SIZE) {
        fclose(f);
        return 0;
    }
    if (memcmp(header, "GBND", 4) != 0) {
        fclose(f);
        return 0;
    }

    clip->version = read_u32le(header + 4);
    clip->tick_rate = read_u32le(header + 8);
    clip->duration_ticks = read_u32le(header + 12);
    clip->num_channels = read_u32le(header + 16);
    memcpy(clip->skeleton_hash, header + 20, 32);
    memcpy(clip->content_hash, header + 52, 32);

    if (clip->duration_ticks == 0 || clip->num_channels == 0) {
        fclose(f);
        return 0;
    }

    size_t n_floats = (size_t)clip->duration_ticks * (size_t)clip->num_channels;
    clip->data = (float *)malloc(n_floats * sizeof(float));
    if (!clip->data) {
        fclose(f);
        return 0;
    }

    // Raw float32 read: this repo's monorepo-wide convention (see
    // packages/common/protocol.h elsewhere) assumes a little-endian host
    // with IEEE754 float32, so no per-value byte-swapping is done here.
    if (fread(clip->data, sizeof(float), n_floats, f) != n_floats) {
        free(clip->data);
        clip->data = NULL;
        fclose(f);
        return 0;
    }

    fclose(f);
    return 1;
}

void gb_free(GBClip *clip) {
    free(clip->data);
    clip->data = NULL;
}

const float *gb_sample(const GBClip *clip, uint32_t tick) {
    if (tick >= clip->duration_ticks) tick = clip->duration_ticks - 1;
    return clip->data + (size_t)tick * (size_t)clip->num_channels;
}

void gb_blend(const GBClip *clip, uint32_t tick_a, uint32_t tick_b, float w, float *out) {
    const float *a = gb_sample(clip, tick_a);
    const float *b = gb_sample(clip, tick_b);
    for (uint32_t i = 0; i < clip->num_channels; i++) {
        out[i] = a[i] + (b[i] - a[i]) * w;
    }
}

int gb_verify(const GBClip *clip) {
    size_t n_bytes = (size_t)clip->duration_ticks * (size_t)clip->num_channels * sizeof(float);
    unsigned char computed[32];
    sha256((const unsigned char *)clip->data, n_bytes, computed);
    return memcmp(computed, clip->content_hash, 32) == 0;
}
