// gskel.c — see gskel.h. Manual byte-offset parsing (not fread-into-struct)
// for the same reason gband.c's own header parsing does this: no reliance
// on the compiler's struct layout matching the file layout, only on the
// documented little-endian/IEEE754 host assumption this whole monorepo
// already makes (see gband.c's own "Raw float32 read" comment).
#include "gskel.h"

#include <stdio.h>
#include <string.h>

#define GSKEL_HEADER_SIZE 12
#define GSKEL_JOINT_RECORD_SIZE 128

static uint32_t read_u32le(const unsigned char *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static int32_t read_i32le(const unsigned char *p) { return (int32_t)read_u32le(p); }
static float read_f32le(const unsigned char *p) {
    uint32_t bits = read_u32le(p);
    float f;
    memcpy(&f, &bits, sizeof(f));
    return f;
}

int gskel_init(const char *path, GSkel *skel) {
    memset(skel, 0, sizeof(*skel));

    FILE *f = fopen(path, "rb");
    if (!f) return 0;

    unsigned char header[GSKEL_HEADER_SIZE];
    if (fread(header, 1, GSKEL_HEADER_SIZE, f) != GSKEL_HEADER_SIZE) { fclose(f); return 0; }
    if (memcmp(header, "GSKL", 4) != 0) { fclose(f); return 0; }

    skel->version = read_u32le(header + 4);
    skel->joint_count = read_u32le(header + 8);
    if (skel->joint_count == 0 || skel->joint_count > GSKEL_MAX_JOINTS) { fclose(f); return 0; }

    unsigned char rec[GSKEL_JOINT_RECORD_SIZE];
    for (uint32_t j = 0; j < skel->joint_count; j++) {
        if (fread(rec, 1, GSKEL_JOINT_RECORD_SIZE, f) != GSKEL_JOINT_RECORD_SIZE) {
            fclose(f);
            return 0;
        }
        GSkelJoint *joint = &skel->joints[j];
        memcpy(joint->name, rec, GSKEL_NAME_LEN);
        joint->name[GSKEL_NAME_LEN - 1] = '\0'; // guarantee termination even if the file didn't
        joint->parent_index = read_i32le(rec + 32);
        for (int k = 0; k < 3; k++) joint->rest_translation[k] = read_f32le(rec + 36 + k * 4);
        for (int k = 0; k < 4; k++) joint->rest_rotation[k] = read_f32le(rec + 48 + k * 4);
        for (int k = 0; k < 16; k++) joint->inverse_bind[k] = read_f32le(rec + 64 + k * 4);
    }

    fclose(f);
    return 1;
}

int gskel_find_joint(const GSkel *skel, const char *name) {
    for (uint32_t j = 0; j < skel->joint_count; j++) {
        if (strncmp(skel->joints[j].name, name, GSKEL_NAME_LEN) == 0) return (int)j;
    }
    return -1;
}
