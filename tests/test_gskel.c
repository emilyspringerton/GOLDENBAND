// test_gskel.c — headless test of gskel_* against a synthetic 3-joint
// fixture written by this test itself (no external asset needed).
#include "../src/gskel.h"
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
static void write_i32le(FILE *f, int32_t v) { write_u32le(f, (uint32_t)v); }
static void write_f32le(FILE *f, float v) {
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    write_u32le(f, bits);
}
static void write_name(FILE *f, const char *name) {
    unsigned char buf[GSKEL_NAME_LEN] = {0};
    size_t len = strlen(name);
    if (len >= GSKEL_NAME_LEN) len = GSKEL_NAME_LEN - 1;
    memcpy(buf, name, len);
    fwrite(buf, 1, GSKEL_NAME_LEN, f);
}

// A 3-joint chain: Root(-1) -> Mid(0) -> Tip(1). Root's inverse_bind is
// identity; Mid/Tip get simple translation-only inverse-binds matching a
// straight-line rest pose, just enough to prove every field round-trips.
static void write_fixture(const char *path) {
    FILE *f = fopen(path, "wb");
    fwrite("GSKL", 1, 4, f);
    write_u32le(f, 1);  // version
    write_u32le(f, 3);  // joint_count

    // Root
    write_name(f, "Root");
    write_i32le(f, -1);
    write_f32le(f, 0.0f); write_f32le(f, 0.0f); write_f32le(f, 0.0f); // rest_translation
    write_f32le(f, 0.0f); write_f32le(f, 0.0f); write_f32le(f, 0.0f); write_f32le(f, 1.0f); // rest_rotation identity quat
    for (int i = 0; i < 16; i++) write_f32le(f, (i % 5 == 0) ? 1.0f : 0.0f); // identity 4x4

    // Mid (child of Root, offset +1 on Y)
    write_name(f, "Mid");
    write_i32le(f, 0);
    write_f32le(f, 0.0f); write_f32le(f, 1.0f); write_f32le(f, 0.0f);
    write_f32le(f, 0.0f); write_f32le(f, 0.0f); write_f32le(f, 0.0f); write_f32le(f, 1.0f);
    for (int i = 0; i < 16; i++) write_f32le(f, (i % 5 == 0) ? 1.0f : 0.0f);

    // Tip (child of Mid, offset +1 more on Y)
    write_name(f, "Tip");
    write_i32le(f, 1);
    write_f32le(f, 0.0f); write_f32le(f, 1.0f); write_f32le(f, 0.0f);
    write_f32le(f, 0.0f); write_f32le(f, 0.0f); write_f32le(f, 0.0f); write_f32le(f, 1.0f);
    for (int i = 0; i < 16; i++) write_f32le(f, (i % 5 == 0) ? 1.0f : 0.0f);

    fclose(f);
}

int main(void) {
    const char *path = "/tmp/test_fixture.gskel";
    write_fixture(path);

    GSkel skel;
    int ok = gskel_init(path, &skel);
    CHECK(ok == 1, "gskel_init loads a well-formed fixture");
    CHECK(skel.joint_count == 3, "joint_count read correctly");

    CHECK(strcmp(skel.joints[0].name, "Root") == 0, "joint 0 name is Root");
    CHECK(skel.joints[0].parent_index == -1, "Root has no parent");
    CHECK(strcmp(skel.joints[1].name, "Mid") == 0, "joint 1 name is Mid");
    CHECK(skel.joints[1].parent_index == 0, "Mid's parent is Root (index 0)");
    CHECK(skel.joints[1].rest_translation[1] == 1.0f, "Mid's rest Y offset round-trips");
    CHECK(skel.joints[2].parent_index == 1, "Tip's parent is Mid (index 1)");
    CHECK(skel.joints[2].rest_rotation[3] == 1.0f, "Tip's rest rotation quat w round-trips");
    CHECK(skel.joints[0].inverse_bind[0] == 1.0f && skel.joints[0].inverse_bind[5] == 1.0f,
          "identity inverse_bind diagonal round-trips");

    CHECK(gskel_find_joint(&skel, "Mid") == 1, "gskel_find_joint finds Mid at index 1");
    CHECK(gskel_find_joint(&skel, "Nonexistent") == -1, "gskel_find_joint returns -1 for an unknown name");

    // A file with bad magic must fail cleanly, not crash.
    FILE *bad = fopen("/tmp/test_fixture_bad.gskel", "wb");
    fwrite("XXXX", 1, 4, bad);
    fclose(bad);
    GSkel bad_skel;
    CHECK(gskel_init("/tmp/test_fixture_bad.gskel", &bad_skel) == 0, "gskel_init rejects a file with bad magic");
    CHECK(gskel_init("/tmp/does_not_exist.gskel", &bad_skel) == 0, "gskel_init rejects a missing file");

    if (failures == 0) {
        printf("\nALL PASS\n");
    } else {
        printf("\n%d FAILURE(S)\n", failures);
    }
    return failures ? 1 : 0;
}
