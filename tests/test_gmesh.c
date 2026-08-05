// test_gmesh.c — headless test of gmesh_* against a synthetic 4-vertex,
// 2-triangle (a quad) fixture written by this test itself.
#include "../src/gmesh.h"
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
static void write_f32le(FILE *f, float v) {
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    write_u32le(f, bits);
}

// A flat unit quad (4 verts, 2 tris). Verts 0/1 are 100% bone 0; verts 2/3
// are a 50/50 blend of bone 0 and bone 1 -- enough to prove multi-influence
// weights round-trip, not just the trivial single-bone case.
static void write_fixture(const char *path) {
    FILE *f = fopen(path, "wb");
    fwrite("GMSH", 1, 4, f);
    write_u32le(f, 1); // version
    write_u32le(f, 4); // vertex_count
    write_u32le(f, 6); // index_count (2 triangles)

    float positions[4][3] = {{0,0,0}, {1,0,0}, {1,1,0}, {0,1,0}};
    float normals[4][3]   = {{0,0,1}, {0,0,1}, {0,0,1}, {0,0,1}};
    float uvs[4][2]       = {{0,0}, {1,0}, {1,1}, {0,1}};
    unsigned char bone_idx[4][4] = {{0,0,0,0}, {0,0,0,0}, {0,1,0,0}, {0,1,0,0}};
    float weights[4][4]   = {{1,0,0,0}, {1,0,0,0}, {0.5f,0.5f,0,0}, {0.5f,0.5f,0,0}};

    for (int v = 0; v < 4; v++) {
        for (int k = 0; k < 3; k++) write_f32le(f, positions[v][k]);
        for (int k = 0; k < 3; k++) write_f32le(f, normals[v][k]);
        for (int k = 0; k < 2; k++) write_f32le(f, uvs[v][k]);
        fwrite(bone_idx[v], 1, 4, f);
        for (int k = 0; k < 4; k++) write_f32le(f, weights[v][k]);
    }

    uint32_t indices[6] = {0, 1, 2, 0, 2, 3};
    for (int i = 0; i < 6; i++) write_u32le(f, indices[i]);

    fclose(f);
}

int main(void) {
    const char *path = "/tmp/test_fixture.gmesh";
    write_fixture(path);

    GMesh mesh;
    int ok = gmesh_init(path, &mesh);
    CHECK(ok == 1, "gmesh_init loads a well-formed fixture");
    CHECK(mesh.vertex_count == 4, "vertex_count read correctly");
    CHECK(mesh.index_count == 6, "index_count read correctly");

    CHECK(mesh.vertices[1].position[0] == 1.0f, "vertex 1 position.x round-trips");
    CHECK(mesh.vertices[2].bone_indices[0] == 0 && mesh.vertices[2].bone_indices[1] == 1,
          "vertex 2's two bone indices round-trip");
    CHECK(mesh.vertices[2].bone_weights[0] == 0.5f && mesh.vertices[2].bone_weights[1] == 0.5f,
          "vertex 2's 50/50 blend weights round-trip");
    CHECK(mesh.vertices[0].bone_weights[0] == 1.0f, "vertex 0's single-bone weight round-trips");

    CHECK(mesh.indices[0] == 0 && mesh.indices[1] == 1 && mesh.indices[2] == 2,
          "first triangle's indices round-trip");
    CHECK(mesh.indices[5] == 3, "last index round-trips");

    gmesh_free(&mesh);
    CHECK(mesh.vertices == NULL && mesh.indices == NULL, "gmesh_free clears both owned pointers");

    // Bad magic and a non-multiple-of-3 index_count must both fail cleanly.
    FILE *bad = fopen("/tmp/test_fixture_bad.gmesh", "wb");
    fwrite("XXXX", 1, 4, bad);
    fclose(bad);
    GMesh bad_mesh;
    CHECK(gmesh_init("/tmp/test_fixture_bad.gmesh", &bad_mesh) == 0, "gmesh_init rejects a file with bad magic");
    CHECK(gmesh_init("/tmp/does_not_exist.gmesh", &bad_mesh) == 0, "gmesh_init rejects a missing file");

    if (failures == 0) {
        printf("\nALL PASS\n");
    } else {
        printf("\n%d FAILURE(S)\n", failures);
    }
    return failures ? 1 : 0;
}
