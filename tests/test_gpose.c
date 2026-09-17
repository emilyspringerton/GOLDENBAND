// test_gpose.c — headless test of the general N-joint FK+skinning path (gpose.c) against a
// synthetic 2-joint skeleton + 1-vertex mesh built in-memory (no asset files needed).
#include "../src/gpose.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;
#define CHECK(cond, label) do { \
    if (cond) { printf("PASS: %s\n", label); } \
    else { printf("FAIL: %s\n", label); failures++; } \
} while (0)

static int near(float a, float b) { return fabsf(a - b) < 1e-4f; }

// A real 2-joint chain: "root" at the origin, "child" offset (1,0,0) from root. Bind pose ==
// rest pose (identity rotations), so each joint's inverse_bind is the real inverse of its own
// bind-pose world matrix.
static void build_skel(GSkel *skel) {
    memset(skel, 0, sizeof(*skel));
    skel->version = 1;
    skel->joint_count = 2;

    strcpy(skel->joints[0].name, "root");
    skel->joints[0].parent_index = -1;
    skel->joints[0].rest_rotation[3] = 1.0f; // identity quat (0,0,0,1)
    // inverse_bind for an identity-transform joint is identity.
    skel->joints[0].inverse_bind[0] = skel->joints[0].inverse_bind[5] =
        skel->joints[0].inverse_bind[10] = skel->joints[0].inverse_bind[15] = 1.0f;

    strcpy(skel->joints[1].name, "child");
    skel->joints[1].parent_index = 0;
    skel->joints[1].rest_translation[0] = 1.0f;
    skel->joints[1].rest_rotation[3] = 1.0f;
    // Bind-pose world for joint 1 is translate(1,0,0); its inverse is translate(-1,0,0).
    skel->joints[1].inverse_bind[0] = skel->joints[1].inverse_bind[5] =
        skel->joints[1].inverse_bind[10] = skel->joints[1].inverse_bind[15] = 1.0f;
    skel->joints[1].inverse_bind[12] = -1.0f;
}

static void test_rest_pose_is_identity_skin(void) {
    GSkel skel;
    build_skel(&skel);

    float pose_rot[8] = {0, 0, 0, 1, 0, 0, 0, 1};
    float pose_trans[6] = {0, 0, 0, 1, 0, 0};
    float skin[GSKEL_MAX_JOINTS][16];
    gpose_compute_skin_matrices(&skel, pose_rot, pose_trans, skin);

    // Posing exactly at rest should give an identity skin matrix for every joint (world == bind).
    int ok = 1;
    for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 16; k++) {
            float expected = (k % 5 == 0) ? 1.0f : 0.0f; // identity diag at 0,5,10,15
            if (!near(skin[j][k], expected)) ok = 0;
        }
    }
    CHECK(ok, "rest pose produces identity skin matrices");
}

static void test_root_rotation_moves_child_vertex(void) {
    GSkel skel;
    build_skel(&skel);

    // Root rotated 90 degrees about Z (quaternion x,y,z,w = 0,0,sin45,cos45); child at rest.
    float half = 0.70710678f;
    float pose_rot[8] = {0, 0, half, half, 0, 0, 0, 1};
    float pose_trans[6] = {0, 0, 0, 1, 0, 0};
    float skin[GSKEL_MAX_JOINTS][16];
    gpose_compute_skin_matrices(&skel, pose_rot, pose_trans, skin);

    // A mesh with one vertex, in bind pose at the child joint's own bind position (1,0,0),
    // 100% weighted to the child joint (index 1).
    GMeshVertex verts[1];
    memset(verts, 0, sizeof(verts));
    verts[0].position[0] = 1.0f;
    verts[0].normal[1] = 1.0f; // arbitrary up-facing normal to exercise the vector transform too
    verts[0].bone_indices[0] = 1;
    verts[0].bone_weights[0] = 1.0f;

    uint32_t indices[3] = {0, 0, 0}; // a degenerate "triangle" reusing the one real vertex
    GMesh mesh;
    memset(&mesh, 0, sizeof(mesh));
    mesh.version = 1;
    mesh.vertex_count = 1;
    mesh.index_count = 3;
    mesh.vertices = verts;
    mesh.indices = indices;

    float out[3 * 6];
    uint32_t n = gpose_skin_mesh(&mesh, skin, out);
    CHECK(n == 3, "gpose_skin_mesh returns index_count vertices");

    // Expected: child's own world position under a 90-degree root rotation about Z is (0,1,0)
    // (world = R90z * T(1,0,0) * inverse_bind(child) applied to (1,0,0) -- see this file's own
    // header math walkthrough / GOLDENBAND source commit message for the derivation).
    CHECK(near(out[0], 0.0f) && near(out[1], 1.0f) && near(out[2], 0.0f),
          "root rotation carries a child-weighted vertex to the correct world position");
    // Same rotation should carry the normal (0,1,0) to (-1,0,0).
    CHECK(near(out[3], -1.0f) && near(out[4], 0.0f) && near(out[5], 0.0f),
          "root rotation carries the vertex normal correctly");
}

int main(void) {
    test_rest_pose_is_identity_skin();
    test_root_rotation_moves_child_vertex();
    printf("%s: %d failure(s)\n", failures == 0 ? "OK" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
