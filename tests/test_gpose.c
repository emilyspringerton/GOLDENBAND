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

// Real, minimal quaternion-rotate-vector helper (standard v' = v + 2w(q_xyz x v) + 2(q_xyz x
// (q_xyz x v)) form), local to this test file -- matches the same "no cross-file coupling to
// gpose.c's own static helpers" discipline the rest of this file already holds itself to (near()
// is its own local copy too, not imported).
static void rotate_vec(const float q[4], const float v[3], float out[3]) {
    float qx = q[0], qy = q[1], qz = q[2], qw = q[3];
    float tx = 2.0f * (qy * v[2] - qz * v[1]);
    float ty = 2.0f * (qz * v[0] - qx * v[2]);
    float tz = 2.0f * (qx * v[1] - qy * v[0]);
    out[0] = v[0] + qw * tx + (qy * tz - qz * ty);
    out[1] = v[1] + qw * ty + (qz * tx - qx * tz);
    out[2] = v[2] + qw * tz + (qx * ty - qy * tx);
}

static const float IDENTITY16[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

static void test_look_at_full_rotation_faces_target(void) {
    // A single root joint at the origin, "forward" = +Z, unclamped (max_angle_deg large).
    float pose_rot[4] = {0, 0, 0, 1};
    float pose_trans[3] = {0, 0, 0};
    float local_axis[3] = {0, 0, 1};
    float target[3] = {1, 0, 0}; // directly along +X -- a real 90-degree turn from +Z

    gpose_look_at(NULL, pose_rot, pose_trans, 0, local_axis, target, IDENTITY16, 180.0f);

    float rotated[3];
    rotate_vec(pose_rot, local_axis, rotated);
    CHECK(near(rotated[0], 1.0f) && near(rotated[1], 0.0f) && near(rotated[2], 0.0f),
          "unclamped look_at rotates local_axis to face the target exactly");
}

static void test_look_at_respects_max_angle_clamp(void) {
    // Same 90-degree-away target, but clamped to a small real angle (the founder's own "up to 30
    // degrees" example) -- the joint should turn TOWARD the target but not reach it.
    float pose_rot[4] = {0, 0, 0, 1};
    float pose_trans[3] = {0, 0, 0};
    float local_axis[3] = {0, 0, 1};
    float target[3] = {1, 0, 0};

    gpose_look_at(NULL, pose_rot, pose_trans, 0, local_axis, target, IDENTITY16, 30.0f);

    float identity_q[4] = {0, 0, 0, 1};
    float turned_deg = 2.0f * acosf(fabsf(pose_rot[3])) * (180.0f / 3.14159265f);
    CHECK(turned_deg <= 30.5f, "clamped look_at does not exceed max_angle_deg (real, small nlerp/slerp approximation error allowed)");
    CHECK(turned_deg > 1.0f, "clamped look_at still turns a real, non-trivial amount toward the target");

    float rotated[3];
    rotate_vec(pose_rot, local_axis, rotated);
    CHECK(rotated[0] > 0.0f, "clamped look_at turns TOWARD +X, just not all the way");
    (void)identity_q;
}

static void test_look_at_leaves_pose_untouched_when_target_is_on_the_joint(void) {
    // A degenerate case: target == the joint's own world position (dir has zero length). Must
    // leave pose_rot exactly as it was, not divide by zero or produce garbage.
    float pose_rot[4] = {0.1f, 0.2f, 0.3f, 0.9273618f}; // an arbitrary, already-normalized quat
    float pose_trans[3] = {5.0f, 6.0f, 7.0f};
    float local_axis[3] = {0, 0, 1};
    float target[3] = {5.0f, 6.0f, 7.0f}; // exactly at the joint's own local position

    float before[4] = {pose_rot[0], pose_rot[1], pose_rot[2], pose_rot[3]};
    gpose_look_at(NULL, pose_rot, pose_trans, 0, local_axis, target, IDENTITY16, 30.0f);
    CHECK(near(pose_rot[0], before[0]) && near(pose_rot[1], before[1]) &&
          near(pose_rot[2], before[2]) && near(pose_rot[3], before[3]),
          "look_at leaves pose_rot untouched when the target coincides with the joint itself");
}

int main(void) {
    test_rest_pose_is_identity_skin();
    test_root_rotation_moves_child_vertex();
    test_look_at_full_rotation_faces_target();
    test_look_at_respects_max_angle_clamp();
    test_look_at_leaves_pose_untouched_when_target_is_on_the_joint();
    printf("%s: %d failure(s)\n", failures == 0 ? "OK" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
