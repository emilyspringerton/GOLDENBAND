// gpose.c — see gpose.h. Self-contained column-major float[16] matrix math -- no external
// dependency beyond gskel.h/gmesh.h, matching this repo's own "no engine dependency" discipline.

#include "gpose.h"
#include <string.h>

static void mat4_identity(float out[16]) {
    memset(out, 0, 16 * sizeof(float));
    out[0] = out[5] = out[10] = out[15] = 1.0f;
}

// Column-major multiply: out = a * b (applying b first, then a -- same convention as
// SHANKPIT's packages/common/mat4.h's own mat4_multiply, so a vendored copy of this file drops
// straight into that convention with no sign/order surprises).
static void mat4_mul(const float a[16], const float b[16], float out[16]) {
    float r[16];
    for (int col = 0; col < 4; col++) {
        for (int row = 0; row < 4; row++) {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++) {
                sum += a[k * 4 + row] * b[col * 4 + k];
            }
            r[col * 4 + row] = sum;
        }
    }
    memcpy(out, r, 16 * sizeof(float));
}

// Real quaternion (x,y,z,w) -> column-major rotation matrix.
static void mat4_from_quat(const float q[4], float out[16]) {
    float x = q[0], y = q[1], z = q[2], w = q[3];
    float xx = x * x, yy = y * y, zz = z * z;
    float xy = x * y, xz = x * z, yz = y * z;
    float wx = w * x, wy = w * y, wz = w * z;
    mat4_identity(out);
    out[0] = 1.0f - 2.0f * (yy + zz);
    out[1] = 2.0f * (xy + wz);
    out[2] = 2.0f * (xz - wy);
    out[4] = 2.0f * (xy - wz);
    out[5] = 1.0f - 2.0f * (xx + zz);
    out[6] = 2.0f * (yz + wx);
    out[8] = 2.0f * (xz + wy);
    out[9] = 2.0f * (yz - wx);
    out[10] = 1.0f - 2.0f * (xx + yy);
}

// Real translation*rotation local transform: T(trans) * R(rot).
static void mat4_from_trs(const float trans[3], const float rot[4], float out[16]) {
    float r[16];
    mat4_from_quat(rot, r);
    r[12] = trans[0];
    r[13] = trans[1];
    r[14] = trans[2];
    memcpy(out, r, 16 * sizeof(float));
}

static void mat4_transform_point(const float m[16], const float v[3], float out[3]) {
    out[0] = m[0] * v[0] + m[4] * v[1] + m[8] * v[2] + m[12];
    out[1] = m[1] * v[0] + m[5] * v[1] + m[9] * v[2] + m[13];
    out[2] = m[2] * v[0] + m[6] * v[1] + m[10] * v[2] + m[14];
}

// w=0 transform -- upper 3x3 only, no translation. A real approximation for normals (correct
// for pure rotation+uniform-scale skin matrices, which is what a real rigid-bone rig produces;
// a full inverse-transpose pass is real future work for non-uniform bone scale, not needed by
// any real asset imported so far).
static void mat4_transform_vector(const float m[16], const float v[3], float out[3]) {
    out[0] = m[0] * v[0] + m[4] * v[1] + m[8] * v[2];
    out[1] = m[1] * v[0] + m[5] * v[1] + m[9] * v[2];
    out[2] = m[2] * v[0] + m[6] * v[1] + m[10] * v[2];
}

void gpose_compute_skin_matrices(const GSkel *skel, const float *pose_rot, const float *pose_trans,
                                  float out_skin_matrices[][16]) {
    float joint_world[GSKEL_MAX_JOINTS][16];
    for (uint32_t j = 0; j < skel->joint_count; j++) {
        float local[16];
        mat4_from_trs(&pose_trans[j * 3], &pose_rot[j * 4], local);
        int32_t parent = skel->joints[j].parent_index;
        if (parent < 0) {
            memcpy(joint_world[j], local, 16 * sizeof(float));
        } else {
            // Real, load-bearing invariant (GSKEL_FORMAT.md): parent_index < j always, so
            // joint_world[parent] is already computed by the time we reach j -- one forward
            // pass, no recursion or topological sort needed.
            mat4_mul(joint_world[parent], local, joint_world[j]);
        }
        mat4_mul(joint_world[j], skel->joints[j].inverse_bind, out_skin_matrices[j]);
    }
}

uint32_t gpose_skin_mesh(const GMesh *mesh, const float skin_matrices[][16], float *out_verts6) {
    for (uint32_t i = 0; i < mesh->index_count; i++) {
        uint32_t vi = mesh->indices[i];
        const GMeshVertex *v = &mesh->vertices[vi];
        float pos[3] = {0, 0, 0};
        float nrm[3] = {0, 0, 0};
        for (int b = 0; b < 4; b++) {
            float w = v->bone_weights[b];
            if (w == 0.0f) continue;
            uint8_t ji = v->bone_indices[b];
            float p[3], n[3];
            mat4_transform_point(skin_matrices[ji], v->position, p);
            mat4_transform_vector(skin_matrices[ji], v->normal, n);
            pos[0] += w * p[0]; pos[1] += w * p[1]; pos[2] += w * p[2];
            nrm[0] += w * n[0]; nrm[1] += w * n[1]; nrm[2] += w * n[2];
        }
        float *out = &out_verts6[i * 6];
        out[0] = pos[0]; out[1] = pos[1]; out[2] = pos[2];
        out[3] = nrm[0]; out[4] = nrm[1]; out[5] = nrm[2];
    }
    return mesh->index_count;
}
