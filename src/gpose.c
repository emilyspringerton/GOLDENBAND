// gpose.c — see gpose.h. Self-contained column-major float[16] matrix math -- no external
// dependency beyond gskel.h/gmesh.h, matching this repo's own "no engine dependency" discipline.

#include "gpose.h"
#include <math.h>
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

void gpose_compute_joint_world(const GSkel *skel, const float *pose_rot, const float *pose_trans,
                                float out_joint_world[][16]) {
    for (uint32_t j = 0; j < skel->joint_count; j++) {
        float local[16];
        mat4_from_trs(&pose_trans[j * 3], &pose_rot[j * 4], local);
        int32_t parent = skel->joints[j].parent_index;
        if (parent < 0) {
            memcpy(out_joint_world[j], local, 16 * sizeof(float));
        } else {
            // Real, load-bearing invariant (GSKEL_FORMAT.md): parent_index < j always, so
            // out_joint_world[parent] is already computed by the time we reach j -- one forward
            // pass, no recursion or topological sort needed.
            mat4_mul(out_joint_world[parent], local, out_joint_world[j]);
        }
    }
}

void gpose_compute_skin_matrices(const GSkel *skel, const float *pose_rot, const float *pose_trans,
                                  float out_skin_matrices[][16]) {
    float joint_world[GSKEL_MAX_JOINTS][16];
    gpose_compute_joint_world(skel, pose_rot, pose_trans, joint_world);
    for (uint32_t j = 0; j < skel->joint_count; j++) {
        mat4_mul(joint_world[j], skel->joints[j].inverse_bind, out_skin_matrices[j]);
    }
}

static void quat_normalize(float q[4]) {
    float len = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (len < 1e-8f) {
        q[0] = q[1] = q[2] = 0.0f;
        q[3] = 1.0f;
        return;
    }
    q[0] /= len; q[1] /= len; q[2] /= len; q[3] /= len;
}

// Real, minimal shortest-arc "rotation from unit vector a to unit vector b" quaternion.
static void quat_from_to(const float a[3], const float b[3], float out[4]) {
    float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    if (dot > 0.999999f) {
        out[0] = out[1] = out[2] = 0.0f;
        out[3] = 1.0f;
        return;
    }
    if (dot < -0.999999f) {
        // 180-degree case: cross with any axis not parallel to a picks a real perpendicular
        // rotation axis.
        float axis[3] = {1.0f, 0.0f, 0.0f};
        if (fabsf(a[0]) > 0.9f) {
            axis[0] = 0.0f; axis[1] = 1.0f; axis[2] = 0.0f;
        }
        float cx = a[1] * axis[2] - a[2] * axis[1];
        float cy = a[2] * axis[0] - a[0] * axis[2];
        float cz = a[0] * axis[1] - a[1] * axis[0];
        float len = sqrtf(cx * cx + cy * cy + cz * cz);
        out[0] = cx / len; out[1] = cy / len; out[2] = cz / len; out[3] = 0.0f;
        return;
    }
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
    out[3] = 1.0f + dot;
    quat_normalize(out);
}

// Real angle (degrees) between two quaternions, shortest-path (sign-corrected dot).
static float quat_angle_between_deg(const float a[4], const float b[4]) {
    float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    if (dot < 0.0f) dot = -dot;
    if (dot > 1.0f) dot = 1.0f;
    return 2.0f * acosf(dot) * (180.0f / 3.14159265f);
}

// Real nlerp, same sign-correction + normalize shape as gband.c's own gb_blend (kept as a
// separate, self-contained copy here rather than a shared include -- gpose.c's own header
// comment commits to "no external dependency beyond gskel.h/gmesh.h").
static void quat_nlerp(const float a[4], const float b[4], float t, float out[4]) {
    float bb[4] = {b[0], b[1], b[2], b[3]};
    float dot = a[0] * bb[0] + a[1] * bb[1] + a[2] * bb[2] + a[3] * bb[3];
    if (dot < 0.0f) {
        bb[0] = -bb[0]; bb[1] = -bb[1]; bb[2] = -bb[2]; bb[3] = -bb[3];
    }
    out[0] = a[0] + (bb[0] - a[0]) * t;
    out[1] = a[1] + (bb[1] - a[1]) * t;
    out[2] = a[2] + (bb[2] - a[2]) * t;
    out[3] = a[3] + (bb[3] - a[3]) * t;
    quat_normalize(out);
}

void gpose_look_at(const GSkel *skel, float *pose_rot, const float *pose_trans, int joint_index,
                    const float local_axis[3], const float target_world[3],
                    const float parent_world[16], float max_angle_deg) {
    (void)skel;
    // target_world -> parent-local space: subtract the parent's own world translation, then
    // un-rotate by the parent's own rotation (transpose of its upper-left 3x3 -- correct because
    // parent_world is a real rigid transform, no scale, per gpose_compute_joint_world's own
    // contract).
    float rel[3] = {
        target_world[0] - parent_world[12],
        target_world[1] - parent_world[13],
        target_world[2] - parent_world[14],
    };
    float local_target[3] = {
        parent_world[0] * rel[0] + parent_world[1] * rel[1] + parent_world[2] * rel[2],
        parent_world[4] * rel[0] + parent_world[5] * rel[1] + parent_world[6] * rel[2],
        parent_world[8] * rel[0] + parent_world[9] * rel[1] + parent_world[10] * rel[2],
    };

    float dir[3] = {
        local_target[0] - pose_trans[joint_index * 3 + 0],
        local_target[1] - pose_trans[joint_index * 3 + 1],
        local_target[2] - pose_trans[joint_index * 3 + 2],
    };
    float len = sqrtf(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
    if (len < 1e-6f) return; // target coincides with the joint itself -- no real direction, leave pose_rot untouched
    dir[0] /= len; dir[1] /= len; dir[2] /= len;

    float desired[4];
    quat_from_to(local_axis, dir, desired);

    float *cur = &pose_rot[joint_index * 4];
    float current[4] = {cur[0], cur[1], cur[2], cur[3]};
    float angle = quat_angle_between_deg(current, desired);
    float t = (angle < 1e-4f || angle <= max_angle_deg) ? 1.0f : (max_angle_deg / angle);
    if (t > 1.0f) t = 1.0f;
    if (t < 0.0f) t = 0.0f;

    float result[4];
    quat_nlerp(current, desired, t, result);
    cur[0] = result[0]; cur[1] = result[1]; cur[2] = result[2]; cur[3] = result[3];
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
