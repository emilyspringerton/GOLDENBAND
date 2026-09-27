// grobot.c — see grobot.h and format/GROBOT_FORMAT.md.
#include "grobot.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define GROBOT_HEADER_SIZE 104
#define GROBOT_RECORD_SIZE 280

static uint32_t rd_u32(const unsigned char *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static double rd_f64(const unsigned char *p) {
    uint64_t v = 0;
    for (int i = 7; i >= 0; i--) v = (v << 8) | p[i];
    double d;
    memcpy(&d, &v, sizeof d);
    return d;
}

static void rd_f64s(const unsigned char *p, double *out, int n) {
    for (int i = 0; i < n; i++) out[i] = rd_f64(p + 8 * i);
}

int grobot_init(const char *path, GRobot *r) {
    memset(r, 0, sizeof *r);
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    unsigned char hdr[GROBOT_HEADER_SIZE];
    if (fread(hdr, 1, sizeof hdr, f) != sizeof hdr || memcmp(hdr, "GRBT", 4) != 0) { fclose(f); return 0; }
    r->version = rd_u32(hdr + 4);
    r->joint_count = rd_u32(hdr + 8);
    r->tcp_joint = (int32_t)rd_u32(hdr + 12);
    rd_f64s(hdr + 16, r->tcp_xyz, 3);
    memcpy(r->name, hdr + 40, GROBOT_NAME_LEN);
    r->name[GROBOT_NAME_LEN - 1] = 0;
    memcpy(r->spec_hash, hdr + 72, 32);
    if (r->version != 1 || r->joint_count == 0 || r->joint_count > GROBOT_MAX_JOINTS ||
        r->tcp_joint >= (int32_t)r->joint_count) {
        fclose(f);
        return 0;
    }
    for (uint32_t i = 0; i < r->joint_count; i++) {
        unsigned char rec[GROBOT_RECORD_SIZE];
        if (fread(rec, 1, sizeof rec, f) != sizeof rec) { fclose(f); return 0; }
        GRobotJoint *j = &r->joints[i];
        memcpy(j->name, rec, 32);
        j->name[31] = 0;
        memcpy(j->child, rec + 32, 32);
        j->child[31] = 0;
        j->parent = (int32_t)rd_u32(rec + 64);
        j->type = rd_u32(rec + 68);
        const unsigned char *p = rec + 72;
        rd_f64s(p, j->origin_xyz, 3); p += 24;
        rd_f64s(p, j->origin_quat, 4); p += 32;
        rd_f64s(p, j->axis, 3); p += 24;
        j->lower = rd_f64(p); j->upper = rd_f64(p + 8); j->velocity = rd_f64(p + 16);
        j->effort = rd_f64(p + 24); j->damping = rd_f64(p + 32); j->mass = rd_f64(p + 40); p += 48;
        rd_f64s(p, j->com, 3); p += 24;
        rd_f64s(p, j->principal_quat, 4); p += 32;
        rd_f64s(p, j->principal_moments, 3);
        // Parent-before-child ordering is required (single forward FK pass), and a moving link
        // must have real mass properties.
        if (j->parent >= (int32_t)i || j->parent < -1 || !(j->mass > 0.0) || (j->type != GROBOT_REVOLUTE && j->type != GROBOT_CONTINUOUS)) {
            fclose(f);
            return 0;
        }
    }
    int extra = fgetc(f);
    fclose(f);
    return extra == EOF;
}

// Rotation taking +Z onto unit axis a (grb hinges turn about their frame's +Z).
static void align_z_to(const double a[3], double q[4]) {
    double c = a[2];
    if (c > 1.0 - 1e-12) { q[0] = q[1] = q[2] = 0; q[3] = 1; return; }
    if (c < -1.0 + 1e-12) { q[0] = 1; q[1] = q[2] = q[3] = 0; return; }
    double axis[3] = {-a[1], a[0], 0.0}; // z x a
    grb_quat_from_axis_angle(axis, acos(c), q);
}

static void pose_mul(const double pa[3], const double qa[4], const double pb[3], const double qb[4],
                     double po[3], double qo[4]) {
    double r[3];
    grb_quat_rotate(qa, pb, r);
    double p[3] = {pa[0] + r[0], pa[1] + r[1], pa[2] + r[2]};
    double q[4];
    grb_quat_mul(qa, qb, q);
    memcpy(po, p, sizeof p);
    memcpy(qo, q, sizeof q);
}

// Pose of frame X (given in a link frame) expressed in that link's BODY (principal/COM) frame.
static void link_to_body(const GRobotJoint *j, const double xp[3], const double xq[4], double bp[3], double bq[4]) {
    double inv[4], d[3] = {xp[0] - j->com[0], xp[1] - j->com[1], xp[2] - j->com[2]};
    grb_quat_conj(j->principal_quat, inv);
    grb_quat_rotate(inv, d, bp);
    grb_quat_mul(inv, xq, bq);
}

void grobot_fk(const GRobot *r, const double bp[3], const double bq[4], const double *q,
               double lp[][3], double lq[][4]) {
    for (uint32_t i = 0; i < r->joint_count; i++) {
        const GRobotJoint *j = &r->joints[i];
        const double *pp = j->parent < 0 ? bp : lp[j->parent];
        const double *pq = j->parent < 0 ? bq : lq[j->parent];
        double fp[3], fq[4], rq[4];
        pose_mul(pp, pq, j->origin_xyz, j->origin_quat, fp, fq);
        grb_quat_from_axis_angle(j->axis, q ? q[i] : 0.0, rq);
        memcpy(lp[i], fp, sizeof fp);
        grb_quat_mul(fq, rq, lq[i]);
    }
}

static void place_bodies(GrbWorld *w, const GRobot *r, const GRobotInstance *inst, const double *q) {
    double lp[GROBOT_MAX_JOINTS][3], lq[GROBOT_MAX_JOINTS][4];
    grobot_fk(r, inst->base_pos, inst->base_rot, q, lp, lq);
    for (uint32_t i = 0; i < r->joint_count; i++) {
        GrbBody *b = &w->bodies[inst->body[i]];
        pose_mul(lp[i], lq[i], r->joints[i].com, r->joints[i].principal_quat, b->pos, b->rot);
    }
}

int grobot_spawn(GrbWorld *w, const GRobot *r, const double bp[3], const double bq[4], const double *q,
                 GRobotInstance *inst) {
    if (w->body_count + r->joint_count > GRB_MAX_BODIES || w->joint_count + r->joint_count > GRB_MAX_JOINTS) return 0;
    memcpy(inst->base_pos, bp, sizeof inst->base_pos);
    memcpy(inst->base_rot, bq, sizeof inst->base_rot);
    double lp[GROBOT_MAX_JOINTS][3], lq[GROBOT_MAX_JOINTS][4];
    grobot_fk(r, bp, bq, q, lp, lq);
    for (uint32_t i = 0; i < r->joint_count; i++) {
        const GRobotJoint *j = &r->joints[i];
        double pos[3], rot[4];
        pose_mul(lp[i], lq[i], j->com, j->principal_quat, pos, rot);
        inst->body[i] = grb_body_add(w, j->mass, j->principal_moments, pos, rot);
        w->bodies[inst->body[i]].friction = 0.0;
    }
    const double zero[3] = {0, 0, 0};
    for (uint32_t i = 0; i < r->joint_count; i++) {
        const GRobotJoint *j = &r->joints[i];
        double align[4], fp[3], fq[4], ap[3], aq[4], cp[3], cq[4];
        align_z_to(j->axis, align);
        // Parent side: origin * align, in the parent link frame (or world, for the base).
        double oq[4];
        grb_quat_mul(j->origin_quat, align, oq);
        int body_a = -1;
        if (j->parent < 0) {
            pose_mul(bp, bq, j->origin_xyz, oq, ap, aq);
        } else {
            body_a = inst->body[j->parent];
            link_to_body(&r->joints[j->parent], j->origin_xyz, oq, ap, aq);
        }
        // Child side: the joint frame sits at the child link origin, rotated by align.
        link_to_body(j, zero, align, cp, cq);
        (void)fp; (void)fq;
        int ji = grb_joint_add_local(w, GRB_JOINT_HINGE, body_a, inst->body[i], ap, aq, cp, cq);
        GrbJoint *gj = &w->joints[ji];
        inst->joint[i] = ji;
        gj->has_limits = j->type == GROBOT_REVOLUTE;
        gj->lower = j->lower;
        gj->upper = j->upper;
        gj->effort_limit = j->effort;
        gj->velocity_limit = j->velocity;
        gj->damping = j->damping;
        grb_joint_sync_angle(w, ji, q ? q[i] : 0.0);
    }
    return 1;
}

void grobot_set_state(GrbWorld *w, const GRobot *r, const GRobotInstance *inst, const double *q, const double *qd) {
    place_bodies(w, r, inst, q);
    // Rigid-body velocities from joint rates, propagated down the tree (world frame).
    double wl[GROBOT_MAX_JOINTS][3], vl[GROBOT_MAX_JOINTS][3], lp[GROBOT_MAX_JOINTS][3], lq[GROBOT_MAX_JOINTS][4];
    grobot_fk(r, inst->base_pos, inst->base_rot, q, lp, lq);
    for (uint32_t i = 0; i < r->joint_count; i++) {
        const GRobotJoint *j = &r->joints[i];
        double pw[3] = {0, 0, 0}, pv[3] = {0, 0, 0}, pp[3];
        memcpy(pp, inst->base_pos, sizeof pp);
        if (j->parent >= 0) {
            memcpy(pw, wl[j->parent], sizeof pw);
            memcpy(pv, vl[j->parent], sizeof pv);
            memcpy(pp, lp[j->parent], sizeof pp);
        }
        double z[3];
        grb_quat_rotate(lq[i], j->axis, z); // the axis is invariant under its own rotation
        double rate = qd ? qd[i] : 0.0;
        double d[3] = {lp[i][0] - pp[0], lp[i][1] - pp[1], lp[i][2] - pp[2]};
        // v_joint = v_parent + w_parent x d; w = w_parent + z*qd
        vl[i][0] = pv[0] + pw[1] * d[2] - pw[2] * d[1];
        vl[i][1] = pv[1] + pw[2] * d[0] - pw[0] * d[2];
        vl[i][2] = pv[2] + pw[0] * d[1] - pw[1] * d[0];
        for (int k = 0; k < 3; k++) wl[i][k] = pw[k] + z[k] * rate;
        GrbBody *b = &w->bodies[inst->body[i]];
        double rc[3] = {b->pos[0] - lp[i][0], b->pos[1] - lp[i][1], b->pos[2] - lp[i][2]};
        b->vel[0] = vl[i][0] + wl[i][1] * rc[2] - wl[i][2] * rc[1];
        b->vel[1] = vl[i][1] + wl[i][2] * rc[0] - wl[i][0] * rc[2];
        b->vel[2] = vl[i][2] + wl[i][0] * rc[1] - wl[i][1] * rc[0];
        memcpy(b->omega, wl[i], sizeof b->omega);
    }
    for (uint32_t i = 0; i < r->joint_count; i++) grb_joint_sync_angle(w, inst->joint[i], q[i]);
}

void grobot_tcp(const GrbWorld *w, const GRobot *r, const GRobotInstance *inst, double out[3]) {
    if (r->tcp_joint < 0) { out[0] = out[1] = out[2] = 0; return; }
    const GRobotJoint *j = &r->joints[r->tcp_joint];
    double bp[3], bq[4], iq[4] = {0, 0, 0, 1};
    link_to_body(j, r->tcp_xyz, iq, bp, bq);
    grb_body_world_point(&w->bodies[inst->body[r->tcp_joint]], bp, out);
}

double grobot_axis_inertia(const GrbWorld *w, const GRobot *r, const GRobotInstance *inst, int ji) {
    const GrbJoint *gj = &w->joints[inst->joint[ji]];
    const GrbBody *cb = &w->bodies[inst->body[ji]];
    double p[3], z[3], zl[3] = {0, 0, 1}, q[4];
    grb_quat_mul(cb->rot, gj->local_rot_b, q);
    grb_quat_rotate(q, zl, z);
    grb_body_world_point(cb, gj->local_pos_b, p);
    double total = 0.0;
    for (uint32_t i = 0; i < r->joint_count; i++) {
        // i is in ji's subtree if walking its parents reaches ji.
        int k = (int)i;
        while (k >= 0 && k != ji) k = r->joints[k].parent;
        if (k != ji) continue;
        const GrbBody *b = &w->bodies[inst->body[i]];
        double zl2[3];
        double c[4];
        grb_quat_conj(b->rot, c);
        grb_quat_rotate(c, z, zl2);
        const double *I = r->joints[i].principal_moments;
        double iz = I[0] * zl2[0] * zl2[0] + I[1] * zl2[1] * zl2[1] + I[2] * zl2[2] * zl2[2];
        double d[3] = {b->pos[0] - p[0], b->pos[1] - p[1], b->pos[2] - p[2]};
        double along = d[0] * z[0] + d[1] * z[1] + d[2] * z[2];
        double perp2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2] - along * along;
        total += iz + r->joints[i].mass * perp2;
    }
    return total;
}
