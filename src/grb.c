// grb.c — GOLDEN BAND rigid body physics. See grb.h for the model, conventions and named limits.
//
// Solver structure, per grb_world_step(dt), per substep h = dt / substeps (XPBD, Müller et al.
// 2020, Algorithm 2 -- one position iteration per substep):
//
//   1. motors:     compute every hinge motor's PD/command torque at substep rate, clamp to the
//                  datasheet effort limit, apply as an equal-and-opposite torque pair.
//   2. integrate:  v += h*(g + F/m); x += h*v;  w += h*I^-1*(tau - w x Iw); q += h/2*[w,0]*q.
//   3. positions:  joints (attachment, alignment, limits), then ground contacts (normal +
//                  static friction), each a zero-compliance XPBD projection.
//   4. velocities: v = (x - x_prev)/h; w = 2*vec(q*q_prev^-1)/h.
//   5. vel. solve: contact dynamic friction + restitution.
#include "grb.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------------------------------------------------------- vector / quaternion helpers

static void v3_set(double o[3], double x, double y, double z) { o[0] = x; o[1] = y; o[2] = z; }
static void v3_copy(double o[3], const double a[3]) { o[0] = a[0]; o[1] = a[1]; o[2] = a[2]; }
static void v3_add(double o[3], const double a[3], const double b[3]) {
    o[0] = a[0] + b[0]; o[1] = a[1] + b[1]; o[2] = a[2] + b[2];
}
static void v3_sub(double o[3], const double a[3], const double b[3]) {
    o[0] = a[0] - b[0]; o[1] = a[1] - b[1]; o[2] = a[2] - b[2];
}
static void v3_scale(double o[3], const double a[3], double s) { o[0] = a[0] * s; o[1] = a[1] * s; o[2] = a[2] * s; }
static void v3_madd(double o[3], const double a[3], double s) { o[0] += a[0] * s; o[1] += a[1] * s; o[2] += a[2] * s; }
static double v3_dot(const double a[3], const double b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
static void v3_cross(double o[3], const double a[3], const double b[3]) {
    double t[3] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    v3_copy(o, t);
}
static double v3_len(const double a[3]) { return sqrt(v3_dot(a, a)); }

void grb_quat_mul(const double a[4], const double b[4], double o[4]) {
    double t[4] = {
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    };
    memcpy(o, t, sizeof t);
}

void grb_quat_conj(const double q[4], double o[4]) { o[0] = -q[0]; o[1] = -q[1]; o[2] = -q[2]; o[3] = q[3]; }

void grb_quat_rotate(const double q[4], const double v[3], double o[3]) {
    // v' = v + 2*w*(u x v) + 2*(u x (u x v)), u = q.xyz
    double u[3] = {q[0], q[1], q[2]}, t[3], t2[3];
    v3_cross(t, u, v);
    v3_scale(t, t, 2.0);
    v3_cross(t2, u, t);
    double r[3] = {v[0] + q[3] * t[0] + t2[0], v[1] + q[3] * t[1] + t2[1], v[2] + q[3] * t[2] + t2[2]};
    v3_copy(o, r);
}

static void quat_rotate_inv(const double q[4], const double v[3], double o[3]) {
    double c[4];
    grb_quat_conj(q, c);
    grb_quat_rotate(c, v, o);
}

void grb_quat_normalize(double q[4]) {
    double n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n < 1e-300) { q[0] = q[1] = q[2] = 0; q[3] = 1; return; }
    q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
}

void grb_quat_from_axis_angle(const double axis[3], double angle, double o[4]) {
    double n = v3_len(axis);
    if (n < 1e-300) { o[0] = o[1] = o[2] = 0; o[3] = 1; return; }
    double s = sin(angle * 0.5) / n;
    o[0] = axis[0] * s; o[1] = axis[1] * s; o[2] = axis[2] * s; o[3] = cos(angle * 0.5);
}

void grb_quat_from_rpy(double roll, double pitch, double yaw, double o[4]) {
    const double X[3] = {1, 0, 0}, Y[3] = {0, 1, 0}, Z[3] = {0, 0, 1};
    double qx[4], qy[4], qz[4], t[4];
    grb_quat_from_axis_angle(X, roll, qx);
    grb_quat_from_axis_angle(Y, pitch, qy);
    grb_quat_from_axis_angle(Z, yaw, qz);
    grb_quat_mul(qz, qy, t);
    grb_quat_mul(t, qx, o);
}

// Rotation vector -> apply as a first-order quaternion increment: q += 0.5*[dtheta,0]*q.
static void quat_add_rotvec(double q[4], const double d[3]) {
    double dq[4] = {d[0], d[1], d[2], 0.0}, t[4];
    grb_quat_mul(dq, q, t);
    q[0] += 0.5 * t[0]; q[1] += 0.5 * t[1]; q[2] += 0.5 * t[2]; q[3] += 0.5 * t[3];
    grb_quat_normalize(q);
}

static double wrap_pi(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

// ---------------------------------------------------------------- body helpers

static int body_dynamic(const GrbBody *b) {
    return b->inv_mass > 0.0 || b->inv_inertia[0] > 0.0 || b->inv_inertia[1] > 0.0 || b->inv_inertia[2] > 0.0;
}

// World-space I^-1 * v.
static void inv_inertia_apply(const GrbBody *b, const double v[3], double o[3]) {
    double l[3];
    quat_rotate_inv(b->rot, v, l);
    l[0] *= b->inv_inertia[0]; l[1] *= b->inv_inertia[1]; l[2] *= b->inv_inertia[2];
    grb_quat_rotate(b->rot, l, o);
}

// Generalized inverse mass of a positional correction along unit n applied at world offset r.
static double gen_inv_mass_lin(const GrbBody *b, const double r[3], const double n[3]) {
    if (!b) return 0.0;
    double rn[3], l[3];
    v3_cross(rn, r, n);
    quat_rotate_inv(b->rot, rn, l);
    return b->inv_mass + l[0] * l[0] * b->inv_inertia[0] + l[1] * l[1] * b->inv_inertia[1] +
           l[2] * l[2] * b->inv_inertia[2];
}

// Generalized inverse mass of a rotational correction about unit axis n.
static double gen_inv_mass_ang(const GrbBody *b, const double n[3]) {
    if (!b) return 0.0;
    double l[3];
    quat_rotate_inv(b->rot, n, l);
    return l[0] * l[0] * b->inv_inertia[0] + l[1] * l[1] * b->inv_inertia[1] + l[2] * l[2] * b->inv_inertia[2];
}

static void apply_lin_impulse_pos(GrbBody *b, const double r[3], const double p[3], double sign) {
    if (!b) return;
    v3_madd(b->pos, p, sign * b->inv_mass);
    double rp[3], d[3];
    v3_cross(rp, r, p);
    inv_inertia_apply(b, rp, d);
    v3_scale(d, d, sign);
    quat_add_rotvec(b->rot, d);
}

static void apply_ang_impulse_pos(GrbBody *b, const double p[3], double sign) {
    if (!b) return;
    double d[3];
    inv_inertia_apply(b, p, d);
    v3_scale(d, d, sign);
    quat_add_rotvec(b->rot, d);
}

static GrbBody *body_ptr(GrbWorld *w, int i) { return i < 0 ? NULL : &w->bodies[i]; }

// XPBD positional projection: move the attachment on A by +corr (and B by -corr), sharing the
// correction by generalized inverse mass. Returns the Lagrange multiplier (impulse * h).
static double solve_linear(GrbBody *a, GrbBody *b, const double ra[3], const double rb[3],
                           const double corr[3], double alpha_tilde) {
    double c = v3_len(corr);
    if (c < 1e-15) return 0.0;
    double n[3];
    v3_scale(n, corr, 1.0 / c);
    double wsum = gen_inv_mass_lin(a, ra, n) + gen_inv_mass_lin(b, rb, n) + alpha_tilde;
    if (wsum <= 0.0) return 0.0;
    double lambda = c / wsum, p[3];
    v3_scale(p, n, lambda);
    apply_lin_impulse_pos(a, ra, p, +1.0);
    apply_lin_impulse_pos(b, rb, p, -1.0);
    return lambda;
}

// XPBD rotational projection: rotate A by +corr (rotation vector) relative to B.
static double solve_angular(GrbBody *a, GrbBody *b, const double corr[3], double alpha_tilde) {
    double theta = v3_len(corr);
    if (theta < 1e-15) return 0.0;
    double n[3];
    v3_scale(n, corr, 1.0 / theta);
    double wsum = gen_inv_mass_ang(a, n) + gen_inv_mass_ang(b, n) + alpha_tilde;
    if (wsum <= 0.0) return 0.0;
    double lambda = theta / wsum, p[3];
    v3_scale(p, n, lambda);
    apply_ang_impulse_pos(a, p, +1.0);
    apply_ang_impulse_pos(b, p, -1.0);
    return lambda;
}

// ---------------------------------------------------------------- public: world/bodies

void grb_world_init(GrbWorld *w, double gx, double gy, double gz) {
    memset(w, 0, sizeof *w);
    v3_set(w->gravity, gx, gy, gz);
    w->substeps = 16;
    w->contact_iterations = 4;
}

int grb_world_add_plane(GrbWorld *w, double nx, double ny, double nz, double offset) {
    if (w->plane_count >= GRB_MAX_PLANES) return -1;
    GrbPlane *p = &w->planes[w->plane_count];
    double n[3] = {nx, ny, nz}, l = v3_len(n);
    if (l < 1e-12) return -1;
    v3_scale(p->normal, n, 1.0 / l);
    p->offset = offset;
    return (int)w->plane_count++;
}

int grb_body_add(GrbWorld *w, double mass, const double I[3], const double pos[3], const double rot[4]) {
    if (w->body_count >= GRB_MAX_BODIES) return -1;
    GrbBody *b = &w->bodies[w->body_count];
    memset(b, 0, sizeof *b);
    v3_copy(b->pos, pos);
    memcpy(b->rot, rot, sizeof b->rot);
    grb_quat_normalize(b->rot);
    if (mass > 0.0) {
        b->inv_mass = 1.0 / mass;
        for (int i = 0; i < 3; i++) b->inv_inertia[i] = (I && I[i] > 0.0) ? 1.0 / I[i] : 0.0;
    }
    b->friction = 0.6;
    b->restitution = 0.0;
    return (int)w->body_count++;
}

void grb_body_set_sphere(GrbBody *b, double r) { b->shape = GRB_SHAPE_SPHERE; b->radius = r; }
void grb_body_set_capsule(GrbBody *b, double r, double hh) {
    b->shape = GRB_SHAPE_CAPSULE; b->radius = r; b->half_height = hh;
}
void grb_body_set_box(GrbBody *b, double hx, double hy, double hz) {
    b->shape = GRB_SHAPE_BOX; v3_set(b->half_extents, hx, hy, hz);
}

void grb_inertia_box(double m, double hx, double hy, double hz, double o[3]) {
    o[0] = m / 3.0 * (hy * hy + hz * hz);
    o[1] = m / 3.0 * (hx * hx + hz * hz);
    o[2] = m / 3.0 * (hx * hx + hy * hy);
}

void grb_inertia_sphere(double m, double r, double o[3]) { o[0] = o[1] = o[2] = 0.4 * m * r * r; }

void grb_inertia_capsule(double m, double r, double hh, double o[3]) {
    // Exact capsule inertia (cylinder of length 2*hh + two hemispheres), mass split by volume.
    double h = 2.0 * hh;
    double vc = M_PI * r * r * h, vs = 4.0 / 3.0 * M_PI * r * r * r;
    double mc = m * vc / (vc + vs), ms = m * vs / (vc + vs); // ms = both hemispheres together
    double iy = mc * r * r * 0.5 + ms * 0.4 * r * r;
    double ix = mc * (h * h / 12.0 + r * r / 4.0) +
                ms * (0.4 * r * r + hh * hh + 3.0 / 8.0 * h * r);
    o[0] = ix; o[1] = iy; o[2] = ix;
}

void grb_body_world_point(const GrbBody *b, const double l[3], double o[3]) {
    double r[3];
    grb_quat_rotate(b->rot, l, r);
    v3_add(o, b->pos, r);
}

void grb_body_add_force_at(GrbBody *b, const double f[3], const double p[3]) {
    v3_add(b->force, b->force, f);
    double r[3], t[3];
    v3_sub(r, p, b->pos);
    v3_cross(t, r, f);
    v3_add(b->torque, b->torque, t);
}

// ---------------------------------------------------------------- public: joints

int grb_joint_add_local(GrbWorld *w, GrbJointType type, int a, int b, const double lpa[3],
                        const double lra[4], const double lpb[3], const double lrb[4]) {
    if (w->joint_count >= GRB_MAX_JOINTS || b < 0 || b >= (int)w->body_count || a >= (int)w->body_count)
        return -1;
    GrbJoint *j = &w->joints[w->joint_count];
    memset(j, 0, sizeof *j);
    j->type = type;
    j->body_a = a;
    j->body_b = b;
    v3_copy(j->local_pos_a, lpa);
    memcpy(j->local_rot_a, lra, sizeof j->local_rot_a);
    v3_copy(j->local_pos_b, lpb);
    memcpy(j->local_rot_b, lrb, sizeof j->local_rot_b);
    grb_quat_normalize(j->local_rot_a);
    grb_quat_normalize(j->local_rot_b);
    j->swing_max = M_PI;
    j->twist_lower = -M_PI;
    j->twist_upper = M_PI;
    return (int)w->joint_count++;
}

int grb_joint_add(GrbWorld *w, GrbJointType type, int a, int b, const double wp[3], const double wr[4]) {
    double lpa[3], lra[4], lpb[3], lrb[4], d[3], c[4];
    if (a < 0) {
        v3_copy(lpa, wp);
        memcpy(lra, wr, sizeof lra);
    } else {
        GrbBody *A = &w->bodies[a];
        v3_sub(d, wp, A->pos);
        quat_rotate_inv(A->rot, d, lpa);
        grb_quat_conj(A->rot, c);
        grb_quat_mul(c, wr, lra);
    }
    if (b < 0 || b >= (int)w->body_count) return -1;
    GrbBody *B = &w->bodies[b];
    v3_sub(d, wp, B->pos);
    quat_rotate_inv(B->rot, d, lpb);
    grb_quat_conj(B->rot, c);
    grb_quat_mul(c, wr, lrb);
    return grb_joint_add_local(w, type, a, b, lpa, lra, lpb, lrb);
}

// World pose of a joint frame on one side. body == NULL means the local frame IS the world frame.
static void joint_frame_world(const GrbBody *body, const double lp[3], const double lr[4], double p[3],
                              double q[4], double r[3]) {
    if (!body) {
        v3_copy(p, lp);
        memcpy(q, lr, 4 * sizeof(double));
        v3_set(r, 0, 0, 0);
        return;
    }
    grb_quat_rotate(body->rot, lp, r);
    v3_add(p, body->pos, r);
    grb_quat_mul(body->rot, lr, q);
}

// Signed hinge angle of frame B relative to frame A about their shared +Z, in (-pi, pi].
static double hinge_raw_angle(const double qa[4], const double qb[4], double axis_out[3]) {
    const double X[3] = {1, 0, 0}, Z[3] = {0, 0, 1};
    double xa[3], xb[3], za[3], zb[3], n[3], c[3];
    grb_quat_rotate(qa, X, xa);
    grb_quat_rotate(qb, X, xb);
    grb_quat_rotate(qa, Z, za);
    grb_quat_rotate(qb, Z, zb);
    v3_add(n, za, zb);
    double l = v3_len(n);
    if (l < 1e-12) v3_copy(n, za); else v3_scale(n, n, 1.0 / l);
    if (axis_out) v3_copy(axis_out, n);
    v3_cross(c, xa, xb);
    return atan2(v3_dot(c, n), v3_dot(xa, xb));
}

static void joint_measure(GrbWorld *w, GrbJoint *j, double *raw_out, double axis[3]) {
    double pa[3], qa[4], ra[3], pb[3], qb[4], rb[3];
    joint_frame_world(body_ptr(w, j->body_a), j->local_pos_a, j->local_rot_a, pa, qa, ra);
    joint_frame_world(body_ptr(w, j->body_b), j->local_pos_b, j->local_rot_b, pb, qb, rb);
    *raw_out = hinge_raw_angle(qa, qb, axis);
}

// Relative angular velocity of B w.r.t. A about the hinge axis.
static double joint_rel_omega(GrbWorld *w, const GrbJoint *j, const double n[3]) {
    double wa[3] = {0, 0, 0}, wb[3], d[3];
    if (j->body_a >= 0) v3_copy(wa, w->bodies[j->body_a].omega);
    v3_copy(wb, w->bodies[j->body_b].omega);
    v3_sub(d, wb, wa);
    return v3_dot(d, n);
}

static double effort_room(const GrbJoint *j, double used);

static void solve_joint(GrbWorld *w, GrbJoint *j) {
    GrbBody *A = body_ptr(w, j->body_a), *B = body_ptr(w, j->body_b);
    double pa[3], qa[4], ra[3], pb[3], qb[4], rb[3];
    const double X[3] = {1, 0, 0}, Z[3] = {0, 0, 1};

    // 1. Rotational part first (alignment + limits), then attachment -- the paper's order.
    joint_frame_world(A, j->local_pos_a, j->local_rot_a, pa, qa, ra);
    joint_frame_world(B, j->local_pos_b, j->local_rot_b, pb, qb, rb);

    if (j->type == GRB_JOINT_FIXED) {
        double c[4], dq[4];
        grb_quat_conj(qa, c);
        grb_quat_mul(qb, c, dq);
        if (dq[3] < 0) { dq[0] = -dq[0]; dq[1] = -dq[1]; dq[2] = -dq[2]; dq[3] = -dq[3]; }
        double corr[3] = {2.0 * dq[0], 2.0 * dq[1], 2.0 * dq[2]};
        solve_angular(A, B, corr, 0.0);
    } else if (j->type == GRB_JOINT_HINGE) {
        double za[3], zb[3], corr[3];
        grb_quat_rotate(qa, Z, za);
        grb_quat_rotate(qb, Z, zb);
        v3_cross(corr, za, zb);
        solve_angular(A, B, corr, 0.0);
        if (j->has_limits) {
            joint_frame_world(A, j->local_pos_a, j->local_rot_a, pa, qa, ra);
            joint_frame_world(B, j->local_pos_b, j->local_rot_b, pb, qb, rb);
            double n[3], raw = hinge_raw_angle(qa, qb, n);
            double phi = j->angle_initialized ? j->angle + wrap_pi(raw - j->raw_angle_prev) : raw;
            double target = phi < j->lower ? j->lower : (phi > j->upper ? j->upper : phi);
            if (target != phi) {
                v3_scale(corr, n, phi - target);
                solve_angular(A, B, corr, 0.0);
            }
        }
        // Servo spring as an XPBD angular drive: compliance 1/kp, multiplier clamped so the
        // implied torque (lambda / h^2) never exceeds the effort budget left after feed-forward.
        if (j->motor == GRB_MOTOR_POSITION && j->kp > 0.0) {
            joint_frame_world(A, j->local_pos_a, j->local_rot_a, pa, qa, ra);
            joint_frame_world(B, j->local_pos_b, j->local_rot_b, pb, qb, rb);
            double n[3], raw = hinge_raw_angle(qa, qb, n);
            double phi = j->angle_initialized ? j->angle + wrap_pi(raw - j->raw_angle_prev) : raw;
            double err = phi - j->target_angle;
            double wsum = gen_inv_mass_ang(A, n) + gen_inv_mass_ang(B, n);
            double h2 = w->substep_h * w->substep_h;
            if (fabs(err) > 0.0 && wsum > 0.0 && h2 > 0.0) {
                double lambda = fabs(err) / (wsum + 1.0 / (j->kp * h2));
                double max_l = effort_room(j, j->sub_torque) * h2;
                if (lambda > max_l) { lambda = max_l; j->saturated = 1; }
                double p[3];
                v3_scale(p, n, err > 0 ? lambda : -lambda);
                apply_ang_impulse_pos(A, p, +1.0);
                apply_ang_impulse_pos(B, p, -1.0);
                j->sub_torque += (err > 0 ? -lambda : lambda) / h2;
            }
        }
    } else if (j->type == GRB_JOINT_BALL) {
        double za[3], zb[3], corr[3];
        grb_quat_rotate(qa, Z, za);
        grb_quat_rotate(qb, Z, zb);
        // Swing cone.
        if (j->swing_max < M_PI) {
            double axis[3];
            v3_cross(axis, za, zb);
            double s = v3_len(axis), theta = atan2(s, v3_dot(za, zb));
            if (theta > j->swing_max && s > 1e-12) {
                v3_scale(corr, axis, (theta - j->swing_max) / s);
                solve_angular(A, B, corr, 0.0);
            }
        }
        // Twist about the mean axis.
        if (j->twist_lower > -M_PI || j->twist_upper < M_PI) {
            joint_frame_world(A, j->local_pos_a, j->local_rot_a, pa, qa, ra);
            joint_frame_world(B, j->local_pos_b, j->local_rot_b, pb, qb, rb);
            double n[3], xa[3], xb[3], c[3];
            grb_quat_rotate(qa, Z, za);
            grb_quat_rotate(qb, Z, zb);
            v3_add(n, za, zb);
            double l = v3_len(n);
            if (l > 1e-9) {
                v3_scale(n, n, 1.0 / l);
                grb_quat_rotate(qa, X, xa);
                grb_quat_rotate(qb, X, xb);
                v3_madd(xa, n, -v3_dot(xa, n));
                v3_madd(xb, n, -v3_dot(xb, n));
                v3_cross(c, xa, xb);
                double phi = atan2(v3_dot(c, n), v3_dot(xa, xb));
                double target = phi < j->twist_lower ? j->twist_lower : (phi > j->twist_upper ? j->twist_upper : phi);
                if (target != phi) {
                    v3_scale(corr, n, phi - target);
                    solve_angular(A, B, corr, 0.0);
                }
            }
        }
    }

    // 2. Positional attachment (all joint types).
    joint_frame_world(A, j->local_pos_a, j->local_rot_a, pa, qa, ra);
    joint_frame_world(B, j->local_pos_b, j->local_rot_b, pb, qb, rb);
    double corr[3];
    v3_sub(corr, pb, pa);
    solve_linear(A, B, ra, rb, corr, 0.0);
}

// ---------------------------------------------------------------- contacts

static int shape_points(const GrbBody *b, double pts[8][3], double *radius) {
    switch (b->shape) {
    case GRB_SHAPE_SPHERE:
        v3_set(pts[0], 0, 0, 0);
        *radius = b->radius;
        return 1;
    case GRB_SHAPE_CAPSULE:
        v3_set(pts[0], 0, b->half_height, 0);
        v3_set(pts[1], 0, -b->half_height, 0);
        *radius = b->radius;
        return 2;
    case GRB_SHAPE_BOX: {
        int k = 0;
        for (int sx = -1; sx <= 1; sx += 2)
            for (int sy = -1; sy <= 1; sy += 2)
                for (int sz = -1; sz <= 1; sz += 2)
                    v3_set(pts[k++], sx * b->half_extents[0], sy * b->half_extents[1], sz * b->half_extents[2]);
        *radius = 0.0;
        return 8;
    }
    default:
        return 0;
    }
}

static void solve_contacts(GrbWorld *w) {
    // 1. Detect: every shape point below a plane becomes a contact, remembered in the body frame
    //    (the surface point, so friction/restitution act where the shape actually touches).
    w->contact_count = 0;
    for (uint32_t bi = 0; bi < w->body_count; bi++) {
        GrbBody *b = &w->bodies[bi];
        if (!body_dynamic(b) || b->shape == GRB_SHAPE_NONE) continue;
        double pts[8][3], radius = 0.0;
        int np = shape_points(b, pts, &radius);
        for (uint32_t pi = 0; pi < w->plane_count; pi++) {
            const GrbPlane *pl = &w->planes[pi];
            for (int k = 0; k < np; k++) {
                double p[3];
                grb_body_world_point(b, pts[k], p);
                if (pl->offset - (v3_dot(pl->normal, p) - radius) <= 0.0) continue;
                if (w->contact_count >= GRB_MAX_CONTACTS) break;
                double pc[3], r[3];
                v3_copy(pc, p);
                v3_madd(pc, pl->normal, -radius);
                v3_sub(r, pc, b->pos);
                GrbContact *c = &w->contacts[w->contact_count++];
                c->body = (int)bi;
                quat_rotate_inv(b->rot, r, c->local_point);
                c->lambda_n = 0.0;
                c->plane = (int)pi;
            }
        }
    }
    // 2. Normal projection, a few Gauss-Seidel sweeps. One sweep is XPBD's default, but a box's
    //    four corners coupled through its rotation leave the summed normal impulse ~5% short of
    //    m*g after one sweep (measured), which directly under-scales Coulomb friction.
    int iters = w->contact_iterations > 0 ? w->contact_iterations : 1;
    for (int it = 0; it < iters; it++) {
        for (uint32_t ci = 0; ci < w->contact_count; ci++) {
            GrbContact *c = &w->contacts[ci];
            GrbBody *b = &w->bodies[c->body];
            const GrbPlane *pl = &w->planes[c->plane];
            double p[3], r[3];
            grb_body_world_point(b, c->local_point, p);
            double depth = pl->offset - v3_dot(pl->normal, p);
            if (depth <= 0.0) continue;
            v3_sub(r, p, b->pos);
            double corr[3];
            v3_scale(corr, pl->normal, depth);
            c->lambda_n += solve_linear(b, NULL, r, r, corr, 0.0);
        }
    }
    // 3. Static friction: undo each contact point's tangential slide this substep, when the
    //    required tangential multiplier is inside the Coulomb cone (mu * lambda_n).
    for (uint32_t ci = 0; ci < w->contact_count; ci++) {
        GrbContact *c = &w->contacts[ci];
        GrbBody *b = &w->bodies[c->body];
        const GrbPlane *pl = &w->planes[c->plane];
        double cur[3], prev[3], dp[3], pr[3];
        grb_body_world_point(b, c->local_point, cur);
        grb_quat_rotate(b->prev_rot, c->local_point, pr);
        v3_add(prev, b->prev_pos, pr);
        v3_sub(dp, cur, prev);
        v3_madd(dp, pl->normal, -v3_dot(dp, pl->normal));
        double slide = v3_len(dp);
        if (slide <= 1e-15) continue;
        double t[3], rr[3];
        v3_scale(t, dp, 1.0 / slide);
        v3_sub(rr, cur, b->pos);
        double wt = gen_inv_mass_lin(b, rr, t);
        double lambda_t = wt > 0 ? slide / wt : 0.0;
        if (lambda_t < b->friction * c->lambda_n) {
            double fix[3];
            v3_scale(fix, dp, -1.0);
            solve_linear(b, NULL, rr, rr, fix, 0.0);
        }
    }
}

static void apply_vel_impulse(GrbBody *b, const double r[3], const double p[3]) {
    v3_madd(b->vel, p, b->inv_mass);
    double rp[3], d[3];
    v3_cross(rp, r, p);
    inv_inertia_apply(b, rp, d);
    v3_add(b->omega, b->omega, d);
}

static void solve_contact_velocities(GrbWorld *w, double h) {
    double gmag = v3_len(w->gravity);
    for (uint32_t ci = 0; ci < w->contact_count; ci++) {
        GrbContact *c = &w->contacts[ci];
        GrbBody *b = &w->bodies[c->body];
        const double *n = w->planes[c->plane].normal;
        double r[3], v[3], wr[3], vpre[3];
        grb_quat_rotate(b->rot, c->local_point, r);
        v3_cross(wr, b->omega, r);
        v3_add(v, b->vel, wr);
        v3_cross(wr, b->pre_omega, r);
        v3_add(vpre, b->pre_vel, wr);
        double vn = v3_dot(v, n), vn_pre = v3_dot(vpre, n);
        // Normal: restitution, and removal of any approaching velocity the position pass did not
        // see (e.g. a friction torque tipping a sliding box onto its leading edge). Resting
        // contact gets no bounce, to avoid jitter.
        double e = fabs(vn) <= 2.0 * gmag * h ? 0.0 : b->restitution;
        double target = -e * vn_pre;
        if (target < 0.0) target = 0.0;
        double dvn = -vn + target, pn = 0.0;
        double wn = gen_inv_mass_lin(b, r, n);
        if (fabs(dvn) > 1e-15 && wn > 0.0) {
            pn = dvn / wn;
            double p[3];
            v3_scale(p, n, pn);
            apply_vel_impulse(b, r, p);
        }
        // Dynamic (Coulomb) friction as an impulse: |p_t| <= mu * (total normal impulse at this
        // contact this substep = lambda_n/h from the position pass + any positive normal impulse
        // just applied), and never more than what stops the contact point's tangential slide.
        // Bounding the IMPULSE (rather than the paper's point-velocity change) keeps a sliding
        // box decelerating at mu*g even when a corner's generalized inverse mass is > 1/m.
        v3_cross(wr, b->omega, r);
        v3_add(v, b->vel, wr);
        vn = v3_dot(v, n);
        double vt[3];
        v3_copy(vt, v);
        v3_madd(vt, n, -vn);
        double vtl = v3_len(vt);
        if (vtl > 1e-12) {
            double t[3];
            v3_scale(t, vt, 1.0 / vtl);
            double wt = gen_inv_mass_lin(b, r, t);
            if (wt > 0.0) {
                double normal_impulse = c->lambda_n / h + (pn > 0.0 ? pn : 0.0);
                double pmag = b->friction * normal_impulse, stop = vtl / wt;
                if (pmag > stop) pmag = stop;
                double p[3];
                v3_scale(p, t, -pmag);
                apply_vel_impulse(b, r, p);
            }
        }
    }
}

// ---------------------------------------------------------------- motors

// Explicit (open-loop) motor torques: TORQUE-mode commands and POSITION-mode feed-forward.
// These do not depend on the state being solved, so applying them explicitly is stable. The
// closed-loop parts (PD spring/damper, velocity limit, passive damping) are implicit -- see
// solve_joint's drive and solve_joint_velocities -- because an explicit PD torque applied to one
// link of a Gauss-Seidel-coupled chain goes unstable at the gains a real servo needs (measured on
// the UR5e: a 10 Hz servo tuned to the downstream inertia sagged 15 degrees and chattered at its
// effort limit).
static void apply_motors(GrbWorld *w, double tau_body[][3]) {
    for (uint32_t ji = 0; ji < w->joint_count; ji++) {
        GrbJoint *j = &w->joints[ji];
        j->sub_torque = 0.0;
        if (j->type != GRB_JOINT_HINGE || j->motor == GRB_MOTOR_OFF) continue;
        double tau = j->motor == GRB_MOTOR_TORQUE ? j->command_torque : j->feedforward;
        if (tau == 0.0) continue;
        if (j->effort_limit > 0.0 && fabs(tau) > j->effort_limit) {
            tau = tau > 0 ? j->effort_limit : -j->effort_limit;
            j->saturated = 1;
        }
        double n[3], raw;
        joint_measure(w, j, &raw, n);
        j->sub_torque = tau;
        if (j->body_a >= 0) v3_madd(tau_body[j->body_a], n, -tau);
        v3_madd(tau_body[j->body_b], n, tau);
    }
}

// Angular velocity impulse P (about unit n) on B, reaction on A.
static void apply_ang_vel_impulse(GrbBody *a, GrbBody *b, const double n[3], double P) {
    double p[3], d[3];
    v3_scale(p, n, P);
    if (b) { inv_inertia_apply(b, p, d); v3_add(b->omega, b->omega, d); }
    if (a) { inv_inertia_apply(a, p, d); v3_madd(a->omega, d, -1.0); }
}

// Remaining effort budget for this substep once `used` N*m is already committed.
static double effort_room(const GrbJoint *j, double used) {
    if (j->effort_limit <= 0.0) return 1e300;
    double r = j->effort_limit - fabs(used);
    return r > 0.0 ? r : 0.0;
}

// Velocity-level joint terms, after the position solve: servo damping (kd), passive viscous
// damping, and the rated-speed limit. All implicit (impulses bounded so they can never overshoot)
// and all -- except passive damping -- drawn from the same effort budget as the spring.
static void solve_joint_velocities(GrbWorld *w, double h) {
    for (uint32_t ji = 0; ji < w->joint_count; ji++) {
        GrbJoint *j = &w->joints[ji];
        if (j->type == GRB_JOINT_BALL && j->damping > 0.0) {
            // Passive viscous damping of the full relative angular velocity (ragdoll "muscle
            // tone"), same bounded-impulse form as the hinge case below.
            GrbBody *A = body_ptr(w, j->body_a), *B = body_ptr(w, j->body_b);
            double wa[3] = {0, 0, 0}, d[3];
            if (A) v3_copy(wa, A->omega);
            v3_sub(d, B->omega, wa);
            double m = v3_len(d);
            if (m > 1e-12) {
                double n[3];
                v3_scale(n, d, 1.0 / m);
                double winv = gen_inv_mass_ang(A, n) + gen_inv_mass_ang(B, n);
                double c = j->damping * h * winv;
                if (winv > 0.0) apply_ang_vel_impulse(A, B, n, -m * (c < 1.0 ? c : 1.0) / winv);
            }
            continue;
        }
        if (j->type != GRB_JOINT_HINGE) continue;
        GrbBody *A = body_ptr(w, j->body_a), *B = body_ptr(w, j->body_b);
        double n[3], raw;
        joint_measure(w, j, &raw, n);
        double winv = gen_inv_mass_ang(A, n) + gen_inv_mass_ang(B, n);
        if (winv <= 0.0) continue;
        if (j->damping > 0.0) {
            double wrel = joint_rel_omega(w, j, n);
            double c = j->damping * h * winv;
            apply_ang_vel_impulse(A, B, n, -wrel * (c < 1.0 ? c : 1.0) / winv);
        }
        if (j->motor == GRB_MOTOR_POSITION && j->kd > 0.0) {
            double wrel = joint_rel_omega(w, j, n);
            double c = j->kd * h * winv;
            double P = (j->target_velocity - wrel) * (c < 1.0 ? c : 1.0) / winv;
            double room = effort_room(j, j->sub_torque) * h;
            if (fabs(P) > room) { P = P > 0 ? room : -room; j->saturated = 1; }
            apply_ang_vel_impulse(A, B, n, P);
            j->sub_torque += P / h;
        }
        if (j->motor != GRB_MOTOR_OFF && j->velocity_limit > 0.0) {
            double wrel = joint_rel_omega(w, j, n);
            if (fabs(wrel) > j->velocity_limit) {
                double P = ((wrel > 0 ? j->velocity_limit : -j->velocity_limit) - wrel) / winv;
                apply_ang_vel_impulse(A, B, n, P);
                j->sub_torque += P / h;
            }
        }
    }
}

static void accumulate_motor_torque(GrbWorld *w, int substeps) {
    for (uint32_t ji = 0; ji < w->joint_count; ji++) {
        GrbJoint *j = &w->joints[ji];
        j->applied_torque += j->sub_torque / substeps;
        if (fabs(j->sub_torque) > j->peak_torque) j->peak_torque = fabs(j->sub_torque);
    }
}

static void update_joint_angles(GrbWorld *w, double h) {
    for (uint32_t ji = 0; ji < w->joint_count; ji++) {
        GrbJoint *j = &w->joints[ji];
        if (j->type != GRB_JOINT_HINGE) continue;
        double n[3], raw;
        joint_measure(w, j, &raw, n);
        if (!j->angle_initialized) {
            j->angle = raw;
            j->angle_initialized = 1;
        } else {
            j->angle += wrap_pi(raw - j->raw_angle_prev);
        }
        j->raw_angle_prev = raw;
        j->angle_velocity = joint_rel_omega(w, j, n);
        (void)h;
    }
}

// ---------------------------------------------------------------- step

void grb_world_step(GrbWorld *w, double dt) {
    int ns = w->substeps > 0 ? w->substeps : 1;
    double h = dt / ns;
    w->substep_h = h;
    double (*tau_body)[3] = w->motor_torque_scratch;
    for (uint32_t ji = 0; ji < w->joint_count; ji++) {
        w->joints[ji].saturated = 0;
        w->joints[ji].applied_torque = 0.0;
        w->joints[ji].peak_torque = 0.0;
    }
    update_joint_angles(w, h);

    for (int s = 0; s < ns; s++) {
        memset(tau_body, 0, sizeof(double) * 3 * w->body_count);
        apply_motors(w, tau_body);

        // Integrate.
        for (uint32_t bi = 0; bi < w->body_count; bi++) {
            GrbBody *b = &w->bodies[bi];
            v3_copy(b->prev_pos, b->pos);
            memcpy(b->prev_rot, b->rot, sizeof b->rot);
            if (!body_dynamic(b)) { v3_set(b->vel, 0, 0, 0); v3_set(b->omega, 0, 0, 0); continue; }
            if (b->inv_mass > 0.0) {
                v3_madd(b->vel, w->gravity, h);
                v3_madd(b->vel, b->force, h * b->inv_mass);
            }
            v3_madd(b->pos, b->vel, h);
            // Euler's rigid body equation in the principal frame, gyroscopic term included.
            double tw[3], tl[3], wl[3], Iw[3], gyro[3], dw[3], dww[3];
            v3_add(tw, b->torque, tau_body[bi]);
            quat_rotate_inv(b->rot, tw, tl);
            quat_rotate_inv(b->rot, b->omega, wl);
            for (int k = 0; k < 3; k++) Iw[k] = b->inv_inertia[k] > 0.0 ? wl[k] / b->inv_inertia[k] : 0.0;
            v3_cross(gyro, wl, Iw);
            for (int k = 0; k < 3; k++) dw[k] = b->inv_inertia[k] * (tl[k] - gyro[k]);
            grb_quat_rotate(b->rot, dw, dww);
            v3_madd(b->omega, dww, h);
            double rv[3];
            v3_scale(rv, b->omega, h);
            quat_add_rotvec(b->rot, rv);
            v3_copy(b->pre_vel, b->vel);
            v3_copy(b->pre_omega, b->omega);
        }

        // Positions.
        for (uint32_t ji = 0; ji < w->joint_count; ji++) solve_joint(w, &w->joints[ji]);
        solve_contacts(w);

        // Velocities from positions.
        for (uint32_t bi = 0; bi < w->body_count; bi++) {
            GrbBody *b = &w->bodies[bi];
            if (!body_dynamic(b)) continue;
            for (int k = 0; k < 3; k++) b->vel[k] = (b->pos[k] - b->prev_pos[k]) / h;
            double c[4], dq[4];
            grb_quat_conj(b->prev_rot, c);
            grb_quat_mul(b->rot, c, dq);
            double sgn = dq[3] >= 0.0 ? 1.0 : -1.0;
            v3_set(b->omega, 2.0 * dq[0] / h * sgn, 2.0 * dq[1] / h * sgn, 2.0 * dq[2] / h * sgn);
        }
        solve_contact_velocities(w, h);
        solve_joint_velocities(w, h);
        accumulate_motor_torque(w, ns);
        update_joint_angles(w, h);
    }

    for (uint32_t bi = 0; bi < w->body_count; bi++) {
        v3_set(w->bodies[bi].force, 0, 0, 0);
        v3_set(w->bodies[bi].torque, 0, 0, 0);
    }
}

void grb_joint_sync_angle(GrbWorld *w, int ji, double hint) {
    if (ji < 0 || ji >= (int)w->joint_count) return;
    GrbJoint *j = &w->joints[ji];
    double n[3], raw;
    joint_measure(w, j, &raw, n);
    j->angle = raw + 2.0 * M_PI * floor((hint - raw) / (2.0 * M_PI) + 0.5);
    j->raw_angle_prev = raw;
    j->angle_initialized = 1;
    j->angle_velocity = joint_rel_omega(w, j, n);
}

// ---------------------------------------------------------------- diagnostics

double grb_kinetic_energy(const GrbWorld *w) {
    double e = 0.0;
    for (uint32_t i = 0; i < w->body_count; i++) {
        const GrbBody *b = &w->bodies[i];
        if (b->inv_mass > 0.0) e += 0.5 / b->inv_mass * v3_dot(b->vel, b->vel);
        double wl[3];
        quat_rotate_inv(b->rot, b->omega, wl);
        for (int k = 0; k < 3; k++)
            if (b->inv_inertia[k] > 0.0) e += 0.5 * wl[k] * wl[k] / b->inv_inertia[k];
    }
    return e;
}

double grb_potential_energy(const GrbWorld *w) {
    double e = 0.0;
    for (uint32_t i = 0; i < w->body_count; i++) {
        const GrbBody *b = &w->bodies[i];
        if (b->inv_mass > 0.0) e -= v3_dot(w->gravity, b->pos) / b->inv_mass;
    }
    return e;
}

void grb_linear_momentum(const GrbWorld *w, double o[3]) {
    v3_set(o, 0, 0, 0);
    for (uint32_t i = 0; i < w->body_count; i++)
        if (w->bodies[i].inv_mass > 0.0) v3_madd(o, w->bodies[i].vel, 1.0 / w->bodies[i].inv_mass);
}

void grb_angular_momentum(const GrbWorld *w, double o[3]) {
    v3_set(o, 0, 0, 0);
    for (uint32_t i = 0; i < w->body_count; i++) {
        const GrbBody *b = &w->bodies[i];
        if (b->inv_mass > 0.0) {
            double xv[3];
            v3_cross(xv, b->pos, b->vel);
            v3_madd(o, xv, 1.0 / b->inv_mass);
        }
        double wl[3], Lw[3];
        quat_rotate_inv(b->rot, b->omega, wl);
        for (int k = 0; k < 3; k++) wl[k] = b->inv_inertia[k] > 0.0 ? wl[k] / b->inv_inertia[k] : 0.0;
        grb_quat_rotate(b->rot, wl, Lw);
        v3_add(o, o, Lw);
    }
}

double grb_max_joint_error(const GrbWorld *w) {
    double worst = 0.0;
    for (uint32_t ji = 0; ji < w->joint_count; ji++) {
        const GrbJoint *j = &w->joints[ji];
        double pa[3], qa[4], ra[3], pb[3], qb[4], rb[3], d[3];
        joint_frame_world(j->body_a >= 0 ? &w->bodies[j->body_a] : NULL, j->local_pos_a, j->local_rot_a, pa, qa, ra);
        joint_frame_world(&w->bodies[j->body_b], j->local_pos_b, j->local_rot_b, pb, qb, rb);
        v3_sub(d, pb, pa);
        double e = v3_len(d);
        if (e > worst) worst = e;
    }
    return worst;
}
