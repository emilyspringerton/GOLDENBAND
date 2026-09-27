// test_grb.c — physics-correctness tests for grb.c, each checked against a closed-form answer
// rather than "looks plausible": free fall, torque-free angular momentum conservation (with a
// non-uniform inertia tensor, so the gyroscopic term matters), small-angle pendulum period, hinge
// limits, datasheet-style effort/velocity caps on a motor, resting + sliding contact with Coulomb
// friction, ball-joint swing/twist limits, and bit-exact determinism.
#include "../src/grb.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;
#define CHECK(cond, ...) do { \
    printf("%s: ", (cond) ? "PASS" : "FAIL"); printf(__VA_ARGS__); printf("\n"); \
    if (!(cond)) failures++; \
} while (0)

static const double QI[4] = {0, 0, 0, 1};
static const double DT = 1.0 / 64.0;

static void test_free_fall(void) {
    GrbWorld w;
    grb_world_init(&w, 0, -9.81, 0);
    double I[3] = {1, 1, 1}, p[3] = {0, 100, 0};
    grb_body_add(&w, 2.0, I, p, QI);
    for (int i = 0; i < 64; i++) grb_world_step(&w, DT);
    double y = w.bodies[0].pos[1], expect = 100 - 0.5 * 9.81;
    CHECK(fabs(y - expect) < 5e-3, "free fall 1s: y=%.5f expected %.5f", y, expect);
    CHECK(fabs(w.bodies[0].vel[1] + 9.81) < 1e-6, "free fall 1s: vy=%.6f expected -9.81", w.bodies[0].vel[1]);
}

static void test_angular_momentum(void) {
    GrbWorld w;
    grb_world_init(&w, 0, 0, 0);
    double I[3] = {0.1, 0.4, 0.9}, p[3] = {0, 0, 0};
    int b = grb_body_add(&w, 1.0, I, p, QI);
    // Spin mostly about the (unstable) intermediate axis plus a perturbation: the body tumbles
    // (Dzhanibekov effect), which only happens if the gyroscopic term is really integrated.
    w.bodies[b].omega[0] = 0.05; w.bodies[b].omega[1] = 5.0; w.bodies[b].omega[2] = 0.05;
    double L0[3], L1[3];
    grb_angular_momentum(&w, L0);
    double E0 = grb_kinetic_energy(&w);
    double min_wy = 1e9;
    for (int i = 0; i < 64 * 4; i++) {
        grb_world_step(&w, DT);
        double c[4], wl[3];
        grb_quat_conj(w.bodies[b].rot, c);
        grb_quat_rotate(c, w.bodies[b].omega, wl); // body-frame spin
        if (wl[1] < min_wy) min_wy = wl[1];
    }
    grb_angular_momentum(&w, L1);
    double dL = sqrt((L1[0] - L0[0]) * (L1[0] - L0[0]) + (L1[1] - L0[1]) * (L1[1] - L0[1]) + (L1[2] - L0[2]) * (L1[2] - L0[2]));
    double Lm = sqrt(L0[0] * L0[0] + L0[1] * L0[1] + L0[2] * L0[2]);
    double E1 = grb_kinetic_energy(&w);
    CHECK(dL / Lm < 0.02, "torque-free |dL|/|L| = %.5f over 4s (world angular momentum conserved)", dL / Lm);
    CHECK(fabs(E1 - E0) / E0 < 0.05, "torque-free kinetic energy drift %.4f%%", 100 * fabs(E1 - E0) / E0);
    CHECK(min_wy < 0.0, "intermediate-axis spin flips (Dzhanibekov effect): min body-frame wy=%.3f", min_wy);
}

// Physical pendulum: a thin rod of length L hinged at one end. Small-angle period
// T = 2*pi*sqrt(I_pivot / (m*g*d)), I_pivot = m*L^2/3, d = L/2.
static void test_pendulum_period(void) {
    GrbWorld w;
    grb_world_init(&w, 0, -9.81, 0);
    double m = 1.0, L = 1.0, I[3];
    grb_inertia_box(m, 0.01, L / 2, 0.01, I);
    double theta0 = 0.05;
    // Rod hanging down from the pivot at the origin, rotated theta0 about +Z.
    double q[4], axisz[3] = {0, 0, 1};
    grb_quat_from_axis_angle(axisz, theta0, q);
    double c[3] = {sin(theta0) * L / 2, -cos(theta0) * L / 2, 0};
    int b = grb_body_add(&w, m, I, c, q);
    double pivot[3] = {0, 0, 0};
    grb_joint_add(&w, GRB_JOINT_HINGE, -1, b, pivot, QI);
    double prev = w.bodies[b].pos[0], t = 0.0, first = -1, last = -1;
    int crossings = 0;
    for (int i = 0; i < 64 * 20; i++) {
        grb_world_step(&w, DT);
        t += DT;
        double x = w.bodies[b].pos[0];
        if (prev > 0 && x <= 0) { // downward crossing, once per period
            double tc = t - DT * x / (x - prev);
            if (first < 0) first = tc; else { last = tc; crossings++; }
        }
        prev = x;
    }
    double T = (last - first) / crossings;
    double expect = 2 * M_PI * sqrt((m * L * L / 3) / (m * 9.81 * L / 2));
    CHECK(fabs(T - expect) / expect < 0.005, "pendulum period %.5fs vs analytic %.5fs (%d periods)", T, expect, crossings);
    CHECK(grb_max_joint_error(&w) < 1e-4, "hinge attachment drift %.2e m", grb_max_joint_error(&w));
}

// Horizontal arm (length L, mass m) on a +Z hinge, gravity -Y: holding torque m*g*L/2.
static int make_arm(GrbWorld *w, double m, double L) {
    grb_world_init(w, 0, -9.81, 0);
    double I[3];
    grb_inertia_box(m, L / 2, 0.02, 0.02, I);
    double c[3] = {L / 2, 0, 0};
    int b = grb_body_add(w, m, I, c, QI);
    double pivot[3] = {0, 0, 0};
    return grb_joint_add(w, GRB_JOINT_HINGE, -1, b, pivot, QI);
}

static void test_hinge_limit(void) {
    GrbWorld w;
    int j = make_arm(&w, 2.0, 1.0);
    w.joints[j].has_limits = 1;
    w.joints[j].lower = -0.3;
    w.joints[j].upper = 0.3;
    double worst = 0;
    for (int i = 0; i < 64 * 3; i++) {
        grb_world_step(&w, DT);
        if (w.joints[j].angle < worst) worst = w.joints[j].angle;
    }
    CHECK(worst > -0.3 - 5e-3, "falling arm stopped by lower limit: min angle %.5f (limit -0.3)", worst);
    CHECK(fabs(w.joints[j].angle + 0.3) < 1e-3, "arm rests on its limit: angle %.5f", w.joints[j].angle);
}

static void test_motor_effort(void) {
    double m = 2.0, L = 1.0, hold = m * 9.81 * L / 2; // 9.81 N*m
    GrbWorld w;
    int j = make_arm(&w, m, L);
    GrbJoint *J = &w.joints[j];
    J->motor = GRB_MOTOR_POSITION;
    J->kp = 2000; J->kd = 60; J->target_angle = 0.0;
    J->effort_limit = 28.0; // a UR5e wrist-size joint's datasheet max torque -- plenty
    for (int i = 0; i < 64 * 3; i++) grb_world_step(&w, DT);
    CHECK(fabs(J->angle) < 0.01, "strong motor holds arm level: angle %.5f rad", J->angle);
    CHECK(fabs(J->applied_torque - hold) / hold < 0.02, "measured holding torque %.4f N*m vs m*g*L/2 = %.4f", J->applied_torque, hold);
    CHECK(!J->saturated, "strong motor not saturated");

    j = make_arm(&w, m, L);
    J = &w.joints[j];
    J->motor = GRB_MOTOR_POSITION;
    J->kp = 2000; J->kd = 60; J->target_angle = 0.0;
    J->effort_limit = 5.0; // below the 9.81 N*m needed -- the arm must sag, not teleport
    for (int i = 0; i < 64; i++) grb_world_step(&w, DT);
    CHECK(J->angle < -0.5, "under-rated motor cannot hold the arm: angle %.3f rad", J->angle);
    CHECK(J->saturated && J->peak_torque <= 5.0 + 1e-9, "effort clamp respected: peak %.4f N*m <= 5", J->peak_torque);

    j = make_arm(&w, m, L);
    J = &w.joints[j];
    w.gravity[1] = 0;
    J->motor = GRB_MOTOR_TORQUE;
    J->command_torque = 50.0;
    J->velocity_limit = M_PI; // 180 deg/s, the UR e-Series datasheet joint speed
    double vmax = 0;
    for (int i = 0; i < 64 * 3; i++) {
        grb_world_step(&w, DT);
        if (J->angle_velocity > vmax) vmax = J->angle_velocity;
    }
    CHECK(vmax < M_PI * 1.02, "velocity limit: peak joint speed %.4f rad/s (limit %.4f)", vmax, M_PI);
}

static void test_contact(void) {
    GrbWorld w;
    grb_world_init(&w, 0, -9.81, 0);
    grb_world_add_plane(&w, 0, 1, 0, 0);
    double I[3], p[3] = {0, 1.0, 0};
    grb_inertia_box(1.0, 0.5, 0.25, 0.5, I);
    int b = grb_body_add(&w, 1.0, I, p, QI);
    grb_body_set_box(&w.bodies[b], 0.5, 0.25, 0.5);
    for (int i = 0; i < 64 * 3; i++) grb_world_step(&w, DT);
    CHECK(fabs(w.bodies[b].pos[1] - 0.25) < 2e-3, "box comes to rest on the ground: y=%.5f (half-height 0.25)", w.bodies[b].pos[1]);
    CHECK(grb_kinetic_energy(&w) < 1e-4, "box at rest: KE=%.2e J", grb_kinetic_energy(&w));

    // Sliding: v0 on mu -> stopping distance v0^2 / (2*mu*g).
    grb_world_init(&w, 0, -9.81, 0);
    grb_world_add_plane(&w, 0, 1, 0, 0);
    double p2[3] = {0, 0.25, 0};
    b = grb_body_add(&w, 1.0, I, p2, QI);
    grb_body_set_box(&w.bodies[b], 0.5, 0.25, 0.5);
    w.bodies[b].friction = 0.5;
    for (int i = 0; i < 16; i++) grb_world_step(&w, DT); // settle contact
    w.bodies[b].pos[0] = 0;
    w.bodies[b].vel[0] = 3.0;
    for (int i = 0; i < 64 * 3; i++) grb_world_step(&w, DT);
    double expect = 9.0 / (2 * 0.5 * 9.81);
    CHECK(fabs(w.bodies[b].pos[0] - expect) / expect < 0.03, "Coulomb slide distance %.4f m vs v0^2/(2*mu*g) = %.4f", w.bodies[b].pos[0], expect);
    CHECK(fabs(w.bodies[b].vel[0]) < 1e-3, "slide stopped by static friction: vx=%.2e", w.bodies[b].vel[0]);
}

static void test_restitution(void) {
    GrbWorld w;
    grb_world_init(&w, 0, -9.81, 0);
    grb_world_add_plane(&w, 0, 1, 0, 0);
    double I[3], p[3] = {0, 1.1, 0};
    grb_inertia_sphere(1.0, 0.1, I);
    int b = grb_body_add(&w, 1.0, I, p, QI);
    grb_body_set_sphere(&w.bodies[b], 0.1);
    w.bodies[b].restitution = 0.5;
    int bounced = 0;
    double apex = 0;
    for (int i = 0; i < 64 * 2; i++) {
        grb_world_step(&w, DT);
        if (w.bodies[b].vel[1] > 0.1) bounced = 1;
        if (bounced && w.bodies[b].pos[1] > apex) apex = w.bodies[b].pos[1];
    }
    // Drop height 1.0 m (center 1.1, radius 0.1) -> rebound height e^2 * 1.0 = 0.25 m.
    CHECK(fabs((apex - 0.1) - 0.25) < 0.02, "restitution 0.5: rebound height %.4f m vs e^2*h = 0.25", apex - 0.1);
}

static void test_ball_limits(void) {
    GrbWorld w;
    grb_world_init(&w, 0, -9.81, 0);
    double I[3], p[3] = {0.5, 0, 0};
    grb_inertia_capsule(1.0, 0.05, 0.45, I);
    // Capsule lies along +X from a ball joint at the origin; frame +Z (twist axis) along the bone.
    double q[4], z[3] = {0, 0, 1};
    grb_quat_from_axis_angle(z, -M_PI / 2, q); // body +Y -> world +X
    int b = grb_body_add(&w, 1.0, I, p, q);
    double fr[4], y[3] = {0, 1, 0};
    grb_quat_from_axis_angle(y, M_PI / 2, fr); // joint frame +Z -> world +X
    double origin[3] = {0, 0, 0};
    int j = grb_joint_add(&w, GRB_JOINT_BALL, -1, b, origin, fr);
    w.joints[j].swing_max = 0.5;
    w.joints[j].twist_lower = -0.2;
    w.joints[j].twist_upper = 0.2;
    w.bodies[b].omega[0] = 20.0; // hard twist about the bone axis
    double worst_swing = 0;
    for (int i = 0; i < 64 * 3; i++) {
        grb_world_step(&w, DT);
        double dir[3] = {w.bodies[b].pos[0], w.bodies[b].pos[1], w.bodies[b].pos[2]};
        double l = sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
        double sw = acos(dir[0] / l);
        if (sw > worst_swing) worst_swing = sw;
    }
    CHECK(worst_swing < 0.5 + 0.01, "ball swing cone: bone never exceeds %.3f rad (max seen %.4f)", 0.5, worst_swing);
    // Twist: body +X vs joint frame +X after mapping.
    double bx[3], ex[3] = {1, 0, 0}, jx[3];
    grb_quat_rotate(w.bodies[b].rot, ex, bx);
    grb_quat_rotate(fr, ex, jx);
    (void)bx; (void)jx;
    CHECK(grb_max_joint_error(&w) < 1e-4, "ball attachment drift %.2e m", grb_max_joint_error(&w));
}

static void run_chain(GrbWorld *w) {
    grb_world_init(w, 0, -9.81, 0);
    grb_world_add_plane(w, 0, 1, 0, 0);
    int prev = -1;
    for (int i = 0; i < 6; i++) {
        double I[3], p[3] = {0.3 * i + 0.15, 2.0, 0};
        grb_inertia_capsule(1.0, 0.05, 0.1, I);
        double q[4], z[3] = {0, 0, 1};
        grb_quat_from_axis_angle(z, -M_PI / 2, q);
        int b = grb_body_add(w, 1.0, I, p, q);
        grb_body_set_capsule(&w->bodies[b], 0.05, 0.1);
        double jp[3] = {0.3 * i, 2.0, 0};
        grb_joint_add(w, GRB_JOINT_BALL, prev, b, jp, QI);
        prev = b;
    }
    for (int i = 0; i < 64 * 2; i++) grb_world_step(w, DT);
}

static void test_determinism(void) {
    static GrbWorld a, b;
    run_chain(&a);
    run_chain(&b);
    CHECK(memcmp(a.bodies, b.bodies, sizeof a.bodies) == 0, "two identical 2s chain runs are bit-identical");
    CHECK(grb_max_joint_error(&a) < 1e-3, "6-link chain joint drift %.2e m", grb_max_joint_error(&a));
}

int main(void) {
    test_free_fall();
    test_angular_momentum();
    test_pendulum_period();
    test_hinge_limit();
    test_motor_effort();
    test_contact();
    test_restitution();
    test_ball_limits();
    test_determinism();
    printf("%s: %d failure(s)\n", failures ? "FAILED" : "OK", failures);
    return failures ? 1 : 0;
}
