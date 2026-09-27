// test_grobot.c — a real Universal Robots UR5e, built from UR's own published mass/inertia/
// kinematics/limits, instantiated as articulated rigid bodies in grb and checked against two
// independent references:
//   1. UR's published DH parameters (zero-pose flange position).
//   2. gbtool's analytic inverse dynamics (RNEA): the torque grb's joint servos actually spend
//      holding a loaded pose must equal the exact gravity torque RNEA computes for that pose.
// Usage: test_grobot <ur5e.grobot> <hold_torques.csv>   (scripts/build_and_test.sh generates both)
#include "../src/grobot.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures = 0;
#define CHECK(cond, ...) do { \
    printf("%s: ", (cond) ? "PASS" : "FAIL"); printf(__VA_ARGS__); printf("\n"); \
    if (!(cond)) failures++; \
} while (0)

#define DEG (M_PI / 180.0)
static const double BASE_P[3] = {0, 0, 0}, BASE_Q[4] = {0, 0, 0, 1};
// Must match the pose scripts/build_and_test.sh bakes for gbtool's RNEA reference.
static const double HOLD_DEG[6] = {0, -60, 45, -60, -90, 0};

static int read_csv_row0(const char *path, double out[6]) {
    FILE *f = fopen(path, "r");
    if (!f) return 0;
    char line[4096];
    if (!fgets(line, sizeof line, f) || !fgets(line, sizeof line, f)) { fclose(f); return 0; }
    fclose(f);
    char *p = strchr(line, ',');
    for (int i = 0; i < 6; i++) {
        if (!p) return 0;
        out[i] = strtod(p + 1, &p);
    }
    return 1;
}

// Critically damped per-joint servo with the same bandwidth on every joint.
static void servo(GrbWorld *w, const GRobot *r, const GRobotInstance *inst, const double *q, double hz) {
    double wn = 2 * M_PI * hz;
    for (uint32_t i = 0; i < r->joint_count; i++) {
        GrbJoint *j = &w->joints[inst->joint[i]];
        double I = grobot_axis_inertia(w, r, inst, (int)i);
        j->motor = GRB_MOTOR_POSITION;
        j->target_angle = q[i];
        j->target_velocity = 0;
        j->kp = I * wn * wn;
        j->kd = 2.0 * I * wn;
    }
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s <ur5e.grobot> <hold_torques.csv>\n", argv[0]);
        return 2;
    }
    static GRobot r;
    CHECK(grobot_init(argv[1], &r), "loads compiled UR5e (%s)", argv[1]);
    CHECK(r.joint_count == 6 && strcmp(r.name, "ur5e") == 0, "6 joints, name %s", r.name);
    double total = 0;
    for (uint32_t i = 0; i < r.joint_count; i++) total += r.joints[i].mass;
    CHECK(fabs(total - (3.761 + 8.058 + 2.846 + 1.37 + 1.3 + 0.365)) < 1e-9, "moving mass %.3f kg = UR's published link masses", total);
    CHECK(fabs(r.joints[0].effort - 150) < 1e-9 && fabs(r.joints[5].effort - 28) < 1e-9 &&
          fabs(r.joints[0].velocity - M_PI) < 1e-12,
          "datasheet limits: shoulder %.0f N*m, wrist3 %.0f N*m, %.0f deg/s", r.joints[0].effort, r.joints[5].effort, r.joints[0].velocity / DEG);

    static GrbWorld w;
    GRobotInstance inst;
    grb_world_init(&w, 0, 0, -9.81);
    CHECK(grobot_spawn(&w, &r, BASE_P, BASE_Q, NULL, &inst), "spawned as %u rigid bodies + %u hinges", w.body_count, w.joint_count);
    double tcp[3];
    grobot_tcp(&w, &r, &inst, tcp);
    double dh[3] = {-0.425 - 0.3922, -(0.1333 + 0.0996), 0.1625 - 0.0997};
    double e = sqrt((tcp[0] - dh[0]) * (tcp[0] - dh[0]) + (tcp[1] - dh[1]) * (tcp[1] - dh[1]) + (tcp[2] - dh[2]) * (tcp[2] - dh[2]));
    CHECK(e < 1e-9, "zero-pose flange (%.4f, %.4f, %.4f) matches UR's DH parameters (err %.1e m)", tcp[0], tcp[1], tcp[2], e);
    double maxang = 0;
    for (int i = 0; i < 6; i++) if (fabs(w.joints[inst.joint[i]].angle) > maxang) maxang = fabs(w.joints[inst.joint[i]].angle);
    CHECK(maxang < 1e-12, "every hinge reads 0 at the zero pose (max %.1e rad)", maxang);

    // Hold a loaded pose; compare the servo effort to RNEA's exact gravity torques.
    double q[6], ref[6];
    for (int i = 0; i < 6; i++) q[i] = HOLD_DEG[i] * DEG;
    grobot_set_state(&w, &r, &inst, q, NULL);
    for (int i = 0; i < 6; i++)
        CHECK(fabs(w.joints[inst.joint[i]].angle - q[i]) < 1e-9, "set_state: %s reads %.4f deg", r.joints[i].name, w.joints[inst.joint[i]].angle / DEG);
    servo(&w, &r, &inst, q, 20.0);
    // Settle, then average the servo effort over one more second (the stiff wrist servos carry
    // a sub-N*m ripple from the Gauss-Seidel coupling; the mean is what statics predicts).
    for (int t = 0; t < 64 * 2; t++) grb_world_step(&w, 1.0 / 64);
    double mean[6] = {0}, maxdev = 0;
    for (int t = 0; t < 64; t++) {
        grb_world_step(&w, 1.0 / 64);
        for (int i = 0; i < 6; i++) {
            mean[i] += w.joints[inst.joint[i]].applied_torque / 64.0;
            double d = fabs(w.joints[inst.joint[i]].angle - q[i]);
            if (d > maxdev) maxdev = d;
        }
    }
    CHECK(maxdev < 1.0 * DEG, "20 Hz servos hold the pose to within %.3f deg", maxdev / DEG);
    CHECK(read_csv_row0(argv[2], ref), "read RNEA reference torques (%s)", argv[2]);
    for (int i = 0; i < 6; i++) {
        double tol = fmax(0.01 * fabs(ref[i]), 0.1); // 1% or 0.1 N*m (<0.1% of rated effort)
        CHECK(fabs(mean[i] - ref[i]) < tol, "%-20s servo holds with %8.3f N*m, RNEA gravity torque %8.3f N*m (err %.3f)",
              r.joints[i].name, mean[i], ref[i], fabs(mean[i] - ref[i]));
    }
    CHECK(grb_max_joint_error(&w) < 5e-4, "joint attachment drift %.2e m under load", grb_max_joint_error(&w));

    // The same pose with the shoulder motor derated below its gravity load must sag: the effort
    // limit is a real physical ceiling, not a bookkeeping number.
    grb_world_init(&w, 0, 0, -9.81);
    grobot_spawn(&w, &r, BASE_P, BASE_Q, q, &inst);
    servo(&w, &r, &inst, q, 10.0);
    w.joints[inst.joint[1]].effort_limit = 0.5 * fabs(ref[1]);
    for (int t = 0; t < 64; t++) grb_world_step(&w, 1.0 / 64);
    double sag = fabs(w.joints[inst.joint[1]].angle - q[1]);
    CHECK(sag > 0.2 && w.joints[inst.joint[1]].saturated,
          "shoulder derated to %.1f N*m (< %.1f needed) cannot hold: sagged %.1f deg, saturated", 0.5 * fabs(ref[1]), fabs(ref[1]), sag / DEG);

    printf("%s: %d failure(s)\n", failures ? "FAILED" : "OK", failures);
    return failures ? 1 : 0;
}
