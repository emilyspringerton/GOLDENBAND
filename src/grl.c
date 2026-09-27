// grl.c — see grl.h.
#include "grl.h"

#include <ctype.h>
#include <stddef.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "sha256.h"
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------------------------------------------------------- rng

uint64_t grl_rng_next(uint64_t *s) {
    uint64_t z = (*s += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

double grl_rng_uniform(uint64_t *s) { return (grl_rng_next(s) >> 11) * (1.0 / 9007199254740992.0); }

double grl_rng_normal(uint64_t *s) {
    double u1 = grl_rng_uniform(s), u2 = grl_rng_uniform(s);
    if (u1 < 1e-300) u1 = 1e-300;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

// ---------------------------------------------------------------- profile

void grl_profile_default(GrlRewardProfile *p) {
    p->w_pose = 0.5;
    p->w_vel = 0.1;
    p->w_ee = 0.4;
    p->w_energy = 0.02;
    p->w_smooth = 0.05;
    p->w_limit = 0.2;
    p->k_pose = 200.0;      // 5 deg total error -> exp(-1.5)
    p->k_vel = 0.5;
    p->k_ee = 2000.0;       // 1 cm -> exp(-0.2)
    p->energy_scale = 200.0;
    p->servo_hz = 5.0;
    p->servo_zeta = 1.0;
    p->action_scale = 0.2;
    p->dr_mass_jitter = 0.1;
}

typedef struct { const char *key; size_t off; } ProfKey;
#define PK(k) {#k, offsetof(GrlRewardProfile, k)}
static const ProfKey prof_keys[] = {
    PK(w_pose), PK(w_vel), PK(w_ee), PK(w_energy), PK(w_smooth), PK(w_limit),
    PK(k_pose), PK(k_vel), PK(k_ee), PK(energy_scale),
    PK(servo_hz), PK(servo_zeta), PK(action_scale), PK(dr_mass_jitter),
};
#define N_PROF_KEYS (sizeof prof_keys / sizeof prof_keys[0])

int grl_profile_load(const char *path, GrlRewardProfile *p) {
    FILE *f = fopen(path, "r");
    if (!f) return 0;
    char line[256];
    int ok = 1, lineno = 0;
    while (fgets(line, sizeof line, f)) {
        lineno++;
        char *h = strchr(line, '#');
        if (h) *h = 0;
        char *eq = strchr(line, '=');
        char *s = line;
        while (isspace((unsigned char)*s)) s++;
        if (!*s) continue;
        if (!eq) { fprintf(stderr, "%s:%d: expected key = value\n", path, lineno); ok = 0; break; }
        *eq = 0;
        char *k = s, *ke = eq - 1;
        while (ke > k && isspace((unsigned char)*ke)) *ke-- = 0;
        char *end;
        double v = strtod(eq + 1, &end);
        while (isspace((unsigned char)*end)) end++;
        if (end == eq + 1 || *end) { fprintf(stderr, "%s:%d: bad number for %s\n", path, lineno, k); ok = 0; break; }
        size_t i;
        for (i = 0; i < N_PROF_KEYS; i++)
            if (strcmp(prof_keys[i].key, k) == 0) { *(double *)((char *)p + prof_keys[i].off) = v; break; }
        if (i == N_PROF_KEYS) { fprintf(stderr, "%s:%d: unknown reward-profile key %s\n", path, lineno, k); ok = 0; break; }
    }
    fclose(f);
    return ok;
}

int grl_profile_format(const GrlRewardProfile *p, char *buf, int cap) {
    int n = 0;
    for (size_t i = 0; i < N_PROF_KEYS && n < cap; i++)
        n += snprintf(buf + n, (size_t)(cap - n), "%s = %.17g\n", prof_keys[i].key,
                      *(const double *)((const char *)p + prof_keys[i].off));
    return n;
}

// ---------------------------------------------------------------- manifest channels

int grl_manifest_channels(const char *path, char names[][64], int max) {
    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    static char buf[1 << 20];
    size_t len = fread(buf, 1, sizeof buf - 1, f);
    fclose(f);
    buf[len] = 0;
    char *p = strstr(buf, "\"channels\"");
    if (!p || !(p = strchr(p, '['))) return -1;
    char *end = strchr(p, ']');
    if (!end) return -1;
    int n = 0;
    while (p < end) {
        char *a = strchr(p, '"');
        if (!a || a > end) break;
        char *b = strchr(a + 1, '"');
        if (!b || b > end || b - a - 1 >= 64 || n >= max) return -1;
        memcpy(names[n], a + 1, (size_t)(b - a - 1));
        names[n][b - a - 1] = 0;
        n++;
        p = b + 1;
    }
    return n;
}

// ---------------------------------------------------------------- env

int grl_env_init(GrlEnv *e, const GRobot *robot, const GBClip *clip, char names[][64], int nch,
                 const GrlRewardProfile *prof, int harmonics) {
    memset(e, 0, sizeof *e);
    e->robot = robot;
    e->clip = clip;
    e->prof = *prof;
    if (harmonics < 0) harmonics = 0;
    if (harmonics > GRL_MAX_HARMONICS) harmonics = GRL_MAX_HARMONICS;
    e->harmonics = harmonics;
    e->act_dim = (int)robot->joint_count;
    e->obs_dim = 2 * e->act_dim + 2 * harmonics + 1;
    int found = 0;
    for (uint32_t j = 0; j < robot->joint_count; j++) {
        e->chan[j] = -1;
        char want[64];
        snprintf(want, sizeof want, "%s.angle", robot->joints[j].name);
        for (int c = 0; c < nch; c++)
            if (strcmp(names[c], want) == 0) { e->chan[j] = c; found++; }
        e->nominal_mass[j] = robot->joints[j].mass;
    }
    if (!found || clip->duration_ticks < 3 || clip->tick_rate == 0) return 0;
    // Compiled reward identity: robot spec + clip content + profile + policy feature set.
    sha256_ctx ctx;
    sha256_init(&ctx);
    sha256_update(&ctx, robot->spec_hash, 32);
    sha256_update(&ctx, clip->content_hash, 32);
    char txt[2048];
    int n = grl_profile_format(prof, txt, sizeof txt);
    n += snprintf(txt + n, sizeof txt - (size_t)n, "harmonics = %d\n", harmonics);
    sha256_update(&ctx, (const uint8_t *)txt, (size_t)n);
    sha256_final(&ctx, e->reward_id);
    return 1;
}

void grl_reference(const GrlEnv *e, int tick, double *q, double *qd) {
    int T = (int)e->clip->duration_ticks;
    int t0 = tick < 0 ? 0 : (tick >= T ? T - 1 : tick);
    int tm = t0 > 0 ? t0 - 1 : 0, tp = t0 < T - 1 ? t0 + 1 : T - 1;
    double span = (double)(tp - tm) / e->clip->tick_rate;
    const float *a = gb_sample(e->clip, (uint32_t)t0), *m = gb_sample(e->clip, (uint32_t)tm), *p = gb_sample(e->clip, (uint32_t)tp);
    for (int j = 0; j < e->act_dim; j++) {
        int c = e->chan[j];
        q[j] = c >= 0 ? a[c] : 0.0;
        if (qd) qd[j] = (c >= 0 && span > 0) ? (p[c] - m[c]) / span : 0.0;
    }
}

static void ref_tcp(const GrlEnv *e, const double *q, double out[3]) {
    double lp[GROBOT_MAX_JOINTS][3], lq[GROBOT_MAX_JOINTS][4];
    const double bp[3] = {0, 0, 0}, bq[4] = {0, 0, 0, 1};
    grobot_fk(e->robot, bp, bq, q, lp, lq);
    int t = e->robot->tcp_joint;
    if (t < 0) { out[0] = out[1] = out[2] = 0; return; }
    double r[3];
    grb_quat_rotate(lq[t], e->robot->tcp_xyz, r);
    for (int k = 0; k < 3; k++) out[k] = lp[t][k] + r[k];
}

static void build_obs(const GrlEnv *e, const double *qref, double *obs) {
    int n = e->act_dim, k = 0;
    for (int j = 0; j < n; j++) obs[k++] = e->world.joints[e->inst.joint[j]].angle - qref[j];
    for (int j = 0; j < n; j++) obs[k++] = 0.1 * e->world.joints[e->inst.joint[j]].angle_velocity;
    double phase = (double)e->tick / (double)(e->clip->duration_ticks - 1);
    for (int h = 1; h <= e->harmonics; h++) {
        obs[k++] = sin(2.0 * M_PI * h * phase);
        obs[k++] = cos(2.0 * M_PI * h * phase);
    }
    obs[k++] = 1.0;
}

static void policy_act(const GrlPolicy *pol, const double *obs, int obs_dim, int act_dim, double *a) {
    for (int i = 0; i < act_dim; i++) {
        double s = 0.0;
        if (pol)
            for (int k = 0; k < obs_dim; k++) s += pol->w[i * obs_dim + k] * obs[k];
        a[i] = s > 1.0 ? 1.0 : (s < -1.0 ? -1.0 : s);
    }
}

double grl_env_rollout(GrlEnv *e, const GrlPolicy *pol, uint64_t dr_seed, float *out_angles) {
    const GRobot *r = e->robot;
    int n = e->act_dim, T = (int)e->clip->duration_ticks;
    double dt = 1.0 / e->clip->tick_rate;
    double q0[GROBOT_MAX_JOINTS], qd0[GROBOT_MAX_JOINTS];
    grl_reference(e, 0, q0, qd0);

    grb_world_init(&e->world, 0, 0, -9.81);
    const double bp[3] = {0, 0, 0}, bq[4] = {0, 0, 0, 1};
    grobot_spawn(&e->world, r, bp, bq, q0, &e->inst);
    // Servo gains from the NOMINAL robot (a real controller is tuned once, on the datasheet
    // model -- it does not know this episode's randomized masses).
    double wn = 2.0 * M_PI * e->prof.servo_hz;
    for (int j = 0; j < n; j++) {
        GrbJoint *gj = &e->world.joints[e->inst.joint[j]];
        double I = grobot_axis_inertia(&e->world, r, &e->inst, j);
        gj->motor = GRB_MOTOR_POSITION;
        gj->kp = I * wn * wn;
        gj->kd = 2.0 * e->prof.servo_zeta * I * wn;
    }
    if (dr_seed && e->prof.dr_mass_jitter > 0.0) {
        uint64_t s = dr_seed;
        for (int j = 0; j < n; j++) {
            double f = 1.0 + e->prof.dr_mass_jitter * (2.0 * grl_rng_uniform(&s) - 1.0);
            GrbBody *b = &e->world.bodies[e->inst.body[j]];
            b->inv_mass /= f;
            for (int k = 0; k < 3; k++) b->inv_inertia[k] /= f;
        }
    }
    grobot_set_state(&e->world, r, &e->inst, q0, qd0);

    memset(e->term, 0, sizeof e->term);
    memset(e->prev_action, 0, sizeof e->prev_action);
    e->ret = 0.0;
    e->saturated_ticks = 0;
    e->peak_speed = e->peak_torque = 0.0;
    if (out_angles)
        for (int j = 0; j < n; j++) out_angles[j] = (float)q0[j];

    double obs[GRL_MAX_OBS], a[GROBOT_MAX_JOINTS], qref[GROBOT_MAX_JOINTS], qdref[GROBOT_MAX_JOINTS];
    int steps = T - 1;
    for (int t = 0; t < steps; t++) {
        e->tick = (uint32_t)t;
        grl_reference(e, t, qref, NULL);
        build_obs(e, qref, obs);
        policy_act(pol, obs, e->obs_dim, n, a);
        grl_reference(e, t + 1, qref, qdref);
        for (int j = 0; j < n; j++) {
            GrbJoint *gj = &e->world.joints[e->inst.joint[j]];
            gj->target_angle = qref[j] + e->prof.action_scale * a[j];
            gj->target_velocity = qdref[j];
        }
        grb_world_step(&e->world, dt);

        double pose = 0, vel = 0, energy = 0, smooth = 0, sat = 0;
        for (int j = 0; j < n; j++) {
            GrbJoint *gj = &e->world.joints[e->inst.joint[j]];
            double dq = gj->angle - qref[j], dv = gj->angle_velocity - qdref[j];
            pose += dq * dq;
            vel += dv * dv;
            energy += fabs(gj->applied_torque * gj->angle_velocity);
            smooth += (a[j] - e->prev_action[j]) * (a[j] - e->prev_action[j]);
            sat += gj->saturated ? 1.0 : 0.0;
            e->prev_action[j] = a[j];
            if (fabs(gj->angle_velocity) > e->peak_speed) e->peak_speed = fabs(gj->angle_velocity);
            if (gj->peak_torque > e->peak_torque) e->peak_torque = gj->peak_torque;
            if (out_angles) out_angles[(t + 1) * n + j] = (float)gj->angle;
        }
        if (sat > 0) e->saturated_ticks++;
        double tcp[3], tref[3];
        grobot_tcp(&e->world, r, &e->inst, tcp);
        ref_tcp(e, qref, tref);
        double ee = (tcp[0] - tref[0]) * (tcp[0] - tref[0]) + (tcp[1] - tref[1]) * (tcp[1] - tref[1]) + (tcp[2] - tref[2]) * (tcp[2] - tref[2]);
        double terms[6] = {exp(-e->prof.k_pose * pose), exp(-e->prof.k_vel * vel), exp(-e->prof.k_ee * ee),
                           -energy / e->prof.energy_scale, -smooth, -sat / n};
        const double w[6] = {e->prof.w_pose, e->prof.w_vel, e->prof.w_ee, e->prof.w_energy, e->prof.w_smooth, e->prof.w_limit};
        for (int k = 0; k < 6; k++) {
            e->term[k] += terms[k] / steps;
            e->ret += w[k] * terms[k] / steps;
        }
    }
    return e->ret;
}
