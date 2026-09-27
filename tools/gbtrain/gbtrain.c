// gbtrain — GOLDEN BAND's v0 training backbone for the RL animation pipeline (HQ-SPEC-SIM-100
// §4/§8 step 3). Takes an authored clip, a datasheet robot and a reward profile; trains a policy
// that makes the PHYSICAL robot (effort- and speed-limited servos, randomized link masses)
// honor the clip; writes out:
//
//   <out>.gpolicy            binary policy artifact (weights + the hashes it was compiled from)
//   <out>.gpolicy.json       provenance + frozen-eval report (baseline vs trained, per term)
//   <out>_rollout.gband(.json)  the motion the trained policy ACTUALLY produced in physics --
//                            a new, physically-honest animation, labeled generative, with the
//                            peak joint speed/torque it really used in its safety block
//
// Optimizer: the cross-entropy method (Rubinstein 1999; the standard derivative-free policy
// search baseline) over a linear policy. Deterministic for a given --seed. Training fitness and
// the frozen eval use DISJOINT domain-randomization seed streams, so the trainer never grades
// its own homework on the eval set (SIM-100 §6 rule 5, in miniature).
//
//   gbtrain --robot ur5e.grobot --clip wave [--profile reward.txt] --out wave_policy
//           [--iters 30] [--pop 32] [--elites 6] [--dr 2] [--harmonics 3] [--seed 1] [--eval-seeds 8]
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../src/gband.h"
#include "../../src/grl.h"
#include "../../src/grobot.h"

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "../../src/sha256.h"
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

static const char *TERM_NAMES[6] = {"pose", "vel", "ee", "energy", "smooth", "limit"};

typedef struct {
    double ret, term[6];
    double saturated_frac, peak_speed, peak_torque;
} EvalResult;

static void hex32(const unsigned char *h, char *out) {
    for (int i = 0; i < 32; i++) sprintf(out + 2 * i, "%02x", h[i]);
}

static uint64_t mix(uint64_t a, uint64_t b) {
    uint64_t s = a ^ (b * 0x9E3779B97F4A7C15ull);
    return grl_rng_next(&s) | 1ull; // never 0: 0 means "nominal physics"
}

// Frozen eval: nominal physics + n held-out DR samples, all averaged.
static EvalResult evaluate(GrlEnv *e, const GrlPolicy *pol, uint64_t seed, int n) {
    EvalResult r;
    memset(&r, 0, sizeof r);
    int total = n + 1;
    for (int i = 0; i < total; i++) {
        uint64_t dr = i == 0 ? 0 : mix(seed ^ 0xE7A1E7A1E7A1E7A1ull, (uint64_t)i);
        r.ret += grl_env_rollout(e, pol, dr, NULL) / total;
        for (int k = 0; k < 6; k++) r.term[k] += e->term[k] / total;
        r.saturated_frac += (double)e->saturated_ticks / (e->clip->duration_ticks - 1) / total;
        if (e->peak_speed > r.peak_speed) r.peak_speed = e->peak_speed;
        if (e->peak_torque > r.peak_torque) r.peak_torque = e->peak_torque;
    }
    return r;
}

static double train_fitness(GrlEnv *e, const GrlPolicy *pol, uint64_t seed, int iter, int dr) {
    double f = 0.0;
    for (int s = 0; s < dr; s++) f += grl_env_rollout(e, pol, mix(seed + (uint64_t)iter * 1000003ull, (uint64_t)s + 1), NULL);
    return f / dr;
}

static int write_gband(const char *path, const GBClip *ref, uint32_t ticks, uint32_t chans, const float *data,
                       unsigned char content_hash[32]) {
    FILE *f = fopen(path, "wb");
    if (!f) return 0;
    size_t nbytes = (size_t)ticks * chans * 4;
    sha256((const uint8_t *)data, nbytes, content_hash);
    unsigned char h[84];
    memcpy(h, "GBND", 4);
    uint32_t v[4] = {1, ref->tick_rate, ticks, chans};
    for (int i = 0; i < 4; i++)
        for (int b = 0; b < 4; b++) h[4 + i * 4 + b] = (unsigned char)(v[i] >> (8 * b));
    memcpy(h + 20, ref->skeleton_hash, 32);
    memcpy(h + 52, content_hash, 32);
    int ok = fwrite(h, 1, 84, f) == 84 && fwrite(data, 1, nbytes, f) == nbytes; // little-endian hosts only
    fclose(f);
    return ok;
}

int main(int argc, char **argv) {
    const char *robot_path = NULL, *clip_name = NULL, *profile_path = NULL, *out = NULL;
    int iters = 30, pop = 32, elites = 6, dr = 2, harmonics = 3, eval_seeds = 8;
    uint64_t seed = 1;
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : NULL;
#define ARG(name) (strcmp(a, name) == 0 && v && (i++, 1))
        if (ARG("--robot")) robot_path = v;
        else if (ARG("--clip")) clip_name = v;
        else if (ARG("--profile")) profile_path = v;
        else if (ARG("--out")) out = v;
        else if (ARG("--iters")) iters = atoi(v);
        else if (ARG("--pop")) pop = atoi(v);
        else if (ARG("--elites")) elites = atoi(v);
        else if (ARG("--dr")) dr = atoi(v);
        else if (ARG("--harmonics")) harmonics = atoi(v);
        else if (ARG("--seed")) seed = strtoull(v, NULL, 10);
        else if (ARG("--eval-seeds")) eval_seeds = atoi(v);
        else { fprintf(stderr, "gbtrain: unknown or incomplete argument %s\n", a); return 1; }
#undef ARG
    }
    if (!robot_path || !clip_name || !out || pop < 2 || elites < 1 || elites > pop || dr < 1 || iters < 0) {
        fprintf(stderr, "usage: gbtrain --robot <x.grobot> --clip <name> --out <name> [--profile p.txt] [--iters n] [--pop n] "
                        "[--elites n] [--dr n] [--harmonics n] [--seed n] [--eval-seeds n]\n");
        return 1;
    }

    static GRobot robot;
    if (!grobot_init(robot_path, &robot)) { fprintf(stderr, "gbtrain: cannot load robot %s\n", robot_path); return 1; }
    char path[1024];
    GBClip clip;
    snprintf(path, sizeof path, "%s.gband", clip_name);
    if (!gb_init(path, &clip) || !gb_verify(&clip)) { fprintf(stderr, "gbtrain: cannot load/verify %s\n", path); return 1; }
    static char names[256][64];
    snprintf(path, sizeof path, "%s.gband.json", clip_name);
    int nch = grl_manifest_channels(path, names, 256);
    if (nch != (int)clip.num_channels) { fprintf(stderr, "gbtrain: manifest %s channels (%d) do not match the clip (%u)\n", path, nch, clip.num_channels); return 1; }
    GrlRewardProfile prof;
    grl_profile_default(&prof);
    if (profile_path && !grl_profile_load(profile_path, &prof)) return 1;
    static GrlEnv env;
    if (!grl_env_init(&env, &robot, &clip, names, nch, &prof, harmonics)) {
        fprintf(stderr, "gbtrain: clip %s animates none of %s's joints (<joint>.angle channels)\n", clip_name, robot.name);
        return 1;
    }
    char rid[65], shash[65], chash[65];
    hex32(env.reward_id, rid);
    hex32(robot.spec_hash, shash);
    hex32(clip.content_hash, chash);
    printf("gbtrain: robot %s, clip %s (%u ticks @ %u/s), reward id %.16s\n", robot.name, clip_name, clip.duration_ticks, clip.tick_rate, rid);
    printf("  servo %.1f Hz zeta %.2f, residual +/-%.3f rad, DR mass +/-%.0f%%, linear policy %d x %d\n",
           prof.servo_hz, prof.servo_zeta, prof.action_scale, 100 * prof.dr_mass_jitter, env.act_dim, env.obs_dim);

    int D = env.act_dim * env.obs_dim;
    static GrlPolicy mu, best, cand[256];
    if (pop > 256) pop = 256;
    memset(&mu, 0, sizeof mu);
    mu.obs_dim = env.obs_dim;
    mu.act_dim = env.act_dim;
    best = mu;
    static double sigma[GROBOT_MAX_JOINTS * GRL_MAX_OBS];
    for (int k = 0; k < D; k++) sigma[k] = 0.05;

    EvalResult base = evaluate(&env, NULL, seed, eval_seeds);
    printf("  baseline (servo only, no policy): frozen-eval return %.4f\n", base.ret);

    uint64_t rng = seed * 0x2545F4914F6CDD1Dull + 7;
    double best_fit = train_fitness(&env, &mu, seed, 0, dr);
    static double fit[256];
    int order[256];
    for (int it = 0; it < iters; it++) {
        for (int c = 0; c < pop; c++) {
            cand[c] = mu;
            if (c > 0) // candidate 0 re-evaluates the current mean (elitism on the same DR draw)
                for (int k = 0; k < D; k++) cand[c].w[k] = mu.w[k] + sigma[k] * grl_rng_normal(&rng);
            fit[c] = train_fitness(&env, &cand[c], seed, it + 1, dr);
            order[c] = c;
        }
        for (int a = 1; a < pop; a++) // insertion sort, descending by fitness (stable -> deterministic)
            for (int b = a; b > 0 && fit[order[b]] > fit[order[b - 1]]; b--) { int t = order[b]; order[b] = order[b - 1]; order[b - 1] = t; }
        for (int k = 0; k < D; k++) {
            double m = 0, v = 0;
            for (int e = 0; e < elites; e++) m += cand[order[e]].w[k] / elites;
            for (int e = 0; e < elites; e++) v += (cand[order[e]].w[k] - m) * (cand[order[e]].w[k] - m) / elites;
            mu.w[k] = m;
            sigma[k] = sqrt(v) + 0.02 * pow(0.9, it); // decaying exploration floor
        }
        if (fit[order[0]] > best_fit) { best_fit = fit[order[0]]; best = cand[order[0]]; }
        printf("  iter %3d  best %.4f  elite-mean %.4f\n", it + 1, fit[order[0]],
               (fit[order[0]] + fit[order[elites - 1]]) / 2);
        fflush(stdout);
    }
    // Final pick by TRAINING fitness only (mean policy vs best-seen), never by the frozen eval.
    double mu_fit = train_fitness(&env, &mu, seed, iters + 1, dr), best_refit = train_fitness(&env, &best, seed, iters + 1, dr);
    GrlPolicy *chosen = mu_fit >= best_refit ? &mu : &best;

    EvalResult tr = evaluate(&env, chosen, seed, eval_seeds);
    printf("\n  frozen eval (nominal + %d held-out DR seeds)    baseline    trained\n", eval_seeds);
    printf("  %-44s %9.4f  %9.4f\n", "return", base.ret, tr.ret);
    for (int k = 0; k < 6; k++) printf("  %-44s %9.4f  %9.4f\n", TERM_NAMES[k], base.term[k], tr.term[k]);
    printf("  %-44s %8.1f%%  %8.1f%%\n", "ticks with a joint at its effort limit", 100 * base.saturated_frac, 100 * tr.saturated_frac);
    printf("  %-44s %9.3f  %9.3f\n", "peak joint speed (rad/s)", base.peak_speed, tr.peak_speed);
    printf("  %-44s %9.2f  %9.2f\n", "peak joint torque (N*m)", base.peak_torque, tr.peak_torque);

    // --- artifacts
    unsigned char pol_hash[32];
    sha256((const uint8_t *)chosen->w, sizeof(double) * (size_t)D, pol_hash);
    snprintf(path, sizeof path, "%s.gpolicy", out);
    FILE *f = fopen(path, "wb");
    if (!f) { fprintf(stderr, "gbtrain: cannot write %s\n", path); return 1; }
    uint32_t hdr[5] = {1, (uint32_t)env.obs_dim, (uint32_t)env.act_dim, (uint32_t)harmonics, 0};
    fwrite("GPOL", 1, 4, f);
    fwrite(hdr, sizeof hdr[0], 5, f);
    fwrite(robot.spec_hash, 1, 32, f);
    fwrite(clip.content_hash, 1, 32, f);
    fwrite(env.reward_id, 1, 32, f);
    fwrite(chosen->w, sizeof(double), (size_t)D, f);
    fclose(f);

    int T = (int)clip.duration_ticks;
    float *angles = malloc(sizeof(float) * (size_t)T * env.act_dim);
    grl_env_rollout(&env, chosen, 0, angles);
    double roll_speed = env.peak_speed, roll_torque = env.peak_torque;
    unsigned char roll_hash[32];
    snprintf(path, sizeof path, "%s_rollout.gband", out);
    if (!write_gband(path, &clip, (uint32_t)T, (uint32_t)env.act_dim, angles, roll_hash)) { fprintf(stderr, "gbtrain: cannot write %s\n", path); return 1; }
    free(angles);
    char rhash[65], phash[65], skhash[65];
    hex32(roll_hash, rhash);
    hex32(pol_hash, phash);
    hex32(clip.skeleton_hash, skhash);
    snprintf(path, sizeof path, "%s_rollout.gband.json", out);
    f = fopen(path, "w");
    fprintf(f, "{\n  \"gband_version\": 1,\n  \"skeleton_hash\": \"%s\",\n  \"content_hash\": \"%s\",\n", skhash, rhash);
    fprintf(f, "  \"tick_rate\": %u,\n  \"duration_ticks\": %d,\n  \"channels\": [", clip.tick_rate, T);
    for (int j = 0; j < env.act_dim; j++) fprintf(f, "%s\"%s.angle\"", j ? ", " : "", robot.joints[j].name);
    fprintf(f, "],\n  \"authorship\": {\n    \"kind\": \"generative\",\n");
    fprintf(f, "    \"who\": \"gbtrain policy %.16s (reward %.16s) tracking clip %.16s on %s in grb physics, nominal masses\"\n  },\n", phash, rid, chash, robot.name);
    fprintf(f, "  \"intent_tags\": [\"policy-rollout\"],\n  \"loop_points\": {\n    \"start_tick\": 0,\n    \"end_tick\": %d\n  },\n", T);
    fprintf(f, "  \"safety\": {\n    \"max_joint_velocity\": %.6f,\n    \"max_joint_torque\": %.6f\n  }\n}\n", roll_speed, roll_torque);
    fclose(f);

    snprintf(path, sizeof path, "%s.gpolicy.json", out);
    f = fopen(path, "w");
    char proftxt[2048];
    grl_profile_format(&prof, proftxt, sizeof proftxt);
    fprintf(f, "{\n  \"gpolicy_version\": 1,\n  \"policy_hash\": \"%s\",\n  \"robot\": \"%s\",\n  \"robot_spec_hash\": \"%s\",\n", phash, robot.name, shash);
    fprintf(f, "  \"clip\": \"%s\",\n  \"clip_content_hash\": \"%s\",\n  \"reward_id\": \"%s\",\n", clip_name, chash, rid);
    fprintf(f, "  \"policy\": {\"kind\": \"linear\", \"obs_dim\": %d, \"act_dim\": %d, \"harmonics\": %d},\n", env.obs_dim, env.act_dim, harmonics);
    fprintf(f, "  \"trainer\": {\"method\": \"cross-entropy\", \"seed\": %llu, \"iters\": %d, \"pop\": %d, \"elites\": %d, \"dr_samples\": %d},\n",
            (unsigned long long)seed, iters, pop, elites, dr);
    fprintf(f, "  \"reward_profile\": \"");
    for (char *c = proftxt; *c; c++) fputs(*c == '\n' ? "\\n" : (char[2]){*c, 0}, f);
    fprintf(f, "\",\n  \"frozen_eval\": {\n    \"eval_seeds\": %d,\n", eval_seeds);
    for (int pass = 0; pass < 2; pass++) {
        EvalResult *r = pass ? &tr : &base;
        fprintf(f, "    \"%s\": {\"return\": %.6f", pass ? "trained" : "baseline", r->ret);
        for (int k = 0; k < 6; k++) fprintf(f, ", \"%s\": %.6f", TERM_NAMES[k], r->term[k]);
        fprintf(f, ", \"saturated_fraction\": %.6f, \"peak_speed\": %.6f, \"peak_torque\": %.6f}%s\n", r->saturated_frac, r->peak_speed, r->peak_torque, pass ? "" : ",");
    }
    fprintf(f, "  }\n}\n");
    fclose(f);
    printf("\n  wrote %s.gpolicy (+.json) and %s_rollout.gband (+.json), policy %.16s\n", out, out, phash);
    gb_free(&clip);
    return tr.ret >= base.ret ? 0 : 3;
}
