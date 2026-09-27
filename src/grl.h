// grl.h — the GOLDEN BAND reward compiler + physics training environment (HQ-SPEC-SIM-100 §4 and
// §8 build step 3, v0). Founder real-time, 2026-09-27: "and the rl animations pipeline".
//
// "The animator's clip is the spec; physics is the implementation; the policy is the compiled
// artifact" (SIM-100 §1). Concretely:
//
//   clip (.gband, authored)  +  reward profile (text, versioned)  +  robot (.grobot, datasheet)
//        -> grl_reward_id()  = sha256 over all three: the compiled reward function's identity
//        -> GrlEnv           = a grb world with the robot servoed to track the clip; each tick
//                              the policy adds a residual to the servo targets, and the env
//                              scores the physical result against the clip
//        -> gbtrain           = optimizes the policy (tools/gbtrain), writes the policy artifact
//                              and the physically-achieved motion back out as a new .gband
//
// Reward terms are the standard imitation-learning set (DeepMimic lineage), each in [0, 1] for
// the tracking terms and <= 0 for the regularizers:
//   pose   exp(-k_pose * sum_j (q_j - q_ref_j)^2)
//   vel    exp(-k_vel  * sum_j (qd_j - qd_ref_j)^2)
//   ee     exp(-k_ee   * |tcp - tcp_ref|^2)          (end effector -- "where charm lives")
//   energy -sum_j |tau_j * qd_j| / energy_scale       (mechanical power actually delivered)
//   smooth -sum_j (a_j - a_prev_j)^2                   (action smoothness)
//   limit  -(fraction of joints at their effort limit this tick)
// Domain randomization (v0): per-episode link-mass jitter, seeded, so the policy's robustness is
// purchased with sampled physics and the DR seed/profile travels with the artifact.
//
// Named v0 limits: fixed-base articulated robots only (no floating-base humanoid yet -- that
// needs contact-rich balance, SIM-100 §8 step 5); linear policy on hand-built features (the
// reward/env API is policy-agnostic; a neural policy is a drop-in for grl_policy_act); CPU,
// single-threaded rollouts (no GPU backbone -- SIM-100's Isaac/MJX adapter is not built).
#ifndef GOLDENBAND_GRL_H
#define GOLDENBAND_GRL_H

#include <stdint.h>
#include "gband.h"
#include "grb.h"
#include "grobot.h"

#define GRL_MAX_HARMONICS 8
#define GRL_MAX_OBS (2 * GROBOT_MAX_JOINTS + 2 * GRL_MAX_HARMONICS + 1)

typedef struct {
    double w_pose, w_vel, w_ee, w_energy, w_smooth, w_limit;
    double k_pose, k_vel, k_ee;
    double energy_scale;       // W
    double servo_hz;           // closed-loop bandwidth of every joint servo
    double servo_zeta;         // damping ratio
    double action_scale;       // rad: residual target offset = action_scale * clamp(a, -1, 1)
    double dr_mass_jitter;     // +/- fraction of each link mass, per episode (0 = off)
} GrlRewardProfile;

void grl_profile_default(GrlRewardProfile *p);
// Loads `key = value` lines (# comments) over the defaults. Unknown keys are an error (a typo
// must not silently fall back to a default). Returns 1 on success.
int grl_profile_load(const char *path, GrlRewardProfile *p);
// Canonical text form (fixed key order, %.17g) -- what the reward id hashes.
int grl_profile_format(const GrlRewardProfile *p, char *buf, int cap);

// Reads the "channels" array out of a .gband.json manifest (tiny scanner, not a JSON parser --
// the manifest is gbtool-generated). Returns the channel count, or -1.
int grl_manifest_channels(const char *manifest_path, char names[][64], int max);

typedef struct {
    const GRobot *robot;
    const GBClip *clip;
    int chan[GROBOT_MAX_JOINTS];     // clip channel index of each joint's ".angle", -1 = hold 0
    GrlRewardProfile prof;
    unsigned char reward_id[32];
    int harmonics;
    int obs_dim, act_dim;
    GrbWorld world;
    GRobotInstance inst;
    double nominal_mass[GROBOT_MAX_JOINTS];
    uint32_t tick;
    double prev_action[GROBOT_MAX_JOINTS];
    // Per-episode accumulators (means over ticks after grl_env_rollout).
    double term[6];                  // pose, vel, ee, energy, smooth, limit
    double ret;                      // mean total reward
    uint32_t saturated_ticks;        // ticks where any joint hit its effort limit
    double peak_speed, peak_torque;  // over the episode, any joint
} GrlEnv;

// Binds a robot + clip + profile. channel_names are the clip's manifest channels. Returns 1 on
// success (at least one robot joint is animated by the clip).
int grl_env_init(GrlEnv *e, const GRobot *robot, const GBClip *clip, char channel_names[][64], int num_channels,
                 const GrlRewardProfile *prof, int harmonics);

// Reference joint state at a (clamped) tick, from the clip. qd by central difference.
void grl_reference(const GrlEnv *e, int tick, double *q, double *qd);

// Linear policy: a = clamp(W * obs, -1, 1). W is act_dim x obs_dim, row-major.
typedef struct {
    int obs_dim, act_dim;
    double w[GROBOT_MAX_JOINTS * GRL_MAX_OBS];
} GrlPolicy;

// Runs one full episode (clip length) with the policy (NULL = zero residual, i.e. the bare servo
// baseline). dr_seed selects the domain-randomization sample (0 = nominal physics). If
// out_angles is non-NULL it receives duration_ticks * act_dim achieved joint angles (row-major).
// Returns the mean per-tick reward; per-term means are left in e->term.
double grl_env_rollout(GrlEnv *e, const GrlPolicy *pol, uint64_t dr_seed, float *out_angles);

// Deterministic PRNG shared by the env's DR and the trainer (splitmix64 -> uniform/normal).
uint64_t grl_rng_next(uint64_t *s);
double grl_rng_uniform(uint64_t *s);
double grl_rng_normal(uint64_t *s);

#endif // GOLDENBAND_GRL_H
