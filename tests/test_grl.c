// test_grl.c — reward compiler / env invariants (the trainer's own improvement + determinism is
// checked end to end by scripts/build_and_test.sh running tools/gbtrain).
// Usage: test_grl <ur5e.grobot> <clip name>
#include "../src/grl.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;
#define CHECK(cond, ...) do { \
    printf("%s: ", (cond) ? "PASS" : "FAIL"); printf(__VA_ARGS__); printf("\n"); \
    if (!(cond)) failures++; \
} while (0)

int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: %s <robot.grobot> <clip>\n", argv[0]); return 2; }
    static GRobot r;
    GBClip clip;
    char path[512];
    static char names[64][64];
    CHECK(grobot_init(argv[1], &r), "robot loads");
    snprintf(path, sizeof path, "%s.gband", argv[2]);
    CHECK(gb_init(path, &clip), "clip loads");
    snprintf(path, sizeof path, "%s.gband.json", argv[2]);
    int n = grl_manifest_channels(path, names, 64);
    CHECK(n == 6 && strcmp(names[0], "shoulder_pan_joint.angle") == 0, "manifest channels read (%d, first %s)", n, n > 0 ? names[0] : "-");

    GrlRewardProfile p, p2;
    grl_profile_default(&p);
    p2 = p;
    p2.k_ee *= 2;
    static GrlEnv e, e2;
    CHECK(grl_env_init(&e, &r, &clip, names, n, &p, 3), "env binds robot + clip");
    CHECK(e.obs_dim == 2 * 6 + 2 * 3 + 1 && e.act_dim == 6, "obs %d / act %d", e.obs_dim, e.act_dim);
    grl_env_init(&e2, &r, &clip, names, n, &p2, 3);
    CHECK(memcmp(e.reward_id, e2.reward_id, 32) != 0, "reward id changes when the reward profile changes");
    grl_env_init(&e2, &r, &clip, names, n, &p, 2);
    CHECK(memcmp(e.reward_id, e2.reward_id, 32) != 0, "reward id changes when the policy feature set changes");

    double a = grl_env_rollout(&e, NULL, 0, NULL);
    double terms[6];
    memcpy(terms, e.term, sizeof terms);
    double b = grl_env_rollout(&e, NULL, 0, NULL);
    CHECK(a == b && memcmp(terms, e.term, sizeof terms) == 0, "nominal rollout is bit-for-bit repeatable (return %.6f)", a);
    for (int k = 0; k < 3; k++) CHECK(terms[k] > 0 && terms[k] <= 1, "tracking term %d in (0,1]: %.4f", k, terms[k]);
    double c = grl_env_rollout(&e, NULL, 12345, NULL);
    CHECK(c != a, "a domain-randomized rollout differs from nominal (%.6f vs %.6f)", c, a);

    // A servo tuned stiffer tracks better: the reward really measures physical tracking.
    GrlRewardProfile stiff = p;
    stiff.servo_hz = 20.0;
    grl_env_init(&e2, &r, &clip, names, n, &stiff, 3);
    double s = grl_env_rollout(&e2, NULL, 0, NULL);
    CHECK(e2.term[0] > terms[0] && e2.term[2] > terms[2], "20 Hz servo tracks better than 5 Hz (pose %.3f > %.3f, ee %.3f > %.3f, return %.3f)",
          e2.term[0], terms[0], e2.term[2], terms[2], s);

    FILE *f = fopen("/tmp/gb_test_grl_bad.txt", "w");
    fprintf(f, "w_pose = 1\nw_posee = 2\n");
    fclose(f);
    GrlRewardProfile q = p;
    CHECK(!grl_profile_load("/tmp/gb_test_grl_bad.txt", &q), "a misspelled reward-profile key is rejected, not ignored");

    gb_free(&clip);
    printf("%s: %d failure(s)\n", failures ? "FAILED" : "OK", failures);
    return failures ? 1 : 0;
}
