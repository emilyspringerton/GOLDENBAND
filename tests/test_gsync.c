// test_gsync.c — real multi-actor frame synchronization barrier (gsync.c).
#include "../src/gsync.h"
#include <stdio.h>

static int failures = 0;
#define CHECK(cond, label) do { \
    if (cond) { printf("PASS: %s\n", label); } \
    else { printf("FAIL: %s\n", label); failures++; } \
} while (0)

static void test_ignition_fires_only_once_all_arrived(void) {
    GSyncRegistry r;
    gsync_registry_init(&r);
    int g = gsync_begin_group(&r, "scientist_grab", 3);
    CHECK(g >= 0, "begin_group returns a valid index for 3 members");

    CHECK(gsync_check_ignition(&r, g) == 0, "no ignition with zero members arrived");

    gsync_mark_arrived(&r, g, 0);
    CHECK(gsync_check_ignition(&r, g) == 0, "no ignition with 1/3 arrived");

    gsync_mark_arrived(&r, g, 1);
    CHECK(gsync_check_ignition(&r, g) == 0, "no ignition with 2/3 arrived");

    gsync_mark_arrived(&r, g, 2);
    CHECK(gsync_check_ignition(&r, g) == 1, "ignition fires the real tick the 3rd member arrives");
    CHECK(gsync_check_ignition(&r, g) == 0, "ignition is a real one-shot latch -- does not re-fire next tick");
}

static void test_duplicate_arrival_is_idempotent(void) {
    GSyncRegistry r;
    gsync_registry_init(&r);
    int g = gsync_begin_group(&r, "pair", 2);
    gsync_mark_arrived(&r, g, 0);
    gsync_mark_arrived(&r, g, 0); // re-report the same member -- must not double-count
    CHECK(gsync_check_ignition(&r, g) == 0, "ignition still requires the OTHER member, a duplicate mark doesn't fake it");
    gsync_mark_arrived(&r, g, 1);
    CHECK(gsync_check_ignition(&r, g) == 1, "ignition fires once the real second distinct member arrives");
}

static void test_early_arrivals_really_wait(void) {
    // Matches the founder's own "because pathfinding speeds could vary, the engine forced
    // early-arriving characters to wait" framing directly: member 0 arrives, then many ticks of
    // nothing happening, THEN member 1 arrives -- ignition must fire exactly at that real moment,
    // not before.
    GSyncRegistry r;
    gsync_registry_init(&r);
    int g = gsync_begin_group(&r, "wait_test", 2);
    gsync_mark_arrived(&r, g, 0);
    for (int tick = 0; tick < 50; tick++) {
        CHECK(gsync_check_ignition(&r, g) == 0, "waiting member does not ignite alone");
    }
    gsync_mark_arrived(&r, g, 1);
    CHECK(gsync_check_ignition(&r, g) == 1, "ignition fires the instant the late member finally arrives");
}

static void test_rearming_by_name_resets_state(void) {
    GSyncRegistry r;
    gsync_registry_init(&r);
    int g1 = gsync_begin_group(&r, "reused", 2);
    gsync_mark_arrived(&r, g1, 0);
    gsync_mark_arrived(&r, g1, 1);
    CHECK(gsync_check_ignition(&r, g1) == 1, "first trigger ignites normally");

    int g2 = gsync_begin_group(&r, "reused", 2); // same name -- a scripted_sequence re-triggered
    CHECK(g1 == g2, "re-arming a group by its own real name reuses the same slot, not a new one");
    CHECK(gsync_check_ignition(&r, g2) == 0, "a freshly re-armed group is NOT already ignited");
    gsync_mark_arrived(&r, g2, 0);
    gsync_mark_arrived(&r, g2, 1);
    CHECK(gsync_check_ignition(&r, g2) == 1, "the re-armed group ignites again on its own second real arrival pair");
}

static void test_rejects_degenerate_member_counts(void) {
    GSyncRegistry r;
    gsync_registry_init(&r);
    CHECK(gsync_begin_group(&r, "solo", 1) == -1, "a 1-member 'sync group' is rejected -- not a real synchronization case");
    CHECK(gsync_begin_group(&r, "toomany", GSYNC_MAX_MEMBERS + 1) == -1, "a group over GSYNC_MAX_MEMBERS is rejected");
}

int main(void) {
    test_ignition_fires_only_once_all_arrived();
    test_duplicate_arrival_is_idempotent();
    test_early_arrivals_really_wait();
    test_rearming_by_name_resets_state();
    test_rejects_degenerate_member_counts();
    printf("%s: %d failure(s)\n", failures == 0 ? "OK" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
