// gsync.c — see gsync.h. Plain, self-contained bookkeeping, no external dependency at all (not
// even gband.h/gskel.h) -- this module doesn't know what an animation clip or a skeleton even
// is, only that some set of named members needs to cross a barrier together.

#include "gsync.h"
#include <string.h>

void gsync_registry_init(GSyncRegistry *r) {
    memset(r, 0, sizeof(*r));
}

int gsync_begin_group(GSyncRegistry *r, const char *name, int member_count) {
    if (member_count < 2 || member_count > GSYNC_MAX_MEMBERS) return -1;

    for (int i = 0; i < r->group_count; i++) {
        if (strncmp(r->groups[i].name, name, GSYNC_GROUP_NAME_LEN) == 0) {
            GSyncGroup *g = &r->groups[i];
            g->active = 1;
            g->expected_count = member_count;
            g->arrived_count = 0;
            g->ignited = 0;
            memset(g->arrived, 0, sizeof(g->arrived));
            return i;
        }
    }

    if (r->group_count >= GSYNC_MAX_GROUPS) return -1;
    int idx = r->group_count++;
    GSyncGroup *g = &r->groups[idx];
    memset(g, 0, sizeof(*g));
    strncpy(g->name, name, GSYNC_GROUP_NAME_LEN - 1);
    g->active = 1;
    g->expected_count = member_count;
    return idx;
}

void gsync_mark_arrived(GSyncRegistry *r, int group_index, int member_index) {
    if (group_index < 0 || group_index >= r->group_count) return;
    GSyncGroup *g = &r->groups[group_index];
    if (!g->active) return;
    if (member_index < 0 || member_index >= g->expected_count) return;
    if (g->arrived[member_index]) return; // already marked -- idempotent
    g->arrived[member_index] = 1;
    g->arrived_count++;
}

int gsync_check_ignition(GSyncRegistry *r, int group_index) {
    if (group_index < 0 || group_index >= r->group_count) return 0;
    GSyncGroup *g = &r->groups[group_index];
    if (!g->active || g->ignited) return 0;
    if (g->arrived_count < g->expected_count) return 0;
    g->ignited = 1; // latch -- every later call returns 0 until gsync_begin_group re-arms it
    return 1;
}
