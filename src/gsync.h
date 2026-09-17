// gsync.h — real multi-actor frame synchronization (S144-XX/SHANKPIT AI_SCRIPTED_ANIMATION
// follow-up, founder real-time, Half-Life scripted_sequence breakdown: "Multi-Character
// Interactions: Frame Synchronization... Identical Naming... The 'Wait' State... Simultaneous
// Ignition... The exact millisecond all targeted actors reached their assigned points, the
// GoldSrc engine locked their animation timelines together.")
//
// GOLDENBAND has no concept of "an actor," "a marker," or "movement" -- those are the consuming
// engine's own domain (SHANKPIT's story_ai.c AI_MODE_SCRIPTED, e.g., which already owns real
// movement-hook-to-marker logic). This module is the one real, minimal, engine-agnostic piece
// underneath that: a named BARRIER. A caller registers N expected members into a group by name,
// each member reports "I've arrived" independently (at its own real time, since pathfinding
// speeds vary -- matching the founder's own "because pathfinding speeds could vary, the engine
// forced early-arriving characters to wait" framing), and gsync_check_ignition(group) returns
// true for exactly one real tick -- the moment every registered member has arrived -- so the
// caller can reset every one of ITS OWN GSeqPlayer instances to elapsed=0 at that same instant,
// achieving real simultaneous ignition without this module ever touching a GSeqPlayer itself.
//
// What this deliberately does NOT cover, named honestly rather than silently assumed: "paired
// animation with root alignment" (the founder's own third scripted_sequence point) is an
// AUTHORING-time concern -- animators authoring two clips together so their root motion lines up
// -- not a runtime one; this module trusts that whatever clips a caller plays once ignition fires
// were already authored to align, the same real "GOLDENBAND assets know nothing about the
// consuming engine" boundary this whole repo already holds itself to.
#ifndef GOLDENBAND_GSYNC_H
#define GOLDENBAND_GSYNC_H

#define GSYNC_MAX_GROUPS 16
#define GSYNC_GROUP_NAME_LEN 48
#define GSYNC_MAX_MEMBERS 8

typedef struct {
    char name[GSYNC_GROUP_NAME_LEN];
    int active;
    int expected_count;
    int arrived_count;
    int arrived[GSYNC_MAX_MEMBERS]; // 1 if member i has reported arrival, else 0
    int ignited;                    // internal one-shot latch, see gsync_check_ignition
} GSyncGroup;

typedef struct {
    GSyncGroup groups[GSYNC_MAX_GROUPS];
    int group_count;
} GSyncRegistry;

void gsync_registry_init(GSyncRegistry *r);

// gsync_begin_group creates a new named group expecting member_count real arrivals (2 <=
// member_count <= GSYNC_MAX_MEMBERS -- a "sync group" of 1 is just an ordinary animation, not a
// real synchronization case). If name already exists, RE-ARMS that same group (clears arrived
// state and the ignited latch) rather than creating a duplicate -- matches a scripted_sequence
// being triggered again. Returns the group's real index (stable until re-armed or the registry
// is reset), or -1 if the registry is full or member_count is out of range.
int gsync_begin_group(GSyncRegistry *r, const char *name, int member_count);

// gsync_mark_arrived records that member_index (0-based, < the group's own expected_count) has
// reached its own marker. No-op on an invalid group_index/member_index, an inactive group, or a
// member already marked (idempotent -- a caller re-confirming arrival every tick is safe).
void gsync_mark_arrived(GSyncRegistry *r, int group_index, int member_index);

// gsync_check_ignition is the real per-tick poll: returns 1 exactly once, the real tick every
// registered member has arrived, and 0 every other call for that same group (including every
// call after that one) until gsync_begin_group re-arms it by name. This "returns true once, not
// a persistent flag the caller has to remember to clear" shape matches this repo's own
// gseq_player_advance precedent for real one-shot state transitions.
int gsync_check_ignition(GSyncRegistry *r, int group_index);

#endif // GOLDENBAND_GSYNC_H
