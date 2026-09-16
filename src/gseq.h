// gseq.h — S144-XX: real animation STITCHING for GOLDEN BAND (founder real-time: "make sure we
// can stitch animations together like James Bond walk turn raise gun shoot").
//
// gband.c/gskel.c/gmesh.c are deliberately tiny, channel-name-blind samplers -- gb_sample/
// gb_blend only ever know "N floats per tick," never which joint a float belongs to (channel
// names live in the .gband.json manifest, a tooling-facing concern, per GBAND_FORMAT.md's own
// split). Chaining several DIFFERENT clips together into one continuous performance (walk, then
// turn, then raise a gun, then shoot) genuinely needs channel-name awareness -- each clip may
// animate a different subset of joints, and matching them up by array index alone would be
// silently wrong the moment two clips don't share an identical channel ordering. This module is
// the one real, minimal place that bridges channel names (from the manifest) to a channel-blind
// GBClip -- still pure data in, pure data out (per-joint quaternion + translation arrays), never
// touching rendering, FK, or engine state -- same "pure data transformation" scope gband.c's own
// sampler already holds itself to.
//
// A GSeq is a real, ordered sequence of clips with a real crossfade (nlerp, matching gb_blend's
// own real, documented nlerp-not-slerp choice for the same reasons -- see GBAND_FORMAT.md's own
// "Quaternion channels" section) between consecutive steps. Missing channels (a clip that
// doesn't touch a given joint) fall back to the skeleton's own real rest pose, the exact same
// documented convention GSKEL_FORMAT.md already establishes for a single clip.
//
// What v0 does NOT cover (real, deliberate, named -- not silently assumed): a single, fixed
// crossfade duration for every transition in a sequence (per-transition override is real,
// straightforward future work); no authored `.gseq` asset format or NOCK UI yet -- a caller
// builds a GSeq by hand in code today (see tests/test_gseq.c for the real, working shape); time-
// based advancement only (gseq_player_advance takes real seconds, not ticks, since different
// clips in one sequence may have different tick_rate values -- this is deliberate, not an
// oversight).
#ifndef GOLDENBAND_GSEQ_H
#define GOLDENBAND_GSEQ_H

#include <stdint.h>
#include "gband.h"
#include "gskel.h"

#define GSEQ_MAX_CHANNELS 256      // generous real cap -- a busy multi-joint skeleton's own full channel count
#define GSEQ_CHANNEL_NAME_LEN 48   // GSKEL_NAME_LEN (32) + room for ".qw"/".tz" etc.
#define GSEQ_MAX_STEPS 16

// GSeqClip -- one loaded .gband clip PLUS its own real channel names (read from the manifest's
// "channels" array), the one real piece gband.c's own binary format doesn't carry. Freed with
// gseq_clip_free.
typedef struct {
    GBClip clip;
    char channel_names[GSEQ_MAX_CHANNELS][GSEQ_CHANNEL_NAME_LEN];
    uint32_t channel_count; // == clip.num_channels; kept alongside for a real, explicit bound
} GSeqClip;

// gseq_clip_load loads gband_path (the binary) and reads channel_names out of
// manifest_json_path's own real "channels" array (a small, real, bounded scanner -- not a
// general JSON parser, same "smallest real thing scoped to one known shape" discipline
// SHANKPIT/packages/world/level_boxes.h's own scanner already established). Returns 1 on
// success, 0 on any failure (missing/malformed binary, missing/malformed manifest, more channels
// than GSEQ_MAX_CHANNELS).
int gseq_clip_load(const char *gband_path, const char *manifest_json_path, GSeqClip *out);
void gseq_clip_free(GSeqClip *c);

typedef struct {
    int clip_index;           // index into the caller-owned GSeqClip[] "library" array
    float duration_seconds;   // <= 0 means "use the clip's own real duration_ticks/tick_rate"
} GSeqStep;

typedef struct {
    GSeqStep steps[GSEQ_MAX_STEPS];
    int step_count;
    float blend_seconds; // real crossfade duration between EVERY consecutive pair of steps (v0 --
                          // one value for the whole sequence, see this file's own header note)
    int loop;             // 1 = loop back to step 0 after the last step; 0 = hold the last step's
                            // own final pose forever once reached
} GSeq;

typedef struct {
    const GSeq *seq;
    const GSeqClip *clips; // caller-owned "library," indexed by each GSeqStep's own clip_index
    int current_step;
    float elapsed_in_step;   // real seconds since the current step started
    int blending;             // 1 while crossfading OUT of blend_from_step INTO current_step
    int blend_from_step;
    float blend_elapsed;      // real seconds since the crossfade started
} GSeqPlayer;

void gseq_player_init(GSeqPlayer *p, const GSeq *seq, const GSeqClip *clips);

// gseq_player_advance moves real time forward. Crosses into the next step (or loops, or holds
// the final pose) exactly once real elapsed_in_step reaches the current step's own real
// duration, starting a real crossfade at that same moment.
void gseq_player_advance(GSeqPlayer *p, float dt_seconds);

// gseq_player_sample_pose resolves the player's CURRENT (possibly mid-crossfade) pose for every
// joint in skel: out_rot gets skel->joint_count real quaternions (x,y,z,w, 4 floats each),
// out_trans gets skel->joint_count real translations (3 floats each) -- caller-owned, sized by
// the caller. A joint with no matching channel in the active clip(s) falls back to that joint's
// own real skeleton rest pose (skel->joints[j].rest_rotation/rest_translation), the same
// documented convention GSKEL_FORMAT.md already establishes for a single clip.
void gseq_player_sample_pose(const GSeqPlayer *p, const GSkel *skel, float *out_rot, float *out_trans);

#endif // GOLDENBAND_GSEQ_H
