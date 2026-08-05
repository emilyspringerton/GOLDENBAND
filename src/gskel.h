// gskel.h — S144-07: the GOLDEN BAND skeleton asset loader.
//
// Same philosophy as gband.h: no engine dependency, no JSON, a bounded
// parser. Pure data -- forward kinematics is the consuming engine's job
// (it needs a real matrix type this repo deliberately doesn't have, see
// format/GMESH_FORMAT.md's own note on this). See format/GSKEL_FORMAT.md
// for the full binary layout.
#ifndef GOLDENBAND_GSKEL_H
#define GOLDENBAND_GSKEL_H

#include <stdint.h>

#define GSKEL_NAME_LEN 32
// Fixed cap, not a dynamic array: every consumer of a GSkel wants
// stack-allocatable, copyable-by-value poses (an array of joint_count world
// matrices) without a matching alloc/free -- 64 joints is comfortably above
// any skeleton this v0 pass anticipates (S144-06's own rig uses 5).
#define GSKEL_MAX_JOINTS 64

typedef struct {
    char name[GSKEL_NAME_LEN];       // null-terminated if shorter than GSKEL_NAME_LEN
    int32_t parent_index;            // -1 for root; always < this joint's own index
    float rest_translation[3];
    float rest_rotation[4];          // quaternion, x/y/z/w
    float inverse_bind[16];          // column-major, matches the consumer's own Mat4 layout
} GSkelJoint;

typedef struct {
    uint32_t version;
    uint32_t joint_count;
    GSkelJoint joints[GSKEL_MAX_JOINTS];
} GSkel;

// gskel_init loads and structurally validates a .gskel file. Returns 1 on
// success, 0 on any failure (bad magic, truncated file, joint_count over
// GSKEL_MAX_JOINTS). No heap allocation -- GSkel is safe to stack-allocate.
int gskel_init(const char *path, GSkel *skel);

// gskel_find_joint returns the index of the joint named `name` (exact
// string match), or -1 if no joint has that name.
int gskel_find_joint(const GSkel *skel, const char *name);

#endif // GOLDENBAND_GSKEL_H
