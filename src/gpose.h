// gpose.h — S144-XX: general forward kinematics + mesh skinning for an arbitrary-joint-count
// GSkel/GMesh pair (founder real-time, 2026-09-17: "we want animations in game and we want
// nicer models so we want to at least build the affordances to start building the animations
// into the games").
//
// Real, found-live gap this closes: SHANKPIT's existing gband_mesh_rig.c hardcodes a 5-joint
// armature with manually-indexed animation channels (HIPS_RX/Spine/Head/L_Arm/R_Arm) -- real and
// working for Tyler's own hand-authored rig, but it can't be pointed at any other skeleton. A
// real uploaded asset (the founder's own Mannequin_F.glb + a paired UAL2_Standard_RM mocap clip,
// both a real 65-joint industry-standard rig) needs a general N-joint FK+skinning path instead.
//
// Deliberately still "pure data transformation" in the same sense gband.c/gskel.c/gmesh.c/
// gseq.c already hold themselves to -- self-contained column-major float[16] matrix math, no
// engine Mat4 type, no GL calls, no rendering. gskel.h's own header comment says forward
// kinematics is "the consuming engine's job" because GOLDENBAND doesn't want to impose an
// engine-specific matrix TYPE; this module honors that by using raw float[16] (column-major,
// matching GSkelJoint.inverse_bind's own documented layout) rather than any one engine's Mat4
// struct, so it's still a real, portable, engine-agnostic reference implementation -- consuming
// engines (SHANKPIT, PAPERCRAFT, ...) vendor a copy of gpose.c the same way SHANKPIT's own
// gband_mesh_rig.c was already ported verbatim from REDGARDEN, converting the output float[16]
// values into their own Mat4 type only at the very last step (a single reinterpret, since the
// layouts already match).
#ifndef GOLDENBAND_GPOSE_H
#define GOLDENBAND_GPOSE_H

#include <stdint.h>
#include "gskel.h"
#include "gmesh.h"

// gpose_compute_skin_matrices runs real forward kinematics over skel's own joint hierarchy
// (walked in index order -- GSKEL_FORMAT.md guarantees parent_index < child index, so a single
// forward pass is correct, no recursion/sorting needed) and produces, for every joint, the real
// skin matrix a vertex bound to that joint should be transformed by: joint_world * joint's own
// inverse_bind.
//
// pose_rot/pose_trans are skel->joint_count quaternions (x,y,z,w) / translations -- the exact
// shape gseq_player_sample_pose already produces (this module has no opinion on where a pose
// comes from: a GSeqPlayer, a single gb_sample call, or the skeleton's own rest pose passed
// straight through).
//
// out_skin_matrices must be caller-owned, at least skel->joint_count entries of 16 floats each.
void gpose_compute_skin_matrices(const GSkel *skel, const float *pose_rot, const float *pose_trans,
                                  float out_skin_matrices[][16]);

// gpose_skin_mesh applies real 4-bone-weight linear blend skinning, flattening mesh's own
// indexed vertex/index buffers into a non-indexed pos+normal triangle list -- 6 floats per
// vertex (position.xyz, normal.xyz), exactly the layout SHANKPIT's own gband_mesh_rig.h
// draw_skinned callback already expects, so a consuming engine's render bridge code changes only
// in how it computes skin_matrices, not in how it consumes the output.
//
// out_verts6 must be caller-owned, at least mesh->index_count * 6 floats. Returns the real
// vertex count written (== mesh->index_count).
uint32_t gpose_skin_mesh(const GMesh *mesh, const float skin_matrices[][16], float *out_verts6);

#endif // GOLDENBAND_GPOSE_H
