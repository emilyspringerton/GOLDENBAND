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

// gpose_compute_joint_world runs real forward kinematics over skel's own joint hierarchy (walked
// in index order -- GSKEL_FORMAT.md guarantees parent_index < child index, so a single forward
// pass is correct, no recursion/sorting needed) and produces, for every joint, its real WORLD
// transform (parent_world * local, local = T(pose_trans)*R(pose_rot)) -- not yet multiplied
// against inverse_bind, unlike a skin matrix. Exposed as its own real primitive (S144-XX/
// GOLDENBAND animation-composition follow-up, founder real-time Half-Life scripted_sequence
// breakdown: "bone controllers... manipulated the axes of specific joints") because anything that
// needs a joint's real world position/orientation without wanting a skin matrix -- gpose_look_at
// is the first real caller -- would otherwise have to duplicate this exact FK loop.
//
// pose_rot/pose_trans are skel->joint_count quaternions (x,y,z,w) / translations -- the exact
// shape gseq_player_sample_pose already produces (this module has no opinion on where a pose
// comes from: a GSeqPlayer, a single gb_sample call, or the skeleton's own rest pose passed
// straight through).
//
// out_joint_world must be caller-owned, at least skel->joint_count entries of 16 floats each.
void gpose_compute_joint_world(const GSkel *skel, const float *pose_rot, const float *pose_trans,
                                float out_joint_world[][16]);

// gpose_compute_skin_matrices is gpose_compute_joint_world plus one more real step per joint
// (joint_world * that joint's own inverse_bind) -- the real skin matrix a vertex bound to that
// joint should be transformed by. See gpose_compute_joint_world's own doc comment for the shared
// pose_rot/pose_trans shape.
//
// out_skin_matrices must be caller-owned, at least skel->joint_count entries of 16 floats each.
void gpose_compute_skin_matrices(const GSkel *skel, const float *pose_rot, const float *pose_trans,
                                  float out_skin_matrices[][16]);

// gpose_look_at -- a real bone-controller override for exactly ONE joint (founder real-time,
// Half-Life scripted_sequence breakdown: "Even while caught in a locked walking or idle script, a
// scientist's head could dynamically turn up to 30 degrees to track Gordon Freeman"). Computes
// the real local rotation that would rotate local_axis (a unit vector in the joint's own local
// space -- e.g. (0,0,1) if that's this rig's own "forward"; callers must know their own rig's
// convention, this function makes no assumption about which axis is "forward") to point at
// target_world, then nlerps between pose_rot[joint_index]'s own EXISTING value and that full
// look-at rotation -- the full amount if the angle between them is <= max_angle_deg, otherwise
// clamped so the joint turns at most max_angle_deg from wherever it already was. Writes the
// blended result directly into pose_rot[joint_index*4 .. +4).
//
// parent_world must be the already-computed gpose_compute_joint_world entry for this joint's own
// PARENT (an identity matrix for a root joint) -- this function does not recompute the FK chain
// itself, and does not touch any other joint's pose_rot; call gpose_compute_joint_world/
// gpose_compute_skin_matrices AFTER this to get correct final world positions for this joint and
// its own children.
//
// Real, deliberate approximation, same reason this whole repo commits to nlerp over slerp
// everywhere else (GBAND_FORMAT.md's own "Quaternion channels" section): the max_angle_deg clamp
// is a nlerp-space linear blend factor, not a true constant-angular-velocity slerp step, so the
// actual resulting angle is only approximately max_angle_deg for large clamps -- exact for small
// ones, which covers the real "turn up to 30 degrees" use case this was built for.
void gpose_look_at(const GSkel *skel, float *pose_rot, const float *pose_trans, int joint_index,
                    const float local_axis[3], const float target_world[3],
                    const float parent_world[16], float max_angle_deg);

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
