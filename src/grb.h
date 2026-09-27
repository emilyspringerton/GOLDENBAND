// grb.h — GOLDEN BAND rigid body physics (founder real-time, 2026-09-27: "upgrade shankpit and
// nock to formal rigid body physics we need to get goldenband rigged up with real robot data from
// industrial data sheets" + "and the rl animations pipeline").
//
// What "formal" means here, concretely -- this replaces the point-mass Verlet/PBD spike
// (SHANKPIT/tools/ragdoll_spike) whose own conclusive ceiling was "no concept of orientation":
//
//   - Every body is a real 6-DOF rigid body: position + unit-quaternion orientation, linear +
//     angular velocity, mass, and a real 3x3 inertia tensor (stored diagonalized in the body's
//     own principal frame -- tooling diagonalizes a full datasheet/URDF tensor at compile time).
//   - Angular motion integrates the full Euler equation, gyroscopic term included:
//     I*dw/dt = tau - w x (I*w). Torque-free angular momentum is conserved (tested).
//   - Joints are XPBD constraints (Müller, Macklin, Chentanez, Jeschke, Kim, "Detailed Rigid Body
//     Simulation with Extended Position Based Dynamics", SCA 2020): positional attachment +
//     rotational alignment, solved with generalized inverse masses, small substeps, one iteration
//     per substep (the paper's own recommended scheme). Hinge (with angle limits), ball (with
//     swing cone + twist limits -- the ragdoll NORTHSTAR's own "v1: real per-bone orientation
//     state + twist limits"), and fixed joints.
//   - Actuators are honest, effort-limited torque sources, not position teleports. A POSITION
//     servo is a PD law evaluated implicitly at SUBSTEP rate (64 Hz x 16 substeps = 1024 Hz, the
//     same order as a real industrial servo loop): the spring is an XPBD angular drive with
//     compliance 1/kp, the damper a bounded velocity impulse, and the torque both imply is
//     clamped to the joint's datasheet effort limit; the datasheet speed limit is enforced the
//     same way. TORQUE mode applies a commanded torque (clamped) directly. The torque actually
//     delivered is recorded per joint -- that number is what the reward compiler's energy term
//     and the feasibility pass read. (Implicit, because an explicit PD torque on one link of a
//     constraint-coupled chain goes unstable at real servo gains -- measured on the UR5e.)
//   - Contacts: sphere/capsule/box against static ground planes, with XPBD static friction and
//     velocity-level dynamic friction + restitution.
//
// Deliberate, named v0 limits (not silently missing): no body-vs-body collision (ground planes
// only -- ragdolls rely on joint limits, robot arms on joint limits), no broadphase, no sleeping,
// no continuous collision detection. Determinism: double math, fixed iteration order, no heap,
// no threads -- same inputs + same build = bit-identical state (tested with memcmp).
//
// Conventions: right-handed; the world "up" is whatever the caller's gravity vector says it is
// (SHANKPIT is y-up, URDF robots are z-up -- grb has no opinion). Quaternions are x,y,z,w. Joint
// frames: the joint's own local +Z is the hinge axis / ball twist axis (the URDF convention).
#ifndef GOLDENBAND_GRB_H
#define GOLDENBAND_GRB_H

#include <stdint.h>

#define GRB_MAX_BODIES 128
#define GRB_MAX_JOINTS 128
#define GRB_MAX_PLANES 4
#define GRB_MAX_CONTACTS 1024

typedef enum { GRB_SHAPE_NONE = 0, GRB_SHAPE_SPHERE, GRB_SHAPE_CAPSULE, GRB_SHAPE_BOX } GrbShapeType;

typedef struct {
    // Pose of the body's principal/center-of-mass frame in world space.
    double pos[3];
    double rot[4];
    double vel[3];
    double omega[3];          // world-space angular velocity, rad/s
    double inv_mass;          // 0 = static/kinematic (immovable)
    double inv_inertia[3];    // principal-frame diagonal of I^-1 (0 on an axis = locked)
    // Collision shape, centered on the body origin. Capsule axis = body-local +Y.
    GrbShapeType shape;
    double radius;            // sphere/capsule
    double half_height;       // capsule: half of the segment between the two hemisphere centers
    double half_extents[3];   // box
    double friction;          // Coulomb mu (static and dynamic share it in v0)
    double restitution;
    // Accumulated external force/torque (world), consumed and cleared by grb_world_step.
    double force[3];
    double torque[3];
    // Solver scratch (valid only inside a step).
    double prev_pos[3];
    double prev_rot[4];
    double pre_vel[3];
    double pre_omega[3];
} GrbBody;

typedef enum { GRB_JOINT_HINGE = 1, GRB_JOINT_BALL, GRB_JOINT_FIXED } GrbJointType;
typedef enum { GRB_MOTOR_OFF = 0, GRB_MOTOR_POSITION, GRB_MOTOR_TORQUE } GrbMotorMode;

typedef struct {
    GrbJointType type;
    int body_a;               // -1 = the static world
    int body_b;
    // Joint frame expressed in each body's own local frame (for body_a == -1: in world).
    double local_pos_a[3], local_rot_a[4];
    double local_pos_b[3], local_rot_b[4];
    // Hinge limits (radians, on the unwrapped angle). has_limits == 0 = free rotation.
    int has_limits;
    double lower, upper;
    // Ball limits: swing cone half-angle and twist range, radians.
    double swing_max;
    double twist_lower, twist_upper;
    // Actuator (hinge only). effort_limit/velocity_limit <= 0 means "no limit".
    GrbMotorMode motor;
    double target_angle;      // POSITION mode
    double target_velocity;   // POSITION mode feed-forward velocity target
    double kp, kd;            // POSITION mode gains (N*m/rad, N*m*s/rad)
    double feedforward;       // added to the PD torque before clamping (N*m)
    double command_torque;    // TORQUE mode
    double effort_limit;      // N*m, datasheet max joint torque
    double velocity_limit;    // rad/s, datasheet max joint speed
    double damping;           // passive viscous joint friction, N*m*s/rad (hinge and ball; not
                              // drawn from the motor's effort budget)
    // Outputs, refreshed every grb_world_step.
    double angle;             // unwrapped hinge angle (rad)
    double angle_velocity;    // rad/s
    double applied_torque;    // mean motor torque over the last step's substeps (N*m)
    double peak_torque;       // max |motor torque| over the last step's substeps
    int saturated;            // 1 if the effort limit clamped the motor during the last step
    // Internal: unwrap bookkeeping, per-substep motor torque.
    double sub_torque;
    double raw_angle_prev;
    int angle_initialized;
} GrbJoint;

typedef struct {
    double normal[3];         // unit normal, pointing out of the solid half-space
    double offset;            // plane = { p : dot(normal, p) == offset }
} GrbPlane;

typedef struct {
    int body;
    double local_point[3];    // contact point in body frame
    double lambda_n;          // normal Lagrange multiplier (impulse*h)
    int plane;
} GrbContact;

typedef struct {
    double gravity[3];
    int substeps;             // default 16
    double substep_h;         // set by grb_world_step
    int contact_iterations;   // normal-contact Gauss-Seidel sweeps per substep, default 4
    uint32_t body_count, joint_count, plane_count;
    GrbBody bodies[GRB_MAX_BODIES];
    GrbJoint joints[GRB_MAX_JOINTS];
    GrbPlane planes[GRB_MAX_PLANES];
    // Contact scratch from the most recent substep.
    uint32_t contact_count;
    GrbContact contacts[GRB_MAX_CONTACTS];
    // Per-substep motor torque accumulator (scratch; kept in the world, not a static, so separate
    // worlds can step on separate threads -- e.g. parallel RL rollouts).
    double motor_torque_scratch[GRB_MAX_BODIES][3];
} GrbWorld;

// --- world ---
void grb_world_init(GrbWorld *w, double gx, double gy, double gz);
int grb_world_add_plane(GrbWorld *w, double nx, double ny, double nz, double offset);
// Advances the world by dt seconds (one engine tick; SHANKPIT uses 1/64) in w->substeps substeps.
void grb_world_step(GrbWorld *w, double dt);

// --- bodies ---
// Adds a dynamic body with the given mass and principal moments (Ixx, Iyy, Izz in its own
// principal frame). mass <= 0 creates a static body. Returns the index, or -1 when full.
int grb_body_add(GrbWorld *w, double mass, const double principal_inertia[3],
                 const double pos[3], const double rot[4]);
void grb_body_set_sphere(GrbBody *b, double radius);
void grb_body_set_capsule(GrbBody *b, double radius, double half_height);
void grb_body_set_box(GrbBody *b, double hx, double hy, double hz);
// Principal moments of standard solids (about the center of mass), for callers without datasheet
// inertia tensors (ragdolls, props).
void grb_inertia_box(double mass, double hx, double hy, double hz, double out[3]);
void grb_inertia_sphere(double mass, double radius, double out[3]);
void grb_inertia_capsule(double mass, double radius, double half_height, double out[3]); // axis Y

void grb_body_add_force_at(GrbBody *b, const double force[3], const double world_point[3]);

// --- joints ---
// Joint frame given in WORLD space at creation time; converted to each body's local frame.
int grb_joint_add(GrbWorld *w, GrbJointType type, int body_a, int body_b,
                  const double world_pos[3], const double world_rot[4]);
// Same, but with explicit per-body local frames (what .grobot compilation produces).
int grb_joint_add_local(GrbWorld *w, GrbJointType type, int body_a, int body_b,
                        const double local_pos_a[3], const double local_rot_a[4],
                        const double local_pos_b[3], const double local_rot_b[4]);

// Re-seeds a hinge's unwrapped-angle bookkeeping from the current body poses, choosing the 2*pi
// branch nearest `hint` (callers that teleport a robot into a pose -- an RL reset -- call this so
// a joint placed at, say, +200 degrees reads +200, not -160).
void grb_joint_sync_angle(GrbWorld *w, int joint, double hint);

// --- diagnostics (tests, reward terms, feasibility) ---
double grb_kinetic_energy(const GrbWorld *w);
double grb_potential_energy(const GrbWorld *w);     // gravitational, relative to the origin
void grb_linear_momentum(const GrbWorld *w, double out[3]);
void grb_angular_momentum(const GrbWorld *w, double out[3]); // about the world origin
// World-space position of a body-local point.
void grb_body_world_point(const GrbBody *b, const double local[3], double out[3]);
// Max positional drift across all joint attachments (meters) -- a constraint-quality metric.
double grb_max_joint_error(const GrbWorld *w);

// --- small math helpers, exported because every consumer (robot builder, ragdoll, env) needs
// the exact same conventions ---
void grb_quat_mul(const double a[4], const double b[4], double out[4]);
void grb_quat_conj(const double q[4], double out[4]);
void grb_quat_rotate(const double q[4], const double v[3], double out[3]);
void grb_quat_normalize(double q[4]);
void grb_quat_from_axis_angle(const double axis[3], double angle, double out[4]);
// URDF rpy convention: R = Rz(yaw) * Ry(pitch) * Rx(roll).
void grb_quat_from_rpy(double roll, double pitch, double yaw, double out[4]);

#endif // GOLDENBAND_GRB_H
