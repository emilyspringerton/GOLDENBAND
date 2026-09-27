// grobot.h — runtime loader for compiled GOLDEN BAND robots (`.grobot`, produced by
// `gbtool robot compile` from a manufacturer-sourced `.grobot.json` spec -- see
// format/GROBOT_FORMAT.md), and the bridge that instantiates one as articulated rigid bodies in a
// grb world: one grb body per moving link (datasheet mass, COM, principal inertia), one hinge per
// joint (datasheet position limits, max joint speed, max joint torque).
//
// Same philosophy as gband.h/gskel.h: fixed binary layout, bounded parser, no heap, no JSON.
#ifndef GOLDENBAND_GROBOT_H
#define GOLDENBAND_GROBOT_H

#include <stdint.h>
#include "grb.h"

#define GROBOT_MAX_JOINTS 64
#define GROBOT_NAME_LEN 32

typedef enum { GROBOT_REVOLUTE = 1, GROBOT_CONTINUOUS = 2 } GRobotJointType;

typedef struct {
    char name[GROBOT_NAME_LEN];
    char child[GROBOT_NAME_LEN];
    int32_t parent;              // joint index whose child link is this joint's parent; -1 = base
    uint32_t type;               // GRobotJointType
    double origin_xyz[3];        // m, in the parent link frame
    double origin_quat[4];       // x,y,z,w
    double axis[3];              // unit, in the joint frame
    double lower, upper;         // rad (ignored for CONTINUOUS)
    double velocity;             // rad/s  -- datasheet max joint speed
    double effort;               // N*m    -- datasheet max joint torque
    double damping;              // N*m*s/rad
    double mass;                 // kg, child link
    double com[3];               // m, child link frame
    double principal_quat[4];    // child link frame -> principal inertia frame
    double principal_moments[3]; // kg*m^2
} GRobotJoint;

typedef struct {
    uint32_t version;
    uint32_t joint_count;
    int32_t tcp_joint;           // joint whose child link carries the tool center point
    double tcp_xyz[3];           // TCP offset in that link's frame
    char name[GROBOT_NAME_LEN];
    unsigned char spec_hash[32]; // sha256 of the .grobot.json spec this was compiled from
    GRobotJoint joints[GROBOT_MAX_JOINTS];
} GRobot;

// A robot placed into a grb world.
typedef struct {
    int body[GROBOT_MAX_JOINTS];   // grb body index of each joint's child link
    int joint[GROBOT_MAX_JOINTS];  // grb joint index of each robot joint
    double base_pos[3];
    double base_rot[4];
} GRobotInstance;

// Loads and validates a .grobot file. Returns 1 on success, 0 on failure. No heap allocation.
int grobot_init(const char *path, GRobot *robot);

// Forward kinematics of every link frame at joint angles q (world, given the base pose).
void grobot_fk(const GRobot *robot, const double base_pos[3], const double base_rot[4], const double *q,
               double link_pos[][3], double link_rot[][4]);

// Adds the robot to the world at joint angles q (NULL = all zero), fixed to the world at the base
// pose. Every hinge gets the datasheet limits/effort/velocity; motors start OFF. Returns 1 on
// success, 0 if the world is out of body/joint slots.
int grobot_spawn(GrbWorld *w, const GRobot *robot, const double base_pos[3], const double base_rot[4],
                 const double *q, GRobotInstance *inst);

// Teleports an already-spawned robot to joint state (q, qd) -- an RL episode reset. qd may be NULL.
void grobot_set_state(GrbWorld *w, const GRobot *robot, const GRobotInstance *inst, const double *q, const double *qd);

// Tool center point in world space, from the simulated body state.
void grobot_tcp(const GrbWorld *w, const GRobot *robot, const GRobotInstance *inst, double out[3]);

// Effective inertia about joint j's axis: the whole downstream subtree treated as one rigid body
// at the current state (kg*m^2). Used to tune a joint servo (kp = I*wn^2, kd = 2*zeta*I*wn) so
// every joint has the same closed-loop bandwidth regardless of how heavy its subtree is.
double grobot_axis_inertia(const GrbWorld *w, const GRobot *robot, const GRobotInstance *inst, int j);

#endif // GOLDENBAND_GROBOT_H
