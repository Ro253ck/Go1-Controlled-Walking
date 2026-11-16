#ifndef STATE_H
#define STATE_H

#include "config.h"
#include <stdint.h>

typedef struct {
    uint64_t seq;          /* incremented by the blackboard on every publication */
    double   sim_time;     /* MuJoCo clock*/

    double   qpos[CFG_NQ]; /* full generalised position, for the renderer */
    double   base_pos[3];  /* simulation ground truth, telemetry only */
    double   base_quat[4]; /* simulation ground truth, reference for the estimator only */
    double   base_vel[3];  /* simulation ground truth, used by the Raibert term */
} robot_state_t;

/* Raw inertial measurements. Written by th_sim, read by th_estimator.*/
typedef struct {
    uint64_t seq;
    double   sim_time;
    double   gyro[3];   /* rad/s, body frame */
    double   acc[3];    /* m/s^2, specific force, body frame */
} imu_t;

/* Estimated attitude. Written by th_estimator, read by th_gait. */
typedef struct {
    uint64_t seq;
    double   q[4];              /* body to world */
    double   roll, pitch, yaw;
} att_t;

/* Safety state. Written by th_watchdog, read by th_sim and th_gait.
   The decision is taken at low rate and low priority; the action is
   carried out by the highest priority thread in the system.*/
enum {
    SAFE_OK = 0,
    SAFE_STOP      /* zero torque on every joint */
};

enum {
    WD_TILT      = 1u << 0,   /* the trunk is tilted beyond the limit     */
    WD_DEAD_GAIT = 1u << 6    /* the gait has stopped publishing          */
};

typedef struct {
    uint64_t seq;
    int      mode;
    uint32_t alarms;       /* mask of the alarms that fired; latched, so it never clears */
} safety_t;

/* Operator input. Written by th_command, read by th_gait. */
typedef struct {
    uint64_t seq;
    double   vx, vy, wz;   /* desired velocity, body frame */
    double   height;       /* desired trunk height */
    int      gait_on;      /* 0 stand still, 1 trot */
    int      quit;

    /* Fault injection, for the watchdog. */
    uint32_t push;         /* incremented on every shove request */
    double   push_force;   /* N, set together with the counter */
    int      fault_gait;   /* 1 = the gait thread stops publishing */

    int      att_on;       /* 1 = the estimated attitude feeds the gait */
} cmd_t;

/* Joint references. Written by th_gait, read by th_sim.*/
typedef struct {
    uint64_t seq;
    double   q[CFG_NUM_JOINTS];
    double   qd[CFG_NUM_JOINTS];
} joint_cmd_t;

#endif /* STATE_H */
