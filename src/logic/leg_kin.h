#ifndef LEG_KIN_H
#define LEG_KIN_H

/* Closed-form kinematics of a single leg.
   Everything is expressed in the hip frame: origin on the abduction axis
   of the leg, axes aligned with the trunk (x forward, y left, z up).
*/

#include "config.h"

/* Link geometry, taken from the model. All measurements are in metres. */
#define LEG_L1   0.213     /* thigh length */
#define LEG_L2   0.213     /* shank length, identical by design */
#define LEG_D    0.08      /* distance from the abduction axis to the leg plane, in absolute value */

/* Position of the hip origin relative to the trunk centre. */
#define LEG_HIP_X 0.1881
#define LEG_HIP_Y 0.04675

/* Joint limits, indexed with JNT_ABAD / JNT_HIP / JNT_KNEE. */
extern const double LEG_Q_MIN[CFG_JOINTS_PER_LEG];
extern const double LEG_Q_MAX[CFG_JOINTS_PER_LEG];

/* The four foot targets in the trunk frame. */
typedef struct {
    double p[CFG_NUM_LEGS][3];
} feet_target_t;

/* Signed lateral offset: the only thing that tells right from left. */
double leg_offset_d(int leg);

/* Origin of the abduction joint in the trunk frame: it is the point where
   the leg attaches to the trunk, and the origin used by leg_ik. */
void leg_abad_pos(int leg, double out[3]);

/* Joint angles -> foot position in the abduction frame. Only used
*  to check that the inverse function works correctly. */
//void leg_fk(int leg, const double q[CFG_JOINTS_PER_LEG], double p[3]);

/* Inverse kinematics: foot position -> joint angles. An unreachable pose
   is brought back to the nearest valid point. */
void leg_ik(int leg, const double p[3], double q[CFG_JOINTS_PER_LEG]);

/* The four feet in the trunk frame -> the twelve joint angles,
   in actuator order. */
void legs_ik_trunk(const feet_target_t *feet, double q[CFG_NUM_JOINTS]);

/* Foot position exactly below the hip for a given trunk height,
   in the trunk frame. It is the reference stance around which
   the gait introduces its variations. */
void leg_nominal_foot(int leg, double height, double p_trunk[3]);

#endif /* LEG_KIN_H */