#include "leg_kin.h"

#include <math.h>

/*Initialisation of the joint limits*/
const double LEG_Q_MIN[CFG_JOINTS_PER_LEG] = { -0.863, -0.686, -2.818 };
const double LEG_Q_MAX[CFG_JOINTS_PER_LEG] = {  0.863,  4.501, -0.888 };

/*They tell which side the leg is on. FL and RL are on the y+ side, FR and RR on the y- side. */
static int leg_is_left(int leg)  { return (leg == LEG_FL || leg == LEG_RL); }
static int leg_is_front(int leg) { return (leg == LEG_FR || leg == LEG_FL); }

/*Returns the distance of the leg from the abduction axis, signed according to which leg we are considering*/
double leg_offset_d(int leg)
{
    return leg_is_left(leg) ? LEG_D : -LEG_D;
}

/*Returns the position of the abduction joint, according to which leg we are considering.
  The front and left ones are positive, the rear and right ones negative*/
void leg_abad_pos(int leg, double out[3])
{
    out[0] = leg_is_front(leg) ? LEG_HIP_X : -LEG_HIP_X;
    out[1] = leg_is_left(leg)  ? LEG_HIP_Y : -LEG_HIP_Y;
    out[2] = 0.0;
}

/*Brings every angle back into the interval [-pi,pi]*/
static double wrap_pi(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// /*Forward kinematics*/
// void leg_fk(int leg, const double q[CFG_JOINTS_PER_LEG], double p[3])
// {
//     const double d   = leg_offset_d(leg);
//     const double s2  = sin(q[JNT_HIP]);
//     const double c2  = cos(q[JNT_HIP]);
//     const double s23 = sin(q[JNT_HIP] + q[JNT_KNEE]);
//     const double c23 = cos(q[JNT_HIP] + q[JNT_KNEE]);

//     /* Foot seen from the thigh attachment, before the abduction: the leg plane. */
//     const double a = -LEG_L1 * s2 - LEG_L2 * s23;
//     const double c = -LEG_L1 * c2 - LEG_L2 * c23;

//     /* Rotating about x leaves x unchanged, and this is what decouples
//        the abduction from the other two joints. */
//     p[0] = a;
//     p[1] = d * cos(q[JNT_ABAD]) - c * sin(q[JNT_ABAD]);
//     p[2] = d * sin(q[JNT_ABAD]) + c * cos(q[JNT_ABAD]);
// }

/*Inverse kinematics*/
void leg_ik(int leg, const double p[3], double q[CFG_JOINTS_PER_LEG])
{
    const double d = leg_offset_d(leg); /*It is a constant*/

    /* Step 1: we look for the abduction angle, which lives in the frontal plane (y-z). */
    double c2 = p[1] * p[1] + p[2] * p[2] - d * d;
    /* requested foot closer to the axis than d allows */
    if (c2 < 0.0) c2 = 0.0;
    const double c = -sqrt(c2);   /* negative: the foot is below the hip */
    
    /* Given the position (py,pz), which is the position (d,c) after the abduction,
       we can obtain the angle between the two like this: */
    q[JNT_ABAD] = wrap_pi(atan2(p[2], p[1]) - atan2(c, d));

    /* Step 2 (x-z plane): the knee, by the law of cosines on the
       hip-knee-foot triangle.*/
    const double L_sq  = p[0] * p[0] + c * c;
    double cos_q3 = (L_sq - LEG_L1 * LEG_L1 - LEG_L2 * LEG_L2) / (2.0 * LEG_L1 * LEG_L2);

    /*Check whether the target is reachable */
    if (cos_q3 >  1.0) cos_q3 =  1.0;
    if (cos_q3 < -1.0) cos_q3 = -1.0;

    q[JNT_KNEE] = -acos(cos_q3);   

    /* Step 3: we use the law of cosines again, this time to
       obtain the hip angle */
    const double L = sqrt(L_sq);
    double cos_psi;

    if (L > 1e-9) {
        cos_psi = (L_sq + LEG_L1*LEG_L1 - LEG_L2*LEG_L2) / (2.0 * LEG_L1 * L);
        if (cos_psi >  1.0) cos_psi =  1.0;
        if (cos_psi < -1.0) cos_psi = -1.0;
    } else {
        cos_psi = 1.0;
    }

    q[JNT_HIP] = wrap_pi(atan2(-p[0], -c) + acos(cos_psi));

    /*Check that every joint is within its limits*/
    for (int j = 0; j < CFG_JOINTS_PER_LEG; j++) {
        if (q[j] < LEG_Q_MIN[j]) q[j] = LEG_Q_MIN[j];
        if (q[j] > LEG_Q_MAX[j]) q[j] = LEG_Q_MAX[j];
    }
}

void legs_ik_trunk(const feet_target_t *feet, double q[CFG_NUM_JOINTS])
{

    for (int leg = 0; leg < CFG_NUM_LEGS; leg++) {
        double abad[3], p_abad[3];

        leg_abad_pos(leg, abad);
        for (int i = 0; i < 3; i++)
            p_abad[i] = feet->p[leg][i] - abad[i];

        leg_ik(leg, p_abad, &q[leg * CFG_JOINTS_PER_LEG]);
    }
}

void leg_nominal_foot(int leg, double height, double p_trunk[3])
{
    double abad[3];
    leg_abad_pos(leg, abad);

    /* Exactly below the abduction joint, offset laterally by d:
       it is the pose that gives exactly zero abduction angle. */
    p_trunk[0] = abad[0];
    p_trunk[1] = abad[1] + leg_offset_d(leg);
    p_trunk[2] = -height;
}