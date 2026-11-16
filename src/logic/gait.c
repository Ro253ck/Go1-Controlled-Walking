#include "gait.h"

#include <math.h>

/* FR and RL share one phase, FL and RR the opposite one: the two feet on the ground at
* any instant are diagonally opposite. */
const double GAIT_TROT_OFFSET[CFG_NUM_LEGS] = { 0.0, 0.5, 0.5, 0.0 };


void gait_params_default(gait_params_t *gp)
{
    gp->period    = CFG_GAIT_CYCLE;
    gp->duty      = CFG_GAIT_DUTY;
    gp->height    = CFG_BODY_HEIGHT;
    gp->clearance = CFG_FOOT_CLEARANCE;
    gp->raibert_k = CFG_RAIBERT_K;
}

/*Returns the point the leg phase has reached*/
double gait_leg_phase(const gait_params_t *gp, double t, int leg)
{
    double ph = fmod(t / gp->period + GAIT_TROT_OFFSET[leg], 1.0);
    if (ph < 0.0) ph += 1.0;
    return ph;
}

/*Returns whether the leg is in stance*/
int gait_in_stance(const gait_params_t *gp, double phase)
{
    return phase < gp->duty;
}

/* Cubic Hermite on [0,1] with tangents imposed at the endpoints. Used for the
   horizontal part of the swing: the foot must lift off and return to the ground
   moving at the speed of the terrain, otherwise it scuffs at lift-off and
   slips at touchdown.
   We use the same derivative for the start and the end, so we only have m
   while u represents how far into the swing we are*/
static double hermite(double p0, double p1, double m, double u)
{
    const double u2 = u * u;
    const double u3 = u2 * u;

    const double h00 =  2.0*u3 - 3.0*u2 + 1.0;
    const double h10 =        u3 - 2.0*u2 + u;
    const double h01 = -2.0*u3 + 3.0*u2;
    const double h11 =        u3 -     u2;

    return h00 * p0 + h10 * m + h01 * p1 + h11 * m;
}

void gait_feet_targets(const gait_params_t *gp,
                       double t,
                       const double v_cmd[3],
                       const double v_meas[2],
                       double amplitude,
                       const gait_attitude_t *att,
                       feet_target_t *out)
{
    const double stance_time = gp->duty * gp->period;

    /* Footstep feedback, Raibert style: going faster than
       commanded shifts the whole pattern forward, so the feet
       land further ahead and brake. Applied to the reference point and not
       only to the touchdown point, which keeps stance and swing continuous
       at the instant of contact. */
    double corr_x = gp->raibert_k * (v_meas[0] - v_cmd[0]) * amplitude;
    double corr_y = gp->raibert_k * (v_meas[1] - v_cmd[1]) * amplitude;

    /* Saturation: right after a step change in the command the error
       is as large as the command itself, and an unbounded correction
       would ask for a foot where the leg cannot reach. */
    if (corr_x >  CFG_RAIBERT_MAX) corr_x =  CFG_RAIBERT_MAX;
    if (corr_x < -CFG_RAIBERT_MAX) corr_x = -CFG_RAIBERT_MAX;
    if (corr_y >  CFG_RAIBERT_MAX) corr_y =  CFG_RAIBERT_MAX;
    if (corr_y < -CFG_RAIBERT_MAX) corr_y = -CFG_RAIBERT_MAX;

    for (int leg = 0; leg < CFG_NUM_LEGS; leg++) {
        double hip[3], ref[3];

        leg_abad_pos(leg, hip);
        leg_nominal_foot(leg, gp->height, ref);

        /* Yaw makes every leg travel at a different speed: the outer
           side of the turn pushes more, exactly as in a tracked
           vehicle. */
        const double vix = v_cmd[0] - v_cmd[2] * hip[1];
        const double viy = v_cmd[1] + v_cmd[2] * hip[0];

        /* Ground covered by this foot during one stance. */
        const double sx = vix * stance_time * amplitude;
        const double sy = viy * stance_time * amplitude;

        const double bx = ref[0] + corr_x;
        const double by = ref[1] + corr_y;

        /* Levelling term */
        double dz = 0.0;
        if (att) {
            dz = -(att->kp * att->roll)  * hip[1] - (att->kp * att->pitch) * hip[0];
            dz *= amplitude;
        }

        const double ph = gait_leg_phase(gp, t, leg);

        if (gait_in_stance(gp, ph)) {
            /* Foot planted on the ground: it slides backwards under the body,
               centred so that on average it sits below the hip. */
            const double s = ph / gp->duty;

            out->p[leg][0] = bx - sx * (s - 0.5);
            out->p[leg][1] = by - sy * (s - 0.5);
            out->p[leg][2] = -gp->height + dz;
        } else {
            const double u = (ph - gp->duty) / (1.0 - gp->duty);

            /* Tangent at the endpoints, in units of u*/
            const double mx = -sx * (1.0 - gp->duty) / gp->duty;
            const double my = -sy * (1.0 - gp->duty) / gp->duty;

            out->p[leg][0] = hermite(bx - 0.5*sx, bx + 0.5*sx, mx, u);
            out->p[leg][1] = hermite(by - 0.5*sy, by + 0.5*sy, my, u);

            /* Bell with zero slope at both ends: the foot lifts and
               sets down with no vertical jerk. */
            out->p[leg][2] = -gp->height + dz
                           + gp->clearance * amplitude
                           * 0.5 * (1.0 - cos(2.0 * M_PI * u));
        }
    }
}