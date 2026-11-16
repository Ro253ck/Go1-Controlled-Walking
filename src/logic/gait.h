#ifndef GAIT_H
#define GAIT_H
 
#include "config.h"
#include "leg_kin.h"

/* Phase offsets that define the trot: the diagonal pairs move
   together. Indexed FR, FL, RR, RL. */
extern const double GAIT_TROT_OFFSET[CFG_NUM_LEGS];

typedef struct {
    double period;      /* T_g, one full stance plus swing */
    double duty;        /* beta, fraction of the cycle spent in stance */
    double height;      /* h0, nominal trunk height */
    double clearance;   /* H, swing apex */
    double raibert_k;
} gait_params_t;

/* Attitude feedback applied through the feet: no force control,
   just a longer leg on the side that has dropped. It is the only
   point where the IMU estimate enters the control loop. */
typedef struct {
    double roll, pitch;    /* estimated attitude */
    double kp;             
} gait_attitude_t;

void gait_params_default(gait_params_t *gp);

/* Phase of a leg at time t, in [0,1). */
double gait_leg_phase(const gait_params_t *gp, double t, int leg);

/* 1 as long as the leg is in stance. */
int gait_in_stance(const gait_params_t *gp, double phase);

/* Foot targets in the trunk frame.*/
void gait_feet_targets(const gait_params_t *gp,
                       double t,
                       const double v_cmd[3],
                       const double v_meas[2],
                       double amplitude,
                       const gait_attitude_t *att,   /* NULL disables it */
                       feet_target_t *out);

#endif /* GAIT_H */