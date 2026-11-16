#ifndef TH_GAIT_H
#define TH_GAIT_H

/* Turns the operator command into joint references: phase
   clock, foot trajectories, inverse kinematics. The only writer of bb_des. */

#include "blackboard.h"
#include "gait.h"
#include "rt_time.h"

#include <stdatomic.h>

typedef struct {
    bb_state_t *bb_state;   /* read: measured body velocity */
    bb_cmd_t   *bb_cmd;     /* read: operator command */
    bb_att_t   *bb_att;     /* read: estimated attitude*/

    bb_des_t   *bb_des;     /* write */
    atomic_int *running;

    gait_params_t params;
    double        att_kp;
    
    rt_periodic_t per;
} th_gait_ctx_t;

void *th_gait_run(void *arg);

#endif /* TH_GAIT_H */