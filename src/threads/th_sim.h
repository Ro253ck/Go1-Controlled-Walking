#ifndef TH_SIM_H
#define TH_SIM_H

#include "blackboard.h"
#include "rt_time.h"
#include "sim.h"

#include <stdatomic.h>

typedef struct {
    sim_t      *sim;
    bb_state_t *bb_state;   /* write */
    bb_imu_t   *bb_imu;     /* write */

    bb_des_t   *bb_des;     /* read: joint references from the gait thread */
    bb_safe_t  *bb_safe;    /* read: acts on the watchdog decision */
    bb_cmd_t   *bb_cmd;     /* read: fault injection only */
    
    atomic_int *running;

    rt_periodic_t per;
} th_sim_ctx_t;

void *th_sim_run(void *arg);

#endif /* TH_SIM_H */