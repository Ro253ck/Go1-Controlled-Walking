#ifndef TH_SIM_H
#define TH_SIM_H

#include "blackboard.h"
#include "rt_time.h"
#include "sim.h"

#include <stdatomic.h>

typedef struct {
    sim_t      *sim;
    bb_state_t *bb_state;   /* scrittura */
    bb_imu_t   *bb_imu;     /* scrittura */

    bb_des_t   *bb_des;     /* lettura: riferimenti di giunto dal thread gait */
    bb_safe_t  *bb_safe;    /* lettura: agisce sulla decisione del watchdog */
    bb_cmd_t   *bb_cmd;     /* lettura: solo iniezione di guasti */
    
    atomic_int *running;

    rt_periodic_t per;
} th_sim_ctx_t;

void *th_sim_run(void *arg);

#endif /* TH_SIM_H */