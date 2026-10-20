#ifndef TH_WATCHDOG_H
#define TH_WATCHDOG_H

/* Supervisione di sicurezza. L'unico scrittore di bb_safe.*/
#include "blackboard.h"
#include "rt_time.h"

#include <stdatomic.h>


typedef struct {
    bb_imu_t   *bb_imu;     /* lettura: grezza, indipendente dall'estimatore */
    bb_des_t   *bb_des;     /* lettura: solo vitalita' */
    bb_safe_t  *bb_safe;    /* scrittura */
    atomic_int *running;

    uint32_t tilt_hist;     /* Sliding Window per accorgersi della caduta*/

    rt_periodic_t per;
} th_watchdog_ctx_t;

void *th_watchdog_run(void *arg);

#endif /* TH_WATCHDOG_H */