#ifndef TH_WATCHDOG_H
#define TH_WATCHDOG_H

/* Safety supervision. The only writer of bb_safe.*/
#include "blackboard.h"
#include "rt_time.h"

#include <stdatomic.h>


typedef struct {
    bb_imu_t   *bb_imu;     /* read: raw, independent of the estimator */
    bb_des_t   *bb_des;     /* read: liveness only */
    bb_safe_t  *bb_safe;    /* write */
    atomic_int *running;

    uint32_t tilt_hist;     /* Sliding window to notice the fall*/

    rt_periodic_t per;
} th_watchdog_ctx_t;

void *th_watchdog_run(void *arg);

#endif /* TH_WATCHDOG_H */