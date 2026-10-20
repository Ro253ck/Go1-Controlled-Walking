#ifndef TH_ESTIMATOR_H
#define TH_ESTIMATOR_H

#include "attitude.h"
#include "blackboard.h"
#include "rt_time.h"

#include <stdatomic.h>

typedef struct {
    bb_imu_t *bb_imu;     /* lettura: le misure inerziali grezze */
    bb_att_t *bb_att;     /* scrittura */
    atomic_int *running;

    filter_t filter;

    rt_periodic_t per;
    uint64_t      n_seed_tries;
} th_estimator_ctx_t;

void *th_estimator_run(void *arg);

#endif /* TH_ESTIMATOR_H */
