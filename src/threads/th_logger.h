#ifndef TH_LOGGER_H
#define TH_LOGGER_H

/* Telemetria su file CSV. La priorita' piu' bassa del sistema. */

#include "blackboard.h"
#include "rt_time.h"

#include <stdatomic.h>
#include <stdio.h>

typedef struct {
    bb_state_t *bb_state;
    bb_att_t   *bb_att;
    atomic_int *running;

    const char *path;
    FILE       *fp;

    rt_periodic_t per;
    rt_acc_t      write_ns;   /* quanto dura davvero la scrittura su file */
} th_logger_ctx_t;

void *th_logger_run(void *arg);

#endif /* TH_LOGGER_H */