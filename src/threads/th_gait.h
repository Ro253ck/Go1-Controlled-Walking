#ifndef TH_GAIT_H
#define TH_GAIT_H

/* Trasforma il comando dell'operatore in riferimenti di giunto: orologio di
   fase, traiettorie dei piedi, cinematica inversa. L'unico scrittore di bb_des. */

#include "blackboard.h"
#include "gait.h"
#include "rt_time.h"

#include <stdatomic.h>

typedef struct {
    bb_state_t *bb_state;   /* lettura velocita' misurata del corpo */
    bb_cmd_t   *bb_cmd;     /* lettura comando dell'operatore */
    bb_att_t   *bb_att;     /* lettura assetto stimato*/

    bb_des_t   *bb_des;     /* scrittura */
    atomic_int *running;

    gait_params_t params;
    double        att_kp;
    
    rt_periodic_t per;
} th_gait_ctx_t;

void *th_gait_run(void *arg);

#endif /* TH_GAIT_H */