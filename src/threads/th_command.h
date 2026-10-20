#ifndef TH_COMMAND_H
#define TH_COMMAND_H

/* Ingresso da tastiera. L'unico scrittore di bb_cmd.*/

#include "blackboard.h"
#include "rt_time.h"

#include <stdatomic.h>

typedef struct {
    bb_cmd_t   *bb_cmd;
    atomic_int *running;

    rt_periodic_t per;
} th_command_ctx_t;

/* Mette il terminale in modalita' raw non bloccante; lo ripristina all'uscita.
   Ritorna 0 quando stdin e' un terminale, -1 altrimenti (in quel caso
   l'ingresso resta inattivo). */
int  th_command_setup_terminal(void);
void th_command_restore_terminal(void);

void th_command_help(void);

void *th_command_run(void *arg);

#endif /* TH_COMMAND_H */