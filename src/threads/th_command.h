#ifndef TH_COMMAND_H
#define TH_COMMAND_H

/* Keyboard input. The only writer of bb_cmd.*/

#include "blackboard.h"
#include "rt_time.h"

#include <stdatomic.h>

typedef struct {
    bb_cmd_t   *bb_cmd;
    atomic_int *running;

    rt_periodic_t per;
} th_command_ctx_t;

/* Puts the terminal into non-blocking raw mode; restores it on exit.
   Returns 0 when stdin is a terminal, -1 otherwise (in that case
   the input stays inactive). */
int  th_command_setup_terminal(void);
void th_command_restore_terminal(void);

void th_command_help(void);

void *th_command_run(void *arg);

#endif /* TH_COMMAND_H */