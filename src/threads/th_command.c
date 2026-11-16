#include "th_command.h"

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

static struct termios g_saved;
static int g_have_terminal = 0;

int th_command_setup_terminal(void)
{
    struct termios raw;

    /*We make stdin non-blocking, that is, when we call read it returns immediately with whatever is there, otherwise nothing*/
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);

    if (!isatty(STDIN_FILENO)) return -1; /*We check that it is a real terminal and not a file.txt or something else*/
    if (tcgetattr(STDIN_FILENO, &g_saved) != 0) return -1; /*We save the current terminal settings into g_saved*/

    raw = g_saved;
    
    raw.c_lflag &= (tcflag_t)~(ICANON | ECHO); /*Sets immediate return of a character and no echo to the screen*/
    /*These are needed for polling reads*/
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;

    /*We apply the new settings*/
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return -1;

    g_have_terminal = 1;
    return 0;
}

/*Restores the original terminal settings*/
void th_command_restore_terminal(void)
{
    if (g_have_terminal) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_saved);
        g_have_terminal = 0;
    }
}

/*Prints the available commands to the screen*/
void th_command_help(void)
{
    puts("  w / s   forward speed        a / d   sideways speed");
    puts("  q / e   turn rate            r / f   body height");
    puts("  space   stop                 g       toggle trot");
    puts("  p       shove (hold for more) k      kill/revive the gait thread");
    puts("  t       attitude correction on/off");
    puts("  x       quit\n");
}

static double clampd(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void *th_command_run(void *arg)
{
    th_command_ctx_t *c = (th_command_ctx_t *)arg;
    cmd_t cmd;

    bb_cmd_read(c->bb_cmd, &cmd);
    rt_periodic_init(&c->per, CFG_COMMAND_PERIOD_NS, CFG_MAX_CATCHUP);

    while (atomic_load(c->running)) {
        char buf[64];
        int  seen[128];
        ssize_t n;
        int  changed = 0;

        rt_periodic_wait(&c->per);

        memset(seen, 0, sizeof(seen));

        /* Drains everything the terminal has queued, but remembers only
           WHICH keys appeared, not how many times. A held key means
           one step per cycle. */
        while ((n = read(STDIN_FILENO, buf, sizeof(buf))) > 0) {
            for (ssize_t i = 0; i < n; i++) {
                unsigned char ch = (unsigned char)buf[i];
                if (ch < 128) seen[ch] = 1;
            }
            if ((size_t)n < sizeof(buf)) break;
        }

        if (seen['w']) { cmd.vx += CFG_VX_STEP; changed = 1; }
        if (seen['s']) { cmd.vx -= CFG_VX_STEP; changed = 1; }
        if (seen['a']) { cmd.vy += CFG_VY_STEP; changed = 1; }
        if (seen['d']) { cmd.vy -= CFG_VY_STEP; changed = 1; }
        if (seen['q']) { cmd.wz += CFG_WZ_STEP; changed = 1; }
        if (seen['e']) { cmd.wz -= CFG_WZ_STEP; changed = 1; }
        if (seen['r']) { cmd.height += CFG_H_STEP; changed = 1; }
        if (seen['f']) { cmd.height -= CFG_H_STEP; changed = 1; }
        if (seen[' ']) { cmd.vx = cmd.vy = cmd.wz = 0.0; changed = 1; }
        if (seen['g']) { cmd.gait_on = !cmd.gait_on; changed = 1; }
        /* Held down, the one-step-per-cycle rule turns this
           into a steady shove instead of an avalanche of impulses. */
        if (seen['p']) { cmd.push++; cmd.push_force = CFG_PUSH_FORCE; changed = 1; }
        if (seen['k']) { cmd.fault_gait = !cmd.fault_gait; changed = 1; }
        if (seen['t']) { cmd.att_on     = !cmd.att_on;     changed = 1; }
        if (seen['x']) { cmd.quit = 1; changed = 1; }

        if (changed) {
            cmd.vx     = clampd(cmd.vx, -CFG_VX_MAX, CFG_VX_MAX);
            cmd.vy     = clampd(cmd.vy, -CFG_VY_MAX, CFG_VY_MAX);
            cmd.wz     = clampd(cmd.wz, -CFG_WZ_MAX, CFG_WZ_MAX);
            cmd.height = clampd(cmd.height, CFG_H_MIN, CFG_H_MAX);

            bb_cmd_write(c->bb_cmd, &cmd);

            if (cmd.quit) atomic_store(c->running, 0);
        }

        rt_periodic_done(&c->per);
    }
    return NULL;
}