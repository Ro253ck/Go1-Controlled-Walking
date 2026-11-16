#ifndef RENDER_H
#define RENDER_H

/* Best-effort visualisation. It runs on the main thread because GLFW and
   OpenGL want a single owning thread, and it stays on SCHED_OTHER
   so it can never steal time from the control loop.

   It never touches the live mjData: it keeps a private copy, loads into it the
   pose published on the blackboard and recomputes only what is needed to draw.
   Locking the live data would hold the mutex for a whole frame. */

#include "state.h"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

typedef struct {
    GLFWwindow   *win;
    const mjModel *m;   /*Read only*/
    mjData       *d;      /* private, for drawing only */
    mjvCamera     cam;
    mjvOption     opt;
    mjvScene      scn;
    mjrContext    con;

    int    btn_left, btn_middle, btn_right; /*Mouse button state*/
    double lastx, lasty; /*Last cursor position*/

    char   hud[384];
    double att_kp;      /* shown on the HUD, set by main after init */
    char   banner[96];
} render_t;

/* Returns 0 on success, -1 when no window could be created
   (run without a display). */
int  render_init(render_t *r, const mjModel *m);
void render_free(render_t *r);

int  render_should_close(render_t *r);
void render_frame(render_t *r, const robot_state_t *st, const cmd_t *cmd,
                  const safety_t *safe, double rt_factor);

#endif /* RENDER_H */
