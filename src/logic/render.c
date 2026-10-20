#include "render.h"

#include <stdio.h>
#include <string.h>

/*Funzioni Callback*/
static void cb_mouse_button(GLFWwindow *w, int button, int act, int mods)
{
    (void)button; (void)act; (void)mods; /*Ignoriamo i paramatri passati perchè vogliamo capire lo stato di tutti non solo di quello cambiato*/
    render_t *r = (render_t *)glfwGetWindowUserPointer(w);

    r->btn_left   = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS;
    r->btn_middle = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    r->btn_right  = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS;
    glfwGetCursorPos(w, &r->lastx, &r->lasty); /*Memorizza la nuova posizione del cursore*/
}

static void cb_mouse_move(GLFWwindow *w, double xpos, double ypos)
{
    render_t *r = (render_t *)glfwGetWindowUserPointer(w);
    int width, height;
    mjtMouse action;

    /*Se non viene premuto nessun pulsante manteniamo la poszione corrente ma la poszione in quanto
    * potrebbe essere cambiata*/
    if (!r->btn_left && !r->btn_middle && !r->btn_right) {
        r->lastx = xpos;
        r->lasty = ypos;
        return;
    }

    glfwGetWindowSize(w, &width, &height); /*Prendiamo dimensione della finestra*/

    /*Capiamo in base ai bottoni premuti l'azione da eseguire*/
    if (r->btn_right)     action = mjMOUSE_MOVE_V;
    else if (r->btn_left) action = mjMOUSE_ROTATE_V;
    else                  action = mjMOUSE_ZOOM;

    /*applichiamo lo spostamento calcolando i due delta necessari*/
    mjv_moveCamera(r->m, action,
                   (xpos - r->lastx) / height,
                   (ypos - r->lasty) / height,
                   &r->scn, &r->cam);
    r->lastx = xpos;
    r->lasty = ypos;
}


static void cb_scroll(GLFWwindow *w, double xoff, double yoff)
{
    (void)xoff;
    render_t *r = (render_t *)glfwGetWindowUserPointer(w);
    mjv_moveCamera(r->m, mjMOUSE_ZOOM, 0.0, -0.05 * yoff, &r->scn, &r->cam);
}

static void cb_key(GLFWwindow *w, int key, int scancode, int act, int mods)
{
    (void)scancode; (void)mods;
    if (act != GLFW_PRESS) return;
    if (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q)
        glfwSetWindowShouldClose(w, GLFW_TRUE);
}


int render_init(render_t *r, const mjModel *m)
{
    memset(r, 0, sizeof(*r));
    r->m = m;

    if (!glfwInit()) {
        fprintf(stderr, "NOTE: glfwInit failed, running headless\n");
        return -1;
    }

    r->win = glfwCreateWindow(1200, 800, "Go1 RTES", NULL, NULL);
    if (!r->win) {
        fprintf(stderr, "NOTE: no window available, running headless\n");
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(r->win);
    glfwSwapInterval(1);   /* il vsync scandisce il ritmo; il disegno non ha scadenze */

    mjv_defaultCamera(&r->cam);
    mjv_defaultOption(&r->opt);
    mjv_defaultScene(&r->scn);
    mjr_defaultContext(&r->con);

    mjv_makeScene(m, &r->scn, 2000);
    mjr_makeContext(m, &r->con, mjFONTSCALE_150);

    /* Segue il tronco cosi' il robot resta inquadrato quando inizia a muoversi. */
    r->cam.type        = mjCAMERA_TRACKING;
    r->cam.trackbodyid = mj_name2id(m, mjOBJ_BODY, "trunk");
    r->cam.distance    = 2.0;
    r->cam.elevation   = -20.0;
    r->cam.azimuth     = 120.0;

    r->d = mj_makeData(m); /*Copia privata di data*/

    glfwSetWindowUserPointer(r->win, r);
    glfwSetMouseButtonCallback(r->win, cb_mouse_button);
    glfwSetCursorPosCallback(r->win, cb_mouse_move);
    glfwSetScrollCallback(r->win, cb_scroll);
    glfwSetKeyCallback(r->win, cb_key);
    return 0;
}

void render_free(render_t *r)
{
    if (!r->win) return;
    if (r->d) mj_deleteData(r->d);
    mjr_freeContext(&r->con);
    mjv_freeScene(&r->scn);
    glfwTerminate();
    r->win = NULL;
}

int render_should_close(render_t *r)
{
    return r->win ? glfwWindowShouldClose(r->win) : 1;
}

/* Traduce la maschera di bit del watchdog in qualcosa di leggibile a schermo. */
static void alarms_text(uint32_t alarms, char *out, size_t n)
{
    const int tilt = (alarms & WD_TILT)      != 0;
    const int dead = (alarms & WD_DEAD_GAIT) != 0;

    snprintf(out, n, "%s%s%s",
             tilt ? "TILT" : "",
             (tilt && dead) ? " " : "",
             dead ? "GAIT-DEAD" : "");
}

static const char *mode_text(int mode)
{
    switch (mode) {
        case SAFE_STOP:   return "STOP";
        default:          return "OK";
    }
}

void render_frame(render_t *r, const robot_state_t *st, const cmd_t *cmd,
                  const safety_t *safe, double rt_factor)
{
    mjrRect viewport = {0, 0, 0, 0};

    /* Carica la posa pubblicata nei dati privati e ne ricava la geometria
       necessaria a disegnarla. Niente di quanto accade qui puo' toccare
       l'anello di controllo. */
    for (int i = 0; i < CFG_NQ; i++)
        r->d->qpos[i] = st->qpos[i];
    r->d->time = st->sim_time;
    mj_forward(r->m, r->d); /*Effettua cinematica diretta della posizione appena prelevata*/

    glfwGetFramebufferSize(r->win, &viewport.width, &viewport.height);
    mjv_updateScene(r->m, r->d, &r->opt, NULL, &r->cam, mjCAT_ALL, &r->scn);
    mjr_render(viewport, &r->scn, &r->con);

    char att_line[48];
    if (cmd->att_on) snprintf(att_line, sizeof(att_line), "ON   kp=%.2f", r->att_kp);
    else             snprintf(att_line, sizeof(att_line), "OFF");

    alarms_text(safe->alarms, r->banner, sizeof(r->banner));

    snprintf(r->hud, sizeof(r->hud),
             "sim time   %.2f s\ntrunk z    %.4f m\npos        %.2f  %.2f m\n"
             "cmd        vx %.2f  vy %.2f  wz %.2f\ngait       %s%s\n"
             "attidute    %s\nsafety     %s%s%s\nreal-time  %.3f",
             st->sim_time, st->base_pos[2], st->base_pos[0], st->base_pos[1],
             cmd->vx, cmd->vy, cmd->wz,
             cmd->gait_on ? "TROT" : "stand",
             cmd->fault_gait ? "  [thread killed]" : "",
             att_line,
             mode_text(safe->mode),
             r->banner[0] ? "  " : "", r->banner,
             rt_factor);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, r->hud, NULL, &r->con);

    /* Una scritta abbastanza grande da notarsi mentre si guarda il robot e
       non il testo: tutto il senso del watchdog e' che agisce proprio quando
       non stai leggendo i numeri. */
    if (safe->mode != SAFE_OK) {
        char line[128];
        snprintf(line, sizeof(line), "WATCHDOG  %s   %s",
                 mode_text(safe->mode), r->banner);
        mjr_text(mjFONT_BIG, line, &r->con, 1.00f, 0.02f, 1.0f, 0.25f, 0.15f);
    }

    glfwSwapBuffers(r->win);
    glfwPollEvents();
}
