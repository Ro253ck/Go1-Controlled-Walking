#ifndef RENDER_H
#define RENDER_H

/* Visualizzazione best-effort. Gira sul thread principale perche' GLFW e
   OpenGL vogliono un unico thread proprietario, e resta su SCHED_OTHER
   cosi' non puo' mai rubare tempo all'anello di controllo.

   Non tocca mai i mjData vivi: ne tiene una copia privata, vi carica la posa
   pubblicata sulla lavagna e ricalcola solo quello che serve a disegnare.
   Bloccare i dati vivi terrebbe il mutex per un intero fotogramma. */

#include "state.h"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

typedef struct {
    GLFWwindow   *win;
    const mjModel *m;   /*In sola lettura*/
    mjData       *d;      /* privati, solo per il disegno */
    mjvCamera     cam;
    mjvOption     opt;
    mjvScene      scn;
    mjrContext    con;

    int    btn_left, btn_middle, btn_right; /*Stato pulsanit del mouse*/
    double lastx, lasty; /*Ultima posizione del cursore*/

    char   hud[384];
    double att_kp;      /* mostrato sull'HUD, impostato da main dopo l'init */
    char   banner[96];
} render_t;

/* Ritorna 0 se riesce, -1 quando non si e' potuta creare nessuna finestra
   (esecuzione senza display). */
int  render_init(render_t *r, const mjModel *m);
void render_free(render_t *r);

int  render_should_close(render_t *r);
void render_frame(render_t *r, const robot_state_t *st, const cmd_t *cmd,
                  const safety_t *safe, double rt_factor);

#endif /* RENDER_H */
