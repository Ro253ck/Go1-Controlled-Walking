#ifndef SIM_H
#define SIM_H

#include "config.h"
#include "state.h"

#include <mujoco/mujoco.h>
#include <stdint.h>

typedef struct {
    /*Definiamo le strutture principali per interagire con mujoco*/
    mjModel *m;
    mjData  *d;

    /* L'attuatore i muove il giunto qadr[i]/vadr[i]. Risolti dal modello
       all'inizializzazione, cosi' la convenzione di indice del C coincide per
       costruzione con l'ordine degli attuatori e non dipende mai dalla
       disposizione di qpos. */
    int qadr[CFG_NUM_JOINTS];
    int vadr[CFG_NUM_JOINTS];

    /* Legge PD, per giunto */
    double kp[CFG_NUM_JOINTS];
    double kd[CFG_NUM_JOINTS];
    double q_des[CFG_NUM_JOINTS];
    double qd_des[CFG_NUM_JOINTS];

    /* Posizione dei sensori dentro d->sensordata, -1 se il sensore manca */
    int adr_gyro;
    int adr_acc;

    /* Scalato dallo stato di sicurezza: 1 normale,  0 per fermare. */
    double gain_scale;

    /* Spinta esterna, applicata per un numero limitato di cicli e poi azzerata. */
    int    trunk_body;
    int    push_left;
    double push_f[3];

    /* Diagnostica */
    uint64_t n_nonfinite;   /* coppie sostituite con zero */
} sim_t;

int  sim_init(sim_t *s, const char *model_path, char *err, int err_sz);
void sim_free(sim_t *s);

/* Carica il keyframe per nome e lo rende il bersaglio del PD. Ritorna 0 se riesce. */
int  sim_reset_to_key(sim_t *s, const char *key_name);

void sim_set_gains(sim_t *s, double kp, double kd);

/* Un ciclo di controllo: PD dallo stato attuale verso q_des, poi avanza la fisica. */
void sim_step(sim_t *s);

/* Programma una forza esterna sul tronco per CFG_PUSH_CYCLES cicli. */
void sim_push(sim_t *s, double fx, double fy, double fz);

/* Compone la vista pubblicabile dello stato. Chiamata fuori da ogni sezione
   critica: la scrittura sulla lavagna diventa cosi' una semplice copia. */
void sim_snapshot(const sim_t *s, robot_state_t *out);

/* IMU grezza come la riportano i sensori, senza nessun filtraggio. */
void sim_imu_snapshot(const sim_t *s, imu_t *out);

#endif 