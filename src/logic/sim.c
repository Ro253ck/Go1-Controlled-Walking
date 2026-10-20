#include "sim.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/*Ci da l'indice dei sensori*/
static int sensor_offset(const mjModel *m, const char *name)
{
    int id = mj_name2id(m, mjOBJ_SENSOR, name);
    return (id < 0) ? -1 : m->sensor_adr[id];
}

/*Setta i valori per il PD*/
void sim_set_gains(sim_t *s, double kp, double kd)
{
    for (int i = 0; i < CFG_NUM_JOINTS; i++) {
        s->kp[i] = kp;
        s->kd[i] = kd;
    }
}

int sim_init(sim_t *s, const char *model_path, char *err, int err_sz)
{
    memset(s, 0, sizeof(*s));

    /*Carica il modello XML*/
    s->m = mj_loadXML(model_path, NULL, err, err_sz);
    if (!s->m) return -1;

    /*Effetuiamo vari controlli per vedere se i parametri che descrivono il modello corrispondono*/
    if (s->m->nu != CFG_NUM_JOINTS) {
        snprintf(err, (size_t)err_sz, "expected %d actuators, model has %ld",
                 CFG_NUM_JOINTS, (long)s->m->nu);
        mj_deleteModel(s->m);
        s->m = NULL;
        return -1;
    }

    if (s->m->nq != CFG_NQ || s->m->nv != CFG_NV) {
        snprintf(err, (size_t)err_sz, "expected nq=%d nv=%d, model has nq=%ld nv=%ld",
                 CFG_NQ, CFG_NV, (long)s->m->nq, (long)s->m->nv);
        mj_deleteModel(s->m);
        s->m = NULL;
        return -1;
    }

    s->d = mj_makeData(s->m);
    if (!s->d) {
        snprintf(err, (size_t)err_sz, "mj_makeData failed");
        mj_deleteModel(s->m);
        s->m = NULL;
        return -1;
    }

    /*Chiediamo al modello dove stanno in qpos e qvel i giunti mossi dagli attuatori*/
    for (int i = 0; i < CFG_NUM_JOINTS; i++) {
        int jid = s->m->actuator_trnid[2 * i];
        s->qadr[i] = s->m->jnt_qposadr[jid];
        s->vadr[i] = s->m->jnt_dofadr[jid];
    }

    s->gain_scale = 1.0;
    s->trunk_body = mj_name2id(s->m, mjOBJ_BODY, "trunk");
    s->push_left  = 0;
    s->adr_gyro = sensor_offset(s->m, "imu_gyro");
    s->adr_acc  = sensor_offset(s->m, "imu_acc");

    sim_set_gains(s, 100.0, 0.0);
    return 0;
}


void sim_free(sim_t *s)
{
    if (s->d) mj_deleteData(s->d);
    if (s->m) mj_deleteModel(s->m);
    s->d = NULL;
    s->m = NULL;
}

/*Utilizzata per settare il robot nella posizone key_frame (posizione di inzio dove lui è perfettamente fermo e non inclinato)*/
int sim_reset_to_key(sim_t *s, const char *key_name)
{
    int k = mj_name2id(s->m, mjOBJ_KEY, key_name);
    if (k < 0) return -1;

    mj_resetDataKeyframe(s->m, s->d, k);

    /* Tiene la posa del keyframe: il bersaglio del PD parte dove parte il
       robot, cosi' il primo ciclo non chiede nessun movimento. */
    for (int i = 0; i < CFG_NUM_JOINTS; i++) {
        s->q_des[i]  = s->d->qpos[s->qadr[i]];
        s->qd_des[i] = 0.0;
    }
    mj_forward(s->m, s->d);
    return 0;
}


void sim_push(sim_t *s, double fx, double fy, double fz)
{
    s->push_f[0] = fx;
    s->push_f[1] = fy;
    s->push_f[2] = fz;
    s->push_left = CFG_PUSH_CYCLES;
}


void sim_step(sim_t *s)
{
    /* xfrc_applied resta impostato finche' non lo si azzera, quindi il
       conto alla rovescia deve riscriverlo a zero, non solo smettere di
       aggiornarlo. */
    if (s->trunk_body >= 0) {
        mjtNum *f = &s->d->xfrc_applied[6 * s->trunk_body];
        for (int i = 0; i < 6; i++){
            f[i] = 0.0;
        }
        if (s->push_left > 0) {
            for (int i = 0; i < 3; i++){
                 f[i] = s->push_f[i];
            }
            s->push_left--;
        }
    }

    for (int i = 0; i < CFG_NUM_JOINTS; i++) {
        /*Calcolo della coppia*/
        double q   = s->d->qpos[s->qadr[i]];
        double qd  = s->d->qvel[s->vadr[i]];
        double tau = s->gain_scale *
                     (s->kp[i] * (s->q_des[i] - q) + s->kd[i] * (s->qd_des[i] - qd));

        /* Controlliamo che il tau non vada mai ad infinito */
        if (!isfinite(tau)) {
            tau = 0.0;
            s->n_nonfinite++;
        }

        /* Controllo Saturazione */
        double lo = s->m->actuator_ctrlrange[2 * i];
        double hi = s->m->actuator_ctrlrange[2 * i + 1];
        if (tau < lo)      tau = lo;
        else if (tau > hi) tau = hi;

        s->d->ctrl[i] = tau;
    }

    /*Manda avanti la simulazione*/
    mj_step(s->m, s->d);
}

/*Riempi la struttura dati robot_state_t*/
void sim_snapshot(const sim_t *s, robot_state_t *out)
{
    out->seq      = 0;   /* assegnato dalla lavagna al momento della pubblicazione */
    out->sim_time = s->d->time;

    for (int i = 0; i < CFG_NQ; i++)
        out->qpos[i] = s->d->qpos[i];

    /* Disposizione del giunto libero: qpos[0..2] traslazione, qpos[3..6]
       quaternione, qvel[0..2] velocita' lineare in coordinate mondo. */
    for (int i = 0; i < 3; i++)
        out->base_pos[i] = s->d->qpos[i];
    for (int i = 0; i < 4; i++)
        out->base_quat[i] = s->d->qpos[3 + i];

    /* Pubblica la velocita' nel sistema corpo, che e' quello in cui vivono il
       comando e il gait. I due coincidono solo mentre il robot guarda nella
       direzione iniziale: lasciarla in coordinate mondo funziona fino alla
       prima curva e poi corrompe silenziosamente il termine di Raibert. */
    {
        mjtNum qinv[4], v_world[3];
        for (int i = 0; i < 3; i++) v_world[i] = s->d->qvel[i];
        mju_negQuat(qinv, &s->d->qpos[3]);
        mju_rotVecQuat(out->base_vel, v_world, qinv);
    }
}

/*Riempi la struttura dati imu_T*/
void sim_imu_snapshot(const sim_t *s, imu_t *out)
{
    out->seq      = 0;
    out->sim_time = s->d->time;

    for (int i = 0; i < 3; i++) {
        out->gyro[i] = (s->adr_gyro >= 0) ? s->d->sensordata[s->adr_gyro + i] : 0.0;
        out->acc[i]  = (s->adr_acc  >= 0) ? s->d->sensordata[s->adr_acc  + i] : 0.0;
    }
}