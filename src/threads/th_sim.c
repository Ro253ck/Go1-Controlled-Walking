#include "th_sim.h"

void *th_sim_run(void *arg)
{
    th_sim_ctx_t *c = (th_sim_ctx_t *)arg;

    robot_state_t st;
    imu_t         imu;
    joint_cmd_t   des;
    safety_t      safe;
    cmd_t         cmd;
    uint32_t      last_push = 0;

    rt_periodic_init(&c->per, CFG_SIM_PERIOD_NS, CFG_MAX_CATCHUP);

    while (atomic_load(c->running)) {
        rt_periodic_wait(&c->per);

        /* Il gait gira cinque volte piu' lentamente, quindi la maggior parte
           dei cicli riusa lo stesso riferimento. Ed e' corretto cosi': vince
           l'ultimo, e il termine di velocita' che viaggia accanto alla
           posizione evita che il PD insegua una scala a gradini. */
        bb_des_read(c->bb_des, &des);

        /* La decisione di supervisione e' presa da un thread a bassa
           priorita', ma viene eseguita qui, in quello a priorita' massima:
           per la sicurezza conta quanto in fretta cambia la coppia, non
           quanto in fretta e' stata presa la decisione. */
        bb_safe_read(c->bb_safe, &safe);

        bb_cmd_read(c->bb_cmd, &cmd);
        if (cmd.push != last_push) {
            last_push = cmd.push;
            sim_push(c->sim, 0.0, cmd.push_force, 0.0);
        }
        c->sim->gain_scale = (safe.mode == SAFE_STOP) ? 0.0 : 1.0;

        for (int i = 0; i < CFG_NUM_JOINTS; i++) {
            c->sim->q_des[i]  = des.q[i];
            c->sim->qd_des[i] = des.qd[i];
        }

        /* Fisica e controllo girano interamente su dati privati del thread. */
        sim_step(c->sim);
        sim_snapshot(c->sim, &st);
        sim_imu_snapshot(c->sim, &imu);

        /* L'unica scrittura condivisa del ciclo, ed e' una semplice copia:
           l'istantanea e' stata composta sopra, fuori da ogni sezione
           critica. */
        bb_state_write(c->bb_state, &st);
        bb_imu_write(c->bb_imu, &imu);

        rt_periodic_done(&c->per);
    }
    return NULL;
}