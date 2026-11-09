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

        /* Leggiamo struttura dati destination */
        bb_des_read(c->bb_des, &des);

        /* Leggiamo struttura dati safety */
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

        /* Aggiorniamo blackboard*/
        bb_state_write(c->bb_state, &st);
        bb_imu_write(c->bb_imu, &imu);

        rt_periodic_done(&c->per);
    }
    return NULL;
}