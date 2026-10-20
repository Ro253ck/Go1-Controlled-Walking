#include "th_gait.h"

#include <math.h>
#include <string.h>

/* Intervallo usato per derivare numericamente il riferimento. Abbastanza
   piccolo da essere accurato, abbastanza grande da restare lontano dalla
   cancellazione numerica. Le traiettorie sono C1 attraverso il confine
   appoggio/volo, quindi una differenza a cavallo di una transizione resta
   comunque significativa. */
#define GAIT_DIFF_DT 1.0e-3

static void solve_at(const gait_params_t *gp, double t,
                     const double v_cmd[3], const double v_meas[2],
                     double amplitude, const gait_attitude_t *att,
                     double q[CFG_NUM_JOINTS])
{
    feet_target_t feet;

    gait_feet_targets(gp, t, v_cmd, v_meas, amplitude, att, &feet);
    legs_ik_trunk(&feet, q);
}

void *th_gait_run(void *arg)
{
    th_gait_ctx_t *c = (th_gait_ctx_t *)arg;

    robot_state_t st;
    cmd_t         cmd;
    att_t         att;
    joint_cmd_t   des;

    double   amplitude = 0.0;

    memset(&des, 0, sizeof(des));
    rt_periodic_init(&c->per, CFG_GAIT_PERIOD_NS, CFG_MAX_CATCHUP);

    while (atomic_load(c->running)) {
        double q_now[CFG_NUM_JOINTS], q_next[CFG_NUM_JOINTS];
        double v_cmd[3], v_meas[2];

        rt_periodic_wait(&c->per);

        /*Lettura delle blackboard necessarie*/
        bb_state_read(c->bb_state, &st);
        bb_cmd_read(c->bb_cmd, &cmd);
        /*Se è presente anche l'attitude leggiamo anche quella*/
        if (c->bb_att) {
            bb_att_read(c->bb_att, &att);
        }

        /* L'andatura entra ed esce gradualmente invece di commutare: ad
           ampiezza zero i piedi restano fermi sui loro punti di riferimento,
           che e' esattamente la posizione eretta, quindi non serve un
           controllore separato per stare in piedi. */
        {
            const double step = ((double)CFG_GAIT_PERIOD_NS / (double)RT_NS_PER_SEC) / CFG_GAIT_RAMP;
            amplitude += cmd.gait_on ? step : -step;
            if (amplitude > 1.0) amplitude = 1.0;
            if (amplitude < 0.0) amplitude = 0.0;
        }

        c->params.height = cmd.height;

        v_cmd[0] = cmd.vx;
        v_cmd[1] = cmd.vy;
        v_cmd[2] = cmd.wz;
        /*La velocità viene presa direttamente dal ground truth*/
        v_meas[0] = st.base_vel[0];
        v_meas[1] = st.base_vel[1];

        /* Il gait gira sul tempo simulato, non su quello di parete: i
           riferimenti che produce devono allinearsi alla fisica che il thread
           sim fa avanzare, e i due orologi si separano appena un ciclo viene
           perso. */
        gait_attitude_t ga;
        const gait_attitude_t *gap = NULL;
        if (c->bb_att && cmd.att_on && att.seq > 0) {
            ga.roll  = att.roll;
            ga.pitch = att.pitch;
            ga.kp    = c->att_kp;
            gap = &ga;
        }

        solve_at(&c->params, st.sim_time, v_cmd, v_meas, amplitude, gap,
                 q_now);
        /*Calcoliamo l'andature anche con un piccolo drift per vedere come varia e calcolare la velcoità*/
        solve_at(&c->params, st.sim_time + GAIT_DIFF_DT, v_cmd, v_meas,
                 amplitude, gap, q_next);
        
        /*Scriviamo la blackboard destionation*/
        for (int i = 0; i < CFG_NUM_JOINTS; i++) {
            des.q[i]  = q_now[i];
            des.qd[i] = (q_next[i] - q_now[i]) / GAIT_DIFF_DT;
        }

        /* Se è stato premuto il tasto per simulare la canecellazione del thread non aggiorniamo la blackboard*/
        if (!cmd.fault_gait)
            bb_des_write(c->bb_des, &des);

        rt_periodic_done(&c->per);
    }
    return NULL;
}