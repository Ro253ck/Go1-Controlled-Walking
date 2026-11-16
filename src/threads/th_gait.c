#include "th_gait.h"

#include <math.h>
#include <string.h>

/* Interval used to differentiate the reference numerically. Small
   enough to be accurate, large enough to stay away from numerical
   cancellation. The trajectories are C1 across the stance/swing
   boundary, so a difference straddling a transition is still
   meaningful. */
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

        /*Reading of the required blackboards*/
        bb_state_read(c->bb_state, &st);
        bb_cmd_read(c->bb_cmd, &cmd);
        /*If the attitude is wired up as well, we read that too*/
        if (c->bb_att) {
            bb_att_read(c->bb_att, &att);
        }

        /* The gait ramps in and out instead of switching: at
           zero amplitude the feet stay put on their reference points,
           which is exactly the standing pose, so no separate
           controller is needed in order to stand. */
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
        /*The velocity is taken directly from the ground truth*/
        v_meas[0] = st.base_vel[0];
        v_meas[1] = st.base_vel[1];

        /* The gait runs on simulated time, not on wall-clock time: the
           references it produces must line up with the physics the sim
           thread advances, and the two clocks separate as soon as a cycle is
           missed. */
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
        /*We also compute the gait with a small time offset, to see how it varies and work out the velocity*/
        solve_at(&c->params, st.sim_time + GAIT_DIFF_DT, v_cmd, v_meas,
                 amplitude, gap, q_next);
        
        /*We write the destination blackboard*/
        for (int i = 0; i < CFG_NUM_JOINTS; i++) {
            des.q[i]  = q_now[i];
            des.qd[i] = (q_next[i] - q_now[i]) / GAIT_DIFF_DT;
        }

        /* If the key that simulates killing the thread has been pressed, we do not update the blackboard*/
        if (!cmd.fault_gait)
            bb_des_write(c->bb_des, &des);

        rt_periodic_done(&c->per);
    }
    return NULL;
}