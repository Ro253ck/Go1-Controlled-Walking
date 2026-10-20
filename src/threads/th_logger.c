#include "th_logger.h"
#include "attitude.h"

#include <math.h>

void *th_logger_run(void *arg)
{
    th_logger_ctx_t *c = (th_logger_ctx_t *)arg;

    robot_state_t st;
    att_t         att;

    rt_acc_reset(&c->write_ns);

    c->fp = fopen(c->path, "w");
    if (!c->fp) return NULL;

    fprintf(c->fp,"t,roll_true,pitch_true,roll_est,pitch_est\n");

    rt_periodic_init(&c->per, CFG_LOGGER_PERIOD_NS, CFG_MAX_CATCHUP);

    while (atomic_load(c->running)) {
        rt_periodic_wait(&c->per);

        /* Leggo le blackboard di interesse */
        bb_state_read(c->bb_state, &st);
        bb_att_read(c->bb_att, &att);

        if (st.seq > 0) {
            double rt, pt;
            quaternion_to_rpy(st.base_quat, &rt, &pt, NULL);

            const int64_t w0 = rt_now_ns();
            fprintf(c->fp,
                    "%.4f,%.5f,%.5f,%.5f,%.5f\n",
                    st.sim_time, rt, pt, att.roll, att.pitch);
            rt_acc_add(&c->write_ns, rt_now_ns() - w0);
        }

        rt_periodic_done(&c->per);
    }

    fclose(c->fp);
    c->fp = NULL;
    return NULL;
}