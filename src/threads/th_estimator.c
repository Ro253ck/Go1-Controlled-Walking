#include "th_estimator.h"

#include <math.h>
#include <string.h>

void *th_estimator_run(void *arg)
{
    th_estimator_ctx_t *c = (th_estimator_ctx_t *)arg;

    imu_t imu;
    att_t out;

    uint64_t last_imu_seq = 0;
    double   last_time    = -1.0;

    memset(&out, 0, sizeof(out));
    rt_periodic_init(&c->per, CFG_ESTIMATOR_PERIOD_NS, CFG_MAX_CATCHUP);

    while (atomic_load(c->running)) {
        rt_periodic_wait(&c->per);

        bb_imu_read(c->bb_imu, &imu);

        if (imu.seq == last_imu_seq) {
            /*Niente di nuovo da integrare*/
            rt_periodic_done(&c->per);
            continue;
        }
        last_imu_seq = imu.seq;

        if (!c->filter.seeded) {
            /* Si parte allineati alla gravita' invece di convergere da un
               assetto arbitrario. Il seeding rifiuta i campioni che non
               somigliano alla gravita', quindi qui si riprova finche' il
               robot non si e' assestato. */
            filter_seed(&c->filter, imu.acc);
            last_time = imu.sim_time;
            c->n_seed_tries++;

            if (!c->filter.seeded && c->n_seed_tries > CFG_ATT_SEED_TRIES) {
                /* Non fidarsi mai di un'attesa illimitata dentro un task
                   periodico: si accetta l'ipotesi di partenza orizzontale e
                   si lascia che sia la correzione a fare il resto. */
                c->filter.seeded = 1;
            }
            rt_periodic_done(&c->per);
            continue;
        }

        {
            /* Tempo simulato, come nel gait: l'intervallo che conta e' quello
               di cui e' avanzata la fisica, non quello impiegato dallo
               scheduler. */
            const double dt = imu.sim_time - last_time;
            last_time = imu.sim_time;

            if (dt > 0.0 && dt < 0.5)
                filter_update(&c->filter, imu.gyro, imu.acc, dt);
        }

        for (int i = 0; i < 4; i++) out.q[i] = c->filter.q[i];
        quaternion_to_rpy(out.q, &out.roll, &out.pitch, &out.yaw);

        bb_att_write(c->bb_att, &out);

        rt_periodic_done(&c->per);
    }
    return NULL;
}