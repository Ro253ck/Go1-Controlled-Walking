#include "th_watchdog.h"

#include <math.h>
#include <string.h>

/* How many bits are set in the history: that is, for how many of the 32
   recorded cycles the condition has been true. */
static int count_ones(uint32_t v)
{
    int n = 0;
    while (v) {
        n += (int)(v & 1u);
        v >>= 1;
    }
    return n;
}

/* Records the condition in a window of 32 cycles and fires when it has
   stayed true for at least `m` of them. At 50 Hz the window is 640 ms. */
static int debounce(uint32_t *hist, int cond, int m)
{
    *hist = (*hist << 1) | (cond ? 1u : 0u);
    return count_ones(*hist) >= m;
}

void *th_watchdog_run(void *arg)
{
    th_watchdog_ctx_t *c = (th_watchdog_ctx_t *)arg;

    imu_t       imu;
    joint_cmd_t des;
    safety_t    safe;

    uint64_t prev_des = 0;
    int      miss_des = 0;

    /*It guarantees that once a fault has happened the robot stays on the ground whatever command is given.
      The simulation has to be restarted to make the robot move again. */
    uint32_t latched = 0;

    memset(&safe, 0, sizeof(safe));
    rt_periodic_init(&c->per, CFG_WATCHDOG_PERIOD_NS, CFG_MAX_CATCHUP);

    while (atomic_load(c->running)) {
        uint32_t alarms = 0;

        rt_periodic_wait(&c->per);

        bb_imu_read(c->bb_imu, &imu);
        bb_des_read(c->bb_des, &des);

        /* We use the accelerometer to obtain the tilt*/
        {
            const double n = sqrt(imu.acc[0]*imu.acc[0] + imu.acc[1]*imu.acc[1]
                                + imu.acc[2]*imu.acc[2]);
            if (n > 1.0 && fabs(n - ATT_GRAVITY) <= CFG_ATT_GATE) {
                double cz = imu.acc[2] / n;
                if (cz >  1.0) cz =  1.0;
                if (cz < -1.0) cz = -1.0;
                const double tilt = acos(cz);

                if (debounce(&c->tilt_hist, tilt > CFG_WD_TILT_MAX, CFG_WD_DEBOUNCE))
                    alarms |= WD_TILT;
            }
        }

        /* Liveness checks */
        miss_des = (des.seq == prev_des) ? miss_des + 1 : 0;
        prev_des = des.seq;

        if (miss_des > CFG_WD_LIVENESS) alarms |= WD_DEAD_GAIT;

        /* Severity: only one possible response, remove the torque. */
        latched |= alarms;
        safe.mode  = latched ? SAFE_STOP : SAFE_OK;
        safe.alarms = latched;

        bb_safe_write(c->bb_safe, &safe);

        rt_periodic_done(&c->per);
    }
    return NULL;
}