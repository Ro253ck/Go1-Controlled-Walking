#ifndef CONFIG_H
#define CONFIG_H

#include "rt_time.h"

/*Thread periods */
#define CFG_SIM_PERIOD_NS       (2   * RT_NS_PER_MS)   /* 500 Hz */
#define CFG_ESTIMATOR_PERIOD_NS (4   * RT_NS_PER_MS)   /* 250 Hz */
#define CFG_GAIT_PERIOD_NS      (10  * RT_NS_PER_MS)   /* 100 Hz */
#define CFG_WATCHDOG_PERIOD_NS  (20  * RT_NS_PER_MS)   /*  50 Hz */
#define CFG_COMMAND_PERIOD_NS   (20  * RT_NS_PER_MS)   /*  50 Hz */
#define CFG_LOGGER_PERIOD_NS    (20  * RT_NS_PER_MS)   /*  50 Hz*/
#define CFG_RENDER_PERIOD_NS    (33  * RT_NS_PER_MS)   /*  30 Hz, best effort */

/* Missed releases replayed back to back before resynchronising: bounds the catch-up burst. */
#define CFG_MAX_CATCHUP         2

/* SCHED_FIFO levels, Rate Monotonic: shorter period, higher priority. */
#define CFG_PRIO_SIM            90
#define CFG_PRIO_ESTIMATOR      80
#define CFG_PRIO_GAIT           70
#define CFG_PRIO_WATCHDOG       60
#define CFG_PRIO_COMMAND        40
#define CFG_PRIO_LOGGER         20

/* Path relative to the project root, which is where the binary must be launched from. */

#define CFG_MODEL_PATH          "models/go1/scene_torque.xml"

#define CFG_NUM_LEGS            4
#define CFG_JOINTS_PER_LEG      3
#define CFG_NUM_JOINTS          (CFG_NUM_LEGS * CFG_JOINTS_PER_LEG)

/* Sizes of the generalised coordinates, checked against the model at startup. */
#define CFG_NQ                  19   /* free joint 3+4, then the 12 leg joints */
#define CFG_NV                  18   /* free joint 3+3, then the 12 leg joints */

/* Index convention used by every array: leg*3 + joint, as in go1_torque.xml. */
enum { LEG_FR = 0, LEG_FL = 1, LEG_RR = 2, LEG_RL = 3 };
enum { JNT_ABAD = 0, JNT_HIP = 1, JNT_KNEE = 2 };

/* kd at zero: the model has its own joint damping. High kp to limit the sag. */
#define CFG_KP_DEFAULT      300.0
#define CFG_KD_DEFAULT        0.0

/* Trot. A duty above 0.5 leaves an interval with four feet on the ground. */

#define CFG_GAIT_CYCLE      0.40    /* T_g, one full stance plus swing, s */
#define CFG_GAIT_DUTY       0.55    /* beta, fraction of the cycle spent in stance */
#define CFG_BODY_HEIGHT     0.26    /* h0, m */
#define CFG_FOOT_CLEARANCE  0.05    /* H, swing apex above the ground, m */
#define CFG_RAIBERT_K       0.15
#define CFG_RAIBERT_MAX     0.06
//#define CFG_GAIT_RAMP       0.50    /* s to ramp the gait in or out */
#define CFG_GAIT_RAMP       1.00

/* Complementary filter*/

#define ATT_GRAVITY         9.81    /* m/s^2; the value used by the MuJoCo model */
#define CFG_ATT_KP          1.0     /*How much we trust the accelerometer for the correction*/
#define CFG_ATT_SEED_TRIES  500     /* ~2 s before seeding anyway */
#define CFG_ATT_GATE        2.0     /* m/s^2 of deviation from g beyond which we ignore */

/* Attitude correction: lengthens the legs on the low side, pulls them in on the high side. */
#define CFG_ATT_KP_LEVEL    0.5

/* Steps per accepted key event: one per cycle, so holding a key ramps at a known rate. */
#define CFG_VX_MAX   0.60
#define CFG_VY_MAX   0.30
#define CFG_WZ_MAX   1.00
#define CFG_VX_STEP  0.05
#define CFG_VY_STEP  0.05
#define CFG_WZ_STEP  0.10
#define CFG_H_MIN    0.20
#define CFG_H_MAX    0.32
#define CFG_H_STEP   0.01

/* Watchdog thresholds: the condition must hold for CFG_WD_DEBOUNCE cycles over a
   sliding window of 32, and the distance from every threshold is logged each cycle. */

#define CFG_WD_TILT_MAX     1.57    /* rad, 90 degrees: trunk on its side, from there a
                                       quadruped cannot get back up */
#define CFG_WD_DEBOUNCE     20      /* over a window of 32 cycles, we tolerate at most 20 dangerous readings*/
#define CFG_WD_LIVENESS     10      /* watchdog cycles with no new publications */

#define CFG_LOG_PATH        "logs/run.csv"

/* Two deliberate disturbances, so supervision can be demonstrated instead of argued. */

#define CFG_PUSH_FORCE      180.0   /*applied force in N */
#define CFG_PUSH_CYCLES     30      /* sim cycles, 60 ms */

#endif /* CONFIG_H */
