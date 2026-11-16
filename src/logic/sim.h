#ifndef SIM_H
#define SIM_H

#include "config.h"
#include "state.h"

#include <mujoco/mujoco.h>
#include <stdint.h>

typedef struct {
    /*We define the main structures used to interact with MuJoCo*/
    mjModel *m;
    mjData  *d;

    /* Actuator i drives the joint at qadr[i]/vadr[i]. Resolved from the model
       at initialisation, so the C index convention coincides by
       construction with the actuator order and never depends on the
       layout of qpos. */
    int qadr[CFG_NUM_JOINTS];
    int vadr[CFG_NUM_JOINTS];

    /* PD law, per joint */
    double kp[CFG_NUM_JOINTS];
    double kd[CFG_NUM_JOINTS];
    double q_des[CFG_NUM_JOINTS];
    double qd_des[CFG_NUM_JOINTS];

    /* Position of the sensors inside d->sensordata, -1 if the sensor is missing */
    int adr_gyro;
    int adr_acc;

    /* Scaled by the safety state: 1 normal, 0 to stop. */
    double gain_scale;

    /* External shove, applied for a limited number of cycles and then cleared. */
    int    trunk_body;
    int    push_left;
    double push_f[3];

    /* Diagnostics */
    uint64_t n_nonfinite;   /* torques replaced with zero */
} sim_t;

int  sim_init(sim_t *s, const char *model_path, char *err, int err_sz);
void sim_free(sim_t *s);

/* Loads the keyframe by name and makes it the PD target. Returns 0 on success. */
int  sim_reset_to_key(sim_t *s, const char *key_name);

void sim_set_gains(sim_t *s, double kp, double kd);

/* One control cycle: PD from the current state towards q_des, then advance the physics. */
void sim_step(sim_t *s);

/* Schedules an external force on the trunk for CFG_PUSH_CYCLES cycles. */
void sim_push(sim_t *s, double fx, double fy, double fz);

/* Composes the publishable view of the state. Called outside every critical
   section: the write to the blackboard then becomes a plain copy. */
void sim_snapshot(const sim_t *s, robot_state_t *out);

/* Raw IMU exactly as the sensors report it, with no filtering. */
void sim_imu_snapshot(const sim_t *s, imu_t *out);

#endif 