#include "sim.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/*Gives us the index of the sensors*/
static int sensor_offset(const mjModel *m, const char *name)
{
    int id = mj_name2id(m, mjOBJ_SENSOR, name);
    return (id < 0) ? -1 : m->sensor_adr[id];
}

/*Sets the values for the PD*/
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

    /*Loads the XML model*/
    s->m = mj_loadXML(model_path, NULL, err, err_sz);
    if (!s->m) return -1;

    /*We run several checks to see whether the parameters describing the model match*/
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

    /*We ask the model where, in qpos and qvel, the joints driven by the actuators are*/
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

    sim_set_gains(s, CFG_KP_DEFAULT, CFG_KD_DEFAULT);
    return 0;
}


void sim_free(sim_t *s)
{
    if (s->d) mj_deleteData(s->d);
    if (s->m) mj_deleteModel(s->m);
    s->d = NULL;
    s->m = NULL;
}

/*Used to set the robot into the key_frame pose (starting pose where it is perfectly still and not tilted)*/
int sim_reset_to_key(sim_t *s, const char *key_name)
{
    int k = mj_name2id(s->m, mjOBJ_KEY, key_name);
    if (k < 0) return -1;

    mj_resetDataKeyframe(s->m, s->d, k);

    /* Holds the keyframe pose: the PD target starts where the
       robot starts, so the first cycle asks for no movement. */
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
    /* xfrc_applied stays set until it is cleared, so the
       countdown must write it back to zero, not just stop
       updating it. */
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
        /*Computation of the torque*/
        double q   = s->d->qpos[s->qadr[i]];
        double qd  = s->d->qvel[s->vadr[i]];
        double tau = s->gain_scale *
                     (s->kp[i] * (s->q_des[i] - q) + s->kd[i] * (s->qd_des[i] - qd));

        /* We check that tau never goes to infinity */
        if (!isfinite(tau)) {
            tau = 0.0;
            s->n_nonfinite++;
        }

        /* Saturation check */
        double lo = s->m->actuator_ctrlrange[2 * i];
        double hi = s->m->actuator_ctrlrange[2 * i + 1];
        if (tau < lo)      tau = lo;
        else if (tau > hi) tau = hi;

        s->d->ctrl[i] = tau;
    }

    /*Advances the simulation*/
    mj_step(s->m, s->d);
}

/*Fills in the robot_state_t data structure*/
void sim_snapshot(const sim_t *s, robot_state_t *out)
{
    out->seq      = 0;   /* assigned by the blackboard at publication time */
    out->sim_time = s->d->time;

    for (int i = 0; i < CFG_NQ; i++)
        out->qpos[i] = s->d->qpos[i];

    /* Free joint layout: qpos[0..2] translation, qpos[3..6]
       quaternion, qvel[0..2] linear velocity in world coordinates. */
    for (int i = 0; i < 3; i++)
        out->base_pos[i] = s->d->qpos[i];
    for (int i = 0; i < 4; i++)
        out->base_quat[i] = s->d->qpos[3 + i];

    /* Publishes the velocity in the body frame, which is the one the
       command and the gait live in. The two coincide only while the robot faces the
       initial direction: leaving it in world coordinates works until the
       first turn and then silently corrupts the Raibert term. */
    {
        mjtNum qinv[4], v_world[3];
        for (int i = 0; i < 3; i++) v_world[i] = s->d->qvel[i];
        mju_negQuat(qinv, &s->d->qpos[3]);
        mju_rotVecQuat(out->base_vel, v_world, qinv);
    }
}

/*Fills in the imu_t data structure*/
void sim_imu_snapshot(const sim_t *s, imu_t *out)
{
    out->seq      = 0;
    out->sim_time = s->d->time;

    for (int i = 0; i < 3; i++) {
        out->gyro[i] = (s->adr_gyro >= 0) ? s->d->sensordata[s->adr_gyro + i] : 0.0;
        out->acc[i]  = (s->adr_acc  >= 0) ? s->d->sensordata[s->adr_acc  + i] : 0.0;
    }
}