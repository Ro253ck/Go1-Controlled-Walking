#include "blackboard.h"
#include "rt_sched.h"

#include <string.h>

/*Stato*/
void bb_state_init(bb_state_t *bb)
{
    memset(&bb->data, 0, sizeof(bb->data));
    rt_acc_reset(&bb->wait);
    rt_mutex_init_pi(&bb->mtx);
}

void bb_state_destroy(bb_state_t *bb)
{
    pthread_mutex_destroy(&bb->mtx);
}

void bb_state_write(bb_state_t *bb, const robot_state_t *in)
{
    int64_t t0 = rt_now_ns();
    pthread_mutex_lock(&bb->mtx);
    int64_t t1 = rt_now_ns();

    uint64_t next = bb->data.seq + 1;
    bb->data      = *in;
    bb->data.seq  = next;

    rt_acc_add(&bb->wait, t1 - t0);
    pthread_mutex_unlock(&bb->mtx);
}

void bb_state_read(bb_state_t *bb, robot_state_t *out)
{
    int64_t t0 = rt_now_ns();
    pthread_mutex_lock(&bb->mtx);
    int64_t t1 = rt_now_ns();

    *out = bb->data;

    rt_acc_add(&bb->wait, t1 - t0);
    pthread_mutex_unlock(&bb->mtx);
}

int bb_state_tryread(bb_state_t *bb, robot_state_t *out)
{
    if (pthread_mutex_trylock(&bb->mtx) != 0)
        return 0;

    *out = bb->data;
    pthread_mutex_unlock(&bb->mtx);
    return 1;
}

/*Comandi operatore */

void bb_cmd_init(bb_cmd_t *bb, const cmd_t *initial)
{
    bb->data = *initial;
    bb->data.seq = 0;
    rt_acc_reset(&bb->wait);
    rt_mutex_init_pi(&bb->mtx);
}

void bb_cmd_destroy(bb_cmd_t *bb) { pthread_mutex_destroy(&bb->mtx); }

void bb_cmd_write(bb_cmd_t *bb, const cmd_t *in)
{
    int64_t t0 = rt_now_ns();
    pthread_mutex_lock(&bb->mtx);
    int64_t t1 = rt_now_ns();

    uint64_t next = bb->data.seq + 1;
    bb->data      = *in;
    bb->data.seq  = next;

    rt_acc_add(&bb->wait, t1 - t0);
    pthread_mutex_unlock(&bb->mtx);
}

void bb_cmd_read(bb_cmd_t *bb, cmd_t *out)
{
    int64_t t0 = rt_now_ns();
    pthread_mutex_lock(&bb->mtx);
    int64_t t1 = rt_now_ns();

    *out = bb->data;

    rt_acc_add(&bb->wait, t1 - t0);
    pthread_mutex_unlock(&bb->mtx);
}

/* joint ref*/

void bb_des_init(bb_des_t *bb, const joint_cmd_t *initial)
{
    bb->data = *initial;
    bb->data.seq = 0;
    rt_acc_reset(&bb->wait);
    rt_mutex_init_pi(&bb->mtx);
}

void bb_des_destroy(bb_des_t *bb) { pthread_mutex_destroy(&bb->mtx); }

void bb_des_write(bb_des_t *bb, const joint_cmd_t *in)
{
    int64_t t0 = rt_now_ns();
    pthread_mutex_lock(&bb->mtx);
    int64_t t1 = rt_now_ns();

    uint64_t next = bb->data.seq + 1;
    bb->data      = *in;
    bb->data.seq  = next;

    rt_acc_add(&bb->wait, t1 - t0);
    pthread_mutex_unlock(&bb->mtx);
}

void bb_des_read(bb_des_t *bb, joint_cmd_t *out)
{
    int64_t t0 = rt_now_ns();
    pthread_mutex_lock(&bb->mtx);
    int64_t t1 = rt_now_ns();

    *out = bb->data;

    rt_acc_add(&bb->wait, t1 - t0);
    pthread_mutex_unlock(&bb->mtx);
}

/*IMU*/

void bb_imu_init(bb_imu_t *bb)
{
    memset(&bb->data, 0, sizeof(bb->data));
    rt_acc_reset(&bb->wait);
    rt_mutex_init_pi(&bb->mtx);
}

void bb_imu_destroy(bb_imu_t *bb) { pthread_mutex_destroy(&bb->mtx); }

void bb_imu_write(bb_imu_t *bb, const imu_t *in)
{
    int64_t t0 = rt_now_ns();
    pthread_mutex_lock(&bb->mtx);
    int64_t t1 = rt_now_ns();

    uint64_t next = bb->data.seq + 1;
    bb->data      = *in;
    bb->data.seq  = next;

    rt_acc_add(&bb->wait, t1 - t0);
    pthread_mutex_unlock(&bb->mtx);
}

void bb_imu_read(bb_imu_t *bb, imu_t *out)
{
    int64_t t0 = rt_now_ns();
    pthread_mutex_lock(&bb->mtx);
    int64_t t1 = rt_now_ns();

    *out = bb->data;

    rt_acc_add(&bb->wait, t1 - t0);
    pthread_mutex_unlock(&bb->mtx);
}

/*Attidute */

void bb_att_init(bb_att_t *bb, const att_t *initial)
{
    bb->data = *initial;
    bb->data.seq = 0;
    rt_acc_reset(&bb->wait);
    rt_mutex_init_pi(&bb->mtx);
}

void bb_att_destroy(bb_att_t *bb) { pthread_mutex_destroy(&bb->mtx); }

void bb_att_write(bb_att_t *bb, const att_t *in)
{
    int64_t t0 = rt_now_ns();
    pthread_mutex_lock(&bb->mtx);
    int64_t t1 = rt_now_ns();

    uint64_t next = bb->data.seq + 1;
    bb->data      = *in;
    bb->data.seq  = next;

    rt_acc_add(&bb->wait, t1 - t0);
    pthread_mutex_unlock(&bb->mtx);
}

void bb_att_read(bb_att_t *bb, att_t *out)
{
    int64_t t0 = rt_now_ns();
    pthread_mutex_lock(&bb->mtx);
    int64_t t1 = rt_now_ns();

    *out = bb->data;

    rt_acc_add(&bb->wait, t1 - t0);
    pthread_mutex_unlock(&bb->mtx);
}

/* watchdog*/

void bb_safe_init(bb_safe_t *bb, const safety_t *initial)
{
    bb->data = *initial;
    bb->data.seq = 0;
    rt_acc_reset(&bb->wait);
    rt_mutex_init_pi(&bb->mtx);
}

void bb_safe_destroy(bb_safe_t *bb) { pthread_mutex_destroy(&bb->mtx); }

void bb_safe_write(bb_safe_t *bb, const safety_t *in)
{
    int64_t t0 = rt_now_ns();
    pthread_mutex_lock(&bb->mtx);
    int64_t t1 = rt_now_ns();

    uint64_t next = bb->data.seq + 1;
    bb->data      = *in;
    bb->data.seq  = next;

    rt_acc_add(&bb->wait, t1 - t0);
    pthread_mutex_unlock(&bb->mtx);
}

void bb_safe_read(bb_safe_t *bb, safety_t *out)
{
    int64_t t0 = rt_now_ns();
    pthread_mutex_lock(&bb->mtx);
    int64_t t1 = rt_now_ns();

    *out = bb->data;

    rt_acc_add(&bb->wait, t1 - t0);
    pthread_mutex_unlock(&bb->mtx);
}
