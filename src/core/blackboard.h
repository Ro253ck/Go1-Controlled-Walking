#ifndef BLACKBOARD_H
#define BLACKBOARD_H

#include "rt_time.h"
#include "state.h"
#include <pthread.h>
#include <stdio.h>

typedef struct {
    pthread_mutex_t mtx;
    rt_acc_t        wait;   /* tempo di acquisizione, caso peggiore su tutti i chiamanti */
    robot_state_t   data;
} bb_state_t;

void bb_state_init(bb_state_t *bb);
void bb_state_destroy(bb_state_t *bb);

/* Scritta solo da th_sim. Assegna il numero di sequenza da se', cosi' il
   chiamante non deve ricordarsi di incrementarlo. */
void bb_state_write(bb_state_t *bb, const robot_state_t *in);

/* Lettura bloccante, per i thread che hanno bisogno del dato per lavorare. */
void bb_state_read(bb_state_t *bb, robot_state_t *out);

/* Lettura non bloccante per i consumatori best-effort come il renderer: con
   la semantica "vince l'ultimo", saltare un ciclo occupato non costa niente e
   garantisce che il chiamante non attenda mai. Ritorna 1 se riesce, 0 se la
   lavagna era occupata. */
int bb_state_tryread(bb_state_t *bb, robot_state_t *out);

/* comando dell'operatore: scritto da th_command, letto da th_gait */
typedef struct {
    pthread_mutex_t mtx;
    rt_acc_t        wait;
    cmd_t           data;
} bb_cmd_t;

void bb_cmd_init(bb_cmd_t *bb, const cmd_t *initial);
void bb_cmd_destroy(bb_cmd_t *bb);
void bb_cmd_write(bb_cmd_t *bb, const cmd_t *in);
void bb_cmd_read(bb_cmd_t *bb, cmd_t *out);

/*  riferimenti dei giunti: scritti da th_gait, letti da th_sim  */
typedef struct {
    pthread_mutex_t mtx;
    rt_acc_t        wait;
    joint_cmd_t     data;
} bb_des_t;

void bb_des_init(bb_des_t *bb, const joint_cmd_t *initial);
void bb_des_destroy(bb_des_t *bb);
void bb_des_write(bb_des_t *bb, const joint_cmd_t *in);
void bb_des_read(bb_des_t *bb, joint_cmd_t *out);

/*misure inerziali: scritte da th_sim  */
typedef struct {
    pthread_mutex_t mtx;
    rt_acc_t        wait;
    imu_t           data;
} bb_imu_t;

void bb_imu_init(bb_imu_t *bb);
void bb_imu_destroy(bb_imu_t *bb);
void bb_imu_write(bb_imu_t *bb, const imu_t *in);
void bb_imu_read(bb_imu_t *bb, imu_t *out);

/*  assetto stimato: scritto da th_estimator */

typedef struct {
    pthread_mutex_t mtx;
    rt_acc_t        wait;
    att_t           data;
} bb_att_t;

void bb_att_init(bb_att_t *bb, const att_t *initial);
void bb_att_destroy(bb_att_t *bb);
void bb_att_write(bb_att_t *bb, const att_t *in);
void bb_att_read(bb_att_t *bb, att_t *out);

/* stato di sicurezza: scritto da th_watchdog  */
typedef struct {
    pthread_mutex_t mtx;
    rt_acc_t        wait;
    safety_t        data;
} bb_safe_t;

void bb_safe_init(bb_safe_t *bb, const safety_t *initial);
void bb_safe_destroy(bb_safe_t *bb);
void bb_safe_write(bb_safe_t *bb, const safety_t *in);
void bb_safe_read(bb_safe_t *bb, safety_t *out);

#endif /* BLACKBOARD_H */