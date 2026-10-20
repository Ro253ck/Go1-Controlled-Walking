#ifndef STATE_H
#define STATE_H
 
#include "config.h"
#include <stdint.h>

typedef struct {
    uint64_t seq;          /* incrementato dalla lavagna a ogni pubblicazione */
    double   sim_time;     /* orologio di MuJoCo*/

    double   qpos[CFG_NQ]; /* posizione generalizzata completa, per il renderer */
    double   base_pos[3];  /* verità di simulazione, solo telemetria */
    double   base_quat[4]; /* verità di simulazione, solo riferimento per l'estimatore */
    double   base_vel[3];  /* verità di simulazione, utilizzata il termine di Raibert */
} robot_state_t;

/* Misure inerziali grezze. Scritte da th_sim, lette da th_estimator.*/
typedef struct {
    uint64_t seq;
    double   sim_time;
    double   gyro[3];   /* rad/s, sistema corpo */
    double   acc[3];    /* m/s^2, forza specifica, sistema corpo */
} imu_t;

/* Assetto stimato. Scritto da th_estimator, letto da th_gait. */
typedef struct {
    uint64_t seq;
    double   q[4];              /* da corpo a mondo */
    double   roll, pitch, yaw;
} att_t;

/* Stato di sicurezza. Scritto da th_watchdog, letto da th_sim e th_gait.
   La decisione è presa a bassa frequenza e bassa priorità; l'azione è
   eseguita dal thread a priorità più alta del sistema.*/
typedef enum {
    SAFE_OK = 0,
    SAFE_STOP      /* coppia nulla su tutti i giunti */
} safe_mode_t;

enum {
    WD_TILT      = 1u << 0,   /* il tronco e' inclinato oltre il limite  */
    WD_DEAD_GAIT = 1u << 6    /* il gait ha smesso di pubblicare         */
};

typedef struct {
    uint64_t seq;
    int      mode;
    uint32_t alarms;       /* maschera degli allarmi scattati; col latch non si spegne */
} safety_t;

/* Ingresso dell'operatore. Scritto da th_command, letto da th_gait. */
typedef struct {
    uint64_t seq;
    double   vx, vy, wz;   /* velocità desiderata, sistema corpo */
    double   height;       /* altezza desiderata del tronco */
    int      gait_on;      /* 0 resta fermo, 1 trotta */
    int      quit;

    /* Iniezione di guasti, per il watchdog. */
    uint32_t push;         /* incrementato a ogni richiesta di spinta */
    double   push_force;   /* N, impostata insieme al contatore */
    int      fault_gait;   /* 1 = il thread del gait smette di pubblicare */
    
    int      att_on;       /* 1 = l'assetto stimato alimenta il gait */
} cmd_t;

/* Riferimenti dei giunti. Scritti da th_gait, letti da th_sim.*/
typedef struct {
    uint64_t seq;
    double   q[CFG_NUM_JOINTS];
    double   qd[CFG_NUM_JOINTS];
} joint_cmd_t;

#endif /* STATE_H */