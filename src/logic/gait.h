#ifndef GAIT_H
#define GAIT_H
 
#include "config.h"
#include "leg_kin.h"

/* Sfasamenti che definiscono il trotto: le coppie diagonali si muovono
   insieme. Indicizzati FR, FL, RR, RL. */
extern const double GAIT_TROT_OFFSET[CFG_NUM_LEGS];

typedef struct {
    double period;      /* T_g, un appoggio più un volo completi */
    double duty;        /* beta, frazione del ciclo trascorsa in appoggio */
    double height;      /* h0, altezza nominale del tronco */
    double clearance;   /* H, apice del volo */
    double raibert_k;   /* guadagno della retroazione sul punto d'appoggio */
} gait_params_t;

/* Retroazione d'assetto applicata attraverso i piedi: nessun controllo di
   forza, solo una gamba più lunga dal lato che si è abbassato. È l'unico
   punto in cui la stima dell'IMU entra nell'anello di controllo. */
typedef struct {
    double roll, pitch;    /* assetto stimato */
    double kp;             
} gait_attitude_t;

void gait_params_default(gait_params_t *gp);

/* Fase di una gamba all'istante t, in [0,1). */
double gait_leg_phase(const gait_params_t *gp, double t, int leg);

/* 1 finché la gamba è in appoggio. */
int gait_in_stance(const gait_params_t *gp, double phase);

/* Obiettivi dei piedi nel sistema del tronco.*/
void gait_feet_targets(const gait_params_t *gp,
                       double t,
                       const double v_cmd[3],
                       const double v_meas[2],
                       double amplitude,
                       const gait_attitude_t *att,   /* NULL lo disabilita */
                       feet_target_t *out);

#endif /* GAIT_H */