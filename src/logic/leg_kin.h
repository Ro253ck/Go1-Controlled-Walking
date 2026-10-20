#ifndef LEG_KIN_H
#define LEG_KIN_H

/* Cinematica di una singola gamba, in forma chiusa.
   Tutto è espresso nel sistema dell'anca: origine sull'asse di abduzione
   della gamba, assi allineati al tronco (x avanti, y sinistra, z su).
*/

#include "config.h"

/* Geometria dei link, ricavata dal modello. Tutte le misure sono in metri. */
#define LEG_L1   0.213     /* lunghezza del femore */
#define LEG_L2   0.213     /* lunghezza della tibia, identica per progetto */
#define LEG_D    0.08      /* distanza dall'asse di abduzione al piano della gamba, in modulo */

/* Posizione dell'origine dell'anca rispetto al centro del tronco. */
#define LEG_HIP_X 0.1881
#define LEG_HIP_Y 0.04675

/* Limiti di giunto, indicizzati con JNT_ABAD / JNT_HIP / JNT_KNEE. */
extern const double LEG_Q_MIN[CFG_JOINTS_PER_LEG];
extern const double LEG_Q_MAX[CFG_JOINTS_PER_LEG];

/* I quattro obiettivi dei piedi nel sistema del tronco. */
typedef struct {
    double p[CFG_NUM_LEGS][3];
} feet_target_t;

/* Scostamento laterale con segno: l'unica cosa che distingue destra da sinistra. */
double leg_offset_d(int leg);

/* Origine dell'anca nel sistema del tronco. */
void leg_hip_pos(int leg, double out[3]);

/* Angoli dei giunti -> posizione del piede nel sistema dell'anca. Serve solo
*  a verificare il corretto funzionamento della funzione inversa. */
//void leg_fk(int leg, const double q[CFG_JOINTS_PER_LEG], double p[3]);

/* Cinematica inversa: posizione del piede -> angoli dei giunti. Una posa
   irraggiungibile viene portata sul punto valido piu' vicino. */
void leg_ik(int leg, const double p[3], double q[CFG_JOINTS_PER_LEG]);

/* I quattro piedi nel sistema del tronco -> i dodici angoli di giunto,
   nell'ordine degli attuatori. */
void legs_ik_trunk(const feet_target_t *feet, double q[CFG_NUM_JOINTS]);

/* Posizione del piede esattamente sotto l'anca per una data altezza del
   tronco, nel sistema del tronco. È l'appoggio di riferimento attorno al
   quale il gait introduce le sue variazioni. */
void leg_nominal_foot(int leg, double height, double p_trunk[3]);

#endif /* LEG_KIN_H */