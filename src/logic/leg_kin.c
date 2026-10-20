#include "leg_kin.h"

#include <math.h>

/*Inizializzazione dei limiti dei giunti*/
const double LEG_Q_MIN[CFG_JOINTS_PER_LEG] = { -0.863, -0.686, -2.818 };
const double LEG_Q_MAX[CFG_JOINTS_PER_LEG] = {  0.863,  4.501, -0.888 };

/*Dicono su quale lato sta la zampa. FL e RL stanno sul lato y+, FR e RR sul lato y-. */
static int leg_is_left(int leg)  { return (leg == LEG_FL || leg == LEG_RL); }
static int leg_is_front(int leg) { return (leg == LEG_FR || leg == LEG_FL); }

/*Ritorna la distanza della gamba dall'asse di abduzione, con segno in base a quale gamba consideriamo*/
double leg_offset_d(int leg)
{
    return leg_is_left(leg) ? LEG_D : -LEG_D;
}

/*Ritorna la posizione del giunto di abduzione, in base a quale gamba consideriamo.
  Le anteriori e le sinistre sono positive, le posteriori e le destre negative*/
void leg_hip_pos(int leg, double out[3])
{
    out[0] = leg_is_front(leg) ? LEG_HIP_X : -LEG_HIP_X;
    out[1] = leg_is_left(leg)  ? LEG_HIP_Y : -LEG_HIP_Y;
    out[2] = 0.0;
}

/*Riporta ogni angolo nell'intervallo [-pi,pi]*/
static double wrap_pi(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// /*Cinematica diretta*/
// void leg_fk(int leg, const double q[CFG_JOINTS_PER_LEG], double p[3])
// {
//     const double d   = leg_offset_d(leg);
//     const double s2  = sin(q[JNT_HIP]);
//     const double c2  = cos(q[JNT_HIP]);
//     const double s23 = sin(q[JNT_HIP] + q[JNT_KNEE]);
//     const double c23 = cos(q[JNT_HIP] + q[JNT_KNEE]);

//     /* Piede visto dall'attacco del femore, prima dell'abduzione: il piano della gamba. */
//     const double a = -LEG_L1 * s2 - LEG_L2 * s23;
//     const double c = -LEG_L1 * c2 - LEG_L2 * c23;

//     /* Ruotare attorno a x lascia x invariata, ed è questo che disaccoppia
//        l'abduzione dagli altri due giunti. */
//     p[0] = a;
//     p[1] = d * cos(q[JNT_ABAD]) - c * sin(q[JNT_ABAD]);
//     p[2] = d * sin(q[JNT_ABAD]) + c * cos(q[JNT_ABAD]);
// }

/*Cinematica inversa*/
void leg_ik(int leg, const double p[3], double q[CFG_JOINTS_PER_LEG])
{
    const double d = leg_offset_d(leg); /*È una costante*/

    /* Passo 1: cerchiamo l'angolo di abduzione, che vive nel piano frontale (y-z). */
    double c2 = p[1] * p[1] + p[2] * p[2] - d * d;
    /* piede richiesto più vicino all'asse di quanto d consenta */
    if (c2 < 0.0) c2 = 0.0;
    const double c = -sqrt(c2);   /* negativo: il piede sta sotto l'anca */
    
    /* Avendo la posizione (py,pz), che è la posizione (d,c) dopo l'abduzione,
       possiamo ricavare l'angolo fra le due in questo modo: */
    q[JNT_ABAD] = wrap_pi(atan2(p[2], p[1]) - atan2(c, d));

    /* Passo 2 (piano x-z): il ginocchio, per il teorema del coseno sul triangolo
       anca-ginocchio-piede. Il segno davanti a 2*L1*L2 è positivo perché q3
       misura lo scostamento dalla gamba distesa, non l'angolo interno. */
    const double L_sq  = p[0] * p[0] + c * c;
    double cos_q3 = (L_sq - LEG_L1 * LEG_L1 - LEG_L2 * LEG_L2) / (2.0 * LEG_L1 * LEG_L2);

    /*Controllo se io bersaglio è raggiunigbile */
    if (cos_q3 >  1.0) cos_q3 =  1.0;
    if (cos_q3 < -1.0) cos_q3 = -1.0;

    q[JNT_KNEE] = -acos(cos_q3);   /* ramo negativo: il ginocchio piega all'indietro */

    /* Passo 3: usiamo di nuovo il teorema del coseno, questa volta per
       ricavare l'angolo dell'anca */
    const double L = sqrt(L_sq);
    double cos_psi;

    if (L > 1e-9) {
        cos_psi = (L_sq + LEG_L1*LEG_L1 - LEG_L2*LEG_L2) / (2.0 * LEG_L1 * L);
        if (cos_psi >  1.0) cos_psi =  1.0;
        if (cos_psi < -1.0) cos_psi = -1.0;
    } else {
        cos_psi = 1.0;
    }

    q[JNT_HIP] = wrap_pi(atan2(-p[0], -c) + acos(cos_psi));

    /*Controllo che tutti i giunti siano dentro i limiti*/
    for (int j = 0; j < CFG_JOINTS_PER_LEG; j++) {
        if (q[j] < LEG_Q_MIN[j]) q[j] = LEG_Q_MIN[j];
        if (q[j] > LEG_Q_MAX[j]) q[j] = LEG_Q_MAX[j];
    }
}

void legs_ik_trunk(const feet_target_t *feet, double q[CFG_NUM_JOINTS])
{

    for (int leg = 0; leg < CFG_NUM_LEGS; leg++) {
        double hip[3], p_hip[3];

        leg_hip_pos(leg, hip);
        for (int i = 0; i < 3; i++)
            p_hip[i] = feet->p[leg][i] - hip[i];

        leg_ik(leg, p_hip, &q[leg * CFG_JOINTS_PER_LEG]);
    }
}

void leg_nominal_foot(int leg, double height, double p_trunk[3])
{
    double hip[3];
    leg_hip_pos(leg, hip);

    /* Esattamente sotto l'anca, scostato lateralmente di d: è la posa che
       dà un angolo di abduzione esattamente nullo. */
    p_trunk[0] = hip[0];
    p_trunk[1] = hip[1] + leg_offset_d(leg);
    p_trunk[2] = -height;
}