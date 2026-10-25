#include "gait.h"

#include <math.h>

/* FR e RL condividono una fase, FL e RR quella opposta: i due piedi a terra in ogni
* istante sono diagonalmente oppsoti. */
const double GAIT_TROT_OFFSET[CFG_NUM_LEGS] = { 0.0, 0.5, 0.5, 0.0 };


void gait_params_default(gait_params_t *gp)
{
    gp->period    = CFG_GAIT_CYCLE;
    gp->duty      = CFG_GAIT_DUTY;
    gp->height    = CFG_BODY_HEIGHT;
    gp->clearance = CFG_FOOT_CLEARANCE;
    gp->raibert_k = CFG_RAIBERT_K;
}

/*Ritorna il punto a cui si trova la fase della gamba*/
double gait_leg_phase(const gait_params_t *gp, double t, int leg)
{
    double ph = fmod(t / gp->period + GAIT_TROT_OFFSET[leg], 1.0);
    if (ph < 0.0) ph += 1.0;
    return ph;
}

/*Ritorna se la gamba è in appoggio*/
int gait_in_stance(const gait_params_t *gp, double phase)
{
    return phase < gp->duty;
}

/* Hermite cubica su [0,1] con tangenti imposte agli estremi. Usata per la
   parte orizzontale del volo: il piede deve staccarsi e tornare a terra
   muovendosi alla velocità del terreno, altrimenti striscia al distacco e
   slitta all'appoggio. 
   Utilizzamo la stessa derivata per l'inizio e la fine quindi abbimao solo m
   Invece u rappresenta a che punto sono del volo*/
static double hermite(double p0, double p1, double m, double u)
{
    const double u2 = u * u;
    const double u3 = u2 * u;

    const double h00 =  2.0*u3 - 3.0*u2 + 1.0;
    const double h10 =        u3 - 2.0*u2 + u;
    const double h01 = -2.0*u3 + 3.0*u2;
    const double h11 =        u3 -     u2;

    return h00 * p0 + h10 * m + h01 * p1 + h11 * m;
}

void gait_feet_targets(const gait_params_t *gp,
                       double t,
                       const double v_cmd[3],
                       const double v_meas[2],
                       double amplitude,
                       const gait_attitude_t *att,
                       feet_target_t *out)
{
    const double stance_time = gp->duty * gp->period;

    /* Retroazione sul punto d'appoggio, alla Raibert: andare più veloce di
       quanto comandato sposta in avanti tutto lo schema, così i piedi
       atterrano più avanti e frenano. Applicata al punto di riferimento e non
       solo al punto di atterraggio, il che mantiene appoggio e volo continui
       all'istante del contatto. */
    double corr_x = gp->raibert_k * (v_meas[0] - v_cmd[0]) * amplitude;
    double corr_y = gp->raibert_k * (v_meas[1] - v_cmd[1]) * amplitude;

    /* Saturazione: subito dopo una variazione a gradino del comando l'errore
       è grande quanto il comando stesso, e una correzione illimitata
       chiederebbe un piede dove la gamba non arriva. */
    if (corr_x >  CFG_RAIBERT_MAX) corr_x =  CFG_RAIBERT_MAX;
    if (corr_x < -CFG_RAIBERT_MAX) corr_x = -CFG_RAIBERT_MAX;
    if (corr_y >  CFG_RAIBERT_MAX) corr_y =  CFG_RAIBERT_MAX;
    if (corr_y < -CFG_RAIBERT_MAX) corr_y = -CFG_RAIBERT_MAX;

    for (int leg = 0; leg < CFG_NUM_LEGS; leg++) {
        double hip[3], ref[3];

        leg_abad_pos(leg, hip);
        leg_nominal_foot(leg, gp->height, ref);

        /* L'imbardata fa viaggiare ogni gamba a velocità diversa: il lato
           esterno della curva spinge di più, esattamente come in un veicolo
           cingolato. */
        const double vix = v_cmd[0] - v_cmd[2] * hip[1];
        const double viy = v_cmd[1] + v_cmd[2] * hip[0];

        /* Terreno percorso da questo piede durante un appoggio. */
        const double sx = vix * stance_time * amplitude;
        const double sy = viy * stance_time * amplitude;

        const double bx = ref[0] + corr_x;
        const double by = ref[1] + corr_y;

        /* Termine di livellamento */
        double dz = 0.0;
        if (att) {
            dz = -(att->kp * att->roll)  * hip[1] - (att->kp * att->pitch) * hip[0];
            dz *= amplitude;
        }

        const double ph = gait_leg_phase(gp, t, leg);

        if (gait_in_stance(gp, ph)) {
            /* Piede piantato a terra: scorre indietro sotto il corpo,
               centrato in modo che in media stia sotto l'anca. */
            const double s = ph / gp->duty;

            out->p[leg][0] = bx - sx * (s - 0.5);
            out->p[leg][1] = by - sy * (s - 0.5);
            out->p[leg][2] = -gp->height + dz;
        } else {
            const double u = (ph - gp->duty) / (1.0 - gp->duty);

            /* Tangente agli estremi, in unità di u*/
            const double mx = -sx * (1.0 - gp->duty) / gp->duty;
            const double my = -sy * (1.0 - gp->duty) / gp->duty;

            out->p[leg][0] = hermite(bx - 0.5*sx, bx + 0.5*sx, mx, u);
            out->p[leg][1] = hermite(by - 0.5*sy, by + 0.5*sy, my, u);

            /* Campana a pendenza nulla ai due estremi: il piede si solleva e
               si posa senza strappi verticali. */
            out->p[leg][2] = -gp->height + dz
                           + gp->clearance * amplitude
                           * 0.5 * (1.0 - cos(2.0 * M_PI * u));
        }
    }
}