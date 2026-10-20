#ifndef CONFIG_H
#define CONFIG_H

#include "rt_time.h"

/* Periodi multipli del passo di integrazione MuJoCo (2 ms). Iperperiodo dei task RT: 20 ms. */
#define CFG_SIM_PERIOD_NS       (2   * RT_NS_PER_MS)   /* 500 Hz, pari al timestep del modello */
#define CFG_ESTIMATOR_PERIOD_NS (4   * RT_NS_PER_MS)   /* 250 Hz */
#define CFG_GAIT_PERIOD_NS      (10  * RT_NS_PER_MS)   /* 100 Hz */
#define CFG_WATCHDOG_PERIOD_NS  (20  * RT_NS_PER_MS)   /*  50 Hz */
#define CFG_COMMAND_PERIOD_NS   (20  * RT_NS_PER_MS)   /*  50 Hz */
#define CFG_LOGGER_PERIOD_NS    (20  * RT_NS_PER_MS)   /*  50 Hz, alzato da 10 per risolvere la fase */
#define CFG_RENDER_PERIOD_NS    (33  * RT_NS_PER_MS)   /*  30 Hz, best effort */

/* Rilasci persi recuperati di fila prima di risincronizzare: limita la raffica di recupero. */
#define CFG_MAX_CATCHUP         2

/* Livelli SCHED_FIFO, Rate Monotonic: periodo piu' breve, priorita' piu' alta. */
#define CFG_PRIO_SIM            90
#define CFG_PRIO_ESTIMATOR      80
#define CFG_PRIO_GAIT           70
#define CFG_PRIO_WATCHDOG       60
#define CFG_PRIO_COMMAND        40
#define CFG_PRIO_LOGGER         20

/* Percorso relativo alla radice del progetto, da cui va lanciato il binario. */

#define CFG_MODEL_PATH          "models/go1/scene_torque.xml"

#define CFG_NUM_LEGS            4
#define CFG_JOINTS_PER_LEG      3
#define CFG_NUM_JOINTS          (CFG_NUM_LEGS * CFG_JOINTS_PER_LEG)

/* Dimensioni delle coordinate generalizzate, verificate sul modello all'avvio. */
#define CFG_NQ                  19   /* giunto libero 3+4, poi i 12 di gamba */
#define CFG_NV                  18   /* giunto libero 3+3, poi i 12 di gamba */

/* Convenzione di indice usata da ogni array: leg*3 + joint, come in go1_torque.xml. */
enum { LEG_FR = 0, LEG_FL = 1, LEG_RR = 2, LEG_RL = 3 };
enum { JNT_ABAD = 0, JNT_HIP = 1, JNT_KNEE = 2 };

/* kd a zero: il modello ha il suo smorzamento di giunto. kp alto per limitare il cedimento. */
#define CFG_KP_DEFAULT      300.0   /* N*m/rad; a kp=100 il beccheggio sbanda di -13.7 mrad */
#define CFG_KD_DEFAULT        0.0

/* Trotto. duty sopra 0.5 lascia un intervallo con quattro piedi a terra. */

#define CFG_GAIT_CYCLE      0.40    /* T_g, appoggio+volo completi, s */
#define CFG_GAIT_DUTY       0.55    /* beta, frazione del ciclo in appoggio */
#define CFG_BODY_HEIGHT     0.26    /* h0, m */
#define CFG_FOOT_CLEARANCE  0.05    /* H, apice del volo da terra, m */
#define CFG_RAIBERT_K       0.15    /* s; ottimo misurato, vicino a sqrt(h/g)=0.163 */
#define CFG_RAIBERT_MAX     0.06    /* m; tiene il piede dentro lo spazio di lavoro */
//#define CFG_GAIT_RAMP       0.50    /* s per far entrare o uscire l'andatura */
#define CFG_GAIT_RAMP       1.00

/* Filtro complementare: costante di tempo 1/kp. Il gate sospende la correzione
   quando la forza specifica misurata si allontana da g, cioe' in accelerazione. */

#define ATT_GRAVITY         9.81    /* m/s^2; il valore usato dal modello MuJoCo */
#define CFG_ATT_KP          1.0     /* 1/s, costante di tempo di un secondo */
#define CFG_ATT_SEED_TRIES  500     /* ~2 s prima di seminare comunque */
#define CFG_ATT_GATE        2.0     /* m/s^2 di scarto da g oltre cui ignorare */

/* Correzione d'assetto: allunga le gambe dal lato basso, le raccoglie dal lato alto. */
#define CFG_ATT_KP_LEVEL    0.5     /* m di corsa del piede per rad; sopra 1.0 l'anello va instabile */

/* Passi per evento tasto accettato: uno per ciclo, cosi' tenere premuto rampa a velocita' nota. */
#define CFG_VX_MAX   0.60
#define CFG_VY_MAX   0.30
#define CFG_WZ_MAX   1.00
#define CFG_VX_STEP  0.05
#define CFG_VY_STEP  0.05
#define CFG_WZ_STEP  0.10
#define CFG_H_MIN    0.20
#define CFG_H_MAX    0.32
#define CFG_H_STEP   0.01

/* Soglie del watchdog: la condizione deve valere per CFG_WD_DEBOUNCE cicli su una
   finestra scorrevole di 32, e la distanza da ogni soglia e' loggata a ogni ciclo. */

#define CFG_WD_TILT_MAX     1.57    /* rad, 90 gradi: tronco sul fianco, da li' un
                                       quadrupede non si rialza piu' */
#define CFG_WD_DEBOUNCE     20      /* su una finestra di 32 cicli, 640 ms a 50 Hz */
#define CFG_WD_LIVENESS     10      /* cicli di watchdog senza nuove pubblicazioni */

#define CFG_LOG_PATH        "logs/run.csv"

/* Due disturbi deliberati, per poter dimostrare la supervisione invece di argomentarla. */

#define CFG_PUSH_FORCE      180.0   /* N di lato, 60 ms: usa due terzi del margine di tilt */
#define CFG_PUSH_CYCLES     30      /* cicli di sim, 60 ms */

#endif /* CONFIG_H */