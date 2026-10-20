#ifndef RT_TIME_H
#define RT_TIME_H

#include <stdint.h>
#include <stdio.h>
#include <time.h>

#define RT_NS_PER_SEC 1000000000LL
#define RT_NS_PER_MS  1000000LL
#define RT_NS_PER_US  1000LL

/* Accumulatore min/max/media, usato per il jitter, il tempo di esecuzione e
   poi per l'attesa sui mutex. Privato di un solo thread: non prende nessun
   lock per conto suo. */
typedef struct {
    int64_t  min;
    int64_t  max;
    int64_t  sum;
    uint64_t n;
} rt_acc_t;

void   rt_acc_reset(rt_acc_t *a);
void   rt_acc_add(rt_acc_t *a, int64_t v);
double rt_acc_mean(const rt_acc_t *a);
void   rt_acc_report_us(const rt_acc_t *a, const char *name, FILE *out);

/* Stato di un task periodico. Un'istanza per thread, mai condivisa. */
typedef struct {
    int64_t  period_ns;
    int64_t  next_ns;    /* prossimo rilascio nominale; sempre start + k*period_ns */
    int64_t  start_ns;   /* istante di risveglio del ciclo in corso */
    int      max_catchup;

    uint64_t cycles;
    uint64_t overruns;   /* cicli ancora in esecuzione oltre il rilascio successivo */
    uint64_t resyncs;    /* volte in cui si e' saltato avanti invece di recuperare */

    rt_acc_t jitter;     /* risveglio effettivo meno rilascio nominale, sempre >= 0 */
    rt_acc_t exec;       /* tempo trascorso fra wait() e done() */
} rt_periodic_t;

int64_t         rt_now_ns(void);
struct timespec rt_ns_to_timespec(int64_t ns);
void            rt_sleep_until(int64_t abs_ns);

void rt_periodic_init(rt_periodic_t *p, int64_t period_ns, int max_catchup);
void rt_periodic_wait(rt_periodic_t *p);   /* dorme fino al prossimo rilascio, registra il jitter */
void rt_periodic_done(rt_periodic_t *p);   /* registra il tempo di esecuzione, avanza sulla griglia */
void rt_periodic_report(const rt_periodic_t *p, const char *name, FILE *out);
void rt_periodic_report_header(FILE *out);

#endif /* RT_TIME_H */