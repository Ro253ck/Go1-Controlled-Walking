#ifndef RT_TIME_H
#define RT_TIME_H

#include <stdint.h>
#include <stdio.h>
#include <time.h>

#define RT_NS_PER_SEC 1000000000LL
#define RT_NS_PER_MS  1000000LL
#define RT_NS_PER_US  1000LL

/* min/max/mean accumulator, used for the jitter, the execution time and
   then for the wait on the mutexes. Private to a single thread: it takes no
   lock of its own. */
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

/* State of a periodic task. One instance per thread, never shared. */
typedef struct {
    int64_t  period_ns;
    int64_t  next_ns;    /* next nominal release; always start + k*period_ns */
    int64_t  start_ns;   /* wake-up instant of the cycle in progress */
    int      max_catchup;

    uint64_t cycles;
    uint64_t overruns;   /* cycles still running past the following release */
    uint64_t resyncs;    /* times we jumped ahead instead of catching up */

    rt_acc_t jitter;     /* actual wake-up minus nominal release, always >= 0 */
    rt_acc_t exec;       /* time elapsed between wait() and done() */
} rt_periodic_t;

int64_t         rt_now_ns(void);
struct timespec rt_ns_to_timespec(int64_t ns);
void            rt_sleep_until(int64_t abs_ns);

void rt_periodic_init(rt_periodic_t *p, int64_t period_ns, int max_catchup);
void rt_periodic_wait(rt_periodic_t *p);   /* sleeps until the next release, records the jitter */
void rt_periodic_done(rt_periodic_t *p);   /* records the execution time, advances on the grid */
void rt_periodic_report(const rt_periodic_t *p, const char *name, FILE *out);
void rt_periodic_report_header(FILE *out);

#endif /* RT_TIME_H */
