#include "rt_time.h"

#include <errno.h>

/*accumulator */

void rt_acc_reset(rt_acc_t *a)
{
    a->min = INT64_MAX;
    a->max = INT64_MIN;
    a->sum = 0;
    a->n   = 0;
}

void rt_acc_add(rt_acc_t *a, int64_t v)
{
    if (v < a->min) a->min = v;
    if (v > a->max) a->max = v;
    a->sum += v;
    a->n++;
}

double rt_acc_mean(const rt_acc_t *a)
{
    return a->n ? (double)a->sum / (double)a->n : 0.0;
}

void rt_acc_report_us(const rt_acc_t *a, const char *name, FILE *out)
{
    const double us = (double)RT_NS_PER_US;

    if (a->n == 0) {
        fprintf(out, "%-22s (no samples)\n", name);
        return;
    }
    fprintf(out, "%-22s n=%-9llu min %7.3f  avg %7.3f  max %8.3f  us\n",
            name, (unsigned long long)a->n,
            (double)a->min / us, rt_acc_mean(a) / us, (double)a->max / us);
}

/* time */
int64_t rt_now_ns(void)
{
    struct timespec ts;
    /* CLOCK_MONOTONIC and not CLOCK_REALTIME so NTP cannot cause trouble*/
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * RT_NS_PER_SEC + ts.tv_nsec;
}

struct timespec rt_ns_to_timespec(int64_t ns)
{
    struct timespec ts;
    /*We fill in the data structure*/
    ts.tv_sec  = (time_t)(ns / RT_NS_PER_SEC);
    ts.tv_nsec = (long)  (ns % RT_NS_PER_SEC);
    return ts;
}

void rt_sleep_until(int64_t abs_ns)
{
    struct timespec ts = rt_ns_to_timespec(abs_ns);
    int rc;

    /*TIMER_ABSTIME makes us sleep until a given instant, not for a duration*/
    while ((rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL)) == EINTR);
    (void)rc;
}

/*periodic task */
void rt_periodic_init(rt_periodic_t *p, int64_t period_ns, int max_catchup)
{
    p->period_ns   = period_ns;
    p->next_ns     = rt_now_ns() + period_ns;
    p->start_ns    = 0;
    p->max_catchup = max_catchup;
    p->cycles      = 0;
    p->overruns    = 0;
    p->resyncs     = 0;
    rt_acc_reset(&p->jitter);
    rt_acc_reset(&p->exec);
}

void rt_periodic_wait(rt_periodic_t *p)
{
    rt_sleep_until(p->next_ns);
    p->start_ns = rt_now_ns();
    /* The jitter is measured against the nominal release, never against the
       previous wake-up: this is what stops the error from
       accumulating. */
    rt_acc_add(&p->jitter, p->start_ns - p->next_ns);
    p->cycles++;
}

void rt_periodic_done(rt_periodic_t *p)
{
    int64_t now = rt_now_ns();

    rt_acc_add(&p->exec, now - p->start_ns);

    /* The grid advances unconditionally, starting from the previous nominal
       release and never from the measured instant: this is what eliminates
       the drift. */
    p->next_ns += p->period_ns;

    if (now > p->next_ns) {
        int64_t missed = (now - p->next_ns) / p->period_ns + 1;
        p->overruns++;
        if (missed > (int64_t)p->max_catchup) {
            /* Too late to replay every missed release without
               starving the lower priority threads: we jump to the first
               future release. The phase of the grid stays intact because
               we add whole periods. */
            p->next_ns += missed * p->period_ns;
            p->resyncs++;
        }
    }
}

void rt_periodic_report_header(FILE *out)
{
    fprintf(out, "%-12s %8s %9s %26s %18s %8s %8s\n",
            "thread", "T [ms]", "cycles",
            "jitter us  min/avg/max", "exec us  avg/max",
            "overrun", "resync");
}

void rt_periodic_report(const rt_periodic_t *p, const char *name, FILE *out)
{
    const double us = (double)RT_NS_PER_US;

    fprintf(out, "%-12s %8.1f %9llu %8.1f %8.1f %8.1f %8.1f %9.1f %8llu %8llu\n",
            name,
            (double)p->period_ns / (double)RT_NS_PER_MS,
            (unsigned long long)p->cycles,
            (double)p->jitter.min / us,
            rt_acc_mean(&p->jitter) / us,
            (double)p->jitter.max / us,
            rt_acc_mean(&p->exec) / us,
            (double)p->exec.max / us,
            (unsigned long long)p->overruns,
            (unsigned long long)p->resyncs);
}