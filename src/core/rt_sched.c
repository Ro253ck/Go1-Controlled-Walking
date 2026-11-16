#define _GNU_SOURCE
#include "rt_sched.h"

#include <errno.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

int rt_lock_memory(void)
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        fprintf(stderr, "WARNING: mlockall failed (%s): page faults may add "
                        "milliseconds of jitter\n", strerror(errno));
        return -1;
    }
    return 0;
}

int rt_have_rt_privileges(void)
{
    struct sched_param sp = { .sched_priority = 1 };
    int  saved = sched_getscheduler(0);
    int  ok;

    /* We probe by trying and undoing it straight away: cheaper and more honest
       than reading the rlimits, since capabilities can grant this
       permission without touching them. */
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
        return 0;

    sp.sched_priority = 0;
    ok = 1;
    sched_setscheduler(0, saved < 0 ? SCHED_OTHER : saved, &sp);
    return ok;
}

int rt_thread_start(rt_thread_t *t)
{
    pthread_attr_t     attr;
    struct sched_param sp;
    int                rc;

    t->started = 0;
    t->is_rt   = 0;

    pthread_attr_init(&attr);

    if (t->priority > 0) {
        sp.sched_priority = t->priority;
        /* Without EXPLICIT_SCHED the policy and priority set below
           are ignored and the thread silently inherits those of whoever
           created it. */
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        pthread_attr_setschedparam(&attr, &sp);
    }

    if (t->cpu >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET((size_t)t->cpu, &set);
        pthread_attr_setaffinity_np(&attr, sizeof(set), &set);
    }

    /* pthread_create returns the error code; it does not set errno. */
    rc = pthread_create(&t->tid, &attr, t->entry, t->arg);

    if (rc == 0) {
        t->started = 1;
        t->is_rt   = (t->priority > 0);
    } else if (rc == EPERM && t->priority > 0) {
        fprintf(stderr, "WARNING: thread '%s' denied SCHED_FIFO, running on " 
                        "SCHED_OTHER: timing figures are not real-time\n", t->name);
        pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
        rc = pthread_create(&t->tid, &attr, t->entry, t->arg);
        t->started = (rc == 0);
    }

    if (rc != 0 && !t->started)
        fprintf(stderr, "ERROR: thread '%s' failed to start: %s\n",t->name, strerror(rc));

    pthread_attr_destroy(&attr);
    return t->started ? 0 : -1;
}

int rt_thread_join(rt_thread_t *t)
{
    if (!t->started) return -1;
    return pthread_join(t->tid, NULL);
}

/*We instantiate priority inheritance as the resource management protocol*/
void rt_mutex_init_pi(pthread_mutex_t *m)
{
    pthread_mutexattr_t ma;

    pthread_mutexattr_init(&ma);
    if (pthread_mutexattr_setprotocol(&ma, PTHREAD_PRIO_INHERIT) != 0) {
        /*On error we exit, there is no point going on since this is a fundamental assumption*/
        fprintf(stderr, "FATAL: PTHREAD_PRIO_INHERIT unavailable\n");
        exit(EXIT_FAILURE);
    }
    pthread_mutex_init(m, &ma);
    pthread_mutexattr_destroy(&ma);
}