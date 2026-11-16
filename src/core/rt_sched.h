#ifndef RT_SCHED_H
#define RT_SCHED_H

#include <pthread.h>

typedef struct {
    const char *name;
    int         priority;   /* SCHED_FIFO priority 1..99, or 0 for SCHED_OTHER */
    int         cpu;        /* CPU to pin it to, -1 for no affinity */
    void     *(*entry)(void *); /*Function pointer*/
    void       *arg;

    pthread_t   tid;
    int         started;
    int         is_rt;      /* 1 only if SCHED_FIFO was actually granted */
} rt_thread_t;

/* Locks the process image in RAM. A page fault costs milliseconds,
   which a 2 ms control cycle cannot absorb. To be called once
   only, before starting any thread. */
int rt_lock_memory(void);

/* Says whether the process can use SCHED_FIFO, so the caller can
   warn once instead of once per thread. */
int rt_have_rt_privileges(void);

int rt_thread_start(rt_thread_t *t);
int rt_thread_join(rt_thread_t *t);

/* Mutex with PTHREAD_PRIO_INHERIT: bounds the blocking a high priority
   thread can suffer, and makes the lock hand-off ordered by
   priority instead of arbitrary. */
void rt_mutex_init_pi(pthread_mutex_t *m);


#endif /* RT_SCHED_H */
