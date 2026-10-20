#ifndef RT_SCHED_H
#define RT_SCHED_H

#include <pthread.h>

typedef struct {
    const char *name;
    int         priority;   /* priorita' SCHED_FIFO 1..99, oppure 0 per SCHED_OTHER */
    int         cpu;        /* CPU su cui fissarlo, -1 per nessuna affinita' */
    void     *(*entry)(void *); /*Puntatore  a funzione*/
    void       *arg;

    pthread_t   tid;
    int         started;
    int         is_rt;      /* 1 solo se SCHED_FIFO e' stato davvero concesso */
} rt_thread_t;

/* Blocca l'immagine del processo in RAM. Un page fault costa millisecondi,
   che un ciclo di controllo da 2 ms non puo' assorbire. Da chiamare una
   volta sola, prima di avviare qualsiasi thread. */
int rt_lock_memory(void);

/* Dice se il processo puo' usare SCHED_FIFO, cosi' il chiamante puo'
   avvisare una volta sola invece che una volta per thread. */
int rt_have_rt_privileges(void);

int rt_thread_start(rt_thread_t *t);
int rt_thread_join(rt_thread_t *t);

/* Mutex con PTHREAD_PRIO_INHERIT: limita il blocco che un thread ad alta
   priorita' puo' subire, e rende il passaggio del lock ordinato per
   priorita' invece che arbitrario. */
void rt_mutex_init_pi(pthread_mutex_t *m);


#endif /* RT_SCHED_H */