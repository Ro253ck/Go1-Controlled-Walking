# Go1 Controlled Walking

Progetto per il corso di **Real-Time Embedded Systems**.

Controllo di camminata per il quadrupede Unitree Go1 simulato in MuJoCo,
scritto in C con thread POSIX. L'oggetto del progetto è l'**architettura
concorrente**: sei task periodici indipendenti, con periodi e priorità
distinti, che comunicano esclusivamente attraverso lavagne (*blackboard*) a
singolo scrittore protette da mutex con ereditarietà di priorità.

## I sei task

| thread      | periodo | priorità | ruolo                                      |
|-------------|---------|----------|--------------------------------------------|
| `sim`       |  2 ms   |    90    | PD sui giunti e avanzamento della fisica   |
| `estimator` |  4 ms   |    80    | filtro di Mahony su giroscopio e accel.    |
| `gait`      | 10 ms   |    70    | traiettorie dei piedi e cinematica inversa |
| `watchdog`  | 20 ms   |    60    | sorveglianza e taglio della coppia         |
| `command`   | 20 ms   |    40    | tastiera non bloccante                     |
| `logger`    | 20 ms   |    20    | scrittura del CSV                          |

Iperperiodo 20 ms. Il rendering gira nel thread principale a ~60 Hz, scandito
dal vsync, e non partecipa all'insieme real-time.

## Flusso dei dati

```
         th_command ──bb_cmd──┐
                              v
  th_sim ──bb_state────────> th_gait ──bb_des──> th_sim ──> mj_step
     │                         ^                   │
     └────bb_imu──> th_estimator ──bb_att──────────┘
            │
            └─────> th_watchdog ──bb_safe─────────> th_sim
```

Ogni lavagna ha **un solo scrittore**. Il percorso di sicurezza è una
moltiplicazione e non un ramo logico: il watchdog non corregge traiettorie,
azzera il guadagno.

## Albero dei file

```
Go1-Controlled-Walking
│
├── src
│   ├── main.c
│   │
│   ├── core
│   │   ├── config.h
│   │   ├── state.h
│   │   ├── rt_time.h        rt_time.c
│   │   ├── rt_sched.h       rt_sched.c
│   │   └── blackboard.h     blackboard.c
│   │
│   ├── logic
│   │   ├── sim.h            sim.c
│   │   ├── leg_kin.h        leg_kin.c
│   │   ├── gait.h           gait.c
│   │   ├── attitude.h       attitude.c
│   │   └── render.h         render.c
│   │
│   └── threads
│       ├── th_sim.h         th_sim.c
│       ├── th_estimator.h   th_estimator.c
│       ├── th_gait.h        th_gait.c
│       ├── th_watchdog.h    th_watchdog.c
│       ├── th_command.h     th_command.c
│       └── th_logger.h      th_logger.c
│
├── models/go1
│   ├── go1_torque.xml
│   ├── scene_torque.xml
│   └── scene_slope.xml
│
├── Makefile
└── logs
```
