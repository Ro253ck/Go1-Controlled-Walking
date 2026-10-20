# Go1 Controlled Walking

Controllo di camminata per il quadrupede **Unitree Go1** simulato in MuJoCo,
scritto in C con thread POSIX. Progetto per il corso di *Real-Time Embedded
Systems*.

Il contenuto tecnico del lavoro è l'**architettura concorrente**: sei task
periodici indipendenti, con priorità e periodi distinti, che comunicano
esclusivamente attraverso lavagne (*blackboard*) protette da mutex con
ereditarietà di priorità. Il controllo in sé — trotto a ciclo aperto con
cinematica inversa in forma chiusa e livellamento dell'assetto — è il carico di
lavoro che dà senso ai vincoli temporali, non il fine del progetto.

## Architettura

Sei thread periodici, tutti su `SCHED_FIFO` quando i permessi lo consentono:

| thread      | periodo  | frequenza | priorità | ruolo                                    |
|-------------|----------|-----------|----------|------------------------------------------|
| `sim`       |   2 ms   |  500 Hz   |    90    | PD sui giunti e avanzamento della fisica |
| `estimator` |   4 ms   |  250 Hz   |    80    | filtro di Mahony su giroscopio e accel.  |
| `gait`      |  10 ms   |  100 Hz   |    70    | traiettorie dei piedi e cinematica inv.  |
| `watchdog`  |  20 ms   |   50 Hz   |    60    | sorveglianza e taglio della coppia       |
| `command`   |  20 ms   |   50 Hz   |    40    | tastiera non bloccante                   |
| `logger`    |  20 ms   |   50 Hz   |    20    | scrittura del CSV                        |

Il rendering gira nel thread principale a ~30 Hz, *best effort*, e non
partecipa all'insieme real-time.

### Comunicazione

Ogni lavagna ha **un solo scrittore** e più lettori, con semantica
*l'ultimo è il migliore*: nessuna coda, nessun produttore che si blocca. La
copia avviene dentro la sezione critica, e un contatore `seq` permette al
lettore di distinguere un dato fresco da uno fermo. Il renderer usa una
variante `tryread` che rinuncia al dato invece di attendere, così un frame
perso non può ritardare un task di controllo.

```
th_command  --bb_cmd-->  th_gait
th_sim      --bb_state-> th_gait, th_logger, render
th_sim      --bb_imu-->  th_estimator, th_watchdog
th_estimator--bb_att-->  th_gait, th_logger
th_gait     --bb_des-->  th_sim, th_watchdog
th_watchdog --bb_safe--> th_sim
```

### Catena di controllo

```
th_gait @ 100 Hz          th_sim @ 500 Hz
  posizioni dei piedi       riferimenti di giunto
  (metri, sist. tronco)     tau = kp*(q_des-q) + kd*(qd_des-qd)
        |                           |
        +-- cinem. inversa -->  bb_des  -->  mj_step
```

Il percorso di sicurezza è deliberatamente una moltiplicazione e non un ramo
logico: il watchdog non corregge traiettorie, azzera il guadagno.

```c
c->sim->gain_scale = (safe.mode == SAFE_STOP) ? 0.0 : 1.0;
```

## Dipendenze

### MuJoCo 3.10.0

Scaricare il pacchetto per Linux dalle *release* di
[google-deepmind/mujoco](https://github.com/google-deepmind/mujoco/releases)
e scompattarlo nella radice del progetto, così da ottenere
`mujoco-3.10.0/include` e `mujoco-3.10.0/lib`:

```bash
tar xzf mujoco-3.10.0-linux-x86_64.tar.gz
```

Il percorso della libreria viene scritto nel binario con `-rpath`, quindi non
serve impostare `LD_LIBRARY_PATH`. Per una collocazione diversa:

```bash
make MJ_DIR=/percorso/della/tua/mujoco
```

### GLFW

```bash
sudo apt install libglfw3-dev
```

### Mesh del Go1

Il modello carica cinque file STL che non sono ridistribuiti in questo repo.
Provengono da [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie)
e sono sotto licenza BSD 3-Clause di Unitree Robotics (il testo è in
`models/go1/assets/LICENSE.unitree`). Si scaricano così:

```bash
mkdir -p models/go1/assets
BASE=https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main/unitree_go1/assets
for m in trunk hip thigh thigh_mirror calf; do curl -L -o models/go1/assets/$m.stl $BASE/$m.stl; done
```

In totale circa 10 MB. Chi ha già `mujoco_menagerie` può copiarne o collegarne
la cartella `unitree_go1/assets`.

## Compilazione

```bash
make
```

Con il *thread sanitizer*, per verificare l'assenza di corse critiche sulle
lavagne:

```bash
make DEBUG=1
```

## Esecuzione

Il programma va lanciato **dalla radice del progetto**: i percorsi del modello
e del file di log sono relativi alla directory di lavoro.

```bash
./build/go1 [durata_s] [kp] [kd] [vx] [wz] [k_raibert] [att_kp]
```

Tutti gli argomenti sono opzionali e posizionali.

| argomento   | default | significato                                        |
|-------------|---------|----------------------------------------------------|
| `durata_s`  | 0       | secondi di simulazione; 0 = finché non si esce     |
| `kp`        | 300.0   | guadagno proporzionale del PD, N·m/rad             |
| `kd`        | 0.0     | guadagno derivativo                                |
| `vx`        | 0.0     | velocità longitudinale iniziale, m/s               |
| `wz`        | 0.0     | velocità di imbardata iniziale, rad/s              |
| `k_raibert` | 0.15    | guadagno della retroazione sul punto d'appoggio    |
| `att_kp`    | 0.5     | guadagno del livellamento, m di corsa per radiante |

Esempi:

```bash
./build/go1                        # interattivo, parametri di default
./build/go1 10 300 0 0.3 0.0       # dieci secondi in avanti a 0.3 m/s
```

Per la scena con il piano inclinato:

```bash
GO1_MODEL=models/go1/scene_slope.xml ./build/go1 10 300 0 0.3
```

### Comandi da tastiera

```
w / s   velocità longitudinale      a / d   velocità laterale
q / e   velocità di imbardata       r / f   altezza del tronco
space   arresto                     g       attiva/disattiva il trotto
p       spinta (tenere premuto)     k       uccide/rianima il thread gait
t       livellamento on/off         x       uscita
```

`k` e `p` servono a provocare i guasti che il watchdog deve riconoscere: `k`
interrompe le pubblicazioni del gait (allarme di vitalità), `p` applica una
forza al tronco fino a ribaltarlo (allarme di inclinazione).

## Uscite

A fine esecuzione viene stampato un rapporto con jitter, tempo di esecuzione,
*overrun* e *resync* per ogni thread, l'attesa sui mutex di ogni lavagna, il
fattore di tempo reale e lo stato del watchdog.

`logs/run.csv` contiene il confronto fra assetto vero e stimato:

```
t, roll_true, pitch_true, roll_est, pitch_est
```

Lo yaw non viene registrato: con solo giroscopio e accelerometro è
inosservabile, e confrontarlo misurerebbe la deriva di un integratore invece
della qualità del filtro.

## Licenze

Il codice in `src/` è opera dell'autore. Le mesh in `models/go1/assets/`, non
incluse nel repository, sono BSD 3-Clause di Unitree Robotics. MuJoCo è
Apache 2.0 di Google DeepMind.
