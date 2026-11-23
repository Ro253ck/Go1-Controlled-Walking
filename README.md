# Go1 Controlled Walking

Project for the **Real-Time Embedded Systems** course.

Walking control for the Unitree Go1 quadruped simulated in MuJoCo, written in
C with POSIX threads. The subject of the project is the **concurrent
architecture**: six independent periodic tasks, with distinct periods and
priorities, communicating exclusively through single-writer *blackboards*
protected by mutexes with priority inheritance.

The control law itself is deliberately simple — an open-loop trot with a
Raibert footstep correction and a joint-space PD. Everything that is
interesting here happens around it: how the tasks are released, how they
share state, how the deadlines are met and what happens when something
goes wrong.

## The six tasks

| thread      | period | priority | role                                     |
|-------------|--------|----------|------------------------------------------|
| `sim`       |  2 ms  |    90    | joint PD and physics stepping            |
| `estimator` |  4 ms  |    80    | Mahony filter on gyroscope and accel.    |
| `gait`      | 10 ms  |    70    | foot trajectories and inverse kinematics |
| `watchdog`  | 20 ms  |    60    | supervision and torque cut               |
| `command`   | 20 ms  |    40    | non-blocking keyboard                    |
| `logger`    | 20 ms  |    20    | CSV writing                              |

Priorities follow **rate monotonic**: shorter period, higher priority. All
six run under `SCHED_FIFO` when the process has the privileges for it; if it
does not, the program prints a warning and carries on with best-effort
timing.

Every task is released by `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)`
on an absolute grid `next += T`. The grid never restarts from the measured
instant, so a late wake-up costs jitter but never accumulates into drift.
An overrun that exceeds `CFG_MAX_CATCHUP` releases resynchronises by whole
periods instead of replaying every missed one, which bounds the catch-up
burst.

## Data flow

```
         th_command ──bb_cmd──┐
                              v
  th_sim ──bb_state────────> th_gait ──bb_des──> th_sim ──> mj_step
     │                         ^                   │
     └────bb_imu──> th_estimator ──bb_att──────────┘
            │
            └─────> th_watchdog ──bb_safe─────────> th_sim
```

Every blackboard has **exactly one writer** — six writes for six
blackboards, verifiable by grep. One writer means no write-write conflict is
possible, so the sequence number is strictly monotonic and every read is a
coherent snapshot of one producer's cycle. The mutex is still needed, for
the read-write race: it guarantees the copy is atomic.

Readers never wait when they do not have to. `bb_state_tryread` is
non-blocking and is used by the render loop, which already holds a valid
copy from the previous frame and would rather show a 2 ms old pose than miss
a vsync. The real-time threads use the blocking read, because they cannot
proceed without the data.

The data-flow graph has cycles — it is a closed control loop — but the
*waiting* graph is empty: no thread ever waits for another to produce.
That is what keeps the task set analysable.

The safety path is a multiplication, not a branch: the watchdog does not
correct trajectories, it zeroes the gain.

```c
c->sim->gain_scale = (safe.mode == SAFE_STOP) ? 0.0 : 1.0;
```

## What the controller does

**Gait.** A trot: the diagonal pairs move together, `T_g = 0.40 s` with a
duty of 0.55, so there is a short interval with all four feet down. During
stance the foot is stationary in the world and slides backwards under the
body; during swing the horizontal motion is a cubic Hermite whose endpoint
tangents match the stance velocity, and the vertical motion is a cosine bell
with zero slope at both ends. The result is a trajectory that is C1 across
the stance/swing boundary on all three axes, so the foot lifts off and
touches down without scuffing or slamming.

**Inverse kinematics.** Closed form, one leg at a time. The abduction angle
is solved in the frontal plane; the hip and knee follow from the law of
cosines in the leg plane. The domain is deliberately restricted — foot below
the hip, knee bending backwards — which makes the solution unique and
removes any possibility of the reference jumping between branches.

**Attitude.** A Mahony complementary filter on the IMU: the gyroscope is
integrated exactly, and the accelerometer pulls the estimate towards the
measured gravity direction with a time constant of `1/kp = 1 s`. Samples
whose magnitude strays more than `CFG_ATT_GATE` from `g` are rejected, since
during a trot most of them are foot impacts rather than gravity. Yaw is not
observable from an accelerometer and is never used by the controller.

**Attitude correction.** The only point where the estimate enters the loop:
the legs on the low side are lengthened and the ones on the high side pulled
in, which tilts the plane of the feet against the measured tilt. It is a
proportional term with a gain below one, so it leaves a residual by design.

**Watchdog.** Two conditions, both debounced over a sliding window of 32
cycles held in a `uint32_t`: trunk tilt beyond the limit, and the gait
thread no longer publishing. It reads the raw IMU rather than the estimator
output, so its decision does not depend on the filter it is meant to
survive. The alarm mask is latched.

## Measured behaviour

A 20 s headless run, `./build/go1 20 300 0 0.3 0`, with `SCHED_FIFO`
granted on all six threads:

| task        | T [ms] | C avg [µs] | C max [µs] | overruns | resyncs |
|-------------|--------|------------|------------|----------|---------|
| `sim`       |   2    |    523.4   |   1111.7   |    0     |    0    |
| `estimator` |   4    |     11.9   |     73.0   |    0     |    0    |
| `gait`      |  10    |     24.5   |     71.7   |    0     |    0    |
| `watchdog`  |  20    |      5.4   |     24.4   |    0     |    0    |
| `command`   |  20    |      9.5   |     58.1   |    0     |    0    |
| `logger`    |  20    |     35.9   |    236.7   |    0     |    0    |

Zero missed deadlines on all six threads, real-time factor 1.00, render at
60 fps with 0 `tryread` skips over 1200 frames.

### Blocking on the shared resources

The blackboards are shared resources, so a task can be blocked by a
lower-priority one holding a mutex it needs. Each blackboard instruments its
own acquisition, `t1 - t0` around the lock, so that blocking is measured
rather than assumed:

```
bb_state   avg 0.74 µs   max 39.8 µs        bb_imu    avg 0.65 µs   max 19.5 µs
bb_cmd     avg 0.49 µs   max 32.9 µs        bb_safe   avg 0.39 µs   max 16.5 µs
bb_des     avg 0.93 µs   max 19.5 µs        bb_att    avg 0.34 µs   max 14.8 µs
```

It stays small because every critical section is `lock → memcpy → unlock` on
a structure of at most ~250 bytes, with no nesting anywhere: no task ever
holds two blackboards at once. Mutexes use `PTHREAD_PRIO_INHERIT`, so the
blocking is bounded instead of unlimited: a low-priority holder is boosted to
the priority of the highest waiter and cannot be preempted by anything in
between.

The render loop in `main` counts as one of those lower-priority tasks: it
runs under `SCHED_OTHER` and touches `bb_state`, `bb_cmd` and `bb_safe`, so
it can block every real-time thread — which is why it releases each lock
immediately and never holds one across a frame.

Schedulability was verified by hand, both with the rate monotonic
utilisation bound and with a response time analysis including the blocking
terms above; the working is not reproduced here.

## Building

Dependencies, neither of which is committed:

- **MuJoCo 3.10.0** — the Linux release tarball from
  [github.com/google-deepmind/mujoco/releases](https://github.com/google-deepmind/mujoco/releases),
  extracted into `mujoco-3.10.0/` at the project root. The Makefile bakes an
  rpath, so no `LD_LIBRARY_PATH` is needed.
- **GLFW** and an OpenGL driver, for the viewer only. Without a display the
  program runs headless and still produces the CSV and the final report.

The robot meshes are not committed either. Copy the five `.stl` files from
[mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie),
`unitree_go1/assets/`, into `models/go1/assets/`:

```
trunk.stl   hip.stl   thigh.stl   thigh_mirror.stl   calf.stl
```

Then:

```bash
make
```

```bash
make DEBUG=1
```

`DEBUG=1` builds unoptimised with ThreadSanitizer, which is how the absence
of data races on the blackboards was checked.

## Running

The binary must be launched from the project root, since the model path is
relative.

```bash
./build/go1
```

```bash
./build/go1 20 300 0 0.3 0
```

Positional arguments, all optional:

| # | argument    | default | meaning                                      |
|---|-------------|---------|----------------------------------------------|
| 1 | `duration`  | 0       | seconds; 0 runs until quit                   |
| 2 | `kp`        | 300     | PD proportional gain, N·m/rad                |
| 3 | `kd`        | 0       | PD derivative gain; the model supplies damping |
| 4 | `vx`        | 0       | initial forward speed, m/s; non-zero also starts the trot |
| 5 | `wz`        | 0       | initial turn rate, rad/s                     |
| 6 | `k_raibert` | 0.15    | footstep feedback gain, s                    |
| 7 | `att_kp`    | 0.5     | levelling gain; 0 starts with it off         |

The scene can be swapped without recompiling, which is how the comparison on
a slope is done:

```bash
GO1_MODEL=models/go1/scene_slope.xml ./build/go1 20 300 0 0.3 0
```

| scene                  | ground                      |
|------------------------|-----------------------------|
| `scene_torque.xml`     | flat                        |
| `scene_slope.xml`      | pitched plane               |
| `scene_slope_roll.xml` | rolled plane                |

### Keyboard

Keys are read from the **terminal**, not from the viewer window, through
`termios` in non-blocking raw mode. Each cycle drains whatever the terminal
has queued and remembers only *which* keys appeared, not how many times, so
holding a key ramps at a known rate instead of flooding the command.

```
  w / s   forward speed        a / d   sideways speed
  q / e   turn rate            r / f   body height
  space   stop                 g       toggle trot
  p       shove (hold for more) k      kill/revive the gait thread
  t       attitude correction on/off
  x       quit
```

`p` and `k` are the two deliberate fault injections: an external force on
the trunk, and the gait thread silently ceasing to publish. Both exist so
the watchdog can be demonstrated rather than argued about. In the viewer
window, `ESC` or `Q` closes the program.

## Output

`logs/run.csv`, one row per logger cycle:

```
t,roll_true,pitch_true,roll_est,pitch_est
```

The true columns come from the simulation's own quaternion and are never fed
back into the controller — they exist so the filter can be scored against
ground truth.

On exit the program prints, per thread, the period, the cycle count, the
jitter min/avg/max, the execution time, and the overrun and resync counters;
then the lock-wait accumulators for each blackboard, the filter's gating
ratio, the estimated against the true attitude, the latched watchdog mask,
and the final pose.

## File tree

```
Go1-Controlled-Walking
│
├── src
│   ├── main.c                  startup, wiring, render loop, final report
│   │
│   ├── core
│   │   ├── config.h            every constant, periods and priorities
│   │   ├── state.h             the plain data types the blackboards carry
│   │   ├── rt_time.h           rt_time.c      absolute-grid periodic release
│   │   ├── rt_sched.h          rt_sched.c     SCHED_FIFO, mlockall, PI mutexes
│   │   └── blackboard.h        blackboard.c   the six single-writer boards
│   │
│   ├── logic                   no pthreads here: pure, testable functions
│   │   ├── sim.h               sim.c          MuJoCo wrapper and joint PD
│   │   ├── leg_kin.h           leg_kin.c      closed-form IK
│   │   ├── gait.h              gait.c         phase clock, foot trajectories
│   │   ├── attitude.h          attitude.c     quaternions, Mahony filter
│   │   └── render.h            render.c       viewer, private mjData
│   │
│   └── threads                 one file per periodic task
│       ├── th_sim.h            th_sim.c
│       ├── th_estimator.h      th_estimator.c
│       ├── th_gait.h           th_gait.c
│       ├── th_watchdog.h       th_watchdog.c
│       ├── th_command.h        th_command.c
│       └── th_logger.h         th_logger.c
│
├── models/go1
│   ├── go1_torque.xml          the robot: bodies, joints, actuators, sensors
│   ├── scene_torque.xml        flat ground, lighting, camera
│   ├── scene_slope.xml         pitched ground
│   └── assets/                 meshes, not committed
│
├── Makefile
└── logs
```

The split between `logic/` and `threads/` is the point of the layout:
nothing in `logic/` includes `pthread.h` or knows that a blackboard exists,
so every control function can be exercised from a ten-line `main` without
starting a single thread.
