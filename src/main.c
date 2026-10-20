
#include "blackboard.h"
#include "config.h"
#include "leg_kin.h"
#include "render.h"
#include "rt_sched.h"
#include "rt_time.h"
#include "sim.h"
#include "th_command.h"
#include "th_estimator.h"
#include "th_gait.h"
#include "th_logger.h"
#include "th_watchdog.h"
#include "th_sim.h"

#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static atomic_int g_running = 1;

static void on_sigint(int sig)
{
    (void)sig;
    atomic_store(&g_running, 0);
}

/* Riferimenti di giunto che tengono l'appoggio nominale. Servono a
   inizializzare bb_des, cosi' il thread sim non vede mai un riferimento
   tutto a zero, che stenderebbe le gambe dritte prima che il thread gait
   abbia pubblicato qualcosa. */
static void seed_stance(joint_cmd_t *des, double height)
{
    feet_target_t feet;

    memset(des, 0, sizeof(*des));
    for (int leg = 0; leg < CFG_NUM_LEGS; leg++)
        leg_nominal_foot(leg, height, feet.p[leg]);
    legs_ik_trunk(&feet, des->q);
}

int main(int argc, char **argv)
{
    int64_t duration_s = (argc > 1) ? atoll(argv[1]) : 0;
    double  kp         = (argc > 2) ? atof(argv[2]) : CFG_KP_DEFAULT;
    double  kd         = (argc > 3) ? atof(argv[3]) : CFG_KD_DEFAULT;
    double  vx0        = (argc > 4) ? atof(argv[4]) : 0.0;
    double  wz0        = (argc > 5) ? atof(argv[5]) : 0.0;
    double  kraib      = (argc > 6) ? atof(argv[6]) : CFG_RAIBERT_K;
    double  att_kp     = (argc > 7) ? atof(argv[7]) : CFG_ATT_KP_LEVEL;

    /*Definitimao tutte le blackboard e strutture principali*/
    sim_t      sim;
    bb_state_t bb_state;
    bb_cmd_t   bb_cmd;
    bb_des_t   bb_des;
    bb_imu_t   bb_imu;
    bb_att_t   bb_att;
    bb_safe_t  bb_safe;

    /*Definiamo tutte i thread context*/
    th_sim_ctx_t     ctx_sim;
    th_gait_ctx_t    ctx_gait;
    th_estimator_ctx_t ctx_est;
    th_command_ctx_t ctx_cmd;
    th_watchdog_ctx_t ctx_wd;
    th_logger_ctx_t   ctx_log;
    rt_thread_t      th[6];

    render_t      rnd;
    robot_state_t st;
    cmd_t         cmd0;
    joint_cmd_t   des0;
    att_t         att0;
    safety_t      safe0;
    rt_acc_t      frame_dt;
    rt_acc_t      roll_mrad, pitch_mrad;   /* telemetria dell'assetto vero */
    char          err[1024] = {0};

    uint64_t frames = 0, busy_skips = 0;
    int      have_window, have_tty;
    int64_t  t0, t_prev, t_end;
    double   rt_factor = 0.0, sim_time_0 = 0.0;
    int      have_origin = 0;

    signal(SIGINT, on_sigint);

    /* Permette a una run di scegliere una scena diversa senza ricompilare,
       ed e' cosi' che si fa il confronto sul piano inclinato. */
    const char *model = getenv("GO1_MODEL");
    if (!model) model = CFG_MODEL_PATH;

    if (sim_init(&sim, model, err, sizeof(err)) != 0) {
        fprintf(stderr, "model load failed: %s\n", err);
        return 1;
    }
    sim_reset_to_key(&sim, "home");
    sim_set_gains(&sim, kp, kd);

    /* Le lavagne partono gia' popolate, cosi' nessun lettore puo' mai
       osservare una struttura azzerata fra l'avvio e la prima pubblicazione. */
    cmd0.vx = vx0;
    cmd0.vy = 0.0;
    cmd0.wz = wz0;
    cmd0.height  = CFG_BODY_HEIGHT;
    cmd0.gait_on = (argc > 4);
    cmd0.quit    = 0;
    cmd0.push    = 0;
    cmd0.push_force = CFG_PUSH_FORCE;
    cmd0.fault_gait = 0;
    cmd0.att_on     = (att_kp != 0.0);
    seed_stance(&des0, CFG_BODY_HEIGHT);

    memset(&att0, 0, sizeof(att0));
    att0.q[0] = 1.0;

    bb_state_init(&bb_state);
    bb_cmd_init(&bb_cmd, &cmd0);
    bb_des_init(&bb_des, &des0);
    bb_imu_init(&bb_imu);
    bb_att_init(&bb_att, &att0);
    memset(&safe0, 0, sizeof(safe0));
    bb_safe_init(&bb_safe, &safe0);

    memset(&st, 0, sizeof(st));
    rt_acc_reset(&frame_dt);
    rt_acc_reset(&roll_mrad);
    rt_acc_reset(&pitch_mrad);

    rt_lock_memory();
    if (!rt_have_rt_privileges())
        fprintf(stderr, "WARNING: no SCHED_FIFO permission, timing is best-effort\n\n");

    ctx_sim.sim      = &sim;
    ctx_sim.bb_state = &bb_state;
    ctx_sim.bb_imu   = &bb_imu;
    ctx_sim.bb_des   = &bb_des;
    ctx_sim.running  = &g_running;
    ctx_sim.bb_safe  = &bb_safe;
    ctx_sim.bb_cmd   = &bb_cmd;

    ctx_gait.bb_state  = &bb_state;
    ctx_gait.bb_cmd    = &bb_cmd;
    ctx_gait.bb_att    = &bb_att;   /* sempre collegato: l'interruttore e' cmd.att_on */
    ctx_gait.bb_des    = &bb_des;
    ctx_gait.running   = &g_running;
    gait_params_default(&ctx_gait.params);
    ctx_gait.params.raibert_k = kraib;
    ctx_gait.att_kp = att_kp;

    ctx_est.bb_imu   = &bb_imu;
    ctx_est.bb_att   = &bb_att;
    ctx_est.running  = &g_running;
    ctx_est.n_seed_tries = 0;
    filter_init(&ctx_est.filter, CFG_ATT_KP, CFG_ATT_GATE);

    memset(&ctx_wd, 0, sizeof(ctx_wd));
    ctx_wd.bb_imu  = &bb_imu;  ctx_wd.bb_des   = &bb_des;
    ctx_wd.bb_safe = &bb_safe; ctx_wd.running  = &g_running;

    memset(&ctx_log, 0, sizeof(ctx_log));
    ctx_log.bb_state = &bb_state; ctx_log.bb_att  = &bb_att;
    ctx_log.running  = &g_running;
    ctx_log.path     = CFG_LOG_PATH;

    ctx_cmd.bb_cmd       = &bb_cmd;
    ctx_cmd.running      = &g_running;

    th[0].name = "sim";     th[0].priority = CFG_PRIO_SIM;
    th[0].entry = th_sim_run;      th[0].arg = &ctx_sim;      th[0].cpu = -1;
    th[1].name = "gait";    th[1].priority = CFG_PRIO_GAIT;
    th[1].entry = th_gait_run;     th[1].arg = &ctx_gait;     th[1].cpu = -1;
    th[2].name = "estimator"; th[2].priority = CFG_PRIO_ESTIMATOR;
    th[2].entry = th_estimator_run; th[2].arg = &ctx_est;     th[2].cpu = -1;
    th[3].name = "watchdog"; th[3].priority = CFG_PRIO_WATCHDOG;
    th[3].entry = th_watchdog_run; th[3].arg = &ctx_wd;       th[3].cpu = -1;
    th[4].name = "command"; th[4].priority = CFG_PRIO_COMMAND;
    th[4].entry = th_command_run;  th[4].arg = &ctx_cmd;      th[4].cpu = -1;
    th[5].name = "logger";  th[5].priority = CFG_PRIO_LOGGER;
    th[5].entry = th_logger_run;   th[5].arg = &ctx_log;      th[5].cpu = -1;

    have_tty = (th_command_setup_terminal() == 0);

    printf("model   : %s\n", model);
    printf("gains   : kp=%.1f kd=%.1f      gait: T=%.2fs duty=%.2f h=%.2fm H=%.3fm\n",
           kp, kd, ctx_gait.params.period, ctx_gait.params.duty,
           ctx_gait.params.height, ctx_gait.params.clearance);
    if (have_tty) th_command_help();
    else          puts("  (stdin is not a terminal: keyboard control inactive)\n");

    /*Creazione dei thread*/
    for (int i = 0; i < 6; i++)
        if (rt_thread_start(&th[i]) != 0) { atomic_store(&g_running, 0); }

    have_window = (render_init(&rnd, sim.m) == 0);
    rnd.att_kp  = att_kp;

    t0 = t_prev = rt_now_ns();
    t_end = duration_s > 0 ? t0 + duration_s * RT_NS_PER_SEC : INT64_MAX;

    while (atomic_load(&g_running) && rt_now_ns() < t_end) {
        cmd_t    cmd_view;
        safety_t safe_view;
        int64_t  now;

        if (have_window && render_should_close(&rnd)) break;

        if (!bb_state_tryread(&bb_state, &st)) busy_skips++;
        bb_cmd_read(&bb_cmd, &cmd_view);
        bb_safe_read(&bb_safe, &safe_view);

        now = rt_now_ns();
        if (!have_origin && st.seq > 0) {
            sim_time_0  = st.sim_time;
            t0          = now;
            t_end       = duration_s > 0 ? t0 + duration_s * RT_NS_PER_SEC : INT64_MAX;
            have_origin = 1;
        }
        if (have_origin && now > t0)
            rt_factor = (st.sim_time - sim_time_0)
                      / ((double)(now - t0) / (double)RT_NS_PER_SEC);

        if (st.seq > 0) {
            /* Solo telemetria: il controllore usa l'assetto stimato dal
               filtro, non questa verita' di simulazione. */
            double r, p;
            quaternion_to_rpy(st.base_quat, &r, &p, NULL);
            rt_acc_add(&roll_mrad,  (int64_t)(1e3 * r));
            rt_acc_add(&pitch_mrad, (int64_t)(1e3 * p));
        }

        if (have_window) {
            render_frame(&rnd, &st, &cmd_view, &safe_view, rt_factor);
            rt_acc_add(&frame_dt, now - t_prev);
            frames++;
        } else {
            rt_sleep_until(now + CFG_RENDER_PERIOD_NS);
        }
        t_prev = now;
    }

    atomic_store(&g_running, 0);
    for (int i = 0; i < 6; i++) rt_thread_join(&th[i]);
    th_command_restore_terminal();

    puts("");
    rt_periodic_report_header(stdout);
    rt_periodic_report(&ctx_sim.per,  "sim",     stdout);
    rt_periodic_report(&ctx_est.per,  "estimator", stdout);
    rt_periodic_report(&ctx_gait.per, "gait",    stdout);
    rt_periodic_report(&ctx_wd.per,   "watchdog", stdout);
    rt_periodic_report(&ctx_cmd.per,  "command", stdout);
    rt_periodic_report(&ctx_log.per,  "logger",  stdout);

    puts("");
    rt_acc_report_us(&bb_state.wait, "bb_state lock wait", stdout);
    rt_acc_report_us(&bb_des.wait,   "bb_des lock wait",   stdout);
    rt_acc_report_us(&bb_cmd.wait,   "bb_cmd lock wait",   stdout);
    rt_acc_report_us(&bb_imu.wait,   "bb_imu lock wait",   stdout);
    rt_acc_report_us(&bb_att.wait,   "bb_att lock wait",   stdout);
    rt_acc_report_us(&bb_safe.wait,  "bb_safe lock wait",  stdout);
    rt_acc_report_us(&ctx_log.write_ns, "logger fprintf",   stdout);

    printf("\n%-22s kp=%.2f gate=%.1f   %llu of %llu updates gated\n", "att filter",
           CFG_ATT_KP, CFG_ATT_GATE,
           (unsigned long long)ctx_est.filter.n_gated,
           (unsigned long long)ctx_est.filter.n_updates);
    {
        att_t a; bb_att_read(&bb_att, &a);
        double rt, pt, yt;
        quaternion_to_rpy(st.base_quat, &rt, &pt, &yt);
        printf("%-22s est  r %+.4f  p %+.4f  y %+.4f   q %+.3f %+.3f %+.3f %+.3f\n",
               "attitude", a.roll, a.pitch, a.yaw, a.q[0], a.q[1], a.q[2], a.q[3]);
        printf("%-22s true r %+.4f  p %+.4f  y %+.4f   q %+.3f %+.3f %+.3f %+.3f\n",
               "", rt, pt, yt, st.base_quat[0], st.base_quat[1], st.base_quat[2], st.base_quat[3]);
    }

    printf("\n%-22s %llu policy, %llu real-time\n", "threads",
           6ULL, (unsigned long long)(th[0].is_rt + th[1].is_rt + th[2].is_rt
                                     + th[3].is_rt + th[4].is_rt + th[5].is_rt));
    {
        safety_t sf; bb_safe_read(&bb_safe, &sf);
        printf("%-22s alarms mask 0x%02x\n", "watchdog", sf.alarms);
    }
    printf("%-22s non-finite %llu\n", "torque",
           (unsigned long long)sim.n_nonfinite);

    printf("\nattitude      roll  %+.1f .. %+.1f mrad (avg %+.1f)\n",
           (double)roll_mrad.min, (double)roll_mrad.max, rt_acc_mean(&roll_mrad));
    printf("              pitch %+.1f .. %+.1f mrad (avg %+.1f)\n",
           (double)pitch_mrad.min, (double)pitch_mrad.max, rt_acc_mean(&pitch_mrad));
    {
        double yaw;
        quaternion_to_rpy(st.base_quat, NULL, NULL, &yaw);
        printf("final pose    x %.3f  y %.3f  z %.3f m   yaw %+.2f rad\n",
               st.base_pos[0], st.base_pos[1], st.base_pos[2], yaw);
    }
    printf("real-time     %.4f", rt_factor);
    if (frame_dt.n > 0)
        printf("   render %.1f fps", (double)RT_NS_PER_SEC / rt_acc_mean(&frame_dt));
    printf("   (%llu tryread skips over %llu frames)\n",
           (unsigned long long)busy_skips, (unsigned long long)frames);

    sim_free(&sim);
    bb_state_destroy(&bb_state);
    bb_cmd_destroy(&bb_cmd);
    bb_des_destroy(&bb_des);
    bb_imu_destroy(&bb_imu);
    bb_att_destroy(&bb_att);
    bb_safe_destroy(&bb_safe);
    if (have_window) render_free(&rnd);
    return 0;
}
