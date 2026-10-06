/* *********************************************************************
 * SOTR 26/27
 * 
 * Linux RT Services - base code
 * 
 * Paulo Pedreiras, /Sept 2026
 * 
 * This code implements a simulation of an Inverted Pendulum 
 * Tasks (control loop split into Sensor, Controller and Actuator):
 * 	- THREAD 1: Plant Physics Simulator (Runs at 1 kHz / 1 ms period) 
 * 	- Sensor (200 Hz), Controller and Actuator (message-triggered)
 * 	- THREAD 3: ASCII Graphics Display (Low-Priority, ~30 Hz / 33 ms period)  
 * 	
 * The code is supposed to work "out-of-the-box".
 * The task periods could be different. Note however that changing  
 * them may require adjusting the controller, which is out of the 
 * scope of this course unit. So, keep the task periods consistent with
 * with the rates provided in this code.
 * 
 * Report any bugs to:
 * 	Paulo Pedreiras, pbrp@ua.pt
 * *********************************************************************/

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <math.h>
#include <unistd.h>
#include <mqueue.h>
#include <fcntl.h>
#include <errno.h>
#include <stdatomic.h>
#include <string.h>
#include <signal.h>
#include <limits.h>
#include <sys/syscall.h>

#define NSEC_PER_SEC 1000000000L

/* System Physical Parameters */
#define M_CART  1.0f   /* Cart mass (kg) */
#define M_POLE  0.1f   /* Pole mass (kg) */
#define LENGTH  0.5f   /* Half-pole length (m) */
#define GRAVITY 9.81f  /* Gravity (m/s^2) */

/* Visualization settings */
#define DISPLAY_WIDTH 60
#define POLE_CHAR_LEN 6

/* Shared State Buffer Structure */
typedef struct {
    float theta;       /* Pendulum angle (rad), 0 = upright */    
    float theta_dot;   /* Angular velocity (rad/s) */
    float x;           /* Cart position (m) */
    float x_dot;       /* Cart velocity (m/s) */
    float force;       /* Control force (N) */
} PlantState;

static PlantState g_state = { .theta = 0.15f, .theta_dot = 0.0f, .x = 0.0f, .x_dot = 0.0f, .force = 0.0f };
static pthread_mutex_t g_state_mutex;
static atomic_bool g_running = true;
static mqd_t sensor_queue = (mqd_t)-1, force_queue = (mqd_t)-1;
static bool use_rt = true;
static int run_seconds = 60;
static sigset_t stop_signals;

enum { PLANT, SENSOR, CONTROLLER, ACTUATOR, DISPLAY, TASK_COUNT };
static const char *task_names[] = {
    "plant", "sensor", "controller", "actuator", "display"
};
static int priorities[] = { 80, 70, 60, 50, 0 };

/* One writer per buffer. No file I/O or allocation in the measured jobs. */
#define MAX_JOBS 100000
typedef struct {
    unsigned long sequence;
    long tid;
    int priority;
    long long release, start, finish, cpu, deadline;
} Job;
static Job jobs[TASK_COUNT][MAX_JOBS];
static size_t job_count[TASK_COUNT], dropped[TASK_COUNT];
typedef struct {
    float theta, theta_dot, x, x_dot;
    unsigned long sequence;
    long long release;
} SensorSample;
typedef struct {
    float force;
    unsigned long sequence;
    long long release;
} ForceMessage;

static long long now_ns(clockid_t clock) {
    struct timespec t;
    clock_gettime(clock, &t);
    return (long long)t.tv_sec * NSEC_PER_SEC + t.tv_nsec;
}
static long long ns_of(struct timespec t) {
    return (long long)t.tv_sec * NSEC_PER_SEC + t.tv_nsec;
}
static void record_job(int task, unsigned long sequence, long long release,
                       long long start, long long cpu_start, long long deadline) {
    long long finish = now_ns(CLOCK_MONOTONIC);
    long long cpu = now_ns(CLOCK_THREAD_CPUTIME_ID) - cpu_start;
    if (job_count[task] == MAX_JOBS) { dropped[task]++; return; }
    jobs[task][job_count[task]++] = (Job){
        sequence, syscall(SYS_gettid), priorities[task],
        release, start, finish, cpu, deadline
    };
}
/* Timed queue operations allow clean shutdown of blocked tasks. */
static struct timespec queue_deadline(void) {
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    t.tv_nsec += 100000000L;
    if (t.tv_nsec >= NSEC_PER_SEC) { t.tv_nsec -= NSEC_PER_SEC; t.tv_sec++; }
    return t;
}
static bool queue_send(mqd_t q, const void *data, size_t size) {
    while (g_running) {
        struct timespec t = queue_deadline();
        if (mq_timedsend(q, data, size, 0, &t) == 0) return true;
        if (errno != EINTR && errno != ETIMEDOUT) {
            perror("mq_timedsend"); g_running = false; break;
        }
    }
    return false;
}
static bool queue_receive(mqd_t q, void *data, size_t size) {
    while (g_running) {
        struct timespec t = queue_deadline();
        ssize_t n = mq_timedreceive(q, data, size, NULL, &t);
        if (n == (ssize_t)size) return true;
        if (n >= 0 || (errno != EINTR && errno != ETIMEDOUT)) {
            if (n >= 0) fprintf(stderr, "Invalid queue message size\n");
            else perror("mq_timedreceive");
            g_running = false; break;
        }
    }
    return false;
}

/* Helper: Add nanoseconds to timespec */
static void timespec_add_ns(struct timespec *t, long ns) {
    t->tv_nsec += ns;
    while (t->tv_nsec >= NSEC_PER_SEC) {
        t->tv_nsec -= NSEC_PER_SEC;
        t->tv_sec++;
    }
}

/* -------------------------------------------------------------------------- */
/* THREAD 1: Plant Physics Simulator (Runs at 1 kHz / 1 ms period)           */
/* -------------------------------------------------------------------------- */
void* physics_plant_thread(void *arg) {
    (void)arg;
    unsigned long sequence = 0;
    struct timespec next_period;
    const long period_ns = 1000000L; /* 1 ms */
    const float dt = 0.001f;
    float u=0, th=0, alpha = 0, acceleration = 0; // Auxiliary variables         		        

    clock_gettime(CLOCK_MONOTONIC, &next_period);

	// Loop, triggered every 1 ms
    while (g_running) {        
		// Wait for next activation
        timespec_add_ns(&next_period, period_ns);        
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
        if (!g_running) break;
        long long start = now_ns(CLOCK_MONOTONIC);
        long long cpu_start = now_ns(CLOCK_THREAD_CPUTIME_ID);

		// Plant state is a structure shared by diverse tasks. Ensure mutual exclusion 
        pthread_mutex_lock(&g_state_mutex);

        u = g_state.force;
        th = g_state.theta;         		

        /* Linearized Pendulum Equations of Motion near upright equilibrium */
        alpha = ((M_CART + M_POLE) * GRAVITY * th - u) / (M_CART * LENGTH);
        acceleration = (u - M_POLE * LENGTH * alpha) / (M_CART + M_POLE);

        /* Euler Integration */
        g_state.theta_dot += alpha * dt;
        g_state.theta     += g_state.theta_dot * dt;
        g_state.x_dot     += acceleration * dt;
        g_state.x         += g_state.x_dot * dt;

        pthread_mutex_unlock(&g_state_mutex);
        record_job(PLANT, sequence++, ns_of(next_period), start,
                   cpu_start, ns_of(next_period) + 1000000LL);
    }
    return NULL;
}

/* Sensor: time-triggered sampling at 200 Hz / 5 ms. */
void *sensor_thread(void *arg) {
    (void)arg;
    struct timespec next_period;
    unsigned long sequence = 0;
    clock_gettime(CLOCK_MONOTONIC, &next_period);
    while (g_running) {
        timespec_add_ns(&next_period, 5000000L);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
        if (!g_running) break;
        long long start = now_ns(CLOCK_MONOTONIC);
        long long cpu_start = now_ns(CLOCK_THREAD_CPUTIME_ID);
        int random_num = -50 + rand() % 101;
        float noise = (float)random_num / 1500;
        pthread_mutex_lock(&g_state_mutex);
        SensorSample sample = {
            g_state.theta + noise, g_state.theta_dot, g_state.x, g_state.x_dot,
            sequence, ns_of(next_period)
        };
        pthread_mutex_unlock(&g_state_mutex);
        bool sent = queue_send(sensor_queue, &sample, sizeof(sample));
        record_job(SENSOR, sequence++, sample.release, start, cpu_start,
                   sample.release + 5000000LL);
        if (!sent) break;
    }
    return NULL;
}

/* Controller: sporadic activation on receipt of a sensor message. */
void *controller_thread(void *arg) {
    (void)arg;
    const float K_theta = 45.0f, K_theta_dot = 10.0f;
    const float K_x = 2.0f, K_x_dot = 4.0f;
    SensorSample sample;
    while (queue_receive(sensor_queue, &sample, sizeof(sample))) {
        long long start = now_ns(CLOCK_MONOTONIC);
        long long cpu_start = now_ns(CLOCK_THREAD_CPUTIME_ID);
        ForceMessage message = {
            K_theta * sample.theta + K_theta_dot * sample.theta_dot +
            K_x * sample.x + K_x_dot * sample.x_dot,
            sample.sequence, sample.release
        };
        bool sent = queue_send(force_queue, &message, sizeof(message));
        record_job(CONTROLLER, sample.sequence, sample.release, start, cpu_start,
                   sample.release + 5000000LL);
        if (!sent) break;
    }
    return NULL;
}

/* Actuator: sporadic activation on receipt of a force message. */
void *actuator_thread(void *arg) {
    (void)arg;
    ForceMessage message;
    while (queue_receive(force_queue, &message, sizeof(message))) {
        long long start = now_ns(CLOCK_MONOTONIC);
        long long cpu_start = now_ns(CLOCK_THREAD_CPUTIME_ID);
        pthread_mutex_lock(&g_state_mutex);
        g_state.force = message.force;
        pthread_mutex_unlock(&g_state_mutex);
        record_job(ACTUATOR, message.sequence, message.release, start, cpu_start,
                   message.release + 5000000LL);
    }
    return NULL;
}

/* -------------------------------------------------------------------------- */
/* THREAD 3: ASCII Graphics Display (Low-Priority, ~30 Hz / 33 ms period)      */
/* -------------------------------------------------------------------------- */
void* display_thread(void *arg) {
    (void)arg;
    unsigned long sequence = 0;
    struct timespec next_period;
    const long period_ns = 33000000L; /* ~33 ms */

    clock_gettime(CLOCK_MONOTONIC, &next_period);

    while (g_running) {
		// Wait for next activation
        timespec_add_ns(&next_period, period_ns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
        if (!g_running) break;
        long long start = now_ns(CLOCK_MONOTONIC);
        long long cpu_start = now_ns(CLOCK_THREAD_CPUTIME_ID);

        // Get a copy of state data 
        pthread_mutex_lock(&g_state_mutex);
        float th = g_state.theta;
        float x  = g_state.x;
        float f  = g_state.force;
        pthread_mutex_unlock(&g_state_mutex);

        /* Map cart x position [-2.0m, +2.0m] to terminal column index */
        int cart_col = (int)((x + 2.0f) / 4.0f * DISPLAY_WIDTH);
        if (cart_col < 2) cart_col = 2;
        if (cart_col > DISPLAY_WIDTH - 3) cart_col = DISPLAY_WIDTH - 3;

        /* Calculate tip offset based on angle theta */
        int tip_offset = (int)(sinf(th) * POLE_CHAR_LEN * 2.5f);
        int tip_col = cart_col + tip_offset;
        if (tip_col < 0) tip_col = 0;
        if (tip_col >= DISPLAY_WIDTH) tip_col = DISPLAY_WIDTH - 1;

        /* Render Frame to Terminal */ 
        printf("\033[H\033[J"); /* Clear screen and reset cursor */
		printf("================ INVERTED PENDULUM (TT + ET) ================\n");

        printf(" Theta: %+.4f rad | Cart Pos: %+.3f m | Force: %+.2f N\n", th, x, f);
        printf("--------------------------------------------------------------\n\n");

        /* Draw Pendulum Tip */
        for (int i = 0; i < DISPLAY_WIDTH; i++) {
            if (i == tip_col) printf("O");
            else printf(" ");
        }
        printf("\n");

        /* Draw Pole Body */
        for (int line = 0; line < 3; line++) {
            int body_col = cart_col + (tip_offset * (3 - line) / 4);
            for (int i = 0; i < DISPLAY_WIDTH; i++) {
                if (i == body_col) printf("|");
                else printf(" ");
            }
            printf("\n");
        }

        /* Draw Cart Base */
        for (int i = 0; i < DISPLAY_WIDTH; i++) {
            if (i == cart_col - 2) printf("[");
            else if (i == cart_col + 2) printf("]");
            else if (i > cart_col - 2 && i < cart_col + 2) printf("=");
            else printf(" ");
        }
        printf("\n");

        /* Draw Track Floor */
        for (int i = 0; i < DISPLAY_WIDTH; i++) {
            if (i == cart_col - 1 || i == cart_col + 1) printf("o");
            else printf("-");
        }
        printf("\n\nPress Ctrl+C to terminate.\n");
        fflush(stdout);
        record_job(DISPLAY, sequence++, ns_of(next_period), start,
                   cpu_start, ns_of(next_period) + 33000000LL);
    }
    return NULL;
}

/* -------------------------------------------------------------------------- */
/* MAIN: Initialization & Thread Creation                                    */
/* -------------------------------------------------------------------------- */
/* All workers get names for KernelShark and explicit scheduling attributes. */
static int create_task(pthread_t *thread, int task, void *(*entry)(void *)) {
    pthread_attr_t attr;
    struct sched_param param = { .sched_priority = priorities[task] };
    int error = pthread_attr_init(&attr);
    if (error) return error;
    error = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (!error) error = pthread_attr_setschedpolicy(&attr,
                         use_rt && task != DISPLAY ? SCHED_FIFO : SCHED_OTHER);
    if (!use_rt || task == DISPLAY) param.sched_priority = 0;
    if (!error) error = pthread_attr_setschedparam(&attr, &param);
    if (!error) error = pthread_create(thread, &attr, entry, NULL);
    pthread_attr_destroy(&attr);
    if (!error) pthread_setname_np(*thread, task_names[task]);
    return error;
}
static bool parse_int(const char *text, int *value) {
    char *end;
    errno = 0;
    long n = strtol(text, &end, 10);
    if (errno || end == text || *end || n < 0 || n > INT_MAX) return false;
    *value = (int)n;
    return true;
}
static bool export_logs(void) {
    FILE *f = fopen("timings.csv", "w");
    if (!f) { perror("timings.csv"); return false; }
    fprintf(f, "task,tid,priority,sequence,release_ns,start_ns,finish_ns,cpu_ns,deadline_ns\n");
    for (int t = 0; t < TASK_COUNT; t++) {
        for (size_t i = 0; i < job_count[t]; i++) {
            Job *j = &jobs[t][i];
            fprintf(f, "%s,%ld,%d,%lu,%lld,%lld,%lld,%lld,%lld\n",
                    task_names[t], j->tid, j->priority, j->sequence, j->release,
                    j->start, j->finish, j->cpu, j->deadline);
        }
        if (dropped[t]) fprintf(stderr, "%s: %zu records dropped\n", task_names[t], dropped[t]);
    }
    bool ok = !ferror(f);
    if (fclose(f)) ok = false;
    return ok;
}
int main(int argc, char *argv[]) {
    /* Usage: ./inverted_pendulum [--normal] [seconds] */
    int first = 1;
    if (argc > 1 && strcmp(argv[1], "--normal") == 0) { use_rt = false; first++; }
    if (argc - first > 1) goto usage;
    if (argc > first && !parse_int(argv[first], &run_seconds)) goto usage;
    if (run_seconds < 1 || run_seconds > 60) goto usage;
    if (!use_rt) for (int i = 0; i < TASK_COUNT; i++) priorities[i] = 0;

    /* Main waits for termination signals; workers inherit this blocked set. */
    sigemptyset(&stop_signals);
    sigaddset(&stop_signals, SIGINT);
    sigaddset(&stop_signals, SIGTERM);
    int error = pthread_sigmask(SIG_BLOCK, &stop_signals, NULL);
    if (error) { errno = error; perror("pthread_sigmask"); return EXIT_FAILURE; }
    srand((unsigned int)time(NULL));
    pthread_mutexattr_t mutex_attr;
    error = pthread_mutexattr_init(&mutex_attr);
    if (error) { errno = error; perror("mutex attributes"); return EXIT_FAILURE; }
    error = pthread_mutexattr_setprotocol(&mutex_attr, PTHREAD_PRIO_INHERIT);
    if (!error) error = pthread_mutex_init(&g_state_mutex, &mutex_attr);
    pthread_mutexattr_destroy(&mutex_attr);
    if (error) { errno = error; perror("mutex init"); return EXIT_FAILURE; }

    int result = EXIT_FAILURE, created = 0;
    pthread_t threads[TASK_COUNT];
    void *(*entries[])(void *) = { physics_plant_thread, sensor_thread,
        controller_thread, actuator_thread, display_thread };
    char sensor_name[64], force_name[64];
    snprintf(sensor_name, sizeof(sensor_name), "/pendulum_sensor_%ld", (long)getpid());
    snprintf(force_name, sizeof(force_name), "/pendulum_force_%ld", (long)getpid());
    struct mq_attr attr = { .mq_maxmsg = 10, .mq_msgsize = sizeof(SensorSample) };
    sensor_queue = mq_open(sensor_name, O_CREAT | O_EXCL | O_RDWR, 0600, &attr);
    if (sensor_queue == (mqd_t)-1) { perror("sensor queue"); goto cleanup; }
    if (mq_unlink(sensor_name)) { perror("sensor unlink"); goto cleanup; }
    attr.mq_msgsize = sizeof(ForceMessage);
    force_queue = mq_open(force_name, O_CREAT | O_EXCL | O_RDWR, 0600, &attr);
    if (force_queue == (mqd_t)-1) { perror("force queue"); goto cleanup; }
    if (mq_unlink(force_name)) { perror("force unlink"); goto cleanup; }
    for (int i = 0; i < TASK_COUNT; i++) {
        error = create_task(&threads[i], i, entries[i]);
        if (error) {
            errno = error; perror(task_names[i]);
            if (error == EPERM) fprintf(stderr,
                "SCHED_FIFO needs RT permission. Use sudo or --normal for functional testing.\n");
            goto cleanup;
        }
        created++;
    }
    for (int i = 0; i < run_seconds && g_running; i++) {
        struct timespec timeout = { .tv_sec = 1 };
        int signal = sigtimedwait(&stop_signals, NULL, &timeout);
        if (signal == SIGINT || signal == SIGTERM) break;
    }
    result = g_running ? EXIT_SUCCESS : EXIT_FAILURE;
cleanup:
    g_running = false;
    for (int i = 0; i < created; i++) pthread_join(threads[i], NULL);
    if (sensor_queue != (mqd_t)-1) mq_close(sensor_queue);
    if (force_queue != (mqd_t)-1) mq_close(force_queue);
    pthread_mutex_destroy(&g_state_mutex);
    if (!export_logs()) result = EXIT_FAILURE;
    printf("\033[H\033[JSimulation complete. Log: timings.csv\n");
    return result;
usage:
    fprintf(stderr, "Usage: %s [--normal] [seconds: 1..60]\n", argv[0]);
    return EXIT_FAILURE;
}

