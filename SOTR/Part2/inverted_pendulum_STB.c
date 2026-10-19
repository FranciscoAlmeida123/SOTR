/* *********************************************************************
 * SOTR 26/27
 * 
 * Linux RT Services - base code
 * 
 * Paulo Pedreiras, /Sept 2026
 * 
 * This code implements a simulation of an Inverted Pendulum 
 * It contains 3 tasks:
 * 	- THREAD 1: Plant Physics Simulator (Runs at 1 kHz / 1 ms period) 
 * 	- THREAD 2: Real-Time Feedback Controller (Runs at 200 Hz / 5 ms period)  
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
#include <semaphore.h>
#include <string.h>

#define NSEC_PER_SEC 1000000000L

/* System Physical Parameters */
#define M_CART  1.0f   /* Cart mass (kg) */
#define M_POLE  0.1f   /* Pole mass (kg) */
#define LENGTH  0.5f   /* Half-pole length (m) */
#define GRAVITY 9.81f  /* Gravity (m/s^2) */

/* Visualization settings */
#define DISPLAY_WIDTH 60
#define POLE_CHAR_LEN 6

/*Shared time origin for the 3 threads*/
static struct timespec control_epoch;


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
static mqd_t sensor_queue = (mqd_t)-1;
static mqd_t force_queue = (mqd_t)-1;
/* ADDED: keep the original program layout; these helpers configure RT threads. */
static bool normal_mode;
static int run_seconds = 60;
static void check_thread(int error, const char *what)
{
    if (error) {
        errno = error;
        if (error == EPERM)
            fprintf(stderr, "RT permission required: use sudo, or --normal for testing.\n");
        perror(what);
        exit(EXIT_FAILURE);
    }
}
static void configure_run(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--normal") == 0) normal_mode = true;
        else if (strcmp(argv[i], "--seconds") == 0 && i+1 < argc) {
            char *end;
            long value = strtol(argv[++i], &end, 10);
            if (end == argv[i] || *end || value < 1 || value > 60) {
                fprintf(stderr, "Seconds must be 1..60\n"); exit(EXIT_FAILURE);
            }
            run_seconds = (int)value;
        } else {
            fprintf(stderr, "Usage: %s [--normal] [--seconds 1..60]\n", argv[0]);
            exit(EXIT_FAILURE);
        }
    }
    /* All subsequently created threads inherit this one-CPU affinity. */
    cpu_set_t allowed, selected;
    if (sched_getaffinity(0, sizeof(allowed), &allowed)) {perror("get affinity");exit(EXIT_FAILURE);}
    int cpu;
    for (cpu=0; cpu<CPU_SETSIZE; cpu++) if (CPU_ISSET(cpu,&allowed)) break;
    if (cpu==CPU_SETSIZE) {fprintf(stderr,"No allowed CPU\n");exit(EXIT_FAILURE);}
    CPU_ZERO(&selected); CPU_SET(cpu,&selected);
    if (sched_setaffinity(0,sizeof(selected),&selected)) {perror("set affinity");exit(EXIT_FAILURE);}
}
static void create_task(pthread_t *tid, void *(*entry)(void *), int priority)
{
    pthread_attr_t attr;
    check_thread(pthread_attr_init(&attr), "thread attributes");
    check_thread(pthread_attr_setinheritsched(&attr,PTHREAD_EXPLICIT_SCHED),"explicit scheduling");
    int policy = !normal_mode && priority ? SCHED_FIFO : SCHED_OTHER;
    check_thread(pthread_attr_setschedpolicy(&attr,policy),"thread policy");
    struct sched_param p = {.sched_priority = policy==SCHED_FIFO ? priority : 0};
    check_thread(pthread_attr_setschedparam(&attr,&p),"thread priority");
    check_thread(pthread_create(tid,&attr,entry,NULL),"pthread_create");
    check_thread(pthread_attr_destroy(&attr),"thread attributes destroy");
}

/* ADDED 4.5: a Tick thread releases the existing task threads from a table. */
static sem_t plant_release, sensor_release, controller_release;
static sem_t actuator_release, display_release;
static pthread_barrier_t startup;
enum { PLANT_BIT=1, SENSOR_BIT=2, CONTROLLER_BIT=4, ACTUATOR_BIT=8, DISPLAY_BIT=16 };
#define TABLE_SLOTS 165 /* lcm(1 ms, 5 ms, 33 ms). */
static unsigned schedule_table[TABLE_SLOTS];
static void enter_scheduler(void)
{
    int e=pthread_barrier_wait(&startup);
    if(e!=PTHREAD_BARRIER_SERIAL_THREAD)check_thread(e,"startup barrier");
}
static void build_schedule(void)
{
    for(unsigned slot=0;slot<TABLE_SLOTS;slot++) {
        schedule_table[slot]=PLANT_BIT;
        if(slot%5==0)schedule_table[slot]|=SENSOR_BIT;
        if(slot%5==1)schedule_table[slot]|=CONTROLLER_BIT;
        if(slot%5==2)schedule_table[slot]|=ACTUATOR_BIT;
        if(slot%33==0)schedule_table[slot]|=DISPLAY_BIT;
    }
}
static bool wait_for_tick(sem_t *release)
{
    while(g_running) {
        struct timespec timeout;
        clock_gettime(CLOCK_REALTIME,&timeout);
        timeout.tv_nsec+=100000000L;
        if(timeout.tv_nsec>=NSEC_PER_SEC) {timeout.tv_nsec-=NSEC_PER_SEC;timeout.tv_sec++;}
        if(sem_timedwait(release,&timeout)==0)return g_running;
        if(errno!=EINTR && errno!=ETIMEDOUT) {perror("sem_timedwait");g_running=false;}
    }
    return false;
}
static void release_task(sem_t *release)
{
    if(sem_post(release)) {perror("sem_post");g_running=false;}
}



/* Helper: Add nanoseconds to timespec */
static void timespec_add_ns(struct timespec *t, long ns) {
    t->tv_nsec += ns;
    while (t->tv_nsec >= NSEC_PER_SEC) {
        t->tv_nsec -= NSEC_PER_SEC;
        t->tv_sec++;
    }
}

/* Sleep until an absolute release time; retry interrupted sleeps. */
static bool wait_release(const struct timespec *release)
{
    int error;

    do {
        error = clock_nanosleep(
            CLOCK_MONOTONIC, TIMER_ABSTIME, release, NULL);
    } while (error == EINTR && g_running);

    if (error != 0) {
        errno = error;
        perror("clock_nanosleep");
        g_running = false;
        return false;
    }

    return g_running;
}

static struct timespec queue_timeout(void)
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);

    t.tv_nsec += 100000000L; /* 100 ms */

    if (t.tv_nsec >= NSEC_PER_SEC) {
        t.tv_nsec -= NSEC_PER_SEC;
        t.tv_sec++;
    }

    return t;
}

static bool send_message(mqd_t queue,
                         const void *message, size_t size)
{
    while (g_running) {
        struct timespec timeout = queue_timeout();

        if (mq_timedsend(queue, (const char *)message,
                         size, 0, &timeout) == 0)
            return true;

        if (errno == EINTR || errno == ETIMEDOUT)
            continue;

        perror("mq_timedsend");
        g_running = false;
        break;
    }

    return false;
}


/* -------------------------------------------------------------------------- */
/* THREAD 1: Plant Physics Simulator (Runs at 1 kHz / 1 ms period)           */
/* -------------------------------------------------------------------------- */
void* physics_plant_thread(void *arg) {
    (void)arg;

    const float dt = 0.001f;
    float u=0, th=0, alpha = 0, acceleration = 0; // Auxiliary variables         		        

    enter_scheduler();

	// Loop, triggered every 1 ms
    while (g_running) {        
		// Wait for next activation
        /* CHANGED: Tick's table, rather than a private timer, releases this job. */
        if (!wait_for_tick(&plant_release)) break;

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
    }
    return NULL;
}

//* -------------------------------------------------------------------------- */
/* THREAD 2: Feedback Controller (Runs at 200 Hz / 5 ms period)     */
/* -------------------------------------------------------------------------- */

//Sensor thread
void *sensor_thread(void *arg)
{
    (void)arg;

    enter_scheduler();

    while (g_running) {
    
    	/* CHANGED: consume a scheduled release token, not a data-ready event. */
        if (!wait_for_tick(&sensor_release)) break;

        /* Preserve the original sensor noise. */
        int random_num = -50 + rand() % 101;
        float noise = (float)random_num / 1500;

        PlantState sample;

        pthread_mutex_lock(&g_state_mutex);
        sample = g_state;
        pthread_mutex_unlock(&g_state_mutex);

        /* Modify the local sample, not the simulated plant. */
        sample.theta += noise;

        if (!send_message(sensor_queue,
                          &sample, sizeof(sample)))
            break;
    }

    return NULL;
}

//Controll thread
void *controller_thread(void *arg)
{
    (void)arg;

    const float K_theta     = 45.0f;
    const float K_theta_dot = 10.0f;
    const float K_x         = 2.0f;
    const float K_x_dot     = 4.0f;

    enter_scheduler();

    while (g_running) {
        /* CHANGED: consume a scheduled release token, not a data-ready event. */
        if (!wait_for_tick(&controller_release)) break;

        PlantState sample;
        ssize_t n = mq_receive(sensor_queue,
                               (char *)&sample,
                               sizeof(sample), NULL);

        if (n == -1) {
            /* No sample available: skip this job without waiting. */
            if (errno == EAGAIN || errno == EINTR)
                continue;

            perror("Controller mq_receive");
            g_running = false;
            break;
        }

        if (n != (ssize_t)sizeof(sample)) {
            g_running = false;
            break;
        }

        float force =
            K_theta     * sample.theta
          + K_theta_dot * sample.theta_dot
          + K_x         * sample.x
          + K_x_dot     * sample.x_dot;

        if (!send_message(force_queue, &force, sizeof(force)))
            break;
    }

    return NULL;
}
//Actuator thread
void *actuator_thread(void *arg)
{
    (void)arg;

    enter_scheduler();

    while (g_running) {
        /* CHANGED: consume a scheduled release token, not a data-ready event. */
        if (!wait_for_tick(&actuator_release)) break;

        float force;
        ssize_t n = mq_receive(force_queue,
                               (char *)&force,
                               sizeof(force), NULL);

        if (n == -1) {
            /* No new force: retain the previously applied force. */
            if (errno == EAGAIN || errno == EINTR)
                continue;

            perror("Actuator mq_receive");
            g_running = false;
            break;
        }

        if (n != (ssize_t)sizeof(force)) {
            g_running = false;
            break;
        }

        pthread_mutex_lock(&g_state_mutex);
        g_state.force = force;
        pthread_mutex_unlock(&g_state_mutex);
    }

    return NULL;
}

/* -------------------------------------------------------------------------- */
/* THREAD 3: ASCII Graphics Display (Low-Priority, ~30 Hz / 33 ms period)      */
/* -------------------------------------------------------------------------- */
void* display_thread(void *arg) {
	(void)arg;


    enter_scheduler();

    while (g_running) {
		// Wait for next activation
        if (!wait_for_tick(&display_release)) break;

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
		printf("================ INVERTED PENDULUM (4.5 TABLE + MQ) ================\n");

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
    }
    return NULL;
}

/* -------------------------------------------------------------------------- */
/* MAIN: Initialization & Thread Creation                                    */
/* -------------------------------------------------------------------------- */

/* ADDED 4.5 bullet 1: high-priority periodic Tick scans and wraps the static table. */
static void *tick_thread(void *arg)
{
    (void)arg;
    enter_scheduler();
    struct timespec next=control_epoch;
    unsigned slot=0;
    while(g_running) {
        if(!wait_release(&next))break;
        unsigned tasks=schedule_table[slot];
        if(tasks&PLANT_BIT)release_task(&plant_release);
        if(tasks&SENSOR_BIT)release_task(&sensor_release);
        if(tasks&CONTROLLER_BIT)release_task(&controller_release);
        if(tasks&ACTUATOR_BIT)release_task(&actuator_release);
        if(tasks&DISPLAY_BIT)release_task(&display_release);
        slot=(slot+1)%TABLE_SLOTS;
        timespec_add_ns(&next,1000000L); /* 1 ms micro-cycle. */
    }
    return NULL;
}

int main(int argc, char **argv) {
    configure_run(argc, argv);

	/* Optional arguments select functional-test mode and run duration. */
    pthread_t plant_thread, sensor_tid, controller_tid, actuator_tid, gui_thread;    
    pthread_t tick_tid;
        
    // 1. Seed the random number generator using current time
	srand((unsigned int)time(NULL));


    // 2. Initialize Mutex
    pthread_mutex_init(&g_state_mutex, NULL);
    
   //Create the queues
char sensor_name[64], force_name[64];

snprintf(sensor_name, sizeof(sensor_name),
         "/pendulum_sensor_%ld", (long)getpid());

snprintf(force_name, sizeof(force_name),
         "/pendulum_force_%ld", (long)getpid());

struct mq_attr queue_attr = {
    .mq_maxmsg = 10,
    .mq_msgsize = sizeof(PlantState)
};

sensor_queue = mq_open(
    sensor_name, O_CREAT | O_EXCL | O_RDWR, 0600, &queue_attr);

if (sensor_queue == (mqd_t)-1) {
    perror("sensor mq_open");
    return EXIT_FAILURE;
}

/* Remove the name; the open queue remains usable. */
if (mq_unlink(sensor_name) == -1) {
    perror("sensor mq_unlink");
    return EXIT_FAILURE;
}

queue_attr.mq_msgsize = sizeof(float);

force_queue = mq_open(
    force_name, O_CREAT | O_EXCL | O_RDWR, 0600, &queue_attr);


/* Check creation before using the queue descriptor. */
if (force_queue == (mqd_t)-1) {
    perror("force mq_open");
    mq_close(sensor_queue);
    pthread_mutex_destroy(&g_state_mutex);
    return EXIT_FAILURE;
}

/* Remove the name once; the open descriptor remains usable. */
if (mq_unlink(force_name) == -1) {
    perror("force mq_unlink");
    mq_close(sensor_queue);
    mq_close(force_queue);
    pthread_mutex_destroy(&g_state_mutex);
    return EXIT_FAILURE;
}

/* Return immediately when a queue is empty or full. */
struct mq_attr nonblocking = {
    .mq_flags = O_NONBLOCK
};

if (mq_setattr(sensor_queue, &nonblocking, NULL) == -1 ||
    mq_setattr(force_queue, &nonblocking, NULL) == -1) {
    perror("mq_setattr");
    mq_close(sensor_queue);
    mq_close(force_queue);
    pthread_mutex_destroy(&g_state_mutex);
    return EXIT_FAILURE;
}

    /* ADDED: initialize scheduled-release semaphores before creating workers. */
    sem_t *releases[]={&plant_release,&sensor_release,&controller_release,
                      &actuator_release,&display_release};
    for(int i=0;i<5;i++)if(sem_init(releases[i],0,0)) {perror("sem_init");exit(EXIT_FAILURE);}
    build_schedule();
    check_thread(pthread_barrier_init(&startup,NULL,7),"startup barrier init");
    /* Tick has higher priority than every worker. Queue data flow is retained. */
    create_task(&plant_thread,physics_plant_thread,60);
    create_task(&sensor_tid,sensor_thread,50);
    create_task(&controller_tid,controller_thread,40);
    create_task(&actuator_tid,actuator_thread,30);
    create_task(&gui_thread,display_thread,0);
    create_task(&tick_tid,tick_thread,70);
    clock_gettime(CLOCK_MONOTONIC,&control_epoch);
    timespec_add_ns(&control_epoch,20000000L);
    enter_scheduler(); /* All six workers exist before Tick begins. */

    /* Run simulation for 60 seconds */
    sleep(run_seconds);

    /* Cleanup */
    g_running = false;
    pthread_join(tick_tid, NULL);
    pthread_join(plant_thread, NULL);
    pthread_join(sensor_tid, NULL);
    pthread_join(controller_tid, NULL);
    pthread_join(actuator_tid, NULL);
    pthread_join(gui_thread, NULL);
    
    /* ADDED: destroy table-release synchronization after all workers stop. */
    for(int i=0;i<5;i++)sem_destroy(releases[i]);
    pthread_barrier_destroy(&startup);

    //Close the queues
    mq_close(sensor_queue);
    mq_close(force_queue);

    pthread_mutex_destroy(&g_state_mutex);

    printf("\033[H\033[JSimulation complete.\n");
    return EXIT_SUCCESS;
}
