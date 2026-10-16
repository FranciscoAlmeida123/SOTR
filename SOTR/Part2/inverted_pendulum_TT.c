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

    struct timespec next_period=control_epoch;

    while (g_running) {
    
    	if (!wait_release(&next_period))
            break;
            	
        timespec_add_ns(&next_period, 5000000L);

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

    /* Release 1 ms after the Sensor, then every 5 ms. */
    struct timespec next_period = control_epoch;
    timespec_add_ns(&next_period, 1000000L);

    while (g_running) {
        if (!wait_release(&next_period))
            break;

        timespec_add_ns(&next_period, 5000000L);

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

    /* Release 2 ms after the Sensor, then every 5 ms. */
    struct timespec next_period = control_epoch;
    timespec_add_ns(&next_period, 2000000L);

    while (g_running) {
        if (!wait_release(&next_period))
            break;

        timespec_add_ns(&next_period, 5000000L);

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
    struct timespec next_period;
    const long period_ns = 33000000L; /* ~33 ms */

    clock_gettime(CLOCK_MONOTONIC, &next_period);

    while (g_running) {
		// Wait for next activation
        timespec_add_ns(&next_period, period_ns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);

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
		printf("================ INVERTED PENDULUM (Non-RT) ================\n");

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
int main(void) {

	//Extracted the arguments (int agrc, char *argv[]) because we are not using the command-line arguments
    pthread_t plant_thread, sensor_tid, controller_tid, actuator_tid, gui_thread;    
    pthread_attr_t thread_attr;
        
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

	/* Common release time, shortly after thread creation begins. */
	clock_gettime(CLOCK_MONOTONIC, &control_epoch);
	timespec_add_ns(&control_epoch, 20000000L); /* 20 ms */
	
    
    /* Initialize thread attributes before use. */
    pthread_attr_init(&thread_attr);
    // Physics Thread
    pthread_create(&plant_thread, &thread_attr, physics_plant_thread, NULL);

    // Controller Thread
    pthread_create(&sensor_tid, &thread_attr, sensor_thread, NULL); //Sensor Thread
    pthread_create(&controller_tid, &thread_attr, controller_thread, NULL); //Controller Thread
    pthread_create(&actuator_tid, &thread_attr, actuator_thread, NULL);  //Actuator Thread

    pthread_attr_destroy(&thread_attr);

    // 4. Create Display Thread (keep it Low-Priority Non-Real-Time) */
    pthread_create(&gui_thread, NULL, display_thread, NULL);

    /* Run simulation for 60 seconds */
    sleep(60);

    /* Cleanup */
    g_running = false;
    pthread_join(plant_thread, NULL);
    pthread_join(sensor_tid, NULL);
    pthread_join(controller_tid, NULL);
    pthread_join(actuator_tid, NULL);
    pthread_join(gui_thread, NULL);
    
    //Close the queues
    mq_close(sensor_queue);
    mq_close(force_queue);

    pthread_mutex_destroy(&g_state_mutex);

    printf("\033[H\033[JSimulation complete.\n");
    return EXIT_SUCCESS;
}
