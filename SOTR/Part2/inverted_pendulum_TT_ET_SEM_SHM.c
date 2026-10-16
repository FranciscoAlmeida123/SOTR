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
#include <semaphore.h>
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

/* Shared buffer: Sensor writes a sample and Controller reads it. */
static PlantState shared_sample;

/* Shared buffer: Controller writes a force and Actuator reads it. */
static float shared_force;

/* Count available sample-buffer slots: initially one empty slot. */
static sem_t sample_empty;

/* Count samples available for the Controller: initially none. */
static sem_t sample_ready;

/* Count available force-buffer slots: initially one empty slot. */
static sem_t force_empty;

/* Count forces available for the Actuator: initially none. */
static sem_t force_ready;


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

/* Wait for a semaphore token, checking periodically for shutdown. */
static bool wait_semaphore(sem_t *semaphore)
{
    while (g_running) {
        struct timespec timeout;
        clock_gettime(CLOCK_REALTIME, &timeout);

        /* sem_timedwait requires an absolute timeout. */
        timespec_add_ns(&timeout, 100000000L);

        /* A successful wait consumes one token. */
        if (sem_timedwait(semaphore, &timeout) == 0)
            return true;

        /* Retry if interrupted or if no token arrived within 100 ms. */
        if (errno == EINTR || errno == ETIMEDOUT)
            continue;

        perror("sem_timedwait");
        g_running = false;
        return false;
    }

    return false;
}

/* Publish a token, allowing a waiting thread to become ready. */
static bool post_semaphore(sem_t *semaphore)
{
    if (sem_post(semaphore) == 0)
        return true;

    perror("sem_post");
    g_running = false;
    return false;
}
//* -------------------------------------------------------------------------- */
/* THREAD 2: Feedback Controller (Runs at 200 Hz / 5 ms period)     */
/* -------------------------------------------------------------------------- */

//Sensor thread
void *sensor_thread(void *arg)
{
    (void)arg;

    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);

    while (g_running) {
        timespec_add_ns(&next_period, 5000000L);

        int error;
        do {
            error = clock_nanosleep(
                CLOCK_MONOTONIC, TIMER_ABSTIME,
                &next_period, NULL);
        } while (error == EINTR && g_running);

        if (!g_running)
            break;

        if (error != 0) {
            errno = error;
            perror("Sensor sleep");
            g_running = false;
            break;
        }

        /* Reserve the sample buffer so unread data cannot be overwritten. */
	if (!wait_semaphore(&sample_empty))
   		break;

	/* Preserve the sensor noise from the original simulation. */
	int random_num = -50 + rand() % 101;
	float noise = (float)random_num / 1500;

	PlantState sample;

	/* Protect the plant snapshot from concurrent physics updates. */
	pthread_mutex_lock(&g_state_mutex);
	sample = g_state;
	pthread_mutex_unlock(&g_state_mutex);

	/* Add noise only to the sampled angle, not to the plant itself. */
	sample.theta += noise;

	/* Write the sample into the shared communication buffer. */
	shared_sample = sample;

	/* Notify the Controller that the complete sample is available. */
	if (!post_semaphore(&sample_ready))
    	break;
    	}

    return NULL;
}
//Controller
void *controller_thread(void *arg)
{
    (void)arg;

    /* Keep the original feedback gains and sign convention. */
    const float K_theta     = 45.0f;
    const float K_theta_dot = 10.0f;
    const float K_x         = 2.0f;
    const float K_x_dot     = 4.0f;

    while (g_running) {
        /* Wait until the Sensor publishes a complete sample. */
        if (!wait_semaphore(&sample_ready))
            break;

        /* Copy locally before allowing the Sensor to reuse the buffer. */
        PlantState sample = shared_sample;

        /* Mark the sample buffer as available for the next sample. */
        if (!post_semaphore(&sample_empty))
            break;

        /* Calculate using the local sample, preserving the original law. */
        float force =
            K_theta     * sample.theta
          + K_theta_dot * sample.theta_dot
          + K_x         * sample.x
          + K_x_dot     * sample.x_dot;

        /* Wait until the Actuator has copied the previous force. */
        if (!wait_semaphore(&force_empty))
            break;

        /* Publish the newly calculated force in shared memory. */
        shared_force = force;

        /* Notify the Actuator that a new force is available. */
        if (!post_semaphore(&force_ready))
            break;
    }

    return NULL;
}
//Actuator
void *actuator_thread(void *arg)
{
    (void)arg;

    while (g_running) {
        /* Wait until the Controller publishes a force. */
        if (!wait_semaphore(&force_ready))
            break;

        /* Copy locally before allowing the Controller to reuse the buffer. */
        float force = shared_force;

        /* Mark the force buffer as available for another value. */
        if (!post_semaphore(&force_empty))
            break;

        /* Protect the plant state while applying the received force. */
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
    
	/* List the semaphores so initialization and error cleanup are consistent. */
	sem_t *semaphores[] = {
    	&sample_empty, &sample_ready,
    	&force_empty, &force_ready
	};

	/* Each buffer starts empty, with no data ready for consumption. */
	unsigned int initial_values[] = { 1, 0, 1, 0 };

	for (int i = 0; i < 4; i++) {
    		/* pshared = 0: semaphore is shared by threads in this process. */
    		if (sem_init(semaphores[i], 0, initial_values[i]) == -1) {
        	perror("sem_init");

        		/* Clean up only the semaphores initialized successfully. */
        		for (int j = 0; j < i; j++)
            		sem_destroy(semaphores[j]);

        		pthread_mutex_destroy(&g_state_mutex);
        		return EXIT_FAILURE;
    		}
    	}
    // 3. Create Control and Simulation Threads
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
    
    /* Destroy semaphores only after all worker threads have stopped using them. */
	sem_destroy(&sample_empty);
	sem_destroy(&sample_ready);
	sem_destroy(&force_empty);
	sem_destroy(&force_ready);

    pthread_mutex_destroy(&g_state_mutex);

    printf("\033[H\033[JSimulation complete.\n");
    return EXIT_SUCCESS;
}
