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
static volatile bool g_running = true;

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
void* control_loop_thread(void *arg) {
    struct timespec next_period;
    const long period_ns = 5000000L; /* 5 ms */
    
    /* Full-State Feedback Gains (LQR / Pole Placement Derived) */
    const float K_theta     =  45.0f;  /* Angle Error (Stabilizes Pole) */
    const float K_theta_dot =  10.0f;  /* Angular Velocity (Damps Pole) */
    const float K_x         =  2.0f;  /* Cart Position (Drives x -> 0) */
    const float K_x_dot     =  4.0f;  /* Cart Velocity (Damps Cart) */

    clock_gettime(CLOCK_MONOTONIC, &next_period);

	// Control loop, triggered every 5 ms
	// Reads sensors, applies the control law and actuates - monolithic
    while (g_running) {
        // Wait for next activation
        timespec_add_ns(&next_period, period_ns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);

		// Add noise to the sample - real sensors suffer from noise and interference 
		// Generate a random integer in a specific range [min, max]
		int min = -50;
		int max = +50;
		int random_num = min + rand() % (max - min + 1);
		float noise = (float)random_num/1500;

		// Execute the controller 
        pthread_mutex_lock(&g_state_mutex);

        // State Feedback Law: u = K_theta*theta + K_theta_dot*theta_dot + K_x*x + K_x_dot*x_dot
        float control_force = (K_theta * (g_state.theta + noise))     + 
                              (K_theta_dot * g_state.theta_dot) + 
                              (K_x         * g_state.x)         + 
                              (K_x_dot     * g_state.x_dot);

        /* Update control force buffer */
        g_state.force = control_force;

        pthread_mutex_unlock(&g_state_mutex);
    }
    return NULL;
}

/* -------------------------------------------------------------------------- */
/* THREAD 3: ASCII Graphics Display (Low-Priority, ~30 Hz / 33 ms period)      */
/* -------------------------------------------------------------------------- */
void* display_thread(void *arg) {
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
int main(int argc, char *argv[]) {
    pthread_t plant_thread, controller_thread, gui_thread;    
    pthread_attr_t thread_attr;
        
    // 1. Seed the random number generator using current time
	srand((unsigned int)time(NULL));


    // 2. Initialize Mutex
    pthread_mutex_init(&g_state_mutex, NULL);
    

    // 3. Create Control and Simulation Threads
    pthread_attr_init(&thread_attr);

    // Physics Thread
    pthread_create(&plant_thread, &thread_attr, physics_plant_thread, NULL);

    // Controller Thread
    pthread_create(&controller_thread, &thread_attr, control_loop_thread, NULL);

    pthread_attr_destroy(&thread_attr);

    // 4. Create Display Thread (keep it Low-Priority Non-Real-Time) */
    pthread_create(&gui_thread, NULL, display_thread, NULL);

    /* Run simulation for 60 seconds */
    sleep(60);

    /* Cleanup */
    g_running = false;
    pthread_join(plant_thread, NULL);
    pthread_join(controller_thread, NULL);
    pthread_join(gui_thread, NULL);
    pthread_mutex_destroy(&g_state_mutex);

    printf("\033[H\033[JSimulation complete.\n");
    return EXIT_SUCCESS;
}
