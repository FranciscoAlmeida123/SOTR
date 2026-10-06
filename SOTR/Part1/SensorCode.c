#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <sys/mman.h>

#define SENSOR_PRIO  70
#define FILTER_PRIO  60
#define DISPLAY_PRIO 50

#define FIFO1_SIZE 2
#define FIFO2_SIZE 4

/* FIFO 1: sensor -> filter */
typedef struct {
    uint16_t data[FIFO1_SIZE];
    int head;
    int tail;
    int count;

    pthread_mutex_t mutex;
    pthread_cond_t not_empty;
    pthread_cond_t not_full;
} fifo1_t;

/* FIFO 2: filter -> display */
typedef struct {
    uint16_t data[FIFO2_SIZE];
    int head;
    int tail;
    int count;

    pthread_mutex_t mutex;
    pthread_cond_t not_empty;
    pthread_cond_t not_full;
} fifo2_t;

fifo1_t fifo1 = {
    .head = 0,
    .tail = 0,
    .count = 0,
    .mutex = PTHREAD_MUTEX_INITIALIZER,
    .not_empty = PTHREAD_COND_INITIALIZER,
    .not_full = PTHREAD_COND_INITIALIZER
};

fifo2_t fifo2 = {
    .head = 0,
    .tail = 0,
    .count = 0,
    .mutex = PTHREAD_MUTEX_INITIALIZER,
    .not_empty = PTHREAD_COND_INITIALIZER,
    .not_full = PTHREAD_COND_INITIALIZER
};

/* Add two timespec values */
struct timespec TsAdd(struct timespec t1, struct timespec t2)
{
    struct timespec result;

    result.tv_sec = t1.tv_sec + t2.tv_sec;
    result.tv_nsec = t1.tv_nsec + t2.tv_nsec;

    if (result.tv_nsec >= 1000000000L) {
        result.tv_sec++;
        result.tv_nsec -= 1000000000L;
    }

    return result;
}

/* Insert a sensor sample into FIFO 1 */
void fifo1_put(uint16_t value)
{
    pthread_mutex_lock(&fifo1.mutex);

    /* Wait while the FIFO is full */
    while (fifo1.count == FIFO1_SIZE) {
        pthread_cond_wait(&fifo1.not_full, &fifo1.mutex);
    }

    fifo1.data[fifo1.tail] = value;
    fifo1.tail = (fifo1.tail + 1) % FIFO1_SIZE;
    fifo1.count++;

    pthread_cond_signal(&fifo1.not_empty);
    pthread_mutex_unlock(&fifo1.mutex);
}

/* Remove a sensor sample from FIFO 1 */
uint16_t fifo1_get(void)
{
    uint16_t value;

    pthread_mutex_lock(&fifo1.mutex);

    /* Wait until a sample is available */
    while (fifo1.count == 0) {
        pthread_cond_wait(&fifo1.not_empty, &fifo1.mutex);
    }

    value = fifo1.data[fifo1.head];
    fifo1.head = (fifo1.head + 1) % FIFO1_SIZE;
    fifo1.count--;

    pthread_cond_signal(&fifo1.not_full);
    pthread_mutex_unlock(&fifo1.mutex);

    return value;
}

/* Insert a filtered value into FIFO 2 */
void fifo2_put(uint16_t value)
{
    pthread_mutex_lock(&fifo2.mutex);

    while (fifo2.count == FIFO2_SIZE) {
        pthread_cond_wait(&fifo2.not_full, &fifo2.mutex);
    }

    fifo2.data[fifo2.tail] = value;
    fifo2.tail = (fifo2.tail + 1) % FIFO2_SIZE;
    fifo2.count++;

    pthread_cond_signal(&fifo2.not_empty);
    pthread_mutex_unlock(&fifo2.mutex);
}

/* Remove a filtered value from FIFO 2 */
uint16_t fifo2_get(void)
{
    uint16_t value;

    pthread_mutex_lock(&fifo2.mutex);

    while (fifo2.count == 0) {
        pthread_cond_wait(&fifo2.not_empty, &fifo2.mutex);
    }

    value = fifo2.data[fifo2.head];
    fifo2.head = (fifo2.head + 1) % FIFO2_SIZE;
    fifo2.count--;

    pthread_cond_signal(&fifo2.not_full);
    pthread_mutex_unlock(&fifo2.mutex);

    return value;
}

/* Sensor: produces one sample every second */
void *SensorTask(void *arg)
{
    (void)arg;

    static uint16_t samples[] = {
        20, 21, 23, 24,
        26, 25, 27, 28,
        30, 29, 31, 32
    };

    const int number_samples =
        sizeof(samples) / sizeof(samples[0]);

    int index = 0;

    struct timespec next_activation;
    struct timespec period = {
        .tv_sec = 1,
        .tv_nsec = 0
    };

    clock_gettime(CLOCK_MONOTONIC, &next_activation);
    next_activation = TsAdd(next_activation, period);

    while (1) {
        clock_nanosleep(
            CLOCK_MONOTONIC,
            TIMER_ABSTIME,
            &next_activation,
            NULL
        );

        next_activation = TsAdd(next_activation, period);

        uint16_t sample = samples[index];

        printf("[SENSOR ] sample = %u\n", sample);

        fifo1_put(sample);

        index++;

        if (index == number_samples) {
            index = 0;
        }
    }

    return NULL;
}

/* Filter: calculates a moving average of four samples */
void *FilterTask(void *arg)
{
    (void)arg;

    uint16_t window[4];

    int position = 0;
    int number_values = 0;
    uint32_t sum = 0;

    while (1) {
        /* Block until a sensor sample arrives */
        uint16_t sample = fifo1_get();

        if (number_values < 4) {
            window[position] = sample;
            sum += sample;

            position = (position + 1) % 4;
            number_values++;
        } else {
            /* Replace the oldest sample */
            sum -= window[position];
            window[position] = sample;
            sum += sample;

            position = (position + 1) % 4;
        }

        /* Produce output once four samples are available */
        if (number_values == 4) {
            uint16_t filtered = (uint16_t)(sum / 4);

            printf("[FILTER ] input = %u -> average = %u\n",
                   sample, filtered);

            fifo2_put(filtered);
        }
    }

    return NULL;
}

/* Display: waits for and prints filtered values */
void *DisplayTask(void *arg)
{
    (void)arg;

    while (1) {
        uint16_t value = fifo2_get();

        printf("[DISPLAY] temperature = %u\n", value);
    }

    return NULL;
}

int main(void)
{
    pthread_t sensor_thread;
    pthread_t filter_thread;
    pthread_t display_thread;

    pthread_attr_t sensor_attr;
    pthread_attr_t filter_attr;
    pthread_attr_t display_attr;

    struct sched_param sensor_param;
    struct sched_param filter_param;
    struct sched_param display_param;

    /* Lock memory */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("mlockall");
    }

    /* Sensor scheduling attributes */
    pthread_attr_init(&sensor_attr);
    pthread_attr_setinheritsched(
        &sensor_attr, PTHREAD_EXPLICIT_SCHED
    );
    pthread_attr_setschedpolicy(&sensor_attr, SCHED_FIFO);

    sensor_param.sched_priority = SENSOR_PRIO;
    pthread_attr_setschedparam(&sensor_attr, &sensor_param);

    /* Filter scheduling attributes */
    pthread_attr_init(&filter_attr);
    pthread_attr_setinheritsched(
        &filter_attr, PTHREAD_EXPLICIT_SCHED
    );
    pthread_attr_setschedpolicy(&filter_attr, SCHED_FIFO);

    filter_param.sched_priority = FILTER_PRIO;
    pthread_attr_setschedparam(&filter_attr, &filter_param);

    /* Display scheduling attributes */
    pthread_attr_init(&display_attr);
    pthread_attr_setinheritsched(
        &display_attr, PTHREAD_EXPLICIT_SCHED
    );
    pthread_attr_setschedpolicy(&display_attr, SCHED_FIFO);

    display_param.sched_priority = DISPLAY_PRIO;
    pthread_attr_setschedparam(&display_attr, &display_param);

    /* Create threads */
    int err;

    err = pthread_create(
        &display_thread, &display_attr, DisplayTask, NULL
    );

    if (err != 0) {
        printf("Error creating display thread: %s\n",
               strerror(err));
        return -1;
    }

    err = pthread_create(
        &filter_thread, &filter_attr, FilterTask, NULL
    );

    if (err != 0) {
        printf("Error creating filter thread: %s\n",
               strerror(err));
        return -1;
    }

    err = pthread_create(
        &sensor_thread, &sensor_attr, SensorTask, NULL
    );

    if (err != 0) {
        printf("Error creating sensor thread: %s\n",
               strerror(err));
        return -1;
    }

    /* Wait forever */
    pthread_join(sensor_thread, NULL);
    pthread_join(filter_thread, NULL);
    pthread_join(display_thread, NULL);

    return 0;
}

/*- Why fixed samples? They make results reproducible, so you can check the filter against known expected averages.
- Are FIFO sizes 2 and 4 suitable? They can work if consumers keep up. Their suitability depends on processing delays and scheduling; when full, this code blocks the producer, which can delay sensor acquisition.
- Is the priority order reasonable? It prioritizes timely acquisition, then processing, then display. That follows the requested data flow, although suitability ultimately depends on each task’s timing requirements.*/












