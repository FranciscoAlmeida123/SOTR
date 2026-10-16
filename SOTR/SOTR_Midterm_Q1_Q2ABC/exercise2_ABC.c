#define _GNU_SOURCE
#include <errno.h>
#include <inttypes.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define PERIOD_NS 100000000L
#define ACTIVATIONS 12

static struct timespec epoch;
static bool normal_mode;

static void fail(const char *what)
{
    perror(what);
    exit(EXIT_FAILURE);
}

static void check_thread(int error, const char *what)
{
    if (error) {
        errno = error;
        if (error == EPERM)
            fprintf(stderr, "Real-time scheduling permission required; use sudo, "
                    "or --normal for a functional test only.\n");
        fail(what);
    }
}

static void add_ns(struct timespec *t, long ns)
{
    t->tv_nsec += ns;
    while (t->tv_nsec >= 1000000000L) {
        t->tv_nsec -= 1000000000L;
        t->tv_sec++;
    }
}

static void sleep_until(const struct timespec *release)
{
    int error;
    do {
        error = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, release, NULL);
    } while (error == EINTR);
    check_thread(error, "clock_nanosleep");
}

static void parse_mode(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "--normal") == 0)
        normal_mode = true;
    else if (argc != 1) {
        fprintf(stderr, "Usage: %s [--normal]\n", argv[0]);
        exit(EXIT_FAILURE);
    }
}

static void init_attr(pthread_attr_t *attr, int priority)
{
    check_thread(pthread_attr_init(attr), "pthread_attr_init");
    check_thread(pthread_attr_setinheritsched(attr, PTHREAD_EXPLICIT_SCHED),
                 "pthread_attr_setinheritsched");
    check_thread(pthread_attr_setschedpolicy(attr,
                 normal_mode ? SCHED_OTHER : SCHED_FIFO),
                 "pthread_attr_setschedpolicy");
    struct sched_param param = { .sched_priority = normal_mode ? 0 : priority };
    check_thread(pthread_attr_setschedparam(attr, &param),
                 "pthread_attr_setschedparam");
}

#include <fcntl.h>
#include <mqueue.h>

static mqd_t state_queue = (mqd_t)-1;
static mqd_t average_queue = (mqd_t)-1;

/* Remove queue names immediately after opening; close descriptors at exit. */
static void close_queues(void)
{
    if (state_queue != (mqd_t)-1) mq_close(state_queue);
    if (average_queue != (mqd_t)-1) mq_close(average_queue);
}

static mqd_t make_queue(const char *name, long size)
{
    struct mq_attr attr = { .mq_maxmsg = 10, .mq_msgsize = size };
    mqd_t q = mq_open(name, O_CREAT | O_EXCL | O_RDWR, 0600, &attr);
    if (q == (mqd_t)-1) fail("mq_open");
    if (mq_unlink(name)) {
        mq_close(q);
        fail("mq_unlink");
    }
    return q;
}

static void send_value(mqd_t q, const void *value, size_t size)
{
    int result;
    do {
        result = mq_send(q, (const char *)value, size, 0);
    } while (result == -1 && errno == EINTR);
    if (result == -1) fail("mq_send");
}

static void receive_value(mqd_t q, void *value, size_t size)
{
    ssize_t result;
    do {
        result = mq_receive(q, (char *)value, size, NULL);
    } while (result == -1 && errno == EINTR);
    if (result == -1) fail("mq_receive");
    if (result != (ssize_t)size) {
        fprintf(stderr, "Unexpected message size\n");
        exit(EXIT_FAILURE);
    }
}

/* Elapsed activation time relative to epoch, printed as ss.mmm. */
static long long activation_ms(void)
{
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now)) fail("clock_gettime");
    long long ns = (long long)(now.tv_sec - epoch.tv_sec) * 1000000000LL
                 + now.tv_nsec - epoch.tv_nsec;
    return ns / 1000000LL;
}

static void *source(void *unused)
{
    (void)unused;
    struct timespec release = epoch;
    for (uint32_t counter = 1; counter <= ACTIVATIONS; counter++) {
        add_ns(&release, PERIOD_NS);
        sleep_until(&release);
        long long ms = activation_ms();
        printf("%02lld.%03lld Source    counter=%" PRIu32 "\n",
               ms / 1000, ms % 1000, counter);
        send_value(state_queue, &counter, sizeof(counter));
    }
    return NULL;
}

static void *processor(void *unused)
{
    (void)unused;
    uint32_t window[3] = {0};
    unsigned int position = 0, count = 0;
    uint64_t sum = 0;
    for (unsigned int job = 0; job < ACTIVATIONS; job++) {
        uint32_t value;
        /* Blocking receive: this task activates when data becomes available. */
        receive_value(state_queue, &value, sizeof(value));
        long long ms = activation_ms();
        if (count == 3) sum -= window[position];
        else count++;
        window[position] = value;
        sum += value;
        position = (position + 1) % 3;
        /* Startup convention: average the samples received so far. */
        float average = (float)sum / count;
        printf("%02lld.%03lld Processor counter=%" PRIu32 " average=%.3f\n",
               ms / 1000, ms % 1000, value, average);
        send_value(average_queue, &average, sizeof(average));
    }
    return NULL;
}

static void *output(void *unused)
{
    (void)unused;
    for (unsigned int job = 0; job < ACTIVATIONS; job++) {
        float average;
        receive_value(average_queue, &average, sizeof(average));
        long long ms = activation_ms();
        printf("%02lld.%03lld Output    average=%.3f\n",
               ms / 1000, ms % 1000, average);
    }
    return NULL;
}

static void start_worker(pthread_t *thread, void *(*entry)(void *),
                         int priority, int cpu)
{
    pthread_attr_t attr;
    init_attr(&attr, priority);
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(cpu, &mask);
    /* Q2.B: affinity is set before the worker can execute. */
    check_thread(pthread_attr_setaffinity_np(&attr, sizeof(mask), &mask),
                 "pthread_attr_setaffinity_np");
    check_thread(pthread_create(thread, &attr, entry, NULL), "pthread_create");
    check_thread(pthread_attr_destroy(&attr), "pthread_attr_destroy");
}

int main(int argc, char **argv)
{
    parse_mode(argc, argv);
    setvbuf(stdout, NULL, _IOLBF, 0);
    if (atexit(close_queues)) {
        fprintf(stderr, "Cannot register queue cleanup\n");
        return EXIT_FAILURE;
    }
    char name[64];
    snprintf(name, sizeof(name), "/exam_counter_%ld", (long)getpid());
    state_queue = make_queue(name, sizeof(uint32_t));
    snprintf(name, sizeof(name), "/exam_average_%ld", (long)getpid());
    average_queue = make_queue(name, sizeof(float));

    cpu_set_t allowed;
    if (sched_getaffinity(0, sizeof(allowed), &allowed)) fail("sched_getaffinity");
    int cpu;
    for (cpu = 0; cpu < CPU_SETSIZE; cpu++)
        if (CPU_ISSET(cpu, &allowed)) break;
    if (cpu == CPU_SETSIZE) {
        fprintf(stderr, "No allowed CPU\n");
        return EXIT_FAILURE;
    }
    fprintf(stderr, "Mode: %s; all workers on CPU %d\n",
            normal_mode ? "SCHED_OTHER (functional test)" : "SCHED_FIFO", cpu);
    if (clock_gettime(CLOCK_MONOTONIC, &epoch)) fail("clock_gettime");
    add_ns(&epoch, 200000000L);
    pthread_t source_tid, processor_tid, output_tid;
    /* Consumers start first and immediately wait for messages. */
    start_worker(&output_tid, output, 50, cpu);
    start_worker(&processor_tid, processor, 60, cpu);
    start_worker(&source_tid, source, 70, cpu);
    check_thread(pthread_join(source_tid, NULL), "join source");
    check_thread(pthread_join(processor_tid, NULL), "join processor");
    check_thread(pthread_join(output_tid, NULL), "join output");
    return EXIT_SUCCESS;
}
