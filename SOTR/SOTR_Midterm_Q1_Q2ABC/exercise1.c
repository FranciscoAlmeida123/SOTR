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

static void *periodic(void *unused)
{
    (void)unused;
    struct timespec release = epoch;
    for (uint32_t counter = 1; counter <= ACTIVATIONS; counter++) {
        add_ns(&release, PERIOD_NS);
        sleep_until(&release);
        printf("%" PRIu32 "\n", counter);
    }
    return NULL;
}

int main(int argc, char **argv)
{
    parse_mode(argc, argv);
    /* Configure output before starting the measured periodic work. */
    setvbuf(stdout, NULL, _IOLBF, 0);
    /* Q1.C: pin the application to one allowed CPU to avoid migrations. */
    cpu_set_t allowed, selected;
    if (sched_getaffinity(0, sizeof(allowed), &allowed)) fail("sched_getaffinity");
    int cpu;
    for (cpu = 0; cpu < CPU_SETSIZE; cpu++)
        if (CPU_ISSET(cpu, &allowed)) break;
    if (cpu == CPU_SETSIZE) return EXIT_FAILURE;
    CPU_ZERO(&selected);
    CPU_SET(cpu, &selected);
    if (sched_setaffinity(0, sizeof(selected), &selected)) fail("sched_setaffinity");
    pthread_attr_t attr;
    init_attr(&attr, 50); /* Q1.B: SCHED_FIFO priority 50. */
    if (clock_gettime(CLOCK_MONOTONIC, &epoch)) fail("clock_gettime");
    add_ns(&epoch, 200000000L); /* Setup margin before the periodic schedule. */
    pthread_t thread;
    check_thread(pthread_create(&thread, &attr, periodic, NULL), "pthread_create");
    check_thread(pthread_attr_destroy(&attr), "pthread_attr_destroy");
    check_thread(pthread_join(thread, NULL), "pthread_join");
    return EXIT_SUCCESS;
}
