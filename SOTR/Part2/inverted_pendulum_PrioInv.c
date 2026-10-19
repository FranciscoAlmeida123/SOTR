/* Self-contained study example: no CSV logging or external C modules. */
#define _GNU_SOURCE
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define MS 1000000LL
typedef struct {
  float theta, theta_dot, x, x_dot, force;
} PlantState;
static PlantState state = {.theta = 0.15f};
static pthread_mutex_t state_lock;
static atomic_bool running = true;
static bool normal_mode, quiet_mode;
static int duration = 60;
static long long epoch, end_time;

static void die(const char *what) {
  perror(what);
  exit(EXIT_FAILURE);
}
static void check(int error, const char *what) {
  if (error) {
    errno = error;
    die(what);
  }
}
static long long now_ns(void) {
  struct timespec t;
  if (clock_gettime(CLOCK_MONOTONIC, &t))
    die("clock_gettime");
  return (long long)t.tv_sec * 1000000000LL + t.tv_nsec;
}
static void sleep_until(long long release) {
  struct timespec t = {release / 1000000000LL, release % 1000000000LL};
  int error;
  do {
    error = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
  } while (error == EINTR);
  check(error, "clock_nanosleep");
}
static bool alive(void) {
  if (now_ns() >= end_time)
    running = false;
  return running;
}
static void setup(int argc, char **argv) {
  /* Default is real-time execution; --normal is only for functional tests. */
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--normal"))
      normal_mode = true;
    else if (!strcmp(argv[i], "--seconds") && i + 1 < argc) {
      char *end;
      long value = strtol(argv[++i], &end, 10);
      if (*end || value < 1 || value > 60) {
        fprintf(stderr, "Duration must be 1..60 seconds\n");
        exit(EXIT_FAILURE);
      }
      duration = (int)value;
    } else {
      fprintf(stderr, "Usage: %s [--normal] [--seconds 1..60]\n", argv[0]);
      exit(EXIT_FAILURE);
    }
  }
  /* Pin main before creating workers: they inherit this single-CPU mask. */
  cpu_set_t allowed, one;
  if (sched_getaffinity(0, sizeof(allowed), &allowed))
    die("get affinity");
  int cpu;
  for (cpu = 0; cpu < CPU_SETSIZE; cpu++)
    if (CPU_ISSET(cpu, &allowed))
      break;
  if (cpu == CPU_SETSIZE) {
    fprintf(stderr, "No allowed CPU\n");
    exit(EXIT_FAILURE);
  }
  CPU_ZERO(&one);
  CPU_SET(cpu, &one);
  if (sched_setaffinity(0, sizeof(one), &one))
    die("set affinity");
  /* Ordinary mutex: PI is intentionally outside the first two bullets. */
  pthread_mutexattr_t attr;
  check(pthread_mutexattr_init(&attr), "mutex attributes");
  check(pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_NONE),
        "ordinary mutex");
  check(pthread_mutex_init(&state_lock, &attr), "state mutex");
  check(pthread_mutexattr_destroy(&attr), "destroy mutex attributes");
  srand(1);
}
static void create_thread(pthread_t *tid, void *(*entry)(void *),
                          int priority) {
  pthread_attr_t attr;
  check(pthread_attr_init(&attr), "thread attributes");
  check(pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED),
        "explicit scheduling");
  int policy = (!normal_mode && priority) ? SCHED_FIFO : SCHED_OTHER;
  check(pthread_attr_setschedpolicy(&attr, policy), "policy");
  struct sched_param p = {.sched_priority =
                              policy == SCHED_FIFO ? priority : 0};
  check(pthread_attr_setschedparam(&attr, &p), "priority");
  int error = pthread_create(tid, &attr, entry, NULL);
  if (error == EPERM)
    fprintf(stderr,
            "RT permission required; use sudo or --normal for testing.\n");
  check(error, "pthread_create");
  check(pthread_attr_destroy(&attr), "destroy thread attributes");
}
void physics_step(void) {
  check(pthread_mutex_lock(&state_lock), "plant lock");
  float alpha =
      ((1.0f + 0.1f) * 9.81f * state.theta - state.force) / (1.0f * 0.5f);
  float acceleration = (state.force - 0.1f * 0.5f * alpha) / (1.0f + 0.1f);
  state.theta_dot += alpha * 0.001f;
  state.theta += state.theta_dot * 0.001f;
  state.x_dot += acceleration * 0.001f;
  state.x += state.x_dot * 0.001f;
  bool finite = isfinite(state.theta) && isfinite(state.x) &&
                isfinite(state.theta_dot) && isfinite(state.x_dot);
  check(pthread_mutex_unlock(&state_lock), "plant unlock");
  if (!finite) {
    fprintf(stderr, "Plant diverged; stopping simulation.\n");
    running = false;
  }
}
PlantState sample_state(void) {
  check(pthread_mutex_lock(&state_lock), "sample lock");
  PlantState s = state;
  check(pthread_mutex_unlock(&state_lock), "sample unlock");
  s.theta += (float)(-50 + rand() % 101) / 1500.0f;
  return s;
}
float control_force(PlantState s) {
  return 45 * s.theta + 10 * s.theta_dot + 2 * s.x + 4 * s.x_dot;
}
void apply_force(float force) {
  check(pthread_mutex_lock(&state_lock), "apply lock");
  state.force = force;
  check(pthread_mutex_unlock(&state_lock), "apply unlock");
}
void display_frame(void) {
  if (quiet_mode)
    return;
  check(pthread_mutex_lock(&state_lock), "display lock");
  PlantState s = state;
  check(pthread_mutex_unlock(&state_lock), "display unlock");
  if (!isfinite(s.x) || !isfinite(s.theta))
    return;
  float position = fmaxf(-2, fminf(2, s.x));
  int cart = (int)((position + 2) / 4 * 60);
  if (cart < 2)
    cart = 2;
  if (cart > 57)
    cart = 57;
  int offset = (int)(sinf(s.theta) * 15), tip = cart + offset;
  if (tip < 0)
    tip = 0;
  if (tip > 59)
    tip = 59;
  printf("\033[H\033[J==== INVERTED PENDULUM ====\nTheta: %+.4f | x: %+.3f | "
         "Force: %+.2f\n",
         s.theta, s.x, s.force);
  for (int i = 0; i < 60; i++)
    putchar(i == tip ? 'O' : ' ');
  putchar('\n');
  for (int row = 0; row < 3; row++) {
    int pole = cart + offset * (3 - row) / 4;
    for (int i = 0; i < 60; i++)
      putchar(i == pole ? '|' : ' ');
    putchar('\n');
  }
  for (int i = 0; i < 60; i++)
    putchar(i == cart - 2       ? '['
            : i == cart + 2     ? ']'
            : abs(i - cart) < 2 ? '='
                                : ' ');
  printf("\n------------------------------------------------------------\nCtrl+"
         "C stops and exports timings.\n");
  fflush(stdout);
}

/* 4.4, bullet 1: shared sample and force each have an ORDINARY mutex.
 * Empty/ready semaphores provide bounded one-slot handoffs; mutexes protect
 * copies.
 */
static PlantState shared_sample;
static float shared_force;
static pthread_mutex_t sample_lock, force_lock;
static sem_t sample_empty, sample_ready, force_empty, force_ready;
static pthread_barrier_t startup;
static void enter(void) {
  int e = pthread_barrier_wait(&startup);
  if (e != PTHREAD_BARRIER_SERIAL_THREAD)
    check(e, "startup barrier");
}
static bool wait_token(sem_t *s) {
  while (alive()) {
    /* Timed waits make normal shutdown possible even if a buffer is empty. */
    struct timespec t;
    if (clock_gettime(CLOCK_REALTIME, &t))
      die("clock_gettime");
    t.tv_nsec += 100000000L;
    if (t.tv_nsec >= 1000000000L) {
      t.tv_nsec -= 1000000000L;
      t.tv_sec++;
    }
    if (!sem_timedwait(s, &t))
      return alive();
    if (errno != EINTR && errno != ETIMEDOUT)
      die("sem_timedwait");
  }
  return false;
}
static void post_token(sem_t *s) {
  if (sem_post(s))
    die("sem_post");
}
static void *plant_thread(void *arg) {
  (void)arg;
  enter();
  for (long long t = epoch + MS; t < end_time && running; t += MS) {
    sleep_until(t);
    if (!alive())
      break;
    physics_step();
  }
  return NULL;
}
static void *sensor_thread(void *arg) {
  (void)arg;
  enter();
  for (long long t = epoch + 5 * MS; t < end_time && running; t += 5 * MS) {
    sleep_until(t);
    if (!wait_token(&sample_empty))
      break;
    PlantState sample = sample_state();
    check(pthread_mutex_lock(&sample_lock), "sensor sample lock");
    shared_sample = sample;
    check(pthread_mutex_unlock(&sample_lock), "sensor sample unlock");
    post_token(&sample_ready);
  }
  return NULL;
}
static void *controller_thread(void *arg) {
  (void)arg;
  enter();
  while (wait_token(&sample_ready)) {
    check(pthread_mutex_lock(&sample_lock), "controller sample lock");
    PlantState sample = shared_sample;
    check(pthread_mutex_unlock(&sample_lock), "controller sample unlock");
    post_token(&sample_empty);
    float force = control_force(sample);
    if (!wait_token(&force_empty))
      break;
    check(pthread_mutex_lock(&force_lock), "controller force lock");
    shared_force = force;
    check(pthread_mutex_unlock(&force_lock), "controller force unlock");
    post_token(&force_ready);
  }
  return NULL;
}
static void *actuator_thread(void *arg) {
  (void)arg;
  enter();
  while (wait_token(&force_ready)) {
    check(pthread_mutex_lock(&force_lock), "actuator force lock");
    float force = shared_force;
    check(pthread_mutex_unlock(&force_lock), "actuator force unlock");
    post_token(&force_empty);
    apply_force(force);
  }
  return NULL;
}
static void *display_thread(void *arg) {
  (void)arg;
  enter();
  for (long long t = epoch + 33 * MS; t < end_time && running; t += 33 * MS) {
    sleep_until(t);
    if (!alive())
      break;
    display_frame();
  }
  return NULL;
}
int main(int argc, char **argv) {
  setup(argc, argv);
  check(pthread_mutex_init(&sample_lock, NULL), "sample mutex");
  check(pthread_mutex_init(&force_lock, NULL), "force mutex");
  if (sem_init(&sample_empty, 0, 1) || sem_init(&sample_ready, 0, 0) ||
      sem_init(&force_empty, 0, 1) || sem_init(&force_ready, 0, 0))
    die("sem_init");
  check(pthread_barrier_init(&startup, NULL, 6), "barrier init");
  pthread_t plant, sensor, controller, actuator, display;
  /* 4.4, bullet 2: spaced RT priorities; simulator higher than control tasks.
   */
  create_thread(&plant, plant_thread, 60);
  create_thread(&sensor, sensor_thread, 50);
  create_thread(&controller, controller_thread, 40);
  create_thread(&actuator, actuator_thread, 30);
  create_thread(&display, display_thread, 0);
  /* Release all workers together, including physics, with one common epoch. */
  epoch = now_ns() + 20 * MS;
  end_time = epoch + (long long)duration * 1000 * MS;
  enter();
  check(pthread_join(plant, NULL), "join plant");
  check(pthread_join(sensor, NULL), "join sensor");
  check(pthread_join(controller, NULL), "join controller");
  check(pthread_join(actuator, NULL), "join actuator");
  check(pthread_join(display, NULL), "join display");
  sem_destroy(&sample_empty);
  sem_destroy(&sample_ready);
  sem_destroy(&force_empty);
  sem_destroy(&force_ready);
  pthread_mutex_destroy(&sample_lock);
  pthread_mutex_destroy(&force_lock);
  pthread_mutex_destroy(&state_lock);
  pthread_barrier_destroy(&startup);
  puts("\nSimulation complete.");
  return EXIT_SUCCESS;
}
