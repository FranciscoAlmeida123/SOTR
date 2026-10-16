# Sample midterm: Q1 and Q2 A–C

## Q1.A — Periodic activation

exercise1.c creates a worker that increments and prints a uint32_t counter every 100 ms. It runs for 12 activations, then main joins it. A finite run makes the demonstration easy to inspect.

The worker advances an absolute release time by 100,000,000 ns each cycle and waits using clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...). Work time is therefore not added to the next period. The first release is epoch + 100 ms. Epoch includes a 200 ms setup margin.

## Q1.B — Real-time policy

The default policy is SCHED_FIFO with priority 50. PTHREAD_EXPLICIT_SCHED ensures the requested attributes are used rather than inherited. All pthread return codes are checked; these APIs return error numbers directly.

Run with sufficient real-time scheduling permissions (for example sudo on a configured lab machine). --normal explicitly switches to SCHED_OTHER for functional testing and does not satisfy the real-time requirement.

## Q1.C — Improve determinism

The solution uses absolute monotonic releases, retries interrupted sleeps without changing the release, selects one allowed CPU to prevent migrations, and avoids application-level dynamic allocation in the periodic loop. It configures output buffering before worker execution.

These choices improve repeatability but do not guarantee a hard deadline. Terminal printing is required by the question and can block; line buffering does not make it deterministic. Optional further measures include successfully locking memory and prefaulting worker stacks, controlling competing workload, and using a suitable RT kernel. Those optional memory measures are not implemented in these files. A missed release currently causes immediate catch-up, rather than dropping jobs.

## Q2.A — Queue pipeline and moving average

exercise2_ABC.c creates three SCHED_FIFO workers:

- Source: priority 70, period 100 ms, sends a uint32_t counter.
- Processor: priority 60, blocks on the first queue, computes the moving average, sends a float.
- Output: priority 50, blocks on the second queue and prints the received float.

Only Source sleeps periodically. Processor and Output are event-triggered by data availability. The queue messages establish the dependency; priorities affect the scheduling of ready workers. Message priority is 0 for every message and is independent of thread priority.

A three-element ring buffer and a running sum implement the moving average. For the first two samples, the average uses the samples available so far (explicit startup convention). Values 1, 2, 3, 4, ... produce 1.0, 1.5, 2.0, 3.0, ... . The uint64_t sum prevents overflow when adding three uint32_t values.

Both queues are created before workers start, using unique names. Their names are unlinked immediately; open descriptors remain valid and are closed at normal exit. All workers process exactly 12 messages, so no shutdown sentinel is needed.

## Q2.B — One CPU

The program obtains its allowed CPU mask, selects the first allowed CPU and applies that CPU to each worker's creation attributes. All three workers therefore start on the same CPU. This avoids assuming CPU 0 is available in every environment.

## Q2.C — Activation timestamps

Every worker samples CLOCK_MONOTONIC at the beginning of its processing: after the periodic wait for Source, and after receiving a message for Processor and Output. Timestamps are elapsed time relative to epoch and are formatted as ss.mmm (milliseconds truncated). At this resolution, all three tasks can show the same timestamp; that does not mean they ran simultaneously.

The program produces 12 activations per worker, covering 1.2 seconds after epoch. It records actual observed starts, not merely scheduled release times. It does not record finish times.

Build and capture an RT run on your lab machine:

```bash
make
sudo ./exercise1
sudo ./exercise2_ABC | tee exercise2_RT_output.txt
```

Copy the captured RT output into the exam answers document for Q2.C. The saved exercise2_functional_output.txt is an actual run from this environment in --normal mode, provided only to demonstrate formatting and calculations. It is not an RT measurement.

## Validation

Both programs compiled with -std=c11 -Wall -Wextra -Wpedantic without diagnostics. Functional runs produced 12 sequential counters and the expected 12 moving averages. The default RT run was attempted but denied at pthread_create with EPERM because this environment lacks real-time scheduling permission. RT timing and worst-case behavior are consequently unverified.

Q2.D–G are outside the requested scope.
