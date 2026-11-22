# Bounded SPURS policy scheduling

A policy module can wait for work produced by another workload in the same
SPURS instance. Running every policy synchronously on one host poll thread
prevents the producer from being admitted while the consumer waits.

Each initialized instance owns one coordinator and exactly `nSpus` persistent
policy workers (one through six). A worker has a stable physical SPU number.
The coordinator rotates its workload cursor, admits work under the instance
lock, and respects the guest-visible maximum contention. It consumes ready and
signal state only on admission. Guest policy execution runs outside admission
locks. Genuine kernel exits, stops, and error returns release the worker's
admission; no guest completion predicate is manufactured.

With `PS3_SPURS_SELF_ADMIT` (default 1; `0` restores coordinator-only
admission), a worker whose policy returns makes the coordinator's admission
decision for its own lane, under the same locks and rules, and starts the next
run itself. Admission wakes only the lane it fills. The coordinator is woken
only while a lane is free: a returning worker that leaves one free hands it the
rest of that pass (same pass number), and a request-line commit wakes it only
then. With every lane admitted it sleeps without a timeout, since nothing can
be admitted until a lane returns; otherwise it keeps the 1 ms timed pass, which
finds requests raised without a notification and drives the idle back-off
probes. `PS3_SPURS_SCHED_STATS=1` prints each instance's worker and
coordinator admissions and coordinator passes every two seconds.

Header consumption and contention updates serialize with SPU lock-line commits
and notify reservations. This preserves event wake delivery without recursively
entering the lock taken by the ordinary guest-memory write accessors.

`Finalize` returns `BUSY` while a policy is admitted or initialization/teardown
is in progress. That response leaves admission and attached queues unchanged.
A quiescent finalize stops, wakes, joins, and closes only its own coordinator and
workers. It retires that instance's resident policy contexts and registrations,
then cancels its attached queues and releases their port bindings. The same EA
can subsequently be initialized again. Removing an admitted workload likewise
returns `BUSY`; changing its maximum contention affects subsequent admission,
without terminating already admitted work.

Resident policy contexts are keyed by instance EA, workload ID, and physical
SPU number. A bounded directory tracks active users, so waiting for a context
lock does not block unrelated instances. Retirement waits for that instance's
users. Replacing a policy or workload data resets local store after releasing
its coherency registration and MFC resources. Kernel-exit classification and
per-run poll counters are thread local.

Priority zero excludes that physical lane. A missing priority table retains the
existing HLE default of allowing each configured lane, represented by priority
one in the published guest table. SetPriorities updates both host and guest
fields. WakeUp only signals the bounded coordinator; it never dispatches an
extra policy thread.

The dynamic per-context MFC/coherency ownership fix is a dependency: resident
contexts must retain independent pending DMA and reservation state across
kernel returns, including more than eight distinct contexts.

The existing global public workload-ID allocation and event-port conventions
remain. This change does not implement per-instance public workload numbering,
priority ordering or preemption, or arbitrary cancellation of guest code that never
returns. A blocked workload still requires an available physical slot for its
producer. Instances with all configured SPUs occupied cannot gain capacity by
creating additional workers.

Run `python3 tools/test_spurs_scheduler.py` on a POSIX host. The fixture links
the real SDK scheduler, policy runner, halt/unwind path, channels, and coherency
implementation with manufactured policy modules. It checks producer/consumer
progress, contention and worker caps, stable physical slots, fair admission,
BUSY immutability, removal, changed contention, reservation notifications,
joined reinitialization, queue ownership, independent instances, resident local
store isolation with identical workload/lane IDs, and concurrent kernel-exit
versus stop classification. It contains no title assets. The prior synchronous
scheduler fails the producer/consumer regression.

`python3 tools/test_spurs_self_admit.py` runs the self-admission fixture with
the switch both ways: contention ceilings, priority rows and CURCONT under
churn with fake SPU kicks, requests raised while every lane is admitted (SPU
commit, API, plain store), a plain store picked up by a free lane, freed
contention reaching an idle lane, two concurrent instances, and the per-run
scheduler cost. `SANITIZE=thread` or `SANITIZE=address` builds it with that
sanitizer; the thread build uses `tests/tsan_spurs_guest_memory.supp`, which
suppresses only accesses through the guest-memory accessors.
