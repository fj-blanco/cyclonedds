// Copyright(c) 2026 Javier Blanco
// SPDX-License-Identifier: EPL-2.0 OR BSD-3-Clause

#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <stdlib.h>

#include "dds/ddsrt/sync.h"
#include "dds/ddsrt/time.h"

/* DDSRT still uses POSIX threads and clocks with this experimental option. */
static void check(int result)
{
  if (result != 0)
    abort();
}

void ddsrt_mutex_init(ddsrt_mutex_t *mutex)
{
  check(k_mutex_init(&mutex->mutex));
  mutex->state_lock = (struct k_spinlock) { 0 };
  mutex->owner = NULL;
  mutex->acquisitions = 0;
  mutex->waiters = 0;
}

void ddsrt_mutex_destroy(ddsrt_mutex_t *mutex)
{
  k_spinlock_key_t key = k_spin_lock(&mutex->state_lock);
  bool idle = mutex->owner == NULL && mutex->acquisitions == 0 && mutex->waiters == 0;
  k_spin_unlock(&mutex->state_lock, key);
  if (!idle)
    abort();
  /* Zephyr has no k_mutex_destroy; reinitialization is supported. */
}

void ddsrt_mutex_lock(ddsrt_mutex_t *mutex)
{
  k_spinlock_key_t key = k_spin_lock(&mutex->state_lock);
  mutex->waiters++;
  k_spin_unlock(&mutex->state_lock, key);
  check(k_mutex_lock(&mutex->mutex, K_FOREVER));
  key = k_spin_lock(&mutex->state_lock);
  mutex->waiters--;
  if (mutex->owner == k_current_get()) {
    mutex->acquisitions++;
  } else {
    mutex->owner = k_current_get();
    mutex->acquisitions = 1;
  }
  k_spin_unlock(&mutex->state_lock, key);
}

bool ddsrt_mutex_trylock(ddsrt_mutex_t *mutex)
{
  k_spinlock_key_t key = k_spin_lock(&mutex->state_lock);
  bool self_owned = mutex->owner == k_current_get();
  k_spin_unlock(&mutex->state_lock, key);
  /* pthread_mutex_trylock on the default DDSRT mutex is not recursive. */
  if (self_owned)
    return false;
  int result = k_mutex_lock(&mutex->mutex, K_NO_WAIT);
  if (result == -EBUSY)
    return false;
  check(result);
  key = k_spin_lock(&mutex->state_lock);
  assert(mutex->owner == NULL);
  mutex->owner = k_current_get();
  mutex->acquisitions = 1;
  k_spin_unlock(&mutex->state_lock, key);
  return true;
}

void ddsrt_mutex_unlock(ddsrt_mutex_t *mutex)
{
  k_spinlock_key_t key = k_spin_lock(&mutex->state_lock);
  bool owned = mutex->owner == k_current_get() && mutex->acquisitions > 0;
  if (owned && --mutex->acquisitions == 0)
    mutex->owner = NULL;
  k_spin_unlock(&mutex->state_lock, key);
  if (!owned)
    abort();
  check(k_mutex_unlock(&mutex->mutex));
}

static void cond_init(ddsrt_cond_t *cond)
{
  check(k_condvar_init(&cond->cond));
  atomic_set(&cond->waiters, 0);
}

static void cond_destroy(ddsrt_cond_t *cond)
{
  if (atomic_get(&cond->waiters) != 0)
    abort();
  /* Zephyr has no k_condvar_destroy; reinitialization is supported. */
}

static int cond_wait(ddsrt_cond_t *cond, ddsrt_mutex_t *mutex, k_timeout_t timeout)
{
  k_spinlock_key_t key = k_spin_lock(&mutex->state_lock);
  bool owned_once = mutex->owner == k_current_get() && mutex->acquisitions == 1;
  if (owned_once) {
    mutex->owner = NULL;
    mutex->acquisitions = 0;
    mutex->waiters++;
  }
  k_spin_unlock(&mutex->state_lock, key);
  if (!owned_once)
    abort();

  atomic_inc(&cond->waiters);
  int result = k_condvar_wait(&cond->cond, &mutex->mutex, timeout);
  atomic_dec(&cond->waiters);
  key = k_spin_lock(&mutex->state_lock);
  assert(mutex->owner == NULL);
  mutex->waiters--;
  mutex->owner = k_current_get();
  mutex->acquisitions = 1;
  k_spin_unlock(&mutex->state_lock, key);
  if (result != 0 && result != -EAGAIN)
    abort();
  return result;
}

static void cond_wait_forever(ddsrt_cond_t *cond, ddsrt_mutex_t *mutex)
{
  check(cond_wait(cond, mutex, K_FOREVER));
}

static int64_t clock_now(unsigned int clock)
{
  switch (clock) {
    case 0: return ddsrt_time_wallclock().v;
    case 1: return ddsrt_time_monotonic().v;
    default: return ddsrt_time_elapsed().v;
  }
}

static k_timeout_t finite_timeout(int64_t remaining_ns)
{
  BUILD_ASSERT(CONFIG_SYS_CLOCK_TICKS_PER_SEC > 0 &&
               CONFIG_SYS_CLOCK_TICKS_PER_SEC <= 1000000000,
               "tick conversion requires a positive rate at most 1 GHz");
#ifdef CONFIG_TIMEOUT_64BIT
  const uint64_t max_ticks = INT64_MAX / 4;
#else
  const uint64_t max_ticks = INT32_MAX / 4;
#endif
  const uint64_t hz = CONFIG_SYS_CLOCK_TICKS_PER_SEC;
  const uint64_t seconds = (uint64_t)remaining_ns / DDS_NSECS_IN_SEC;
  const uint64_t nanos = (uint64_t)remaining_ns % DDS_NSECS_IN_SEC;
  uint64_t ticks;
  if (seconds >= max_ticks / hz) {
    ticks = max_ticks;
  } else {
    ticks = seconds * hz +
      (nanos * hz + DDS_NSECS_IN_SEC - 1) / DDS_NSECS_IN_SEC;
    if (ticks > max_ticks)
      ticks = max_ticks;
  }
  if (ticks == 0)
    ticks = 1;
  return K_TICKS((k_ticks_t)ticks);
}

static bool cond_waituntil(ddsrt_cond_t *cond, ddsrt_mutex_t *mutex,
                           int64_t deadline, unsigned int clock)
{
  if (deadline == DDS_NEVER) {
    cond_wait_forever(cond, mutex);
    return true;
  }
  for (;;) {
    int64_t now = clock_now(clock);
    if (now < 0)
      abort();
    if (deadline <= now)
      return false;
    if (cond_wait(cond, mutex, finite_timeout(deadline - now)) == 0)
      return true;
    /* A chunk or a rounded timeout expired; retain the original deadline. */
  }
}

#define COND_VARIANT(suffix, type, clock) \
  void ddsrt_cond##suffix##_init(type *cond) { cond_init(cond); } \
  void ddsrt_cond##suffix##_destroy(type *cond) { cond_destroy(cond); } \
  void ddsrt_cond##suffix##_wait(type *cond, ddsrt_mutex_t *mutex) \
    { cond_wait_forever(cond, mutex); } \
  void ddsrt_cond##suffix##_signal(type *cond) \
    { check(k_condvar_signal(&cond->cond)); } \
  void ddsrt_cond##suffix##_broadcast(type *cond) \
    { if (k_condvar_broadcast(&cond->cond) < 0) abort(); }

COND_VARIANT(, ddsrt_cond_t, 0)
COND_VARIANT(_wctime, ddsrt_cond_wctime_t, 0)
COND_VARIANT(_mtime, ddsrt_cond_mtime_t, 1)
COND_VARIANT(_etime, ddsrt_cond_etime_t, 2)

bool ddsrt_cond_wctime_waituntil(ddsrt_cond_wctime_t *cond, ddsrt_mutex_t *mutex,
                                  ddsrt_wctime_t abstime)
{ return cond_waituntil(cond, mutex, abstime.v, 0); }
bool ddsrt_cond_mtime_waituntil(ddsrt_cond_mtime_t *cond, ddsrt_mutex_t *mutex,
                                 ddsrt_mtime_t abstime)
{ return cond_waituntil(cond, mutex, abstime.v, 1); }
bool ddsrt_cond_etime_waituntil(ddsrt_cond_etime_t *cond, ddsrt_mutex_t *mutex,
                                 ddsrt_etime_t abstime)
{ return cond_waituntil(cond, mutex, abstime.v, 2); }

void ddsrt_rwlock_init(ddsrt_rwlock_t *rwlock)
{
  check(k_mutex_init(&rwlock->guard));
  check(k_condvar_init(&rwlock->changed));
  rwlock->readers = 0;
  rwlock->waiting_writers = 0;
  rwlock->writer = NULL;
}

void ddsrt_rwlock_destroy(ddsrt_rwlock_t *rwlock)
{
  check(k_mutex_lock(&rwlock->guard, K_FOREVER));
  bool idle = rwlock->readers == 0 && rwlock->waiting_writers == 0 && rwlock->writer == NULL;
  check(k_mutex_unlock(&rwlock->guard));
  if (!idle)
    abort();
}

void ddsrt_rwlock_read(ddsrt_rwlock_t *rwlock)
{
  check(k_mutex_lock(&rwlock->guard, K_FOREVER));
  while (rwlock->writer != NULL || rwlock->waiting_writers != 0)
    check(k_condvar_wait(&rwlock->changed, &rwlock->guard, K_FOREVER));
  rwlock->readers++;
  check(k_mutex_unlock(&rwlock->guard));
}

void ddsrt_rwlock_write(ddsrt_rwlock_t *rwlock)
{
  check(k_mutex_lock(&rwlock->guard, K_FOREVER));
  rwlock->waiting_writers++;
  while (rwlock->writer != NULL || rwlock->readers != 0)
    check(k_condvar_wait(&rwlock->changed, &rwlock->guard, K_FOREVER));
  rwlock->waiting_writers--;
  rwlock->writer = k_current_get();
  check(k_mutex_unlock(&rwlock->guard));
}

bool ddsrt_rwlock_tryread(ddsrt_rwlock_t *rwlock)
{
  int result = k_mutex_lock(&rwlock->guard, K_NO_WAIT);
  if (result == -EBUSY)
    return false;
  check(result);
  bool acquired = rwlock->writer == NULL && rwlock->waiting_writers == 0;
  if (acquired)
    rwlock->readers++;
  check(k_mutex_unlock(&rwlock->guard));
  return acquired;
}

bool ddsrt_rwlock_trywrite(ddsrt_rwlock_t *rwlock)
{
  int result = k_mutex_lock(&rwlock->guard, K_NO_WAIT);
  if (result == -EBUSY)
    return false;
  check(result);
  bool acquired = rwlock->writer == NULL && rwlock->readers == 0;
  if (acquired)
    rwlock->writer = k_current_get();
  check(k_mutex_unlock(&rwlock->guard));
  return acquired;
}

void ddsrt_rwlock_unlock(ddsrt_rwlock_t *rwlock)
{
  check(k_mutex_lock(&rwlock->guard, K_FOREVER));
  if (rwlock->writer == k_current_get()) {
    rwlock->writer = NULL;
  } else if (rwlock->readers > 0) {
    rwlock->readers--;
  } else {
    abort();
  }
  if (rwlock->writer == NULL && rwlock->readers == 0 &&
      k_condvar_broadcast(&rwlock->changed) < 0)
    abort();
  check(k_mutex_unlock(&rwlock->guard));
}

K_MUTEX_DEFINE(once_guard);
K_CONDVAR_DEFINE(once_changed);

void ddsrt_once(ddsrt_once_t *control, ddsrt_once_fn init_fn)
{
  check(k_mutex_lock(&once_guard, K_FOREVER));
  while (control->state == 1)
    check(k_condvar_wait(&once_changed, &once_guard, K_FOREVER));
  if (control->state == 2) {
    check(k_mutex_unlock(&once_guard));
    return;
  }
  assert(control->state == 0);
  control->state = 1;
  check(k_mutex_unlock(&once_guard));
  init_fn();
  check(k_mutex_lock(&once_guard, K_FOREVER));
  control->state = 2;
  if (k_condvar_broadcast(&once_changed) < 0)
    abort();
  check(k_mutex_unlock(&once_guard));
}
