// Copyright(c) 2026 Javier Blanco
// SPDX-License-Identifier: EPL-2.0 OR BSD-3-Clause

#ifndef DDSRT_ZEPHYR_SYNC_H
#define DDSRT_ZEPHYR_SYNC_H

#include <stdint.h>
#include <zephyr/kernel.h>

typedef struct {
  struct k_mutex mutex;
  struct k_spinlock state_lock;
  k_tid_t owner;
  unsigned int acquisitions;
  unsigned int waiters;
} ddsrt_mutex_t;

typedef struct {
  struct k_condvar cond;
  atomic_t waiters;
} ddsrt_cond_t;

typedef ddsrt_cond_t ddsrt_cond_wctime_t;
typedef ddsrt_cond_t ddsrt_cond_mtime_t;
typedef ddsrt_cond_t ddsrt_cond_etime_t;

typedef struct {
  struct k_mutex guard;
  struct k_condvar changed;
  unsigned int readers;
  unsigned int waiting_writers;
  k_tid_t writer;
} ddsrt_rwlock_t;

typedef struct {
  unsigned char state;
} ddsrt_once_t;
#define DDSRT_ONCE_INIT { 0 }

#endif /* DDSRT_ZEPHYR_SYNC_H */
