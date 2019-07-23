/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#ifndef _DPL_OS_TYPES_H
#define _DPL_OS_TYPES_H

#include <time.h>
#include <sys/time.h>

#include <signal.h>
#include <stdbool.h>
#include <pthread.h>
#include <semaphore.h>

#include "dpl/dpl_eventq.h"
#include "dpl/dpl_time.h"

#ifndef UINT32_MAX
#define UINT32_MAX  0xFFFFFFFFUL
#endif

#ifndef INT32_MAX
#define INT32_MAX   0x7FFFFFFFL
#endif

#define OS_TIME_MAX UINT32_MAX
#define OS_STIME_MAX INT32_MAX

/* Used to wait forever for events and mutexs */
#define OS_TIMEOUT_NEVER (OS_TIME_MAX)
#define DPL_WAIT_FOREVER (OS_TIMEOUT_NEVER)

/* The highest and lowest task priorities */
#define OS_TASK_PRI_HIGHEST (sched_get_priority_max(SCHED_RR))
#define OS_TASK_PRI_LOWEST  (sched_get_priority_min(SCHED_RR))


//typedef uint32_t timer_t;

//typedef int os_sr_t;
typedef int dpl_stack_t;

struct dpl_event {
    uint8_t             ev_queued;
    dpl_event_fn        *ev_cb;
    void                *ev_arg;
};

struct dpl_eventq {
    void               *q;
};

struct dpl_callout {
    struct dpl_event    c_ev;
    struct dpl_eventq  *c_evq;
    uint32_t    c_ticks;
    timer_t     c_timer;
    bool        c_active;
};

struct dpl_mutex {
    pthread_mutex_t         lock;
    pthread_mutexattr_t     attr;
    struct timespec         wait;
};

struct dpl_sem {
    sem_t                   lock;
};

struct dpl_task {
    pthread_t               handle;
    pthread_attr_t          attr;
    struct sched_param      param;
    const char*             name;
};



#endif // _DPL_OS_TYPES_H
