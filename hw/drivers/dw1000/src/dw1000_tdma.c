/**
 * Copyright (C) 2017-2018, Decawave Limited, All Rights Reserved
 * 
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

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "sysinit/sysinit.h"
#include "os/os.h"
#include "os/queue.h"
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#include "hal/hal_bsp.h"
#ifdef ARCH_sim
#include "mcu/mcu_sim.h"
#endif

#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>

#if MYNEWT_VAL(DW1000_CCP_ENABLED)
#include <dw1000/dw1000_ccp.h>
#endif

#if MYNEWT_VAL(TDMA_ENABLED) 
#include <dw1000/dw1000_tdma.h>

static void tdma_superframe_event_cb(struct os_event * ev);
static void slot_timer_cb(void * arg);
static void slot0_event_cb(struct os_event * ev);

#ifdef TDMA_TASKS_ENABLE
static void tdma_tasks_init(struct _tdma_instance_t * inst);
static void tdma_task(void *arg);
#endif

tdma_instance_t * 
tdma_init(struct _dw1000_dev_instance_t * inst, uint32_t period, uint16_t nslots){
    assert(inst);
    tdma_instance_t * tdma;
    if (inst->tdma == NULL) {
        tdma = (tdma_instance_t *) malloc(sizeof(struct _tdma_instance_t) + nslots * sizeof(struct _tdma_slot_t *)); 
        assert(tdma);
        memset(tdma, 0, sizeof(tdma_instance_t) + nslots * sizeof(struct _tdma_slot_t * ));
        tdma->status.selfmalloc = 1;
        os_error_t err = os_mutex_init(&tdma->mutex);
        assert(err == OS_OK);
        tdma->nslots = nslots; 
        tdma->period = period; 
        tdma->parent = inst;
#ifdef TDMA_TASKS_ENABLE
        tdma->task_prio = inst->task_prio + 1;
#endif
        inst->tdma = tdma;
    }else{
        tdma = inst->tdma;
    }
    tdma->status.awaiting_superframe = 1; 
#if MYNEWT_VAL(DW1000_CCP_ENABLED)    
    clkcal_set_postprocess(inst->ccp->clkcal, (void * )tdma_superframe_event_cb);
#endif
    
    tdma_assign_slot(tdma, slot0_event_cb, 0, NULL);
    os_cputime_timer_init(&tdma->slot[0]->timer, slot_timer_cb, (void *) tdma->slot[0]);
    os_cputime_timer_relative(&tdma->slot[0]->timer, (tdma->period - MYNEWT_VAL(OS_LATENCY)));

    inst->status.initialized = true;

#ifdef TDMA_TASKS_ENABLE
    tdma_tasks_init(tdma);
#endif
    return inst->tdma;
}

void 
tdma_free(tdma_instance_t * inst){
    assert(inst);  
    if (inst->status.selfmalloc)
        free(inst);
    else
        inst->status.initialized = 0;
}


#ifdef TDMA_TASKS_ENABLE
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tdma_tasks_init()
 *
 */
static void tdma_tasks_init(struct _tdma_instance_t * inst)
{
    /* Check if the tasks are already initiated */
    if (!os_eventq_inited(&inst->eventq))
    {
        /* Use a dedicate event queue for tdma events */
        os_eventq_init(&inst->eventq);
        os_task_init(&inst->task_str, "dw1000_tdma",
                     tdma_task,
                     (void *) inst,
                     inst->task_prio, OS_WAIT_FOREVER,
                     inst->task_stack,
                     DW1000_DEV_TASK_STACK_SZ);
    }       
}

static void tdma_task(void *arg)
{
    tdma_instance_t * inst = arg;
    while (1) {
        os_eventq_run(&inst->eventq);
    }
}

#endif

void 
tdma_assign_slot(struct _tdma_instance_t * inst, void (* callout )(struct os_event *), uint16_t idx, void * arg){

    assert(idx < inst->nslots);

    if (inst->status.initialized && idx == 0)
        assert(0);

    if (inst->slot[idx] == NULL){
        inst->slot[idx] = (tdma_slot_t  *) malloc(sizeof(struct _tdma_slot_t)); 
        assert(inst->slot[idx]);
        memset(inst->slot[idx], 0, sizeof(struct _tdma_slot_t));
    }else{
        memset(inst->slot[idx], 0, sizeof(struct _tdma_slot_t));
    }
    inst->slot[idx]->idx = idx;
    inst->slot[idx]->parent = inst;

    os_cputime_timer_init(&inst->slot[idx]->timer, slot_timer_cb, (void *) inst->slot[idx]);
#ifdef TDMA_TASKS_ENABLE
    os_callout_init(&inst->slot[idx]->event_cb, &inst->eventq, callout, (void *) inst->slot[idx]);
#else
    os_callout_init(&inst->slot[idx]->event_cb, &inst->parent->eventq, callout, (void *) inst->slot[idx]);
#endif
}

void 
tdma_release_slot(struct _tdma_instance_t * inst, uint16_t idx){
    assert(idx < inst->nslots);
    assert(inst->slot[idx]);
    free(inst->slot[idx]);
    inst->slot[idx] =  NULL;
}

/*! 
 * @fn tmda_superframe_event_cb(struct os_event * ev)
 *
 * @brief This event is generated by ccp/clkcal complete event. This event defines the start of an superframe epoch. 
 * The event also schedules a tdma_superframe_timer_cb which turns on the receiver in advance of the next superframe epoch. 
 *
 * input parameters
 * @param inst - struct os_event *  
 *
 * output parameters
 *
 * returns none 
 */
static void 
tdma_superframe_event_cb(struct os_event * ev){
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

//    uint32_t utime = os_cputime_ticks_to_usecs(os_cputime_get32());
//    printf("{\"utime\": %lu,\"msg\": \"superframe_event_cb\"}\n",utime);

    clkcal_instance_t * clkcal = (clkcal_instance_t *)ev->ev_arg;
    dw1000_ccp_instance_t * ccp = (void *)clkcal->ccp; 
    dw1000_dev_instance_t * inst = ccp->parent;
    tdma_instance_t * tdma = inst->tdma;
    uint32_t cputime = os_cputime_get32() - os_cputime_usecs_to_ticks(MYNEWT_VAL(OS_LATENCY));
    
    tdma->status.awaiting_superframe = 0;
    hal_timer_start_at(&tdma->slot[0]->timer, cputime + os_cputime_usecs_to_ticks(dw1000_dwt_usecs_to_usecs(tdma->period)));
    for (uint16_t i = 1; i < tdma->nslots; i++) {
        if (tdma->slot[i]){
            hal_timer_start_at(&tdma->slot[i]->timer, cputime + os_cputime_usecs_to_ticks(dw1000_dwt_usecs_to_usecs(i * tdma->period/tdma->nslots)));            
        }
    }
}


/*! 
 * @fn slot0_event_cb(void * arg)
 *
 * @brief 
 *
 * input parameters
 * @param inst - void * arg 
 *
 * output parameters
 *
 * returns none 
 */
static void 
slot0_event_cb(struct os_event * ev){

    assert(ev);
    tdma_slot_t * slot = (tdma_slot_t *) ev->ev_arg;
    tdma_instance_t * tdma = slot->parent;
    dw1000_dev_instance_t * inst = tdma->parent;
    
//    uint32_t utime = os_cputime_ticks_to_usecs(os_cputime_get32());
//    printf("{\"utime\": %lu,\"msg\": \"slot0_event_cb\"}\n",utime);
 
    if (inst->status.sleeping)
        dw1000_dev_wakeup(inst);
  
    for (uint16_t i = 1; i < tdma->nslots; i++) {
        if (tdma->slot[i]){
            os_cputime_timer_stop(&tdma->slot[i]->timer);
        }
    }
 
    tdma->status.awaiting_superframe = 1; 
    dw1000_set_delay_start(inst, 0);
    dw1000_set_rx_timeout(inst, 0);
    if(dw1000_start_rx(inst).start_rx_error){
        uint32_t utime = os_cputime_ticks_to_usecs(os_cputime_get32());
        printf("{\"utime\": %lu,\"msg\": \"slot0_timer_cb:start_rx_error\"}\n",utime);
    }
}

/*! 
 * @fn slot_event_cb(void * arg)
 *
 * @brief 
 *
 * input parameters
 * @param inst - void * arg 
 *
 * output parameters
 *
 * returns none 
 */
static void 
slot_timer_cb(void * arg){

    assert(arg);

    tdma_slot_t * slot = (tdma_slot_t *) arg;
    tdma_instance_t * tdma = slot->parent;

  // uint32_t utime = os_cputime_ticks_to_usecs(os_cputime_get32());
  //  printf("{\"utime\": %lu,\"msg\": \"slot[%d]_timer_cb\"}\n",utime, slot->idx);

#ifdef TDMA_TASKS_ENABLE
    os_eventq_put(&tdma->eventq, &slot->event_cb.c_ev);
#else
    os_eventq_put(&tdma->parent->eventq, &slot->event_cb.c_ev);
#endif
}


#endif //MYNEWT_VAL(TDMA_ENABLED)