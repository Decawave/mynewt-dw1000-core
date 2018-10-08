/*
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

/**
 * @file dw1000_tdma.c
 * @author paul kettle
 * @date 2018
 * @brief TDMA 
 *
 * @details This is the base class of tdma which initialises tdma instance, assigns slots for each node and does ranging continuously based on
 * addresses.
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
#include <tdma/dw1000_tdma.h>

//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

static void tdma_superframe_event_cb(struct os_event * ev);
static void slot_timer_cb(void * arg);
static bool tdma_rx_complete_cb(struct _dw1000_dev_instance_t * inst);
static bool tdma_tx_complete_cb(struct _dw1000_dev_instance_t * inst);

#ifdef TDMA_TASKS_ENABLE
static void tdma_tasks_init(struct _tdma_instance_t * inst);
static void tdma_task(void *arg);
#endif

/** 
 * API to initialise the tdma instance. Sets the clkcal postprocess and 
 * assings the slot callback function for slot0. 
 *
 * @param inst     Pointer to  _dw1000_dev_instance_t. 
 * @param period   CCP period.
 * @param nslots   Total slots to be allocated between two frames.
 * @return tdma_instance_t* 
 */

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
        tdma->task_prio = inst->task_prio + 2;
#endif
        inst->tdma = tdma;
    }else{
        tdma = inst->tdma;
    }

    dw1000_extension_callbacks_t tdma_cbs = {
        .id = DW1000_TDMA,
        .rx_complete_cb = tdma_rx_complete_cb,
        .tx_complete_cb = tdma_tx_complete_cb
    };
   
    dw1000_add_extension_callbacks(inst, tdma_cbs);

#ifdef TDMA_TASKS_ENABLE
    os_callout_init(&tdma->event_cb, &tdma->eventq, tdma_superframe_event_cb, (void *) tdma);
#else
    os_callout_init(&tdma->event_cb, &inst->eventq, tdma_superframe_event_cb, (void *) tdma);
#endif

    tdma->status.initialized = true;
    tdma->os_epoch = os_cputime_get32();

#ifdef TDMA_TASKS_ENABLE
    tdma_tasks_init(tdma);
#endif
    return inst->tdma;
}

/**
 * API to free memory allocated for tdma slots.
 *
 * @param inst  Pointer to tdma_instance_t.
 *
 * @return void
 */
void 
tdma_free(tdma_instance_t * inst){
    assert(inst);  
    if (inst->status.selfmalloc)
        free(inst);
    else
        inst->status.initialized = 0;
}


#ifdef TDMA_TASKS_ENABLE
/**
 * API to initialise a higher priority task for the tdma slot tasks.
 *
 * @param inst  Pointer to  _tdma_instance_t.
 *
 * @return void
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

/**
 * API for task function of tdma.
 *
 * @param arg   Pointer to an argument of void type.
 *
 * @return void
 */

static void tdma_task(void *arg)
{
    tdma_instance_t * inst = arg;
    while (1) {
        os_eventq_run(&inst->eventq);
    }
}
#endif

/**
 * Interrupt context tdma_rx_complete callback. Used to define eopch for tdma actavities 
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @return bool based on the totality of the handling which is false this implementation. 
 */

static bool 
tdma_rx_complete_cb(struct _dw1000_dev_instance_t * inst){

    tdma_instance_t * tdma = inst->tdma;
   
    if (inst->fctrl_array[0] == FCNTL_IEEE_BLINK_CCP_64){
        DIAGMSG("{\"utime\": %lu,\"msg\": \"tdma_rx_complete_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
        if (inst->tdma != NULL && inst->tdma->status.initialized){
            tdma->os_epoch = os_cputime_get32();
#ifdef TDMA_TASKS_ENABLE
            os_eventq_put(&inst->tdma->eventq, &inst->tdma->event_cb.c_ev);
#else
            os_eventq_put(&inst->eventq, &inst->tdma->event_cb.c_ev);
#endif
        }   
        return true;
    }
    return false;
}

/**
 * Interrupt context tdma_tx_complete callback. Used to define eopch for tdma actavities 
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @return bool based on the totality of the handling which is false this implementation. 
 */

static bool 
tdma_tx_complete_cb(struct _dw1000_dev_instance_t * inst){

   tdma_instance_t * tdma = inst->tdma;
   
    if (inst->fctrl_array[0] == FCNTL_IEEE_BLINK_CCP_64){
        DIAGMSG("{\"utime\": %lu,\"msg\": \"tdma_tx_complete_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
        if (inst->tdma != NULL && inst->tdma->status.initialized){
            tdma->os_epoch = os_cputime_get32();
#ifdef TDMA_TASKS_ENABLE
            os_eventq_put(&inst->tdma->eventq, &inst->tdma->event_cb.c_ev);
#else
            os_eventq_put(&inst->eventq, &inst->tdma->event_cb.c_ev);
#endif
        }   
        return true;
    }
    return false;
}



/**
 * API to intialise slot instance for the slot.Also initialise a timer and assigns callback for each slot.
 *
 * @param inst       Pointer to _tdma_instance_t.
 * @param callout    Callback for the particular slot.
 * @param idx        Slot number.
 * @param arg        Argument to the callback.
 *
 * @return void
 */

void 
tdma_assign_slot(struct _tdma_instance_t * inst, void (* callout )(struct os_event *), uint16_t idx, void * arg){

    assert(idx < inst->nslots);

    if (inst->status.initialized == false)
       return;

    if (inst->slot[idx] == NULL){
        inst->slot[idx] = (tdma_slot_t  *) malloc(sizeof(struct _tdma_slot_t)); 
        assert(inst->slot[idx]);
        memset(inst->slot[idx], 0, sizeof(struct _tdma_slot_t));
    }else{
        memset(inst->slot[idx], 0, sizeof(struct _tdma_slot_t));
    }
    inst->slot[idx]->idx = idx;
    inst->slot[idx]->parent = inst;
    inst->slot[idx]->arg = arg;

    os_cputime_timer_init(&inst->slot[idx]->timer, slot_timer_cb, (void *) inst->slot[idx]);
#ifdef TDMA_TASKS_ENABLE
    os_callout_init(&inst->slot[idx]->event_cb, &inst->eventq, callout, (void *) inst->slot[idx]);
#else
    os_callout_init(&inst->slot[idx]->event_cb, &inst->parent->eventq, callout, (void *) inst->slot[idx]);
#endif
}

/**
 * API to free the slot instance.
 *
 * @param inst   Pointer to _tdma_instance_t.
 * @param idx    Slot number.
 *
 * @return void
 */
void 
tdma_release_slot(struct _tdma_instance_t * inst, uint16_t idx){
    assert(idx < inst->nslots);
    assert(inst->slot[idx]);
    free(inst->slot[idx]);
    inst->slot[idx] =  NULL;
}

/** 
 * This event is generated by ccp/clkcal complete event. This event defines the start of an superframe epoch. 
 * The event also schedules a tdma_superframe_timer_cb which turns on the receiver in advance of the next superframe epoch. 
 *
 * @param ev   Pointer to os_event.
 *
 * @return void 
 */
static void 
tdma_superframe_event_cb(struct os_event * ev){
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);
    
    DIAGMSG("{\"utime\": %lu,\"msg\": \"tdma_superframe_event_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

    tdma_instance_t * tdma = (void *)ev->ev_arg;
    for (uint16_t i = 0; i < tdma->nslots; i++) {
        if (tdma->slot[i]){
            os_cputime_timer_stop(&tdma->slot[i]->timer);
        }
    }
    for (uint16_t i = 0; i < tdma->nslots; i++) {
        if (tdma->slot[i]){
            hal_timer_start_at(&tdma->slot[i]->timer, 
                tdma->os_epoch 
                + os_cputime_usecs_to_ticks((uint32_t) (i * dw1000_dwt_usecs_to_usecs(tdma->period)/tdma->nslots))
                - (uint32_t)ceilf(dw1000_phy_SHR_duration(&tdma->parent->attrib))
                - os_cputime_usecs_to_ticks(MYNEWT_VAL(OS_LATENCY))
            );
        }
    }
}



/**
 * API for each slot. This function then puts
 * the callback provided by the user in the tdma event queue.
 *
 * @param arg    A void type argument. 
 *
 * @return void
 */
static void 
slot_timer_cb(void * arg){

    assert(arg);

    tdma_slot_t * slot = (tdma_slot_t *) arg;
    tdma_instance_t * tdma = slot->parent;

    //DIAGMSG("{\"utime\": %lu,\"msg\": \"slot_timer_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
  
#ifdef TDMA_TASKS_ENABLE
    os_eventq_put(&tdma->eventq, &slot->event_cb.c_ev);
#else
    os_eventq_put(&tdma->parent->eventq, &slot->event_cb.c_ev);
#endif
}


