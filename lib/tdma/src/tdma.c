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
#include <tdma/tdma.h>

#if MYNEWT_VAL(CCP_ENABLED)
#include <ccp/ccp.h>
#endif

#include <stats/stats.h>

STATS_NAME_START(tdma_stat_section)
    STATS_NAME(tdma_stat_section, slot_timer_cnt)
    STATS_NAME(tdma_stat_section, superframe_cnt)
    STATS_NAME(tdma_stat_section, rx_complete)
    STATS_NAME(tdma_stat_section, tx_complete)
STATS_NAME_END(tdma_stat_section)

//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

static void tdma_superframe_event_cb(struct os_event * ev);
static void slot_timer_cb(void * arg);
static bool rx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);
static bool tx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);

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
        memset(tdma, 0, sizeof(struct _tdma_instance_t) + nslots * sizeof(struct _tdma_slot_t * ));
        tdma->status.selfmalloc = 1;
        os_error_t err = os_mutex_init(&tdma->mutex);
        assert(err == OS_OK);
        tdma->nslots = nslots; 
        tdma->period = period; 
        tdma->parent = inst;
#ifdef TDMA_TASKS_ENABLE
        tdma->task_prio = inst->task_prio + 0x4;
#endif
        inst->tdma = tdma;
    }else{
        tdma = inst->tdma;
    }

    inst->tdma->cbs = (dw1000_mac_interface_t){
        .id = DW1000_TDMA,
        .tx_complete_cb = tx_complete_cb,
        .rx_complete_cb = rx_complete_cb
    };
    dw1000_mac_append_interface(inst, &inst->tdma->cbs);
    
    int rc = stats_init(
                STATS_HDR(inst->tdma->stat),
                STATS_SIZE_INIT_PARMS(inst->tdma->stat, STATS_SIZE_32),
                STATS_NAME_INIT_PARMS(tdma_stat_section)
            );
    assert(rc == 0);

#if  MYNEWT_VAL(DW1000_DEVICE_0) && !MYNEWT_VAL(DW1000_DEVICE_1)
    rc = stats_register("tdma", STATS_HDR(inst->tdma->stat));
#elif  MYNEWT_VAL(DW1000_DEVICE_0) && MYNEWT_VAL(DW1000_DEVICE_1)
    if (inst->idx == 0)
        rc |= stats_register("tdma0", STATS_HDR(inst->tdma->stat));
    else
        rc |= stats_register("tdma1", STATS_HDR(inst->tdma->stat));
#endif
    assert(rc == 0);

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

/**
 * API to initialise the package, only one ccp service required in the system.
 *
 *
 * @return void
 */

void tdma_pkg_init(void){

    printf("{\"utime\": %lu,\"msg\": \"tdma_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

#if MYNEWT_VAL(DW1000_DEVICE_0) 
        tdma_init(hal_dw1000_inst(0), MYNEWT_VAL(TDMA_PERIOD), MYNEWT_VAL(TDMA_NSLOTS)); 
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
        tdma_init(hal_dw1000_inst(1), MYNEWT_VAL(TDMA_PERIOD), MYNEWT_VAL(TDMA_NSLOTS)); 
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
        tdma_init(hal_dw1000_inst(2), MYNEWT_VAL(TDMA_PERIOD), MYNEWT_VAL(TDMA_NSLOTS));     
#endif

}

#ifdef TDMA_TASKS_ENABLE


/**
 * API to feed the sanity watchdog
 *
 * @return void
 */
#if MYNEWT_VAL(TDMA_SANITY_INTERVAL) > 0 
static void
sanity_feeding_cb(struct os_event * ev)
{
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    tdma_instance_t * tdma = (void *)ev->ev_arg;
    os_sanity_task_checkin(0);
    os_callout_reset(&tdma->sanity_cb, OS_TICKS_PER_SEC);
}
#endif

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
                     inst->task_prio,
#if MYNEWT_VAL(TDMA_SANITY_INTERVAL) > 0
                     OS_TICKS_PER_SEC * MYNEWT_VAL(TDMA_SANITY_INTERVAL),
#else
                     OS_WAIT_FOREVER,
#endif
                     inst->task_stack,
                     DW1000_DEV_TASK_STACK_SZ);
    }       
#if MYNEWT_VAL(TDMA_SANITY_INTERVAL) > 0
    os_callout_init(&inst->sanity_cb, &inst->eventq, sanity_feeding_cb, (void *) inst);
    os_callout_reset(&inst->sanity_cb, OS_TICKS_PER_SEC);
#endif
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
rx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){

    tdma_instance_t * tdma = inst->tdma;
   
    if (inst->ccp->status.valid && inst->fctrl_array[0] == FCNTL_IEEE_BLINK_CCP_64){
        STATS_INC(inst->tdma->stat, rx_complete);
        DIAGMSG("{\"utime\": %lu,\"msg\": \"tdma:rx_complete_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
        if (inst->tdma != NULL && inst->tdma->status.initialized){
            tdma->os_epoch = inst->ccp->os_epoch;//os_cputime_get32();
#ifdef TDMA_TASKS_ENABLE
            os_eventq_put(&inst->tdma->eventq, &inst->tdma->event_cb.c_ev);
#else
            os_eventq_put(&inst->eventq, &inst->tdma->event_cb.c_ev);
#endif
        }   
        return false; // TDMA is an observer and should not return true
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
tx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){

   tdma_instance_t * tdma = inst->tdma;
    
    if (inst->fctrl_array[0] == FCNTL_IEEE_BLINK_CCP_64 && inst->ccp->config.role == CCP_ROLE_MASTER){
        STATS_INC(inst->tdma->stat, tx_complete);
        DIAGMSG("{\"utime\": %lu,\"msg\": \"tdma:tx_complete_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
        if (inst->tdma != NULL && inst->tdma->status.initialized){
            tdma->os_epoch = os_cputime_get32();
#ifdef TDMA_TASKS_ENABLE
            os_eventq_put(&inst->tdma->eventq, &inst->tdma->event_cb.c_ev);
#else
            os_eventq_put(&inst->eventq, &inst->tdma->event_cb.c_ev);
#endif
        }   
        return false;   // TDMA is an observer and should not return true
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
    if (inst->slot[idx]) {
        free(inst->slot[idx]);
        inst->slot[idx] =  NULL;
    }
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
    
    STATS_INC(tdma->stat, superframe_cnt);

    for (uint16_t i = 0; i < tdma->nslots; i++) {
        if (tdma->slot[i]){
            os_cputime_timer_stop(&tdma->slot[i]->timer);
        }
    }
    for (uint16_t i = 0; i < tdma->nslots; i++) {
        if (tdma->slot[i]){
            hal_timer_start_at(&tdma->slot[i]->timer, tdma->os_epoch
                + os_cputime_usecs_to_ticks(
                    (uint32_t) (i * dw1000_dwt_usecs_to_usecs(tdma->period/tdma->nslots))
                    - (uint32_t)ceilf(dw1000_phy_SHR_duration(&tdma->parent->attrib)) 
                    - MYNEWT_VAL(OS_LATENCY))
            );
        }
    }
#if MYNEWT_VAL(TDMA_SANITY_INTERVAL) > 0 
    struct os_task * t = os_sched_get_current_task();
    os_sanity_task_checkin(t);
#endif
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

    DIAGMSG("{\"utime\": %lu,\"msg\": \"slot_timer_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

    STATS_INC(tdma->stat, slot_timer_cnt);

#ifdef TDMA_TASKS_ENABLE
    os_eventq_put(&tdma->eventq, &slot->event_cb.c_ev);
#else
    os_eventq_put(&tdma->parent->eventq, &slot->event_cb.c_ev);
#endif
}

/**
 * API to stop tdma operation. Releases each slot and stops all cputimer callbacks
 *
 * @param inst       Pointer to _tdma_instance_t.
 *
 * @return void
 */
void 
tdma_stop(struct _tdma_instance_t * tdma)
{
    for (uint16_t i = 0; i < tdma->nslots; i++) {
        if (tdma->slot[i]){
            os_cputime_timer_stop(&tdma->slot[i]->timer);
            tdma_release_slot(tdma, i);
        }
    }
}

