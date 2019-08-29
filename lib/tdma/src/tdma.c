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

#include <ccp/ccp.h>

#if MYNEWT_VAL(WCS_ENABLED)
#include <wcs/wcs.h>
#endif

#include <stats/stats.h>

#if MYNEWT_VAL(TDMA_STATS)
STATS_NAME_START(tdma_stat_section)
    STATS_NAME(tdma_stat_section, slot_timer_cnt)
    STATS_NAME(tdma_stat_section, superframe_cnt)
    STATS_NAME(tdma_stat_section, rx_complete)
    STATS_NAME(tdma_stat_section, tx_complete)
STATS_NAME_END(tdma_stat_section)

#define TDMA_STATS_INC(__X) STATS_INC(tdma->stat, __X)
#else
#define TDMA_STATS_INC(__X) {}
#endif

//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

static void tdma_superframe_event_cb(struct dpl_event * ev);
static void slot_timer_cb(void * arg);
static bool rx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);
static bool tx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);

#ifdef TDMA_TASKS_ENABLE
static void tdma_tasks_init(struct _tdma_instance_t * inst);
static void tdma_task(void *arg);
#endif

/**
 * @fn tdma_init(struct _dw1000_dev_instance_t * inst, uint32_t period, uint16_t nslots)
 * @brief API to initialise the tdma instance. Sets the clkcal postprocess and
 * assings the slot callback function for slot0.
 *
 * @param inst     Pointer to  _dw1000_dev_instance_t. 
 * @param nslots   Total slots to be allocated between two frames.
 *
 * @return tdma_instance_t*
 */
tdma_instance_t * 
tdma_init(struct _dw1000_dev_instance_t * inst, uint16_t nslots)
{
    assert(inst);
    tdma_instance_t * tdma = (tdma_instance_t*)dw1000_mac_find_cb_inst_ptr(inst, DW1000_TDMA);

    if (tdma == NULL) {
        tdma = (tdma_instance_t *) malloc(sizeof(struct _tdma_instance_t) + nslots * sizeof(struct _tdma_slot_t *));
        assert(tdma);
        memset(tdma, 0, sizeof(struct _tdma_instance_t) + nslots * sizeof(struct _tdma_slot_t * ));
        tdma->status.selfmalloc = 1;
        dpl_error_t err = dpl_mutex_init(&tdma->mutex);
        assert(err == DPL_OK);
        tdma->nslots = nslots; 
        tdma->dev_inst = inst;
#ifdef TDMA_TASKS_ENABLE
        tdma->task_prio = inst->task_prio + 0x6;
#endif
    }

    tdma->cbs = (dw1000_mac_interface_t){
        .id = DW1000_TDMA,
        .inst_ptr = (void*)tdma,
        .tx_complete_cb = tx_complete_cb,
        .rx_complete_cb = rx_complete_cb
    };
    dw1000_mac_append_interface(inst, &tdma->cbs);

    tdma->ccp = (dw1000_ccp_instance_t*)dw1000_mac_find_cb_inst_ptr(inst, DW1000_CCP);
    assert(tdma->ccp);
    
#if MYNEWT_VAL(TDMA_STATS)
    int rc = stats_init(
                STATS_HDR(tdma->stat),
                STATS_SIZE_INIT_PARMS(tdma->stat, STATS_SIZE_32),
                STATS_NAME_INIT_PARMS(tdma_stat_section)
            );
    assert(rc == 0);

#if  MYNEWT_VAL(DW1000_DEVICE_0) && !MYNEWT_VAL(DW1000_DEVICE_1)
    rc = stats_register("tdma", STATS_HDR(tdma->stat));
#elif  MYNEWT_VAL(DW1000_DEVICE_0) && MYNEWT_VAL(DW1000_DEVICE_1)
    if (inst->idx == 0)
        rc |= stats_register("tdma0", STATS_HDR(tdma->stat));
    else
        rc |= stats_register("tdma1", STATS_HDR(tdma->stat));
#endif
    assert(rc == 0);
#endif

    dpl_event_init(&tdma->superframe_event, tdma_superframe_event_cb, (void *) tdma);
    tdma->status.initialized = true;

    tdma->os_epoch = os_cputime_get32();

#ifdef TDMA_TASKS_ENABLE
    tdma_tasks_init(tdma);
#endif
    return tdma;
}

/**
 * @fn tdma_free(tdma_instance_t * inst)
 * @brief API to free memory allocated for tdma slots.
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
 * @fn tdma_pkg_init(void)
 * @ brief API to initialise the package, only one ccp service required in the system.
 *
 * @return void
 */

void tdma_pkg_init(void){
#if MYNEWT_VAL(DW1000_PKG_INIT_LOG)
    printf("{\"utime\": %lu,\"msg\": \"tdma_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
#endif

#if MYNEWT_VAL(DW1000_DEVICE_0) 
        tdma_init(hal_dw1000_inst(0), MYNEWT_VAL(TDMA_NSLOTS));
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
        tdma_init(hal_dw1000_inst(1), MYNEWT_VAL(TDMA_NSLOTS));
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
        tdma_init(hal_dw1000_inst(2), MYNEWT_VAL(TDMA_NSLOTS));
#endif

}

#ifdef TDMA_TASKS_ENABLE

/**
 * @fn sanity_feeding_cb(struct os_event * ev)
 * @brief API to feed the sanity watchdog
 *
 * @return void
 */
#if MYNEWT_VAL(TDMA_SANITY_INTERVAL) > 0
static void
sanity_feeding_cb(struct dpl_event * ev)
{
    assert(ev != NULL);
    tdma_instance_t * tdma = (tdma_instance_t * )dpl_event_get_arg(ev);
    assert(tdma);

    os_sanity_task_checkin(0);
    dpl_callout_reset(&tdma->sanity_cb, OS_TICKS_PER_SEC);
}
#endif

/**
 * @fn tdma_tasks_init(struct _tdma_instance_t * inst)
 * @brief API to initialise a higher priority task for the tdma slot tasks.
 *
 * @param inst  Pointer to  _tdma_instance_t.
 *
 * @return void
 */
static void
tdma_tasks_init(struct _tdma_instance_t * inst)
{
    /* Check if the tasks are already initiated */
    if (!dpl_eventq_inited(&inst->eventq))
    {
        /* Use a dedicate event queue for tdma events */
        dpl_eventq_init(&inst->eventq);
        dpl_task_init(&inst->task_str, "dw1000_tdma",
                     tdma_task,
                     (void *) inst,
                     inst->task_prio,
#if MYNEWT_VAL(TDMA_SANITY_INTERVAL) > 0
                     OS_TICKS_PER_SEC * MYNEWT_VAL(TDMA_SANITY_INTERVAL),
#else
                     DPL_WAIT_FOREVER,
#endif
                     inst->task_stack,
                     DW1000_DEV_TASK_STACK_SZ);
    }
#if MYNEWT_VAL(TDMA_SANITY_INTERVAL) > 0
    dpl_callout_init(&inst->sanity_cb, &inst->eventq, sanity_feeding_cb, (void *) inst);
    dpl_callout_reset(&inst->sanity_cb, OS_TICKS_PER_SEC);
#endif
}

/**
 * @fn tdma_task(void *arg)
 * @brief API for task function of tdma.
 *
 * @param arg   Pointer to an argument of void type.
 *
 * @return void
 */
static void
tdma_task(void *arg){
    tdma_instance_t * inst = arg;
    while (1) {
        dpl_eventq_run(&inst->eventq);
    }
}
#endif

/**
 * @fn rx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
 * @brief Interrupt context tdma_rx_complete callback. Used to define eopch for tdma actavities
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @param cbs   Pointer to dw1000_mac_interface_t.
 *
 * @return bool based on the totality of the handling which is false this implementation.
 */
static bool
rx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    tdma_instance_t * tdma = (tdma_instance_t*)cbs->inst_ptr;
    dw1000_ccp_instance_t *ccp = tdma->ccp;

    if (ccp->status.valid && inst->fctrl_array[0] == FCNTL_IEEE_BLINK_CCP_64){
        TDMA_STATS_INC(rx_complete);
        DIAGMSG("{\"utime\": %lu,\"msg\": \"tdma:rx_complete_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
        if (tdma != NULL && tdma->status.initialized){
            tdma->os_epoch = ccp->os_epoch;
#ifdef TDMA_TASKS_ENABLE
            dpl_eventq_put(&tdma->eventq, &tdma->superframe_event);
#else
            dpl_eventq_put(&inst->eventq, &tdma->superframe_event);
#endif
        }
        return false; // TDMA is an observer and should not return true
    }
    return false;
}

/**
 * @fn tx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
 * @brief Interrupt context tdma_tx_complete callback. Used to define eopch for tdma actavities
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @param cbs   Pointer to dw1000_mac_interface_t.
 *
 * @return bool based on the totality of the handling which is false this implementation.
 */
static bool
tx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    tdma_instance_t * tdma = (tdma_instance_t*)cbs->inst_ptr;
    dw1000_ccp_instance_t *ccp = tdma->ccp;

    if (inst->fctrl_array[0] == FCNTL_IEEE_BLINK_CCP_64 && ccp->config.role == CCP_ROLE_MASTER){
        TDMA_STATS_INC(tx_complete);
        DIAGMSG("{\"utime\": %lu,\"msg\": \"tdma:tx_complete_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
        if (tdma != NULL && tdma->status.initialized){
            tdma->os_epoch = ccp->os_epoch;
#ifdef TDMA_TASKS_ENABLE
            dpl_eventq_put(&tdma->eventq, &tdma->superframe_event);
#else
            dpl_eventq_put(&inst->eventq, &tdma->superframe_event);
#endif
        }
        return false;   // TDMA is an observer and should not return true
    }
    return false;
}

/**
 * @fn tdma_assign_slot(struct _tdma_instance_t * inst, void (* call_back )(struct os_event *), uint16_t idx, void * arg)
 * @brief API to intialise slot instance for the slot.Also initialise a timer and assigns callback for each slot.
 *
 * @param inst       Pointer to _tdma_instance_t.
 * @param call_back  Callback for the particular slot.
 * @param idx        Slot number.
 * @param arg        Argument to the callback.
 *
 * @return void
 */
void
tdma_assign_slot(struct _tdma_instance_t * inst, void (* call_back )(struct dpl_event *), uint16_t idx, void * arg)
{
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

    dpl_event_init(&inst->slot[idx]->event, call_back, (void *) inst->slot[idx]);
    os_cputime_timer_init(&inst->slot[idx]->timer, slot_timer_cb, (void *) inst->slot[idx]);

#ifdef TDMA_TASKS_ENABLE
    dpl_callout_init(&inst->slot[idx]->event_cb, &inst->eventq, callout, (void *) inst->slot[idx]);
#else
    dpl_callout_init(&inst->slot[idx]->event_cb, &inst->dev_inst->eventq, callout, (void *) inst->slot[idx]);
#endif
}

/**
 * @fn tdma_release_slot(struct _tdma_instance_t * inst, uint16_t idx)
 * @brief API to free the slot instance.
 *
 * @param inst   Pointer to _tdma_instance_t.
 * @param idx    Slot number.
 *
 * @return void
 */
void
tdma_release_slot(struct _tdma_instance_t * inst, uint16_t idx)
{
    assert(idx < inst->nslots);
    if (inst->slot[idx]) {
        os_cputime_timer_stop(&inst->slot[idx]->timer);
        free(inst->slot[idx]);
        inst->slot[idx] =  NULL;
    }
}

/**
 * @fn tdma_superframe_event_cb(struct os_event * ev)
 * @brief This event is generated by ccp/clkcal complete event. This event defines the start of an superframe epoch.
 * The event also schedules a tdma_superframe_timer_cb which turns on the receiver in advance of the next superframe epoch.
 *
 * @param ev   Pointer to os_event.
 *
 * @return void
 */
static void
tdma_superframe_event_cb(struct dpl_event * ev){

    assert(ev != NULL);
    assert(dpl_event_get_arg(ev) != NULL);

    DIAGMSG("{\"utime\": %lu,\"msg\": \"tdma_superframe_event_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
    tdma_instance_t * tdma = (tdma_instance_t *) dpl_event_get_arg(ev);
    struct _dw1000_dev_instance_t * inst = tdma->dev_inst;
    dw1000_ccp_instance_t * ccp = tdma->ccp;
    
    TDMA_STATS_INC(superframe_cnt);

    for (uint16_t i = 0; i < tdma->nslots; i++) {
        if (tdma->slot[i]){
            os_cputime_timer_stop(&tdma->slot[i]->timer);
        }
    }
    for (uint16_t i = 0; i < tdma->nslots; i++) {
        if (tdma->slot[i]){
            hal_timer_start_at(&tdma->slot[i]->timer, tdma->os_epoch
                + os_cputime_usecs_to_ticks(
                    (uint32_t) (i * dw1000_dwt_usecs_to_usecs(ccp->period/tdma->nslots))
                    - (uint32_t)ceilf(dw1000_phy_SHR_duration(&inst->attrib)) 
                    - MYNEWT_VAL(OS_LATENCY))
            );
        }
    }
}

/**
 * @fn slot_timer_cb(void * arg)
 * @brief API for each slot. This function then puts
 * the callback provided by the user in the tdma event queue.
 *
 * @param arg    A void type argument.
 *
 * @return void
 */
static void
slot_timer_cb(void * arg)
{
    assert(arg);

    tdma_slot_t * slot = (tdma_slot_t *) arg;
    /* No point in continuing if this slot is NULL */
    if (slot == NULL) {
        return;
    }
    tdma_instance_t * tdma = slot->parent;

    DIAGMSG("{\"utime\": %lu,\"msg\": \"slot_timer_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

    TDMA_STATS_INC(slot_timer_cnt);

#ifdef TDMA_TASKS_ENABLE
    dpl_eventq_put(&tdma->eventq, &slot->event);
#else
    dpl_eventq_put(&tdma->dev_inst->eventq, &slot->event);
#endif
}

/**
 * @fn tdma_stop(struct _tdma_instance_t * tdma)
 * @brief API to stop tdma operation. Releases each slot and stops all cputimer callbacks
 *
 * @param tdma      Pointer to _tdma_instance_t.
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


/**
 * Function for calculating the start of the slot for a tx operation
 *
 * @param inst       Pointer to struct _dw1000_dev_instance_t
 * @param idx        Slot index
 *
 * @return dx_time   The time for a tx operation to start
 */
uint64_t
tdma_tx_slot_start(struct _tdma_instance_t * tdma, float idx)
{
    dw1000_ccp_instance_t * ccp = tdma->ccp;

#if MYNEWT_VAL(WCS_ENABLED)
    wcs_instance_t * wcs = ccp->wcs;
    uint64_t dx_time = (ccp->local_epoch + (uint64_t) wcs_dtu_time_adjust(wcs, ((idx * ((uint64_t)ccp->period << 16))/tdma->nslots)));
    // uint64_t dx_time = (ccp->local_epoch + (uint64_t) roundf((1.0l + wcs->skew) * (double)((idx * (uint64_t)ccp->period * 65536)/tdma->nslots)));
#else
    uint64_t dx_time = (ccp->local_epoch + (uint64_t) ((idx * ((uint64_t)ccp->period << 16)/tdma->nslots)));
#endif
    return dx_time;
}

/**
 * Function for calculating the start of the slot for a rx operation.
 * taking into account that the preamble needs to be sent before the
 * RMARKER, which marks the time of the frame, is sent.
 *
 * @param inst       Pointer to struct _dw1000_dev_instance_t
 * @param idx        Slot index
 *
 * @return dx_time   The time for a rx operation to start
 */
uint64_t
tdma_rx_slot_start(struct _tdma_instance_t * tdma, float idx)
{
    uint64_t dx_time = tdma_tx_slot_start(tdma, idx);
    uint64_t rx_stable =  MYNEWT_VAL(TIME_TO_RX_STABLE);
    dx_time = (dx_time - ((uint64_t)ceilf(dw1000_usecs_to_dwt_usecs(dw1000_phy_SHR_duration(&tdma->dev_inst->attrib) + rx_stable)) << 16));
    return dx_time;
}
