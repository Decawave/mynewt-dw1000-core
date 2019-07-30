/*
 * Copyright 2018, Decawave Limited, All Rights Reserved
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
 * @file dw1000_pan.c
 * @autjor paul kettle
 * @date 2018
 * @brief Personal Area Network
 *
 * @details This is the pan base class which utilises the functions to allocate/deallocate the resources on pan_master,sets callbacks, enables  * blink_requests.
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <os/os.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include "bsp/bsp.h"
#include <imgmgr/imgmgr.h>

#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_ftypes.h>

#if MYNEWT_VAL(CCP_ENABLED)
#include <ccp/ccp.h>
#endif
#if MYNEWT_VAL(TDMA_ENABLED)
#include <tdma/tdma.h>
#endif

// #define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

#if MYNEWT_VAL(PAN_ENABLED)
#include <pan/pan.h>

//! Buffers for pan frames
#if MYNEWT_VAL(DW1000_DEVICE_0)
static pan_frame_t g_pan_0[] = {
    [0] = {
        .fctrl = FCNTL_IEEE_BLINK_TAG_64,    // frame control (FCNTL_IEEE_BLINK_64 to indicate a data frame using 16-bit addressing).
        .seq_num = 0x0,
    },
    [1] = {
        .fctrl = FCNTL_IEEE_BLINK_TAG_64,    // frame control (FCNTL_IEEE_BLINK_64 to indicate a data frame using 16-bit addressing).
    }
};
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
static pan_frame_t g_pan_1[] = {
    [0] = {
        .fctrl = FCNTL_IEEE_BLINK_TAG_64,    // frame control (FCNTL_IEEE_BLINK_64 to indicate a data frame using 16-bit addressing).
        .seq_num = 0x0,
    },
    [1] = {
        .fctrl = FCNTL_IEEE_BLINK_TAG_64,    // frame control (FCNTL_IEEE_BLINK_64 to indicate a data frame using 16-bit addressing).
    }
};
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
static pan_frame_t g_pan_2[] = {
    [0] = {
        .fctrl = FCNTL_IEEE_BLINK_TAG_64,    // frame control (FCNTL_IEEE_BLINK_64 to indicate a data frame using 16-bit addressing).
        .seq_num = 0x0,
    },
    [1] = {
        .fctrl = FCNTL_IEEE_BLINK_TAG_64,    // frame control (FCNTL_IEEE_BLINK_64 to indicate a data frame using 16-bit addressing).
    }
};
#endif

#include <stats/stats.h>
STATS_SECT_START(pan_stat_section)
    STATS_SECT_ENTRY(pan_request)
    STATS_SECT_ENTRY(pan_listen)
    STATS_SECT_ENTRY(pan_reset)
    STATS_SECT_ENTRY(relay_tx)
    STATS_SECT_ENTRY(lease_expiry)
    STATS_SECT_ENTRY(tx_complete)
    STATS_SECT_ENTRY(rx_complete)
    STATS_SECT_ENTRY(rx_unsolicited)
    STATS_SECT_ENTRY(rx_error)
    STATS_SECT_ENTRY(tx_error)
    STATS_SECT_ENTRY(rx_timeout)
    STATS_SECT_ENTRY(reset)
STATS_SECT_END

STATS_NAME_START(pan_stat_section)
    STATS_NAME(pan_stat_section, pan_request)
    STATS_NAME(pan_stat_section, pan_listen)
    STATS_NAME(pan_stat_section, pan_reset)
    STATS_NAME(pan_stat_section, relay_tx)
    STATS_NAME(pan_stat_section, lease_expiry)
    STATS_NAME(pan_stat_section, tx_complete)
    STATS_NAME(pan_stat_section, rx_complete)
    STATS_NAME(pan_stat_section, rx_unsolicited)
    STATS_NAME(pan_stat_section, rx_error)
    STATS_NAME(pan_stat_section, tx_error)
    STATS_NAME(pan_stat_section, rx_timeout)
    STATS_NAME(pan_stat_section, reset)
STATS_NAME_END(pan_stat_section)

static STATS_SECT_DECL(pan_stat_section) g_stat; //!< Stats instance

static dw1000_pan_config_t g_config = {
    .tx_holdoff_delay = MYNEWT_VAL(PAN_TX_HOLDOFF),         // Send Time delay in usec.
    .rx_timeout_period = MYNEWT_VAL(PAN_RX_TIMEOUT),        // Receive response timeout in usec.
    .lease_time = MYNEWT_VAL(PAN_LEASE_TIME),               // Lease time in seconds
    .network_role = MYNEWT_VAL(PAN_NETWORK_ROLE)            // Role in the network (Anchor/Tag/...)
};

static bool rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static void pan_postprocess(struct os_event * ev);
static void lease_expiry_cb(struct os_event * ev);

static dw1000_mac_interface_t g_cbs[] = {
        [0] = {
            .id = DW1000_PAN,
            .rx_complete_cb = rx_complete_cb,
            .tx_complete_cb = tx_complete_cb,
            .rx_timeout_cb = rx_timeout_cb,
            .reset_cb = reset_cb
        },
#if MYNEWT_VAL(DW1000_DEVICE_1)
        [1] = {
            .id = DW1000_RNG,
            .rx_complete_cb = rx_complete_cb,
            .tx_complete_cb = tx_complete_cb,
            .rx_timeout_cb = rx_timeout_cb,
            .reset_cb = reset_cb
        },
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
        [2] = {
            .id = DW1000_RNG,
            .rx_complete_cb = rx_complete_cb,
            .tx_complete_cb = tx_complete_cb,
            .rx_timeout_cb = rx_timeout_cb,
            .reset_cb = reset_cb
        }
#endif
};

/**
 * @fn dw1000_pan_init(dw1000_dev_instance_t * inst,  dw1000_pan_config_t * config, uint16_t nframes)
 * @brief API to initialise pan parameters.
 *
 * @param inst     Pointer to dw1000_dev_instance_t.
 * @param config   Pointer to dw1000_pan_config_t.
 * @param nrfames  number of frames defined to store pan frames.
 *
 * @return dw1000_pan_instance_t
 */
dw1000_pan_instance_t *
dw1000_pan_init(dw1000_dev_instance_t * inst,  dw1000_pan_config_t * config, uint16_t nframes)
{
    assert(inst);

    dw1000_pan_instance_t *pan = (dw1000_pan_instance_t*)dw1000_mac_find_cb_inst_ptr(inst, DW1000_PAN);
    if (pan == NULL ) {
        pan = (dw1000_pan_instance_t *) malloc(sizeof(dw1000_pan_instance_t) + nframes * sizeof(pan_frame_t *));
        assert(pan);
        memset(pan, 0, sizeof(dw1000_pan_instance_t));
        pan->status.selfmalloc = 1;
        pan->nframes = nframes;
    }

    pan->dev_inst = inst;
    pan->config = config;
    pan->control = (dw1000_pan_control_t){
        .postprocess = false,
    };

    os_error_t err = os_sem_init(&pan->sem, 0x1);
    assert(err == OS_OK);

    dw1000_pan_set_postprocess(pan, pan_postprocess);

    err = stats_init(
        STATS_HDR(g_stat),
        STATS_SIZE_INIT_PARMS(g_stat, STATS_SIZE_32),
        STATS_NAME_INIT_PARMS(pan_stat_section)
        );
    err |= stats_register("pan", STATS_HDR(g_stat));
    assert(err == OS_OK);

    pan->status.valid = true;
    pan->status.initialized = 1;
    return pan;
}

/**
 * @fn dw1000_pan_set_frames(dw1000_dev_instance_t * inst, pan_frame_t pan[], uint16_t nframes)
 * @brief API to set the pointer to the frame buffers.
 *
 * @param inst      Pointer to dw1000_dev_instance_t.
 * @param twr[]     Pointer to frame buffers.
 * @param nframes   Number of buffers defined to store the discovery data.
 *
 * @return void
 */
void
dw1000_pan_set_frames(dw1000_pan_instance_t *pan, pan_frame_t pan_f[], uint16_t nframes)
{
    assert(nframes <= pan->nframes);
    for (uint16_t i = 0; i < nframes; i++)
        pan->frames[i] = &pan_f[i];
}

/**
 * @fn pan_pkg_init(void)
 * @brief API to initialise the pan package.
 *
 * @return void
 */
void
pan_pkg_init(void)
{
    dw1000_pan_instance_t *pan;
    printf("{\"utime\": %lu,\"msg\": \"pan_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

#if MYNEWT_VAL(DW1000_DEVICE_0)
    g_cbs[0].inst_ptr = pan = dw1000_pan_init(hal_dw1000_inst(0), &g_config, sizeof(g_pan_0)/sizeof(pan_frame_t));
    dw1000_pan_set_frames(pan, g_pan_0, sizeof(g_pan_0)/sizeof(pan_frame_t));
    dw1000_mac_append_interface(hal_dw1000_inst(0), &g_cbs[0]);
    os_callout_init(&pan->pan_lease_callout_expiry, os_eventq_dflt_get(), lease_expiry_cb, (void *) pan);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    g_cbs[1].inst_ptr = pan = dw1000_pan_init(hal_dw1000_inst(1), &g_config, sizeof(g_pan_1)/sizeof(pan_frame_t));
    dw1000_pan_set_frames(pan, g_pan_1, sizeof(g_pan_1)/sizeof(pan_frame_t));
    dw1000_mac_append_interface(hal_dw1000_inst(1), &g_cbs[1]);
    os_callout_init(&pan->pan_lease_callout_expiry, os_eventq_dflt_get(), lease_expiry_cb, (void *) pan);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    g_cbs[2].inst_ptr = pan = dw1000_pan_init(hal_dw1000_inst(2), &g_config, sizeof(g_pan_2)/sizeof(pan_frame_t));
    dw1000_pan_set_frames(pan, g_pan_2, sizeof(g_pan_2)/sizeof(pan_frame_t));
    dw1000_mac_append_interface(hal_dw1000_inst(2), &g_cbs[2]);
    os_callout_init(&pan->pan_lease_callout_expiry, os_eventq_dflt_get(), lease_expiry_cb, (void *) pan);
#endif
}

/**
 * @fn dw1000_pan_free(dw1000_dev_instance_t * inst)
 * @brief API to free pan resources.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return void
 */
void
dw1000_pan_free(dw1000_pan_instance_t *pan){
    assert(pan);
    if (pan->status.selfmalloc) {
        free(pan);
    } else {
        pan->status.initialized = 0;
    }
}

/**
 * @fn dw1000_pan_set_postprocess(dw1000_dev_instance_t * inst, os_event_fn * pan_postprocess)
 * @brief API to set pan_postprocess.
 *
 * @param inst              Pointer to dw1000_dev_instance_t.
 * @param pan_postprocess   Pointer to os_event_fn.
 *
 * @return void
 */
void
dw1000_pan_set_postprocess(dw1000_pan_instance_t *pan, os_event_fn * cb)
{
    pan->postprocess_event.ev_cb  = cb;
    pan->postprocess_event.ev_arg = (void *) pan;
    pan->control.postprocess = true;
}

/**
 * @fn pan_postprocess(struct os_event * ev)
 * @brief This a template which should be replaced by the pan_master by a event that tracks UUIDs
 * and allocated PANIDs and SLOTIDs.
 *
 * @param ev  Pointer to os_events.
 *
 * @return void
 */
static void
pan_postprocess(struct os_event * ev){
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

#if MYNEWT_VAL(PAN_VERBOSE)
    dw1000_pan_instance_t * pan = (dw1000_pan_instance_t *)ev->ev_arg;
    dw1000_dev_instance_t * inst = pan->dev_inst;
    pan_frame_t * frame = pan->frames[(pan->idx)%pan->nframes];
    if(pan->status.valid && frame->long_address == inst->my_long_address)
        printf("{\"utime\": %lu,\"UUID\": \"%llX\",\"ID\": \"%X\",\"PANID\": \"%X\",\"slot\": %d}\n",
            os_cputime_ticks_to_usecs(os_cputime_get32()),
            frame->long_address,
            frame->short_address,
            frame->pan_id,
            frame->slot_id
        );
    else if (frame->code == DWT_PAN_REQ)
        printf("{\"utime\": %lu,\"UUID\": \"%llX\",\"seq_num\": %d}\n",
            os_cputime_ticks_to_usecs(os_cputime_get32()),
            frame->long_address,
            frame->seq_num
        );
    else if (frame->code == DWT_PAN_RESP)
        printf("{\"utime\": %lu,\"UUID\": \"%llX\",\"ID\": \"%X\",\"PANID\": \"%X\",\"slot\": %d}\n",
            os_cputime_ticks_to_usecs(os_cputime_get32()),
            frame->long_address,
            frame->short_address,
            frame->pan_id,
            frame->slot_id
        );
#endif
}

/**
 * @fn lease_expiry_cb(struct os_event * ev)
 * @brief Function called when our lease is about to expire
 *
 * @param ev  Pointer to os_events.
 *
 * @return void
 */
static void
lease_expiry_cb(struct os_event * ev)
{
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);
    dw1000_pan_instance_t * pan = (dw1000_pan_instance_t *)ev->ev_arg;
    dw1000_dev_instance_t * inst = pan->dev_inst;
    STATS_INC(g_stat, lease_expiry);
    pan->status.valid = false;
    pan->status.lease_expired = true;
    inst->slot_id = 0xffff;

    DIAGMSG("{\"utime\": %lu,\"msg\": \"pan_lease_expired\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
    if (pan->control.postprocess) {
        os_eventq_put(&inst->eventq, &pan->postprocess_event);
    }
}

/**
 * @fn rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
 * @brief This is an internal static function that executes on both the pan_master Node and the TAG/ANCHOR
 * that initiated the blink. On the pan_master the postprocess function should allocate a PANID and a SLOTID,
 * while on the TAG/ANCHOR the returned allocations are assigned and the PAN discover event is stopped. The pan
 * discovery resources can be released.
 *
 * @param inst    Pointer to dw1000_dev_instance_t.
 * @param cbs     Pointer to dw1000_mac_interface_t.
 *
 * @return bool
 */
static bool
rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    dw1000_pan_instance_t * pan = (dw1000_pan_instance_t *)cbs->inst_ptr;
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64) {
        if (pan->status.valid == false && pan->config->role == PAN_ROLE_SLAVE) {
            /* Grab all packets if we're not provisioned as slave */
            return true;
        }
        return false;
    }

    if (os_sem_get_count(&pan->sem) == 1){
        /* Unsolicited */
        STATS_INC(g_stat, rx_unsolicited);
        return false;
    }

    STATS_INC(g_stat, rx_complete);
    pan_frame_t * frame = pan->frames[(pan->idx)%pan->nframes];

    /* Ignore frames that are too long */
    if (inst->frame_len > sizeof(struct _pan_frame_t)) {
        return false;
    }
    memcpy(frame->array, inst->rxbuf, inst->frame_len);

    if (pan->config->role == PAN_ROLE_RELAY &&
        frame->rpt_count < frame->rpt_max &&
        frame->long_address != inst->my_long_address) {
        frame->rpt_count++;
        dw1000_set_wait4resp(inst, true);
        dw1000_write_tx_fctrl(inst, inst->frame_len, 0);
        pan->status.start_tx_error = dw1000_start_tx(inst).start_tx_error;
        dw1000_write_tx(inst, frame->array, 0, inst->frame_len);
        STATS_INC(g_stat, relay_tx);
    }

    switch(frame->code) {
    case DWT_PAN_REQ:
        STATS_INC(g_stat, pan_request);
        if (pan->config->role == PAN_ROLE_MASTER) {
            /* Prevent another request coming in whilst processing this one */
            dw1000_stop_rx(inst);
        } else {
            return true;
        }
        break;
    case DWT_PAN_RESP:
        if(frame->long_address == inst->my_long_address){
            /* TAG/ANCHOR side */
            inst->my_short_address = frame->short_address;
            inst->PANID = frame->pan_id;
            inst->slot_id = frame->slot_id;
            pan->status.valid = true;
            pan->status.lease_expired = false;
            os_callout_stop(&pan->pan_lease_callout_expiry);
            if (frame->lease_time > 0) {
                /* Calculate when our lease expires */
                uint32_t exp_tics;
                uint32_t lease_us = 1000000;
                lease_us = (uint32_t)(frame->lease_time)*1000000;
#if MYNEWT_VAL(CCP_ENABLED)
                dw1000_ccp_instance_t *ccp = (dw1000_ccp_instance_t*)dw1000_mac_find_cb_inst_ptr(inst, DW1000_CCP);
                lease_us -= (inst->rxtimestamp>>16) - (ccp->local_epoch>>16);
#endif
                os_time_ms_to_ticks(lease_us/1000, &exp_tics);
                os_callout_reset(&pan->pan_lease_callout_expiry, exp_tics);
            }
        } else {
            return true;
        }
        break;
    case DWT_PAN_RESET:
        STATS_INC(g_stat, pan_reset);
        if (pan->config->role != PAN_ROLE_MASTER) {
            pan->status.valid = false;
            pan->status.lease_expired = true;
            inst->slot_id = 0xffff;
            os_callout_stop(&pan->pan_lease_callout_expiry);
        } else {
            return false;
        }
    default:
        return false;
        break;
    }

    /* Postprocess, all roles */
    if (pan->control.postprocess) {
        os_eventq_put(&inst->eventq, &pan->postprocess_event);
    }

    /* Release sem */
    if (os_sem_get_count(&pan->sem) == 0) {
        os_error_t err = os_sem_release(&pan->sem);
        assert(err == OS_OK);
    }
    return true;
}

/**
 * @fn tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
 * @brief API for transmit complete callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @param cbs   Pointer to dw1000_mac_interface_t.
 *
 * @return bool
 */
static bool
tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    dw1000_pan_instance_t * pan = (dw1000_pan_instance_t *)cbs->inst_ptr;
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        return false;
    }
    pan->idx++;
    STATS_INC(g_stat, tx_complete);
    return true;
}

/**
 * @fn reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
 * @brief API for reset callback.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @param cbs    Pointer to dw1000_mac_interface_t.
 *
 * @return bool
 */
static bool
reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    dw1000_pan_instance_t * pan = (dw1000_pan_instance_t *)cbs->inst_ptr;
    if (os_sem_get_count(&pan->sem) == 0){
        STATS_INC(g_stat, reset);
        os_error_t err = os_sem_release(&pan->sem);
        assert(err == OS_OK);
        return true;
    }
    return false;
}

/**
 * @fn rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
 * @brief API for receive timeout callback.
 *
 * @param inst    Pointer to dw1000_dev_instance_t.
 * @param cbs     Pointer to dw1000_mac_interface_t.
 *
 * @return bool
 */
static bool
rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    dw1000_pan_instance_t * pan = (dw1000_pan_instance_t *)cbs->inst_ptr;
    if (os_sem_get_count(&pan->sem) == 0){
        STATS_INC(g_stat, rx_timeout);
        os_error_t err = os_sem_release(&pan->sem);
        assert(err == OS_OK);
        return true;
    }
    return false;
}

/**
 * @fn dw1000_pan_listen(dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode)
 * @brief Listen for PAN requests / resets
 *
 * @param inst          Pointer to dw1000_dev_instance_t.
 * @param mode          dw1000_dev_modes_t of DWT_BLOCKING and DWT_NONBLOCKING.
 *
 * @return dw1000_dev_status_t
 */
dw1000_dev_status_t
dw1000_pan_listen(dw1000_pan_instance_t * pan, dw1000_dev_modes_t mode)
{
    dw1000_dev_instance_t * inst = pan->dev_inst;
    os_error_t err = os_sem_pend(&pan->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    STATS_INC(g_stat, pan_listen);

    if(dw1000_start_rx(inst).start_rx_error){
        STATS_INC(g_stat, rx_error);
        err = os_sem_release(&pan->sem);
        assert(err == OS_OK);
    }

    if (mode == DWT_BLOCKING){
        err = os_sem_pend(&pan->sem, OS_TIMEOUT_NEVER);
        assert(err == OS_OK);
        err = os_sem_release(&pan->sem);
        assert(err == OS_OK);
    }

    return inst->status;
}

/**
 * @fn dw1000_pan_blink(dw1000_dev_instance_t * inst, uint16_t role, dw1000_dev_modes_t mode, uint64_t delay)
 * @brief A Personal Area Network blink request is a discovery phase in which a TAG/ANCHOR seeks to discover
 * an available PAN Master. The outcome of this process is a PANID and SLOTID assignment.
 *
 * @param inst     Pointer to dw1000_dev_instance_t.
 * @param role     Requested role in the network
 * @param mode     BLOCKING and NONBLOCKING modes of dw1000_dev_modes_t.
 * @param delay    When to send this blink
 *
 * @return dw1000_pan_status_t
 */
dw1000_pan_status_t
dw1000_pan_blink(dw1000_pan_instance_t *pan, uint16_t role,
                 dw1000_dev_modes_t mode, uint64_t delay)
{
    dw1000_dev_instance_t * inst = pan->dev_inst;
    os_error_t err = os_sem_pend(&pan->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    STATS_INC(g_stat, pan_request);
    pan_frame_t * frame = pan->frames[(pan->idx)%pan->nframes];

    frame->seq_num += pan->nframes;
    frame->long_address = inst->my_long_address;
    frame->code = DWT_PAN_REQ;
    frame->rpt_count = 0;
    frame->rpt_max = MYNEWT_VAL(PAN_RPT_MAX);
    frame->role = role;
    frame->lease_time = pan->config->lease_time;
    imgr_my_version(&frame->fw_ver);

    dw1000_set_delay_start(inst, delay);
    dw1000_write_tx_fctrl(inst, sizeof(struct _pan_frame_t), 0);
    dw1000_write_tx(inst, frame->array, 0, sizeof(struct _pan_frame_t));
    dw1000_set_wait4resp(inst, true);
    dw1000_set_rx_timeout(inst, pan->config->rx_timeout_period);
    pan->status.start_tx_error = dw1000_start_tx(inst).start_tx_error;

    if (pan->status.start_tx_error){
        STATS_INC(g_stat, tx_error);
        DIAGMSG("{\"utime\": %lu,\"msg\": \"pan_blnk_txerr\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
        // Half Period Delay Warning occured try for the next epoch
        // Use seq_num to detect this on receiver size
        os_sem_release(&pan->sem);
    }
    else if(mode == DWT_BLOCKING){
        err = os_sem_pend(&pan->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions
        os_sem_release(&pan->sem);
        assert(err == OS_OK);
    }
    return pan->status;
}

/**
 * @fn dw1000_pan_reset(dw1000_dev_instance_t * inst, uint64_t delay)
 * @brief A Pan reset message is a broadcast to all nodes having a pan assigned address
 * instructing them to reset and renew their address. Normally issued by a restarted
 * master.
 *
 * @param inst     Pointer to dw1000_dev_instance_t.
 * @param delay    When to send this reset
 *
 * @return dw1000_pan_status_t
 */
dw1000_pan_status_t
dw1000_pan_reset(dw1000_pan_instance_t * pan, uint64_t delay)
{
    dw1000_dev_instance_t * inst = pan->dev_inst;
    pan_frame_t * frame = pan->frames[(pan->idx)%pan->nframes];

    frame->seq_num += pan->nframes;
    frame->long_address = inst->my_long_address;
    frame->code = DWT_PAN_RESET;

    dw1000_set_delay_start(inst, delay);
    dw1000_write_tx_fctrl(inst, sizeof(struct _pan_frame_t), 0);
    dw1000_write_tx(inst, frame->array, 0, sizeof(struct _pan_frame_t));
    dw1000_set_wait4resp(inst, false);
    pan->status.start_tx_error = dw1000_start_tx(inst).start_tx_error;

    if (pan->status.start_tx_error){
        STATS_INC(g_stat, tx_error);
        DIAGMSG("{\"utime\": %lu,\"msg\": \"pan_reset_tx_err\"}\n", os_cputime_ticks_to_usecs(os_cputime_get32()));
    }
    return pan->status;
}

/**
 * @fn dw1000_pan_start(dw1000_dev_instance_t * inst, dw1000_pan_role_t role)
 * @brief A Personal Area Network blink is a discovery phase in which a TAG/ANCHOR seeks to discover
 * an available PAN Master. The pan_master does not
 * need to call this function.
 *
 * @param inst    Pointer to dw1000_dev_instance_t.
 * @param role    dw1000_pan_role_t of PAN_ROLE_MASTER, PAN_ROLE_SLAVE ,PAN_ROLE_RELAY.
 * @param network_role network_role_t, The role in the network application (NETWORK_ROLE_ANCHOR, NETWORK_ROLE_TAG)
 *
 * @return void
 */
void
dw1000_pan_start(dw1000_pan_instance_t * pan, dw1000_pan_role_t role, network_role_t network_role)
{
    pan->config->role = role;
    pan->config->network_role = network_role;

    if (pan->config->role == PAN_ROLE_MASTER) {
        /* Nothing for now */
    } else if (pan->config->role == PAN_ROLE_SLAVE) {
        pan->idx = 0x1;
        pan->status.valid = false;

#if MYNEWT_VAL(PAN_VERBOSE)
        printf("{\"utime\": %lu,\"PAN\": \"%s\"}\n",
               os_cputime_ticks_to_usecs(os_cputime_get32()),
               "Provisioning"
            );
#endif
    }
}

/**
 * @fn dw1000_pan_lease_remaining(dw1000_dev_instance_t * inst)
 * @brief Checks time to lease expiry
 *
 * @param inst    Pointer to dw1000_dev_instance_t.
 *
 * @return uint32_t ms to expiry, 0 if already expired
 */
uint32_t
dw1000_pan_lease_remaining(dw1000_pan_instance_t * pan)
{
    os_time_t rt = os_callout_remaining_ticks(&pan->pan_lease_callout_expiry, os_time_get());
    return os_time_ticks_to_ms32(rt);
}


#if MYNEWT_VAL(TDMA_ENABLED)
/**
 * @fn dw1000_pan_slot_timer_cb
 * @brief tdma slot handler for pan slots
 *
 * @param struct os_event* event pointer with argument set to the pan instance
 *
 * @return void
 */
void 
dw1000_pan_slot_timer_cb(struct os_event * ev)
{
    assert(ev);
    tdma_slot_t * slot = (tdma_slot_t *) ev->ev_arg;
    tdma_instance_t * tdma = slot->parent;
    dw1000_ccp_instance_t *ccp = tdma->ccp;
    dw1000_dev_instance_t *inst = tdma->dev_inst;
    dw1000_pan_instance_t *pan = (dw1000_pan_instance_t*)slot->arg;
    assert(pan);
    uint16_t idx = slot->idx;

    /* Check if we are to act as a Master Node in the network */
    if (inst->role&DW1000_ROLE_PAN_MASTER) {
        static uint8_t _pan_cycles = 0;

        /* Broadcast an initial reset message to clear all leases */
        if (_pan_cycles < 8) {
            _pan_cycles++;
            dw1000_pan_reset(pan, tdma_tx_slot_start(tdma, idx));
        } else {
            uint64_t dx_time = tdma_rx_slot_start(tdma, idx);
            dw1000_set_rx_timeout(inst, 3*ccp->period/tdma->nslots/4);
            dw1000_set_delay_start(inst, dx_time);
            dw1000_set_on_error_continue(inst, true);
            dw1000_pan_listen(pan, DWT_BLOCKING);
        }
    } else {
        /* Act as a slave Node in the network */
        if (pan->status.valid && dw1000_pan_lease_remaining(pan)>MYNEWT_VAL(PAN_LEASE_EXP_MARGIN)) {
            /* Our lease is still valid - just listen */
            uint16_t timeout;
            if (pan->config->role == PAN_ROLE_RELAY) {
                timeout = 3*ccp->period/tdma->nslots/4;
            } else {
                /* Only listen long enough to get any resets from master */
                timeout = dw1000_phy_frame_duration(&inst->attrib, sizeof(sizeof(struct _pan_frame_t)))
                    + MYNEWT_VAL(XTALT_GUARD);
            }
            dw1000_set_rx_timeout(inst, timeout);
            dw1000_set_delay_start(inst, tdma_rx_slot_start(tdma, idx));
            dw1000_set_on_error_continue(inst, true);
            if (dw1000_pan_listen(pan, DWT_BLOCKING).start_rx_error) {
                STATS_INC(g_stat, rx_error);
            }
        } else {
            /* Subslot 0 is for master reset, subslot 1 is for sending requests */
            uint64_t dx_time = tdma_tx_slot_start(tdma, (float)idx+1.0f/16);
            dw1000_pan_blink(pan, pan->config->network_role, DWT_BLOCKING, dx_time);
        }
    }
}
#endif // MYNEWT_VAL(TDMA_ENABLED)


#endif // PAN_ENABLED
