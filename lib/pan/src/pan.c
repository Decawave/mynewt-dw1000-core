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
    .lease_time = MYNEWT_VAL(PAN_LEASE_TIME)                // Lease time in seconds
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
 * API to initialise pan parameters.
 *
 * @param inst     Pointer to dw1000_dev_instance_t.
 * @param config   Pointer to dw1000_pan_config_t.
 *
 * @return dw1000_pan_instance_t
 */
dw1000_pan_instance_t *
dw1000_pan_init(dw1000_dev_instance_t * inst,  dw1000_pan_config_t * config, uint16_t nframes)
{
    assert(inst);

    if (inst->pan == NULL ) {
        inst->pan = (dw1000_pan_instance_t *) malloc(sizeof(dw1000_pan_instance_t) + nframes * sizeof(pan_frame_t *));
        assert(inst->pan);
        memset(inst->pan, 0, sizeof(dw1000_pan_instance_t));
        inst->pan->status.selfmalloc = 1;
        inst->pan->nframes = nframes;
    }

    inst->pan->parent = inst;
    inst->pan->config = config;
    inst->pan->control = (dw1000_pan_control_t){
        .postprocess = false,
    };

    os_error_t err = os_sem_init(&inst->pan->sem, 0x1);
    assert(err == OS_OK);

    dw1000_pan_set_postprocess(inst, pan_postprocess);

    err = stats_init(
        STATS_HDR(g_stat),
        STATS_SIZE_INIT_PARMS(g_stat, STATS_SIZE_32),
        STATS_NAME_INIT_PARMS(pan_stat_section)
        );
    err |= stats_register("pan", STATS_HDR(g_stat));
    assert(err == OS_OK);

    inst->pan->status.valid = true;
    inst->pan->status.initialized = 1;
    return inst->pan;
}

/**
 * API to set the pointer to the frame buffers.
 *
 * @param inst      Pointer to dw1000_dev_instance_t.
 * @param twr[]     Pointer to frame buffers.
 * @param nframes   Number of buffers defined to store the ranging data.
 *
 * @return void
 */
inline void
dw1000_pan_set_frames(dw1000_dev_instance_t * inst, pan_frame_t pan[], uint16_t nframes)
{
    assert(nframes <= inst->pan->nframes);
    for (uint16_t i = 0; i < nframes; i++)
        inst->pan->frames[i] = &pan[i];
}


/**
 * API to initialise the pan package.
 *
 * @return void
 */
void pan_pkg_init(void){

    printf("{\"utime\": %lu,\"msg\": \"pan_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

#if MYNEWT_VAL(DW1000_DEVICE_0)
    dw1000_pan_init(hal_dw1000_inst(0), &g_config, sizeof(g_pan_0)/sizeof(pan_frame_t));
    dw1000_pan_set_frames(hal_dw1000_inst(0), g_pan_0, sizeof(g_pan_0)/sizeof(pan_frame_t));
    dw1000_mac_append_interface(hal_dw1000_inst(0), &g_cbs[0]);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    dw1000_pan_init(hal_dw1000_inst(1), &g_config, sizeof(g_pan_1)/sizeof(pan_frame_t));
    dw1000_pan_set_frames(hal_dw1000_inst(1), g_pan_1, sizeof(g_pan_1)/sizeof(pan_frame_t));
    dw1000_mac_append_interface(hal_dw1000_inst(1), &g_cbs[1]);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    dw1000_pan_init(hal_dw1000_inst(2), &g_config, sizeof(g_pan_2)/sizeof(pan_frame_t));
    dw1000_pan_set_frames(hal_dw1000_inst(2), g_pan_2, sizeof(g_pan_2)/sizeof(pan_frame_t));
    dw1000_mac_append_interface(hal_dw1000_inst(2), &g_cbs[2]);
#endif
}


/**
 * API to free pan resources.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return void
 */
void
dw1000_pan_free(dw1000_dev_instance_t * inst){
    assert(inst->pan);
    if (inst->pan->status.selfmalloc)
        free(inst->pan);
    else
        inst->pan->status.initialized = 0;
}

/**
 * API to set pan_postprocess.
 *
 * @param inst              Pointer to dw1000_dev_instance_t.
 * @param pan_postprocess   Pointer to os_event_fn.
 *
 * @return void
 */
void
dw1000_pan_set_postprocess(dw1000_dev_instance_t * inst, os_event_fn * pan_postprocess){
    dw1000_pan_instance_t * pan = inst->pan;
    os_callout_init(&pan->pan_callout_postprocess, os_eventq_dflt_get(),
                    pan_postprocess, (void *) inst);
    os_callout_init(&pan->pan_lease_callout_expiry, os_eventq_dflt_get(),
                    lease_expiry_cb, (void *) inst);

    pan->control.postprocess = true;
}

/**
 * This a template which should be replaced by the pan_master by a event that tracks UUIDs
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
    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_pan_instance_t * pan = inst->pan;
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
}

/**
 * Function called when our lease is about to expire
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
    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_pan_instance_t * pan = inst->pan;
    STATS_INC(g_stat, lease_expiry);
    pan->status.valid = false;
    pan->status.lease_expired = true;
    inst->slot_id = 0xffff;

    DIAGMSG("{\"utime\": %lu,\"msg\": \"pan_lease_expired\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
    if (pan->control.postprocess)
        os_eventq_put(&inst->eventq, &pan->pan_callout_postprocess.c_ev);
}


/**
 * This is an internal static function that executes on both the pan_master Node and the TAG/ANCHOR
 * that initiated the blink. On the pan_master the postprocess function should allocate a PANID and a SLOTID,
 * while on the TAG/ANCHOR the returned allocations are assigned and the PAN discover event is stopped. The pan
 * discovery resources can be released.
 *
 * @param inst    Pointer to dw1000_dev_instance_t.
 *
 * @return bool
 */
static bool
rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64) {
        if (inst->pan->status.valid == false && inst->pan->config->role == PAN_ROLE_SLAVE) {
            /* Grab all packets if we're not provisioned as slave */
            return true;
        }
        return false;
    }

    if (os_sem_get_count(&inst->pan->sem) == 1){
        /* Unsolicited */
        STATS_INC(g_stat, rx_unsolicited);
        return false;
    }

    STATS_INC(g_stat, rx_complete);
    dw1000_pan_instance_t * pan = inst->pan;
    pan_frame_t * frame = pan->frames[(pan->idx)%pan->nframes];

    /* Ignore frames that are too long */
    if (inst->frame_len > sizeof(struct _pan_frame_t)) {
        return false;
    }
    memcpy(frame->array, inst->rxbuf, inst->frame_len);

    switch(frame->code) {
    case DWT_PAN_REQ:
        STATS_INC(g_stat, pan_request);
        if (inst->pan->config->role == PAN_ROLE_MASTER) {
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
#if MYNEWT_VAL(CCP_ENABLED)
                lease_us = frame->lease_time*inst->ccp->period;
                lease_us -= (inst->rxtimestamp>>16) - (inst->ccp->epoch>>16);
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
        if (pan->config->role == PAN_ROLE_SLAVE) {
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
    if (pan->control.postprocess)
        os_eventq_put(&inst->eventq, &pan->pan_callout_postprocess.c_ev);

    /* Release sem */
    os_error_t err = os_sem_release(&pan->sem);
    assert(err == OS_OK);
    return true;
}

/**
 * API for transmit complete callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return bool
 */
static bool
tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    //printf("pan_tx_complete_cb\n");
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        return false;
    }
    dw1000_pan_instance_t * pan = inst->pan;
    pan->idx++;
    STATS_INC(g_stat, tx_complete);
    return true;
}

/**
 * API for reset callback.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 *
 * @return bool
 */
static bool
reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    if (os_sem_get_count(&inst->pan->sem) == 0){
        STATS_INC(g_stat, reset);
        os_error_t err = os_sem_release(&inst->pan->sem);
        assert(err == OS_OK);
        return true;
    }
    return false;
}

/**
 * API for receive timeout callback.
 *
 * @param inst    Pointer to dw1000_dev_instance_t.
 *
 * @return bool
 */
static bool
rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    if (os_sem_get_count(&inst->pan->sem) == 0){
        STATS_INC(g_stat, rx_timeout);
        os_error_t err = os_sem_release(&inst->pan->sem);
        assert(err == OS_OK);
        return true;
    }
    return false;
}

/**
 * Listen for PAN requests / resets
 *
 * @param inst          Pointer to dw1000_dev_instance_t.
 *
 * @return dw1000_dev_status_t 
 */
dw1000_dev_status_t 
dw1000_pan_listen(dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode)
{
    os_error_t err = os_sem_pend(&inst->pan->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    STATS_INC(g_stat, pan_listen);

    if(dw1000_start_rx(inst).start_rx_error){
        STATS_INC(g_stat, rx_error);
        err = os_sem_release(&inst->pan->sem);
        assert(err == OS_OK);
    }

    if (mode == DWT_BLOCKING){
        err = os_sem_pend(&inst->pan->sem, OS_TIMEOUT_NEVER);
        assert(err == OS_OK);
        err = os_sem_release(&inst->pan->sem);
        assert(err == OS_OK);
    }

    return inst->status;
}


/**
 * A Personal Area Network blink request is a discovery phase in which a TAG/ANCHOR seeks to discover
 * an available PAN Master. The outcome of this process is a PANID and SLOTID assignment.
 *
 * @param inst     Pointer to dw1000_dev_instance_t.
 * @param role     Requested role in the network
 * @param mode     BLOCKING and NONBLOCKING modes.
 * @param delay    When to send this blink
 *
 * @return dw1000_pan_status_t
 */
dw1000_pan_status_t
dw1000_pan_blink(dw1000_dev_instance_t * inst, uint16_t role,
                 dw1000_dev_modes_t mode, uint64_t delay)
{
    os_error_t err = os_sem_pend(&inst->pan->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    STATS_INC(g_stat, pan_request);
    dw1000_pan_instance_t * pan = inst->pan;
    pan_frame_t * frame = pan->frames[(pan->idx)%pan->nframes];

    frame->seq_num += inst->pan->nframes;
    frame->long_address = inst->my_long_address;
    frame->code = DWT_PAN_REQ;
    frame->role = role;
    frame->lease_time = pan->config->lease_time;
    imgr_my_version(&frame->fw_ver);

    dw1000_set_delay_start(inst, delay);
    dw1000_write_tx_fctrl(inst, sizeof(struct _pan_frame_t), 0, true);
    dw1000_write_tx(inst, frame->array, 0, sizeof(struct _pan_frame_t));
    dw1000_set_wait4resp(inst, true);
    dw1000_set_rx_timeout(inst, pan->config->rx_timeout_period);
    pan->status.start_tx_error = dw1000_start_tx(inst).start_tx_error;

    if (pan->status.start_tx_error){
        STATS_INC(g_stat, tx_error);
        DIAGMSG("{\"utime\": %lu,\"msg\": \"pan_blnk_txerr\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
        // Half Period Delay Warning occured try for the next epoch
        // Use seq_num to detect this on receiver size
        os_sem_release(&inst->pan->sem);
    }
    else if(mode == DWT_BLOCKING){
        err = os_sem_pend(&inst->pan->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions
        os_sem_release(&inst->pan->sem);
        assert(err == OS_OK);
    }
    return pan->status;
}

/**
 * A Pan reset message is a broadcast to all nodes having a pan assigned address
 * instructing them to reset and renew their address. Normally issued by a restarted
 * master.
 *
 * @param inst     Pointer to dw1000_dev_instance_t.
 * @param delay    When to send this reset
 *
 * @return dw1000_pan_status_t
 */
dw1000_pan_status_t
dw1000_pan_reset(dw1000_dev_instance_t * inst, uint64_t delay)
{
    dw1000_pan_instance_t * pan = inst->pan;
    pan_frame_t * frame = pan->frames[(pan->idx)%pan->nframes];

    frame->seq_num += inst->pan->nframes;
    frame->long_address = inst->my_long_address;
    frame->code = DWT_PAN_RESET;

    dw1000_set_delay_start(inst, delay);
    dw1000_write_tx_fctrl(inst, sizeof(struct _pan_frame_t), 0, true);
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
 * A Personal Area Network blink is a discovery phase in which a TAG/ANCHOR seeks to discover
 * an available PAN Master. The pan_master does not
 * need to call this function.
 *
 * @param inst    Pointer to dw1000_dev_instance_t.
 *
 * @return void
 */
void
dw1000_pan_start(dw1000_dev_instance_t * inst, dw1000_pan_role_t role)
{
    dw1000_pan_instance_t * pan = inst->pan;
    pan->config->role = role;

    if (pan->config->role == PAN_ROLE_MASTER) {
        /* Nothing for now */
    } else if (pan->config->role == PAN_ROLE_SLAVE) {
        pan->idx = 0x1;
        pan->status.valid = false;

        printf("{\"utime\": %lu,\"PAN\": \"%s\"}\n",
               os_cputime_ticks_to_usecs(os_cputime_get32()),
               "Provisioning"
            );
    }
}

/**
 * Checks time to lease expiry
 *
 * @param inst    Pointer to dw1000_dev_instance_t.
 *
 * @return uint32_t ms to expiry, 0 if already expired
 */
uint32_t
dw1000_pan_lease_remaining(dw1000_dev_instance_t * inst)
{
    os_time_t rt = os_callout_remaining_ticks(&inst->pan->pan_lease_callout_expiry, os_time_get());
    return os_time_ticks_to_ms32(rt);
}

#endif // PAN_ENABLED
