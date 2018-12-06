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


#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_ftypes.h>

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

static dw1000_pan_config_t g_config = {
    .tx_holdoff_delay = MYNEWT_VAL(PAN_TX_HOLDOFF),         // Send Time delay in usec.
    .rx_timeout_period = MYNEWT_VAL(PAN_RX_TIMEOUT)         // Receive response timeout in usec.
};

static bool rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static void pan_postprocess(struct os_event * ev);

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
        printf("{\"utime\":%lu,\"UUID\":\"%llX\",\"ID\":\"%X\",\"PANID\":\"%X\",\"slot\":%d}\n",
            os_cputime_ticks_to_usecs(os_cputime_get32()),
            frame->long_address,
            frame->short_address,
            frame->pan_id,
            frame->slot_id
        );
    else if (inst->frame_len == sizeof(struct _ieee_blink_frame_t))
        printf("{\"utime\":%lu,\"UUID\":\"%llX\",\"seq_num\":%d}\n",
            os_cputime_ticks_to_usecs(os_cputime_get32()),
            frame->long_address,
            frame->seq_num
        );
    else if (inst->frame_len == sizeof(struct _pan_frame_resp_t))
        printf("{\"utime\":%lu,\"UUID\":\"%llX\",\"ID\":\"%X\",\"PANID\":\"%X\",\"slot\":%d}\n",
            os_cputime_ticks_to_usecs(os_cputime_get32()),
            frame->long_address,
            frame->short_address,
            frame->pan_id,
            frame->slot_id
        );
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
        return false;
    }

    dw1000_pan_instance_t * pan = inst->pan;
    pan_frame_t * frame = pan->frames[(pan->idx)%pan->nframes];

    /* Ignore frames that are too long */
    if (inst->frame_len > sizeof(struct _pan_frame_t)) {
        return false;
    }
    memcpy(frame->array, inst->rxbuf, inst->frame_len);

    switch(frame->code) {
    case DWT_PAN_REQ:
        break;
    case DWT_PAN_RESP:
        if(frame->long_address == inst->my_long_address){
            /* TAG/ANCHOR side */
            inst->my_short_address = frame->short_address;
            inst->PANID = frame->pan_id;
            inst->slot_id = frame->slot_id;
            pan->status.valid = true;
        }
        break;
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
        os_sem_release(&inst->pan->sem);  
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
        os_sem_release(&inst->pan->sem);  
        return true;
    }
    return false;
}

/**
 * Listen for PAN requests
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

    if(dw1000_start_rx(inst).start_rx_error){
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

    dw1000_pan_instance_t * pan = inst->pan;
    pan_frame_t * frame = pan->frames[(pan->idx)%pan->nframes];

    frame->seq_num += inst->pan->nframes;
    frame->long_address = inst->my_long_address;
    frame->code = DWT_PAN_REQ;
    frame->role = role;

    dw1000_write_tx(inst, frame->array, 0, sizeof(struct _pan_frame_req_t));
    dw1000_write_tx_fctrl(inst, sizeof(struct _pan_frame_req_t), 0, true);
    dw1000_set_wait4resp(inst, true);
    dw1000_set_delay_start(inst, delay);
    dw1000_set_rx_timeout(inst, pan->config->rx_timeout_period);
    pan->status.start_tx_error = dw1000_start_tx(inst).start_tx_error;
    if (pan->status.start_tx_error){
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

        printf("{\"utime\":%lu,\"PAN\":\"%s\"}\n",
               os_cputime_ticks_to_usecs(os_cputime_get32()),
               "Provisioning"
            );
    }
}

#endif // PAN_ENABLED
