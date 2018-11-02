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
 * @file twr_ss.c
 * @author paul kettle
 * @date 2018
 * @brief Range 
 *
 * @details This is the rng base class which utilises the functions to enable/disable the configurations related to rng.
 *
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>
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
#include <nranges/nranges.h>
#if MYNEWT_VAL(WCS_ENABLED)
#include <wcs/wcs.h>
#endif
#include <dsp/polyval.h>

//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

static bool rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);
static bool rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);
static bool rx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);

static dw1000_mac_interface_t g_cbs = {
            .id = DW1000_NRNG_SS,
            .rx_complete_cb = rx_complete_cb,
            .rx_timeout_cb = rx_timeout_cb,
            .rx_error_cb = rx_error_cb,
};


static dw1000_rng_config_t g_config = {
    .tx_holdoff_delay = MYNEWT_VAL(TWR_SS_NRNG_TX_HOLDOFF),         // Send Time delay in usec.
    .rx_timeout_period = MYNEWT_VAL(TWR_SS_NRNG_RX_TIMEOUT),        // Receive response timeout in usec
    .tx_guard_delay = MYNEWT_VAL(TWR_SS_NRNG_TX_GUARD_DELAY)        // Guard delay to be added between each frame from node
};
/**
 * API to initialise the rng_ss package.
 *
 *
 * @return void
 */

void twr_ss_nrng_pkg_init(void){

    printf("{\"utime\": %lu,\"msg\": \"twr_ss_nrng_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
    dw1000_mac_append_interface(hal_dw1000_inst(0), &g_cbs);
}


/**
 * API to free the allocated resources.
 *
 * @param inst  Pointer to dw1000_rng_instance_t.
 *
 * @return void 
 */
void 
twr_ss_nrng_free(dw1000_dev_instance_t * inst){
    assert(inst); 
    dw1000_mac_remove_interface(inst, DW1000_NRNG_SS);
}

/**
 * API for get local config callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
dw1000_rng_config_t * 
twr_ss_nrng_config(dw1000_dev_instance_t * inst){
    return &g_config;
}


/**
 * API for receive timeout callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
static bool 
rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    if(inst->fctrl != FCNTL_IEEE_N_RANGES_16){
        return false;
    }
    assert(inst->nrng);
    switch(inst->nrng->code){
        case DWT_SS_TWR_NRNG ... DWT_SS_TWR_NRNG_FINAL:
            {
                if(!(SLIST_EMPTY(&inst->interface_cbs))){
                    SLIST_FOREACH(cbs, &inst->interface_cbs, next){
                        if (cbs!=NULL && cbs->complete_cb)
                            if(cbs->complete_cb(inst, cbs)) continue;
                    }
                }
                dw1000_nrng_instance_t * nrng = inst->nrng;
                os_error_t err = os_sem_release(&nrng->sem);
                assert(err == OS_OK);
                break;
            }
        default:
            return false;
    }
    return true;
}

/**
 * API for receive error callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
static bool 
rx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    /* Place holder */
    if(inst->fctrl != FCNTL_IEEE_N_RANGES_16){
        return false;
    }
    assert(inst->nrng);
    dw1000_nrng_instance_t * nrng = inst->nrng;
    os_error_t err = os_sem_release(&nrng->sem);
    assert(err == OS_OK);
    return true;
}

/**
 * API for receive complete callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
static bool 
rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    if(inst->fctrl != FCNTL_IEEE_N_RANGES_16){
        return false;
    }
    assert(inst->nrng);
    dw1000_nrng_instance_t * nrng = inst->nrng;
    dw1000_rng_config_t * config = dw1000_nrng_get_config(inst, DWT_SS_TWR_NRNG);
    switch(inst->nrng->code){
        case DWT_SS_TWR_NRNG:
            {
                // This code executes on the device that is responding to a request
                DIAGMSG("{\"utime\": %lu,\"msg\": \"DWT_SS_TWR\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
                dw1000_nrng_instance_t * nrng = inst->nrng; 
                nrng_frame_t * frame = nrng->frames[(++nrng->idx)%(nrng->nframes/FRAMES_PER_RANGE)][FIRST_FRAME_IDX];
                uint16_t slot_id = inst->slot_id;
                if (inst->frame_len >= sizeof(nrng_request_frame_t))
                    dw1000_read_rx(inst, frame->array, 0, sizeof(nrng_request_frame_t));
                else
                    break;
                if(!(slot_id >= frame->start_slot_id && slot_id <= frame->end_slot_id))
                    break;

#if MYNEWT_VAL(WCS_ENABLED)                
                uint64_t request_timestamp = wcs_read_rxtime(inst);
                frame->carrier_integrator  = 0.0l;
#else
                uint64_t request_timestamp = dw1000_read_rxtime(inst);
                frame->carrier_integrator  = dw1000_read_carrier_integrator(inst);
#endif
                uint64_t response_tx_delay = request_timestamp + (((uint64_t)config->tx_holdoff_delay
                            + (uint64_t)((slot_id-1) * ((uint64_t)config->tx_guard_delay
                                    + (dw1000_usecs_to_dwt_usecs(dw1000_phy_frame_duration(&inst->attrib, sizeof(nrng_response_frame_t)))))))<< 16);
                uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;

                frame->reception_timestamp = request_timestamp;
                frame->transmission_timestamp = response_timestamp;
                frame->dst_address = frame->src_address;
                frame->src_address = inst->my_short_address;
                frame->code = DWT_SS_TWR_NRNG_T1;
                frame->slot_id = slot_id;

                dw1000_write_tx(inst, frame->array, 0, sizeof(nrng_response_frame_t));
                dw1000_write_tx_fctrl(inst, sizeof(nrng_response_frame_t), 0, true);
                dw1000_set_wait4resp(inst, false);
                dw1000_set_delay_start(inst, response_tx_delay);
                if (dw1000_start_tx(inst).start_tx_error){
                    if (cbs!=NULL && cbs->start_tx_error_cb)
                        cbs->start_tx_error_cb(inst, cbs);
                }
                break;
            }
        case DWT_SS_TWR_NRNG_T1:
            {
                // This code executes on the device that initiated a request, and is now preparing the final timestamps
                DIAGMSG("{\"utime\": %lu,\"msg\": \"DWT_SS_TWR_T1\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
                uint16_t nnodes = nrng->nnodes;
                uint16_t idx = 0;
                nrng_frame_t temp_frame;
                if (inst->frame_len >= sizeof(nrng_response_frame_t))
                    dw1000_read_rx(inst, temp_frame.array, 0, sizeof(nrng_response_frame_t));
                else
                    break;

                uint16_t node_slot_id = temp_frame.slot_id;
                uint16_t end_slot_id = temp_frame.end_slot_id;
                nrng->idx = idx = node_slot_id - temp_frame.start_slot_id;
                if(idx < (nnodes-1)){
                    // At the start the device will wait for the entire nnodes to respond as a single huge timeout.
                    // When a node respond we will recalculate the remaining time to be waited for as (total_nodes - completed_nodes)*(phy_duaration + guard_delay)
                    uint16_t phy_duration = dw1000_phy_frame_duration(&inst->attrib, sizeof(nrng_response_frame_t));
                    uint16_t timeout = ((phy_duration + dw1000_dwt_usecs_to_usecs(config->tx_guard_delay)) * (end_slot_id - node_slot_id));
                    dw1000_set_rx_timeout(inst, timeout);
                    dw1000_start_rx(inst);
                }
                nrng_frame_t * frame = nrng->frames[idx][FIRST_FRAME_IDX];
                memcpy(frame, &temp_frame, sizeof(nrng_response_frame_t));
#if MYNEWT_VAL(WCS_ENABLED) 
                frame->request_timestamp = wcs_read_txtime_lo(inst);   // This corresponds to when the original request was actually sent
                frame->response_timestamp = wcs_read_rxtime_lo(inst);  // This corresponds to the response just received   
                frame->carrier_integrator  = 0.0l;
#else
                frame->request_timestamp = dw1000_read_txtime_lo(inst);   // This corresponds to when the original request was actually sent
                frame->response_timestamp = dw1000_read_rxtime_lo(inst);  // This corresponds to the response just received   
                frame->carrier_integrator  = dw1000_read_carrier_integrator(inst);
#endif
                frame->dst_address = frame->src_address;
                frame->src_address = inst->my_short_address;
                frame->code = DWT_SS_TWR_NRNG_FINAL;

                if(idx == nnodes-1){
                    os_sem_release(&nrng->sem);
                    if(!(SLIST_EMPTY(&inst->interface_cbs))){
                        SLIST_FOREACH(cbs, &inst->interface_cbs, next){
                            if (cbs!=NULL && cbs->complete_cb)
                                if(cbs->complete_cb(inst, cbs)) continue;
                        }
                    }
                }
                break;
            }
        default:
            return false;
            break;
    }
    return true;
}
