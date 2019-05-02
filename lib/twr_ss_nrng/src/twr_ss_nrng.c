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
#include <nrng/nrng.h>
#if MYNEWT_VAL(WCS_ENABLED)
#include <wcs/wcs.h>
#endif
#include <dsp/polyval.h>
#include <rng/slots.h>

#define WCS_DTU MYNEWT_VAL(WCS_DTU)

//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

static bool rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool rx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);
static bool reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);

static dw1000_mac_interface_t g_cbs = {
    .id = DW1000_NRNG_SS,
    .rx_complete_cb = rx_complete_cb,
    .rx_timeout_cb = rx_timeout_cb,
    .rx_error_cb = rx_error_cb,
    .reset_cb = reset_cb
};

static dw1000_rng_config_t g_config = {
    .tx_holdoff_delay = MYNEWT_VAL(TWR_SS_NRNG_TX_HOLDOFF),         // Send Time delay in usec.
    .rx_timeout_delay = MYNEWT_VAL(TWR_SS_NRNG_RX_TIMEOUT),        // Receive response timeout in usec
    .tx_guard_delay = MYNEWT_VAL(TWR_SS_NRNG_TX_GUARD_DELAY)        // Guard delay to be added between each frame from node
};


/**
 * API to initialise the rng_ss package.
 *
 *
 * @return void
 */
void twr_ss_nrng_pkg_init(void){
#if MYNEWT_VAL(DW1000_PKG_INIT_LOG)
    printf("{\"utime\": %lu,\"msg\": \"ss_nrng_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
#endif
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
 * API for receive error callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
static bool 
rx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    /* Place holder */
    if(inst->fctrl != FCNTL_IEEE_RANGE_16)
        return false;
    
    dw1000_nrng_instance_t * nrng = inst->nrng;
    if(os_sem_get_count(&nrng->sem) == 0){
        NRNG_STATS_INC(rx_error);
        os_error_t err = os_sem_release(&nrng->sem);
        assert(err == OS_OK);
        return true;
    }
    return false;
}


/**
 * API for receive timeout callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
static bool 
rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    dw1000_nrng_instance_t * nrng = inst->nrng;
    if(os_sem_get_count(&nrng->sem) == 1)
        return false;

    if(os_sem_get_count(&nrng->sem) == 0){
        NRNG_STATS_INC(rx_timeout);
        // In the case of a NRNG timeout is used to mark the end of the request 
        // and is used to call the completion callback  
        if(!(SLIST_EMPTY(&inst->interface_cbs))){
            SLIST_FOREACH(cbs, &inst->interface_cbs, next){
            if (cbs!=NULL && cbs->complete_cb)
                if(cbs->complete_cb(inst, cbs)) continue;
            }
        }
        os_error_t err = os_sem_release(&nrng->sem);
        assert(err == OS_OK);
    }    
    return true;
}


/** 
 * API for reset_cb of nrng interface
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 * @return true on sucess
 */
static bool
reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){

    if(os_sem_get_count(&inst->nrng->sem) == 0){
        os_error_t err = os_sem_release(&inst->nrng->sem);  
        assert(err == OS_OK);
        NRNG_STATS_INC(reset);
        return true;
    }
    else 
        return false;
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
    assert(inst->nrng);
    dw1000_nrng_instance_t * nrng = inst->nrng;

    if(inst->fctrl != FCNTL_IEEE_RANGE_16)
        return false;
    
    if(os_sem_get_count(&nrng->sem) == 1){ 
        // unsolicited inbound
        NRNG_STATS_INC(rx_unsolicited);
        return false;
    }

    dw1000_rng_config_t * config = dw1000_nrng_get_config(inst, DWT_SS_TWR_NRNG);
    nrng_request_frame_t * _frame = (nrng_request_frame_t * )inst->rxbuf;

    if (_frame->dst_address != inst->my_short_address && _frame->dst_address != BROADCAST_ADDRESS)
        return true;
   
    NRNG_STATS_INC(rx_complete);

    switch(_frame->code){
        case DWT_SS_TWR_NRNG:
            {
                // This code executes on the device that is responding to a request
                DIAGMSG("{\"utime\": %lu,\"msg\": \"DWT_SS_TWR_NRNG\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
                if (inst->frame_len < sizeof(nrng_request_frame_t)) 
                    break;
                uint16_t slot_idx;    
#if MYNEWT_VAL(CELL_ENABLED)
                if (_frame->ptype != PTYPE_CELL) 
                    break;
                if (_frame->cell_id != inst->cell_id)
                    break; 
                if (_frame->slot_mask & (1UL << inst->slot_id))
                    slot_idx = BitIndex(_frame->slot_mask, 1UL << inst->slot_id, SLOT_POSITION);
                else
                    break;
#else
                if (_frame->bitfield & (1UL << inst->slot_id))
                    slot_idx = BitIndex(_frame->bitfield, 1UL << inst->slot_id, SLOT_POSITION);
                else
                    break;
#endif         
                nrng_final_frame_t * frame = (nrng_final_frame_t *) nrng->frames[(++nrng->idx)%nrng->nframes];
                memcpy(frame->array, inst->rxbuf, sizeof(nrng_request_frame_t));

                uint64_t request_timestamp = inst->rxtimestamp;
                uint64_t response_tx_delay = request_timestamp 
                            + (((uint64_t)config->tx_holdoff_delay
                            + (uint64_t)(slot_idx * ((uint64_t)config->tx_guard_delay
                            + (uint64_t)(dw1000_usecs_to_dwt_usecs(dw1000_phy_frame_duration(&inst->attrib, sizeof(nrng_response_frame_t)))))))<< 16);
                uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;

#if MYNEWT_VAL(WCS_ENABLED)
                wcs_instance_t * wcs = inst->ccp->wcs;
                frame->reception_timestamp = (uint32_t)(wcs_local_to_master(wcs, request_timestamp)) & 0xFFFFFFFFULL;
                frame->transmission_timestamp = (uint32_t)(wcs_local_to_master(wcs, response_timestamp)) & 0xFFFFFFFFULL;
#else
                frame->reception_timestamp = request_timestamp & 0xFFFFFFFFULL;
                frame->transmission_timestamp = response_timestamp & 0xFFFFFFFFULL;
#endif
                frame->dst_address = _frame->src_address;
                frame->src_address = inst->my_short_address;
                frame->code = DWT_SS_TWR_NRNG_T1;
                frame->slot_id = slot_idx;
                frame->seq_num = _frame->seq_num;

#if MYNEWT_VAL(WCS_ENABLED)
                frame->carrier_integrator  = 0.0l;
#else
                frame->carrier_integrator  = - inst->carrier_integrator;
#endif
                dw1000_write_tx(inst, frame->array, 0, sizeof(nrng_response_frame_t));
                dw1000_write_tx_fctrl(inst, sizeof(nrng_response_frame_t), 0);
                dw1000_set_wait4resp(inst, false);
                dw1000_set_delay_start(inst, response_tx_delay);

                if (dw1000_start_tx(inst).start_tx_error){
                    os_sem_release(&nrng->sem);  
                    if (cbs!=NULL && cbs->start_tx_error_cb) {
                        cbs->start_tx_error_cb(inst, cbs);
                    }
                }else{
                    os_sem_release(&nrng->sem);
                }
            break;
            }
        case DWT_SS_TWR_NRNG_T1:
            {
                // This code executes on the device that initiated a request, and is now preparing the final timestamps
                DIAGMSG("{\"utime\": %lu,\"msg\": \"DWT_SS_TWR_NRNG_T1\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
                if (inst->frame_len < sizeof(nrng_response_frame_t))
                    break;  

                nrng_response_frame_t * _frame = (nrng_response_frame_t * )inst->rxbuf;
                uint16_t idx = _frame->slot_id;

                // Reject out of sequence ranges, this should never occur in a well behaved system
                if (inst->nrng->seq_num != _frame->seq_num)
                    break;

                nrng_frame_t * frame = nrng->frames[(nrng->idx + idx)%nrng->nframes];
                memcpy(frame, inst->rxbuf, sizeof(nrng_response_frame_t));

                uint64_t response_timestamp = 0x0;
                if (inst->status.lde_error == 0) 
                   response_timestamp = inst->rxtimestamp;

#if MYNEWT_VAL(WCS_ENABLED)
                wcs_instance_t * wcs = inst->ccp->wcs;
                frame->request_timestamp = wcs_local_to_master(wcs, dw1000_read_txtime(inst)) & 0xFFFFFFFFULL;
                frame->response_timestamp = wcs_local_to_master(wcs, response_timestamp) & 0xFFFFFFFFULL;
#else
                frame->request_timestamp = dw1000_read_txtime_lo(inst) & 0xFFFFFFFFUL;
                frame->response_timestamp  = (uint32_t)(response_timestamp & 0xFFFFFFFFULL);
#endif
                frame->dst_address = frame->src_address;
                frame->src_address = inst->my_short_address;
                frame->code = DWT_SS_TWR_NRNG_FINAL;
#if MYNEWT_VAL(WCS_ENABLED)
                frame->carrier_integrator  = 0.0l;
#else
                frame->carrier_integrator  = inst->carrier_integrator;
#endif
                if(inst->config.rxdiag_enable) {
                    memcpy(&frame->diag, &inst->rxdiag, sizeof(struct _dw1000_dev_rxdiag_t));
                }
                if(idx == nrng->nnodes-1){
                     dw1000_set_rx_timeout(inst, 1); // Triger timeout event
                }else{
                    // Incrementally reduce the remaining timeout calculation in accordance with what is still to come.
                    uint16_t timeout = usecs_to_response(inst,     
                                nrng->nnodes - idx,                // no. of remaining frames
                                config,                            // Guard delay 
                                dw1000_phy_frame_duration(&inst->attrib, sizeof(nrng_response_frame_t)) // frame duration in usec
                            ) + config->rx_timeout_delay;          // TOF allowance.
                    dw1000_set_rx_timeout(inst, timeout);
                }
            break;
            }
        default:
                return false;
            break;
        }
    return true;
}

