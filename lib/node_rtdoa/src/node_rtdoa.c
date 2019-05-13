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
#include <ccp/ccp.h>
#include <rtdoa/rtdoa.h>
#include <wcs/wcs.h>
#include <dsp/polyval.h>
#include <rng/slots.h>

#define WCS_DTU MYNEWT_VAL(WCS_DTU)

#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

static bool tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool rx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);
static bool reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);

static dw1000_mac_interface_t g_cbs = {
    .id = DW1000_RTDOA,
    .tx_complete_cb = tx_complete_cb,
    .rx_complete_cb = rx_complete_cb,
    .rx_timeout_cb = rx_timeout_cb,
    .rx_error_cb = rx_error_cb,
    .reset_cb = reset_cb
};

/**
 * API to initialise the rng_ss package.
 *
 *
 * @return void
 */
void node_rtdoa_pkg_init(void){
#if MYNEWT_VAL(DW1000_PKG_INIT_LOG)
    printf("{\"utime\": %lu,\"msg\": \"node_rtdoa_pkg_init\"}\n", os_cputime_ticks_to_usecs(os_cputime_get32()));
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
node_rtdoa_free(dw1000_dev_instance_t * inst){
    assert(inst); 
    dw1000_mac_remove_interface(inst, DW1000_RTDOA);
}

dw1000_dev_status_t
dw1000_rtdoa_request(dw1000_dev_instance_t * inst, uint64_t delay)
{
    /* This function executes on the device that initiates the rtdoa sequence */
    assert(inst->rtdoa);
    dw1000_rtdoa_instance_t * rtdoa = inst->rtdoa;

    os_error_t err = os_sem_pend(&rtdoa->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);
    RTDOA_STATS_INC(rtdoa_request);

    rtdoa->req_frame = rtdoa->frames[rtdoa->idx%rtdoa->nframes];
    rtdoa_request_frame_t * frame = (rtdoa_request_frame_t *) rtdoa->req_frame;

    frame->seq_num = ++rtdoa->seq_num;
    frame->code = DWT_RTDOA_REQUEST;
    frame->src_address = inst->my_short_address;
    frame->dst_address = BROADCAST_ADDRESS;
    frame->slot_modulus = MYNEWT_VAL(RTDOA_NNODES);
    frame->rpt_count = 0;
    frame->rpt_max = MYNEWT_VAL(RTDOA_MAX_CASCADE_RPTS);

    dw1000_set_delay_start(inst, delay);

    /* Calculate tx_timestamp */
    wcs_instance_t * wcs = inst->ccp->wcs;
    frame->tx_timestamp = (wcs_local_to_master64(wcs, delay) & 0xFFFFFFFFFFFFFE00ULL);
    frame->tx_timestamp += inst->tx_antenna_delay;
    /* Also set the local rx_timestamp to allow us to also transmit in the next part */
    rtdoa->req_frame->rx_timestamp = frame->tx_timestamp;

    dw1000_write_tx(inst, frame->array, 0, sizeof(rtdoa_request_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(rtdoa_request_frame_t), 0);
    dw1000_set_wait4resp(inst, false);

    if (dw1000_start_tx(inst).start_tx_error) {
        RTDOA_STATS_INC(start_tx_error);
        if (os_sem_get_count(&rtdoa->sem) == 0) {
            err = os_sem_release(&rtdoa->sem);
            assert(err == OS_OK);
        }
    } else {
        err = os_sem_pend(&rtdoa->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions
        assert(err == OS_OK);
        err = os_sem_release(&rtdoa->sem);
        assert(err == OS_OK);
    }
    return inst->status;
}


static dw1000_dev_status_t
tx_rtdoa_response(dw1000_dev_instance_t * inst, uint64_t delay)
{
    /* This function executes on the device that responds to a rtdoa request */
    assert(inst->rtdoa);
    dw1000_rtdoa_instance_t * rtdoa = inst->rtdoa;

    RTDOA_STATS_INC(rtdoa_response);

    rtdoa_response_frame_t * frame = (rtdoa_response_frame_t *) rtdoa->frames[++rtdoa->idx%rtdoa->nframes];

    frame->seq_num = ++rtdoa->seq_num;
    frame->code = DWT_RTDOA_RESP;
    frame->src_address = inst->my_short_address;
    frame->dst_address = BROADCAST_ADDRESS;
    frame->slot_id = inst->slot_id%rtdoa->req_frame->slot_modulus + 1;

    dw1000_set_delay_start(inst, delay& 0x000000FFFFFFFE00ULL);

    /* Calculate tx-time using wcs */
    wcs_instance_t * wcs = inst->ccp->wcs;  
    frame->tx_timestamp = (wcs_local_to_master64(wcs, delay) & 0xFFFFFFFFFFFFFE00ULL) + inst->tx_antenna_delay;

    dw1000_write_tx(inst, frame->array, 0, sizeof(rtdoa_response_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(rtdoa_response_frame_t), 0);
    dw1000_set_wait4resp(inst, false);

    if (dw1000_start_tx(inst).start_tx_error) {
        RTDOA_STATS_INC(start_tx_error);
        if (os_sem_get_count(&rtdoa->sem) == 0) {
            os_error_t err = os_sem_release(&rtdoa->sem);
            assert(err == OS_OK);
        }
    }
    
    return inst->status;
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
    
    dw1000_rtdoa_instance_t * rtdoa = inst->rtdoa;
    if(os_sem_get_count(&rtdoa->sem) == 0){
        RTDOA_STATS_INC(rx_error);
        os_error_t err = os_sem_release(&rtdoa->sem);
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
    dw1000_rtdoa_instance_t * rtdoa = inst->rtdoa;
    if(os_sem_get_count(&rtdoa->sem) == 1) {
        return false;
    }

    if(os_sem_get_count(&rtdoa->sem) == 0){
        os_error_t err = os_sem_release(&rtdoa->sem);
        assert(err == OS_OK);
        RTDOA_STATS_INC(rx_timeout);
    }    
    return true;
}


/** 
 * API for reset_cb of rtdoa interface
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 * @return true on sucess
 */
static bool
reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    if(os_sem_get_count(&inst->rtdoa->sem) == 0){
        os_error_t err = os_sem_release(&inst->rtdoa->sem);  
        assert(err == OS_OK);
        RTDOA_STATS_INC(reset);
        return true;
    }
    else 
        return false;
}

/**
 * API for transmit complete callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
static bool 
tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    assert(inst->rtdoa);
    dw1000_rtdoa_instance_t * rtdoa = inst->rtdoa;
    dw1000_rng_config_t * config = &rtdoa->config;

    /* If we sent the request or a repeat of the request, send response now */
    rtdoa_request_frame_t * frame = (rtdoa_request_frame_t *) rtdoa->frames[rtdoa->idx%rtdoa->nframes];
    if (frame->code == DWT_RTDOA_REQUEST) {
        /* Transmit response packet */
        /* Rx-timestamp will be compensated for relay */
        uint64_t dx_time = rtdoa->req_frame->rx_timestamp;
        /* usecs to dwt usecs? */
        uint8_t slot_idx = inst->slot_id%rtdoa->req_frame->slot_modulus + 1;
        dx_time += (rtdoa_usecs_to_response(inst, (rtdoa_request_frame_t*)rtdoa->req_frame, slot_idx, config,
                                            dw1000_phy_frame_duration(&inst->attrib, sizeof(rtdoa_response_frame_t))) << 16);

        tx_rtdoa_response(inst, dx_time);
        // printf("dx_time %d %llx d:%llx\n", slot_idx, dx_time, dx_time - rtdoa->req_frame->rx_timestamp);
        return true;
    }

    if(os_sem_get_count(&inst->rtdoa->sem) == 0){
        os_error_t err = os_sem_release(&inst->rtdoa->sem);  
        assert(err == OS_OK);
        return true;
    } else {
        return false;
    }
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
    assert(inst->rtdoa);
    dw1000_rtdoa_instance_t * rtdoa = inst->rtdoa;

    if(inst->fctrl != FCNTL_IEEE_RANGE_16)
        return false;

    if(os_sem_get_count(&rtdoa->sem) == 1){ 
        // unsolicited inbound
        RTDOA_STATS_INC(rx_unsolicited);
        return false;
    }

    //dw1000_rng_config_t * config = &rtdoa->config;
    rtdoa_request_frame_t * _frame = (rtdoa_request_frame_t * )inst->rxbuf;

    if (_frame->dst_address != inst->my_short_address && _frame->dst_address != BROADCAST_ADDRESS)
        return true;
   
    RTDOA_STATS_INC(rx_complete);

    switch(_frame->code){
        case DWT_RTDOA_REQUEST:
            {
                // This code executes on the device that is responding to a request
                if (inst->frame_len < sizeof(rtdoa_request_frame_t)) 
                    break;

                rtdoa_frame_t * frame = (rtdoa_frame_t *) rtdoa->frames[(++rtdoa->idx)%rtdoa->nframes];
                rtdoa->req_frame = frame;
                memcpy(frame->array, inst->rxbuf, sizeof(rtdoa_request_frame_t));

                /* Deliberately in local timeframe */
                frame->rx_timestamp = inst->rxtimestamp;
    
                /* Compensate for time of flight using the ccp function */
                if (inst->ccp->tof_comp_cb) {
                    frame->rx_timestamp -= inst->ccp->tof_comp_cb(0, frame->src_address);
                }

                /* Compensate for relays */
                if (frame->rpt_count != 0) {
                    RTDOA_STATS_INC(rx_relayed);
                    uint32_t repeat_dly = frame->rpt_count*rtdoa->config.tx_holdoff_delay;
                    frame->rx_timestamp -= (repeat_dly << 16);
                }

                /* Send a cascade relay if this is an ok relay */
                if (frame->rpt_count < frame->rpt_max) {
                    rtdoa_request_frame_t tx_frame;
                    memcpy(tx_frame.array, frame->array, sizeof(tx_frame));
                    tx_frame.src_address = inst->my_short_address;
                    tx_frame.rpt_count++;
                    uint64_t tx_timestamp = inst->rxtimestamp + tx_frame.rpt_count*((uint64_t)inst->ccp->config.tx_holdoff_dly<<16);
                    tx_timestamp &= 0x0FFFFFFFE00UL;
                    dw1000_set_delay_start(inst, tx_timestamp);
                    tx_timestamp += inst->tx_antenna_delay;

                    tx_frame.tx_timestamp = wcs_local_to_master64(inst->ccp->wcs, tx_timestamp);
                    dw1000_write_tx_fctrl(inst, sizeof(tx_frame), 0);
                    dw1000_write_tx(inst, tx_frame.array, 0, sizeof(tx_frame));
                    if (dw1000_start_tx(inst).start_tx_error) {
                        RTDOA_STATS_INC(tx_relay_error);
                        if(os_sem_get_count(&inst->rtdoa->sem) == 0){
                            os_sem_release(&inst->rtdoa->sem);
                        }
                    } else {
                        RTDOA_STATS_INC(tx_relay_ok);
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

