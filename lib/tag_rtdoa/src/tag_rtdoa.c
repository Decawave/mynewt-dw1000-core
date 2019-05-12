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

static bool rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool rx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);
static bool reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);

static dw1000_mac_interface_t g_cbs = {
    .id = DW1000_RTDOA,
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
void
tag_rtdoa_pkg_init(void)
{
#if MYNEWT_VAL(DW1000_PKG_INIT_LOG)
    printf("{\"utime\": %lu,\"msg\": \"tag_rtdoa_pkg_init\"}\n", os_cputime_ticks_to_usecs(os_cputime_get32()));
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
tag_rtdoa_free(dw1000_dev_instance_t * inst)
{
    assert(inst); 
    dw1000_mac_remove_interface(inst, DW1000_RTDOA);
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
    if(os_sem_get_count(&rtdoa->sem) == 0) {
        RTDOA_STATS_INC(rx_timeout);
        os_error_t err = os_sem_release(&rtdoa->sem);
        assert(err == OS_OK);
    } else {    
        return false;
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
    int64_t new_timeout;
    assert(inst->rtdoa);
    dw1000_rtdoa_instance_t * rtdoa = inst->rtdoa;

    if(inst->fctrl != FCNTL_IEEE_RANGE_16)
        return false;

    if(os_sem_get_count(&rtdoa->sem) == 1){ 
        // unsolicited inbound
        RTDOA_STATS_INC(rx_unsolicited);
        return false;
    }

    rtdoa_request_frame_t * _frame = (rtdoa_request_frame_t * )inst->rxbuf;

    if (_frame->dst_address != inst->my_short_address &&
        _frame->dst_address != BROADCAST_ADDRESS) {
        return true;
    }
   
    RTDOA_STATS_INC(rx_complete);

    switch(_frame->code){
        case DWT_RTDOA_REQUEST:
        {
            // The initial packet in the rtdoa sequence either from the master or
            // one of the relaying nodes
            if (inst->frame_len < sizeof(rtdoa_request_frame_t)) {
                break;
            }
            
            rtdoa_frame_t * frame = (rtdoa_frame_t *) rtdoa->frames[(++rtdoa->idx)%rtdoa->nframes];
            rtdoa->req_frame = frame;
            memcpy(frame->array, inst->rxbuf, sizeof(rtdoa_request_frame_t));
            
            frame->rx_timestamp = wcs_local_to_master64(inst->ccp->wcs, inst->rxtimestamp);
            
            /* Compensate for relays */
            if (frame->rpt_count != 0) {
                RTDOA_STATS_INC(rx_relayed);
                uint32_t repeat_dly = frame->rpt_count*rtdoa->config.tx_holdoff_delay;
                frame->rx_timestamp = (inst->ccp->master_epoch.timestamp - (repeat_dly << 16));
            }

            /* A good rtdoa_req packet has been received, stop the receiver */
            dw1000_stop_rx(inst);
            /* Adjust timeout and delayed start to match when the responses will arrive */
            uint64_t dx_time = inst->rxtimestamp;
            dx_time += (rtdoa_usecs_to_response(inst, (rtdoa_request_frame_t*)rtdoa->req_frame, 0, &rtdoa->config,
                            dw1000_phy_frame_duration(&inst->attrib, sizeof(rtdoa_response_frame_t))) << 16);

            dw1000_set_delay_start(inst, dx_time);
            if(dw1000_start_rx(inst).start_rx_error){
                os_sem_release(&rtdoa->sem);
                RTDOA_STATS_INC(start_rx_error);
            } else {
                printf("q\n");
            }
            goto adj_to_return;
            break; 
        }
        case DWT_RTDOA_RESP:
        {
            // The packets following the initial request from all nodes
            if (inst->frame_len < sizeof(rtdoa_response_frame_t)) {
                break;
            }
            
            rtdoa_frame_t * frame = (rtdoa_frame_t *) rtdoa->frames[(++rtdoa->idx)%rtdoa->nframes];
            memcpy(frame->array, inst->rxbuf, sizeof(rtdoa_request_frame_t));
            frame->rx_timestamp = wcs_local_to_master64(inst->ccp->wcs, inst->rxtimestamp);
            
            /* Place this on a queue for processing later */
            printf("r%x %lld\n", _frame->src_address, frame->rx_timestamp - rtdoa->req_frame->rx_timestamp);
            break; 
        }
        default:
            return false;
            break;
        }
    return true;

adj_to_return:
    new_timeout = (int64_t)inst->rtdoa->timeout - (int64_t)inst->rxtimestamp;
    if (new_timeout < 0) new_timeout = 1;
    dw1000_set_rx_timeout(inst, (uint16_t)(new_timeout>>16));
    return true;
}

