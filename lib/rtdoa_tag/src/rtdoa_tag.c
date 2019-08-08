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
#include <rtdoa_tag/rtdoa_tag.h>
#include <wcs/wcs.h>
#include <dsp/polyval.h>
#include <rng/slots.h>

#define WCS_DTU MYNEWT_VAL(WCS_DTU)

#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

static dw1000_rng_config_t g_config = {
    .tx_holdoff_delay = MYNEWT_VAL(RTDOA_TX_HOLDOFF),       // Send Time delay in usec.
    .rx_timeout_delay = MYNEWT_VAL(RTDOA_RX_TIMEOUT),       // Receive response timeout in usec
    .tx_guard_delay = MYNEWT_VAL(RTDOA_TX_GUARD_DELAY)
};

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
rtdoa_tag_pkg_init(void)
{
    struct _dw1000_rtdoa_instance_t *rtdoa = 0;
#if MYNEWT_VAL(DW1000_PKG_INIT_LOG)
    printf("{\"utime\": %lu,\"msg\": \"rtdoa_tag_pkg_init\"}\n", os_cputime_ticks_to_usecs(os_cputime_get32()));
#endif

#if MYNEWT_VAL(DW1000_DEVICE_0)
    g_cbs.inst_ptr = rtdoa = dw1000_rtdoa_init(hal_dw1000_inst(0), &g_config, MYNEWT_VAL(RTDOA_NFRAMES));
    dw1000_rtdoa_set_frames(rtdoa, MYNEWT_VAL(RTDOA_NFRAMES));
#endif
    dw1000_mac_append_interface(hal_dw1000_inst(0), &g_cbs);

    /* Assume that the ccp has been added to the dev instance before rtdoa */
    rtdoa->ccp = (dw1000_ccp_instance_t*)dw1000_mac_find_cb_inst_ptr(hal_dw1000_inst(0), DW1000_CCP);
}


/**
 * API to free the allocated resources.
 *
 * @param inst  Pointer to dw1000_rng_instance_t.
 *
 * @return void 
 */
void 
rtdoa_tag_free(dw1000_dev_instance_t * inst)
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
rx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    dw1000_rtdoa_instance_t * rtdoa = (dw1000_rtdoa_instance_t *)cbs->inst_ptr;
    if(inst->fctrl != FCNTL_IEEE_RANGE_16)
        return false;

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
    dw1000_rtdoa_instance_t * rtdoa = (dw1000_rtdoa_instance_t *)cbs->inst_ptr;
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
    dw1000_rtdoa_instance_t * rtdoa = (dw1000_rtdoa_instance_t *)cbs->inst_ptr;
    if(os_sem_get_count(&rtdoa->sem) == 0){
        os_error_t err = os_sem_release(&rtdoa->sem);  
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
    dw1000_rtdoa_instance_t * rtdoa = (dw1000_rtdoa_instance_t *)cbs->inst_ptr;
    int64_t new_timeout;
    dw1000_ccp_instance_t *ccp = rtdoa->ccp;
    wcs_instance_t * wcs = ccp->wcs;

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
            RTDOA_STATS_INC(rtdoa_request);
            
            rtdoa_frame_t * frame = (rtdoa_frame_t *) rtdoa->frames[(++rtdoa->idx)%rtdoa->nframes];
            rtdoa->req_frame = frame;
            memcpy(frame->array, inst->rxbuf, sizeof(rtdoa_request_frame_t));
            memcpy(&frame->diag, &inst->rxdiag, sizeof(dw1000_dev_rxdiag_t));
            
            /* Deliberately in local timeframe */
            frame->rx_timestamp = inst->rxtimestamp;
            
            /* Compensate for relays */
            uint32_t repeat_dly = 0;
            if (frame->rpt_count != 0) {
                RTDOA_STATS_INC(rx_relayed);
                repeat_dly = frame->rpt_count*rtdoa->config.tx_holdoff_delay;
                frame->rx_timestamp -= (repeat_dly << 16)*(1.0l - wcs->skew);
            }

            /* A good rtdoa_req packet has been received, stop the receiver */
            dw1000_stop_rx(inst);
            /* Adjust timeout and delayed start to match when the responses will arrive */
            uint64_t dx_time = inst->rxtimestamp - repeat_dly;
            dx_time += (rtdoa_usecs_to_response(inst, (rtdoa_request_frame_t*)rtdoa->req_frame, 0, &rtdoa->config,
                            dw1000_phy_frame_duration(&inst->attrib, sizeof(rtdoa_response_frame_t))) << 16);

            /* Subtract the preamble time */
            dx_time -= dw1000_phy_SHR_duration(&inst->attrib);
            dw1000_set_delay_start(inst, dx_time);
            if(dw1000_start_rx(inst).start_rx_error){
                os_sem_release(&rtdoa->sem);
                RTDOA_STATS_INC(start_rx_error);
            }

            /* Set new timeout */
            new_timeout = ((int64_t)rtdoa->timeout - (int64_t)inst->rxtimestamp) >> 16;
            if (new_timeout < 1) new_timeout = 1;
            dw1000_set_rx_timeout(inst, (uint16_t)new_timeout);
            /* Early return as we don't need to adjust timeout again */
            return true;
            break;
        }
        case DWT_RTDOA_RESP:
        {
            // The packets following the initial request from all nodes
            if (inst->frame_len < sizeof(rtdoa_response_frame_t)) {
                break;
            }
            RTDOA_STATS_INC(rtdoa_response);
            
            rtdoa_frame_t * frame = (rtdoa_frame_t *) rtdoa->frames[(++rtdoa->idx)%rtdoa->nframes];
            memcpy(frame->array, inst->rxbuf, sizeof(rtdoa_request_frame_t));
            memcpy(&frame->diag, &inst->rxdiag, sizeof(dw1000_dev_rxdiag_t));
            frame->rx_timestamp = inst->rxtimestamp;
            break; 
        }
        default:
            return false;
            break;
    }

    /* Adjust existing timeout instead of resetting it (faster) */
    new_timeout = ((int64_t)rtdoa->timeout - (int64_t)inst->rxtimestamp) >> 16;
    if (new_timeout < 1) new_timeout = 1;
    dw1000_write_reg(inst, RX_FWTO_ID, RX_FWTO_OFFSET, (uint16_t)new_timeout, sizeof(uint16_t));
    return true;
}

