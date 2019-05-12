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
#include <rtdoa/rtdoa.h>
#include <rng/rng.h>
#if MYNEWT_VAL(WCS_ENABLED)
#include <wcs/wcs.h>
#endif
#if MYNEWT_VAL(CCP_ENABLED)
#include <ccp/ccp.h>
#endif
#include <rng/slots.h>

static dw1000_rng_config_t g_config = {
    .tx_holdoff_delay = MYNEWT_VAL(RTDOA_TX_HOLDOFF),         // Send Time delay in usec.
    .rx_timeout_delay = MYNEWT_VAL(RTDOA_RX_TIMEOUT),       // Receive response timeout in usec
    .tx_guard_delay = MYNEWT_VAL(RTDOA_TX_GUARD_DELAY)
};

#if MYNEWT_VAL(RTDOA_STATS)
STATS_NAME_START(rtdoa_stat_section)
    STATS_NAME(rtdoa_stat_section, rtdoa_request)
    STATS_NAME(rtdoa_stat_section, rtdoa_response)
    STATS_NAME(rtdoa_stat_section, rtdoa_listen)
    STATS_NAME(rtdoa_stat_section, rx_complete)
    STATS_NAME(rtdoa_stat_section, rx_error)
    STATS_NAME(rtdoa_stat_section, rx_timeout)
    STATS_NAME(rtdoa_stat_section, rx_relayed)
    STATS_NAME(rtdoa_stat_section, tx_relay_error)
    STATS_NAME(rtdoa_stat_section, tx_relay_ok)
    STATS_NAME(rtdoa_stat_section, start_rx_error)
    STATS_NAME(rtdoa_stat_section, rx_unsolicited)
    STATS_NAME(rtdoa_stat_section, start_tx_error)
    STATS_NAME(rtdoa_stat_section, reset)
STATS_NAME_END(rtdoa_stat_section)
#endif

dw1000_rtdoa_instance_t *
dw1000_rtdoa_init(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config, uint16_t nframes)
{
    assert(inst);

    if (inst->rtdoa == NULL ) {
        inst->rtdoa = (dw1000_rtdoa_instance_t*) malloc(sizeof(dw1000_rtdoa_instance_t) + nframes * sizeof(rtdoa_frame_t * )); 
        assert(inst->rtdoa);
        memset(inst->rtdoa, 0, sizeof(dw1000_rtdoa_instance_t));
        inst->rtdoa->status.selfmalloc = 1;
    }
    os_error_t err = os_sem_init(&inst->rtdoa->sem, 0x1); 
    assert(err == OS_OK);

    dw1000_rtdoa_instance_t * rtdoa = inst->rtdoa; // Updating the Global Instance of rtdoa
    rtdoa->parent = inst;
    rtdoa->nframes = nframes;
    rtdoa->idx = 0xFFFF;
    rtdoa->seq_num = 0;
    
    if (config != NULL ){
        dw1000_rtdoa_config(inst, config);
    }

#if MYNEWT_VAL(RTDOA_STATS)
    int rc = stats_init(
                    STATS_HDR(inst->rtdoa->stat),
                    STATS_SIZE_INIT_PARMS(inst->rtdoa->stat, STATS_SIZE_32),
                    STATS_NAME_INIT_PARMS(rtdoa_stat_section)
            );
   
#if  MYNEWT_VAL(DW1000_DEVICE_0) && !MYNEWT_VAL(DW1000_DEVICE_1)
        rc |= stats_register("rtdoa", STATS_HDR(inst->rtdoa->stat));
#elif  MYNEWT_VAL(DW1000_DEVICE_0) && MYNEWT_VAL(DW1000_DEVICE_1)
    if (inst == hal_dw1000_inst(0))
        rc |= stats_register("rtdoa0", STATS_HDR(inst->rtdoa->stat));
    else
        rc |= stats_register("rtdoa1", STATS_HDR(inst->rtdoa->stat));
#endif
    assert(rc == 0);
#endif
    return rtdoa;
}

/**
 * API to free the allocated resources.
 *
 * @param inst  Pointer to dw1000_rng_instance_t.
 *
 * @return void 
 */
void
dw1000_rtdoa_free(dw1000_rtdoa_instance_t * inst){

    assert(inst);
    if (inst->status.selfmalloc){
        for(int i =0; i< inst->nframes; i++){
            free(inst->frames[i]);
            inst->frames[i] = NULL;
        }
        free(inst);
    }
    else
        inst->status.initialized = 0;
}

inline void
dw1000_rtdoa_set_frames(dw1000_dev_instance_t * inst, uint16_t nframes)
{
    assert(inst);
    dw1000_rtdoa_instance_t * rtdoa = inst->rtdoa;
    assert(nframes <= rtdoa->nframes);
    rtdoa_frame_t default_frame = {
        .PANID = 0xDECA,
        .fctrl = FCNTL_IEEE_RANGE_16,
        .code = DWT_RTDOA_INVALID
    };
    for (uint16_t i = 0; i < nframes; i++){
        rtdoa->frames[i] = (rtdoa_frame_t * ) malloc(sizeof(rtdoa_frame_t));
        assert(rtdoa->frames[i]);
        memcpy(rtdoa->frames[i], &default_frame, sizeof(rtdoa_frame_t));
    }
}

/**
 * 
 *
 * @param inst          Pointer to dw1000_dev_instance_t. 
 * @param ranges        []] to return results  
 * @param nranges       side of  ranges[]
 * @param code          base address of curcular buffer
 *
 * @return valid mask
 */
uint32_t
dw1000_rtdoa_get_ranges(dw1000_dev_instance_t * inst, float ranges[], uint16_t nranges, uint16_t base)
{
    uint32_t mask = 0;
    return mask;
}


/**
 * API to assign the config parameters to range instance.
 *
 * @param inst    Pointer to dw1000_dev_instance_t. 
 * @param config  Pointer to dw1000_rng_config_t.
 *
 * @return dw1000_dev_status_t 
 */
dw1000_dev_status_t
dw1000_rtdoa_config(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config){
    assert(inst);
    assert(config);

    memcpy(&inst->rtdoa->config, config, sizeof(dw1000_rng_config_t));
    return inst->status;
}

void rtdoa_pkg_init(void){
#if MYNEWT_VAL(DW1000_PKG_INIT_LOG)
    printf("{\"utime\": %lu,\"msg\": \"rtdoa_pkg_init\"}\n", os_cputime_ticks_to_usecs(os_cputime_get32()));
#endif

#if MYNEWT_VAL(DW1000_DEVICE_0)
    dw1000_rtdoa_init(hal_dw1000_inst(0), &g_config, MYNEWT_VAL(RTDOA_NFRAMES));
    dw1000_rtdoa_set_frames(hal_dw1000_inst(0), MYNEWT_VAL(RTDOA_NFRAMES));
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    dw1000_rtdoa_init(hal_dw1000_inst(1), &g_config, MYNEWT_VAL(RTDOA_NFRAMES));
    dw1000_rtdoa_set_frames(hal_dw1000_inst(1), MYNEWT_VAL(RTDOA_NFRAMES));
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    dw1000_rtdoa_init(hal_dw1000_inst(2), &g_config, MYNEWT_VAL(RTDOA_NFRAMES));
    dw1000_rtdoa_set_frames(hal_dw1000_inst(2), MYNEWT_VAL(RTDOA_NFRAMES));
#endif

}

/** 
 * Help function to calculate the delay between cascading requests
 * 
 * @param inst Pointer to dw1000_dev_instance_t * 
 * @param slot_id 0 for master, and increasing
 * @return void
 */
uint32_t
rtdoa_usecs_to_response(dw1000_dev_instance_t * inst, rtdoa_request_frame_t * req,
                        uint16_t nslots, dw1000_rng_config_t * config, uint32_t duration)
{    
    uint32_t ret = 0;
    /* Repeat part */
    ret += (req->rpt_max - req->rpt_count+1)*config->tx_holdoff_delay;

    /* Response part */
    // ret += nslots * ( duration + (uint32_t) dw1000_dwt_usecs_to_usecs(config->tx_guard_delay));
    ret += nslots * ( duration + (uint32_t)config->tx_guard_delay );
    return ret;
}

/* TODO: Move to node_rtdoa? */
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
    frame->slot_modulus = 16;   /* XXX generalise / config / mynewt_val */
    frame->rpt_count = 0;
    frame->rpt_max = 4;

    dw1000_set_delay_start(inst, delay);
#if MYNEWT_VAL(WCS_ENABLED)       
    /* Another node is clock master - calculate tx-time using wcs */
    wcs_instance_t * wcs = inst->ccp->wcs;  
    frame->tx_timestamp = (wcs_local_to_master(wcs, delay) & 0xFFFFFFFFFFFFFE00ULL);
#else
    /* Local node is clock master - easy to calculate the tx-time */
    frame->tx_timestamp = inst->ccp->master_epoch.timestamp & 0xFFFFFF0000000000ULL;
    frame->tx_timestamp|= (delay& 0xFFFFFFFFFFFFFE00ULL);
#endif
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


/**
 * API to listen as a slave node
 *
 * @param inst          Pointer to dw1000_dev_instance_t.
 *
 * @return dw1000_dev_status_t 
 */
dw1000_dev_status_t 
dw1000_rtdoa_listen(dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode, uint64_t delay, uint16_t timeout)
{
    assert(inst->rtdoa);
    dw1000_rtdoa_instance_t * rtdoa = inst->rtdoa;

    os_error_t err = os_sem_pend(&rtdoa->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    /* Setup start time and overall timeout */
    dw1000_set_delay_start(inst, delay);
    dw1000_set_rx_timeout(inst, timeout);
    rtdoa->timeout = (delay + (((uint64_t)timeout)<<16))&0xFFFFFFFFFFUL;

    RTDOA_STATS_INC(rtdoa_listen);
    if(dw1000_start_rx(inst).start_rx_error){
        err = os_sem_release(&rtdoa->sem);
        assert(err == OS_OK);
        RTDOA_STATS_INC(start_rx_error);
    }
    if (mode == DWT_BLOCKING){
        err = os_sem_pend(&rtdoa->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions 
        assert(err == OS_OK);
        err = os_sem_release(&rtdoa->sem);
        assert(err == OS_OK);
    }
   return inst->status;
}


float
dw1000_rtdoa_tdoa_between_frames(struct _dw1000_dev_instance_t * inst, rtdoa_frame_t *first_frame, rtdoa_frame_t *final_frame)
{
#if 0
    float ToF = 0;
    uint64_t T1R, T1r, T2R, T2r;
    int64_t nom,denom;

    switch(final_frame->code){
    case DWT_RTDOA_REQUEST ... DWT_RTDOA_RESP: {
            assert(first_frame != NULL);
            ToF = ((first_frame->response_timestamp - first_frame->request_timestamp)
                    -  (first_frame->transmission_timestamp - first_frame->reception_timestamp))/2.0f;
            break;
            }
        default: break;
    }
    return ToF;
#endif
    return 0.0f;
}

