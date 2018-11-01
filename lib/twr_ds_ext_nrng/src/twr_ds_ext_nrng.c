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
#include <rng/rng.h>
#include <dsp/polyval.h>

//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

static bool rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);
static bool rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);
static bool rx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);
static bool reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);
static bool tx_final_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t *cbs);
static void send_final_msg(dw1000_dev_instance_t * inst , nrng_frame_t * frame);

static dw1000_mac_interface_t g_cbs = {
            .id = DW1000_NRNG_DS_EXT,
            .rx_complete_cb = rx_complete_cb,
            .rx_timeout_cb = rx_timeout_cb,
            .rx_error_cb = rx_error_cb,
            .reset_cb = reset_cb,
            .final_cb = tx_final_cb,
};


static dw1000_rng_config_t g_config = {
    .tx_holdoff_delay = MYNEWT_VAL(TWR_DS_EXT_NRNG_TX_HOLDOFF),         // Send Time delay in usec.
    .rx_timeout_period = MYNEWT_VAL(TWR_DS_EXT_NRNG_RX_TIMEOUT),        // Receive response timeout in usec
    .tx_guard_delay = MYNEWT_VAL(TWR_DS_EXT_NRNG_TX_GUARD_DELAY)        // Guard delay to be added between each frame from node
};
/**
 * API to initialise the rng_ss package.
 *
 *
 * @return void
 */

void twr_ds_ext_nrng_pkg_init(void){

    printf("{\"utime\": %lu,\"msg\": \"twr_ds_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
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
twr_ds_ext_nrng_free(dw1000_dev_instance_t * inst){
    assert(inst); 
    dw1000_mac_remove_interface(inst, DW1000_NRNG_DS_EXT);
}

/**
 * API for get local config callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
dw1000_rng_config_t * 
twr_ds_ext_nrng_config(dw1000_dev_instance_t * inst){
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
    dw1000_nrng_instance_t * nrng = inst->nrng;
    if(nrng->device_type == DWT_NRNG_INITIATOR){ // only if the device is an initiator
        if(nrng->resp_count && nrng->t1_final_flag){
            nrng_frame_t * frame = nrng->frames[nrng->idx][SECOND_FRAME_IDX];
            send_final_msg(inst,frame);
        }else{
            os_error_t err = os_sem_release(&nrng->sem);
            assert(err == OS_OK);
            nrng->resp_count = 0;
        }
    }else{
        os_error_t err = os_sem_release(&nrng->sem);
        assert(err == OS_OK);
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
 * API for reset_cb of rng interface
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 * @return true on sucess
 */
static bool
reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    printf("Inside %s in %s \n",__FILE__, __func__);

    if(os_sem_get_count(&inst->nrng->sem) == 0){
        os_error_t err = os_sem_release(&inst->nrng->sem);  
        assert(err == OS_OK);
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
    if(inst->fctrl != FCNTL_IEEE_N_RANGES_16){
        return false;
    }
    assert(inst->nrng);
    dw1000_nrng_instance_t * nrng = inst->nrng;
    uint16_t code, dst_address;
    dw1000_rng_config_t * config = dw1000_nrng_get_config(inst, DWT_DS_TWR_NRNG_EXT);
    dw1000_dev_control_t control = inst->control_rx_context;
    dw1000_read_rx(inst, (uint8_t *) &code, offsetof(nrng_request_frame_t,code), sizeof(uint16_t));
    dw1000_read_rx(inst, (uint8_t *) &dst_address, offsetof(nrng_request_frame_t,dst_address), sizeof(uint16_t));
    // For initiator: Only Allow the packets with dst_address matching with device my_short_address.
    // For responder: Only Allow the packets with dst_address matching with device my_short_address/Broadcast address.
    if (dst_address != inst->my_short_address && (dst_address != BROADCAST_ADDRESS || nrng->device_type == DWT_NRNG_INITIATOR) ){
        inst->control = inst->control_rx_context;
        dw1000_restart_rx(inst, control);
        return true;
    }
    switch (code){
        case DWT_DS_TWR_NRNG_EXT ... DWT_DS_TWR_NRNG_EXT_FINAL:
            switch(code){
                case DWT_DS_TWR_NRNG_EXT:
                    {
                        // This code executes on the device that is responding to a original request
                        // printf("nrng\n");
                        nrng_frame_t * frame = nrng->frames[(++nrng->idx)%(nrng->nframes/FRAMES_PER_RANGE)][FIRST_FRAME_IDX];
                        uint16_t slot_id = inst->slot_id;
                        if (inst->frame_len >= sizeof(nrng_request_frame_t))
                            dw1000_read_rx(inst, frame->array, 0, sizeof(nrng_request_frame_t));
                        else
                            break;
                        if(!(slot_id >= frame->start_slot_id && slot_id <= frame->end_slot_id))
                            break;

                        uint64_t request_timestamp = dw1000_read_rxtime(inst);
                        uint64_t response_tx_delay = request_timestamp + (((uint64_t)config->tx_holdoff_delay
                                    + (uint64_t)((slot_id-1) * ((uint64_t)config->tx_guard_delay
                                            + (dw1000_usecs_to_dwt_usecs(dw1000_phy_frame_duration(&inst->attrib, sizeof(nrng_response_frame_t)))))))<< 16);
                        uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;

                        frame->reception_timestamp =  request_timestamp;
                        frame->transmission_timestamp =  response_timestamp;

                        frame->dst_address = frame->src_address;
                        frame->src_address = inst->my_short_address;
                        frame->code = DWT_DS_TWR_NRNG_EXT_T1;
                        frame->slot_id = inst->slot_id;
                        dw1000_write_tx(inst, frame->array, 0, sizeof(nrng_response_frame_t));
                        dw1000_write_tx_fctrl(inst, sizeof(nrng_response_frame_t), 0, true);
                        dw1000_set_wait4resp(inst, true);
                        uint16_t timeout =config->tx_holdoff_delay + (frame->end_slot_id - slot_id + 1) * (dw1000_phy_frame_duration(&inst->attrib, sizeof(nrng_response_frame_t))
                                + config->rx_timeout_period
                                + config->tx_guard_delay);
                        dw1000_set_rx_timeout(inst, timeout);
                        dw1000_set_delay_start(inst, response_tx_delay);
                        if (dw1000_start_tx(inst).start_tx_error)
                            os_sem_release(&nrng->sem);
                        break;
                    }
                case DWT_DS_TWR_NRNG_EXT_T1:
                    {
                        // This code executes on the device that initiated the original request, and is now preparing the next series of timestamps
                        // The 1st frame now contains a local copy of the initial first side of the double sided scheme.
                        // printf("T1 \n");
                        nrng->resp_count++;
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
                        nrng_frame_t * next_frame = nrng->frames[idx][SECOND_FRAME_IDX];
                        memcpy(frame, &temp_frame, sizeof(nrng_response_frame_t));

                        frame->request_timestamp = next_frame->request_timestamp = dw1000_read_txtime_lo(inst);    // This corresponds to when the original request was actually sent
                        frame->response_timestamp = next_frame->response_timestamp = dw1000_read_rxtime_lo(inst);  // This corresponds to the response just received

                        uint8_t seq_num = frame->seq_num;
                        frame->dst_address = frame->src_address;
                        frame->src_address = inst->my_short_address;
                        frame->code = DWT_DS_TWR_NRNG_EXT_T2;
                        frame->seq_num = seq_num + 1;
                        // Note:: Advance to next frame
                        frame = next_frame;
                        frame->dst_address = 0xffff;
                        frame->src_address = inst->my_short_address;
                        frame->seq_num = seq_num + 1;
                        frame->code = DWT_DS_TWR_NRNG_EXT_T2;
                        frame->start_slot_id = temp_frame.start_slot_id;
                        frame->end_slot_id = temp_frame.end_slot_id;

                        uint64_t request_timestamp = dw1000_read_rxtime(inst);
                        uint64_t response_timestamp = (request_timestamp & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;
                        frame->reception_timestamp = request_timestamp;
                        frame->transmission_timestamp = response_timestamp;
                        nrng->t1_final_flag = 1;
                        if(idx == (nnodes - 1))
                        {
                            uint16_t timeout = (((nnodes) * (dw1000_usecs_to_dwt_usecs(dw1000_phy_frame_duration(&inst->attrib, sizeof(nrng_frame_t))))
                                        + (nnodes - 1)*(config->tx_guard_delay))
                                    + config->tx_holdoff_delay         // Remote side turn arroud time.
                                    + config->rx_timeout_period);
                            dw1000_set_rx_timeout(inst, timeout);

                            dw1000_write_tx(inst, frame->array, 0, sizeof(nrng_request_frame_t));
                            dw1000_write_tx_fctrl(inst, sizeof(nrng_request_frame_t), 0, true);
                            dw1000_set_wait4resp(inst, true);
                            if (dw1000_start_tx(inst).start_tx_error)
                                os_sem_release(&nrng->sem);

                            nrng->resp_count = 0;
                            nrng->t1_final_flag = 0;
                        }
                        break;
                    }
                case DWT_DS_TWR_NRNG_EXT_T2:
                    {
                        // This code executes on the device that responded to the original request, and is now preparing the final timestamps
                        // printf("T2\n"); 
                        uint16_t slot_id = inst->slot_id;
                        nrng_frame_t * frame = nrng->frames[(++nrng->idx)%(nrng->nframes/FRAMES_PER_RANGE)][SECOND_FRAME_IDX];

                        if (inst->frame_len >= sizeof(nrng_request_frame_t))
                            dw1000_read_rx(inst,  frame->array, 0, sizeof(nrng_request_frame_t));
                        else
                            break;
                        if(!(slot_id >= frame->start_slot_id && slot_id <= frame->end_slot_id))
                            break;
                        uint64_t request_timestamp = dw1000_read_rxtime(inst);
                        uint64_t response_tx_delay = request_timestamp + (((uint64_t)config->tx_holdoff_delay
                                    + (uint64_t)((inst->slot_id-1) * ((uint64_t)config->tx_guard_delay 
                                            + dw1000_usecs_to_dwt_usecs(dw1000_phy_frame_duration(&inst->attrib, sizeof(nrng_frame_t))))))<< 16);
                        frame->request_timestamp = dw1000_read_txtime_lo(inst); // This corresponds to when the original request was actually sent
                        frame->response_timestamp = dw1000_read_rxtime_lo(inst);  // This corresponds to the response just received
                        frame->dst_address = frame->src_address;
                        frame->src_address = inst->my_short_address;
                        frame->code = DWT_DS_TWR_NRNG_EXT_FINAL;
                        frame->slot_id = slot_id;
                        
                        // Final callback, prior to transmission, use this callback to populate the EXTENDED_FRAME fields.
                        if (cbs!=NULL && cbs->final_cb) 
                            cbs->final_cb(inst, cbs);

                        dw1000_write_tx(inst, frame->array, 0, sizeof(nrng_frame_t));
                        dw1000_write_tx_fctrl(inst, sizeof(nrng_frame_t), 0, true);
                        dw1000_set_wait4resp(inst, false);
                        dw1000_set_delay_start(inst, response_tx_delay);
                        if (dw1000_start_tx(inst).start_tx_error)
                            os_sem_release(&nrng->sem);
                        else{
                            os_sem_release(&nrng->sem);
                            dw1000_mac_interface_t * cbs = NULL;
                            if(!(SLIST_EMPTY(&inst->interface_cbs))){
                                SLIST_FOREACH(cbs, &inst->interface_cbs, next){
                                    if (cbs!=NULL && cbs->complete_cb)
                                        if(cbs->complete_cb(inst, cbs)) continue;
                                }
                            }
                        }
                        break;
                    }
                case  DWT_DS_TWR_NRNG_EXT_FINAL:
                    {
                        // This code executes on the device that initialed the original request, and has now receive the final response timestamp.
                        // This marks the completion of the double-single-two-way request.
                        // printf("Final\n");
                        nrng->resp_count++;
                        uint16_t nnodes = nrng->nnodes;
                        uint16_t idx = 0;
                        nrng_frame_t temp;
                        if (inst->frame_len >= sizeof(nrng_frame_t))
                            dw1000_read_rx(inst, (uint8_t *)&temp, 0, sizeof(nrng_frame_t));
                        uint16_t node_slot_id = temp.slot_id;
                        uint16_t end_slot_id = temp.end_slot_id;
                        nrng->idx = idx = node_slot_id - temp.start_slot_id;
                        if(idx < nnodes-1){
                            // At the start the device will wait for the entire nnodes to respond as a single huge timeout.
                            // When a node respond we will recalculate the remaining time to be waited for as (total_nodes - completed_nodes)*(phy_duaration + guard_delay)
                            uint16_t phy_duration = dw1000_phy_frame_duration(&inst->attrib, sizeof(nrng_frame_t));
                            uint16_t timeout = ((phy_duration + config->tx_guard_delay) * (end_slot_id - node_slot_id));
                            dw1000_set_rx_timeout(inst, timeout);
                            dw1000_start_rx(inst);
                        }
                        nrng_frame_t *frame = nrng->frames[idx][SECOND_FRAME_IDX];

                        nrng->t1_final_flag = 0;
                        frame->request_timestamp = temp.request_timestamp;
                        frame->response_timestamp = temp.response_timestamp;
                        frame->code = temp.code;
                        frame->dst_address = temp.src_address;
                        frame->transmission_timestamp = dw1000_read_txtime_lo(inst);
                        if(idx == nnodes -1){
                            os_sem_release(&nrng->sem);
                            nrng->resp_count = 0;
                            dw1000_mac_interface_t * cbs = NULL;
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
    }
    return true;
}

static void 
send_final_msg(dw1000_dev_instance_t * inst , nrng_frame_t * frame){
    //printf("final_cb\n");
    assert(inst->nrng);
    dw1000_nrng_instance_t * nrng = inst->nrng;
    dw1000_rng_config_t * config = dw1000_nrng_get_config(inst, DWT_DS_TWR_NRNG_EXT);
    uint16_t nnodes = nrng->nnodes;
    dw1000_write_tx(inst, frame->array, 0, sizeof(nrng_request_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(nrng_request_frame_t), 0, true);
    dw1000_set_wait4resp(inst, true);
    uint16_t timeout = (((nnodes) * (dw1000_usecs_to_dwt_usecs(dw1000_phy_frame_duration(&inst->attrib, sizeof(nrng_frame_t))))
            + (nnodes - 1)*(config->tx_guard_delay))
            + config->tx_holdoff_delay         // Remote side turn arroud time.
            + config->rx_timeout_period);
    dw1000_set_rx_timeout(inst, timeout);

    nrng->resp_count = 0;
    nrng->t1_final_flag = 0;

    if (dw1000_start_tx(inst).start_tx_error)
        os_sem_release(&nrng->sem);
}

static bool
tx_final_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t *cbs){
    dw1000_nrng_instance_t * nrng = inst->nrng;
    nrng_frame_t * frame = nrng->frames[nrng->idx%(nrng->nframes/FRAMES_PER_RANGE)][SECOND_FRAME_IDX];

    frame->cartesian.x = MYNEWT_VAL(LOCAL_COORDINATE_X);
    frame->cartesian.y = MYNEWT_VAL(LOCAL_COORDINATE_Y);
    frame->cartesian.z = MYNEWT_VAL(LOCAL_COORDINATE_Z);

    frame->spherical_variance.range = MYNEWT_VAL(RANGE_VARIANCE);
    frame->spherical_variance.azimuth = -1;
    frame->spherical_variance.zenith = -1;
    return true;
}

