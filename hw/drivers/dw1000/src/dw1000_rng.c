/**
 * Copyright (C) 2017-2018, Decawave Limited, All Rights Reserved
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
#include <dw1000/dw1000_ftypes.h>
#include <dw1000/dw1000_rng.h>

static void rng_tx_complete_cb(dw1000_dev_instance_t * inst);
static void rng_rx_complete_cb(dw1000_dev_instance_t * inst);
static void rng_rx_timeout_cb(dw1000_dev_instance_t * inst);
static void rng_rx_error_cb(dw1000_dev_instance_t * inst);

dw1000_rng_instance_t * dw1000_rng_init(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config){

    assert(inst);
    if (inst->rng == NULL ) {
        inst->rng = (dw1000_rng_instance_t *) malloc(sizeof(dw1000_rng_instance_t));
        assert(inst->rng);
        memset(inst->rng, 0, sizeof(dw1000_rng_instance_t));
        inst->rng->status.selfmalloc = 1;
    }
    os_error_t err = os_sem_init(&inst->rng->sem, 0x1); 
    assert(err == OS_OK);

    if (config != NULL ){
        inst->rng->config = config;
        dw1000_rng_config(inst, config);
    }

    dw1000_rng_set_callbacks(inst, rng_tx_complete_cb, rng_rx_complete_cb, rng_rx_timeout_cb, rng_rx_error_cb);

    inst->rng->status.initialized = 1;

    return inst->rng;
}

void dw1000_rng_free(dw1000_rng_instance_t * inst){

    assert(inst);  
    if (inst->status.selfmalloc)
        free(inst);
    else
        inst->status.initialized = 0;
}

void dw1000_rng_set_callbacks(dw1000_dev_instance_t * inst,  dw1000_dev_cb_t rng_tx_complete_cb, dw1000_dev_cb_t rng_rx_complete_cb,  dw1000_dev_cb_t rng_rx_timeout_cb,  dw1000_dev_cb_t rng_rx_error_cb){
    inst->rng_tx_complete_cb = rng_tx_complete_cb;
    inst->rng_rx_complete_cb = rng_rx_complete_cb;
    inst->rng_rx_timeout_cb = rng_rx_timeout_cb;
    inst->rng_rx_error_cb = rng_rx_error_cb;
}

inline void dw1000_rng_set_frames(dw1000_dev_instance_t * inst, ss_twr_frame_t * transaction){
    if (transaction != NULL) 
        inst->rng->ss_twr = transaction;
}

dw1000_dev_status_t dw1000_rng_config(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config){

    assert(inst);
    assert(config);

    inst->rng->config = config;
    dw1000_set_wait4resp_delay(inst, config->wait4resp_delay);
    dw1000_set_rx_timeout(inst, config->rx_timeout_period);

   return inst->status;
}


dw1000_dev_status_t dw1000_rng_request(dw1000_dev_instance_t * inst, uint16_t dst_address, dw1000_rng_modes_t protocal){

    /* Semaphore lock for multi-threaded applications */
    os_error_t err = os_sem_pend(&inst->rng->sem, OS_TIMEOUT_NEVER);
    assert(err == OS_OK);
    
    ss_twr_frame_t * ss_twr  = inst->rng->ss_twr;    
    dw1000_rng_config_t * config = inst->rng->config;

    ss_twr->request.seq_num++;
    ss_twr->request.code = DWT_SS_TWR;
    ss_twr->request.src_address = inst->my_short_address;
    ss_twr->request.dst_address = dst_address;

    dw1000_write_tx(inst, (uint8_t *) & ss_twr->request, 0, sizeof(ieee_rng_request_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_request_frame_t), 0, true); 
    dw1000_set_wait4resp(inst, true, config->wait4resp_delay, config->rx_timeout_period);
    dw1000_start_tx(inst);
    
    err = os_sem_pend(&inst->rng->sem, 1000); // Wait for completion of transactions units os_clicks
    inst->status.range_request_timeout = (err == OS_TIMEOUT);
    os_sem_release(&inst->rng->sem);

    if (inst->status.start_tx_error || inst->status.rx_error || inst->status.range_request_timeout ||  inst->status.rx_timeout_error)
        ss_twr->request.seq_num--;

   return inst->status;
}

static void rng_tx_complete_cb(dw1000_dev_instance_t * inst){

   // Unblock Semaphore after last transmission
   ss_twr_frame_t * ss_twr  = inst->rng->ss_twr; 
   if (ss_twr->response.src_address != inst->my_short_address) 
        return;
   if (ss_twr->response.code == DWT_SS_TWR_FINAL || ss_twr->response.code == DWT_SDS_TWR_T2) 
        os_sem_release(&inst->rng->sem);  
}

static void rng_rx_complete_cb(dw1000_dev_instance_t * inst){

    hal_gpio_toggle(LED_1);

    uint16_t code, dst_address; 
    ss_twr_frame_t * ss_twr  = inst->rng->ss_twr;    
    dw1000_rng_config_t * config = inst->rng->config;
    uint64_t request_timestamp, response_tx_delay, response_timestamp;
  

    if (inst->fctrl == 0x8841){ 
        dw1000_read_rx(inst, (uint8_t *) &code, offsetof(ieee_rng_request_frame_t,code), sizeof(uint16_t));
        dw1000_read_rx(inst, (uint8_t *) &dst_address, offsetof(ieee_rng_request_frame_t,dst_address), sizeof(uint16_t));    
    }else{
        printf("unknown ranging frame type\n");
        return;
    }
   printf("dst_address = %X my_short_address = %X\n", dst_address, inst->my_short_address);
    if (dst_address != inst->my_short_address) 
        return;
    
    switch (code){
        case DWT_SS_TWR:

            if (inst->frame_len <= sizeof(ieee_rng_request_frame_t))
                dw1000_read_rx(inst, (uint8_t *) &ss_twr->request, 0, sizeof(ieee_rng_request_frame_t));
            else 
                break; 
            /* 
            *  We need to send final timestamp within the response frame. Timestamps and delayed transmission times are both 
            *  expressed in device time units (DTU). We need to add the desired response delay to the response timestamp to derive 
            *  the final transmission time. The response_tx_delay time resolution is 512 device time units which means that 
            *  the lower 9 bits of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit 
            *  words by shifting the all-zero lower 8 bits.
            * 
            *  Note: System time and timestamps are designed to be based on the time units which are nominally at 64 GHz, 
            *  or more precisely 499.2 MHz × 128 which is 63.8976 GHz. In line with this when the DW1000 is in idle mode with 
            *  the digital PLL enabled, the System Time Counter is incremented at a rate of 125 MHz in units of 512.
            *  The nine low-order bits of SYS_TIME register are thus always zero. 
            *  The counter wrap period of the clock is then: 2^40 / (128×499.2e6) = 17.2074 seconds.
            */

            // request_timestamp, response_tx_delay & response_timestamp are all (DTU)
            // wait4resp_delay is in (usec)
            // 1 (usec) = 1 << 9 (DTU) 
            
            request_timestamp = dw1000_read_rxtime(inst);  
            response_tx_delay = request_timestamp + ((uint64_t)config->wait4resp_delay << 15); 
            response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + (inst->tx_antenna_delay << 2);
  
            ss_twr->response.reception_timestamp = request_timestamp;
            ss_twr->response.transmission_timestamp = response_timestamp;
            ss_twr->response.dst_address = ss_twr->request.src_address;
            ss_twr->response.src_address = inst->my_short_address;
            ss_twr->response.code = DWT_SS_TWR_T1;

            dw1000_write_tx(inst, (uint8_t *)&ss_twr->response, 0, sizeof(ieee_rng_response_frame_t));
            dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_response_frame_t), 0, true); 
            dw1000_set_wait4resp(inst, true, config->wait4resp_delay, config->rx_timeout_period); // switch in received frame now

            if (dw1000_start_tx_delayed(inst, response_tx_delay).start_tx_error)
                os_sem_release(&inst->rng->sem);  
            break;

        case DWT_SS_TWR_T1:

            if (inst->frame_len <= sizeof(ieee_rng_response_frame_t))
                dw1000_read_rx(inst,  (uint8_t *) &ss_twr->response, 0, sizeof(ieee_rng_response_frame_t));
            else 
                break;

            ss_twr->request_timestamp = dw1000_read_txtime_lo(inst);   // This corresponds to when the original request was actually sent
            ss_twr->response_timestamp = dw1000_read_rxtime_lo(inst);  // This corresponds to the response just received            
            ss_twr->response.code = DWT_SS_TWR_FINAL;
            ss_twr->response.dst_address = ss_twr->response.src_address;
            ss_twr->response.src_address = inst->my_short_address;

            request_timestamp = dw1000_read_rxtime(inst);  
            response_tx_delay = request_timestamp + ((uint64_t)config->wait4resp_delay << 15); 

            // Transmit timestamp final report
            dw1000_write_tx(inst, (uint8_t *)&ss_twr, 0, sizeof(ss_twr_frame_t));
            dw1000_write_tx_fctrl(inst, sizeof(ss_twr_frame_t), 0, true); 
            dw1000_set_wait4resp(inst, false, config->wait4resp_delay, config->rx_timeout_period); // switch in received frame now
            //dw1000_start_tx(inst);
            if (dw1000_start_tx_delayed(inst, response_tx_delay).start_tx_error)
                os_sem_release(&inst->rng->sem);  
            break;

        case  DWT_SS_TWR_FINAL:
         if (inst->frame_len <= sizeof(ss_twr_frame_t))
                dw1000_read_rx(inst,  (uint8_t *) &ss_twr, 0, sizeof(ss_twr_frame_t));

        os_sem_release(&inst->rng->sem);
        break;

        default: break;
    }  
}

static void rng_rx_timeout_cb(dw1000_dev_instance_t * inst){
    os_error_t err = os_sem_release(&inst->rng->sem);
    assert(err == OS_OK);
}

static void rng_rx_error_cb(dw1000_dev_instance_t * inst){
    os_error_t err = os_sem_release(&inst->rng->sem);
    assert(err == OS_OK);
}
