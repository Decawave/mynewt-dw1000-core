/**
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
#include <dw1000/dw1000_ftypes.h>
#include <dw1000/dw1000_rng.h>

#ifndef SS_TWR_ENABLE
#define SS_TWR_ENABLE
#endif

#ifndef DS_TWR_ENABLE
#define DS_TWR_ENABLE
#endif

static void rng_tx_complete_cb(dw1000_dev_instance_t * inst);
static void rng_rx_complete_cb(dw1000_dev_instance_t * inst);
static void rng_rx_timeout_cb(dw1000_dev_instance_t * inst);
static void rng_rx_error_cb(dw1000_dev_instance_t * inst);

dw1000_rng_instance_t * 
dw1000_rng_init(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config){
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
    inst->rng_tx_final_cb = NULL;
    inst->rng->status.initialized = 1;
    return inst->rng;
}

void 
dw1000_rng_free(dw1000_rng_instance_t * inst){
    assert(inst);  
    if (inst->status.selfmalloc)
        free(inst);
    else
        inst->status.initialized = 0;
}

void 
dw1000_rng_set_callbacks(dw1000_dev_instance_t * inst,  dw1000_dev_cb_t rng_tx_complete_cb, dw1000_dev_cb_t rng_rx_complete_cb,  dw1000_dev_cb_t rng_rx_timeout_cb,  dw1000_dev_cb_t rng_rx_error_cb){
    inst->rng_tx_complete_cb = rng_tx_complete_cb;
    inst->rng_rx_complete_cb = rng_rx_complete_cb;
    inst->rng_rx_timeout_cb = rng_rx_timeout_cb;
    inst->rng_rx_error_cb = rng_rx_error_cb;
}

inline void 
dw1000_rng_set_frames(dw1000_dev_instance_t * inst, twr_frame_t ss_twr[], uint16_t nframes){
    if (ss_twr != NULL)
    {
        inst->rng->nframes = nframes;
        for (uint16_t i = 0; i < nframes; i++)
            inst->rng->twr[i] = ss_twr[i];
    }
}

dw1000_dev_status_t 
dw1000_rng_config(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config){
    assert(inst);
    assert(config);

    inst->rng->config = config;
    return inst->status;
}


dw1000_dev_status_t 
dw1000_rng_request(dw1000_dev_instance_t * inst, uint16_t dst_address, dw1000_rng_modes_t code){

    // This function executes on the device that initiates a request 
    /* Semaphore lock for multi-threaded applications */
    os_error_t err = os_sem_pend(&inst->rng->sem, OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    for (uint16_t i = 0; i < inst->rng->nframes; i++)
        inst->rng->twr[i].request.code = DWT_TWR_INVALID;
    
    twr_frame_t * twr  = &inst->rng->twr[0];    
    dw1000_rng_config_t * config = inst->rng->config;

    twr->request.seq_num++;
    twr->request.code = code;
    twr->request.src_address = inst->my_short_address;
    twr->request.dst_address = dst_address;

    dw1000_write_tx(inst, (uint8_t *) & twr->request, 0, sizeof(ieee_rng_request_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_request_frame_t), 0, true);     
    dw1000_set_wait4resp(inst, true);    
    dw1000_set_rx_timeout(inst, config->rx_timeout_period); 
    dw1000_start_tx(inst);
    
    err = os_sem_pend(&inst->rng->sem, 10000); // Wait for completion of transactions units os_clicks
    inst->status.request_timeout = (err == OS_TIMEOUT);
    os_sem_release(&inst->rng->sem);
    
    if (inst->status.start_tx_error || inst->status.rx_error || inst->status.request_timeout ||  inst->status.rx_timeout_error)
        twr->request.seq_num--;

   return inst->status;
}

static void 
rng_tx_complete_cb(dw1000_dev_instance_t * inst)
{
  //  printf("inst->rng->nframes=%d %d %d\n", inst->rng->nframes, inst->rng->twr[0].response.code , inst->rng->twr[1].response.code);
   // Unblock Semaphore after last transmission
   if (inst->rng->twr[0].response.code == DWT_SS_TWR_FINAL || inst->rng->twr[0].response.code == DWT_SS_TWR_T1){
        os_sem_release(&inst->rng->sem);  
   }
#ifdef  DS_TWR_ENABLE
    else{ 
        if(inst->rng->nframes > 1) 
            if (inst->rng->twr[1].response.code ==  DWT_DS_TWR_FINAL){
                os_sem_release(&inst->rng->sem);  
            }
    }
#endif
}

static void 
rng_rx_complete_cb(dw1000_dev_instance_t * inst)
{
    hal_gpio_toggle(LED_1);
    uint16_t code, dst_address; 
    dw1000_rng_config_t * config = inst->rng->config;

    if (inst->fctrl == 0x8841){ 
        dw1000_read_rx(inst, (uint8_t *) &code, offsetof(ieee_rng_request_frame_t,code), sizeof(uint16_t));
        dw1000_read_rx(inst, (uint8_t *) &dst_address, offsetof(ieee_rng_request_frame_t,dst_address), sizeof(uint16_t));    
    }else{
        return;
    }
    if (dst_address != inst->my_short_address)
        return;

    switch (code){
#ifdef SS_TWR_ENABLE
        case DWT_SS_TWR ... DWT_SS_TWR_FINAL:
            switch(code){
                case DWT_SS_TWR:
                    {
                        // This code executes on the device that is responding to a request
                        // printf("DWT_SS_TWR\n");
                        twr_frame_t * twr = &inst->rng->twr[0];
                        if (inst->frame_len <= sizeof(ieee_rng_request_frame_t))
                            dw1000_read_rx(inst, (uint8_t *) &twr->request, 0, sizeof(ieee_rng_request_frame_t));
                        else 
                            break; 
                    
                        uint64_t request_timestamp = dw1000_read_rxtime(inst);  
                        uint64_t response_tx_delay = request_timestamp + ((uint64_t)config->tx_holdoff_delay << 15); 
                        uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;
        
                        twr->response.reception_timestamp = request_timestamp;
                        twr->response.transmission_timestamp = response_timestamp;
                        twr->response.dst_address = twr->request.src_address;
                        twr->response.src_address = inst->my_short_address;
                        twr->response.code = DWT_SS_TWR_T1;

                        dw1000_write_tx(inst, (uint8_t *)&twr->response, 0, sizeof(ieee_rng_response_frame_t));
                        dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_response_frame_t), 0, true); 
                        dw1000_set_wait4resp(inst, true);    
                        dw1000_set_delay_start(inst, response_tx_delay);
                        dw1000_set_rx_timeout(inst, config->rx_timeout_period); 

                        if (dw1000_start_tx(inst).start_tx_error)
                            os_sem_release(&inst->rng->sem);  
                        break;
                    }
                case DWT_SS_TWR_T1:
                    {
                        // This code executes on the device that initiated a request, and is now preparing the final timestamps
                        // printf("DWT_SS_TWR_T1\n");
                        twr_frame_t * twr = &inst->rng->twr[0];
                        if (inst->frame_len <= sizeof(ieee_rng_response_frame_t))
                            dw1000_read_rx(inst,  (uint8_t *) &twr->response, 0, sizeof(ieee_rng_response_frame_t));
                        else 
                            break;

                        twr->request_timestamp = dw1000_read_txtime_lo(inst);   // This corresponds to when the original request was actually sent
                        twr->response_timestamp = dw1000_read_rxtime_lo(inst);  // This corresponds to the response just received            
                        twr->response.dst_address = twr->response.src_address;
                        twr->response.src_address = inst->my_short_address;
                        twr->response.code = DWT_SS_TWR_FINAL;

                        // Final callback, prior to transmission, use this callback to populate the FUSION_EXTENDED_FRAME fields.
                        if (inst->rng_tx_final_cb != NULL)
                            inst->rng_tx_final_cb(inst);
                    
                        // Transmit timestamp final report
                        dw1000_write_tx(inst, (uint8_t *) twr, 0, sizeof(twr_frame_t));
                        dw1000_write_tx_fctrl(inst, sizeof(twr_frame_t), 0, true); 

                        if (dw1000_start_tx(inst).start_tx_error)
                            os_sem_release(&inst->rng->sem);  
                        break;
                    }
                case  DWT_SS_TWR_FINAL:
                    {
                        // This code executes on the device that responded to the original request, and has now receive the response final timestamp. 
                        // This marks the completion of the single-size-two-way request. This final 4th message is perhaps optional in some applicaiton. 
                        // printf("DWT_SS_TWR_FINAL\n");

                        twr_frame_t * twr  = &inst->rng->twr[0]; 
                        if (inst->frame_len <= sizeof(twr_frame_t))
                            dw1000_read_rx(inst,  (uint8_t *) twr, 0, sizeof(twr_frame_t));
                            
                        os_sem_release(&inst->rng->sem);
                        break;
                    }
                default: 
                    printf("Unsupported SS_TWR code\n");
                    break;
             }
             break;
#endif //SS_TWR_ENABLE
#ifdef DS_TWR_ENABLE
        case DWT_DS_TWR ... DWT_DS_TWR_FINAL:
            switch(code){
                    case DWT_DS_TWR:
                        {
                            // This code executes on the device that is responding to a original request
                            // printf("DWT_DS_TWR\n");
                            twr_frame_t * twr = &inst->rng->twr[0];
                            if (inst->frame_len <= sizeof(ieee_rng_request_frame_t))
                                dw1000_read_rx(inst, (uint8_t *) &twr->request, 0, sizeof(ieee_rng_request_frame_t));
                            else 
                                break; 

                            uint64_t request_timestamp = dw1000_read_rxtime(inst);  
                            uint64_t response_tx_delay = request_timestamp + ((uint64_t)config->tx_holdoff_delay << 15); 
                            uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;
            
                            twr->response.reception_timestamp = request_timestamp;
                            twr->response.transmission_timestamp = response_timestamp;
                            twr->response.dst_address = twr->request.src_address;
                            twr->response.src_address = inst->my_short_address;
                            twr->response.code = DWT_DS_TWR_T1;

                            dw1000_write_tx(inst, (uint8_t *)&twr->response, 0, sizeof(ieee_rng_response_frame_t));
                            dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_response_frame_t), 0, true); 
                            dw1000_set_wait4resp(inst, true);    
                            dw1000_set_delay_start(inst, response_tx_delay);   
                            dw1000_set_rx_timeout(inst, config->rx_timeout_period); 

                            if (dw1000_start_tx(inst).start_tx_error)
                                os_sem_release(&inst->rng->sem);  
                            break;
                        }
                    case DWT_DS_TWR_T1:
                        {
                            // This code executes on the device that initiated the original request, and is now preparing the next series of timestamps
                            // The 1st frame now contains a local copy of the initial first side of the double sided scheme. 
                            // printf("DWT_DS_TWR_T1\n");
                            twr_frame_t * twr = &inst->rng->twr[0];
                            if (inst->frame_len <= sizeof(ieee_rng_response_frame_t))
                                dw1000_read_rx(inst,  (uint8_t *) &inst->rng->twr[0].response, 0, sizeof(ieee_rng_response_frame_t));
                            else 
                                break;

                            twr->request_timestamp = dw1000_read_txtime_lo(inst);   // This corresponds to when the original request was actually sent
                            twr->response_timestamp = dw1000_read_rxtime_lo(inst);  // This corresponds to the response just received      
                            
                            // Note:: Switching from 1st to 2nd ss_twr frame    
                            twr = &inst->rng->twr[1];
                            twr->response.dst_address = inst->rng->twr[0].response.src_address;
                            twr->response.src_address = inst->my_short_address;
                            twr->response.seq_num = inst->rng->twr[0].response.seq_num;
                            twr->response.code = DWT_DS_TWR_T2;

                            uint64_t request_timestamp = dw1000_read_rxtime(inst);  
                            uint64_t response_tx_delay = request_timestamp + ((uint64_t)config->tx_holdoff_delay << 15); 
                            uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;
                            
                            twr->response.reception_timestamp = request_timestamp;
                            twr->response.transmission_timestamp = response_timestamp;

                            // Final callback, prior to transmission, use this callback to populate the FUSION_EXTENDED_FRAME fields.
                            if (inst->rng_tx_final_cb != NULL)
                                inst->rng_tx_final_cb(inst);

                            dw1000_write_tx(inst, (uint8_t *) twr, 0, sizeof(twr_frame_t));
                            dw1000_write_tx_fctrl(inst, sizeof(twr_frame_t), 0, true); 
                            dw1000_set_wait4resp(inst, true);    
                            dw1000_set_delay_start(inst, response_tx_delay);   
                            dw1000_set_rx_timeout(inst, config->rx_timeout_period); 
                        
                            if (dw1000_start_tx(inst).start_tx_error)
                                os_sem_release(&inst->rng->sem);  
                            break; 
                        }

                    case DWT_DS_TWR_T2:
                        {
                            // This code executes on the device that responded to the original request, and is now preparing the final timestamps
                            // printf("DWT_SDS_TWR_T2\n");
                            twr_frame_t * twr = &inst->rng->twr[1];
                            if (inst->frame_len <= sizeof(twr_frame_t))
                                dw1000_read_rx(inst,  (uint8_t *) twr, 0, sizeof(twr_frame_t));
                            else 
                                break;

                            twr->request_timestamp = dw1000_read_txtime_lo(inst);   // This corresponds to when the original request was actually sent
                            twr->response_timestamp = dw1000_read_rxtime_lo(inst);  // This corresponds to the response just received            
                            twr->response.dst_address = twr->response.src_address;
                            twr->response.src_address = inst->my_short_address;
                            twr->response.code = DWT_DS_TWR_FINAL;
                    
                            // Final callback, prior to transmission, use this callback to populate the FUSION_EXTENDED_FRAME fields.
                            if (inst->rng_tx_final_cb != NULL)
                                inst->rng_tx_final_cb(inst);

                            // Transmit timestamp final report
                            dw1000_write_tx(inst, (uint8_t *) twr, 0, sizeof(twr_frame_t));
                            dw1000_write_tx_fctrl(inst, sizeof(twr_frame_t), 0, true); 

                            if (dw1000_start_tx(inst).start_tx_error)
                                os_sem_release(&inst->rng->sem);  
                            break;
                        }
                    case  DWT_DS_TWR_FINAL:
                        {
                            // This code executes on the device that initialed the original request, and has now receive the final response timestamp. 
                            // This marks the completion of the double-single-two-way request. 
                            // printf("DWT_SDS_TWR_FINAL\n");
                            twr_frame_t * twr = &inst->rng->twr[1];
                            if (inst->frame_len <= sizeof(twr_frame_t))
                                dw1000_read_rx(inst,  (uint8_t *) twr, 0, sizeof(twr_frame_t));

                            os_sem_release(&inst->rng->sem);
                            break;
                        }
                    default: 
                        printf("Unsupported DS_TWR code\n");
                        break;
                }
            break;
#endif //DS_TWR_ENABLE
        default: 
        break;
    }  
}

static void 
rng_rx_timeout_cb(dw1000_dev_instance_t * inst){
    os_error_t err = os_sem_release(&inst->rng->sem);
    assert(err == OS_OK);
}

static void 
rng_rx_error_cb(dw1000_dev_instance_t * inst){
    os_error_t err = os_sem_release(&inst->rng->sem);
    assert(err == OS_OK);
}
