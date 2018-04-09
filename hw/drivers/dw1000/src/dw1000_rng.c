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
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_ftypes.h>
#include <dw1000/dw1000_rng.h>


static void rng_tx_complete_cb(dw1000_dev_instance_t * inst);
static void rng_rx_complete_cb(dw1000_dev_instance_t * inst);
static void rng_rx_timeout_cb(dw1000_dev_instance_t * inst);
static void rng_rx_error_cb(dw1000_dev_instance_t * inst);
static void rng_tx_final_cb(dw1000_dev_instance_t * inst);

dw1000_rng_instance_t * 
dw1000_rng_init(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config, uint16_t nframes){

    assert(inst);
    if (inst->rng == NULL ) {
        inst->rng = (dw1000_rng_instance_t *) malloc(sizeof(dw1000_rng_instance_t) + nframes * sizeof(twr_frame_t *)); // struct + flexible array member
        assert(inst->rng);
        memset(inst->rng, 0, sizeof(dw1000_rng_instance_t));
        inst->rng->status.selfmalloc = 1;
        inst->rng->nframes = nframes;
    }

    os_error_t err = os_sem_init(&inst->rng->sem, 0x1); 
    assert(err == OS_OK);

    if (config != NULL ){
        inst->rng->config = config;
        dw1000_rng_config(inst, config);
    }

    dw1000_rng_set_callbacks(inst, rng_tx_complete_cb, rng_rx_complete_cb, rng_rx_timeout_cb, rng_rx_error_cb);
    dw1000_rng_set_tx_final_cb(inst, rng_tx_final_cb);

    inst->rng_interface_extension_cb = NULL;
    inst->rng->status.initialized = 1;
    return inst->rng;
}

void 
dw1000_rng_free(dw1000_rng_instance_t * inst){
   
    assert(inst);  
   // free(inst->twr);
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

void 
dw1000_rng_set_callbacks_extension(dw1000_dev_instance_t * inst,  dw1000_dev_cb_t rng_rx_timeout_extension_cb, dw1000_dev_cb_t rng_rx_error_extension_cb,  dw1000_dev_cb_t rng_interface_extension_cb){
    inst->rng_rx_timeout_extension_cb = rng_rx_timeout_extension_cb;
    inst->rng_rx_error_extension_cb = rng_rx_error_extension_cb;
    inst->rng_interface_extension_cb = rng_interface_extension_cb;
}

inline void 
dw1000_rng_set_frames(dw1000_dev_instance_t * inst, twr_frame_t twr[], uint16_t nframes){
        assert(nframes <= inst->rng->nframes);
        for (uint16_t i = 0; i < nframes; i++)
            inst->rng->twr[i] = &twr[i];
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
    os_error_t err = os_sem_pend(&inst->rng->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    for (uint16_t i = 0; i < inst->rng->nframes; i++)
        inst->rng->twr[i]->code = DWT_TWR_INVALID;
    
    twr_frame_t * twr  = inst->rng->twr[0];    
    dw1000_rng_config_t * config = inst->rng->config;

    twr->seq_num++;
    twr->code = code;
    twr->src_address = inst->my_short_address;
    twr->dst_address = dst_address;

    dw1000_write_tx(inst, (uint8_t *) twr, 0, sizeof(ieee_rng_request_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_request_frame_t), 0, true);     
    dw1000_set_wait4resp(inst, true);    
    dw1000_set_rx_timeout(inst, config->rx_timeout_period); 
    dw1000_start_tx(inst);
    
    err = os_sem_pend(&inst->rng->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions 
    os_sem_release(&inst->rng->sem);
    
    if (inst->status.start_tx_error || inst->status.start_rx_error || inst->status.rx_error || inst->status.request_timeout ||  inst->status.rx_timeout_error)
        twr->seq_num--;

   return inst->status;
}

float 
dw1000_rng_twr_to_tof(twr_frame_t twr[], dw1000_rng_modes_t code){
    float ToF = 0;
    uint64_t T1R, T1r, T2R, T2r;
    int64_t nom,denom;

    switch(code){
        case DWT_SS_TWR:
            ToF = ((twr[0].response_timestamp - twr[0].request_timestamp) 
                    -  (twr[0].transmission_timestamp - twr[0].reception_timestamp))/2.; 
        break;
        case DWT_DS_TWR:
            T1R = (twr[0].response_timestamp - twr[0].request_timestamp); 
            T1r = (twr[0].transmission_timestamp  - twr[0].reception_timestamp);         
            T2R = (twr[1].response_timestamp - twr[1].request_timestamp); 
            T2r = (twr[1].transmission_timestamp  - twr[1].reception_timestamp); 
            nom = T1R * T2R  - T1r * T2r;
            denom = T1R + T2R  + T1r + T2r;
            ToF = (float) (nom) / denom; 
        break;
        default: break;       
    }
    return ToF;
}

uint32_t 
dw1000_rng_twr_to_tof_sym(twr_frame_t twr[], dw1000_rng_modes_t code){
    uint32_t ToF = 0;
    uint64_t T1R, T1r, T2R, T2r;

    switch(code){
        case DWT_SS_TWR:
            ToF = ((twr[0].response_timestamp - twr[0].request_timestamp) 
                    -  (twr[0].transmission_timestamp - twr[0].reception_timestamp))/2.; 
        break;
        case DWT_DS_TWR:
            T1R = (twr[0].response_timestamp - twr[0].request_timestamp); 
            T1r = (twr[0].transmission_timestamp  - twr[0].reception_timestamp);         
            T2R = (twr[1].response_timestamp - twr[1].request_timestamp); 
            T2r = (twr[1].transmission_timestamp  - twr[1].reception_timestamp); 
            ToF = (T1R - T1r + T2R - T2r) >> 2;  
        break;
        default: break;       
    }
    return ToF;
}

static void 
rng_tx_final_cb(dw1000_dev_instance_t * inst){

    twr_frame_t * twr = inst->rng->twr[1];
    twr->cartesian.x = MYNEWT_VAL(LOCAL_COORDINATE_X);
    twr->cartesian.y = MYNEWT_VAL(LOCAL_COORDINATE_Y);
    twr->cartesian.z = MYNEWT_VAL(LOCAL_COORDINATE_Z);
    twr->spherical.range = dw1000_rng_tof_to_meters(dw1000_rng_twr_to_tof(*inst->rng->twr, DWT_DS_TWR));
    twr->spherical_variance.range = MYNEWT_VAL(RANGE_VARIANCE);
    twr->spherical_variance.azimuth = -1;
    twr->spherical_variance.zenith = -1;
    twr->utime = os_time_get();//dw1000_read_systime(inst)/128;
}

static void 
rng_tx_complete_cb(dw1000_dev_instance_t * inst)
{
   // Unblock Semaphore after last transmission
   if (inst->rng->twr[0]->code == DWT_SS_TWR_FINAL || inst->rng->twr[0]->code == DWT_SS_TWR_T1){
        os_sem_release(&inst->rng->sem);  
   }
#ifdef  DS_TWR_ENABLE
    else{ 
        if(inst->rng->nframes > 1) 
            if (inst->rng->twr[1]->code ==  DWT_DS_TWR_FINAL || inst->rng->twr[1]->code ==  DWT_DS_TWR_EXT_FINAL){
                os_sem_release(&inst->rng->sem);  
            }
    }
#endif
    if (inst->fctrl_array[0] == FCNTL_IEEE_BLINK_CCP_64){ 
        // Clock Calibration Packet Received
        if (inst->ccp_tx_complete_cb != NULL)
            inst->ccp_tx_complete_cb(inst); 
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

static void 
rng_rx_complete_cb(dw1000_dev_instance_t * inst)
{

    uint16_t code, dst_address; 
    dw1000_rng_config_t * config = inst->rng->config;
    if (inst->fctrl == FCNTL_IEEE_RANGE_16){ 
        dw1000_read_rx(inst, (uint8_t *) &code, offsetof(ieee_rng_request_frame_t,code), sizeof(uint16_t));
        dw1000_read_rx(inst, (uint8_t *) &dst_address, offsetof(ieee_rng_request_frame_t,dst_address), sizeof(uint16_t));    
    }
    else if (inst->fctrl_array[0] == FCNTL_IEEE_BLINK_CCP_64){ 
        uint64_t clock_master;
        dw1000_read_rx(inst, (uint8_t *) &clock_master, offsetof(ieee_blink_frame_t,ext_address), sizeof(uint64_t));    
       
        if (inst->ccp_rx_complete_cb != NULL && inst->clock_master == clock_master)
            inst->ccp_rx_complete_cb(inst);   
        
        inst->control = inst->control_rx_context;
        dw1000_start_rx(inst); 
        return;  
    }
    if (dst_address != inst->my_short_address){
        inst->control = inst->control_rx_context;
        dw1000_start_rx(inst); 
        return;
    }
    
    hal_gpio_toggle(LED_1);

    switch (code){
#ifdef SS_TWR_ENABLE
        case DWT_SS_TWR ... DWT_SS_TWR_FINAL:
            switch(code){
                case DWT_SS_TWR:
                    {
                        // This code executes on the device that is responding to a request
                        // printf("DWT_SS_TWR\n");
                        twr_frame_t * twr = inst->rng->twr[0];
                        if (inst->frame_len >= sizeof(ieee_rng_request_frame_t))
                            dw1000_read_rx(inst, twr->array, 0, sizeof(ieee_rng_request_frame_t));
                        else 
                            break; 
                    
                        uint64_t request_timestamp = dw1000_read_rxtime(inst);  
                        uint64_t response_tx_delay = request_timestamp + ((uint64_t)config->tx_holdoff_delay << 15); 
                        uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;
        
                        twr->reception_timestamp = request_timestamp;
                        twr->transmission_timestamp = response_timestamp;
                        twr->dst_address = twr->src_address;
                        twr->src_address = inst->my_short_address;
                        twr->code = DWT_SS_TWR_T1;

                        dw1000_write_tx(inst, twr->array, 0, sizeof(ieee_rng_response_frame_t));
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
                        twr_frame_t * twr = inst->rng->twr[0];
                        if (inst->frame_len >= sizeof(ieee_rng_response_frame_t))
                            dw1000_read_rx(inst, twr->array, 0, sizeof(ieee_rng_response_frame_t));
                        else 
                            break;

                        twr->request_timestamp = dw1000_read_txtime_lo(inst);   // This corresponds to when the original request was actually sent
                        twr->response_timestamp = dw1000_read_rxtime_lo(inst);  // This corresponds to the response just received            
                        twr->dst_address = twr->src_address;
                        twr->src_address = inst->my_short_address;
                        twr->code = DWT_SS_TWR_FINAL;
                    
                        // Transmit timestamp final report
                        dw1000_write_tx(inst, twr->array, 0, sizeof(twr_frame_final_t));
                        dw1000_write_tx_fctrl(inst, sizeof(twr_frame_final_t), 0, true); 

                        if (dw1000_start_tx(inst).start_tx_error)
                            os_sem_release(&inst->rng->sem);  
                        break;
                    }
                case  DWT_SS_TWR_FINAL:
                    {
                        // This code executes on the device that responded to the original request, and has now receive the response final timestamp. 
                        // This marks the completion of the single-size-two-way request. This final 4th message is perhaps optional in some applicaiton. 
                        // printf("DWT_SS_TWR_FINAL\n");

                        twr_frame_t * twr  = inst->rng->twr[0]; 
                        if (inst->frame_len >= sizeof(twr_frame_final_t))
                            dw1000_read_rx(inst, twr->array, 0, sizeof(twr_frame_final_t));
                            
                        os_sem_release(&inst->rng->sem);
                        break;
                    }
                default: 
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
                            twr_frame_t * twr = inst->rng->twr[0];
                            if (inst->frame_len >= sizeof(ieee_rng_request_frame_t))
                                dw1000_read_rx(inst, twr->array, 0, sizeof(ieee_rng_request_frame_t));
                            else 
                                break; 

                            uint64_t request_timestamp = dw1000_read_rxtime(inst);  
                            uint64_t response_tx_delay = request_timestamp + ((uint64_t)config->tx_holdoff_delay << 15); 
                            uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;
            
                            twr->reception_timestamp =  request_timestamp;
                            twr->transmission_timestamp =  response_timestamp;

                            twr->dst_address = twr->src_address;
                            twr->src_address = inst->my_short_address;
                            twr->code = DWT_DS_TWR_T1;

                            dw1000_write_tx(inst, twr->array, 0, sizeof(ieee_rng_response_frame_t));
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
                            twr_frame_t * twr = inst->rng->twr[0];
                            if (inst->frame_len >= sizeof(ieee_rng_response_frame_t))
                                dw1000_read_rx(inst, twr->array, 0, sizeof(ieee_rng_response_frame_t));
                            else 
                                break;

                            inst->rng->twr[0]->request_timestamp = inst->rng->twr[1]->request_timestamp = dw1000_read_txtime_lo(inst);   // This corresponds to when the original request was actually sent
                            inst->rng->twr[0]->response_timestamp = inst->rng->twr[1]->response_timestamp = dw1000_read_rxtime_lo(inst);  // This corresponds to the response just received      
                            
                            // Note:: Switching from 1st to 2nd ss_twr frame    
                            twr = inst->rng->twr[1];
                            twr->dst_address = inst->rng->twr[0]->src_address;
                            twr->src_address = inst->my_short_address;
                            twr->seq_num = inst->rng->twr[0]->seq_num;
                            twr->code = DWT_DS_TWR_T2;

                            uint64_t request_timestamp = dw1000_read_rxtime(inst);  
                            uint64_t response_tx_delay = request_timestamp + ((uint64_t)config->tx_holdoff_delay << 15); 
                            uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;
                            
                            twr->reception_timestamp = request_timestamp;
                            twr->transmission_timestamp = response_timestamp;

                            dw1000_write_tx(inst, twr->array, 0, sizeof(twr_frame_final_t));
                            dw1000_write_tx_fctrl(inst, sizeof(twr_frame_final_t), 0, true); 
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
                            twr_frame_t * twr = inst->rng->twr[1];
                            if (inst->frame_len >= sizeof(twr_frame_final_t))
                                dw1000_read_rx(inst,  twr->array, 0, sizeof(twr_frame_final_t));
                            else 
                                break;

                            inst->rng->twr[0]->request_timestamp = inst->rng->twr[1]->request_timestamp;
                            inst->rng->twr[0]->response_timestamp = inst->rng->twr[1]->response_timestamp;

                            twr->request_timestamp = dw1000_read_txtime_lo(inst);   // This corresponds to when the original request was actually sent
                            twr->response_timestamp = dw1000_read_rxtime_lo(inst);  // This corresponds to the response just received            
                            twr->dst_address = twr->src_address;
                            twr->src_address = inst->my_short_address;
                            twr->code = DWT_DS_TWR_FINAL;

                            // Transmit timestamp final report
                            dw1000_write_tx(inst, twr->array, 0, sizeof(twr_frame_final_t));
                            dw1000_write_tx_fctrl(inst, sizeof(twr_frame_final_t), 0, true); 

                            if (dw1000_start_tx(inst).start_tx_error)
                                os_sem_release(&inst->rng->sem);  
                            break;
                        }
                    case  DWT_DS_TWR_FINAL:
                        {
                            // This code executes on the device that initialed the original request, and has now receive the final response timestamp. 
                            // This marks the completion of the double-single-two-way request. 
                            // printf("DWT_SDS_TWR_FINAL\n");
                            twr_frame_t * twr = inst->rng->twr[1];
                            if (inst->frame_len >= sizeof(twr_frame_final_t))
                                dw1000_read_rx(inst, twr->array, 0, sizeof(twr_frame_final_t));

                            os_sem_release(&inst->rng->sem);
                            break;
                        }
                    default: 
                        break;
                }
            break;
#endif //DS_TWR_ENABLE
#ifdef DS_TWR_EXT_ENABLE
        case DWT_DS_TWR_EXT ... DWT_DS_TWR_EXT_FINAL:
            switch(code){
                    case DWT_DS_TWR_EXT:
                        {
                            // This code executes on the device that is responding to a original request
                            twr_frame_t * twr = inst->rng->twr[0];
                            if (inst->frame_len >= sizeof(ieee_rng_request_frame_t))
                                dw1000_read_rx(inst, twr->array, 0, sizeof(ieee_rng_request_frame_t));
                            else 
                                break; 

                            uint64_t request_timestamp = dw1000_read_rxtime(inst);  
                            uint64_t response_tx_delay = request_timestamp + ((uint64_t)config->tx_holdoff_delay << 15); 
                            uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;
            
                            twr->reception_timestamp = request_timestamp;
                            twr->transmission_timestamp = response_timestamp;

                            twr->dst_address = twr->src_address;
                            twr->src_address = inst->my_short_address;
                            twr->code = DWT_DS_TWR_EXT_T1;

                            dw1000_write_tx(inst, twr->array, 0, sizeof(ieee_rng_response_frame_t));
                            dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_response_frame_t), 0, true); 
                            dw1000_set_wait4resp(inst, true);    
                            dw1000_set_delay_start(inst, response_tx_delay);   
                            dw1000_set_rx_timeout(inst, config->rx_timeout_period); 

                            if (dw1000_start_tx(inst).start_tx_error)
                                os_sem_release(&inst->rng->sem);  
                            break;
                        }
                    case DWT_DS_TWR_EXT_T1:
                        {
                            // This code executes on the device that initiated the original request, and is now preparing the next series of timestamps
                            // The 1st frame now contains a local copy of the initial first side of the double sided scheme. 
                            twr_frame_t * twr = inst->rng->twr[0];
                            if (inst->frame_len >= sizeof(ieee_rng_response_frame_t))
                                dw1000_read_rx(inst,  twr->array, 0, sizeof(ieee_rng_response_frame_t));
                            else 
                                break;

                            twr->request_timestamp = inst->rng->twr[1]->request_timestamp = dw1000_read_txtime_lo(inst);   // This corresponds to when the original request was actually sent
                            twr->response_timestamp = inst->rng->twr[1]->response_timestamp = dw1000_read_rxtime_lo(inst);  // This corresponds to the response just received      
                            
                            // Note:: Switching from 1st to 2nd ss_twr frame    
                            twr = inst->rng->twr[1];
                            twr->dst_address = inst->rng->twr[0]->src_address;
                            twr->src_address = inst->my_short_address;
                            twr->seq_num = inst->rng->twr[0]->seq_num;
                            twr->code = DWT_DS_TWR_EXT_T2;

                            uint64_t request_timestamp = dw1000_read_rxtime(inst);  
                            uint64_t response_tx_delay = request_timestamp + ((uint64_t)config->tx_holdoff_delay << 15); 
                            uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;
                            
                            twr->reception_timestamp = request_timestamp;
                            twr->transmission_timestamp = response_timestamp;

                            // Final callback, prior to transmission, use this callback to populate the FUSION_EXTENDED_FRAME fields.
                            if (inst->rng_tx_final_cb != NULL)
                                inst->rng_tx_final_cb(inst);

                            dw1000_write_tx(inst, twr->array, 0, sizeof(twr_frame_t));
                            dw1000_write_tx_fctrl(inst, sizeof(twr_frame_t), 0, true); 
                            dw1000_set_wait4resp(inst, true);    
                            dw1000_set_delay_start(inst, response_tx_delay);   
                            dw1000_set_rx_timeout(inst, config->rx_timeout_period); 
                        
                            if (dw1000_start_tx(inst).start_tx_error)
                                os_sem_release(&inst->rng->sem);  

                            break; 
                        }

                    case DWT_DS_TWR_EXT_T2:
                        {
                            // This code executes on the device that responded to the original request, and is now preparing the final timestamps
                            twr_frame_t * twr = inst->rng->twr[1];
                            if (inst->frame_len >= sizeof(twr_frame_t))
                                dw1000_read_rx(inst, twr->array, 0, sizeof(twr_frame_t));
                            else 
                                break;

                            inst->rng->twr[0]->request_timestamp = inst->rng->twr[1]->request_timestamp;
                            inst->rng->twr[0]->response_timestamp = inst->rng->twr[1]->response_timestamp;

                            twr->request_timestamp = dw1000_read_txtime_lo(inst);   // This corresponds to when the original request was actually sent
                            twr->response_timestamp = dw1000_read_rxtime_lo(inst);  // This corresponds to the response just received            
                            twr->dst_address = twr->src_address;
                            twr->src_address = inst->my_short_address;
                            twr->code = DWT_DS_TWR_EXT_FINAL;

                            // Final callback, prior to transmission, use this callback to populate the FUSION_EXTENDED_FRAME fields.
                            if (inst->rng_tx_final_cb != NULL)
                                inst->rng_tx_final_cb(inst);

                            // Transmit timestamp final report
                            dw1000_write_tx(inst, twr->array, 0, sizeof(twr_frame_t));
                            dw1000_write_tx_fctrl(inst, sizeof(twr_frame_t), 0, true); 

                            if (dw1000_start_tx(inst).start_tx_error)
                                os_sem_release(&inst->rng->sem);  
                            break;
                        }
                    case  DWT_DS_TWR_EXT_FINAL:
                        {
                            // This code executes on the device that initialed the original request, and has now receive the final response timestamp. 
                            // This marks the completion of the double-single-two-way request. 
                            twr_frame_t * twr = inst->rng->twr[1];
                            if (inst->frame_len >= sizeof(twr_frame_t))
                                dw1000_read_rx(inst, twr->array, 0, sizeof(twr_frame_t));

                            os_sem_release(&inst->rng->sem);
                            break;
                        }
                    default: 
                        break;
                }
            break;
#endif //DS_TWR_EXT_ENABLE
        default: 
            // Use this callback to extend interface and ranging services
            if (inst->rng_interface_extension_cb != NULL)
                inst->rng_interface_extension_cb(inst);
            break;
    }  
}



