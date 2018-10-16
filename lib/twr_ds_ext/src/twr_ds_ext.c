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
#include <rng/rng.h>
#include <dsp/polyval.h>

//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

static bool rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool rx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool tx_final_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool start_tx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);

static dw1000_mac_interface_t g_cbs[] = {
        [0] = {
            .id = DW1000_RNG_DS_EXT,
            .rx_complete_cb = rx_complete_cb,
            .rx_timeout_cb = rx_timeout_cb,
            .rx_error_cb = rx_error_cb,
            .reset_cb = reset_cb,
            .final_cb = tx_final_cb,
            .start_tx_error_cb = start_tx_error_cb
        },
#if MYNEWT_VAL(DW1000_DEVICE_1)
        [1] = {
            .id = DW1000_RNG_DS_EXT,
            .rx_complete_cb = rx_complete_cb,
        },
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
        [2] = {
            .id = DW1000_RNG_DS_EXT,
            .rx_complete_cb = rx_complete_cb,
        }
#endif
};


static dw1000_rng_config_t g_config = {
    .tx_holdoff_delay = MYNEWT_VAL(TWR_DS_EXT_TX_HOLDOFF),         // Send Time delay in usec.
    .rx_timeout_period = MYNEWT_VAL(TWR_DS_EXT_RX_TIMEOUT)       // Receive response timeout in usec
};
/**
 * API to initialise the rng_ss package.
 *
 *
 * @return void
 */

void twr_ds_ext_pkg_init(void){

    printf("{\"utime\": %lu,\"msg\": \"twr_ds_ext_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

#if MYNEWT_VAL(DW1000_DEVICE_0)
    dw1000_mac_append_interface(hal_dw1000_inst(0), &g_cbs[0]);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    dw1000_mac_append_interface(hal_dw1000_inst(1), &g_cbs[1]);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    dw1000_mac_append_interface(hal_dw1000_inst(1), &g_cbs[2]);
#endif
  
}


/**
 * API to free the allocated resources.
 *
 * @param inst  Pointer to dw1000_rng_instance_t.
 *
 * @return void 
 */
void 
twr_ds_ext_free(dw1000_dev_instance_t * inst){
    assert(inst); 
    dw1000_mac_remove_interface(inst, DW1000_RNG_DS);
}

/**
 * API for get local config callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
dw1000_rng_config_t * 
twr_ds_ext_config(dw1000_dev_instance_t * inst){
    return &g_config;
}

/**
 * API for final transmission to calculate range.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 *
 * @return void
 */
static bool
tx_final_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){

    dw1000_rng_instance_t * rng = inst->rng; 
    twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];

    DIAGMSG("{\"utime\": %lu,\"msg\": \"tx_final_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

    frame->cartesian.x = MYNEWT_VAL(LOCAL_COORDINATE_X);
    frame->cartesian.y = MYNEWT_VAL(LOCAL_COORDINATE_Y);
    frame->cartesian.z = MYNEWT_VAL(LOCAL_COORDINATE_Z);
  
#if MYNEWT_VAL(DW1000_BIAS_CORRECTION_ENABLED)
    if (inst->config.bias_correction_enable){ 
        float range = dw1000_rng_tof_to_meters(dw1000_rng_twr_to_tof(rng)); 
        float bias = 2 * dw1000_rng_bias_correction(inst, 
                    dw1000_rng_path_loss(
                        MYNEWT_VAL(DW1000_DEVICE_TX_PWR),
                        MYNEWT_VAL(DW1000_DEVICE_ANT_GAIN),
                        MYNEWT_VAL(DW1000_DEVICE_FREQ),
                        range)
                    );
        frame->spherical.range = range - bias;
    }
#else
    frame->spherical.range = dw1000_rng_tof_to_meters(dw1000_rng_twr_to_tof(rng));
#endif
    frame->spherical_variance.range = MYNEWT_VAL(RANGE_VARIANCE);
    frame->spherical_variance.azimuth = -1;
    frame->spherical_variance.zenith = -1;
    frame->utime = os_cputime_ticks_to_usecs(os_cputime_get32());

    return true;
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

   if(os_sem_get_count(&inst->rng->sem) == 0){
        printf("{\"utime\": %lu,\"log\": \"rng_rx_timeout_cb\",\"%s\":%d}\n",os_cputime_ticks_to_usecs(os_cputime_get32()),__FILE__, __LINE__); 
        os_sem_release(&inst->rng->sem);
        return true;
    }
    else
        return false;
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

    if (inst->fctrl != FCNTL_IEEE_RANGE_16)
        return false;
    else if(os_sem_get_count(&inst->rng->sem) == 0){
        os_error_t err = os_sem_release(&inst->rng->sem);   
        assert(err == OS_OK);
        return true;
    }
    else
        return false;
}

/** 
 * API for reset_cb of rng interface
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 * @return true on sucess
 */
static bool
reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){

    if(os_sem_get_count(&inst->rng->sem) == 0){
        os_error_t err = os_sem_release(&inst->rng->sem);  
        assert(err == OS_OK);
        return true;
    }
    else 
       return false;

}

/**
 * API for receive start error callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
static bool 
start_tx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    printf("{\"utime\": %lu,\"log\": \"start_tx_error_cb\",\"%s\":%d}\n",os_cputime_ticks_to_usecs(os_cputime_get32()),__FILE__, __LINE__); 
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
   if (inst->fctrl != FCNTL_IEEE_RANGE_16)
        return false;

    switch(inst->rng->code){
        case DWT_DS_TWR_EXT:
            {
                // This code executes on the device that is responding to a original request
                DIAGMSG("{\"utime\": %lu,\"msg\": \"DWT_DS_TWR_EXT\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

                dw1000_rng_instance_t * rng = inst->rng; 
                twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];
                if (inst->frame_len == sizeof(ieee_rng_request_frame_t))
                    dw1000_read_rx(inst, frame->array, 0, sizeof(ieee_rng_request_frame_t));
                else 
                    break; 

                uint64_t request_timestamp = dw1000_read_rxtime(inst);  
                uint64_t response_tx_delay = request_timestamp + ((uint64_t) g_config.tx_holdoff_delay << 16);
                uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;
            
                frame->reception_timestamp = request_timestamp;
                frame->transmission_timestamp = response_timestamp;

                frame->dst_address = frame->src_address;
                frame->src_address = inst->my_short_address;
                frame->code = DWT_DS_TWR_EXT_T1;

                dw1000_write_tx(inst, frame->array, 0, sizeof(ieee_rng_response_frame_t));
                dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_response_frame_t), 0, true); 
                dw1000_set_wait4resp(inst, true);    
                dw1000_set_delay_start(inst, response_tx_delay);   
                uint16_t timeout = dw1000_phy_frame_duration(&inst->attrib, sizeof(ieee_rng_response_frame_t)) 
                                + g_config.rx_timeout_period        
                                + g_config.tx_holdoff_delay;         // Remote side turn arroud time. 
                 dw1000_set_rx_timeout(inst, timeout); 

                if (dw1000_start_tx(inst).start_tx_error){
                    os_sem_release(&rng->sem);  
                    if (cbs!=NULL && cbs->start_tx_error_cb) 
                        cbs->start_tx_error_cb(inst, cbs);
                }
                break;
            }
        case DWT_DS_TWR_EXT_T1:
            {
                // This code executes on the device that initiated the original request, and is now preparing the next series of timestamps
                // The 1st frame now contains a local copy of the initial first side of the double sided scheme. 
                DIAGMSG("{\"utime\": %lu,\"msg\": \"DWT_DS_TWR_T1\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

                dw1000_rng_instance_t * rng = inst->rng; 
                twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];
                twr_frame_t * next_frame = rng->frames[(rng->idx+1)%rng->nframes];
   
                if (inst->frame_len == sizeof(ieee_rng_response_frame_t))
                    dw1000_read_rx(inst, frame->array + sizeof(ieee_rng_request_frame_t),  
                                            sizeof(ieee_rng_request_frame_t), 
                                            sizeof(ieee_rng_response_frame_t) - sizeof(ieee_rng_request_frame_t)
                    );
                else 
                    break;

                frame->request_timestamp = next_frame->request_timestamp = dw1000_read_txtime_lo(inst);    // This corresponds to when the original request was actually sent
                frame->response_timestamp = next_frame->response_timestamp = dw1000_read_rxtime_lo(inst);  // This corresponds to the response just received      
                        
                uint16_t src_address = frame->src_address; 
                uint8_t seq_num = frame->seq_num; 

                // Note:: Advance to next frame 
                frame = next_frame;                            
                frame->dst_address = src_address;
                frame->src_address = inst->my_short_address;
                frame->seq_num = seq_num + 1;
                frame->code = DWT_DS_TWR_EXT_T2;

                uint64_t request_timestamp = dw1000_read_rxtime(inst);  
                uint64_t response_tx_delay = request_timestamp + ((uint64_t)g_config.tx_holdoff_delay << 16);
                uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;
                            
                frame->reception_timestamp = request_timestamp;
                frame->transmission_timestamp = response_timestamp;

                // Final callback, prior to transmission, use this callback to populate the EXTENDED_FRAME fields.
                dw1000_mac_interface_t * cbs = NULL;

                if (cbs!=NULL && cbs->final_cb) 
                    cbs->final_cb(inst, cbs);

                dw1000_write_tx(inst, frame->array, 0, sizeof(twr_frame_t));
                dw1000_write_tx_fctrl(inst, sizeof(twr_frame_t), 0, true); 
                dw1000_set_wait4resp(inst, true);    
                dw1000_set_delay_start(inst, response_tx_delay);   
                uint16_t timeout = dw1000_phy_frame_duration(&inst->attrib, sizeof(twr_frame_t)) 
                                + g_config.rx_timeout_period        
                                + g_config.tx_holdoff_delay;         // Remote side turn arroud time. 
                dw1000_set_rx_timeout(inst, timeout); 
                        
                if (dw1000_start_tx(inst).start_tx_error){
                    os_sem_release(&rng->sem);  
                    if (cbs!=NULL && cbs->start_tx_error_cb) 
                        cbs->start_tx_error_cb(inst, cbs);
                }	
                break; 
            }

        case DWT_DS_TWR_EXT_T2:
            {
                 // This code executes on the device that responded to the original request, and is now preparing the final timestamps
                DIAGMSG("{\"utime\": %lu,\"msg\": \"DWT_DS_TWR_T2\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

                dw1000_rng_instance_t * rng = inst->rng; 
                twr_frame_t * previous_frame = rng->frames[(rng->idx-1)%rng->nframes];
                twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];

                if (inst->frame_len >= sizeof(twr_frame_t))
                    dw1000_read_rx(inst, frame->array + sizeof(ieee_rng_request_frame_t),  
                                            sizeof(ieee_rng_request_frame_t), 
                                            sizeof(twr_frame_t) - sizeof(ieee_rng_request_frame_t)
                    );
                else 
                    break;

                previous_frame->request_timestamp = frame->request_timestamp;
                previous_frame->response_timestamp = frame->response_timestamp;
                frame->request_timestamp = dw1000_read_txtime_lo(inst);   // This corresponds to when the original request was actually sent
                frame->response_timestamp = dw1000_read_rxtime_lo(inst);  // This corresponds to the response just received            
                frame->dst_address = frame->src_address;
                frame->src_address = inst->my_short_address;
                frame->code = DWT_DS_TWR_EXT_FINAL;

                // Final callback, prior to transmission, use this callback to populate the EXTENDED_FRAME fields.
                if (cbs!=NULL && cbs->final_cb) 
                    cbs->final_cb(inst, cbs);    
              
                // Transmit timestamp final report
                dw1000_write_tx(inst, frame->array, 0, sizeof(twr_frame_t));
                dw1000_write_tx_fctrl(inst, sizeof(twr_frame_t), 0, true); 
                         
                if (dw1000_start_tx(inst).start_tx_error){
                    os_sem_release(&rng->sem);  
                    if (cbs!=NULL && cbs->start_tx_error_cb) 
                        cbs->start_tx_error_cb(inst, cbs);
                }else{
                    os_sem_release(&rng->sem);
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
        case  DWT_DS_TWR_EXT_FINAL:
            {
                // This code executes on the device that initialed the original request, and has now receive the final response timestamp. 
                // This marks the completion of the double-single-two-way request. 
                DIAGMSG("{\"utime\": %lu,\"msg\": \"DWT_DS_TWR_FINAL\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

                dw1000_rng_instance_t * rng = inst->rng; 
                twr_frame_t * frame = inst->rng->frames[(rng->idx)%rng->nframes];
                if (inst->frame_len >= sizeof(twr_frame_t))
                    dw1000_read_rx(inst, frame->array + sizeof(ieee_rng_request_frame_t),  
                                            sizeof(ieee_rng_request_frame_t), 
                                            sizeof(twr_frame_t) - sizeof(ieee_rng_request_frame_t)
                    );     
                os_sem_release(&rng->sem);
                dw1000_mac_interface_t * cbs = NULL;
                if(!(SLIST_EMPTY(&inst->interface_cbs))){ 
                    SLIST_FOREACH(cbs, &inst->interface_cbs, next){    
                        if (cbs!=NULL && cbs->complete_cb) 
                            if(cbs->complete_cb(inst, cbs)) continue;         
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


