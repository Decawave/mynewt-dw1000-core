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
 * @file twr_ds_ext.c
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

#include <stats/stats.h>
#include <uwb/uwb.h>
#include <rng/rng.h>
#include <dsp/polyval.h>

//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

static bool rx_complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs);
static bool tx_final_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs);
static bool reset_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs);
static bool start_tx_error_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs);

static struct uwb_mac_interface g_cbs[] = {
        [0] = {
            .id = UWBEXT_RNG_DS_EXT,
            .rx_complete_cb = rx_complete_cb,
            .reset_cb = reset_cb,
            .final_cb = tx_final_cb,
            .start_tx_error_cb = start_tx_error_cb
        },
#if MYNEWT_VAL(DW1000_DEVICE_1) || MYNEWT_VAL(DW1000_DEVICE_2)
        [1] = {
            .id = UWBEXT_RNG_DS_EXT,
            .rx_complete_cb = rx_complete_cb,
            .reset_cb = reset_cb,
            .final_cb = tx_final_cb,
            .start_tx_error_cb = start_tx_error_cb
        },
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
        [2] = {
            .id = UWBEXT_RNG_DS_EXT,
            .rx_complete_cb = rx_complete_cb,
            .reset_cb = reset_cb,
            .final_cb = tx_final_cb,
            .start_tx_error_cb = start_tx_error_cb
        }
#endif
};

STATS_SECT_START(twr_ds_ext_stat_section)
    STATS_SECT_ENTRY(complete)
    STATS_SECT_ENTRY(tx_error)
    STATS_SECT_ENTRY(reset)
STATS_SECT_END

STATS_NAME_START(twr_ds_ext_stat_section)
    STATS_NAME(twr_ds_ext_stat_section, complete)
    STATS_NAME(twr_ds_ext_stat_section, tx_error)
    STATS_NAME(twr_ds_ext_stat_section, reset)
STATS_NAME_END(twr_ds_ext_stat_section)

static STATS_SECT_DECL(twr_ds_ext_stat_section) g_stat;

static dw1000_rng_config_t g_config = {
    .tx_holdoff_delay = MYNEWT_VAL(TWR_DS_EXT_TX_HOLDOFF),         // Send Time delay in usec.
    .rx_timeout_delay = MYNEWT_VAL(TWR_DS_EXT_RX_TIMEOUT)       // Receive response timeout in usec
};

static struct rng_config_list g_rng_cfgs[] = {
    [0] = {
        .rng_code = DWT_DS_TWR_EXT,
        .config = &g_config
    },
#if MYNEWT_VAL(DW1000_DEVICE_1) ||  MYNEWT_VAL(DW1000_DEVICE_2)
    [1] = {
        .rng_code = DWT_DS_TWR_EXT,
        .config = &g_config
    },
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    [2] = {
        .rng_code = DWT_DS_TWR_EXT,
        .config = &g_config
    },
#endif
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
    g_cbs[0].inst_ptr = (dw1000_rng_instance_t*)uwb_mac_find_cb_inst_ptr(uwb_dev_idx_lookup(0), UWBEXT_RNG);
    assert(g_cbs[0].inst_ptr);
    uwb_mac_append_interface(uwb_dev_idx_lookup(0), &g_cbs[0]);
    dw1000_rng_append_config(g_cbs[0].inst_ptr, &g_rng_cfgs[0]);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    g_cbs[1].inst_ptr = (dw1000_rng_instance_t*)uwb_mac_find_cb_inst_ptr(uwb_dev_idx_lookup(1), UWBEXT_RNG);
    uwb_mac_append_interface(uwb_dev_idx_lookup(1), &g_cbs[1]);
    dw1000_rng_append_config(g_cbs[1].inst_ptr, &g_rng_cfgs[1]);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    g_cbs[2].inst_ptr = (dw1000_rng_instance_t*)uwb_mac_find_cb_inst_ptr(uwb_dev_idx_lookup(2), UWBEXT_RNG);
    uwb_mac_append_interface(uwb_dev_idx_lookup(2), &g_cbs[2]);
    dw1000_rng_append_config(g_cbs[2].inst_ptr, &g_rng_cfgs[2]);
#endif
  
    int rc = stats_init(
        STATS_HDR(g_stat),
        STATS_SIZE_INIT_PARMS(g_stat, STATS_SIZE_32),
        STATS_NAME_INIT_PARMS(twr_ds_ext_stat_section));
    assert(rc == 0);
    
    rc = stats_register("twr_ds_ext", STATS_HDR(g_stat));
    assert(rc == 0);
}


/**
 * API to free the allocated resources.
 *
 * @param inst  Pointer to dw1000_rng_instance_t.
 *
 * @return void 
 */
void 
twr_ds_ext_free(struct uwb_dev * inst){
    assert(inst); 
    uwb_mac_remove_interface(inst, UWBEXT_RNG_DS_EXT);
}

/**
 * API for final transmission to calculate range.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 *
 * @return void
 */
static bool
tx_final_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
{
    dw1000_rng_instance_t * rng = (dw1000_rng_instance_t *)cbs->inst_ptr;
    twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];

    frame->cartesian.x = MYNEWT_VAL(LOCAL_COORDINATE_X);
    frame->cartesian.y = MYNEWT_VAL(LOCAL_COORDINATE_Y);
    frame->cartesian.z = MYNEWT_VAL(LOCAL_COORDINATE_Z);
  
#if MYNEWT_VAL(DW1000_BIAS_CORRECTION_ENABLED)
    if (inst->config.bias_correction_enable){ 
        float range = dw1000_rng_tof_to_meters(dw1000_rng_twr_to_tof(rng,rng->idx)); 
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
    frame->spherical.range = dw1000_rng_tof_to_meters(dw1000_rng_twr_to_tof(rng,rng->idx));
#endif
    frame->spherical_variance.range = MYNEWT_VAL(RANGE_VARIANCE);
    frame->spherical_variance.azimuth = -1;
    frame->spherical_variance.zenith = -1;
    frame->utime = os_cputime_ticks_to_usecs(os_cputime_get32());

    return true;
}

/**
 * API for receive start error callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
static bool 
start_tx_error_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
{
    STATS_INC(g_stat, tx_error);
    return true;
}


/** 
 * API for reset_cb of rng interface
 *å
 * @param inst   Pointer to dw1000_dev_instance_t. 
 * @return true on sucess
 */
static bool
reset_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
{
    dw1000_rng_instance_t * rng = (dw1000_rng_instance_t *)cbs->inst_ptr;
    if(dpl_sem_get_count(&rng->sem) == 0){
        STATS_INC(g_stat, reset);
        dpl_error_t err = dpl_sem_release(&rng->sem);
        assert(err == DPL_OK);
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
rx_complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
{
   if (inst->fctrl != FCNTL_IEEE_RANGE_16)
        return false;

    dw1000_rng_instance_t * rng = (dw1000_rng_instance_t *)cbs->inst_ptr;
    if(dpl_sem_get_count(&rng->sem) == 1){ 
        // unsolicited inbound
        return false;
    }
        
    switch(rng->code){
        case DWT_DS_TWR_EXT:
            {
                // This code executes on the device that is responding to a original request

                twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];
                
                uint64_t request_timestamp = inst->rxtimestamp;
                uint64_t response_tx_delay = request_timestamp + ((uint64_t) g_config.tx_holdoff_delay << 16);
                uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;

                frame->reception_timestamp =  (uint32_t) (request_timestamp & 0xFFFFFFFFUL);
                frame->transmission_timestamp =  (uint32_t) (response_timestamp & 0xFFFFFFFFUL);

                frame->dst_address = frame->src_address;
                frame->src_address = inst->my_short_address;
#if MYNEWT_VAL(WCS_ENABLED)
                frame->carrier_integrator  = 0.0l;
#else
                frame->carrier_integrator  = - inst->carrier_integrator;
#endif
                frame->code = DWT_DS_TWR_EXT_T1;

                uwb_write_tx(inst, frame->array, 0, sizeof(ieee_rng_response_frame_t));
                uwb_write_tx_fctrl(inst, sizeof(ieee_rng_response_frame_t), 0);
                uwb_set_wait4resp(inst, true);    

                uint16_t frame_duration = uwb_phy_frame_duration(inst,sizeof(ieee_rng_response_frame_t));
                uint16_t shr_duration  = uwb_phy_SHR_duration(inst);
                uint16_t data_duration = frame_duration - shr_duration;
                uwb_set_wait4resp_delay(inst, g_config.tx_holdoff_delay - data_duration - shr_duration);  

                uwb_set_delay_start(inst, response_tx_delay);
                uwb_set_rx_timeout(inst,  frame_duration + g_config.tx_holdoff_delay + g_config.rx_timeout_delay);
                // Disable default behavor, do not RXENAB on RXFCG thereby avoiding rx timeout events  
                uwb_set_rxauto_disable(inst, true);

                if (uwb_start_tx(inst).start_tx_error){
                    dpl_sem_release(&rng->sem);
                    if (cbs!=NULL && cbs->start_tx_error_cb) 
                        cbs->start_tx_error_cb(inst, cbs);
                }
                break;
            }
        case DWT_DS_TWR_EXT_T1:
            {
                // This code executes on the device that initiated the original request, and is now preparing the next series of timestamps
                // The 1st frame now contains a local copy of the initial first side of the double sided scheme. 
         

                if (inst->frame_len != sizeof(ieee_rng_response_frame_t)) 
                    break;

                twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];
                twr_frame_t * next_frame = rng->frames[(rng->idx+1)%rng->nframes];
   
                uint64_t request_timestamp = inst->rxtimestamp;
                frame->request_timestamp = next_frame->request_timestamp = uwb_read_txtime_lo32(inst); // This corresponds to when the original request was actually sent
                frame->response_timestamp = next_frame->response_timestamp = (uint32_t)(request_timestamp & 0xFFFFFFFFUL); // This corresponds to the response just received
                     
                uint16_t src_address = frame->src_address; 
                uint8_t seq_num = frame->seq_num; 

#if MYNEWT_VAL(WCS_ENABLED)
                frame->carrier_integrator  = 0.0l;
#else
                frame->carrier_integrator  = inst->carrier_integrator;
#endif
                // Note:: Advance to next frame 
                frame = next_frame;                            
                frame->dst_address = src_address;
                frame->src_address = inst->my_short_address;
                frame->seq_num = seq_num + 1;
                frame->code = DWT_DS_TWR_EXT_T2;

                if(inst->status.lde_error)
                    break;

                uint64_t response_tx_delay = request_timestamp + ((uint64_t)g_config.tx_holdoff_delay << 16);
                uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;
                   
                frame->reception_timestamp =  (uint32_t) (request_timestamp & 0xFFFFFFFFUL);
                frame->transmission_timestamp =  (uint32_t) (response_timestamp & 0xFFFFFFFFUL);

                // Final callback, prior to transmission, use this callback to populate the EXTENDED_FRAME fields.
                if (cbs!=NULL && cbs->final_cb) 
                    cbs->final_cb(inst, cbs);

                uwb_write_tx(inst, frame->array, 0, sizeof(twr_frame_t));
                uwb_write_tx_fctrl(inst, sizeof(twr_frame_t), 0);
                uwb_set_wait4resp(inst, true);   

                uint16_t frame_duration = uwb_phy_frame_duration(inst,sizeof(twr_frame_t));
                uint16_t shr_duration  = uwb_phy_SHR_duration(inst);
                uint16_t data_duration = frame_duration - shr_duration;
                uwb_set_wait4resp_delay(inst, g_config.tx_holdoff_delay - data_duration - shr_duration);
                uwb_set_delay_start(inst, response_tx_delay);
                uwb_set_rx_timeout(inst, frame_duration + g_config.rx_timeout_delay);

                // Disable default behavor, do not RXENAB on RXFCG thereby avoiding rx timeout events on sucess  
                uwb_set_rxauto_disable(inst, true);
                        
                if (uwb_start_tx(inst).start_tx_error){
                    dpl_sem_release(&rng->sem);
                    if (cbs!=NULL && cbs->start_tx_error_cb) 
                        cbs->start_tx_error_cb(inst, cbs);
                }	
                break; 
            }

        case DWT_DS_TWR_EXT_T2:
            {
                // This code executes on the device that responded to the original request, and is now preparing the final timestamps
        
                twr_frame_t * previous_frame = rng->frames[(uint16_t)(rng->idx-1)%rng->nframes];
                twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];

                previous_frame->request_timestamp = frame->request_timestamp;
                previous_frame->response_timestamp = frame->response_timestamp;

                uint64_t request_timestamp = inst->rxtimestamp;
                frame->request_timestamp = uwb_read_txtime_lo32(inst);   // This corresponds to when the original request was actually sent
                frame->response_timestamp = (uint32_t) (request_timestamp & 0xFFFFFFFFUL);  // This corresponds to the response just received       
                
                frame->dst_address = frame->src_address;
                frame->src_address = inst->my_short_address;
#if MYNEWT_VAL(WCS_ENABLED)
                frame->carrier_integrator  = 0.0l;
#else
                frame->carrier_integrator  = - inst->carrier_integrator;
#endif
                frame->code = DWT_DS_TWR_EXT_FINAL;

                // Final callback, prior to transmission, use this callback to populate the EXTENDED_FRAME fields.
                if (cbs!=NULL && cbs->final_cb) 
                    cbs->final_cb(inst, cbs);    
              
                // Transmit timestamp final report
                uwb_write_tx(inst, frame->array, 0, sizeof(twr_frame_t));
                uwb_write_tx_fctrl(inst, sizeof(twr_frame_t), 0);
                uint64_t final_tx_delay = inst->rxtimestamp + ((uint64_t) g_config.tx_holdoff_delay << 16);
                uwb_set_delay_start(inst, final_tx_delay);

                if (uwb_start_tx(inst).start_tx_error){
                    dpl_sem_release(&rng->sem);
                    if (cbs!=NULL && cbs->start_tx_error_cb) 
                        cbs->start_tx_error_cb(inst, cbs);
                }else{
                    STATS_INC(g_stat, complete); 
                    dpl_sem_release(&rng->sem);
                    struct uwb_mac_interface * cbs = NULL;
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
           
                uwb_stop_rx(inst); // Need to prevent timeout event 
                
                STATS_INC(g_stat, complete);          
                dpl_sem_release(&rng->sem);
                struct uwb_mac_interface * cbs = NULL;
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


