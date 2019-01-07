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

#include <stats/stats.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_ftypes.h>
#include <rng/rng.h>
#include <dsp/polyval.h>

#if MYNEWT_VAL(WCS_ENABLED)
#include <wcs/wcs.h>
#endif

#if MYNEWT_VAL(RNG_VERBOSE)
#define DIAGMSG(s,u) printf(s,u)
#endif
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

static bool rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);
static bool reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);
static bool start_tx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);

static dw1000_mac_interface_t g_cbs[] = {
        [0] = {
            .id = DW1000_RNG_SS,
            .rx_complete_cb = rx_complete_cb,
            .start_tx_error_cb = start_tx_error_cb,
            .reset_cb = reset_cb
        },
#if MYNEWT_VAL(DW1000_DEVICE_1)
        [1] = {
            .id = DW1000_RNG_SS,
            .rx_complete_cb = rx_complete_cb,
            .start_tx_error_cb = start_tx_error_cb,
            .reset_cb = reset_cb
        },
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
        [2] = {
            .rx_complete_cb = rx_complete_cb,
            .start_tx_error_cb = start_tx_error_cb,
            .reset_cb = reset_cb
        }
#endif
};


STATS_SECT_START(twr_ss_stat_section)
    STATS_SECT_ENTRY(complete)
    STATS_SECT_ENTRY(tx_error)
    STATS_SECT_ENTRY(reset)
STATS_SECT_END

STATS_NAME_START(twr_ss_stat_section)
    STATS_NAME(twr_ss_stat_section, complete)
    STATS_NAME(twr_ss_stat_section, tx_error)
    STATS_NAME(twr_ss_stat_section, reset)
STATS_NAME_END(twr_ss_stat_section)

static STATS_SECT_DECL(twr_ss_stat_section) g_stat;

static dw1000_rng_config_t g_config = {
    .tx_holdoff_delay = MYNEWT_VAL(TWR_SS_TX_HOLDOFF),         // Send Time delay in usec.
    .rx_timeout_delay = MYNEWT_VAL(TWR_SS_RX_TIMEOUT)       // Receive response timeout in usec
};

/**
 * API to initialise the rng_ss package.
 *
 *
 * @return void
 */

void twr_ss_pkg_init(void){

    printf("{\"utime\": %lu,\"msg\": \"twr_ss_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

#if MYNEWT_VAL(DW1000_DEVICE_0)
    dw1000_mac_append_interface(hal_dw1000_inst(0), &g_cbs[0]);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    dw1000_mac_append_interface(hal_dw1000_inst(1), &g_cbs[1]);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    dw1000_mac_append_interface(hal_dw1000_inst(1), &g_cbs[2]);
#endif

    int rc = stats_init(
    STATS_HDR(g_stat),
    STATS_SIZE_INIT_PARMS(g_stat, STATS_SIZE_32),
    STATS_NAME_INIT_PARMS(twr_ss_stat_section));
    rc |= stats_register("twr_ss", STATS_HDR(g_stat));
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
twr_ss_free(dw1000_dev_instance_t * inst){
    assert(inst); 
    dw1000_mac_remove_interface(inst, DW1000_RNG_SS);
}


/**
 * API for get local config callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
dw1000_rng_config_t * 
twr_ss_config(dw1000_dev_instance_t * inst){
    return &g_config;
}



/**
 * API for start tx error callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
static bool 
start_tx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    STATS_INC(g_stat, tx_error);
    return true;
}


/** 
 * API for reset_cb of rng interface
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 * @return true on sucess
 */
static bool
reset_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){

    if(os_sem_get_count(&inst->rng->sem) == 0){
        STATS_INC(g_stat, reset);
        os_error_t err = os_sem_release(&inst->rng->sem);  
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
rx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{    
    if (inst->fctrl != FCNTL_IEEE_RANGE_16)
        return false;
        
    if(os_sem_get_count(&inst->rng->sem) == 1) // unsolicited inbound
        return false;
    
    dw1000_rng_instance_t * rng = inst->rng; 
    twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes]; // Frame already read within loader layers.

    switch(inst->rng->code){
        case DWT_SS_TWR:
            {
                // This code executes on the device that is responding to a request

                uint64_t request_timestamp = inst->rxtimestamp;
                uint64_t response_tx_delay = request_timestamp + ((uint64_t) g_config.tx_holdoff_delay << 16);
                uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;

#if MYNEWT_VAL(WCS_ENABLED) 
                frame->reception_timestamp = wcs_local_to_master(inst, request_timestamp) & 0xFFFFFFFFUL;
                frame->transmission_timestamp = wcs_local_to_master(inst, response_timestamp) & 0xFFFFFFFFUL;
#else
                frame->reception_timestamp = request_timestamp & 0xFFFFFFFFUL;
                frame->transmission_timestamp = response_timestamp & 0xFFFFFFFFUL;
#endif

                frame->dst_address = frame->src_address;
                frame->src_address = inst->my_short_address;
                frame->code = DWT_SS_TWR_T1;
                
#if MYNEWT_VAL(WCS_ENABLED)
                frame->carrier_integrator  = 0.0l;
#else
                frame->carrier_integrator  = - inst->carrier_integrator;
#endif
               // Write the second part of the response
                dw1000_write_tx(inst, frame->array ,0 ,sizeof(ieee_rng_response_frame_t));
                dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_response_frame_t), 0, true); 
                dw1000_set_wait4resp(inst, true);   

                uint16_t timeout = dw1000_phy_frame_duration(&inst->attrib, sizeof(ieee_rng_response_frame_t)) 
                                        + g_config.rx_timeout_delay        
                                        + g_config.tx_holdoff_delay;         // Remote side turn arroud time. 

                dw1000_set_delay_start(inst, response_tx_delay);
                dw1000_set_rx_timeout(inst, timeout); 

                if (dw1000_start_tx(inst).start_tx_error){
                    os_sem_release(&rng->sem);  
                    if (cbs!=NULL && cbs->start_tx_error_cb) 
                        cbs->start_tx_error_cb(inst, cbs);
                }
                break;
            }
        case DWT_SS_TWR_T1:
            {
                // This code executes on the device that initiated a request, and is now preparing the final timestamps
                if (inst->frame_len != sizeof(ieee_rng_response_frame_t))
                    break;

                if(inst->status.lde_error)
                    break;

                uint64_t response_timestamp = inst->rxtimestamp;
#if MYNEWT_VAL(WCS_ENABLED)           
                frame->request_timestamp = wcs_local_to_master(inst, dw1000_read_txtime(inst)) & 0xFFFFFFFFUL;
                frame->response_timestamp = wcs_local_to_master(inst, response_timestamp) & 0xFFFFFFFFUL;
#else
                frame->request_timestamp = dw1000_read_txtime_lo(inst) & 0xFFFFFFFFUL;
                frame->response_timestamp  = (uint32_t)(response_timestamp & 0xFFFFFFFFUL);
#endif
                frame->dst_address = frame->src_address;
                frame->src_address = inst->my_short_address;
                frame->code = DWT_SS_TWR_FINAL;
#if MYNEWT_VAL(WCS_ENABLED)
                frame->carrier_integrator  = 0.0l;
#else
                frame->carrier_integrator  = inst->carrier_integrator;
#endif              
                // Transmit timestamp final report
                dw1000_write_tx(inst, frame->array, 0,  sizeof(twr_frame_final_t));
                dw1000_write_tx_fctrl(inst, sizeof(twr_frame_final_t), 0, true);
                if (dw1000_start_tx(inst).start_tx_error){
                    os_sem_release(&rng->sem);  
                    if (cbs!=NULL && cbs->start_tx_error_cb) 
                        cbs->start_tx_error_cb(inst, cbs);
                }
                else{    
                    STATS_INC(g_stat, complete);
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
        case  DWT_SS_TWR_FINAL:
            {
                // This code executes on the device that responded to the original request, and has now receive the response final timestamp. 
                // This marks the completion of the single-size-two-way request. This final 4th message is perhaps optional in some applicaiton. 

                if (inst->frame_len != sizeof(twr_frame_final_t))
                   break;

                STATS_INC(g_stat, complete);
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




