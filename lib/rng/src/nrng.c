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
#include <rng/nrng.h>
#include <rng/rng.h>
#if MYNEWT_VAL(TWR_DS_NRNG_ENABLED)
#include <twr_ds_nrng/twr_ds_nrng.h>
#endif
#if MYNEWT_VAL(TWR_SS_NRNG_ENABLED)
#include <twr_ss_nrng/twr_ss_nrng.h>
#endif
#if MYNEWT_VAL(TWR_DS_EXT_NRNG_ENABLED)
#include <twr_ds_ext_nrng/twr_ds_ext_nrng.h>
#endif
#if MYNEWT_VAL(WCS_ENABLED)
#include <wcs/wcs.h>
#endif
#if MYNEWT_VAL(CCP_ENABLED)
#include <ccp/ccp.h>
#endif
#include <rng/slots.h>
#if MYNEWT_VAL(NRNG_VERBOSE)
#include <rng/nrng_encode.h>
#endif

#if MYNEWT_VAL(NRNG_VERBOSE)
static bool complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
#endif

static dw1000_mac_interface_t g_cbs[] = {
        [0] = {
            .id = DW1000_NRNG,
#if MYNEWT_VAL(NRNG_VERBOSE)
            .complete_cb  = complete_cb,
#endif
        },
#if MYNEWT_VAL(DW1000_DEVICE_1)
        [1] = {
            .id = DW1000_NRNG,
#if MYNEWT_VAL(NRNG_VERBOSE)
            .complete_cb  = complete_cb,
#endif
        },
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
        [2] = {
            .id = DW1000_NRNG,
#if MYNEWT_VAL(NRNG_VERBOSE)
            .complete_cb  = complete_cb,
#endif
        }
#endif
};


static dw1000_rng_config_t g_config = {
    .tx_holdoff_delay = MYNEWT_VAL(NRNG_TX_HOLDOFF),         // Send Time delay in usec.
    .rx_timeout_delay = MYNEWT_VAL(NRNG_RX_TIMEOUT),       // Receive response timeout in usec
    .tx_guard_delay = MYNEWT_VAL(NRNG_TX_GUARD_DELAY)
};

STATS_NAME_START(nrng_stat_section)
    STATS_NAME(nrng_stat_section, nrng_request)
    STATS_NAME(nrng_stat_section, rx_error)
    STATS_NAME(nrng_stat_section, rx_timeout)
    STATS_NAME(nrng_stat_section, tx_error)
    STATS_NAME(nrng_stat_section, start_tx_error_cb)
    STATS_NAME(nrng_stat_section, rx_unsolicited)
    STATS_NAME(nrng_stat_section, reset)
STATS_NAME_END(nrng_stat_section)


dw1000_nrng_instance_t *
dw1000_nrng_init(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config, dw1000_nrng_device_type_t type, uint16_t nframes, uint16_t nnodes){
    assert(inst);

    if (inst->nrng == NULL ) {
        inst->nrng = (dw1000_nrng_instance_t*) malloc(sizeof(dw1000_nrng_instance_t) + nframes * sizeof(nrng_frame_t * )); // struct + flexible array member
        assert(inst->nrng);
        memset(inst->nrng, 0, sizeof(dw1000_nrng_instance_t));
        inst->nrng->status.selfmalloc = 1;
    }

    dw1000_nrng_instance_t *nrng = inst->nrng; // Updating the Global Instance of nrng
    nrng->parent = inst;
    nrng->nframes = nframes;
    nrng->nnodes = nnodes;
    nrng->device_type = type;
    nrng->idx = 0xFFFF;
    nrng->resp_count = nrng->t1_final_flag = 0;
    nrng->seq_num = 0;
 
    if (config != NULL ){
        dw1000_nrng_config(inst, config);
    }
 
    int rc = stats_init(
                    STATS_HDR(inst->nrng->stat),
                    STATS_SIZE_INIT_PARMS(inst->nrng->stat, STATS_SIZE_32),
                    STATS_NAME_INIT_PARMS(nrng_stat_section)
            );
   
#if  MYNEWT_VAL(DW1000_DEVICE_0) && !MYNEWT_VAL(DW1000_DEVICE_1)
        rc |= stats_register("nrng", STATS_HDR(inst->nrng->stat));
#elif  MYNEWT_VAL(DW1000_DEVICE_0) && MYNEWT_VAL(DW1000_DEVICE_1)
    if (inst == hal_dw1000_inst(0))
        rc |= stats_register("nrng0", STATS_HDR(inst->nrng->stat));
    else
        rc |= stats_register("nrng1", STATS_HDR(inst->nrng->stat));
#endif
    assert(rc == 0);

    return nrng;
}

/**
 * API to free the allocated resources.
 *
 * @param inst  Pointer to dw1000_rng_instance_t.
 *
 * @return void 
 */
void
dw1000_nrng_free(dw1000_nrng_instance_t * inst){

    assert(inst);
    if (inst->status.selfmalloc){
        for(int i =0; i< inst->nframes/FRAMES_PER_RANGE; i++)
            for(int j = 0; j< FRAMES_PER_RANGE; j++){
                free(inst->frames[i][j]);
                inst->frames[i][j] = NULL;
            }
        free(inst);
    }
    else
        inst->status.initialized = 0;
}

inline void
dw1000_nrng_set_frames(dw1000_dev_instance_t * inst, uint16_t nframes){
        assert(inst);
        dw1000_nrng_instance_t * nrng = inst->nrng;
        assert(nframes <= nrng->nframes);
        nrng_frame_t default_frame = {
            .PANID = 0xDECA,
            .fctrl = FCNTL_IEEE_RANGE_16,
            .code = DWT_DS_TWR_NRNG_INVALID
        };
        for (uint16_t i = 0; i < nframes/FRAMES_PER_RANGE; i++)
            for(uint16_t j =0; j < FRAMES_PER_RANGE; j++){
                nrng->frames[i][j] = (nrng_frame_t * ) malloc(sizeof(nrng_frame_t));
                memcpy(nrng->frames[i][j], &default_frame, sizeof(nrng_frame_t));
            }
}

/**
 * API to configure dw1000 to start transmission after certain delay.
 *
 * @param inst          Pointer to dw1000_dev_instance_t. 
 * @param dst_address   Address of the receiver to whom range request to be sent. 
 * @param delay         Time until which request has to be resumed. 
 * @param code          Represents mode of ranging DWT_SS_TWR enables single sided two way ranging DWT_DS_TWR enables double sided 
 * two way ranging DWT_DS_TWR_EXT enables double sided two way ranging with extended frame.
 *
 * @return dw1000_dev_status_t
 */
dw1000_rng_config_t *
dw1000_nrng_get_config(dw1000_dev_instance_t * inst, dw1000_rng_modes_t code){

    dw1000_rng_config_t * config;

    switch (code){
#if MYNEWT_VAL(TWR_DS_NRNG_ENABLED) 
        case  DWT_DS_TWR_NRNG:                     //!< Double sided TWR
            config = twr_ds_nrng_config(inst);
            break;
#endif
#if MYNEWT_VAL(TWR_DS_EXT_NRNG_ENABLED) 
        case DWT_DS_TWR_NRNG_EXT:                  //!< Double sided TWR in extended mode 
            config = twr_ds_ext_nrng_config(inst);
            break;
#endif
#if MYNEWT_VAL(TWR_SS_NRNG_ENABLED) 
        case  DWT_SS_TWR_NRNG:                     //!< Single sided TWR
            config = twr_ss_nrng_config(inst);
            break;
#endif
        default:
            assert(0);
    }
    return config;
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
dw1000_nrng_config(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config){
    assert(inst);
    assert(config);

    memcpy(&inst->nrng->config, config, sizeof(dw1000_rng_config_t));
    return inst->status;
}

void nrng_pkg_init(void){

    printf("{\"utime\": %lu,\"msg\": \"nrng_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

#if MYNEWT_VAL(DW1000_DEVICE_0)
    dw1000_nrng_init(hal_dw1000_inst(0), &g_config, (dw1000_nrng_device_type_t) MYNEWT_VAL(NRNG_DEVICE_TYPE), MYNEWT_VAL(NRNG_NFRAMES), MYNEWT_VAL(NRNG_NNODES));
    dw1000_nrng_set_frames(hal_dw1000_inst(0), MYNEWT_VAL(NRNG_NFRAMES));
    dw1000_mac_append_interface(hal_dw1000_inst(0), &g_cbs[0]);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    dw1000_nrng_init(hal_dw1000_inst(1), &g_config, (dw1000_nrng_device_type_t) MYNEWT_VAL(NRNG_DEVICE_TYPE), MYNEWT_VAL(NRNG_NFRAMES), MYNEWT_VAL(NRNG_NNODES));
    dw1000_nrng_set_frames(hal_dw1000_inst(1), MYNEWT_VAL(NRNG_NFRAMES));
    dw1000_mac_append_interface(hal_dw1000_inst(1), &g_cbs[1]);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    dw1000_nrng_init(hal_dw1000_inst(2), &g_config, (dw1000_nrng_device_type_t) MYNEWT_VAL(NRNG_DEVICE_TYPE), MYNEWT_VAL(NRNG_NFRAMES), MYNEWT_VAL(NRNG_NNODES));
    dw1000_nrng_set_frames(hal_dw1000_inst(2), MYNEWT_VAL(NRNG_NFRAMES));
    dw1000_mac_append_interface(hal_dw1000_inst(2), &g_cbs[2]);
#endif

}

dw1000_dev_status_t 
dw1000_nrng_request_delay_start(dw1000_dev_instance_t * inst, uint16_t dst_address, uint64_t delay, dw1000_rng_modes_t code, uint16_t slot_mask, uint16_t cell_id){
    dw1000_nrng_instance_t * nrng = inst->nrng;   
   
    nrng->control.delay_start_enabled = 1;
    nrng->delay = delay;
    dw1000_nrng_request(inst, dst_address, code, slot_mask, cell_id);
    nrng->control.delay_start_enabled = 0;

    return inst->status;
}

/** 
 * Help function to calculate the delay between cascading requests
 * 
 * @param inst Pointer to dw1000_dev_instance_t * 
 * @param slot_id 0 for master, and increasing
 * @return void
 */
static inline uint32_t
usecs_to_response(dw1000_dev_instance_t * inst, uint16_t nslots, dw1000_rng_config_t * config, uint32_t duration){
    uint32_t ret = nslots * ( duration + (uint32_t) dw1000_dwt_usecs_to_usecs(config->tx_guard_delay));
    return ret;
}

dw1000_dev_status_t
dw1000_nrng_request(dw1000_dev_instance_t * inst, uint16_t dst_address, dw1000_rng_modes_t code, uint16_t slot_mask, uint16_t cell_id){

    // This function executes on the device that initiates a request
    assert(inst->nrng);
    
    dw1000_rng_instance_t * rng = inst->rng;
    dw1000_nrng_instance_t * nrng = inst->nrng;

    os_error_t err = os_sem_pend(&rng->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);
    STATS_INC(nrng->stat, nrng_request);

    dw1000_rng_config_t * config = dw1000_nrng_get_config(inst, code);
    nrng->nnodes = NumberOfBits(slot_mask); // Number of nodes involved in request
    nrng->idx += nrng->nnodes;
    nrng_request_frame_t * frame = (nrng_request_frame_t *) nrng->frames[nrng->idx%(nrng->nframes/FRAMES_PER_RANGE)][FIRST_FRAME_IDX];

    frame->seq_num = ++nrng->seq_num;
    inst->rng->code = frame->code = code;
    frame->src_address = inst->my_short_address;
    frame->dst_address = dst_address;
   
#if MYNEWT_VAL(CELL_ENABLED)
    frame->ptype = PTYPE_CELL;
    frame->cell_id = nrng->cell_id = cell_id;
    frame->slot_mask = nrng->slot_mask = slot_mask;
#else
    frame->ptype = PTYPE_RANGE;
    frame->end_slot_id = cell_id;
    frame->start_slot_id = slot_mask;
#endif
   
    dw1000_write_tx(inst, frame->array, 0, sizeof(nrng_request_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(nrng_request_frame_t), 0, true);
    dw1000_set_wait4resp(inst, true);

    uint16_t timeout = config->tx_holdoff_delay         // Remote side turn arround time.
                        + usecs_to_response(inst,       // Aggregated timeout of all responses
                            nrng->nnodes,               // no. of expected frames
                            config,                    
                            dw1000_phy_frame_duration(&inst->attrib, sizeof(nrng_response_frame_t)) // in usec
                        ) 
                        + config->rx_timeout_delay;     // TOF allowance.
   
    dw1000_set_rx_timeout(inst, timeout);

    if (nrng->control.delay_start_enabled)
        dw1000_set_delay_start(inst, nrng->delay); 
    
    dw1000_set_dblrxbuff(inst, true);  
    
    if (dw1000_start_tx(inst).start_tx_error){
        STATS_INC(nrng->stat, start_tx_error_cb);
        if (os_sem_get_count(&rng->sem) == 0) 
            os_sem_release(&rng->sem);
    }

    err = os_sem_pend(&rng->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions
    assert(err == OS_OK);
    err = os_sem_release(&rng->sem);
    assert(err == OS_OK);

    // dw1000_set_dblrxbuff(inst, false);
    
    return inst->status;
}


float
dw1000_nrng_twr_to_tof_frames(struct _dw1000_dev_instance_t * inst, nrng_frame_t *first_frame, nrng_frame_t *final_frame){
    float ToF = 0;
    uint64_t T1R, T1r, T2R, T2r;
    int64_t nom,denom;

    switch(final_frame->code){
        case DWT_DS_TWR_NRNG ... DWT_DS_TWR_NRNG_END:
        case DWT_DS_TWR_NRNG_EXT ... DWT_DS_TWR_NRNG_EXT_END:
            assert(first_frame != NULL);
            assert(final_frame != NULL);
            T1R = (first_frame->response_timestamp - first_frame->request_timestamp);
            T1r = (first_frame->transmission_timestamp  - first_frame->reception_timestamp);
            T2R = (final_frame->response_timestamp - final_frame->request_timestamp);
            T2r = (final_frame->transmission_timestamp - final_frame->reception_timestamp);
            nom = T1R * T2R  - T1r * T2r;
            denom = T1R + T2R  + T1r + T2r;
            ToF = (float) (nom) / denom;
            break;
        case DWT_SS_TWR_NRNG ... DWT_SS_TWR_NRNG_FINAL:{
            assert(first_frame != NULL);
#if MYNEWT_VAL(WCS_ENABLED)
            ToF = ((first_frame->response_timestamp - first_frame->request_timestamp)
                    -  (first_frame->transmission_timestamp - first_frame->reception_timestamp))/2.;
#else
            float skew = dw1000_calc_clock_offset_ratio(inst, first_frame->carrier_integrator);
            ToF = ((first_frame->response_timestamp - first_frame->request_timestamp)
                    -  (first_frame->transmission_timestamp - first_frame->reception_timestamp) * (1 - skew))/2.;
#endif
            break;
            }
        default: break;
    }
    return ToF;
}


#if MYNEWT_VAL(NRNG_VERBOSE)

static void
complete_ev_cb(struct os_event *ev) {
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    nrng_encode(inst->nrng, inst->nrng->seq_num, inst->nrng->idx);
}

struct os_callout nrng_callout;
static bool
complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){

        if (inst->fctrl != FCNTL_IEEE_RANGE_16)
        return false;

        os_callout_init(&nrng_callout, os_eventq_dflt_get(), complete_ev_cb, inst);
        os_eventq_put(os_eventq_dflt_get(), &nrng_callout.c_ev);
        return false;
}


#endif
