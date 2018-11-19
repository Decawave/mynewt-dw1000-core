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
#include <nranges/nranges.h>
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

static bool reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);
static bool rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);
static bool rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t *);

static dw1000_rng_config_t g_config = {
    .tx_holdoff_delay = MYNEWT_VAL(NRNG_TX_HOLDOFF),         // Send Time delay in usec.
    .rx_timeout_period = MYNEWT_VAL(NRNG_RX_TIMEOUT),       // Receive response timeout in usec
    .tx_guard_delay = MYNEWT_VAL(NRNG_TX_GUARD_DELAY)
};

STATS_SECT_START(nrng_stat_section)
    STATS_SECT_ENTRY(nrng_request)
    STATS_SECT_ENTRY(nrng_listen)
    STATS_SECT_ENTRY(rx_complete)
    STATS_SECT_ENTRY(rx_error)
    STATS_SECT_ENTRY(rx_timeout)
    STATS_SECT_ENTRY(tx_error)
    STATS_SECT_ENTRY(start_tx_error_cb)
    STATS_SECT_ENTRY(rx_unsolicited)
    STATS_SECT_ENTRY(reset)
STATS_SECT_END

STATS_NAME_START(nrng_stat_section)
    STATS_NAME(nrng_stat_section, nrng_request)
    STATS_NAME(nrng_stat_section, nrng_listen)
    STATS_NAME(nrng_stat_section, rx_complete)
    STATS_NAME(nrng_stat_section, rx_error)
    STATS_NAME(nrng_stat_section, rx_timeout)
    STATS_NAME(nrng_stat_section, tx_error)
    STATS_NAME(nrng_stat_section, start_tx_error_cb)
    STATS_NAME(nrng_stat_section, rx_unsolicited)
    STATS_NAME(nrng_stat_section, reset)
STATS_NAME_END(nrng_stat_section)

static STATS_SECT_DECL(nrng_stat_section) g_stat; //!< Stats instance

static dw1000_mac_interface_t g_cbs = {
    .id = DW1000_NRNG,
    .rx_complete_cb = rx_complete_cb,
    .rx_timeout_cb = rx_timeout_cb,
    .reset_cb = reset_cb,
};

dw1000_nrng_instance_t *
dw1000_nrng_init(dw1000_dev_instance_t * inst, dw1000_rng_config_t* config, dw1000_nrng_device_type_t type, uint16_t nframes, uint16_t nnodes){
    assert(inst);

    if (inst->nrng == NULL ) {
        inst->nrng = (dw1000_nrng_instance_t*) malloc(sizeof(dw1000_nrng_instance_t) + nframes * sizeof(nrng_frame_t*)); // struct + flexible array member
        assert(inst->nrng);
        memset(inst->nrng, 0, sizeof(dw1000_nrng_instance_t));
        inst->nrng->status.selfmalloc = 1;
    }

    dw1000_nrng_instance_t *nrng = inst->nrng; // Updating the Global Instance of nrng
    nrng->nframes = nframes;
    nrng->nnodes = nnodes;
    nrng->device_type = type;
    nrng->idx = 0xFFFF;
    nrng->resp_count = nrng->t1_final_flag = 0;
 
    os_error_t err = os_sem_init(&nrng->sem, 0x1);
    assert(err == OS_OK);

    if (config != NULL ){
        dw1000_nrng_config(inst, config);
    }
    int rc = stats_init(
                    STATS_HDR(g_stat),
                    STATS_SIZE_INIT_PARMS(g_stat, STATS_SIZE_32),
                    STATS_NAME_INIT_PARMS(nrng_stat_section)
            );
    rc |= stats_register("nrng", STATS_HDR(g_stat));

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
            .fctrl = FCNTL_IEEE_N_RANGES_16,
            .code = DWT_DS_TWR_NRNG_INVALID
        };
        for (uint16_t i = 0; i < nframes/FRAMES_PER_RANGE; i++)
            for(uint16_t j =0; j < FRAMES_PER_RANGE; j++){
                nrng->frames[i][j] = (nrng_frame_t*)malloc(sizeof(nrng_frame_t));
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
dw1000_nrng_get_config(dw1000_dev_instance_t * inst, dw1000_nrng_modes_t code){

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
            config = &g_config;
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

void dw1000_nrng_pkg_init(void)
{
    dw1000_dev_instance_t * inst = hal_dw1000_inst(0);
    dw1000_nrng_init(inst, &g_config, (dw1000_nrng_device_type_t)MYNEWT_VAL(NRNG_DEVICE_TYPE), MYNEWT_VAL(NRNG_NFRAMES), MYNEWT_VAL(NRNG_NNODES));
    dw1000_nrng_set_frames(inst, MYNEWT_VAL(NRNG_NFRAMES));
    dw1000_mac_append_interface(hal_dw1000_inst(0), &g_cbs);
}

dw1000_dev_status_t 
dw1000_nrng_request_delay_start(dw1000_dev_instance_t * inst, uint16_t dst_address, uint64_t delay, dw1000_nrng_modes_t code, uint16_t start_slot_id, uint16_t end_slot_id){
   
    dw1000_nrng_instance_t * nrng = inst->nrng;    

    dw1000_set_dblrxbuff(inst, true);
    nrng->control.delay_start_enabled = 1;
    nrng->delay = delay;
    dw1000_nrng_request(inst, dst_address, code, start_slot_id, end_slot_id);
    nrng->control.delay_start_enabled = 0;
    dw1000_set_dblrxbuff(inst, false);
    return inst->status;
}

dw1000_dev_status_t
dw1000_nrng_request(dw1000_dev_instance_t * inst, uint16_t dst_address, dw1000_nrng_modes_t code, uint16_t start_slot_id, uint16_t end_slot_id){

    // This function executes on the device that initiates a request
    assert(inst->nrng);
    dw1000_nrng_instance_t * nrng = inst->nrng;
    os_error_t err = os_sem_pend(&nrng->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);
    STATS_INC(g_stat, nrng_request);
    dw1000_rng_config_t * config = dw1000_nrng_get_config(inst, code);
    nrng_frame_t * frame  = nrng->frames[0][FIRST_FRAME_IDX];

    frame->seq_num++;
    inst->nrng->code = frame->code = code;
    frame->src_address = inst->my_short_address;
    frame->dst_address = dst_address;
    frame->start_slot_id = start_slot_id;
    frame->end_slot_id = end_slot_id;
    dw1000_write_tx(inst, frame->array, 0, sizeof(nrng_request_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(nrng_request_frame_t), 0, true);
    dw1000_set_wait4resp(inst, true);
    uint16_t timeout = (((nrng->nnodes)*((dw1000_usecs_to_dwt_usecs(dw1000_phy_frame_duration(&inst->attrib, sizeof(nrng_response_frame_t)))))
                       + (nrng->nnodes - 1)*(config->tx_guard_delay))
                       + config->tx_holdoff_delay         // Remote side turn arroud time.
                       + config->rx_timeout_period);
    dw1000_set_rx_timeout(inst, timeout);

    if (nrng->control.delay_start_enabled)
       dw1000_set_delay_start(inst, nrng->delay);
             
    if (dw1000_start_tx(inst).start_tx_error){
        STATS_INC(g_stat, start_tx_error_cb);
        if(!(SLIST_EMPTY(&inst->interface_cbs))){
            dw1000_mac_interface_t *temp = NULL;
            SLIST_FOREACH(temp, &inst->interface_cbs, next){
                if(temp != NULL && temp->start_tx_error_cb)
                    temp->start_tx_error_cb(inst, temp);
            }
            os_sem_release(&nrng->sem);
        }
    }

    err = os_sem_pend(&nrng->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions
    os_sem_release(&nrng->sem);
    
    return inst->status;
}

/**
 * API to initialise range listen.
 *
 * @param inst          Pointer to dw1000_dev_instance_t.
 *
 * @return dw1000_dev_status_t 
 */
dw1000_dev_status_t
dw1000_nrng_listen(dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode){

    assert(inst);
    dw1000_nrng_instance_t * nrng = inst->nrng;
//    dw1000_set_dblrxbuff(inst, true);
    os_error_t err = os_sem_pend(&nrng->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    STATS_INC(g_stat, nrng_listen);
    if(dw1000_start_rx(inst).start_rx_error){
        STATS_INC(g_stat, rx_error);
        err = os_sem_release(&nrng->sem);
        assert(err == OS_OK);
    }

    if (mode == DWT_BLOCKING){
        err = os_sem_pend(&nrng->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions 
        assert(err == OS_OK);
        err = os_sem_release(&nrng->sem);
        assert(err == OS_OK);
    }
//    dw1000_set_dblrxbuff(inst, false);

   return inst->status;
}

/** 
 * API for reset_cb of rng interface
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 * @return true on sucess
 */
static bool
reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    if(os_sem_get_count(&inst->nrng->sem) == 0){
        STATS_INC(g_stat, reset);
        os_error_t err = os_sem_release(&inst->nrng->sem);
        assert(err == OS_OK);
        return true;
    }
    else
       return false;
}

/** 
 * API for timeout of rng interface
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 * @return true on sucess
 */
static bool
rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    if(os_sem_get_count(&inst->nrng->sem) == 0 && inst->nrng->device_type != DWT_NRNG_INITIATOR){
        STATS_INC(g_stat, rx_timeout);
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
rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    if(inst->fctrl != FCNTL_IEEE_N_RANGES_16){
        return false;
    }
    assert(inst->nrng);
    dw1000_nrng_instance_t * nrng = inst->nrng;
    if(os_sem_get_count(&nrng->sem) == 1){
        // unsolicited inbound
        STATS_INC(g_stat, rx_unsolicited);
        return false;
    }
 
    uint16_t code, dst_address;
    dw1000_read_rx(inst, (uint8_t *) &code, offsetof(nrng_request_frame_t,code), sizeof(uint16_t));
    dw1000_read_rx(inst, (uint8_t *) &dst_address, offsetof(nrng_request_frame_t,dst_address), sizeof(uint16_t));

    // For initiator: Only Allow the packets with dst_address matching with device my_short_address.
    // For responder: Only Allow the packets with dst_address matching with device my_short_address/Broadcast address.
    if (dst_address != inst->my_short_address && (dst_address != BROADCAST_ADDRESS || nrng->device_type == DWT_NRNG_INITIATOR) ){
        return true;
    }
    inst->nrng->code = code;
    switch(code){
        case DWT_SS_TWR_NRNG ... DWT_DS_TWR_NRNG_EXT_END:
            STATS_INC(g_stat, rx_complete);
            return false;
        default:
            return true;
    }
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
            wcs_instance_t * wcs = inst->ccp->wcs;
            float skew = wcs->skew;
#else
            float skew = dw1000_calc_clock_offset_ratio(inst, first_frame->carrier_integrator);
#endif
            ToF = ((first_frame->response_timestamp - first_frame->request_timestamp)
                    -  (first_frame->transmission_timestamp - first_frame->reception_timestamp) * (1 - skew))/2.;
            break;
            }
        default: break;
    }
    return ToF;
}

