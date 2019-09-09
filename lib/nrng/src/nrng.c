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

#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_ftypes.h>
#include <nrng/nrng.h>
#include <rng/rng.h>
#if MYNEWT_VAL(WCS_ENABLED)
#include <wcs/wcs.h>
#endif
#if MYNEWT_VAL(CIR_ENABLED)
#include <cir/cir.h>
#endif
#include <rng/slots.h>
#if MYNEWT_VAL(NRNG_VERBOSE)
#include <nrng/nrng_encode.h>
static bool complete_cb(struct uwb_dev * udev, struct uwb_mac_interface * cbs);
#endif

static dw1000_rng_config_t g_config = {
    .tx_holdoff_delay = MYNEWT_VAL(NRNG_TX_HOLDOFF),         // Send Time delay in usec.
    .rx_timeout_delay = MYNEWT_VAL(NRNG_RX_TIMEOUT),       // Receive response timeout in usec
    .tx_guard_delay = MYNEWT_VAL(NRNG_TX_GUARD_DELAY)
};

#if MYNEWT_VAL(NRNG_STATS)
STATS_NAME_START(nrng_stat_section)
    STATS_NAME(nrng_stat_section, nrng_request)
    STATS_NAME(nrng_stat_section, nrng_listen)
    STATS_NAME(nrng_stat_section, rx_complete)
    STATS_NAME(nrng_stat_section, rx_error)
    STATS_NAME(nrng_stat_section, rx_timeout)
    STATS_NAME(nrng_stat_section, complete)
    STATS_NAME(nrng_stat_section, start_rx_error)
    STATS_NAME(nrng_stat_section, rx_unsolicited)
    STATS_NAME(nrng_stat_section, tx_error)
    STATS_NAME(nrng_stat_section, start_tx_error)
    STATS_NAME(nrng_stat_section, reset)
STATS_NAME_END(nrng_stat_section)
#endif

/**
 * @fn dw1000_nrng_init(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config, dw1000_nrng_device_type_t type, uint16_t nframes, uint16_t nnodes)
 * @brief API to initialize nrng parameters and configuration.
 *
 * @param inst      Pointer to dw1000_dev_instance_t.
 * @param config    Pointer to the structure dw1000_rng_config_t.
 * @param type      dw1000_rng_device_type_t of DWT_NRNG_INITIATOR or DWT_NRNG_RESPONDER.
 * @param nframes   Number of buffers defined to store the ranging data.
 * @param nnodes    number of nodes.
 *
 * @return dw1000_nrng_instance_t pointer
 */
dw1000_nrng_instance_t *
dw1000_nrng_init(struct uwb_dev * inst, dw1000_rng_config_t * config,
                 dw1000_nrng_device_type_t type, uint16_t nframes, uint16_t nnodes)
{
    assert(inst);

    dw1000_nrng_instance_t *nrng = (dw1000_nrng_instance_t*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_NRNG);
    if (nrng == NULL) {
        nrng = (dw1000_nrng_instance_t*) malloc(sizeof(dw1000_nrng_instance_t) + nframes * sizeof(nrng_frame_t * )); 
        assert(nrng);
        memset(nrng, 0, sizeof(dw1000_nrng_instance_t));
        nrng->status.selfmalloc = 1;
    }
    dpl_error_t err = dpl_sem_init(&nrng->sem, 0x1); 
    assert(err == DPL_OK);

    nrng->dev_inst = inst;
    nrng->nframes = nframes;
    nrng->nnodes = nnodes;
    nrng->device_type = type;
    nrng->idx = 0xFFFF;
    nrng->resp_count = nrng->t1_final_flag = 0;
    nrng->seq_num = 0;
    
    if (config != NULL ){
        dw1000_nrng_config(nrng, config);
    }
    nrng->cbs = (struct uwb_mac_interface){
        .id = UWBEXT_NRNG,
        .inst_ptr = nrng,
#if MYNEWT_VAL(NRNG_VERBOSE)
        .complete_cb = complete_cb,
#endif
    };
    uwb_mac_append_interface(inst, &nrng->cbs);

#if MYNEWT_VAL(NRNG_STATS)
    int rc = stats_init(
                    STATS_HDR(nrng->stat),
                    STATS_SIZE_INIT_PARMS(nrng->stat, STATS_SIZE_32),
                    STATS_NAME_INIT_PARMS(nrng_stat_section)
            );

#if  MYNEWT_VAL(DW1000_DEVICE_0) && !MYNEWT_VAL(DW1000_DEVICE_1)
        rc |= stats_register("nrng", STATS_HDR(nrng->stat));
#elif  MYNEWT_VAL(DW1000_DEVICE_0) && MYNEWT_VAL(DW1000_DEVICE_1)
    if (inst->idx == 0)
        rc |= stats_register("nrng0", STATS_HDR(nrng->stat));
    else
        rc |= stats_register("nrng1", STATS_HDR(nrng->stat));
#endif
    assert(rc == 0);
#endif
    return nrng;
}

/**
 * @fn dw1000_nrng_free(dw1000_nrng_instance_t * inst)
 * @brief API to free the allocated resources.
 *
 * @param inst  Pointer to dw1000_rng_instance_t.
 *
 * @return void
 */
void
dw1000_nrng_free(dw1000_nrng_instance_t * inst)
{
    assert(inst);
    uwb_mac_remove_interface(inst->dev_inst, inst->cbs.id);

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

/**
 * @fn dw1000_nrng_set_frames(dw1000_dev_instance_t * inst, uint16_t nframes)
 * @brief API to set the pointer to the nrng frame buffers.
 *
 * @param inst      Pointer to dw1000_dev_instance_t.
 * @param nframes   Number of buffers defined to store the ranging data.
 *
 * @return void
 */
inline void
dw1000_nrng_set_frames(dw1000_nrng_instance_t * nrng, uint16_t nframes)
{
    assert(nframes <= nrng->nframes);
    nrng_frame_t default_frame = {
        .PANID = 0xDECA,
        .fctrl = FCNTL_IEEE_RANGE_16,
        .code = DWT_DS_TWR_NRNG_INVALID
    };
    for (uint16_t i = 0; i < nframes; i++){
        nrng->frames[i] = (nrng_frame_t * ) malloc(sizeof(nrng_frame_t));
        assert(nrng->frames[i]);
        memcpy(nrng->frames[i], &default_frame, sizeof(nrng_frame_t));
    }
}

/**
 * API to configure dw1000 to start transmission after certain delay.
 *
 * @param inst          Pointer to dw1000_dev_instance_t. 
 * @param ranges        []] to return results  
 * @param nranges       side of  ranges[]
 * @param code          base address of curcular buffer
 *
 * @return valid mask
 */
uint32_t
dw1000_nrng_get_ranges(dw1000_nrng_instance_t * nrng, float ranges[], uint16_t nranges, uint16_t base)
{
    uint32_t mask = 0;

    // Which slots responded with a valid frames
    for (uint16_t i=0; i < nranges; i++){
        if (nrng->slot_mask & 1UL << i){
            // the set of all requested slots
            uint16_t idx = BitIndex(nrng->slot_mask, 1UL << i, SLOT_POSITION); 
            nrng_frame_t * frame = nrng->frames[(base + idx)%nrng->nframes];
            if (frame->code == DWT_SS_TWR_NRNG_FINAL && frame->seq_num == nrng->seq_num){
                // the set of all positive responses
                mask |= 1UL << i;
            }
        }
    }
    // Construct output vector 
    uint16_t j = 0;
    for (uint16_t i=0; i < nranges; i++){
        if (mask & 1UL << i){
            uint16_t idx = BitIndex(nrng->slot_mask, 1UL << i, SLOT_POSITION); 
            nrng_frame_t * frame = nrng->frames[(base + idx)%nrng->nframes];
            ranges[j++] = dw1000_rng_tof_to_meters(dw1000_nrng_twr_to_tof_frames(nrng->dev_inst, frame, frame));
        }
    }
    return mask;
}

/**
 * @fn dw1000_nrng_get_config(dw1000_dev_instance_t * inst, dw1000_rng_modes_t code)
 * @brief API to get configuration using dw1000_rng_modes_t.
 *
 * @param inst          Pointer to dw1000_dev_instance_t.
 * @param code          Represents mode of ranging DWT_SS_TWR enables single sided two way ranging DWT_DS_TWR enables double sided
 * two way ranging DWT_DS_TWR_EXT enables double sided two way ranging with extended frame.
 *
 * @return dw1000_rng_config_t
 */
dw1000_rng_config_t *
dw1000_nrng_get_config(dw1000_nrng_instance_t * nrng, dw1000_rng_modes_t code)
{
    struct rng_config_list * cfgs;

    if(!(SLIST_EMPTY(&nrng->rng_configs))){ 
        SLIST_FOREACH(cfgs, &nrng->rng_configs, next){    
            if (cfgs != NULL && cfgs->rng_code == code) {
                return cfgs->config;
            }
        }
    }
    return &g_config;
}

/**
 * Add config extension for different rng services.
 *
 * @param rng        Pointer to dw1000_rng_instance_t.
 * @param callbacks  callback instance.
 * @return void
 */
void
dw1000_nrng_append_config(dw1000_nrng_instance_t * nrng, struct rng_config_list *cfgs)
{
    assert(nrng);

    if(!(SLIST_EMPTY(&nrng->rng_configs))) {
        struct rng_config_list * prev_cfgs = NULL;
        struct rng_config_list * cur_cfgs = NULL;
        SLIST_FOREACH(cur_cfgs, &nrng->rng_configs, next){
            prev_cfgs = cur_cfgs;
        }
        SLIST_INSERT_AFTER(prev_cfgs, cfgs, next);
    } else {
        SLIST_INSERT_HEAD(&nrng->rng_configs, cfgs, next);
    }
}


/**
 * API to remove config.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @param id    ID of the service.
 * @return void
 */
void
dw1000_nrng_remove_config(dw1000_nrng_instance_t * nrng, dw1000_rng_modes_t code)
{
    assert(nrng);
    struct rng_config_list * cfgs = NULL;
    SLIST_FOREACH(cfgs, &nrng->rng_configs, next){
        if(cfgs->rng_code == code){
            SLIST_REMOVE(&nrng->rng_configs, cfgs, rng_config_list, next);
            break;
        }
    }
}

/**
 * @fn dw1000_nrng_config(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config)
 * @brief API to assign the config parameters to range instance.
 *
 * @param inst    Pointer to dw1000_dev_instance_t.
 * @param config  Pointer to dw1000_rng_config_t.
 *
 * @return struct uwb_dev_status
 */
struct uwb_dev_status
dw1000_nrng_config(dw1000_nrng_instance_t * nrng, dw1000_rng_config_t * config)
{
    assert(config);

    memcpy(&nrng->config, config, sizeof(dw1000_rng_config_t));
    return nrng->dev_inst->status;
}


void nrng_pkg_init(void)
{
#if MYNEWT_VAL(DW1000_PKG_INIT_LOG)
    printf("{\"utime\": %lu,\"msg\": \"nrng_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
#endif

    dw1000_nrng_instance_t *nrng;
    struct uwb_dev *udev;
#if MYNEWT_VAL(DW1000_DEVICE_0)
    udev = uwb_dev_idx_lookup(0);
    nrng = dw1000_nrng_init(udev, &g_config, (dw1000_nrng_device_type_t) MYNEWT_VAL(NRNG_DEVICE_TYPE), MYNEWT_VAL(NRNG_NFRAMES), MYNEWT_VAL(NRNG_NNODES));
    dw1000_nrng_set_frames(nrng, MYNEWT_VAL(NRNG_NFRAMES));
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    udev = uwb_dev_idx_lookup(1);
    nrng = dw1000_nrng_init(udev, &g_config, (dw1000_nrng_device_type_t) MYNEWT_VAL(NRNG_DEVICE_TYPE), MYNEWT_VAL(NRNG_NFRAMES), MYNEWT_VAL(NRNG_NNODES));
    dw1000_nrng_set_frames(nrng, MYNEWT_VAL(NRNG_NFRAMES));
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    udev = uwb_dev_idx_lookup(2);
    nrng = dw1000_nrng_init(udev, &g_config, (dw1000_nrng_device_type_t) MYNEWT_VAL(NRNG_DEVICE_TYPE), MYNEWT_VAL(NRNG_NFRAMES), MYNEWT_VAL(NRNG_NNODES));
    dw1000_nrng_set_frames(nrng, MYNEWT_VAL(NRNG_NFRAMES));
#endif

}

/**
 * @fn dw1000_nrng_request_delay_start(dw1000_dev_instance_t * inst, uint16_t dst_address, uint64_t delay, dw1000_rng_modes_t code, uint16_t slot_mask, uint16_t cell_id)
 * @brief API to configure dw1000 to start transmission after certain delay.
 *
 * @param inst          Pointer to dw1000_dev_instance_t.
 * @param dst_address   Address of the receiver to whom range request to be sent.
 * @param delay         Time until which request has to be resumed.
 * @param code          Represents mode of ranging DWT_SS_TWR enables single sided two way ranging DWT_DS_TWR enables double sided
 * two way ranging DWT_DS_TWR_EXT enables double sided two way ranging with extended frame.
 * @param slot_mast     nrng_request_frame_t of masked slot number
 * @param cell_id       nrng_request_frame_t of cell id number
 * @return struct uwb_dev_status
 */
struct uwb_dev_status
dw1000_nrng_request_delay_start(dw1000_nrng_instance_t * nrng, uint16_t dst_address, uint64_t delay,
                                dw1000_rng_modes_t code, uint16_t slot_mask, uint16_t cell_id)
{
    nrng->control.delay_start_enabled = 1;
    nrng->delay = delay;
    dw1000_nrng_request(nrng, dst_address, code, slot_mask, cell_id);
    nrng->control.delay_start_enabled = 0;

    return nrng->dev_inst->status;
}

/**
 * @fn usecs_to_response(struct uwb_dev * inst, uint16_t nslots, dw1000_rng_config_t * config, uint32_t duration)
 * @brief Help function to calculate the delay between cascading requests
 * TODO: Change name to be globally ok (prepend nrng_?)
 * @param inst         Pointer to struct uwb_dev *
 * @param nslots       number of slot
 * @param config       Pointer to dw1000_rng_config_t.
 * @param duration     Time delay between request.
 *
 * @return ret of uint32_t constant
 */
uint32_t
usecs_to_response(struct uwb_dev * inst, uint16_t nslots, dw1000_rng_config_t * config, uint32_t duration){
    uint32_t ret = nslots * ( duration + (uint32_t) uwb_dwt_usecs_to_usecs(config->tx_guard_delay));
    return ret;
}

/**
 * @fn dw1000_nrng_request(dw1000_dev_instance_t * inst, uint16_t dst_address, dw1000_rng_modes_t code, uint16_t slot_mask, uint16_t cell_id){
 * @brief API to initialise nrng request.
 *
 * @param inst          Pointer to dw1000_dev_instance_t.
 * @param dst_address   Address of the receiver to whom range request to be sent.
 * @param code          Represents mode of ranging DWT_SS_TWR enables single sided two way ranging DWT_DS_TWR enables double sided
 * two way ranging DWT_DS_TWR_EXT enables double sided two way ranging with extended frame.
 * @param slot_mast     nrng_request_frame_t of masked slot number
 * @param cell_id       nrng_request_frame_t of cell id number
 *
 * @return struct uwb_dev_status
 */
struct uwb_dev_status
dw1000_nrng_request(dw1000_nrng_instance_t * nrng, uint16_t dst_address, dw1000_rng_modes_t code, uint16_t slot_mask, uint16_t cell_id)
{
    // This function executes on the device that initiates a request
    struct uwb_dev * inst = nrng->dev_inst;
    assert(inst);

    dpl_error_t err = dpl_sem_pend(&nrng->sem,  DPL_TIMEOUT_NEVER);
    assert(err == DPL_OK);
    NRNG_STATS_INC(nrng_request);

    dw1000_rng_config_t * config = dw1000_nrng_get_config(nrng, code);
    nrng->nnodes = NumberOfBits(slot_mask); // Number of nodes involved in request
    nrng->idx += nrng->nnodes;
    nrng_request_frame_t * frame = (nrng_request_frame_t *) nrng->frames[nrng->idx%nrng->nframes];

    frame->seq_num = ++nrng->seq_num;
    frame->code = code;
    frame->src_address = inst->uid;
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

    uwb_write_tx(inst, frame->array, 0, sizeof(nrng_request_frame_t));
    uwb_write_tx_fctrl(inst, sizeof(nrng_request_frame_t), 0);
    uwb_set_wait4resp(inst, true);

    uint16_t timeout = config->tx_holdoff_delay         // Remote side turn arround time.
                        + usecs_to_response(inst,       // Remaining timeout
                            nrng->nnodes,               // no. of expected frames
                            config,
                            uwb_phy_frame_duration(inst, sizeof(nrng_response_frame_t)) // in usec
                        ) + config->rx_timeout_delay;     // TOF allowance.
    uwb_set_rx_timeout(inst, timeout);

    if (nrng->control.delay_start_enabled)
        uwb_set_delay_start(inst, nrng->delay); 

    // The DW1000 has a bug that render the hardware auto_enable feature useless when used in conjunction with the double buffering. 
    // Consequently, we inhibit this use-case in the PHY-Layer and instead manually perform reenable in the MAC-layer. 
    // This does impact the guard delay time as we need to allow time for the MAC-layer to respond.

    if(inst->config.dblbuffon_enabled)
        assert(inst->config.rxauto_enable == 0);
    //dw1000_set_dblrxbuff(inst, true);  
    
    if (uwb_start_tx(inst).start_tx_error){
        NRNG_STATS_INC(start_tx_error);
        if (dpl_sem_get_count(&nrng->sem) == 0) {
            err = dpl_sem_release(&nrng->sem);
            assert(err == DPL_OK);
        }
    }else{
        err = dpl_sem_pend(&nrng->sem, DPL_TIMEOUT_NEVER); // Wait for completion of transactions
        assert(err == DPL_OK);
        err = dpl_sem_release(&nrng->sem);
        assert(err == DPL_OK);
    }
    // dw1000_set_dblrxbuff(inst, false);
    return inst->status;
}


/**
 * API to initialise range request.
 *
 * @param inst          Pointer to dw1000_dev_instance_t.
 *
 * @return struct uwb_dev_status
 */
struct uwb_dev_status
dw1000_nrng_listen(dw1000_nrng_instance_t * nrng, dw1000_dev_modes_t mode)
{
    dpl_error_t err = dpl_sem_pend(&nrng->sem,  DPL_TIMEOUT_NEVER);
    assert(err == DPL_OK);

    // Download the CIR on the response    
#if MYNEWT_VAL(CIR_ENABLED)   
    cir_enable(((struct _dw1000_dev_instance_t*)nrng->dev_inst)->cir, true);
#endif 
    
    NRNG_STATS_INC(nrng_listen);
    if(uwb_start_rx(nrng->dev_inst).start_rx_error){
        err = dpl_sem_release(&nrng->sem);
        assert(err == DPL_OK);
        NRNG_STATS_INC(start_rx_error);
    }
    if (mode == DWT_BLOCKING){
        err = dpl_sem_pend(&nrng->sem, DPL_TIMEOUT_NEVER); // Wait for completion of transactions 
        assert(err == DPL_OK);
        err = dpl_sem_release(&nrng->sem);
        assert(err == DPL_OK);
    }
   return nrng->dev_inst->status;
}

/**
 * @fn dw1000_nrng_twr_to_tof_frames(struct _dw1000_dev_instance_t * inst, nrng_frame_t *first_frame, nrng_frame_t *final_frame){
 * @brief API to calculate time of flight based on type of ranging.
 *
 * @param inst          Pointer to dw1000_dev_instance_t.
 * @param first_frame   Pointer to the first nrng frame.
 * @param final_frame   Poinetr to the final nrng frame.
 *
 * @return Time of flight in float
 */
float
dw1000_nrng_twr_to_tof_frames(struct uwb_dev * inst, nrng_frame_t *first_frame, nrng_frame_t *final_frame)
{
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
                    -  (first_frame->transmission_timestamp - first_frame->reception_timestamp))/2.0f;
#else
            float skew = dw1000_calc_clock_offset_ratio((dw1000_dev_instance_t*)inst, first_frame->carrier_integrator);
            ToF = ((first_frame->response_timestamp - first_frame->request_timestamp)
                    -  (first_frame->transmission_timestamp - first_frame->reception_timestamp) * (1 - skew))/2.0f;
#endif
            break;
            }
        default: break;
    }
    return ToF;
}

#if MYNEWT_VAL(NRNG_VERBOSE)

/**
 * @fn complete_ev_cb(struct os_event *ev)
 * @brief API for nrng complete event callback and print nrng logs into json format.
 *
 * @param ev    Pointer to os_event.
 *
 * @return true on sucess
 */
static void
complete_ev_cb(struct dpl_event *ev) {
    assert(ev != NULL);
    assert(dpl_event_get_arg(ev));

    dw1000_nrng_instance_t * nrng = (dw1000_nrng_instance_t *) dpl_event_get_arg(ev);
    nrng_encode(nrng, nrng->seq_num, nrng->idx);
    nrng->slot_mask = 0; 
}

struct dpl_event nrng_event;
/**
 * @fn complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
 * @brief API for nrng complete callback and put complete_event_cb in queue.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @param cbs    Pointer to dw1000_mac_interface_t.
 *
 * @return true on sucess
 */
static bool
complete_cb(struct uwb_dev * udev, struct uwb_mac_interface * cbs)
{
    if (udev->fctrl != FCNTL_IEEE_RANGE_16)
        return false;
    dw1000_nrng_instance_t * nrng = (dw1000_nrng_instance_t *)cbs->inst_ptr;
    if(dpl_sem_get_count(&nrng->sem) == 0){
        dpl_event_init(&nrng_event, complete_ev_cb, (void*) nrng);
        dpl_eventq_put(dpl_eventq_dflt_get(), &nrng_event);
    }
    return false;
}

#endif
