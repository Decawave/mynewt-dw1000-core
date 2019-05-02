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
 * @file survey.c
 * @author paul kettle
 * @date 02/2019
 * 
 * @brief automatic site survey 
 * @details The site survey process involves constructing a matrix of (n * n -1) ranges between n node. 
 * For this we designate a slot in the superframe that performs a nrng_requst to all other nodes, a slot for broadcasting
 * the result between nodes. The a JSON encoder sentense of the survey is available with SURVEY_VERBOSE enabled.  
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <os/os.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include <stats/stats.h>
#include <bsp/bsp.h>

#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_hal.h>
#if MYNEWT_VAL(SURVEY_ENABLED)
#include <survey/survey.h>
#endif
#if MYNEWT_VAL(TDMA_ENABLED)
#include <tdma/tdma.h>
#endif
#if MYNEWT_VAL(CCP_ENABLED)
#include <ccp/ccp.h>
#endif
#if MYNEWT_VAL(WCS_ENABLED)
#include <wcs/wcs.h>
#endif
#if MYNEWT_VAL(NRNG_ENABLED)
#include <rng/rng.h>
#include <nrng/nrng.h>
#include <rng/slots.h>
#endif
#if MYNEWT_VAL(SURVEY_VERBOSE)
static void survey_complete_cb(struct os_event *ev);
#include <survey/survey_encode.h>
#endif

//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

static bool rx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);

STATS_NAME_START(survey_stat_section)
    STATS_NAME(survey_stat_section, request)
    STATS_NAME(survey_stat_section, listen)
    STATS_NAME(survey_stat_section, rx_unsolicited)
    STATS_NAME(survey_stat_section, start_tx_error)
    STATS_NAME(survey_stat_section, start_rx_error)
    STATS_NAME(survey_stat_section, broadcaster)
    STATS_NAME(survey_stat_section, receiver)
    STATS_NAME(survey_stat_section, rx_timeout)
    STATS_NAME(survey_stat_section, reset)
STATS_NAME_END(survey_stat_section)

survey_status_t survey_request(survey_instance_t * survey, uint64_t dx_time);
survey_status_t survey_listen(survey_instance_t * survey, uint64_t dx_time);
survey_status_t survey_broadcaster(survey_instance_t * survey, uint64_t dx_time);
survey_status_t survey_receiver(survey_instance_t * survey, uint64_t dx_time);

/**
 *
 * @return survey_instance_t * 
 */
survey_instance_t * 
survey_init(struct _dw1000_dev_instance_t * inst, uint16_t nnodes, uint16_t nframes){
    assert(inst);
    
    if (inst->survey == NULL ) {
        survey_instance_t * survey = (survey_instance_t *) malloc(sizeof(survey_instance_t) + nframes * sizeof(survey_nrngs_t * )); 
        assert(survey);
        memset(survey, 0, sizeof(survey_instance_t) + nframes * sizeof(survey_nrngs_t * ));
    
        for (uint16_t j = 0; j < nframes; j++){
            survey->nrngs[j] = (survey_nrngs_t *) malloc(sizeof(survey_nrngs_t) + nnodes * sizeof(survey_nrng_t * )); // Variable array alloc
            assert(survey->nrngs[j]);
            memset(survey->nrngs[j], 0, sizeof(survey_nrngs_t) + nnodes * sizeof(survey_nrng_t * ));

            for (uint16_t i = 0; i < nnodes; i++){
                survey->nrngs[j]->nrng[i] = (survey_nrng_t * ) malloc(sizeof(survey_nrng_t) + nnodes * sizeof(float)); 
                assert(survey->nrngs[j]->nrng[i]);
                memset(survey->nrngs[j]->nrng[i], 0, sizeof(survey_nrng_t) + nnodes * sizeof(float));
            }
        }

        survey->frame = (survey_broadcast_frame_t *) malloc(sizeof(survey_broadcast_frame_t) + nnodes * sizeof(float)); 
        assert(survey->frame);
        memset(survey->frame, 0, sizeof(survey_broadcast_frame_t) + nnodes * sizeof(float));
        survey_broadcast_frame_t frame = {
            .PANID = 0xDECA,
            .fctrl = FCNTL_IEEE_RANGE_16,
            .dst_address = 0xffff,      //broadcast
            .src_address = inst->my_short_address,
            .code = DWT_SURVEY_BROADCAST
        };

        memcpy(survey->frame, &frame, sizeof(survey_broadcast_frame_t));
        survey->status.selfmalloc = 1;
        survey->nnodes = nnodes; 
        survey->nframes = nframes; 
        survey->parent = inst;
        inst->survey = survey;
        os_error_t err = os_sem_init(&inst->survey->sem, 0x1); 
        assert(err == OS_OK);
    }else{
        assert(inst->survey->nnodes == nnodes);
    }
    inst->survey->status.initialized = 1;
    inst->survey->config = (survey_config_t){
        .rx_timeout_delay = MYNEWT_VAL(SURVEY_RX_TIMEOUT)
    };

    inst->survey->cbs = (dw1000_mac_interface_t){
        .id = DW1000_SURVEY,
        .rx_complete_cb = rx_complete_cb,
        .tx_complete_cb = tx_complete_cb,
        .rx_timeout_cb = rx_timeout_cb,
        .reset_cb = reset_cb
    };

#if MYNEWT_VAL(SURVEY_VERBOSE)
    inst->survey->survey_complete_cb = survey_complete_cb;
#endif
    dw1000_mac_append_interface(inst, &inst->survey->cbs);

    int rc = stats_init(
                STATS_HDR(inst->survey->stat),
                STATS_SIZE_INIT_PARMS(inst->survey->stat, STATS_SIZE_32),
                STATS_NAME_INIT_PARMS(survey_stat_section)
            );
    assert(rc == 0);

    rc = stats_register("survey", STATS_HDR(inst->survey->stat));
    assert(rc == 0);

    return inst->survey;
}

/** 
 * Deconstructor
 * 
 * @param inst   Pointer to survey_instance_t * 
 * @return void
 */
void 
survey_free(survey_instance_t * inst){
    assert(inst);  
    
    if (inst->status.selfmalloc){
        inst->parent->survey = NULL;
        for (uint16_t j = 0; j < inst->nframes; j++){
            for (uint16_t i = 0; i < inst->nnodes; i++)
                free(inst->nrngs[j]->nrng[i]);
            free(inst->nrngs[j]);
        }
        free(inst->frame);
        free(inst);
    }else{
        inst->status.initialized = 0;
    }
}

/**
 * API to initialise the package
 *
 * @return void
 */
void 
survey_pkg_init(void){

    printf("{\"utime\": %lu,\"msg\": \"survey_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

#if MYNEWT_VAL(DW1000_DEVICE_0)
    survey_init(hal_dw1000_inst(0), MYNEWT_VAL(SURVEY_NNODES), MYNEWT_VAL(SURVEY_NFRAMES));
#endif
}

#if MYNEWT_VAL(SURVEY_VERBOSE)
/**
 * API for verbose logging of survey results.
 * 
 * @param struct os_event
 * @return none
 */
static void survey_complete_cb(struct os_event *ev) {
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    survey_instance_t * survey = (survey_instance_t *) ev->ev_arg;
    survey_encode(survey, survey->seq_num, survey->idx);    
}
#endif

/**
 * Callback to schedule nrng request survey
 * 
 * @param struct os_event
 * @return none
 * @brief This callback is call from a TDMA slot (SURVEY_BROADCAST_SLOT) assigned in the toplevel main. 
 * The variable SURVEY_MASK is used to derive the sequencing of the node.
 */
void 
survey_slot_range_cb(struct os_event *ev){
    assert(ev);
    assert(ev->ev_arg);

    tdma_slot_t * slot = (tdma_slot_t *) ev->ev_arg;
    tdma_instance_t * tdma = slot->parent;
    dw1000_dev_instance_t * inst = tdma->parent;
    dw1000_ccp_instance_t * ccp = inst->ccp;
    survey_instance_t * survey = inst->survey;
    survey->seq_num = (ccp->seq_num & ((uint32_t)~0UL << MYNEWT_VAL(SURVEY_MASK))) >> MYNEWT_VAL(SURVEY_MASK);
    
    if(ccp->seq_num % survey->nnodes == inst->slot_id){
        uint64_t dx_time = tdma_tx_slot_start(inst, slot->idx) & 0xFFFFFFFE00UL;
        survey_request(survey, dx_time);
    }
    else{
        uint64_t dx_time = tdma_rx_slot_start(inst, slot->idx) & 0xFFFFFFFE00UL;
        survey_listen(survey, dx_time); 
    }
}


/**
 * Callback to schedule survey broadcasts
 * 
 * @param struct os_event
 * @return none
 * @brief This callback is call from a TDMA slot (SURVEY_BROADCAST_SLOT) assigned in the toplevel main. 
 * The variable SURVEY_MASK is used to derive the sequencing of the node.
 */
static struct os_callout survey_complete_callout;

void 
survey_slot_broadcast_cb(struct os_event *ev){
    assert(ev);
    assert(ev->ev_arg);

    tdma_slot_t * slot = (tdma_slot_t *) ev->ev_arg;
    tdma_instance_t * tdma = slot->parent;
    dw1000_dev_instance_t * inst = tdma->parent;
    dw1000_ccp_instance_t * ccp = inst->ccp;
    survey_instance_t * survey = inst->survey;
    survey->seq_num = (ccp->seq_num & ((uint32_t)~0UL << MYNEWT_VAL(SURVEY_MASK))) >> MYNEWT_VAL(SURVEY_MASK);

    if(ccp->seq_num % survey->nnodes == inst->slot_id){
        uint64_t dx_time = tdma_tx_slot_start(inst, slot->idx) & 0xFFFFFFFE00UL;
        survey_broadcaster(survey, dx_time);
    }else{
        uint64_t dx_time = tdma_rx_slot_start(inst, slot->idx) & 0xFFFFFFFE00UL;
        survey_receiver(survey, dx_time);  
    }
    if(ccp->seq_num % survey->nnodes == survey->nnodes - 1 && survey->survey_complete_cb){
        os_callout_init(&survey_complete_callout, os_eventq_dflt_get(), survey->survey_complete_cb, survey);
        os_eventq_put(os_eventq_dflt_get(), &survey_complete_callout.c_ev);
    }
}

/**
 * API to initiaate a nrng request from a node to node survey
 *
 * @param inst pointer to _dw1000_dev_instance_t.
 * @param dx_time time to start suevey
 * @return survey_status_t
 * 
 */
survey_status_t 
survey_request(survey_instance_t * survey, uint64_t dx_time){
    assert(survey);

    dw1000_dev_instance_t * inst = survey->parent;
    STATS_INC(inst->survey->stat, request);
    
    uint32_t slot_mask = ~(~0UL << (survey->nnodes));
    dw1000_nrng_request_delay_start(inst, 0xffff, dx_time, DWT_SS_TWR_NRNG, slot_mask, 0);
    
    survey_nrngs_t * nrngs = survey->nrngs[(survey->idx)%survey->nframes];
    nrngs->nrng[inst->slot_id]->mask = dw1000_nrng_get_ranges(inst, 
                                nrngs->nrng[inst->slot_id]->rng, 
                                survey->nnodes, 
                                inst->nrng->idx
                            );
    return survey->status;
}


survey_status_t  
survey_listen(survey_instance_t * survey, uint64_t dx_time){
    assert(survey);

    dw1000_dev_instance_t * inst = survey->parent;
    STATS_INC(inst->survey->stat, listen);

    dw1000_set_delay_start(inst, dx_time);
    uint16_t timeout = dw1000_phy_frame_duration(&inst->attrib, sizeof(nrng_request_frame_t))
                        + inst->nrng->config.rx_timeout_delay;         
    dw1000_set_rx_timeout(inst, timeout + 0x1000);
    dw1000_nrng_listen(inst, DWT_BLOCKING);

    return survey->status;
}


/**
 * API to broadcasts survey results
 *
 * @param inst pointer to _dw1000_dev_instance_t.
 * @param dx_time time to start broadcast
 * @return survey_status_t
 * 
 */
survey_status_t  
survey_broadcaster(survey_instance_t * survey, uint64_t dx_time){
    assert(survey);

    os_error_t err = os_sem_pend(&survey->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);
    STATS_INC(survey->stat, broadcaster);

    dw1000_dev_instance_t * inst = survey->parent;
    survey_nrngs_t * nrngs = survey->nrngs[survey->idx%survey->nframes];

    survey->frame->mask = nrngs->nrng[inst->slot_id]->mask;
    survey->frame->seq_num = survey->seq_num;
    survey->frame->slot_id = inst->slot_id;

    uint16_t nnodes = NumberOfBits(survey->frame->mask);
    survey->status.empty = nnodes == 0;
    if (survey->status.empty){
        err = os_sem_release(&survey->sem);
        assert(err == OS_OK);
        return survey->status;
    }

    assert(nnodes < survey->nnodes);
    memcpy(survey->frame->rng, nrngs->nrng[inst->slot_id]->rng, nnodes * sizeof(float));
    
    uint16_t n = sizeof(struct _survey_broadcast_frame_t) + nnodes * sizeof(float);
    dw1000_write_tx(inst, survey->frame->array, 0, n);
    dw1000_write_tx_fctrl(inst, n, 0);
    dw1000_set_delay_start(inst, dx_time); 
    
    survey->status.start_tx_error = dw1000_start_tx(inst).start_tx_error;
    if (survey->status.start_tx_error){
        STATS_INC(survey->stat, start_tx_error);
        if (os_sem_get_count(&survey->sem) == 0) 
            os_sem_release(&survey->sem);
    }else{
        err = os_sem_pend(&survey->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions 
        assert(err == OS_OK);
        err = os_sem_release(&survey->sem);
        assert(err == OS_OK);
    }
    return survey->status;
}

/**
 * API to receive survey broadcasts
 *
 * @param inst pointer to _dw1000_dev_instance_t.
 * @param dx_time time to start received
 * @return survey_status_t
 * 
 */
survey_status_t  
survey_receiver(survey_instance_t * survey, uint64_t dx_time){
    assert(survey);

    dw1000_dev_instance_t * inst = survey->parent;
    os_error_t err = os_sem_pend(&survey->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);
    STATS_INC(survey->stat, receiver);

    uint16_t n = sizeof(struct _survey_broadcast_frame_t) + survey->nnodes * sizeof(float);
    uint16_t timeout = dw1000_phy_frame_duration(&inst->attrib, n) 
                        + survey->config.rx_timeout_delay;
    dw1000_set_rx_timeout(inst, timeout); 

    survey->status.start_rx_error = dw1000_start_rx(inst).start_rx_error;
    if(survey->status.start_rx_error){
        STATS_INC(survey->stat, start_rx_error);
        err = os_sem_release(&survey->sem);
        assert(err == OS_OK); 
    }else{
        err = os_sem_pend(&survey->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions 
        assert(err == OS_OK);
        err = os_sem_release(&survey->sem);
        assert(err == OS_OK);
    }
    return survey->status;
}

/**
 * API for receive survey broadcasts.
 * 
 * @param inst pointer to dw1000_dev_instance_t
 * @param cbs pointer todw1000_mac_interface_t
 * @return true on sucess
 */
static bool 
rx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{   
    assert(inst->survey);
    survey_instance_t * survey = inst->survey; 
    dw1000_ccp_instance_t * ccp = inst->ccp;

    if(inst->fctrl != FCNTL_IEEE_RANGE_16)
        return false;

    if(os_sem_get_count(&survey->sem) == 1){ // unsolicited inbound
        STATS_INC(survey->stat, rx_unsolicited);
        return false;
    }
  
    if(inst->frame_len < sizeof(survey_broadcast_frame_t))
       return false;

    survey_broadcast_frame_t * frame = ((survey_broadcast_frame_t * ) inst->rxbuf);

    if(frame->dst_address != 0xffff)
        return false;
    if(ccp->seq_num % survey->nnodes == 0)
        survey->idx++;  // advance the nrngs idx at begining of sequence.

    switch(frame->code) {
        case DWT_SURVEY_BROADCAST:
            {   
                if (frame->cell_id != inst->cell_id)
                    return false;
                if (frame->seq_num != survey->seq_num) 
                    break;
                uint16_t n = sizeof(survey_broadcast_frame_t) + survey->nnodes * sizeof(float);
                if (inst->frame_len > n || frame->slot_id > survey->nnodes - 1) {
                    return false;
                }
                survey_nrngs_t * nrngs = survey->nrngs[survey->idx%survey->nframes];
                uint16_t nnodes = NumberOfBits(frame->mask);
                survey->status.empty = nnodes == 0;
                if(!survey->status.empty){
                    nrngs->mask |= 1U << frame->slot_id;
                    nrngs->nrng[frame->slot_id]->mask = frame->mask;
                    memcpy(nrngs->nrng[frame->slot_id]->rng, frame->rng, nnodes * sizeof(float));
                    break;
                }else{      
                    nrngs->nrng[frame->slot_id]->mask = 0;              
                    break;
                }
            }
            break;
        default: 
            return false;
    }
    os_error_t err = os_sem_release(&survey->sem);
    assert(err == OS_OK);
    return true;
}


/**
 * API for transmission complete callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
static bool
tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){

    survey_instance_t * survey = inst->survey; 

    if(os_sem_get_count(&survey->sem) == 1)
        return false;
   
    os_error_t err = os_sem_release(&survey->sem);
    assert(err == OS_OK);
    
    return false;
}


/**
 * API for timeout complete callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
static bool
rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    
    survey_instance_t * survey = inst->survey; 
    
    if(os_sem_get_count(&survey->sem) == 1)
        return false;   

    os_error_t err = os_sem_release(&survey->sem);  
    assert(err == OS_OK);
    STATS_INC(survey->stat, rx_timeout);
    
    return true;
}


/** 
 * API for reset_cb of survey interface
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 * @return true on sucess
 */
static bool
reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    
    survey_instance_t * survey = inst->survey; 

    if(os_sem_get_count(&survey->sem) == 1)
        return false;

    os_error_t err = os_sem_release(&survey->sem);  
    assert(err == OS_OK);
    STATS_INC(survey->stat, reset);

    return true;    
}



