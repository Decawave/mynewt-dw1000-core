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
 * @file dw1000_range.c
 * @author paul kettle
 * @date 2018
 * @brief Ranging
 * 
 * @details This is the range base class which utilises the functions to do ranging services using multiple nodes.
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

#if MYNEWT_VAL(DW1000_RANGE)
#include <range/dw1000_range.h>

static void postprocess(struct os_event * ev);
static bool range_complete_cb(dw1000_dev_instance_t * inst);
static bool range_error_cb(dw1000_dev_instance_t * inst);
static bool range_tx_complete_cb(dw1000_dev_instance_t* inst);
static struct os_callout range_callout_timer;
static struct os_callout range_callout_postprocess;

/**
 * API to start ranging by sending the range request to the node_addr[] sequentially.
 * This function is called by the range_callout_timer callout from default queue.
 *
 * @param ev   Pointer to os_events.
 *
 * @return void
 */

static void
range_timer_ev_cb(struct os_event *ev) {
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_range_instance_t *range = inst->range;
    
    assert(range->node_addr != NULL);
    assert(range->nnodes > 0);
    
    os_error_t err = os_sem_pend(&inst->range->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);
 
    dw1000_rng_request(inst, range->node_addr[range->idx++%range->nnodes], range->config.code);

    os_callout_reset(&range_callout_timer, OS_TICKS_PER_SEC * (range->period - MYNEWT_VAL(OS_LATENCY)) * 1e-6 );
}

/**
 * API to initialise the timer based callout(range_callout_timer) to periodically callback the range_timer_ev_cb.
 *        
 * @param inst   Pointer to dw1000_dev_instance_t. 
 *
 * @return void
 */

static void 
range_timer_init(dw1000_dev_instance_t *inst) {
    assert(inst);
    assert(inst->range);
    os_callout_init(&range_callout_timer, os_eventq_dflt_get(), range_timer_ev_cb, (void *) inst);
    os_callout_reset(&range_callout_timer, OS_TICKS_PER_SEC/100);
    dw1000_range_instance_t * range = inst->range; 
    range->status.timer_enabled = true;
}

/**
 * API to check for the type of packet received.
 * If the packet is of ranging type, then it calls range_postprocess by pushing into the event queue.
 * else the extension_cb->next function is called.
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 *
 * @return bool
 */

static bool range_complete_cb(dw1000_dev_instance_t *inst){
    if(inst->fctrl != FCNTL_IEEE_RANGE_16){
        return false;
    }
    assert(inst);
    assert(inst->range);
    dw1000_range_instance_t *range = inst->range;
    if(range->status.started == 1){
        range->rng_idx_list[(range->rng_idx_cnt++)%range->nnodes] = ((inst->rng->idx)%inst->rng->nframes);
        if(range->config.postprocess && ((range->idx%range->nnodes) == 0)){
            uint16_t *temp;
            temp = range->rng_idx_list;
            range->rng_idx_list = range->pp_idx_list;
            range->pp_idx_list = temp;
            range->pp_idx_cnt = range->rng_idx_cnt;
            range->rng_idx_cnt = 0;
            os_eventq_put(os_eventq_dflt_get(), &range_callout_postprocess.c_ev);
        }
    }
    return true;
}

/**
 * This is an internal static function called when an error occured in the receiving the correct range packet.
 * If the error occured during the range process then, then range postprocess is pushed into the event queue 
 * else the extension_cb->next function is called.
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 *
 * @return bool
 */

static bool range_error_cb(dw1000_dev_instance_t *inst){
    assert(inst);
    if(inst->fctrl != FCNTL_IEEE_RANGE_16){
        return false;
    }
    assert(inst->range);
    dw1000_range_instance_t *range = inst->range;
    if(range->status.started == 1){
        if(range->config.postprocess && ((range->idx%range->nnodes) == 0)){
            uint16_t *temp;
            temp = range->rng_idx_list;
            range->rng_idx_list = range->pp_idx_list;
            range->pp_idx_list = temp;
            range->pp_idx_cnt = range->rng_idx_cnt;
            range->rng_idx_cnt = 0;
            os_eventq_put(os_eventq_dflt_get(), &range_callout_postprocess.c_ev);
        }
    }
    return true;
}

/**
 * API for range transmit complete callback.
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 *
 * @return bool
 */

static bool
range_tx_complete_cb(dw1000_dev_instance_t* inst){
    if(inst->fctrl != FCNTL_IEEE_RANGE_16){
        return false;
    }
    return true;
}

/**
 * API to initiate the range_callout_postprocess(os callout) with rng_postprocess 
 * and links to default queue.
 *
 * @param inst              Pointer to dw1000_dev_instance_t.
 * @param rng_postprocess   Pointer to the os_event_fn.
 *
 * @return void
 */

static void range_reg_postprocess(dw1000_dev_instance_t * inst, os_event_fn * rng_postprocess){
    assert(inst);
    assert(inst->range);
    dw1000_range_instance_t *range = inst->range;
    os_callout_init(&range_callout_postprocess, os_eventq_dflt_get(), rng_postprocess, (void *) inst);
    range->config.postprocess = true;
}

/**
 * API for postprocess.
 *
 * @param ev   Pointer to the os_event.
 * @return void
 */

static void postprocess(struct os_event * ev){
	assert(ev != NULL);
	assert(ev->ev_arg != NULL);

        dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
        dw1000_range_instance_t *range = inst->range;
        
        if(range->postprocess != NULL)
            range->postprocess(ev);
        
        for(uint16_t i = 0 ; i < inst->range->nnodes ; i++)
            os_sem_release(&range->sem);
}

/**
 * API to initialise various parameters of range instance like status bits, semaphores,callbacks, ext_callbacks, postprocess. 
 *
 * @param inst       Pointer to dw1000_dev_instance_t. 
 * @param nnodes     Number of nodes to range with.
 * @param node_addr  List of short addresses of nodes to range with.
 * @return dw1000_range_instance_t 
 */

dw1000_range_instance_t * 
dw1000_range_init(dw1000_dev_instance_t * inst, uint16_t nnodes, uint16_t node_addr[]){
    assert(inst);
    dw1000_extension_callbacks_t range_cbs;
    if (inst->range == NULL ) {
        inst->range = (dw1000_range_instance_t *) malloc(sizeof(dw1000_range_instance_t) + 
        nnodes * sizeof(uint16_t) + nnodes * sizeof(uint16_t) + nnodes * sizeof(uint16_t)); 
        assert(inst->range);
        memset(inst->range, 0, sizeof(dw1000_range_instance_t));
        inst->range->status.selfmalloc = 1;
        inst->range->nnodes = nnodes;
        inst->range->idx = 0;
        inst->range->rng_idx_cnt = 0;
        inst->range->pp_idx_cnt = 0;
        inst->range->status.started = 0;
    }else{
        assert(inst->range->nnodes == nnodes);
    }

    os_error_t err = os_sem_init(&inst->range->sem, inst->range->nnodes);
    assert(err == OS_OK);
    inst->range->parent = inst;
    inst->range->period = MYNEWT_VAL(RANGE_PERIOD);
    inst->range->config = (dw1000_range_config_t){
        .postprocess = false,
        .code = DWT_DS_TWR,
    };

    inst->range->node_addr = &inst->range->var_mem_block[0];
    inst->range->rng_idx_list = &inst->range->var_mem_block[nnodes];
    inst->range->pp_idx_list = &inst->range->var_mem_block[nnodes + nnodes];

    dw1000_range_set_nodes(inst, node_addr, nnodes);

    range_cbs.rx_complete_cb = range_complete_cb;
    range_cbs.tx_complete_cb = range_tx_complete_cb;
    range_cbs.rx_timeout_cb =range_error_cb;
    range_cbs.rx_error_cb = range_error_cb;
    range_cbs.tx_error_cb = range_error_cb;
    dw1000_range_set_ext_callbacks(inst, range_cbs);
 
   range_reg_postprocess(inst, &postprocess);

    inst->range->status.initialized = 1;
    return inst->range;
}

/**
 * API to free the acquired resources.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @return void
 */
void 
dw1000_range_free(dw1000_dev_instance_t *inst){
    assert(inst);
    dw1000_remove_extension_callbacks(inst, DW1000_RANGE);
    if (inst->range->status.selfmalloc)
        free(inst->range);
    else{
        inst->range->status.initialized = 0;
        inst->range->status.started = 0;
    }
}

/**
 * APi to register the extension call backs of ranging with the inst->extension_cb. 
 *
 * @param inst                                Pointer to dw1000_range_instance_t.
 * @param dw1000_remove_extension_callbacks   Set of extension type call backs.
 * @return void
 */
void dw1000_range_set_ext_callbacks(dw1000_dev_instance_t * inst, dw1000_extension_callbacks_t range_cbs){
    assert(inst);
    range_cbs.id = DW1000_RANGE;
    dw1000_add_extension_callbacks(inst, range_cbs);
}

/**
 * API to register range post process.
 *
 * @param inst                Pointer to dw1000_range_instance_t. 
 * @param range_postprocess   Pointer to the range post process.
 * @return void
 */
void 
dw1000_range_set_postprocess(dw1000_dev_instance_t * inst, os_event_fn * range_postprocess){
    dw1000_range_instance_t * range = inst->range;
    range->postprocess = range_postprocess;
}

/**
 * API to start the ranging by calling the range_timer_init().
 *
 * @param inst     Pointer to dw1000_range_instance_t. 
 * @param code     It corresponds to the mode of ranging
 *           \n  DWT_SS_TWR    single side two way ranging
 *           \n  DWT_DS_TWR    double side two way ranging
 * @return void
 */
void 
dw1000_range_start(dw1000_dev_instance_t * inst, dw1000_rng_modes_t code){
    assert(inst);
    assert(inst->range);
    // Initialise frame timestamp to current time
    dw1000_range_instance_t * range = inst->range; 
    range->status.valid = false;
    range->config.code = code;
    range->status.started = 1;
    range_timer_init(inst);
}

/**
 * API to stop the ranging. 
 *
 * @param inst        Pointer to dw1000_range_instance_t.
 * @return void
 */
void 
dw1000_range_stop(dw1000_dev_instance_t * inst){
    assert(inst);
    assert(inst->range);
    os_callout_stop(&range_callout_timer);
    inst->range->status.started = 0;
}

/**
 * API to initialise the range->node_addr pointer with the array of node addresses.
 *
 * @param inst         Pointer to dw1000_range_instance_t.
 * @param node_add[]   Pointer to the array of node addresses.
 * @param nnodes       Number of nodes to range with.
 * @return void
 */
inline void
dw1000_range_set_nodes(dw1000_dev_instance_t *inst, uint16_t node_addr[], uint16_t nnodes)
{
    assert(inst);
    assert(inst->range);
    for(uint16_t i = 0;i < nnodes;i++)
        inst->range->node_addr[i] = node_addr[i];
}

/**
 * API to reallocate the memory for storing the node addresses based on the nnodes param.
 * Initialises the range structure with the default values.
 *
 * @param inst          Pointer to dw1000_range_instance_t.
 * @param node_addr[]   Pointer to the list of node addresses.
 * @param nnodes        Number of nodes to range with.
 *
 * @return void
 */
void
dw1000_range_reset_nodes(dw1000_dev_instance_t * inst, uint16_t node_addr[], uint16_t nnodes){
    assert(inst);
    assert(inst->range);
    
    if(nnodes > inst->range->nnodes){
        inst->range = (dw1000_range_instance_t *)realloc(inst->range, sizeof(dw1000_range_instance_t) + 
        nnodes * sizeof(uint16_t) + nnodes * sizeof(uint16_t) + nnodes * sizeof(uint16_t));
        assert(inst->range);

        inst->range->node_addr = &inst->range->var_mem_block[0];
        inst->range->rng_idx_list = &inst->range->var_mem_block[nnodes];
        inst->range->pp_idx_list = &inst->range->var_mem_block[nnodes + nnodes];
    }
    inst->range->idx = 0;
    inst->range->nnodes = nnodes;
    inst->range->rng_idx_cnt = 0;
    inst->range->pp_idx_cnt = 0;

    dw1000_range_set_nodes(inst, node_addr, nnodes);
    os_error_t err = os_sem_init(&inst->range->sem, inst->rng->nframes/2);
    assert(err == OS_OK);
}

/**
 * API to reset the frames by rellocating the memory based on the number of frames parameter
 * and initializes the frames with the default values.
 *
 * @param inst         Pointer to dw1000_range_instance_t.
 * @param twr[]        Pointer to the array of frames.
 * @param nframes      Number of frames.
 *
 * @return void
 */
void
dw1000_rng_reset_frames(dw1000_dev_instance_t * inst, twr_frame_t twr[], uint16_t nframes){
    assert(inst);
    assert(inst->range);
    if(nframes > inst->rng->nframes){
        inst->rng = (dw1000_rng_instance_t *) realloc(inst->rng, sizeof(dw1000_rng_instance_t) +
                nframes * sizeof(twr_frame_t *));
        assert(inst->rng);
    }
    inst->rng->idx = 0xFFFE;
    inst->rng->nframes = nframes;
    dw1000_rng_set_frames(inst, twr, nframes);
}
#endif //MYNEWT_VAL(DW1000_RANGE)
