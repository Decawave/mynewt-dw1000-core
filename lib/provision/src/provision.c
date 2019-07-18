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
 * @file provision.c
 * @author paul kettle
 * @date 2018
 * @brief provisioning
 *
 * @details This is the provision base class that scans for the available nodes and store their addresses.
 *
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

#if MYNEWT_VAL(RNG_ENABLED)
#include <rng/rng.h>
#endif

#if MYNEWT_VAL(PROVISION_ENABLED)

#include <provision/provision.h>

static dw1000_provision_config_t g_config = {
    .tx_holdoff_delay = MYNEWT_VAL(PROVISION_TX_HOLDOFF),         // Send Time delay in usec.
    .rx_timeout_period = MYNEWT_VAL(PROVISION_RX_TIMEOUT)              // Receive response timeout in usec.
};

static bool provision_rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool provision_rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool provision_rx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool provision_tx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool provision_tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static void provision_postprocess(struct os_event * ev);

/**
 * API to initialise the provision package.
 *
 * @return void
 */
void provision_pkg_init(void){

    printf("{\"utime\": %lu,\"msg\": \"provision_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

    g_config.period = MYNEWT_VAL(PROVISION_PERIOD)*1e-3;
    g_config.postprocess = false;
    g_config.max_node_count = MYNEWT_VAL(NUM_NODES);
#if MYNEWT_VAL(DW1000_DEVICE_0)
    dw1000_provision_init(hal_dw1000_inst(0), g_config);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    dw1000_provision_init(hal_dw1000_inst(1), g_config);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    dw1000_provision_init(hal_dw1000_inst(2), g_config);
#endif
}

/**
 * API for provision timer event callback.
 *
 * @param ev  pointer to os_events.
 * @return void
 */
static void
provision_timer_ev_cb(struct os_event *ev) {
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_provision_instance_t * provision = inst->provision;
    provision->num_node_count = 0;
    if(dw1000_provision_request(inst, DWT_BLOCKING).start_tx_error)
      os_callout_reset(&provision->provision_callout_timer, OS_TICKS_PER_SEC * (provision->config.period));
}

/**
 * API for provision timer init.
 *
 * @param inst poinetr to dw1000_dev_instance_t.
 * @return void
 */
static void
provision_timer_init(dw1000_dev_instance_t * inst) {
    assert(inst != NULL);
    assert(inst->provision != NULL);
    dw1000_provision_instance_t* provision = inst->provision;
    os_callout_init(&provision->provision_callout_timer, os_eventq_dflt_get(), provision_timer_ev_cb, (void *) inst);
    os_callout_reset(&provision->provision_callout_timer,provision->config.period*OS_TICKS_PER_SEC);
}

/**
 * API to allocate resources on TAG & Anchor for provisioning
 * can be freed on TAG & ANCHOR on once provision have been completed.
 *
 * @param inst    Pointer to dw1000_dev_instance_t.
 * @param config  Configures provision features.
 *
 * @return dw1000_provision_instance_t
 */
dw1000_provision_instance_t*
dw1000_provision_init(dw1000_dev_instance_t * inst, dw1000_provision_config_t config)
{
    assert(inst);
    
    dw1000_provision_instance_t *provision = (dw1000_provision_instance_t*)dw1000_mac_find_cb_inst_ptr(inst, DW1000_PROVISION);
    if (provision == NULL ){
        provision = (dw1000_provision_instance_t *) malloc(sizeof(dw1000_provision_instance_t) + config.max_node_count*sizeof(uint16_t));
        assert(provision);
        memset(provision, 0, sizeof(dw1000_provision_instance_t));
        provision->status.selfmalloc = 1;
    }

    provision->dev_inst = inst;
    provision->idx = 0x0;
    provision->nframes = 2;
    memcpy(&provision->config,&config,sizeof(dw1000_provision_config_t));
    inst->provision->cbs = (dw1000_mac_interface_t){
        .id = DW1000_PROVISION,
        .inst_ptr = provision,
        .tx_complete_cb = provision_tx_complete_cb,
        .rx_complete_cb = provision_rx_complete_cb,
        .rx_timeout_cb = provision_rx_timeout_cb,
        .rx_error_cb = provision_rx_error_cb,
        .tx_error_cb = provision_tx_error_cb,
    };
    dw1000_mac_append_interface(inst, &inst->provision->cbs);

    provision->status.provision_status = PROVISION_INVALID;
    os_error_t err = os_sem_init(&inst->provision->sem, 0x1);
    assert(err == OS_OK);

    dw1000_provision_set_postprocess(provision, &provision_postprocess);
    provision->status.initialized = 1;
    return provision;
}

/**
 * API to free allocated provision resources.
 *
 * @param inst  Pointer to dw1000_provision_instance_t.
 *
 * @return void
 */

void
dw1000_provision_free(dw1000_provision_instance_t * provision){
    assert(provision != NULL);
    dw1000_mac_remove_interface(provision->dev_inst, DW1000_PROVISION);
    if (provision->status.selfmalloc){
        free(provision);
    }
    else
        provision->status.initialized = 0;
}

/**
 * API to set post_process.
 *
 * @param inst                   Pointer to dw1000_provision_instance_t.
 * @param provision_postprocess  Pointer to os_event_fn.
 *
 * @return void
 */
void
dw1000_provision_set_postprocess(dw1000_provision_instance_t * provision, os_event_fn * provision_postprocess){
    assert(provision != NULL);
    os_callout_init(&provision->provision_callout_postprocess, os_eventq_dflt_get(), provision_postprocess, (void *) provision);
    provision->config.postprocess = true;
}

/**
 * This is a template which should be replaced by the provision initiator by a event that can just pass the
 * information to some other layers like application to kick start some other event. Currently just prints out
 * the information about the provisioned devices.
 *
 * @param ev    Pointer to os_events.
 *
 * @return void
 */
static void
provision_postprocess(struct os_event * ev){
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);
    /* NODE: ev->ev_arg is a dw1000_dev_provision_t */
}

/**
 * API that executes on both the provision intiator and the TAG/ANCHOR
 * that replies to the beacon. On the provision initiator the post process function can send the database to the application
 * and kick start some other task like ranging or so. In responder there is no need of post process and should just go to
 * rx mode again.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @return bool
 */
static bool
provision_rx_complete_cb(dw1000_dev_instance_t* inst, dw1000_mac_interface_t * cbs)
{
    assert(inst != NULL);
    if(inst->fctrl != FCNTL_IEEE_PROVISION_16){
        return false;
    }
    dw1000_provision_instance_t * provision = (dw1000_provision_instance_t *)cbs->inst_ptr;
    uint16_t  frame_idx = provision->idx;
    uint16_t code, dst_address;
    dw1000_provision_config_t config = provision->config;

    dw1000_read_rx(inst, (uint8_t *) &code, offsetof(ieee_rng_request_frame_t,code), sizeof(uint16_t));
    dw1000_read_rx(inst, (uint8_t *) &dst_address, offsetof(ieee_rng_request_frame_t,dst_address), sizeof(uint16_t));

    if ((dst_address != inst->my_short_address) && (dst_address != (uint16_t)0xFFFF)){
        dw1000_start_rx(inst);
            //inst->rng_rx_error_cb(inst);
        return true;
    }
    provision_frame_t * frame = &provision->frames[(frame_idx)%provision->nframes];
    switch(code)
    {
        case DWT_PROVISION_START:
            {
                if (inst->frame_len >= sizeof(ieee_rng_request_frame_t))
                    dw1000_read_rx(inst, frame->array, 0, sizeof(ieee_rng_request_frame_t));
                else{
                    dw1000_start_rx(inst);
                        //inst->rng_rx_error_cb(inst);
                    break;
                }
                uint8_t delay_factor = 1;  //Delay_factor for NODE_0
                if(inst->slot_id > 0) // if device is of NODE type
                   delay_factor = (inst->slot_id) ;  //Increase the delay factor for late response for Anchor provisioning
                uint64_t request_timestamp = dw1000_read_rxtime(inst);
                uint64_t response_tx_delay = request_timestamp + ((uint64_t)(config.tx_holdoff_delay*delay_factor) << 16);
                frame->dst_address = frame->src_address;
                frame->src_address = inst->my_short_address;
                frame->code = DWT_PROVISION_RESP;
                dw1000_write_tx(inst, frame->array, 0, sizeof(ieee_rng_response_frame_t));
                dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_response_frame_t), 0);
                dw1000_set_wait4resp(inst,true);
                dw1000_set_delay_start(inst, response_tx_delay);
                //dw1000_set_rx_timeout(inst,0);
                if (dw1000_start_tx(inst).start_tx_error){
                }
                break;
            }
        case DWT_PROVISION_RESP:
            {
                if (inst->frame_len >= sizeof(ieee_rng_response_frame_t))
                    dw1000_read_rx(inst, frame->array, 0, sizeof(ieee_rng_response_frame_t));
                else{
                    dw1000_start_rx(inst);
                        //inst->rng_rx_error_cb(inst);
                    break;
                }
                if(provision_add_node(provision, frame->src_address) == PROVISION_ERROR){
                    //If the addition fails when the number of nodes exceeds max allowed count callout the postprocess
                    os_error_t err = os_sem_release(&provision->sem);
                    assert(err == OS_OK);
                    provision->status.provision_status = PROVISION_DONE;
                    if (provision->config.postprocess){
                        os_eventq_put(os_eventq_dflt_get(), &provision->provision_callout_postprocess.c_ev);
                    }
                    break;
                }
                dw1000_start_rx(inst);
                    //inst->rng_rx_error_cb(inst);
                break;
            }
        default:
            printf("Wrong request \n");
    }
    return true;
}

/**
 * API for rx_timeout callback.
 *
 * @param inst      pointer to dw1000_dev_instance_t.
 *
 * @return bool
 */
static bool
provision_rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    assert(inst != NULL);
    if(inst->fctrl != FCNTL_IEEE_PROVISION_16){
        return false;
    }
    dw1000_provision_instance_t * provision = (dw1000_provision_instance_t *)cbs->inst_ptr;
    if(provision->status.provision_status == PROVISION_START){
        os_error_t err = os_sem_release(&provision->sem);
        assert(err == OS_OK);
        provision->status.provision_status = PROVISION_DONE;
        if (provision->config.postprocess){
            os_eventq_put(os_eventq_dflt_get(), &provision->provision_callout_postprocess.c_ev);
        }
    }else{
        dw1000_start_rx(inst);
    }
    return true;
}

/**
 * API for rx_error callback.
 *
 * @param inst    Pointer to dw1000_dev_instance_t.

 * @return bool
 */
static bool
provision_rx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    assert(inst != NULL);
    if(inst->fctrl != FCNTL_IEEE_PROVISION_16){
         return false;
    }
    dw1000_provision_instance_t * provision = (dw1000_provision_instance_t *)cbs->inst_ptr;
    if(provision->status.provision_status == PROVISION_START){
        os_error_t err = os_sem_release(&provision->sem);
        assert(err == OS_OK);
        provision->status.provision_status = PROVISION_DONE;
    }else{
        dw1000_start_rx(inst);
    }
    return true;
}

/**
 * API for tx_error callback.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 *
 * @return bool
 */
static bool
provision_tx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    assert(inst != NULL);
    if(inst->fctrl != FCNTL_IEEE_PROVISION_16){
        return false;
    }
    return true;
}

/**
 * API for tx_complete callback.
 *
 * @param inst pointer to dw1000_dev_instance_t.
 *
 * @return bool
 */
static bool
provision_tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    //Place holder
    if(inst->fctrl != FCNTL_IEEE_PROVISION_16){
        return false;
    }
    return true;
}

/**
 * API to send Provision request.It is a phase where a one device helps to finds all the accessible nodes in the range.
 * The outcome is a database of all the nearby nodes.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @param mode  BLOCKING and NONBLOCKING modes.
 *
 * @return dw1000_provision_status_t
 */
dw1000_provision_status_t
dw1000_provision_request(dw1000_provision_instance_t * provision, dw1000_dev_modes_t mode)
{
    os_error_t err = os_sem_pend(&provision->sem,OS_TIMEOUT_NEVER);
    assert(err == OS_OK);
    provision_frame_t * frame = &provision->frames[(provision->idx++)%provision->nframes];

    provision->status.provision_status = PROVISION_START;
    frame->seq_num++;
    frame->fctrl = FCNTL_IEEE_PROVISION_16;
    frame->PANID = 0xDECA;
    frame->code = DWT_PROVISION_START;
    frame->src_address = inst->my_short_address;
    frame->dst_address = BROADCAST_ADDRESS;
    dw1000_write_tx(inst, frame->array, 0, sizeof(ieee_rng_response_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_response_frame_t), 0);
    dw1000_set_wait4resp(inst, true);
    dw1000_set_rx_timeout(inst, provision->config.rx_timeout_period);
    provision->status.start_tx_error = dw1000_start_tx(inst).start_tx_error;
    if (provision->status.start_tx_error){
        os_sem_release(&provision->sem);
        provision->status.provision_status = PROVISION_DONE;
    }
    else if(mode == DWT_BLOCKING){
        err = os_sem_pend(&provision->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions
        os_sem_release(&provision->sem);
    }
   return provision->status;
}

/**
 * This function schedules the provision timer event.This function
 * should be called for a TAG/NODE which should send a BEACON and the other
 * node should be in receive mode.
 *
 * @param inst    Pointer to dw1000_dev_instance_t.
 *
 * @return void
 */
void
dw1000_provision_start(dw1000_provision_instance_t * provision)
{
    provision->idx = 0x0;
    provision->status.valid = true;
    provision_timer_init(provision);
}

/**
 * API to stop the provision process.
 *
 * @param inst    Pointer to dw1000_dev_instance_t.
 *
 * @return void
 */
void
dw1000_provision_stop(dw1000_provision_instance_t * provision)
{
    provision->status.valid = false;
    os_callout_stop(&provision->provision_callout_timer);
}

/**
 * API to add a new provision node to its database and stores it in provision's
 * instance.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @param addr   Short address of the device to be added.
 *
 * @return dw1000_provision_error_t
 */
dw1000_provision_error_t
provision_add_node(dw1000_provision_instance_t * provision, uint16_t addr)
{
    if((provision->num_node_count+1) > provision->config.max_node_count)
        return PROVISION_ERROR;
    provision->dev_addr[provision->num_node_count++] = addr;
    return PROVISION_SUCCESS;
}

/**
 * API to delete a node from its database.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @param addr  Short address of the device to be removed.
 *
 * return dw1000_provision_error_t
 */
dw1000_provision_error_t
provision_delete_node(dw1000_provision_instance_t * provision, uint16_t addr){
    for(uint8_t i=0; i < provision->num_node_count; i++){
        if(provision->dev_addr[i] == addr){
            for(int j=i;j< provision->num_node_count-1 ; j++ ){
                provision->dev_addr[j] = provision->dev_addr[j+1];
            }
            provision->num_node_count--;
            return PROVISION_SUCCESS;
        }
    }
    return PROVISION_ERROR;
}
#endif //PROVISION_ENABLED
