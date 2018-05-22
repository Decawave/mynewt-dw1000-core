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

#if MYNEWT_VAL(DW1000_PROVISION)
#include <dw1000/dw1000_provision.h>
static void provision_rx_complete_cb(dw1000_dev_instance_t * inst);
static void provision_rx_timeout_cb(dw1000_dev_instance_t * inst);
static void provision_rx_error_cb(dw1000_dev_instance_t * inst);
static void provision_tx_complete_cb(dw1000_dev_instance_t * inst);
static void provision_postprocess(struct os_event * ev);

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

static void
provision_timer_init(dw1000_dev_instance_t * inst) {
    assert(inst != NULL);
    assert(inst->provision != NULL);
    dw1000_provision_instance_t* provision = inst->provision;
    os_callout_init(&provision->provision_callout_timer, os_eventq_dflt_get(), provision_timer_ev_cb, (void *) inst);
    os_callout_reset(&provision->provision_callout_timer,provision->config.period*OS_TICKS_PER_SEC);
}

/*! 
 * @fn dw1000_provision_init(dw1000_dev_instance_t * inst)
 *
 * @brief Allocate resources on TAG & Anchor for provisioning
 * can be freeded on TAG & ANCHOR on once provision have been completed
 *
 * input parameters
 * @param inst - Pointer to dev instance
 *
 * output parameters
 *
 * returns dw1000_provision_instance_t*
 */
dw1000_provision_instance_t*
dw1000_provision_init(dw1000_dev_instance_t * inst, dw1000_provision_config_t config){
    assert(inst);
    if (inst->provision == NULL ){
        inst->provision = (dw1000_provision_instance_t *) malloc(sizeof(dw1000_provision_instance_t) + config.max_node_count*sizeof(uint16_t));
        assert(inst->provision);
        memset(inst->provision, 0, sizeof(dw1000_provision_instance_t));
        inst->provision->status.selfmalloc = 1;
    }
    dw1000_provision_instance_t* provision = inst->provision;
    assert(provision!=NULL);

    provision->parent = inst;
    provision->idx = 0x0;
    provision->nframes = 2;
    memcpy(&provision->config,&config,sizeof(dw1000_provision_config_t));
    provision->status.provision_status = PROVISION_INVALID;
    os_error_t err = os_sem_init(&inst->provision->sem, 0x1);
    assert(err == OS_OK);

    dw1000_provision_set_callbacks(inst, provision_rx_complete_cb, provision_tx_complete_cb, provision_rx_timeout_cb, provision_rx_error_cb);
    dw1000_provision_set_postprocess(inst, &provision_postprocess);
    inst->provision->status.initialized = 1;
    return inst->provision;
}

/*! 
 * @fn dw1000_provision_free(dw1000_dev_instance_t * inst)
 *
 * @brief Free resources and restore default behaviour. 
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * inst
 *
 * output parameters
 *
 * returns none
 */
void
dw1000_provision_free(dw1000_dev_instance_t * inst){
    assert(inst != NULL);
    assert(inst->provision != NULL);
    dw1000_provision_set_callbacks(inst,(dw1000_dev_cb_t)NULL, (dw1000_dev_cb_t)NULL, (dw1000_dev_cb_t)NULL, (dw1000_dev_cb_t)NULL);
    if (inst->provision->status.selfmalloc){
        if(inst->provision->dev_addr != NULL){
            free(inst->provision->dev_addr);
        }
        free(inst->provision);
    }
    else
        inst->status.initialized = 0;
}

/*! 
 * @fn dw1000_provision_set_callbacks(dw1000_dev_instance_t * inst,dw1000_dev_cb_t provision_rx_complete_cb, dw1000_dev_cb_t provision_tx_complete_cb,\
 *                                    dw1000_dev_cb_t provision_rx_timeout_cb, dw1000_dev_cb_t provision_rx_error_cb)
 *
 * @brief Sets the callbacks to be called for provision related rx_complete, rx_timeout, etc
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 * @param inst - dw1000_dev_cb_t  provision_rx_complete_cb
 * @param inst - dw1000_dev_cb_t  provision_tx_complete_cb
 * @param inst - dw1000_dev_cb_t  provision_rx_timeout_cb
 * @param inst - dw1000_dev_cb_t  provision_rx_error_cb
 *
 * output parameters
 *
 * returns none
 */
void 
dw1000_provision_set_callbacks(dw1000_dev_instance_t * inst, dw1000_dev_cb_t provision_rx_complete_cb, dw1000_dev_cb_t provision_tx_complete_cb,\
    dw1000_dev_cb_t provision_rx_timeout_cb, dw1000_dev_cb_t provision_rx_error_cb){
    inst->provision_rx_complete_cb = provision_rx_complete_cb;
    inst->provision_tx_complete_cb = provision_tx_complete_cb;
    inst->provision_rx_timeout_cb = provision_rx_timeout_cb;
    inst->provision_rx_error_cb = provision_rx_error_cb;
}


/*! 
 * @fn dw1000_provision_set_postprocess(dw1000_dev_instance_t * inst, os_event_fn * provision_postprocess)
 *
 * @brief Replace default behavor. Required on provision intiator
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 * @param provision_postprocess - os_event_fn*
 *
 * output parameters
 *
 * returns none
 */
void 
dw1000_provision_set_postprocess(dw1000_dev_instance_t * inst, os_event_fn * provision_postprocess){
    assert(inst != NULL);
    assert(inst->provision != NULL);
    dw1000_provision_instance_t * provision = inst->provision; 
    os_callout_init(&provision->provision_callout_postprocess, os_eventq_dflt_get(), provision_postprocess, (void *) inst);
    provision->config.postprocess = true;
}

/*! 
 * @fn provision_postprocess(struct os_event * ev)
 *
 * @brief This a template which should be replaced by the provision intiator by a event that can just pass the
 * information to some other layers like application to kick start some other event. Currently just prints out
 * the information about the provisioned devices
 *
 * input parameters
 * @param ev - struct os_event *
 *
 * output parameters
 *
 * returns none
 */
static void
provision_postprocess(struct os_event * ev){
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);
    printf("Default post process implementation \n");
}

/*! 
 * @fn provision_rx_complete_cb(dw1000_dev_instance_t * inst)
 *
 * @brief This is an internal static function that executes on both the provision intiator and the TAG/ANCHOR 
 * that replies to the beacon. On the provision intiator the postprecess function can send the database to the application
 * and kick start some other task like ranging or so. In responder there is no need of post process and should just go to
 * rx mode again 
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 *
 * output parameters
 *
 * returns none
 */
static void
provision_rx_complete_cb(dw1000_dev_instance_t* inst){
    assert(inst != NULL);
    assert(inst->provision != NULL);
    uint16_t  frame_idx = inst->provision->idx;
    uint16_t code, dst_address;
    dw1000_provision_instance_t * provision = inst->provision;
    dw1000_provision_config_t config = provision->config;

    if (inst->fctrl == 0x8841){
        dw1000_read_rx(inst, (uint8_t *) &code, offsetof(ieee_rng_request_frame_t,code), sizeof(uint16_t));
        dw1000_read_rx(inst, (uint8_t *) &dst_address, offsetof(ieee_rng_request_frame_t,dst_address), sizeof(uint16_t));
    }else{
        return;
    }
    if ((dst_address != inst->my_short_address) && (dst_address != (uint16_t)0xFFFF))
    {
        return;
    }
    provision_frame_t * frame = &provision->frames[(frame_idx)%provision->nframes];
    switch(code)
    {
        case DWT_PROVISION_START:
            {
                if (inst->frame_len >= sizeof(ieee_rng_request_frame_t))
                    dw1000_read_rx(inst, frame->array, 0, sizeof(ieee_rng_request_frame_t));
                else{
                    if (dw1000_restart_rx(inst, inst->control).start_rx_error)
                        inst->rng_rx_error_cb(inst);
                    break;
                }
                uint8_t delay_factor = 1;  //Delay_factor for NODE_0
                if(inst->slot_id > 0) // if device is of NODE type
                   delay_factor = (inst->slot_id) * 4;  //Increase the delay factor for late response for Anchor provisioning
                uint64_t request_timestamp = dw1000_read_rxtime(inst);
                uint64_t response_tx_delay = request_timestamp + ((uint64_t)(config.tx_holdoff_delay*delay_factor) << 16);
                frame->dst_address = frame->src_address;
                frame->src_address = inst->my_short_address;
                frame->code = DWT_PROVISION_RESP;

                dw1000_write_tx(inst, frame->array, 0, sizeof(ieee_rng_response_frame_t));
                dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_response_frame_t), 0, true);
                dw1000_set_wait4resp(inst,true);
                dw1000_set_delay_start(inst, response_tx_delay);
                dw1000_set_rx_timeout(inst,0);
                if (dw1000_start_tx(inst).start_tx_error){
                    printf("DWT_PROVISION_START tx error\n");
                }
                break;
            }
        case DWT_PROVISION_RESP:
            {
                if (inst->frame_len >= sizeof(ieee_rng_response_frame_t))
                    dw1000_read_rx(inst, frame->array, 0, sizeof(ieee_rng_response_frame_t));
                else{
                    if (dw1000_restart_rx(inst,inst->control).start_rx_error)
                        inst->rng_rx_error_cb(inst);
                    break;
                }
                if(provision_add_node(inst,frame->src_address) == PROVISION_ERROR){
                    //If the addition fails when the number of nodes exceeds max allowed count callout the postprocess
                    os_error_t err = os_sem_release(&provision->sem);
                    assert(err == OS_OK);
                    provision->status.provision_status = PROVISION_DONE;
                    if (provision->config.postprocess){
                        os_eventq_put(os_eventq_dflt_get(), &provision->provision_callout_postprocess.c_ev);
                    }
                    break;
                }
                if (dw1000_restart_rx(inst, inst->control).start_rx_error)
                    inst->rng_rx_error_cb(inst);
                break;
            }
        default:
            printf("Wrong request \n");
    }
}

/*! 
 * @fn dw1000_rx_timeout_cb(dw1000_dev_instance_t * inst)
 *
 * @brief Handle the rx timeout case for provisioning
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * inst
 *
 * output parameters
 *
 * returns none
 */
static void
provision_rx_timeout_cb(dw1000_dev_instance_t * inst){
    assert(inst != NULL);
    assert(inst->provision != NULL);
    dw1000_provision_instance_t *provision = inst->provision;
    if(provision->status.provision_status == PROVISION_START){
        os_error_t err = os_sem_release(&provision->sem);
        assert(err == OS_OK);
		provision->status.provision_status = PROVISION_DONE;
        if (provision->config.postprocess){
            os_eventq_put(os_eventq_dflt_get(), &provision->provision_callout_postprocess.c_ev);
        }
    }else{
        inst->control = inst->control_rx_context;
        dw1000_start_rx(inst);
    }
}

/*! 
 * @fn dw1000_rx_error_cb(dw1000_dev_instance_t * inst)
 *
 * @brief Handle the rx error case for provisioning
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * inst
 *
 * output parameters
 *
 * returns none
 */
static void
provision_rx_error_cb(dw1000_dev_instance_t * inst){
    assert(inst != NULL);
    assert(inst->provision != NULL);
    if(inst->provision->status.provision_status == PROVISION_START){
        os_error_t err = os_sem_release(&inst->provision->sem);
        assert(err == OS_OK);
        inst->provision->status.provision_status = PROVISION_DONE;
    }else{
        inst->control = inst->control_rx_context;
        dw1000_start_rx(inst);
    }
}

/*! 
 * @fn dw1000_tx_complete_cb(dw1000_dev_instance_t * inst)
 *
 * @brief Handle the tx complete case. If required add the support
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * inst
 *
 * output parameters
 *
 * returns none
 */
static void
provision_tx_complete_cb(dw1000_dev_instance_t * inst){
    //Place holder
}

/*! 
 * @fn dw1000_provision_request(dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode)
 *
 * @brief Provisioning is a phase where a Tag finds all the adjucent Nodes or Nodes find 
 * nodes adjacent to it. The outcome is a database of all the nearby nodes   
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 * @param mode - dw1000_devmodes_t for DWT_BLOCKING, DWT_NONBLOCKING.
 *
 * output parameters
 *
 * returns dw1000_provision_status_t 
 */
dw1000_provision_status_t 
dw1000_provision_request(dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode){

    os_error_t err = os_sem_pend(&inst->provision->sem,OS_TIMEOUT_NEVER);
    assert(err == OS_OK);
    dw1000_provision_instance_t * provision = inst->provision;
    provision_frame_t * frame = &provision->frames[(provision->idx++)%provision->nframes];

    provision->status.provision_status = PROVISION_START;
    frame->seq_num++;
    frame->fctrl = 0x8841;
    frame->PANID = 0xDECA;
    frame->code = DWT_PROVISION_START;
    frame->src_address = inst->my_short_address;
    frame->dst_address = BROADCAST_ADDRESS;
    dw1000_write_tx(inst, frame->array, 0, sizeof(ieee_rng_response_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_response_frame_t), 0, true);
    dw1000_set_wait4resp(inst, true);
    dw1000_set_rx_timeout(inst, provision->config.rx_timeout_period);
    provision->status.start_tx_error = dw1000_start_tx(inst).start_tx_error;
    if (provision->status.start_tx_error){
        os_sem_release(&inst->provision->sem);
        provision->status.provision_status = PROVISION_DONE;
    }
    else if(mode == DWT_BLOCKING){
        err = os_sem_pend(&inst->provision->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions 
        os_sem_release(&inst->provision->sem);
    }
   return provision->status;
}

/*!
 * @fn dw1000_provision_start(dw1000_dev_instance_t * inst)
 *
 * @brief This function schedules the provision timer event.This function
 * should be called for a TAG/NODE which should send a BEACON and the other
 * node should be in receive mode
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * inst
 *
 * output parameters
 *
 * returns none
 */
void
dw1000_provision_start(dw1000_dev_instance_t * inst){
    assert(inst != NULL);
    assert(inst->provision != NULL);
    dw1000_provision_instance_t * provision = inst->provision;
    provision->idx = 0x0;
    provision->status.valid = true;
    provision_timer_init(inst);
}

/*!
 * @fn dw1000_provision_stop(dw1000_dev_instance_t * inst)
 *
 * @brief This function stops the provision process
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * inst
 *
 * output parameters
 *
 * returns none
 */
void
dw1000_provision_stop(dw1000_dev_instance_t * inst){
    assert(inst != NULL);
    assert(inst->provision != NULL);
    inst->provision->status.valid = false;
    os_callout_stop(&inst->provision->provision_callout_timer);
}

/*!
 * @fn provision_add_node(dw1000_dev_instance_t *inst, uint16_t addr)
 *
 * @brief This function adds a new node to its database and stores it in provision's
 * instance
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * inst
 * @param addr - short address of the device to be added
 *
 * output parameters
 *
 * returns dw1000_provision_error_t
 */
dw1000_provision_error_t
provision_add_node(dw1000_dev_instance_t *inst, uint16_t addr){
    assert(inst != NULL);
    assert(inst->provision != NULL);
    dw1000_provision_instance_t *provision = inst->provision;
    if((provision->num_node_count+1) > provision->config.max_node_count)
        return PROVISION_ERROR;
    provision->dev_addr[provision->num_node_count++] = addr;
    return PROVISION_SUCCESS;
}

/*!
 * @fn provision_delete_node(dw1000_dev_instance_t *inst, uint16_t addr)
 *
 * @brief This function deletes a node from its database
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * inst
 * @param addr - short address of the device to be removed
 *
 * output parameters
 *
 * returns dw1000_provision_error_t
 */
dw1000_provision_error_t
provision_delete_node(dw1000_dev_instance_t *inst, uint16_t addr){
    assert(inst != NULL);
    assert(inst->provision != NULL);
    dw1000_provision_instance_t *provision = inst->provision;
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
#endif
