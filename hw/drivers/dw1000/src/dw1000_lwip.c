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
#include <dw1000/dw1000_ftypes.h>
#include <dw1000/dw1000_lwip.h>


// TODOs::This file is a place holder for the lwip project

static void rx_complete_cb(dw1000_dev_instance_t * inst);
static void tx_complete_cb(dw1000_dev_instance_t * inst);
static void rx_timeout_cb(dw1000_dev_instance_t * inst);
static void rx_error_cb(dw1000_dev_instance_t * inst);

dw1000_lwip_instance_t * 
dw1000_lwip_init(dw1000_dev_instance_t * inst, dw1000_lwip_config_t * config){

    dw1000_lwip_instance_t * lwip = inst->lwip;
    assert(inst);
    if (lwip == NULL ){
        lwip = inst->lwip  = (dw1000_lwip_instance_t *) malloc(sizeof(dw1000_lwip_instance_t));
        assert(lwip);
        memset(lwip,0,sizeof(dw1000_lwip_instance_t));
        lwip->status.selfmalloc = 1;
    }

    os_error_t err = os_sem_init(&lwip->sem, 0xFFFF); 
    assert(err == OS_OK);

    dw1000_lwip_set_callbacks(inst->lwip, tx_complete_cb, rx_complete_cb, rx_timeout_cb, rx_error_cb);
    lwip->status.initialized = 1;

    return lwip;
}


void 
dw1000_lwip_free(dw1000_lwip_instance_t * inst){

    assert(inst);  
    if (inst->status.selfmalloc)
        free(inst);
    else
        inst->status.initialized = 0;
}

void 
dw1000_lwip_set_callbacks(dw1000_lwip_instance_t * inst, dw1000_dev_cb_t tx_complete_cb, dw1000_dev_cb_t rx_complete_cb,  dw1000_dev_cb_t rx_timeout_cb,  dw1000_dev_cb_t rx_error_cb)
{
    inst->dev->tx_complete_cb = tx_complete_cb;
    inst->dev->rx_complete_cb = rx_complete_cb;
    inst->dev->rx_timeout_cb = rx_timeout_cb;
    inst->dev->rx_error_cb = rx_error_cb;
}

dw1000_lwip_status_t 
dw1000_lwip_write(dw1000_lwip_instance_t * inst, dw1000_lwip_config_t * rng_config, dw1000_lwip_modes_t mode){

    /* Semaphore lock for multi-threaded applications */
    os_error_t err = os_sem_pend(&inst->sem, OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    // TODOs::
    return inst->status;
}


static void 
rx_complete_cb(dw1000_dev_instance_t * inst){

    dw1000_lwip_instance_t * lwip = (dw1000_lwip_instance_t * ) inst->lwip;

    hal_gpio_toggle(LED_1);

    os_error_t err = os_sem_release(&lwip->sem);
    assert(err == OS_OK);
}

static void 
tx_complete_cb(dw1000_dev_instance_t * inst){

    dw1000_lwip_instance_t * lwip = (dw1000_lwip_instance_t * ) inst->lwip;

    os_error_t err = os_sem_release(&lwip->sem);
    assert(err == OS_OK);
}

static void 
rx_timeout_cb(dw1000_dev_instance_t * inst){

    dw1000_lwip_instance_t * lwip = (dw1000_lwip_instance_t * ) inst->lwip;

    os_error_t err = os_sem_release(&lwip->sem);
    assert(err == OS_OK);

}

static void 
rx_error_cb(dw1000_dev_instance_t * inst){

    dw1000_lwip_instance_t * lwip = (dw1000_lwip_instance_t * ) inst->lwip;
   
    os_error_t err = os_sem_release(&lwip->sem);
    assert(err == OS_OK);

}
