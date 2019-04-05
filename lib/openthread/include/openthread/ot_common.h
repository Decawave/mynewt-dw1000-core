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
 * @file dw1000_lwip.h
 * @author paul kettle
 * @date 2018
 * 
 * @brief lwip service
 * @details This is the lwip base class which utilizes the functions to do the configurations related to lwip layer based on dependencies.
 *
 */

#ifndef _OT_COMMON_H_
#define _OT_COMMON_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/hal_spi.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_ftypes.h>
#include <dw1000/dw1000_phy.h>

#include <openthread/instance.h>

#define MAX_OT_FRAMELEN 127
#define LOG_PARSE_BUFFER_SIZE 128
#define IEEE802154_FRAME_PENDING (1 << 4)

typedef struct _ot_status_t{
    uint32_t selfmalloc:1;             //!< Internal flag for memory garbage collection 
    uint32_t initialized:1;            //!< Instance allocated 
    uint32_t start_tx_error:1;         //!< Set for start transmit error 
    uint32_t start_rx_error:1;         //!< Set for start receive error 
    uint32_t tx_frame_error:1;         //!< Set transmit frame error
    uint32_t rx_error:1;               //!< Set for receive error
    uint32_t rx_timeout_error:1;       //!< Set for receive timeout error 
    uint32_t request_timeout:1;        //!< Set for request timeout
}ot_status_t;

typedef struct _ot_instance_t{
    struct _dw1000_dev_instance_t * dev;               //!< Structure for DW1000 instance 
    dw1000_mac_interface_t cbs;                        //!< OT mac interface callbacks
    struct os_sem sem;                                 //!< Structure for OS semaphores
    otInstance *sInstance;                             //!< Instance to OT stack
    ot_status_t status;                                //!< OT error status
    struct os_eventq eventq;                           //!< Structure of os events
    os_stack_t task_stack[DW1000_DEV_TASK_STACK_SZ * 4]    //!< Stack size of each task
        __attribute__((aligned(OS_STACK_ALIGNMENT)));
    uint8_t task_prio;                                 //!< Task priority
    struct os_task task_str;                           //!< Structure for task
}ot_instance_t;


ot_instance_t *
ot_init(dw1000_dev_instance_t * inst);
void ot_post_init(dw1000_dev_instance_t * inst, otInstance *aInstance);

void
ot_free(ot_instance_t * inst);

void RadioInit(dw1000_dev_instance_t* inst);
void PlatformInit(dw1000_dev_instance_t* inst);

#ifdef __cplusplus
}
#endif
#endif /* _OT_COMMON_H_ */
