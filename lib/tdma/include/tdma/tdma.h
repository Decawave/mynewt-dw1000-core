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
 * @file dw1000_tdma.h
 * @author paul kettle
 * @date 2018
 * @brief TDMA  
 *
 * @details  This is the base class of tdma which initialises tdma instance, assigns slots for each node and does ranging continuously based on * addresses.
 */
#ifndef _DW1000_TDMA_H_
#define _DW1000_TDMA_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <dw1000/dw1000_ftypes.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_phy.h>
#include <os/queue.h>


#define TDMA_TASKS_ENABLE

//! Structure of TDMA
typedef struct _tdma_status_t{
    uint16_t selfmalloc:1;            //!< Internal flag for memory garbage collection
    uint16_t initialized:1;           //!< Instance allocated 
    uint16_t awaiting_superframe:1;   //!< Superframe of tdma
}tdma_status_t;

//! Structure of tdma_slot
typedef struct _tdma_slot_t{
    struct _tdma_instance_t * parent;  //!< Pointer to _tdma_instance_ti
    struct hal_timer timer;            //!< Timer
    struct os_callout event_cb;        //!< Sturcture of event_cb
    uint16_t idx;                      //!< Slot number
    void * arg;                      //!< Optional argument
}tdma_slot_t; 

//! Structure of tdma instance
typedef struct _tdma_instance_t{
    struct _dw1000_dev_instance_t * parent;  //!< Pointer to _dw1000_dev_instance_t
    tdma_status_t status;                    //!< Status of tdma 
    dw1000_mac_interface_t cbs;              //!< MAC Layer Callbacks
    struct os_mutex mutex;                   //!< Structure of os_mutex  
    uint16_t idx;                            //!< Slot number
    uint16_t nslots;                         //!< Number of slots 
    uint32_t period;                         //!< Period of each tdma
    uint32_t os_epoch;                          //!< Epoch timestamp
    struct os_callout event_cb;              //!< Sturcture of event_cb
#ifdef TDMA_TASKS_ENABLE
    struct os_eventq eventq;                 //!< Structure of os events
    struct os_task task_str;                 //!< Structure of os tasks
    uint8_t task_prio;                       //!< Priority of tasks
    os_stack_t task_stack[DW1000_DEV_TASK_STACK_SZ]   //!< Stack size of each task
        __attribute__((aligned(OS_STACK_ALIGNMENT)));
#endif
    struct _tdma_slot_t * slot[];           //!< Dynamically allocated slot
}tdma_instance_t; 

struct _tdma_instance_t * tdma_init(struct _dw1000_dev_instance_t * inst, uint32_t period, uint16_t nslots);
void tdma_free(struct _tdma_instance_t * inst);
void tdma_assign_slot(struct _tdma_instance_t * inst, void (* callout )(struct os_event *), uint16_t idx, void * arg);
void tdma_release_slot(struct _tdma_instance_t * inst, uint16_t idx);

#ifdef __cplusplus
}
#endif

#endif //_DW1000_TDMA_H_
