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
 * @file dw1000_ccp.h
 * @author paul kettle
 * @date 2018
 * 
 * @brief clock calibration packets
 * @details This is the ccp base class which utilises the functions to enable/disable the configurations related to ccp.
 *
 */

#ifndef _DW1000_CCP_H_
#define _DW1000_CCP_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/hal_spi.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_ftypes.h>
#if MYNEWT_VAL(CLOCK_CALIBRATION_ENABLED)
#include <clkcal/clkcal.h>    
#endif
#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED)
#include <dsp/sosfilt.h>
#include <dsp/polyval.h>
#endif

//! Timestamps and blink frame format  of ccp frame.
typedef union {
//! Frame format of ccp.
    struct _ccp_frame_t{
//! Frame format of blink frame.
        struct _ieee_blink_frame_t;          
        uint64_t transmission_timestamp;    //!< Transmission timestamp
        uint64_t reception_timestamp;       //!< Reception timestamp
        float correction_factor;            //!< Receiver clock correction factor
    }__attribute__((__packed__, aligned(1)));
    uint8_t array[sizeof(struct _ccp_frame_t)];
}ccp_frame_t;

//! Status parameters of ccp.
typedef struct _dw1000_ccp_status_t{
    uint16_t selfmalloc:1;            //!< Internal flag for memory garbage collection 
    uint16_t initialized:1;           //!< Instance allocated 
    uint16_t valid:1;                 //!< Set for valid parameters 
    uint16_t start_tx_error:1;        //!< Set for start transmit error 
    uint16_t start_rx_error:1;        //!< Set for start request error 
    uint16_t timer_enabled:1;         //!< Indicates timer is enabled 
}dw1000_ccp_status_t;

//! ccp config parameters.  
typedef struct _dw1000_ccp_config_t{
    uint16_t postprocess:1;           //!< CCP postprocess
    uint16_t fs_xtalt_autotune:1;     //!< Autotune XTALT to Clock Master
}dw1000_ccp_config_t;

//! Extension ids for services.
typedef enum _dw1000_ccp_role_t{
    CCP_ROLE_MASTER,                        //!< Clock calibration packet master mode
    CCP_ROLE_SLAVE                          //!< Clock calibration packet slave mode
}dw1000_ccp_role_t;

//! ccp instance parameters.
typedef struct _dw1000_ccp_instance_t{
    struct _dw1000_dev_instance_t * parent;     //!< Pointer to _dw1000_dev_instance_t
#if MYNEWT_VAL(CLOCK_CALIBRATION_ENABLED)
    struct _clkcal_instance_t * clkcal;         //!< Wireless clock calibration 
#endif
#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED)
    struct _sos_instance_t * xtalt_sos;        //!< Sturcture of xtalt_sos 
#endif
    dw1000_mac_interface_t cbs;                     //!< MAC Layer Callbacks
    uint64_t uuid;                                  //!< Clock Master UUID
    struct os_sem sem;                              //!< Structure containing os semaphores
    struct os_callout callout_postprocess;          //!< Structure of callout_postprocess
    dw1000_ccp_status_t status;                     //!< DW1000 ccp status parameters
    dw1000_ccp_config_t config;                     //!< DW1000 ccp config parameters
    uint64_t epoch;
    uint32_t os_epoch;
    uint32_t period;                                //!< Pulse repetition period
    uint16_t nframes;                               //!< Number of buffers defined to store the data 
    uint16_t idx;                                   //!< Indicates number of DW1000 instances 
    struct hal_timer timer;                         //!< Timer structure
    struct os_eventq eventq;                        //!< Event queues
    struct os_callout event_cb;                     //!< Event callback
    struct os_task task_str;                        //!< os_task structure  
    uint8_t task_prio;                              //!< Priority based task
    os_stack_t task_stack[DW1000_DEV_TASK_STACK_SZ]
        __attribute__((aligned(OS_STACK_ALIGNMENT))); //!< Task stack size
    ccp_frame_t * frames[];                          //!< Buffers to ccp frames
}dw1000_ccp_instance_t; 

dw1000_ccp_instance_t * dw1000_ccp_init(dw1000_dev_instance_t * inst,  uint16_t nframes, uint64_t clock_master);
void dw1000_ccp_free(dw1000_ccp_instance_t * inst);
void dw1000_ccp_set_postprocess(dw1000_ccp_instance_t * inst, os_event_fn * ccp_postprocess); 
void dw1000_ccp_start(dw1000_dev_instance_t * inst, dw1000_ccp_role_t role);
void dw1000_ccp_stop(dw1000_dev_instance_t * inst);

#ifdef __cplusplus
}
#endif
#endif /* _DW1000_CCP_H_ */
