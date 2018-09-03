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
 * @file dw1000_pan.h
 * @author paul kettle
 * @date 2018
 * @brief Personal Area Network
 *
 * @details This is the pan base class which utilises the functions to allocate/deallocate the resources on pan_master,sets callbacks, enables  * blink_requests.
 *
 */

#ifndef _DW1000_PAN_H_
#define _DW1000_PAN_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/hal_spi.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_rng.h>
#include <dw1000/dw1000_ftypes.h>

//! Union of response frame format
typedef union{
//! Structure containing pan response frame format
    struct _pan_frame_resp_t{
//! Structure of IEEE blink frame
        struct _ieee_blink_frame_t;
        uint16_t pan_id;                     //!< Assigned pan_id
        uint16_t short_address;              //!< Assigned device_id
        uint8_t slot_id;                     //!< Assigned slot_id
    }__attribute__((__packed__, aligned(1)));
    uint8_t array[sizeof(struct _pan_frame_resp_t)];
}pan_frame_resp_t;

//! Union of pan frame format
typedef union {
//! Structure of pan frame format
    struct _pan_frame_t{
        struct _pan_frame_resp_t;
        uint64_t transmission_timestamp;    //!< Transmission timestamp
        uint64_t reception_timestamp;       //!< Reception timestamp
        float correction_factor;            //!< Receiver clock correction factor
    }__attribute__((__packed__, aligned(1)));
    uint8_t array[sizeof(struct _pan_frame_t)];
}pan_frame_t;

//! Pan status parameters
typedef struct _dw1000_pan_status_t{
    uint16_t selfmalloc:1;                 //!< Internal flag for memory garbage collection
    uint16_t initialized:1;                //!< Instance allocated
    uint16_t valid:1;                      //!< Set for valid parameters
    uint16_t start_tx_error:1;             //!< Set for start transmit error
    uint16_t timer_enabled:1;              //!< Indicates timer is enabled
}dw1000_pan_status_t;

//! Pan configure parameters
typedef struct _dw1000_pan_config_t{
    uint32_t rx_holdoff_delay;        //!< Delay between frames, in UWB usec.
    uint16_t rx_timeout_period;       //!< Receive response timeout, in UWB usec.
    uint32_t tx_holdoff_delay;        //!< Delay between frames, in UWB usec.
}dw1000_pan_config_t;

//! Pan control parameters 
typedef struct _dw1000_pan_control_t{
    uint16_t postprocess:1;           //!< Pan postprocess
}dw1000_pan_control_t;

//! Pan instance parameters
typedef struct _dw1000_pan_instance_t{
    struct _dw1000_dev_instance_t * parent;      //!< pointer to _dw1000_dev_instance_t
    struct os_sem sem;                           //!< Structure containing os semaphores
    struct os_sem sem_waitforsucess;             //!< Structure containig os semaphores
    dw1000_pan_status_t status;                  //!< DW1000 pan status parameters
    dw1000_pan_control_t control;                //!< DW1000 pan control parameters
    struct os_callout pan_callout_timer;         //!< Structure of pan_callout_timer
    struct os_callout pan_callout_postprocess;   //!< Structure of pan_callout_postprocess
    dw1000_pan_config_t * config;                //!< DW1000 pan config parameters
    uint32_t period;                             //!< Pulse repetition period
    uint16_t nframes;                            //!< Number of buffers defined to store the data
    uint16_t idx;                                //!< Indicates number of DW1000 instances
    pan_frame_t * frames[];                      //!< Buffers to pan frames
}dw1000_pan_instance_t; 

dw1000_pan_instance_t * dw1000_pan_init(dw1000_dev_instance_t * inst,  dw1000_pan_config_t * config);
void dw1000_pan_free(dw1000_dev_instance_t * inst);
void dw1000_pan_set_ext_callbacks(dw1000_dev_instance_t * inst, dw1000_extension_callbacks_t pan_cbs);
void dw1000_pan_set_postprocess(dw1000_dev_instance_t * inst, os_event_fn * postprocess); 
void dw1000_pan_start(dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode);
void dw1000_pan_stop(dw1000_dev_instance_t * inst);

#ifdef __cplusplus
}
#endif
#endif /* _DW1000_PAN_H_ */
