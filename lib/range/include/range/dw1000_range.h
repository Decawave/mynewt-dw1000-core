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
 * @file dw1000_range.h
 * @author paul kettle
 * @date 2018
 * @brief Ranging
 * 
 * @details This is the range base class which utilises the functions to do ranging services using multiple nodes.
 */

#ifndef _DW1000_RANGE_H_
#define _DW1000_RANGE_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/hal_spi.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_ftypes.h>
#include <dw1000/dw1000_rng.h>

//! Range configuration parameters
typedef struct _dw1000_range_config_t{
    uint16_t postprocess:1;              //!< Sets postprocess
    dw1000_rng_modes_t code:8;           //!< Response code for the request 
}dw1000_range_config_t;

//! Range status parameters
typedef struct _dw1000_range_status_t{ 
    uint16_t selfmalloc:1;                //!< Internal flag for memory garbage collection
    uint16_t initialized:1;               //!< Instance allocated
    uint16_t started:1;                   //!< Starts range calculation
    uint16_t valid:1;                     //!< Sets for valid parameters
    uint16_t start_tx_error:1;            //!< Start transmit error
    uint16_t rx_timeout_error:1;          //!< Receive timeout error
    uint16_t rx_error:1;                  //!< Receive error
    uint16_t request_timeout_error:1;     //!< Request timeout error
    uint16_t timer_enabled:1;             //!< Indicates timer is enabled
}dw1000_range_status_t;

//! Structure of DW1000 range instance
typedef struct _dw1000_range_instance_t{
    struct _dw1000_dev_instance_t * parent;  //!< Device instance structure
    dw1000_range_status_t status;            //!< DW1000 range status
    dw1000_range_config_t config;            //!< DW1000 configuration
    os_event_fn *postprocess;                //!< An event of os   
    struct os_sem sem;                       //!< os_semaphore
    uint32_t period;                         //!< Range period
    uint8_t idx;                             //!< Index of nodes
    uint16_t nnodes;                         //!< NUmber of nodes to range with
    uint16_t *node_addr;                     //!< Address of each node
    uint16_t rng_idx_cnt;                    //!< To keep track of number of nodes ranged with
    uint16_t *rng_idx_list;                  //!< list of reserved addresses
    uint16_t pp_idx_cnt;                     //!< To keep track of number of nodes ranged with
    uint16_t *pp_idx_list;                   //!< list of reserved addresses
    uint16_t var_mem_block[];                //!< Dynamic memory block  
}dw1000_range_instance_t;

dw1000_range_instance_t * dw1000_range_init(dw1000_dev_instance_t * inst, uint16_t nnodes, uint16_t node_addr[]);
void dw1000_range_free(dw1000_dev_instance_t * inst);
void dw1000_range_set_ext_callbacks(dw1000_dev_instance_t * inst, dw1000_extension_callbacks_t range_cbs);
void dw1000_range_set_postprocess(dw1000_dev_instance_t * inst, os_event_fn * range_postprocess); 
void dw1000_range_start(dw1000_dev_instance_t * inst, dw1000_rng_modes_t code);
void dw1000_range_stop(dw1000_dev_instance_t * inst);
void dw1000_rng_reset_frames(dw1000_dev_instance_t * inst, twr_frame_t twr[], uint16_t nframes);
void dw1000_range_reset_nodes(dw1000_dev_instance_t * inst, uint16_t node_addr[], uint16_t nnodes);
void dw1000_range_set_nodes(dw1000_dev_instance_t * inst, uint16_t node_addr[], uint16_t nnodes);
#ifdef __cplusplus
}
#endif
#endif /* _DW1000_RANGE_H_ */
