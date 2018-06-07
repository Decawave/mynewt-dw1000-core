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

typedef struct _dw1000_range_config_t{
    uint16_t postprocess:1;
    dw1000_rng_modes_t code:8;
}dw1000_range_config_t;

typedef struct _dw1000_range_status_t{
    uint16_t selfmalloc:1;
    uint16_t initialized:1;
    uint16_t started:1;
    uint16_t valid:1;
    uint16_t start_tx_error:1;
    uint16_t rx_timeout_error:1;
    uint16_t rx_error:1;
    uint16_t request_timeout_error:1;
    uint16_t timer_enabled:1;
}dw1000_range_status_t;

typedef struct _dw1000_range_instance_t{
    struct _dw1000_dev_instance_t * parent;
    dw1000_range_status_t status;
    dw1000_range_config_t config;
    os_event_fn *postprocess;
    struct os_sem sem;
    uint32_t period;
    uint8_t idx;
    uint16_t nnodes;
    uint16_t *node_addr;
    uint16_t rng_idx_cnt;
    uint16_t *rng_idx_list;
    uint16_t pp_idx_cnt;
    uint16_t *pp_idx_list;
    uint16_t var_mem_block[];
}dw1000_range_instance_t;

dw1000_range_instance_t * dw1000_range_init(dw1000_dev_instance_t * inst, uint16_t nnodes, uint16_t node_addr[]);
void dw1000_range_free(dw1000_dev_instance_t * inst);
void dw1000_range_set_callbacks(dw1000_dev_instance_t * inst, dw1000_dev_cb_t dw1000_range_cb, dw1000_dev_cb_t dw1000_error_cb);
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
