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

#ifndef _DW1000_PROVISIONING_H_
#define _DW1000_PROVISIONING_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/hal_spi.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_ftypes.h>

typedef enum _dw1000_provision_state_t{
    PROVISION_INVALID,
    PROVISION_START,
    PROVISION_DONE,
}dw1000_provision_state_t;

typedef enum _dw1000_provision_error_t{
    PROVISION_SUCCESS,
    PROVISION_ERROR,
}dw1000_provision_error_t;

typedef struct _dw1000_provision_status_t{
    uint16_t selfmalloc:1;
    uint16_t initialized:1;
    uint16_t valid:1;
    uint16_t start_tx_error:1;
    uint16_t rx_timeout_error:1;
    dw1000_provision_state_t provision_status;
}dw1000_provision_status_t;

typedef struct _dw1000_provision_config_t{
   uint32_t tx_holdoff_delay;        // Delay between frames, in UWB usec.
   uint16_t rx_timeout_period;       // Receive response timeout, in UWB usec
   uint16_t period;
   uint16_t max_node_count; 
   uint16_t postprocess:1;
}dw1000_provision_config_t;

typedef union {
    struct _provision_frame_t{
        struct _ieee_rng_response_frame_t;
        uint32_t request_timestamp;     // request transmission timestamp.
        uint32_t response_timestamp;    // response reception timestamp.
    } __attribute__((__packed__, aligned(1)));
    uint8_t array[sizeof(struct _ieee_rng_response_frame_t)];
}provision_frame_t;

typedef struct _dw1000_provision_instance_t{
    struct _dw1000_dev_instance_t * parent;
    struct os_sem sem;
    dw1000_provision_status_t status;
    dw1000_provision_config_t config;
    struct os_callout provision_callout_timer;
    struct os_callout provision_callout_postprocess;
    uint16_t nframes;
    uint16_t idx;
    uint16_t num_node_count;
    provision_frame_t frames[2];  //No need to have a variable frame size for provisioning
    uint16_t dev_addr[];
}dw1000_provision_instance_t;

dw1000_provision_instance_t * dw1000_provision_init(dw1000_dev_instance_t * inst, dw1000_provision_config_t config);
dw1000_provision_status_t dw1000_provision_request(dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode);
void dw1000_provision_free(dw1000_dev_instance_t * inst);
void dw1000_provision_set_callbacks(dw1000_dev_instance_t * inst,  dw1000_dev_cb_t provision_rx_complete_cb,\
     dw1000_dev_cb_t provision_tx_complete_cb, dw1000_dev_cb_t provision_rx_timeout_cb, dw1000_dev_cb_t provision_rx_error_cb);
void dw1000_provision_start(dw1000_dev_instance_t * inst);
void dw1000_provision_stop(dw1000_dev_instance_t * inst);
void dw1000_provision_set_postprocess(dw1000_dev_instance_t * inst, os_event_fn * provision_postprocess);
dw1000_provision_error_t provision_add_node(dw1000_dev_instance_t *inst, uint16_t addr);
dw1000_provision_error_t provision_delete_node(dw1000_dev_instance_t *inst, uint16_t addr);
#ifdef __cplusplus
}
#endif
#endif /* _DW1001_RNG_H_ */
