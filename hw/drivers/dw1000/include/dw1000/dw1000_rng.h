/**
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

#ifndef _DW1000_RNG_H_
#define _DW1000_RNG_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/hal_spi.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_ftypes.h>

typedef struct _dw1000_rng_config_t{
   uint32_t wait4resp_delay;         // Delay between frames, in UWB usec.
   uint16_t rx_timeout_period;       // Receive response timeout, in UWB usec.
}dw1000_rng_config_t;

typedef enum _dw1000_rng_modes_t{
    DW1000_SS_TWR=0,
    DW1000_SS_TWR_T1,
    DW1000_DS_TWR,
    DW1000_DS_TWR_T1,
    DW1000_DS_TWR_T2,
    DW1000_TOA
}dw1000_rng_modes_t;

typedef struct _dw1000_rng_status_t{
    uint16_t selfmalloc:1;
    uint16_t initialized:1;
    uint16_t mac_error:1;
}dw1000_rng_status_t;

typedef struct _ss_twr_range_t{
    union {
        ieee_rng_request_frame_t request;
        ieee_rng_response_frame_t response;
    }__attribute__((__packed__)); 
    uint32_t request_timestamp;     // request transmission timestamp.
    uint32_t response_timestamp;    // reception reception timestamp.
}ss_twr_range_t;

typedef struct _ds_twr_range_t{
    union {
        ieee_rng_request_frame_t request;
        ieee_rng_response_frame_t response;
    }__attribute__((__packed__)); 
    uint32_t request_timestamp[2];     // request transmission timestamp.
    uint32_t response_timestamp[2];    // reception reception timestamp.
}ds_twr_range_t;

typedef struct _dw1000_rng_instance_t{
    struct _dw1000_dev_instance_t * dev;
    struct os_sem sem;
    ss_twr_range_t * ss_twr;
    ds_twr_range_t * ds_twr;
    dw1000_rng_config_t * config;
    dw1000_rng_status_t status;
}dw1000_rng_instance_t;




dw1000_rng_instance_t * dw1000_rng_init(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config);
void dw1000_rng_free(dw1000_rng_instance_t * inst);
dw1000_dev_status_t dw1000_rng_config(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config);
void dw1000_rng_set_callbacks(dw1000_dev_instance_t * inst,  dw1000_dev_cb_t rng_tx_complete_cb, dw1000_dev_cb_t rng_rx_complete_cb, dw1000_dev_cb_t rng_rx_timeout_cb,  dw1000_dev_cb_t rng_rx_error_cb);
dw1000_dev_status_t dw1000_rng_request(dw1000_dev_instance_t * inst, uint16_t dst_address, dw1000_rng_modes_t protocal);
void dw1000_rng_set_frames(dw1000_dev_instance_t * inst, ss_twr_range_t * range);


#ifdef __cplusplus
}
#endif
#endif /* _DW1000_RNG_H_ */
