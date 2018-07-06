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

#ifndef _DW1000_RNG_H_
#define _DW1000_RNG_H_

#include <stdlib.h>
#include <stdint.h>
#include "dw1000/triad.h"

#if MYNEWT_VAL(DW1000_SS_TWR_ENABLED)
#define SS_TWR_ENABLE
#endif
#if MYNEWT_VAL(DW1000_DS_TWR_ENABLED)
#define DS_TWR_ENABLE
#endif
#if MYNEWT_VAL(DW1000_DS_TWR_EXT_ENABLED)
#define DS_TWR_EXT_ENABLE
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/hal_spi.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_ftypes.h>
#include <dw1000/triad.h>

typedef struct _dw1000_rng_config_t{
   uint32_t rx_holdoff_delay;        // Delay between frames, in UWB usec.
   uint32_t tx_holdoff_delay;        // Delay between frames, in UWB usec.
   uint16_t rx_timeout_period;       // Receive response timeout, in UWB usec.
   uint16_t bias_correction:1;       // Enable range bias correction
}dw1000_rng_config_t;

typedef struct _dw1000_rng_control_t{
    uint16_t delay_start_enabled:1;
}dw1000_rng_control_t;

typedef enum _dw1000_rng_modes_t{
    DWT_TWR_INVALID = 0,
    DWT_SS_TWR,
    DWT_SS_TWR_T1,
    DWT_SS_TWR_FINAL,
    DWT_SS_TWR_END,
    DWT_DS_TWR,
    DWT_DS_TWR_T1,
    DWT_DS_TWR_T2,
    DWT_DS_TWR_FINAL,
    DWT_DS_TWR_END,
    DWT_DS_TWR_EXT,
    DWT_DS_TWR_EXT_T1,
    DWT_DS_TWR_EXT_T2,
    DWT_DS_TWR_EXT_FINAL,
    DWT_DS_TWR_EXT_END,
    DWT_PROVISION_START,
    DWT_PROVISION_RESP,
}dw1000_rng_modes_t;

typedef struct _dw1000_rng_status_t{
    uint16_t selfmalloc:1;
    uint16_t initialized:1;
    uint16_t mac_error:1;
    uint16_t invalid_code_error:1;
}dw1000_rng_status_t;

typedef struct _twr_frame_final_t{
        struct _ieee_rng_response_frame_t;
        uint32_t request_timestamp;     // request transmission timestamp.
        uint32_t response_timestamp;    // response reception timestamp.
} __attribute__((__packed__, aligned(1))) twr_frame_final_t;

typedef struct _twr_data_t{
                uint64_t utime;
                triad_t spherical;                  // measurement triad spherical coordinates
                triad_t spherical_variance;         // measurement variance triad 
                triad_t cartesian;                  // position triad local coordinates
          //      triad_t cartesian_variance;         // position estimated variance triad 
}twr_data_t;

typedef union {
    struct _twr_frame_t{
        struct _twr_frame_final_t;
#ifdef DS_TWR_EXT_ENABLE
        union {
            struct _twr_data_t;
            uint8_t payload[sizeof(struct _twr_data_t)];
        };
#endif
    } __attribute__((__packed__, aligned(1)));
    uint8_t array[sizeof(struct _twr_frame_t)];
} twr_frame_t;

typedef struct _dw1000_rng_callbacks_t{
    void (* rng_tx_complete_cb) (struct _dw1000_dev_instance_t *);
    void (* rng_rx_complete_cb) (struct _dw1000_dev_instance_t *);
    void (* rng_rx_timeout_cb) (struct _dw1000_dev_instance_t *);
    void (* rng_rx_error_cb) (struct _dw1000_dev_instance_t *);
    void (* rng_tx_final_cb) (struct _dw1000_dev_instance_t *);
}dw1000_rng_callbacks_t;

typedef struct{
    void (* rng_rx_timeout_extension_cb) (struct _dw1000_dev_instance_t *);
    void (* rng_rx_error_extension_cb) (struct _dw1000_dev_instance_t *);
    void (* rng_interface_extension_cb) (struct _dw1000_dev_instance_t *);
}dw1000_rng_callbacks_extension_t;

typedef struct _dw1000_rng_instance_t{
    struct _dw1000_dev_instance_t * dev;
    struct os_sem sem;
    uint64_t delay;
    dw1000_rng_config_t * config;
    dw1000_rng_control_t control;
    dw1000_rng_status_t status;
    uint16_t idx;
    uint16_t nframes;
    twr_frame_t * frames[];
}dw1000_rng_instance_t; 

dw1000_rng_instance_t * dw1000_rng_init(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config, uint16_t nframes);
void dw1000_rng_free(dw1000_rng_instance_t * inst);
dw1000_dev_status_t dw1000_rng_config(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config);
void dw1000_rng_set_callbacks(dw1000_dev_instance_t * inst,  dw1000_dev_cb_t rng_tx_complete_cb, dw1000_dev_cb_t rng_rx_complete_cb, dw1000_dev_cb_t rng_rx_timeout_cb,  dw1000_dev_cb_t rng_rx_error_cb);
void dw1000_rng_set_callbacks_extension(dw1000_dev_instance_t * inst,  dw1000_dev_cb_t rng_rx_timeout_extension_cb, dw1000_dev_cb_t rng_rx_error_extension_cb,  dw1000_dev_cb_t rng_interface_extension_cb);
dw1000_dev_status_t dw1000_rng_request(dw1000_dev_instance_t * inst, uint16_t dst_address, dw1000_rng_modes_t protocal);
dw1000_dev_status_t dw1000_rng_request_delay_start(dw1000_dev_instance_t * inst, uint16_t dst_address, uint64_t delay, dw1000_rng_modes_t protocal);
void dw1000_rng_set_frames(dw1000_dev_instance_t * inst, twr_frame_t twr[], uint16_t nframes);
#if MYNEWT_VAL(DW1000_RANGE)
float dw1000_rng_twr_to_tof(twr_frame_t *fframe, twr_frame_t *nframe);
#else
float dw1000_rng_twr_to_tof(dw1000_rng_instance_t * rng);
#endif

float dw1000_rng_path_loss(float Pt, float G, float fc, float R);
float dw1000_rng_bias_correction(dw1000_dev_instance_t * inst, float Pr);
uint32_t dw1000_rng_twr_to_tof_sym(twr_frame_t twr[], dw1000_rng_modes_t code);
#define dw1000_rng_tof_to_meters(ToF) (float)(ToF * 299792458 * (1.0/499.2e6/128.0)) 
#define dw1000_rng_set_interface_extension_cb(inst, cb) inst->rng_interface_extension_cb = cb 
#define dw1000_rng_set_rx_timeout_extension_cb(inst, cb) inst->rng_rx_timeout_extension_cb = cb 
#define dw1000_rng_set_error_extension_cb(inst, cb) inst->rng_rx_error_extension_cb = cb
#define dw1000_rng_set_tx_final_cb(inst, cb) inst->rng_tx_final_cb = cb 
#define dw1000_rng_set_complete_cb(inst, cb) inst->rng_complete_cb = cb 

#ifdef __cplusplus
}
#endif
#endif /* _DW1000_RNG_H_ */
