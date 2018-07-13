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

typedef union {
    struct _ccp_frame_t{
        struct _ieee_blink_frame_t;
        uint64_t transmission_timestamp;    // transmission timestamp.
        uint64_t reception_timestamp;       // reception timestamp.
        float correction_factor;         // receiver clock correction factor
    }__attribute__((__packed__, aligned(1)));
    uint8_t array[sizeof(struct _ieee_blink_frame_t)];
}ccp_frame_t;

typedef struct _dw1000_ccp_status_t{
    uint16_t selfmalloc:1;
    uint16_t initialized:1;
    uint16_t valid:1;
    uint16_t start_tx_error:1;
    uint16_t timer_enabled:1;
}dw1000_ccp_status_t;

typedef struct _dw1000_ccp_config_t{
    uint16_t postprocess:1;
    uint16_t fs_xtalt_autotune:1;
}dw1000_ccp_config_t;

typedef struct _dw1000_ccp_instance_t{
    struct _dw1000_dev_instance_t * parent;
#if MYNEWT_VAL(CLOCK_CALIBRATION_ENABLED)
    struct _clkcal_instance_t * clkcal;
#endif
#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED)
    struct _sos_instance_t * xtalt_sos;
#endif
    struct os_sem sem;
    struct os_callout callout_timer;
    struct os_callout callout_postprocess;
    dw1000_ccp_status_t status;
    dw1000_ccp_config_t config;
    uint32_t period;
    uint16_t nframes;
    uint16_t idx;
    ccp_frame_t * frames[];
}dw1000_ccp_instance_t; 

dw1000_ccp_instance_t * dw1000_ccp_init(dw1000_dev_instance_t * inst,  uint16_t nframes, uint64_t clock_master);
void dw1000_ccp_free(dw1000_ccp_instance_t * inst);
void dw1000_ccp_set_ext_callbacks(dw1000_dev_instance_t * inst, dw1000_extension_callbacks_t ccp_cbs);
void dw1000_ccp_set_postprocess(dw1000_ccp_instance_t * inst, os_event_fn * ccp_postprocess); 
void dw1000_ccp_start(dw1000_dev_instance_t * inst);
void dw1000_ccp_stop(dw1000_dev_instance_t * inst);

#ifdef __cplusplus
}
#endif
#endif /* _DW1000_CCP_H_ */
