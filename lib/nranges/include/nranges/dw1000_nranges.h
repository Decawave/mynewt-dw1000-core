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

#ifndef _DW1000_N_RANGES_H_
#define _DW1000_N_RANGES_H_

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

#define FCNTL_IEEE_N_RANGES_16 0x88C1

typedef enum _dw1000_nranges_modes_t{
    DWT_DS_TWR_NRNG = 17,
    DWT_DS_TWR_NRNG_T1,
    DWT_DS_TWR_NRNG_T2,
    DWT_DS_TWR_NRNG_FINAL,
    DWT_DS_TWR_NRNG_END,
    DWT_DS_TWR_NRNG_EXT,
    DWT_DS_TWR_NRNG_EXT_T1,
    DWT_DS_TWR_NRNG_EXT_T2,
    DWT_DS_TWR_NRNG_EXT_FINAL,
    DWT_DS_TWR_NRNG_EXT_END
}dw1000_nranges_modes_t;

typedef enum _dw1000_nranges_device_type_t{
    DWT_NRNG_INITIATOR,
    DWT_NRNG_RESPONDER
}dw1000_nranges_device_type_t;

typedef struct _dw1000_nranges_instance_t{
    uint16_t nnodes;
    uint16_t resp_count;
    uint16_t timeout_count;
    uint16_t t1_final_flag;
    dw1000_nranges_device_type_t device_type;
    struct os_sem sem;
}dw1000_nranges_instance_t;

dw1000_nranges_instance_t * dw1000_nranges_init(dw1000_dev_instance_t * inst,  dw1000_nranges_instance_t * nranges);
dw1000_dev_status_t dw1000_nranges_request_delay_start(dw1000_dev_instance_t * inst, uint16_t dst_address, uint64_t delay, dw1000_rng_modes_t code);
dw1000_dev_status_t dw1000_nranges_request(dw1000_dev_instance_t * inst, uint16_t dst_address, dw1000_nranges_modes_t code);
void dw1000_nranges_set_ext_callbacks(dw1000_dev_instance_t * inst, dw1000_extension_callbacks_t nranges_cbs);
void send_final_msg(dw1000_dev_instance_t * inst, twr_frame_t * frame);
float dw1000_nranges_twr_to_tof_frames(twr_frame_t *first_frame, twr_frame_t *final_frame);

#ifdef __cplusplus
}
#endif
#endif /* _DW1000_N_RANGES_H_ */
