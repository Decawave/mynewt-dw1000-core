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

#ifndef _DW1000_NRNG_H_
#define _DW1000_NRNG_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/hal_spi.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_ftypes.h>
#include <dw1000/triad.h>
#include <stats/stats.h>

#if MYNEWT_VAL(RNG_ENABLED)
#include <rng/rng.h>
#include <rng/slots.h>
#endif

STATS_SECT_START(nrng_stat_section)
    STATS_SECT_ENTRY(nrng_request)
    STATS_SECT_ENTRY(rx_error)
    STATS_SECT_ENTRY(rx_timeout)
    STATS_SECT_ENTRY(tx_error)
    STATS_SECT_ENTRY(start_tx_error_cb)
    STATS_SECT_ENTRY(rx_unsolicited)
    STATS_SECT_ENTRY(reset)
STATS_SECT_END

#define FRAMES_PER_RANGE       2
#define FIRST_FRAME_IDX        0
#define SECOND_FRAME_IDX       1

typedef enum _dw1000_nrng_device_type_t{
    DWT_NRNG_INITIATOR,
    DWT_NRNG_RESPONDER
}dw1000_nrng_device_type_t;

//! N-Ranges request frame
typedef union {
    struct _nrng_request_frame_t{
        struct _ieee_rng_request_frame_t;
        struct _slot_payload_t; //!< slot bitfields for request 
    }__attribute__((__packed__,aligned(1)));
    uint8_t array[sizeof(struct _nrng_request_frame_t)]; //!< Array of size nrng request frame
} nrng_request_frame_t;

//! N-Ranges response frame
typedef union {
    struct _nrng_response_frame_t{
       struct _ieee_rng_response_frame_t;
       uint8_t slot_id;  //!< slot_idx of transmitting anchor
    }__attribute__((__packed__,aligned(1)));
    uint8_t array[sizeof(struct _nrng_response_frame_t)]; //!< Array of size nrng response frame
} nrng_response_frame_t;

//! N-Ranges response frame
typedef union {
    struct  _nrng_final_frame_t{
        struct _nrng_response_frame_t;
        uint32_t request_timestamp;
        uint32_t response_timestamp;
        int32_t carrier_integrator;
    }__attribute__((__packed__,aligned(1)));
    uint8_t array[sizeof(struct _nrng_final_frame_t)]; //!< Array of size range final frame
} nrng_final_frame_t;

//! N-Ranges ext response frame format
typedef union {
    struct _nrng_frame_t{
        struct _nrng_final_frame_t;
#if MYNEWT_VAL(TWR_DS_EXT_NRNG_ENABLED)
        union {
            struct _twr_data_t;                            //!< Structure of twr_data
            uint8_t payload[sizeof(struct _twr_data_t)];   //!< Payload of size twr_data 
        };
#endif
    } __attribute__((__packed__, aligned(1)));
    uint8_t array[sizeof(struct _nrng_frame_t)];        //!< Array of size twr_frame
} nrng_frame_t;


typedef struct _dw1000_nrng_instance_t{
    struct _dw1000_dev_instance_t * parent;
    STATS_SECT_DECL(nrng_stat_section) stat; //!< Stats instance
    uint16_t nframes;
    uint16_t nnodes;
    uint16_t slot_mask;
    uint16_t valid_mask;
    uint16_t cell_id;
    uint16_t resp_count;
    uint16_t t1_final_flag;
    uint64_t delay;
    uint8_t seq_num;
    dw1000_nrng_device_type_t device_type;
    dw1000_rng_status_t status;
    dw1000_rng_control_t control;
    dw1000_rng_config_t config;
    uint16_t idx;
    nrng_frame_t *frames[][FRAMES_PER_RANGE];
}dw1000_nrng_instance_t;

dw1000_nrng_instance_t * dw1000_nrng_init(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config, dw1000_nrng_device_type_t type, uint16_t nframes, uint16_t nnodes);
dw1000_dev_status_t dw1000_nrng_request_delay_start(dw1000_dev_instance_t * inst, uint16_t dst_address, uint64_t delay, dw1000_rng_modes_t code, uint16_t start_slot_id, uint16_t end_slot_id);
dw1000_dev_status_t dw1000_nrng_request(dw1000_dev_instance_t * inst, uint16_t dst_address, dw1000_rng_modes_t code, uint16_t start_slot_id, uint16_t end_slot_id);
float dw1000_nrng_twr_to_tof_frames(struct _dw1000_dev_instance_t * inst, nrng_frame_t *first_frame, nrng_frame_t *final_frame);
void dw1000_nrng_set_frames(dw1000_dev_instance_t* inst, uint16_t nframes);
dw1000_dev_status_t dw1000_nrng_config(struct _dw1000_dev_instance_t* inst, dw1000_rng_config_t * config);
dw1000_rng_config_t * dw1000_nrng_get_config(dw1000_dev_instance_t * inst, dw1000_rng_modes_t code);
dw1000_dev_status_t dw1000_nrng_listen(dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode);
#ifdef __cplusplus
}
#endif
#endif /* _DW1000_NRNG_H_ */
