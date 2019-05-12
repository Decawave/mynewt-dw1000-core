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

#ifndef _RTDOA_H_
#define _RTDOA_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/hal_spi.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_ftypes.h>
#include <euclid/triad.h>
#include <stats/stats.h>

#if MYNEWT_VAL(RNG_ENABLED)
#include <rng/rng.h>
#include <rng/slots.h>
#endif

#if MYNEWT_VAL(RTDOA_STATS)
    STATS_SECT_START(rtdoa_stat_section)
    STATS_SECT_ENTRY(rtdoa_request)
    STATS_SECT_ENTRY(rtdoa_response)
    STATS_SECT_ENTRY(rtdoa_listen)
    STATS_SECT_ENTRY(rx_complete)
    STATS_SECT_ENTRY(rx_error)
    STATS_SECT_ENTRY(rx_timeout)
    STATS_SECT_ENTRY(rx_relayed)
    STATS_SECT_ENTRY(tx_relay_error)
    STATS_SECT_ENTRY(tx_relay_ok)
    STATS_SECT_ENTRY(start_rx_error)
    STATS_SECT_ENTRY(rx_unsolicited)
    STATS_SECT_ENTRY(start_tx_error)
    STATS_SECT_ENTRY(reset)
STATS_SECT_END

#define RTDOA_STATS_INC(__X) STATS_INC(inst->rtdoa->stat, __X)
#else
#define RTDOA_STATS_INC(__X) {}
#endif

typedef enum _dw1000_rtdoa_device_type_t{
    DWT_RTDOA_NONE=0,
    DWT_RTDOA_INITIATOR,
    DWT_RTDOA_RESPONDER,
    DWT_RTDOA_SUBSCRIBER
} dw1000_rtdoa_device_type_t;
    
//! N-Ranges request frame
typedef union {
    struct _rtdoa_request_frame_t{
        struct _ieee_rng_request_frame_t;
        uint64_t tx_timestamp;
        uint8_t slot_modulus;       //!< How many slots to send in
        uint8_t rpt_count;          //!< Repeat level
        uint8_t rpt_max;            //!< Repeat max level
    }__attribute__((__packed__,aligned(1)));
    uint8_t array[sizeof(struct _rtdoa_request_frame_t)];
} rtdoa_request_frame_t;

//! RTDoA response frame
typedef union {
    struct _rtdoa_response_frame_t{
       struct _ieee_rng_request_frame_t;
        uint64_t tx_timestamp;
        uint8_t slot_id;          //!< slot_idx of transmitting anchor
        // Anchor Location??
    }__attribute__((__packed__,aligned(1)));
    uint8_t array[sizeof(struct _rtdoa_response_frame_t)]; //!< Array of size nrng response frame
} rtdoa_response_frame_t;

//! RTDoA ext response frame format
typedef union {
    struct _rtdoa_frame_t{
        struct _rtdoa_request_frame_t;
        uint64_t rx_timestamp;
        struct _dw1000_dev_rxdiag_t diag;
    } __attribute__((__packed__, aligned(1)));
    uint8_t array[sizeof(struct _rtdoa_frame_t)];
} rtdoa_frame_t;


typedef struct _dw1000_rtdoa_instance_t{
    struct _dw1000_dev_instance_t * parent;
#if MYNEWT_VAL(NRNG_STATS)
    STATS_SECT_DECL(rtdoa_stat_section) stat; //!< Stats instance
#endif
    uint16_t resp_count;
    uint64_t delay;
    uint8_t seq_num;
    uint16_t nframes;
    struct os_sem sem;                          //!< Structure of semaphores
    dw1000_mac_interface_t cbs;                 //!< MAC Layer Callbacks
    dw1000_rng_status_t status;
    dw1000_rng_control_t control;
    dw1000_rng_config_t config;
    dw1000_rtdoa_device_type_t device_type;
    uint16_t idx;
    rtdoa_frame_t * req_frame;
    rtdoa_frame_t * frames[];
} dw1000_rtdoa_instance_t;

dw1000_rtdoa_instance_t * dw1000_rtdoa_init(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config, uint16_t nframes);
dw1000_dev_status_t dw1000_rtdoa_request(dw1000_dev_instance_t * inst, uint64_t delay);
float dw1000_rtdoa_tdoa_between_frames(struct _dw1000_dev_instance_t * inst, rtdoa_frame_t *first_frame, rtdoa_frame_t *final_frame);
void dw1000_rtdoa_set_frames(dw1000_dev_instance_t * inst, uint16_t nframes);
dw1000_dev_status_t dw1000_rtdoa_config(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config);

dw1000_dev_status_t dw1000_rtdoa_listen(dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode, uint64_t delay);
uint32_t rtdoa_usecs_to_response(dw1000_dev_instance_t * inst, rtdoa_request_frame_t * req,
                                 uint16_t nslots, dw1000_rng_config_t * config, uint32_t duration);

#ifdef __cplusplus
}
#endif
#endif /* _RTDOA_H_ */
