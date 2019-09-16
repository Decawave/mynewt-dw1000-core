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
 * @file uwb_rng.h
 * @athor paul kettle
 * @date 2018
 * @brief Range
 *
 * @details This is the rng base class which utilises the functions to enable/disable the configurations related to rng.
 *
 */

#ifndef __UWB_RNG_H_
#define __UWB_RNG_H_

#include <stdlib.h>
#include <stdint.h>
#include "euclid/triad.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <dpl/dpl.h>
#include <uwb/uwb.h>
#include <uwb/uwb_ftypes.h>
#include <dw1000/dw1000_dev.h>
#include <euclid/triad.h>
#include <stats/stats.h>
#include <uwb_rng/slots.h>

#if MYNEWT_VAL(RNG_STATS)
STATS_SECT_START(rng_stat_section)
    STATS_SECT_ENTRY(rng_request)
    STATS_SECT_ENTRY(rng_listen)
    STATS_SECT_ENTRY(tx_complete)
    STATS_SECT_ENTRY(rx_complete)
    STATS_SECT_ENTRY(rx_unsolicited)
    STATS_SECT_ENTRY(rx_error)
    STATS_SECT_ENTRY(tx_error)
    STATS_SECT_ENTRY(rx_timeout)
    STATS_SECT_ENTRY(reset)
STATS_SECT_END
#endif

//! Range configuration parameters.
struct uwb_rng_config{
   uint32_t rx_holdoff_delay;        //!< Delay between frames, in UWB usec.
   uint32_t tx_holdoff_delay;        //!< Delay between frames, in UWB usec.
   uint32_t tx_guard_delay;          //!< Delay between frames from subsequent nodes, in UWB sec.
   uint16_t rx_timeout_delay;        //!< Receive response timeout, in UWB usec.
   uint16_t bias_correction:1;       //!< Enable range bias correction polynomial
};

//! Range control parameters.
typedef struct _uwb_rng_control_t{
    uint16_t delay_start_enabled:1;  //!< Set for enabling delayed start
}uwb_rng_control_t;

//! Ranging modes.
typedef enum _uwb_rng_modes_t{
    DWT_TWR_INVALID = 0,             //!< Invalid TWR
    DWT_SS_TWR = 0x10,               //!< Single sided TWR 
    DWT_SS_TWR_T1,                   //!< Response for single sided TWR 
    DWT_SS_TWR_FINAL,                //!< Final response of single sided TWR 
    DWT_SS_TWR_END,                  //!< End of single sided TWR 
    DWT_SS_TWR_EXT,                  //!< Single sided TWR in extended mode
    DWT_SS_TWR_EXT_T1,               //!< Response for single sided TWR in extended mode
    DWT_SS_TWR_EXT_FINAL,            //!< Final response of single sided TWR in extended mode
    DWT_SS_TWR_EXT_END,              //!< End of single sided TWR in extended mode
    DWT_DS_TWR = 0x20,                      //!< Double sided TWR 
    DWT_DS_TWR_T1,                   //!< Response for double sided TWR 
    DWT_DS_TWR_T2,                   //!< Response for double sided TWR 
    DWT_DS_TWR_FINAL,                //!< Final response of double sided TWR 
    DWT_DS_TWR_END,                  //!< End of double sided TWR 
    DWT_DS_TWR_EXT,                  //!< Double sided TWR in extended mode 
    DWT_DS_TWR_EXT_T1,               //!< Response for double sided TWR in extended mode 
    DWT_DS_TWR_EXT_T2,               //!< Response for double sided TWR in extended mode 
    DWT_DS_TWR_EXT_FINAL,            //!< Final response of double sided TWR in extended mode 
    DWT_DS_TWR_EXT_END,              //!< End of double sided TWR in extended mode 
    DWT_PROVISION_START = 0x30,      //!< Start of provision
    DWT_PROVISION_RESP,              //!< End of provision
    DWT_SS_TWR_NRNG = 0x40,
    DWT_SS_TWR_NRNG_T1,
    DWT_SS_TWR_NRNG_FINAL,
    DWT_SS_TWR_NRNG_END,
    DWT_SS_TWR_NRNG_EXT,
    DWT_SS_TWR_NRNG_EXT_T1,
    DWT_SS_TWR_NRNG_EXT_FINAL,
    DWT_SS_TWR_NRNG_EXT_END,
    DWT_DS_TWR_NRNG = 0x50,
    DWT_DS_TWR_NRNG_T1,
    DWT_DS_TWR_NRNG_T2,
    DWT_DS_TWR_NRNG_FINAL,
    DWT_DS_TWR_NRNG_END,
    DWT_DS_TWR_NRNG_EXT,
    DWT_DS_TWR_NRNG_EXT_T1,
    DWT_DS_TWR_NRNG_EXT_T2,
    DWT_DS_TWR_NRNG_EXT_FINAL,
    DWT_DS_TWR_NRNG_EXT_END,
    DWT_DS_TWR_NRNG_INVALID,
    DWT_SURVEY_REQUEST = 0x60,
    DWT_SURVEY_BROADCAST,
    DWT_RTDOA_INVALID = 0x80,
    DWT_RTDOA_REQUEST,
    DWT_RTDOA_RESP,
}uwb_rng_modes_t;

//! Range status parameters
typedef struct _uwb_rng_status_t{
    uint16_t selfmalloc:1;           //!< Internal flag for memory garbage collection
    uint16_t initialized:1;          //!< Instance allocated
    uint16_t mac_error:1;            //!< Error caused due to frame filtering
    uint16_t invalid_code_error:1;   //!< Error due to invalid code
}uwb_rng_status_t;

//!  TWR final frame format
typedef struct _twr_frame_final_t{
        struct _ieee_rng_response_frame_t;
        uint32_t request_timestamp;     //!< Request transmission timestamp
        uint32_t response_timestamp;    //!< Response reception timestamp
        int32_t carrier_integrator;     //!< carrier integrator value
} __attribute__((__packed__, aligned(1))) twr_frame_final_t;

//! TWR data format
typedef struct _twr_data_t{
                uint64_t utime;                     //!< CPU time to usecs
                triad_t spherical;                  //!< Measurement triad spherical coordinates
                triad_t spherical_variance;         //!< Measurement variance triad
                triad_t cartesian;                  //!< Position triad local coordinates
          //      triad_t cartesian_variance;       //!< Position estimated variance triad
}twr_data_t;

//! TWR frame format
typedef union {
    //! Structure of TWR frame
    struct _twr_frame_t{
        //! Structure of TWR final frame
        struct _twr_frame_final_t;
        union {
            //! Structure of TWR data
            struct _twr_data_t;                            //!< Structure of twr_data
            uint8_t payload[sizeof(struct _twr_data_t)];   //!< Payload of size twr_data
        };
    } __attribute__((__packed__, aligned(1)));
    uint8_t array[sizeof(struct _twr_frame_t)];        //!< Array of size twr_frame
} twr_frame_t;

struct rng_config_list {
    uint16_t rng_code;
    struct uwb_rng_config *config;
    SLIST_ENTRY(rng_config_list) next;
};
    
//! Structure of range instance
struct uwb_rng_instance{
    struct uwb_dev * dev_inst;              //!< Structure of uwb_dev
#if MYNEWT_VAL(UWB_WCS_ENABLED)
    struct uwb_ccp_instance * ccp_inst;     //!< Structure of CCP
#endif
#if MYNEWT_VAL(RNG_STATS)
    STATS_SECT_DECL(rng_stat_section) stat; //!< Stats instance
#endif
    uint16_t code;                          //!< Range profile code
    uint16_t seq_num;                       //!< Local sequence number
    struct dpl_sem sem;                     //!< Structure of semaphores
    uint64_t delay;                         //!< Delay in transmission
    struct uwb_rng_config config;           //!< Structure of range config
    uwb_rng_control_t control;              //!< Structure of range control
    uwb_rng_status_t status;                //!< Structure of range status
    uint16_t idx;                           //!< Input index to circular buffer 
    uint16_t idx_current;                   //!< Output index to circular buffer 
    uint16_t nframes;                       //!< Number of buffers defined to store the ranging data
    SLIST_HEAD(, rng_config_list) rng_configs;
    twr_frame_t * frames[];                 //!< Pointer to twr buffers
};

    
void rng_pkg_init(void);
struct uwb_rng_instance * uwb_rng_init(struct uwb_dev * dev, struct uwb_rng_config * config, uint16_t nframes);
void uwb_rng_free(struct uwb_rng_instance * rng);
struct uwb_dev_status uwb_rng_config(struct uwb_rng_instance * rng, struct uwb_rng_config * config);
struct uwb_dev_status uwb_rng_request(struct uwb_rng_instance * rng, uint16_t dst_address, uwb_rng_modes_t protocal);
struct uwb_dev_status uwb_rng_listen(struct uwb_rng_instance * rng, uwb_dev_modes_t mode);
struct uwb_dev_status uwb_rng_request_delay_start(struct uwb_rng_instance * rng, uint16_t dst_address, uint64_t delay, uwb_rng_modes_t protocal);
struct uwb_rng_config * uwb_rng_get_config(struct uwb_rng_instance * rng, uwb_rng_modes_t code);
void uwb_rng_set_frames(struct uwb_rng_instance * rng, twr_frame_t twr[], uint16_t nframes);
float uwb_rng_twr_to_tof(struct uwb_rng_instance * rng, uint16_t idx);
float uwb_rng_tof_to_meters(float ToF);
float uwb_rng_is_los(float rssi, float fppl);

float uwb_rng_path_loss(float Pt, float G, float fc, float R);
float uwb_rng_bias_correction(struct uwb_dev * dev, float Pr);
uint32_t uwb_rng_twr_to_tof_sym(twr_frame_t twr[], uwb_rng_modes_t code);

void uwb_rng_append_config(struct uwb_rng_instance * rng, struct rng_config_list *cfgs);
void uwb_rng_remove_config(struct uwb_rng_instance * rng, uwb_rng_modes_t code);

    
#ifdef __cplusplus
}
#endif

#endif /* _DW1000_RNG_H_ */
