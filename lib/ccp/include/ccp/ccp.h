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
 * @file ccp.h
 * @author paul kettle
 * @date 2018
 * 
 * @brief clock calibration packets
 * @details This is the ccp base class which utilises the functions to enable/disable the configurations related to ccp.
 *
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
#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED)
#include <dsp/sosfilt.h>
#include <dsp/polyval.h>
#endif

#if MYNEWT_VAL(CCP_STATS)
STATS_SECT_START(ccp_stat_section)
    STATS_SECT_ENTRY(master_cnt)
    STATS_SECT_ENTRY(slave_cnt)
    STATS_SECT_ENTRY(wcs_resets)
    STATS_SECT_ENTRY(send)
    STATS_SECT_ENTRY(listen)
    STATS_SECT_ENTRY(tx_complete)
    STATS_SECT_ENTRY(rx_complete)
    STATS_SECT_ENTRY(rx_relayed)
    STATS_SECT_ENTRY(rx_unsolicited)
    STATS_SECT_ENTRY(txrx_error)
    STATS_SECT_ENTRY(tx_start_error)
    STATS_SECT_ENTRY(tx_relay_error)
    STATS_SECT_ENTRY(tx_relay_ok)
    STATS_SECT_ENTRY(rx_timeout)
    STATS_SECT_ENTRY(reset)
STATS_SECT_END
#endif

// XXX This needs to be made bitfield-safe. Not sure the ifdefs below are enough
typedef union _ccp_timestamp_t{
    struct {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        uint64_t lo:40;
        uint64_t hi:23;
        uint64_t halfperiod:1;
#endif
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
        uint64_t halfperiod:1;
        uint64_t hi:23;
        uint64_t lo:40;
#endif
    };
    uint64_t timestamp;
}ccp_timestamp_t;

//! Timestamps and blink frame format  of ccp frame.
typedef union {
    //! Frame format of ccp blink frame.
    struct _ccp_blink_frame_t{
        struct _ieee_blink_frame_t;
        uint16_t short_address;                 //!< Short Address
        union {
            struct _transmission_interval_struct{
                uint64_t transmission_interval:40; //!< Transmission interval. Also used to correct transmission timestamp in relays
            };
            uint8_t ti_array[sizeof(struct _transmission_interval_struct)];
        }__attribute__((__packed__, aligned(1)));
        ccp_timestamp_t transmission_timestamp; //!< Transmission timestamp
        uint8_t rpt_count;                      //!< Repeat level
        uint8_t rpt_max;                        //!< Repeat max level
    }__attribute__((__packed__, aligned(1)));
    uint8_t array[sizeof(struct _ccp_blink_frame_t)];
}ccp_blink_frame_t;

//! Timestamps and blink frame format  of ccp frame.
typedef union {
//! Frame format of ccp frame.
    struct _ccp_frame_t{
        struct _ccp_blink_frame_t;          
        uint64_t reception_timestamp;       //!< Reception timestamp
        int32_t carrier_integrator;         //!< Receiver carrier_integrator
        int32_t rxttcko;                    //!< Receiver time tracking offset
    }__attribute__((__packed__, aligned(1)));
    uint8_t array[sizeof(struct _ccp_frame_t)];
}ccp_frame_t;

//! Status parameters of ccp.
typedef struct _dw1000_ccp_status_t{
    uint16_t selfmalloc:1;            //!< Internal flag for memory garbage collection 
    uint16_t initialized:1;           //!< Instance allocated 
    uint16_t valid:1;                 //!< Set for valid parameters 
    uint16_t start_tx_error:1;        //!< Set for start transmit error 
    uint16_t start_rx_error:1;        //!< Set for start request error
    uint16_t rx_timeout_error:1;      //!< Receive timeout error 
    uint16_t timer_enabled:1;         //!< Indicates timer is enabled 
}dw1000_ccp_status_t;

//! Extension ids for services.
typedef enum _dw1000_ccp_role_t{
    CCP_ROLE_MASTER,                        //!< Clock calibration packet master mode
    CCP_ROLE_SLAVE,                         //!< Clock calibration packet slave mode
    CCP_ROLE_RELAY                          //!< Clock calibration packet master replay mode
}dw1000_ccp_role_t;

//! Callback for fetching clock source tof compensation
typedef uint32_t (*dw1000_ccp_tof_compensation_cb_t)(uint16_t short_addr);

//! ccp config parameters.  
typedef struct _dw1000_ccp_config_t{
    uint16_t postprocess:1;           //!< CCP postprocess
    uint16_t fs_xtalt_autotune:1;     //!< Autotune XTALT to Clock Master
    uint16_t role:4;                  //!< dw1000_ccp_role_t
    uint16_t tx_holdoff_dly;          //!< Relay nodes holdoff
}dw1000_ccp_config_t;

//! ccp instance parameters.
typedef struct _dw1000_ccp_instance_t{
    struct _dw1000_dev_instance_t * parent;     //!< Pointer to _dw1000_dev_instance_t
#if MYNEWT_VAL(CCP_STATS)
    STATS_SECT_DECL(ccp_stat_section) stat;     //!< Stats instance
#endif
#if MYNEWT_VAL(WCS_ENABLED)
    struct _wcs_instance_t * wcs;               //!< Wireless clock calibration 
#endif

#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED)
    struct _sos_instance_t * xtalt_sos;         //!< Sturcture of xtalt_sos
#endif
    dw1000_mac_interface_t cbs;                     //!< MAC Layer Callbacks
    uint64_t master_euid;                           //!< Clock Master EUID, used to reset wcs if master changes
    struct os_sem sem;                              //!< Structure containing os semaphores
    struct os_callout callout_postprocess;          //!< Structure of callout_postprocess
    dw1000_ccp_status_t status;                     //!< DW1000 ccp status parameters
    dw1000_ccp_config_t config;                     //!< DW1000 ccp config parameters
    ccp_timestamp_t master_epoch;                   //!< ccp event referenced to master systime
    uint64_t local_epoch;                           //!< ccp event referenced to local systime
    uint32_t os_epoch;                              //!< ccp event referenced to ostime
    dw1000_ccp_tof_compensation_cb_t tof_comp_cb;   //!< tof compensation callback
    uint32_t period;                                //!< Pulse repetition period
    uint16_t nframes;                               //!< Number of buffers defined to store the data 
    uint16_t idx;                                   //!< Circular buffer index pointer  
    uint8_t seq_num;                                //!< Clock Master reported sequence number
    struct hal_timer timer;                         //!< Timer structure
    struct os_eventq eventq;                        //!< Event queues
    struct os_callout event_cb;                     //!< Event callback
    struct os_task task_str;                        //!< os_task structure  
    uint8_t task_prio;                              //!< Priority based task
    os_stack_t task_stack[DW1000_DEV_TASK_STACK_SZ]
        __attribute__((aligned(OS_STACK_ALIGNMENT))); //!< Task stack size
    ccp_frame_t * frames[];                          //!< Buffers to ccp frames
}dw1000_ccp_instance_t; 

uint64_t ccp_local_to_master(dw1000_dev_instance_t *inst, uint32_t timestamp_local);
dw1000_ccp_instance_t * dw1000_ccp_init(dw1000_dev_instance_t * inst,  uint16_t nframes);
void dw1000_ccp_free(dw1000_ccp_instance_t * inst);
void dw1000_ccp_set_postprocess(dw1000_ccp_instance_t * inst, os_event_fn * ccp_postprocess); 
void dw1000_ccp_set_tof_comp_cb(dw1000_ccp_instance_t * inst, dw1000_ccp_tof_compensation_cb_t tof_comp_cb);
void dw1000_ccp_start(dw1000_dev_instance_t * inst, dw1000_ccp_role_t role);
void dw1000_ccp_stop(dw1000_dev_instance_t * inst);

#ifdef __cplusplus
}
#endif
#endif /* _DW1000_CCP_H_ */
