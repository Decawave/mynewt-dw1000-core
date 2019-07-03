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
 * @file dw1000_rng.h
 * @athor paul kettle
 * @date 2018
 * @brief Range 
 *
 * @details This is the rng base class which utilises the functions to enable/disable the configurations related to rng.
 *
 */

#ifndef _NMGR_UWB_H_
#define _NMGR_UWB_H_


#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/hal_spi.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_ftypes.h>

#define NMGR_UWB_MTU_STD (128 -  sizeof(struct _ieee_std_frame_t) - sizeof(uint16_t) - 2/*CRC*/)
#define NMGR_UWB_MTU_EXT (1023 - sizeof(struct _ieee_std_frame_t) - sizeof(uint16_t) - 2/*CRC*/)

//! IEEE 802.15.4 standard data frame.
typedef union {
//! Structure of standard frame
    struct _nmgr_uwb_header{
        uint16_t fctrl;             //!< Frame control (0x8841 to indicate a data frame using 16-bit addressing)
        uint8_t seq_num;            //!< Sequence number, incremented for each new frame
        uint16_t PANID;             //!< PANID (0xDECA)
        uint16_t dst_address;       //!< Destination address
        uint16_t src_address;       //!< Source address
        uint16_t code;              //!< Response code for the request
        uint8_t  rpt_count;         //!< Repeat level
        uint8_t  rpt_max;           //!< Repeat max level
    }__attribute__((__packed__,aligned(1)));
    uint8_t array[sizeof(struct _ieee_std_frame_t)];  //!< Array of size standard frame
} nmgr_uwb_frame_header_t;

typedef struct _nmgr_uwb_instance_t {
    struct _dw1000_dev_instance_t* parent;
    uint8_t frame_seq_num;
    struct os_sem sem;
    struct os_mqueue tx_q;
} nmgr_uwb_instance_t;

typedef enum _nmgr_uwb_codes_t{
    NMGR_CMD_STATE_SEND = 1,
    NMGR_CMD_STATE_RSP,
    NMGR_CMD_STATE_INVALID    
}nmgr_uwb_codes_t;

uint16_t nmgr_uwb_mtu(struct os_mbuf *m, int idx);
nmgr_uwb_instance_t* nmgr_uwb_init(dw1000_dev_instance_t* inst);
int nmgr_uwb_tx(dw1000_dev_instance_t* inst, uint16_t dst_addr, uint16_t code, struct os_mbuf *m, uint64_t dx_time);

/* Sychronous model */
dw1000_dev_status_t nmgr_uwb_listen(dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode, uint64_t delay, uint16_t timeout);
int uwb_nmgr_process_tx_queue(dw1000_dev_instance_t* inst, uint64_t dx_time);
int uwb_nmgr_queue_tx(dw1000_dev_instance_t* inst, uint16_t dst_addr, uint16_t code, struct os_mbuf *om);
#ifdef __cplusplus
}
#endif

#endif /* _NMGR_UWB_H_ */
