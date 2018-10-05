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
 * @file dw1000_ftypes.h
 * @author paul kettle
 * @date 2018
 * @brief ftypes file
 *
 * @details This is the ftypes base class which include all the frames implemented in dw1000.
 */

#ifndef _DW1000_FTYPES_H_
#define _DW1000_FTYPES_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FCNTL_IEEE_BLINK_CCP_64 0xC5        //!< CCP blink frame control 
#define FCNTL_IEEE_BLINK_TAG_64 0x56        //!< Tag blink frame control 
#define FCNTL_IEEE_BLINK_ANC_64 0x57        //!< Anchor blink frame control
#define FCNTL_IEEE_RANGE_16     0x8841      //!< Range frame control 
#define FCNTL_IEEE_PROVISION_16 0x8844      //!< Provision frame control

//! IEEE 802.15.4e standard blink. It is a 12-byte frame composed of the following fields.
typedef union{
//! Structure of IEEE blink frame
    struct _ieee_blink_frame_t{
        uint8_t fctrl;              //!< Frame type (0xC5 for a blink) using 64-bit addressing
        uint8_t seq_num;            //!< Sequence number, incremented for each new frame
        uint64_t long_address;      //!< Device ID
    }__attribute__((__packed__,aligned(1)));
    uint8_t array[sizeof(struct _ieee_blink_frame_t)]; //!< Array of size blink frame
}ieee_blink_frame_t;

//! ISO/IEC 24730-62:2013 standard blink. It is a 14-byte frame composed of the following fields.
typedef union {
//! Structure of extended blink frame
    struct _ieee_blink_frame_ext_t{
        uint8_t fctrl;              //!< Frame type (0xC5 for a blink) using 64-bit addressing
        uint8_t seq_num;            //!< Sequence number, incremented for each new frame
        uint64_t address;           //!< Device ID
        uint8_t encoding;           //!< 0x43 to indicate no extended ID
        uint8_t EXT_header ;        //!< 0x02 to indicate tag is listening for a response immediately
    }__attribute__((__packed__,aligned(1)));
    uint8_t array[sizeof(struct _ieee_blink_frame_ext_t)];  //!< Array of size extended blink frame 
}ieee_blink_frame_ext_t;

//! IEEE 802.15.4 standard ranging frames
typedef union {
//! Structure of range request frame
    struct _ieee_rng_request_frame_t{
        uint16_t fctrl;             //!< Frame control (0x8841 to indicate a data frame using 16-bit addressing)
        uint8_t seq_num;            //!< Sequence number, incremented for each new frame
        uint16_t PANID;             //!< PANID (0xDECA)
        uint16_t dst_address;       //!< Destination address
        uint16_t src_address;       //!< Source address
        uint16_t code;              //!< Response code for the request 
    }__attribute__((__packed__,aligned(1)));    
    uint8_t array[sizeof(struct _ieee_rng_request_frame_t)];  //!< Array of size range request frame
} ieee_rng_request_frame_t;

//! Standard response frame
typedef union {
//! Structure of range response frame
    struct  _ieee_rng_response_frame_t{
        struct _ieee_rng_request_frame_t;
        uint32_t reception_timestamp;    //!< Request reception timestamp
        uint32_t transmission_timestamp; //!< Response transmission timestamp
    }__attribute__((__packed__,aligned(1)));
    uint8_t array[sizeof(struct _ieee_rng_response_frame_t)]; //!< Array of size range response frame
} ieee_rng_response_frame_t;

//! IEEE 802.15.4 standard data frame.
typedef union {
//! Structure of standard frame
    struct _ieee_std_frame_t{
        uint16_t fctrl;             //!< Frame control (0x8841 to indicate a data frame using 16-bit addressing)
        uint8_t seq_num;            //!< Sequence number, incremented for each new frame
        uint16_t PANID;             //!< PANID (0xDECA)
        uint16_t dst_address;       //!< Destination address
        uint16_t src_address;       //!< Source address
        uint16_t code;              //!< Response code for the request 
    }__attribute__((__packed__,aligned(1)));          
    uint8_t array[sizeof(struct _ieee_std_frame_t)];  //!< Array of size standard frame
} ieee_std_frame_t;


#ifdef __cplusplus
}
#endif
#endif /* _DW1000_FTYPES_H_ */
