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

#ifndef _DW1000_FTYPES_H_
#define _DW1000_FTYPES_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FCNTL_IEEE_BLINK_CCP_64 0xC5
#define FCNTL_IEEE_BLINK_TAG_64 0x56 
#define FCNTL_IEEE_BLINK_ANC_64 0x57
#define FCNTL_IEEE_RANGE_16     0x8841
#define FCNTL_IEEE_PROVISION_16 0x8844

// IEEE 802.15.4e standard blink. It is a 12-byte frame composed of the following fields
typedef union{
    struct _ieee_blink_frame_t{
        uint8_t fctrl;              // frame type (0xC5 for a blink) using 64-bit addressing
        uint8_t seq_num;            // sequence number, incremented for each new frame.
        uint64_t long_address;      // device ID
    }__attribute__((__packed__));
    uint8_t array[sizeof(struct _ieee_blink_frame_t)];
}ieee_blink_frame_t;

// ISO/IEC 24730-62:2013 standard blink. It is a 14-byte frame composed of the following fields
typedef union {
    struct _ieee_blink_frame_ext_t{
        uint8_t fctrl;              // frame type (0xC5 for a blink) using 64-bit addressing
        uint8_t seq_num;            // sequence number, incremented for each new frame.
        uint64_t address;           // device ID
        uint8_t encoding;           // 0x43 to indicate no extended ID
        uint8_t EXT_header ;        // 0x02 to indicate tag is listening for a response immediately
    }__attribute__((__packed__,aligned(1)));
    uint8_t array[sizeof(struct _ieee_blink_frame_ext_t)];
}ieee_blink_frame_ext_t;

// IEEE 802.15.4 standard ranging frames
typedef union {
    struct _ieee_rng_request_frame_t{
        uint16_t fctrl;             // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        uint8_t seq_num;            // sequence number, incremented for each new frame.
        uint16_t PANID;             // PANID (0xDECA)
        uint16_t dst_address;       // destination address
        uint16_t src_address;       // source address
        uint16_t code;    
    }__attribute__((__packed__,aligned(1)));    
    uint8_t array[sizeof(struct _ieee_rng_request_frame_t)];
} ieee_rng_request_frame_t;

typedef union {
    struct  _ieee_rng_response_frame_t{
        struct _ieee_rng_request_frame_t;
        uint32_t reception_timestamp;    // request reception timestamp.
        uint32_t transmission_timestamp; // response transmission timestamp.
    }__attribute__((__packed__,aligned(1)));
    uint8_t array[sizeof(struct _ieee_rng_response_frame_t)];
} ieee_rng_response_frame_t;

// IEEE 802.15.4 standard data frame
typedef union {
    struct _ieee_std_frame_t{
        uint16_t fctrl;             // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        uint8_t seq_num;            // sequence number, incremented for each new frame.
        uint16_t PANID;             // PANID (0xDECA)
        uint16_t dst_address;       // destination address
        uint16_t src_address;       // source address
        uint16_t code;    
    }__attribute__((__packed__,aligned(1)));          
    uint8_t array[sizeof(struct _ieee_std_frame_t)];
} ieee_std_frame_t;


#ifdef __cplusplus
}
#endif
#endif /* _DW1000_FTYPES_H_ */
