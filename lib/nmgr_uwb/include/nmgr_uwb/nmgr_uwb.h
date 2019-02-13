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

#define NMGR_UWB_MTU (128 - sizeof(struct _ieee_std_frame_t) - sizeof(uint16_t) - 2/*CRC*/)

typedef union{
    struct _nmgr_uwb_frame_t{
        struct _ieee_std_frame_t;
        union _payload{
            struct nmgr_hdr hdr;
            uint8_t payload[NMGR_UWB_MTU];
        };
    }__attribute__((__packed__,aligned(1)));
    uint8_t array[sizeof(struct _nmgr_uwb_frame_t)];
}nmgr_uwb_frame_t;

typedef struct _dw1000_nmgr_uwb_instance_t{
    nmgr_uwb_frame_t* frame;
    struct _dw1000_dev_instance_t* parent;
}dw1000_nmgr_uwb_instance_t;

typedef enum _nmgr_uwb_codes_t{
    NMGR_CMD_STATE_SEND = 1,
    NMGR_CMD_STATE_RSP,
    NMGR_CMD_STATE_INVALID    
}nmgr_uwb_codes_t;

#ifdef __cplusplus
}
#endif

dw1000_nmgr_uwb_instance_t* dw1000_nmgr_uwb_init(dw1000_dev_instance_t* inst);

#endif /* _DW1000_RNG_H_ */
