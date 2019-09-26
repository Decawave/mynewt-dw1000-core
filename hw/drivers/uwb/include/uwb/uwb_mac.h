/*
 * Copyright 2019, Decawave Limited, All Rights Reserved 
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

#ifndef _UWB_MAC_H_
#define _UWB_MAC_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_BROADCAST_ADDRESS  0xffff  //!< UWB Broadcast addresss
    
//! constants for specifying TX Preamble length in symbols.
//! These are defined to allow them be directly written into byte 2 of the TX_FCTRL register.
//! (i.e. a four bit value destined for bits 20..18 but shifted left by 2 for byte alignment)
#define DWT_PLEN_4096   0x0C    //! Standard preamble length 4096 symbols
#define DWT_PLEN_2048   0x28    //! Non-standard preamble length 2048 symbols
#define DWT_PLEN_1536   0x18    //! Non-standard preamble length 1536 symbols
#define DWT_PLEN_1024   0x08    //! Standard preamble length 1024 symbols
#define DWT_PLEN_512    0x34    //! Non-standard preamble length 512 symbols
#define DWT_PLEN_256    0x24    //! Non-standard preamble length 256 symbols
#define DWT_PLEN_128    0x14    //! Non-standard preamble length 128 symbols
#define DWT_PLEN_64     0x04    //! Standard preamble length 64 symbols
#define DWT_PLEN_32     0x00    //! When setting length 32 symbols this is 0x0,  which is programmed to byte 2 of the TX_FCTRL register

//! constants for specifying the (Nominal) mean Pulse Repetition Frequency.
//! These are defined for direct write (with a shift if necessary) to CHAN_CTRL and TX_FCTRL regs.
#define DWT_PRF_16M     1   //!< UWB PRF 16 MHz
#define DWT_PRF_64M     2   //!< UWB PRF 64 MHz
#define DWT_PRF_SCP     3   //!< SCP UWB PRF ~100 MHz

#define DWT_SFDTOC_DEF  0x1041  //!< Default SFD timeout value
#define DWT_PHRMODE_STD 0x0     //!< standard PHR mode
#define DWT_PHRMODE_EXT 0x3     //!< DW proprietary extended frames PHR mode

#define DWT_PHRRATE_STD 0x0     // standard PHR rate
#define DWT_PHRRATE_DTA 0x1     // PHR at data rate (6M81)

//! Multiplication factors to convert carrier integrator value to a frequency offset in Hz
#define DWT_FREQ_OFFSET_MULTIPLIER          (998.4e6/2.0/1024.0/131072.0)
#define DWT_FREQ_OFFSET_MULTIPLIER_110KB    (998.4e6/2.0/8192.0/131072.0)

//! Multiplication factors to convert frequency offset in Hertz to PPM crystal offset
// NB: also changes sign so a positive value means the local RX clock is
// running slower than the remote TX device.
#define DWT_HZ_TO_PPM_MULTIPLIER_CHAN_1     (-1.0e6/3494.4e6)
#define DWT_HZ_TO_PPM_MULTIPLIER_CHAN_2     (-1.0e6/3993.6e6)
#define DWT_HZ_TO_PPM_MULTIPLIER_CHAN_3     (-1.0e6/4492.8e6)
#define DWT_HZ_TO_PPM_MULTIPLIER_CHAN_4     (-1.0e6/4492.8e6)
#define DWT_HZ_TO_PPM_MULTIPLIER_CHAN_5     (-1.0e6/6489.6e6)
#define DWT_HZ_TO_PPM_MULTIPLIER_CHAN_7     (-1.0e6/6489.6e6)

#ifdef __cplusplus
}
#endif
#endif /* _UWB_MAC_H_ */
