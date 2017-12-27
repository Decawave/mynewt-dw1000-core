/**
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

#ifndef _DW1000_OTP_H_
#define _DW1000_OTP_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/hal_spi.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>

// OTP addresses definitions
#define OTP_LDOTUNE_ADDRESS (0x04)
#define OTP_PARTID_ADDRESS (0x06)
#define OTP_LOTID_ADDRESS  (0x07)
#define OTP_VBAT_ADDRESS   (0x08)
#define OTP_VTEMP_ADDRESS  (0x09)
#define OTP_XTRIM_ADDRESS  (0x1E)

uint32_t _dw1000_otp_read(dw1000_dev_instance_t * inst, uint16_t address);
void dw1000_opt_read(dw1000_dev_instance_t * inst, uint32_t address, uint32_t * buffer, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* _DW1000_OTP_H_ */
