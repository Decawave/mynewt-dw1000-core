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

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <os/os.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>

#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_otp.h>


void dw1000_phy_otp_read(struct _dw1000_dev_instance_t * inst, uint32_t address, uint32_t * buffer, uint16_t length)
{
    dw1000_phy_sysclk_XTAL(inst); // NOTE: Set system clock to XTAL - this is necessary to make sure the values read by _dwt_otpread are reliable

    for(uint16_t i=0; i<length; i++)
        buffer[i] = _dw1000_otp_read(inst, address + i) ;

    dw1000_phy_sysclk_SEQ(inst);
}

uint32_t _dw1000_otp_read(struct _dw1000_dev_instance_t * inst, uint16_t address)
{
    // Write the address
    dw1000_write_reg(inst, OTP_IF_ID, OTP_ADDR, address, sizeof(uint16_t));

    // Perform OTP Read - Manual read mode has to be set
    dw1000_write_reg(inst, OTP_IF_ID, OTP_CTRL, OTP_CTRL_OTPREAD | OTP_CTRL_OTPRDEN, sizeof(uint8_t));
    dw1000_write_reg(inst, OTP_IF_ID, OTP_CTRL, 0x0, sizeof(uint8_t)); // OTPREAD is self clearing but OTPRDEN is not

    // Read data, available 40ns after rising edge of OTP_READ
    os_cputime_delay_usecs(1);
    return  (uint32_t) dw1000_read_reg(inst, OTP_IF_ID, OTP_RDAT, sizeof(uint32_t));
}

