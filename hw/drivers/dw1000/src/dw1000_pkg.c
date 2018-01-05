/**
 * Copyright (C) 2017-2018, Decawave Limited, All Rights Reserved
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
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_phy.h>


// This function is call from within sysinit() and sysinit_app()
void dw1000_pkg_init(void){

#if MYNEWT_VAL(DW1000_DEVICE_0)
    dw1000_dev_init(hal_dw1000_inst(0), MYNEWT_VAL(DW1000_DEVICE_0_SPI_IDX));
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    dw1000_dev_init(hal_dw1000_inst(1), MYNEWT_VAL(DW1000_DEVICE_1_SPI_IDX));
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    dw1000_dev_init(hal_dw1000_inst(2), MYNEWT_VAL(DW1000_DEVICE_2_SPI_IDX));
#endif

}