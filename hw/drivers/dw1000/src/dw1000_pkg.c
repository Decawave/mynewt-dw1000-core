/*
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

/**
 * @file dw1000_pkg.c
 * @author paul kettle
 * @date 2018
 * @brief package file
 *
 * @details This is the pkg class which utilises the function to initialize the dw1000 instances depending on the availability. 
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

#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

/**
 * API to initialize the dw1000 instances.
 *
 * @param void
 * @return void
 */
void dw1000_pkg_init(void){

    DIAGMSG("{\"utime\": %lu,\"msg\": \"dw1000_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

#if MYNEWT_VAL(DW1000_DEVICE_0)
    dw1000_dev_config(hal_dw1000_inst(0));
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    dw1000_dev_config(hal_dw1000_inst(1));
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    dw1000_dev_config(hal_dw1000_inst(2));
#endif

}
