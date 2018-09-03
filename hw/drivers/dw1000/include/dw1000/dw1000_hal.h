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
 * @file dw1000_hal.h
 * @author paul kettle
 * @date 2018
 * @brief Hardware Abstraction Layer
 *
 * @details This is the hal base class which utilises functions to perform the necessary actions at hal. 
 *
 */

#ifndef _DW1000_HAL_H_
#define _DW1000_HAL_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_phy.h>

struct _dw1000_dev_instance_t * hal_dw1000_inst(uint8_t idx);     //!< Structure of hal instances.
void hal_dw1000_reset(struct _dw1000_dev_instance_t * inst);
void hal_dw1000_read(struct _dw1000_dev_instance_t * inst, const uint8_t * cmd, uint8_t cmd_size, uint8_t * buffer, uint16_t length);
void hal_dw1000_write(struct _dw1000_dev_instance_t * inst, const uint8_t * cmd, uint8_t cmd_size, uint8_t * buffer, uint16_t length);
void hal_dw1000_wakeup(struct _dw1000_dev_instance_t * inst);
int hal_dw1000_get_rst(struct _dw1000_dev_instance_t * inst);

#ifdef __cplusplus
}
#endif

#endif /* _DW1000_HAL_H_ */
