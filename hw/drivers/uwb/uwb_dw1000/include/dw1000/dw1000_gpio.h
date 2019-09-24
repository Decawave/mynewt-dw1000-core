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
 * @file dw1000_gpio.h
 * @author paul kettle
 * @date 2018
 * @brief General Purpose Input Output
 *
 * @details This is the gpio base class which utilises functions to enable/disable all the configurations related to GPIO.
 */

#ifndef _DW1000_GPIO_H_
#define _DW1000_GPIO_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/hal_spi.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>

//! Defined constants for "mode" bit field parameter passed to dwt_set_leds() function.
typedef enum _dw1000_led_modes_t{
    DWT_LEDS_DISABLE = 1 << 0,   //!< Set for disabling LEDS
    DWT_LEDS_ENABLE = 1 << 1,    //!< Set for enabling LEDS
    DWT_LEDS_INIT_BLINK = 1 << 2 //!< Set for initiation blink
}dw1000_led_modes_t;

void dw1000_gpio4_config_ext_pa(struct _dw1000_dev_instance_t * inst);
void dw1000_gpio5_config_ext_txe(struct _dw1000_dev_instance_t * inst);
void dw1000_gpio6_config_ext_rxe(struct _dw1000_dev_instance_t * inst);

void dw1000_gpio_config_leds(struct _dw1000_dev_instance_t * inst, dw1000_led_modes_t mode);
void dw1000_gpio_set_value(struct _dw1000_dev_instance_t * inst, uint8_t gpioNum, uint8_t value);
void dw1000_gpio_set_direction(struct _dw1000_dev_instance_t * inst, uint8_t gpioNum, uint8_t direction);
uint32_t dw1000_gpio_get_values(struct _dw1000_dev_instance_t * inst);

void dw1000_gpio_init_out(struct _dw1000_dev_instance_t * inst, int gpioNum, int val);
void dw1000_gpio_init_in(struct _dw1000_dev_instance_t * inst, int gpioNum);
int dw1000_gpio_read(struct _dw1000_dev_instance_t * inst, uint8_t gpioNum);
void dw1000_gpio_write(struct _dw1000_dev_instance_t * inst, int gpioNum, int val);

#ifdef __cplusplus
}
#endif

#endif /* _DW1000_GPIO_H_ */
