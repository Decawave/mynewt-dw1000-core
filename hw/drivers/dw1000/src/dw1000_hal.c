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

#include <stdint.h>
#include <stddef.h>
#include <assert.h>
//#include <os/os_cputime.h>
#include <os/os_dev.h>
#include <syscfg/syscfg.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>

#if MYNEWT_VAL(DW1000_DEVICE_0)
static dw1000_dev_instance_t hal_dw1000_instances[]= {
    #if  MYNEWT_VAL(DW1000_DEVICE_0)
    [0] = {
            .rst_pin  = MYNEWT_VAL(DW1000_DEVICE_0_RST),
            .ss_pin = MYNEWT_VAL(DW1000_DEVICE_0_SS),
            .irq_pin  = MYNEWT_VAL(DW1000_DEVICE_0_IRQ),
            .spi_settings = {
                .data_order = HAL_SPI_MSB_FIRST,
                .data_mode = HAL_SPI_MODE0,
                .baudrate = MYNEWT_VAL(DW1000_DEVICE_BAUDRATE_LOW),
                .word_size = HAL_SPI_WORD_SIZE_8BIT
            },
            .rx_antenna_delay = MYNEWT_VAL(DW1000_DEVICE_0_RX_ANT_DLY),
            .tx_antenna_delay = MYNEWT_VAL(DW1000_DEVICE_0_TX_ANT_DLY),
            .status = {0},
            .config = {
                .rxdiag_enable = 1,
                .dblbuffon_enabled = 1,
                .rxauto_enable = 1
            },
            .spi_mutex = 0
    },
    #if  MYNEWT_VAL(DW1000_DEVICE_1)
    [1] = {
            .rst_pin  = MYNEWT_VAL(DW1000_DEVICE_1_RST),
            .ss_pin = MYNEWT_VAL(DW1000_DEVICE_1_SS),
            .irq_pin  = MYNEWT_VAL(DW1000_DEVICE_1_IRQ),
            .spi_settings = {
                .data_order = HAL_SPI_MSB_FIRST,
                .data_mode = HAL_SPI_MODE0,
                .baudrate = MYNEWT_VAL(DW1000_DEVICE_BAUDRATE_LOW),
                .word_size = HAL_SPI_WORD_SIZE_8BIT
            },
            .rx_antenna_delay = MYNEWT_VAL(DW1000_DEVICE_1_RX_ANT_DLY),
            .tx_antenna_delay = MYNEWT_VAL(DW1000_DEVICE_1_TX_ANT_DLY),
            .status = {0},
            .spi_mutex = 0
    },
    #if  MYNEWT_VAL(DW1000_DEVICE_2)
    [2] = {
            .rst_pin  = MYNEWT_VAL(DW1000_DEVICE_2_RST),
            .ss_pin = MYNEWT_VAL(DW1000_DEVICE_2_SS),
            .irq_pin  = MYNEWT_VAL(DW1000_DEVICE_2_IRQ),
            .spi_settings = {
                .data_order = HAL_SPI_MSB_FIRST,
                .data_mode = HAL_SPI_MODE0,
                .baudrate = MYNEWT_VAL(DW1000_DEVICE_BAUDRATE_LOW),
                .word_size = HAL_SPI_WORD_SIZE_8BIT
            },
            .rx_antenna_delay = MYNEWT_VAL(DW1000_DEVICE_2_RX_ANT_DLY),
            .tx_antenna_delay = MYNEWT_VAL(DW1000_DEVICE_2_TX_ANT_DLY),
            .status = {0},
            .spi_mutex = 0
    }
    #endif
    #endif
    #endif
};
#endif

dw1000_dev_instance_t * 
hal_dw1000_inst(uint8_t idx){
    
#if  MYNEWT_VAL(DW1000_DEVICE_0) 
#if  MYNEWT_VAL(DW1000_DEVICE_1)
    assert(idx < 2);  // Only two instance for chosen bsp
#else
    assert(idx < 1);  // Only one instance for chosen bsp
#endif
    return &hal_dw1000_instances[idx];
#else
    assert(0);  // no instance for chosen bsp
#endif


}

void  
hal_dw1000_reset(dw1000_dev_instance_t * inst)
{
    assert(inst);

    hal_gpio_init_out(inst->ss_pin, 1);
    hal_gpio_init_out(inst->rst_pin, 0);

    hal_gpio_write(inst->rst_pin, 0);
    hal_gpio_write(inst->rst_pin, 1);
    hal_gpio_init_in(inst->rst_pin, HAL_GPIO_PULL_NONE);

    os_time_delay(2 * OS_TICKS_PER_SEC / 1000);
}

void 
hal_dw1000_read(dw1000_dev_instance_t * inst, const uint8_t * cmd, uint8_t cmd_size, uint8_t * buffer, uint16_t length)
{
    os_error_t err;
    if (inst->spi_mutex) {
        err = os_mutex_pend(inst->spi_mutex, OS_WAIT_FOREVER);
        assert(err == OS_OK);
    }
  
    hal_gpio_write(inst->ss_pin, 0);

    for(uint8_t i = 0; i < cmd_size; i++)
        hal_spi_tx_val(inst->spi_num, cmd[i]);
    for(uint16_t i = 0; i < length; i++)
        buffer[i] = hal_spi_tx_val(inst->spi_num, 0);
 
    hal_gpio_write(inst->ss_pin, 1);

    if (inst->spi_mutex) {
        err = os_mutex_release(inst->spi_mutex);
        assert(err == OS_OK);
    }
}

void 
hal_dw1000_write(dw1000_dev_instance_t * inst, const uint8_t * cmd, uint8_t cmd_size, uint8_t * buffer, uint16_t length)
{
    os_error_t err;
    if (inst->spi_mutex) {
        err = os_mutex_pend(inst->spi_mutex, OS_WAIT_FOREVER);
        assert(err == OS_OK);
    }

    hal_gpio_write(inst->ss_pin, 0);

    for(uint8_t i = 0; i < cmd_size; i++)
        hal_spi_tx_val(inst->spi_num, cmd[i]);
    for(uint16_t i = 0; i < length; i++)
        hal_spi_tx_val(inst->spi_num, buffer[i]);
     
    hal_gpio_write(inst->ss_pin, 1);

    if (inst->spi_mutex) {
        err = os_mutex_release(inst->spi_mutex);
        assert(err == OS_OK);
    }
}

void 
hal_dw1000_wakeup(dw1000_dev_instance_t * inst)
{
    os_error_t err;
    os_sr_t sr;
    OS_ENTER_CRITICAL(sr);
    if (inst->spi_mutex) {
        err = os_mutex_pend(inst->spi_mutex, OS_WAIT_FOREVER);
        assert(err == OS_OK);
    }
    
    hal_spi_disable(inst->spi_num);
    hal_gpio_write(inst->ss_pin, 0);

    // Need to hold chip select for a minimum of 600us
    os_cputime_delay_usecs(2000);

    hal_gpio_write(inst->ss_pin, 1);
    hal_spi_enable(inst->spi_num);

    if (inst->spi_mutex) {
        err = os_mutex_release(inst->spi_mutex);
        assert(err == OS_OK);
    }

    // Waiting for XTAL to start and stabilise - 5ms safe
    // (check PLL bit in IRQ?)
    os_cputime_delay_usecs(5000);

    OS_EXIT_CRITICAL(sr);
}

/* Read the current level of the rst pin.
 * When sleeping dw1000 will let this pin should go low */
int
hal_dw1000_get_rst(dw1000_dev_instance_t * inst)
{
    return hal_gpio_read(inst->rst_pin);
}
