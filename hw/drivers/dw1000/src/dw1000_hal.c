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
                .channel = 5,                       //!< channel number {1, 2, 3, 4, 5, 7 }
                .prf = DWT_PRF_64M,                 //!< Pulse Repetition Frequency {DWT_PRF_16M or DWT_PRF_64M}
                .dataRate = DWT_BR_6M8,             //!< Data Rate {DWT_BR_110K, DWT_BR_850K or DWT_BR_6M8}
                .rx = {
                    .pacLength = DWT_PAC8,          //!< Acquisition Chunk Size DWT_PAC8..DWT_PAC64 (Relates to RX preamble length)
                    .preambleCodeIndex = 9,         //!< RX preamble code
                    .sfdType = 0,                   //!< Boolean should we use non-standard SFD for better performance
                    .phrMode = DWT_PHRMODE_STD,     //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
                    .sfdTimeout = (256 + 1 + 8 - 8) //!< SFD timeout value (in symbols) (preamble length + 1 + SFD length - PAC size). Used in RX only. 
                },
                .tx ={
                    .preambleCodeIndex = 9,         //!< TX preamble code
                    .preambleLength = DWT_PLEN_256  //!< DWT_PLEN_64..DWT_PLEN_4096
                },
                .txrf={
                    .PGdly = TC_PGDELAY_CH5,
                    .BOOSTNORM = dw1000_power_value(DW1000_txrf_config_9db, 5),
                    .BOOSTP500 = dw1000_power_value(DW1000_txrf_config_9db, 5),
                    .BOOSTP250 = dw1000_power_value(DW1000_txrf_config_9db, 5),
                    .BOOSTP125 = dw1000_power_value(DW1000_txrf_config_9db, 5)
                }, 
                .rxdiag_enable = 1,
                .dblbuffon_enabled = 1,
#if MYNEWT_VAL(DW1000_MAC_FILTERING)
                .framefilter_enabled = 1,
#endif
#if MYNEWT_VAL(DW1000_BIAS_CORRECTION_ENABLED)
                .bias_correction_enable = 1,
#endif
                .rxauto_enable = 1
            },
            .spi_mutex = 0,
            .interrupt_task_prio = 5
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
            .config = {
                .channel = 5,                       //!< channel number {1, 2, 3, 4, 5, 7 }
                .prf = DWT_PRF_64M,                 //!< Pulse Repetition Frequency {DWT_PRF_16M or DWT_PRF_64M}
                .dataRate = DWT_BR_6M8,             // Data rate. 
                .rx = {
                    .pacLength = DWT_PAC8,          //!< Acquisition Chunk Size (Relates to RX preamble length)
                    .preambleCodeIndex = 9,         //!< RX preamble code
                    .sfdType = 0,                   //!< Boolean should we use non-standard SFD for better performance
                    .phrMode = DWT_PHRMODE_STD,     //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
                    .sfdTimeout = (256 + 1 + 8 - 8) //!< SFD timeout value (in symbols) (preamble length + 1 + SFD length - PAC size). Used in RX only. 
                },
                .tx ={
                    .preambleCodeIndex = 9,         //!< TX preamble code
                    .preambleLength = DWT_PLEN_256  //!< DWT_PLEN_64..DWT_PLEN_4096
                },
                .txrf={
                    .PGdly = TC_PGDELAY_CH5,
                    .BOOSTNORM = dw1000_power_value(DW1000_txrf_config_0db, 0),
                    .BOOSTP500 = dw1000_power_value(DW1000_txrf_config_0db, 0),
                    .BOOSTP250 = dw1000_power_value(DW1000_txrf_config_0db, 0),
                    .BOOSTP125 = dw1000_power_value(DW1000_txrf_config_0db, 0)
                }, 
                .rxdiag_enable = 1,
                .dblbuffon_enabled = 1,
#if MYNEWT_VAL(DW1000_MAC_FILTERING)
                .framefilter_enabled = 1,
#endif
                .rxauto_enable = 1,
            },
            .spi_mutex = 0,
            .interrupt_task_prio = 6
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
             .config = {
                .channel = 5,                       //!< channel number {1, 2, 3, 4, 5, 7 }
                .prf = DWT_PRF_64M,                 //!< Pulse Repetition Frequency {DWT_PRF_16M or DWT_PRF_64M}
                .dataRate = DWT_BR_6M8,             // Data rate. 
                .rx = {
                    .pacLength = DWT_PAC8,          //!< Acquisition Chunk Size (Relates to RX preamble length)
                    .preambleCodeIndex = 9,         //!< RX preamble code
                    .sfdType = 0,                   //!< Boolean should we use non-standard SFD for better performance
                    .phrMode = DWT_PHRMODE_STD,     //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
                    .sfdTimeout = (256 + 1 + 8 - 8) //!< SFD timeout value (in symbols) (preamble length + 1 + SFD length - PAC size). Used in RX only. 
                },
                .tx ={
                    .preambleCodeIndex = 9,         //!< TX preamble code
                    .preambleLength = DWT_PLEN_256  //!< DWT_PLEN_64..DWT_PLEN_4096
                },
                .txrf={
                    .PGdly = TC_PGDELAY_CH5,
                    .BOOSTNORM = dw1000_power_value(DW1000_txrf_config_0db, 0),
                    .BOOSTP500 = dw1000_power_value(DW1000_txrf_config_0db, 0),
                    .BOOSTP250 = dw1000_power_value(DW1000_txrf_config_0db, 0),
                    .BOOSTP125 = dw1000_power_value(DW1000_txrf_config_0db, 0)
                }, 
                .rxdiag_enable = 1,
                .dblbuffon_enabled = 1,
#if MYNEWT_VAL(DW1000_MAC_FILTERING)
                .framefilter_enabled = 1,
#endif
                .rxauto_enable = 1
            },
            .spi_mutex = 0
            .interrupt_task_prio = 7
    }
    #endif
    #endif
    #endif
};
#endif

struct _dw1000_dev_instance_t * 
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
hal_dw1000_reset(struct _dw1000_dev_instance_t * inst)
{
    assert(inst);

    hal_gpio_init_out(inst->ss_pin, 1);
    hal_gpio_init_out(inst->rst_pin, 0);

    hal_gpio_write(inst->rst_pin, 0);
    hal_gpio_write(inst->rst_pin, 1);
    hal_gpio_init_in(inst->rst_pin, HAL_GPIO_PULL_NONE);

    os_cputime_delay_usecs(5000);
}

void 
hal_dw1000_read(struct _dw1000_dev_instance_t * inst, const uint8_t * cmd, uint8_t cmd_size, uint8_t * buffer, uint16_t length)
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
hal_dw1000_write(struct _dw1000_dev_instance_t * inst, const uint8_t * cmd, uint8_t cmd_size, uint8_t * buffer, uint16_t length)
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
hal_dw1000_wakeup(struct _dw1000_dev_instance_t * inst)
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
hal_dw1000_get_rst(struct _dw1000_dev_instance_t * inst)
{
    return hal_gpio_read(inst->rst_pin);
}
