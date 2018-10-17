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
 * @file dw1000_hal.c
 * @author paul kettle
 * @date 2018
 * @brief Hardware Abstraction Layer
 *
 * @details This is the hal base class which utilises functions to perform the necessary actions at hal. 
 *
 */

#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <string.h>
#include <os/os_cputime.h>
#include <os/os_dev.h>
#include <syscfg/syscfg.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include <dw1000/dw1000_hal.h>

#if MYNEWT_VAL(DW1000_DEVICE_0)
/* Needed for DMA transfer operations */
static const uint8_t tx_buffer[MYNEWT_VAL(DW1000_HAL_SPI_BUFFER_SIZE)] __attribute__ ((aligned (8))) = {0};

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
            .attrib = {                //!< phy attritubes per the IEEE802.15.4-2011 standard, Table 99 and Table 101 
                .Tpsym = 1.01760,      //!< Preamble symbols duration (usec) for MPRF of 62.89Mhz
                .Tbsym = 1.02564,      //!< Baserate symbols duration (usec) 850khz
                .Tdsym = 0.12821/0.87, //!< Datarate symbols duration (usec) 6.81Mhz adjusted for RS coding
                .nsfd = 8,             //!< Number of symbols in start of frame delimiter
                .nsync = 128,          //!< Number of symbols in preamble sequence
                .nphr = 16             //!< Number of symbols in phy header
            },
            .config = {
                .channel = 5,                       //!< channel number {1, 2, 3, 4, 5, 7 }
                .prf = DWT_PRF_64M,                 //!< Pulse Repetition Frequency {DWT_PRF_16M or DWT_PRF_64M}
                .dataRate = DWT_BR_6M8,             //!< Data Rate {DWT_BR_110K, DWT_BR_850K or DWT_BR_6M8}
                .rx = {
                    .pacLength = DWT_PAC8,          //!< Acquisition Chunk Size DWT_PAC8..DWT_PAC64 (Relates to RX preamble length)
                    .preambleCodeIndex = 9,         //!< RX preamble code
                    .sfdType = 0,                   //!< Boolean should we use non-standard SFD for better performance
                    .phrMode = DWT_PHRMODE_STD,     //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
                    .sfdTimeout = (128 + 1 + 8 - 8) //!< SFD timeout value (in symbols) (preamble length + 1 + SFD length - PAC size). Used in RX only. 
                },
                .tx ={
                    .preambleCodeIndex = 9,         //!< TX preamble code
                    .preambleLength = DWT_PLEN_128  //!< DWT_PLEN_64..DWT_PLEN_4096
                },
                .txrf={
                    .PGdly = TC_PGDELAY_CH5,
                    //.power = 0x2A4A6A8A,
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
                .LDE_enable = 1,
                .LDO_enable = 0,
                .sleep_enable = 1,
                .wakeup_rx_enable = 1,     //!< Wakeup to Rx state
                .rxauto_enable = 1         //!< On error re-enable
            },
            .spi_sem = 0,
            .task_prio = 5
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
            .attrib = {                //!< phy attritubes per the IEEE802.15.4-2011 standard, Table 99 and Table 101 
                .Tpsym = 1.01760,      //!< Preamble symbols duration (usec) for MPRF of 62.89Mhz
                .Tbsym = 1.02564,      //!< Baserate symbols duration (usec) 850khz
                .Tdsym = 0.12821/0.87, //!< Datarate symbols duration (usec) 6.81Mhz adjusted for RS coding
                .nsfd = 8,             //!< Number of symbols in start of frame delimiter
                .nsync = 128,          //!< Number of symbols in preamble sequence
                .nphr = 16             //!< Number of symbols in phy header
            },
            .config = {
                .channel = 5,                       //!< channel number {1, 2, 3, 4, 5, 7 }
                .prf = DWT_PRF_64M,                 //!< Pulse Repetition Frequency {DWT_PRF_16M or DWT_PRF_64M}
                .dataRate = DWT_BR_6M8,             //!< Data rate. 
                .rx = {
                    .pacLength = DWT_PAC8,          //!< Acquisition Chunk Size (Relates to RX preamble length)
                    .preambleCodeIndex = 9,         //!< RX preamble code
                    .sfdType = 0,                   //!< Boolean should we use non-standard SFD for better performance
                    .phrMode = DWT_PHRMODE_STD,     //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
                    .sfdTimeout = (128 + 1 + 8 - 8) //!< SFD timeout value (in symbols) (preamble length + 1 + SFD length - PAC size). Used in RX only. 
                },
                .tx ={
                    .preambleCodeIndex = 9,         //!< TX preamble code
                    .preambleLength = DWT_PLEN_128  //!< DWT_PLEN_64..DWT_PLEN_4096
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
            .spi_sem = 0,
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
            .attrib = {                //!< phy attritubes per the IEEE802.15.4-2011 standard, Table 99 and Table 101 
                .Tpsym = 1.01760,      //!< Preamble symbols duration (usec) for MPRF of 62.89Mhz
                .Tbsym = 1.02564,      //!< Baserate symbols duration (usec) 850khz
                .Tdsym = 0.12821/0.87, //!< Datarate symbols duration (usec) 6.81Mhz adjusted for RS coding
                .nsfd = 8,             //!< Number of symbols in start of frame delimiter
                .nsync = 128,          //!< Number of symbols in preamble sequence
                .nphr = 16             //!< Number of symbols in phy header
            },
             .config = {
                .channel = 5,                       //!< channel number {1, 2, 3, 4, 5, 7 }
                .prf = DWT_PRF_64M,                 //!< Pulse Repetition Frequency {DWT_PRF_16M or DWT_PRF_64M}
                .dataRate = DWT_BR_6M8,             // Data rate. 
                .rx = {
                    .pacLength = DWT_PAC8,          //!< Acquisition Chunk Size (Relates to RX preamble length)
                    .preambleCodeIndex = 9,         //!< RX preamble code
                    .sfdType = 0,                   //!< Boolean should we use non-standard SFD for better performance
                    .phrMode = DWT_PHRMODE_STD,     //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
                    .sfdTimeout = (128 + 1 + 8 - 8) //!< SFD timeout value (in symbols) (preamble length + 1 + SFD length - PAC size). Used in RX only. 
                },
                .tx ={
                    .preambleCodeIndex = 9,         //!< TX preamble code
                    .preambleLength = DWT_PLEN_128  //!< DWT_PLEN_64..DWT_PLEN_4096
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
            .spi_sem = 0
            .interrupt_task_prio = 7
    }
    #endif
    #endif
    #endif
};
#endif
/**
 * API to choose DW1000 instances based on parameters.
 *
 * @param idx  Indicates number of instances for the chosen bsp. 
 * @return dw1000_dev_instance_t 
 */

struct _dw1000_dev_instance_t * 
hal_dw1000_inst(uint8_t idx){
    
#if  MYNEWT_VAL(DW1000_DEVICE_0) 
#if  MYNEWT_VAL(DW1000_DEVICE_1)
    assert(idx < 2);  
#else
    assert(idx < 1);  
#endif
    return &hal_dw1000_instances[idx];
#else
    assert(0);  
#endif


}

/**
 * API to reset all the gpio pins.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @return void
 */
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

/**
 * API to perform a blocking read over SPI
 *
 * @param inst      Pointer to dw1000_dev_instance_t.
 * @param cmd       Represents an array of masked attributes like reg,subindex,operation,extended,subaddress.
 * @param cmd_size  Represents value based on the cmd attributes.
 * @param buffer    Results are stored into the buffer.
 * @param length    Represents buffer length.
 * @return void
 */
void 
hal_dw1000_read(struct _dw1000_dev_instance_t * inst,
                const uint8_t * cmd, uint8_t cmd_size,
                uint8_t * buffer, uint16_t length)
{
    os_error_t err;
    assert(inst->spi_sem);
    err = os_sem_pend(inst->spi_sem, OS_TIMEOUT_NEVER);
    assert(err == OS_OK);
    hal_gpio_write(inst->ss_pin, 0);

    hal_spi_txrx(inst->spi_num, (void*)cmd, 0, cmd_size);
    for(uint16_t i = 0; i < length; i++)
        buffer[i] = hal_spi_tx_val(inst->spi_num, 0);

    hal_gpio_write(inst->ss_pin, 1);

    err = os_sem_release(inst->spi_sem);
    assert(err == OS_OK);
}


/**
 * Interrupt context callback for nonblocking SPI-functions
 *
 * @param ev    pointer to os_event
 * @return void
 */
void
hal_dw1000_spi_txrx_cb(void *arg, int len)
{
    os_error_t err;
    struct _dw1000_dev_instance_t * inst = arg;
    assert(inst!=0);

    /* Check for longer nonblocking read/write op */
    if (inst->spi_nb_sem.sem_tokens == 0) {
        err = os_sem_release(&inst->spi_nb_sem);
        assert(err == OS_OK);
    } else {
        hal_gpio_write(inst->ss_pin, 1);
        err = os_sem_release(inst->spi_sem);
        assert(err == OS_OK);
    }
}


/**
 * API to perform a non-blocking read from SPI
 *
 * @param inst      Pointer to dw1000_dev_instance_t.
 * @param cmd       Represents an array of masked attributes like reg,subindex,operation,extended,subaddress.
 * @param cmd_size  Represents value based on the cmd attributes.
 * @param buffer    Results are stored into the buffer.
 * @param length    Represents buffer length.
 * @return void
 */
void 
hal_dw1000_read_noblock(struct _dw1000_dev_instance_t * inst, const uint8_t * cmd, uint8_t cmd_size, uint8_t * buffer, uint16_t length)
{
    int rc;
    os_error_t err;
    assert(inst->spi_sem);

    err = os_sem_pend(inst->spi_sem, OS_TIMEOUT_NEVER);
    assert(err == OS_OK);
    
    hal_gpio_write(inst->ss_pin, 0);

    /* Send command portion */
    hal_spi_txrx(inst->spi_num, (void*)cmd, 0, cmd_size);

    /* Nonblocking reads can only do a maximum of 255 bytes at a time. And
     * not read more than what can fit in the tx_buffer at a time. */
    int step = (MYNEWT_VAL(DW1000_HAL_SPI_BUFFER_SIZE) > 255) ? 255 :
        MYNEWT_VAL(DW1000_HAL_SPI_BUFFER_SIZE);
    int bytes_left = length;
    for (int offset = 0;offset<length;offset+=step) {
        int bytes_to_read = (bytes_left > step) ? step : bytes_left;
        bytes_left-=bytes_to_read;

        /* Only use the spi_nb_sem if needed */
        if (bytes_left) {
            err = os_sem_pend(&inst->spi_nb_sem, OS_TIMEOUT_NEVER);
            assert(err == OS_OK);
        }

        rc = hal_spi_txrx_noblock(inst->spi_num, (void*)tx_buffer,
                                  (void*)buffer+offset, bytes_to_read);
        assert(rc==OS_OK);

        /* Only wait for this round if there is more data to read */
        if (bytes_left) {
            err = os_sem_pend(&inst->spi_nb_sem, OS_TIMEOUT_NEVER);
            assert(err == OS_OK);

            err = os_sem_release(&inst->spi_nb_sem);
            assert(err == OS_OK);
        }
    }

    /* Reaquire semaphore after rx complete */
    err = os_sem_pend(inst->spi_sem, OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    err = os_sem_release(inst->spi_sem);
    assert(err == OS_OK);
}


/**
 * API to perform a blocking write over SPI
 *
 * @param inst      Pointer to dw1000_dev_instance_t.
 * @param cmd       Represents an array of masked attributes like reg,subindex,operation,extended,subaddress.
 * @param cmd_size  Length of command array
 * @param buffer    Data buffer to be sent to device
 * @param length    Represents buffer length. 
 * @return void
 */
void 
hal_dw1000_write(struct _dw1000_dev_instance_t * inst, const uint8_t * cmd, uint8_t cmd_size, uint8_t * buffer, uint16_t length)
{
    os_error_t err;
    assert(inst->spi_sem);
    err = os_sem_pend(inst->spi_sem, OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    hal_gpio_write(inst->ss_pin, 0);

    hal_spi_txrx(inst->spi_num, (void*)cmd, 0, cmd_size);
    hal_spi_txrx(inst->spi_num, (void*)buffer, 0, length);
     
    hal_gpio_write(inst->ss_pin, 1);

    err = os_sem_release(inst->spi_sem);
    assert(err == OS_OK);
}


/**
 * API to perform a nonblocking write over SPI
 *
 * @param inst      Pointer to dw1000_dev_instance_t.
 * @param cmd       Represents an array of masked attributes like reg,subindex,operation,extended,subaddress.
 * @param cmd_size  Length of command array
 * @param buffer    Data buffer to be sent to device
 * @param length    Represents buffer length. 
 * @return void
 */
void 
hal_dw1000_write_noblock(struct _dw1000_dev_instance_t * inst, const uint8_t * cmd, uint8_t cmd_size, uint8_t * buffer, uint16_t length)
{
    int rc = OS_OK;
    os_error_t err;
    assert(length);
    assert(inst->spi_sem);
    err = os_sem_pend(inst->spi_sem, OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    hal_gpio_write(inst->ss_pin, 0);
    rc = hal_spi_txrx(inst->spi_num, (void*)cmd, 0, cmd_size);
    assert(rc==OS_OK);

    /* Nonblocking writes can only do a maximum of 255 bytes at a time */
    int step = 255;
    int bytes_left = length;
    for (int offset = 0;offset<length;offset+=step) {
        int bytes_to_write = (bytes_left > step) ? step : bytes_left;
        bytes_left-=bytes_to_write;

        /* Only use the spi_nb_sem if needed */
        if (bytes_left) {
            err = os_sem_pend(&inst->spi_nb_sem, OS_TIMEOUT_NEVER);
            assert(err == OS_OK);
        }

        rc = hal_spi_txrx_noblock(inst->spi_num, (void*)buffer+offset,
                                  0, bytes_to_write);
        assert(rc==OS_OK);

        /* Only wait for this round if there is more data to read */
        if (bytes_left) {
            /* Wait for this round of writing to complete */
            err = os_sem_pend(&inst->spi_nb_sem, OS_TIMEOUT_NEVER);
            assert(err == OS_OK);
            
            err = os_sem_release(&inst->spi_nb_sem);
            assert(err == OS_OK);
        }
    }
}


/**
 * API to wake dw1000 from sleep mode
 *
 * @param inst  Pointer to dw1000_dev_instance_t. 
 * @return void
 */
void 
hal_dw1000_wakeup(struct _dw1000_dev_instance_t * inst)
{
    os_error_t err;
    os_sr_t sr;
    assert(inst->spi_sem);
    err = os_sem_pend(inst->spi_sem, OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    OS_ENTER_CRITICAL(sr);
    
    hal_spi_disable(inst->spi_num);
    hal_gpio_write(inst->ss_pin, 0);

    // Need to hold chip select for a minimum of 600us
    os_cputime_delay_usecs(2000);

    hal_gpio_write(inst->ss_pin, 1);
    hal_spi_enable(inst->spi_num);

    // Waiting for XTAL to start and stabilise - 5ms safe
    // (check PLL bit in IRQ?)
    os_cputime_delay_usecs(5000);

    OS_EXIT_CRITICAL(sr);

    err = os_sem_release(inst->spi_sem);
    assert(err == OS_OK);
}

/**
 * API to read the current level of the rst pin. 
 * When sleeping dw1000 will let this pin go low. 
 *
 * @param inst  Pointer to dw1000_dev_instance_t
 * @return status of rst_pin
 */
int
hal_dw1000_get_rst(struct _dw1000_dev_instance_t * inst)
{
    return hal_gpio_read(inst->rst_pin);
}
