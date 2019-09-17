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
 * @file dw1000_dev.c
 * @author paul kettle  
 * @date 2018 
 * @brief Device file
 *
 * @details This is the dev base class which utilises the functions to perform initialization and necessary configurations on device.
 *
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <os/os.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include <stats/stats.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_phy.h>

#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

//! Structure of DW1000 attributes.
typedef struct _dw1000_cmd{
    uint32_t reg:6;          //!< Indicates the register to be read or write into
    uint32_t subindex:1;     //!< Indicates offset address of the register
    uint32_t operation:1;    //!< Read or Write operation
    uint32_t extended:1;     //!< If subaddress is higher than 128
    uint32_t subaddress:15;  //!< Indicates subaddress of register
} dw1000_cmd_t;

/**
 * API to perform dw1000_read from given address.
 *
 * @param inst          Pointer to dw1000_dev_instance_t. 
 * @param reg           Member of dw1000_cmd_t structure. 
 * @param subaddress    Member of dw1000_cmd_t structure. 
 * @param buffer        Result is stored in buffer.
 * @param length        Represents buffer length.
 * @return struct uwb_dev_status
 */

struct uwb_dev_status
dw1000_read(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, uint8_t * buffer, uint16_t length)
{
    assert(reg <= 0x3F); // Record number is limited to 6-bits.
    assert((subaddress <= 0x7FFF) && ((subaddress + length) <= 0x7FFF)); // Index and sub-addressable area are limited to 15-bits.

    dw1000_cmd_t cmd = {
        .reg = reg,
        .subindex = subaddress != 0,
        .operation = 0, //Read
        .extended = subaddress > 0x7F,
        .subaddress = subaddress
    };

    uint8_t header[] = {
        [0] = cmd.operation << 7 | cmd.subindex << 6 | cmd.reg,
        [1] = cmd.extended << 7 | (uint8_t) (subaddress),
        [2] = (uint8_t) (subaddress >> 7)
    };

    uint8_t len = cmd.subaddress?(cmd.extended?3:2):1;
    /* Possible issue here when reading shorter amounts of data
     * using the nonblocking read with double buffer. Asserts on 
     * mutex releases seen in calling function when reading frames of length 8 */
    if (length < MYNEWT_VAL(DW1000_DEVICE_SPI_RD_MAX_NOBLOCK) ||
        inst->uwb_dev.config.blocking_spi_transfers) {
        hal_dw1000_read(inst, header, len, buffer, length);
    } else {
        hal_dw1000_read_noblock(inst, header, len, buffer, length);
    }

    return inst->uwb_dev.status;
}

/**
 * API to performs dw1000_write into given address.
 *
 * @param inst          Pointer to dw1000_dev_instance_t. 
 * @param reg           Member of dw1000_cmd_t structure. 
 * @param subaddress    Member of dw1000_cmd_t structure. 
 * @param buffer        Result is stored in buffer.
 * @param length        Represents buffer length.
 * @return struct uwb_dev_status
 */

struct uwb_dev_status
dw1000_write(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, uint8_t * buffer, uint16_t length)
{
    assert(reg <= 0x3F); // Record number is limited to 6-bits.
    assert((subaddress <= 0x7FFF) && ((subaddress + length) <= 0x7FFF)); // Index and sub-addressable area are limited to 15-bits.

  dw1000_cmd_t cmd = {
        .reg = reg,
        .subindex = subaddress != 0,
        .operation = 1, //Write
        .extended = subaddress > 0x7F,
        .subaddress = subaddress
    };

    uint8_t header[] = {
        [0] = cmd.operation << 7 | cmd.subindex << 6 | cmd.reg,
        [1] = cmd.extended << 7 | (uint8_t) (subaddress),
        [2] = (uint8_t) (subaddress >> 7)
    };

    uint8_t len = cmd.subaddress?(cmd.extended?3:2):1; 
    /* Only use non-blocking write if the length of the write justifies it */
    if (len+length < 4 || inst->uwb_dev.config.blocking_spi_transfers) {
        hal_dw1000_write(inst, header, len, buffer, length);
    } else {
        hal_dw1000_write_noblock(inst, header, len, buffer, length);
    }
    return inst->uwb_dev.status;
}

/**
 * API to read data from dw1000 register based on given parameters.
 *
 * @param inst          Pointer to dw1000_dev_instance_t. 
 * @param reg           Register from where data is read. 
 * @param subaddress    Address where data is read. 
 * @param val           value to be read.
 * @param nbytes        Length of data.
 * @return   buffer.value
 */
uint64_t 
dw1000_read_reg(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, size_t nbytes)
{
    union _buffer{
        uint8_t array[sizeof(uint64_t)];
        uint64_t value;
    } __attribute__((__packed__, aligned (8))) buffer;

    assert(reg <= 0x3F); // Record number is limited to 6-bits.
    assert((subaddress <= 0x7FFF) && ((subaddress + nbytes) <= 0x7FFF)); // Index and sub-addressable area are limited to 15-bits.
    assert(nbytes <= sizeof(uint64_t));

    dw1000_cmd_t cmd = {
        .reg = reg,
        .subindex = subaddress != 0,
        .operation = 0, //Read
        .extended = subaddress > 0x7F,
        .subaddress = subaddress
    };

    uint8_t header[] = {
        [0] = cmd.operation << 7 | cmd.subindex << 6 | cmd.reg,
        [1] = cmd.extended << 7 | (uint8_t) (subaddress),
        [2] = (uint8_t) (subaddress >> 7)
    };

    uint8_t len = cmd.subaddress?(cmd.extended?3:2):1;
    hal_dw1000_read(inst, header, len, buffer.array, nbytes);  // result is stored in the buffer

    return buffer.value;
} 

/**
 * API to write data into dw1000 register based on given parameters.
 *
 * @param inst          Pointer to dw1000_dev_instance_t.
 * @param reg           Register from where data is written into.
 * @param subaddress    Address where writing of data begins. 
 * @param val           Value to be written. 
 * @param nbytes        Length of data.
 * @return   buffer.value
 */
void 
dw1000_write_reg(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, uint64_t val, size_t nbytes)
{
     union _buffer{
        uint8_t array[sizeof(uint64_t)];
        uint64_t value;
    } __attribute__((__packed__))  buffer;

    buffer.value = val;
    assert(nbytes <= sizeof(uint64_t));
    assert(reg <= 0x3F); // Record number is limited to 6-bits.
    assert((subaddress <= 0x7FFF) && ((subaddress + nbytes) <= 0x7FFF)); // Index and sub-addressable area are limited to 15-bits.

    dw1000_cmd_t cmd = {
        .reg = reg,
        .subindex = subaddress != 0,
        .operation = 1, //Write
        .extended = subaddress > 0x7F,
        .subaddress = subaddress
    };

    uint8_t header[] = {
        [0] = cmd.operation << 7 | cmd.subindex << 6 | cmd.reg,
        [1] = cmd.extended << 7 | (uint8_t) (subaddress),
        [2] = (uint8_t) (subaddress >> 7)
    };

    uint8_t len = cmd.subaddress?(cmd.extended?3:2):1;
    hal_dw1000_write(inst, header, len, buffer.array, nbytes);
} 

/**
 * API to do softreset on dw1000 by writing data into PMSC_CTRL0_SOFTRESET_OFFSET.
 *
 * @param inst  Pointer to dw1000_dev_instance_t. 
 * @return void
 */
void 
dw1000_softreset(dw1000_dev_instance_t * inst)
{
    // Set system clock to XTI
    dw1000_phy_sysclk_XTAL(inst);
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL1_OFFSET, PMSC_CTRL1_PKTSEQ_DISABLE, sizeof(uint16_t)); // Disable PMSC ctrl of RF and RX clk blocks
    dw1000_write_reg(inst, AON_ID, AON_WCFG_OFFSET, 0x0, sizeof(uint16_t)); // Clear any AON auto download bits (as reset will trigger AON download)
    dw1000_write_reg(inst, AON_ID, AON_CFG0_OFFSET, 0x0, sizeof(uint8_t));  // Clear the wake-up configuration    
    // Uploads always-on (AON) data array and configuration
    dw1000_write_reg(inst, AON_ID, AON_CTRL_OFFSET, 0x0, sizeof(uint8_t)); // Clear the register
    dw1000_write_reg(inst, AON_ID, AON_CTRL_OFFSET, AON_CTRL_SAVE, sizeof(uint8_t));
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_ALL, sizeof(uint8_t));// Reset HIF, TX, RX and PMSC

    // DW1000 needs a 10us sleep to let clk PLL lock after reset - the PLL will automatically lock after the reset
    os_cputime_delay_usecs(10);

    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_CLEAR, sizeof(uint8_t)); // Clear reset
}


/**
 * API to configure dw1000.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @returns OS_OK on success
 */
int 
dw1000_dev_config(dw1000_dev_instance_t * inst)
{
    int rc;
    int timeout = 3;

retry:
    inst->spi_settings.baudrate = MYNEWT_VAL(DW1000_DEVICE_BAUDRATE_LOW);
    hal_dw1000_reset(inst);
    rc = hal_spi_disable(inst->spi_num);
    assert(rc == 0);
    rc = hal_spi_config(inst->spi_num, &inst->spi_settings);
    assert(rc == 0);
    hal_spi_set_txrx_cb(inst->spi_num, hal_dw1000_spi_txrx_cb, (void*)inst);    
    rc = hal_spi_enable(inst->spi_num);
    assert(rc == 0);

    inst->uwb_dev.device_id = dw1000_read_reg(inst, DEV_ID_ID, 0, sizeof(uint32_t));
    inst->uwb_dev.status.initialized = (inst->uwb_dev.device_id == DWT_DEVICE_ID);
    if (!inst->uwb_dev.status.initialized && --timeout)
    {
        /* In case dw1000 was sleeping */
        dw1000_dev_wakeup(inst);
        goto retry;
    }

    if(!inst->uwb_dev.status.initialized)
    {
        return OS_TIMEOUT;
    }

    dw1000_phy_init(inst, NULL);

    /* It's now safe to increase the SPI baudrate > 4M */
    inst->spi_settings.baudrate = MYNEWT_VAL(DW1000_DEVICE_BAUDRATE_HIGH);
    rc = hal_spi_disable(inst->spi_num);
    assert(rc == 0);
    rc = hal_spi_config(inst->spi_num, &inst->spi_settings);
    assert(rc == 0);
    rc = hal_spi_enable(inst->spi_num);
    assert(rc == 0);

    inst->uwb_dev.pan_id = MYNEWT_VAL(PANID);
    inst->uwb_dev.uid = inst->part_id & 0xffff;

    if (inst == hal_dw1000_inst(0)) {
#if  MYNEWT_VAL(DW_DEVICE_ID_0)
        inst->uwb_dev.uid = MYNEWT_VAL(DW_DEVICE_ID_0);
#endif
    } else if (inst == hal_dw1000_inst(1)){
#if  MYNEWT_VAL(DW_DEVICE_ID_1)
        inst->uwb_dev.uid = MYNEWT_VAL(DW_DEVICE_ID_1);
#endif
    } else if (inst == hal_dw1000_inst(2)){
#if  MYNEWT_VAL(DW_DEVICE_ID_2)
        inst->uwb_dev.uid = MYNEWT_VAL(DW_DEVICE_ID_2);
#endif
    }
    inst->uwb_dev.euid = (((uint64_t)inst->lot_id) << 32) + inst->part_id;

    dw1000_set_panid(inst,inst->uwb_dev.pan_id);
    dw1000_mac_init(inst, NULL);

    return OS_OK;
}

/**
 * API to free the acquired resources.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @return void
 */
void 
dw1000_dev_free(dw1000_dev_instance_t * inst){
    assert(inst);  
    hal_spi_disable(inst->spi_num);  

    if (inst->uwb_dev.status.selfmalloc)
        free(inst);
    else
        inst->uwb_dev.status.initialized = 0;
}


/**
 * API to set the sleep counter to new value, this function programs the high 16-bits of the 28-bit counter.
 *
 * NOTE: this function needs to be run before dw1000_dev_configure_sleep, also the SPI freq has to be < 3MHz
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @param count Value of the sleep counter to program.
 *
 * @return void
 */
void 
dw1000_dev_set_sleep_timer(dw1000_dev_instance_t * inst, uint16_t count)
{
    dw1000_phy_sysclk_XTAL(inst); // Force system clock to be the 19.2 MHz XTI clock.
    dw1000_write_reg(inst, AON_ID, AON_CFG1_OFFSET, 0x0, sizeof(uint8_t)); // Disable the sleep counter
    dw1000_write_reg(inst, AON_ID, AON_CFG0_SLEEP_TIM_OFFSET, count, sizeof(uint16_t));     // Write new sleep counter
    dw1000_write_reg(inst, AON_ID, AON_CFG1_OFFSET, AON_CFG1_SLEEP_CEN | AON_CFG1_LPOSC_CAL, sizeof(uint8_t));   // Enable the sleep counter
    dw1000_write_reg(inst, AON_ID, AON_CTRL_OFFSET, AON_CTRL_UPL_CFG, sizeof(uint8_t));     // Upload array 
    dw1000_write_reg(inst, AON_ID, AON_CTRL_OFFSET, 0, sizeof(uint8_t));                    // Upload array 
    dw1000_phy_sysclk_SEQ(inst); // The system clock will run off the 19.2 MHz XTI clock until the PLL is calibrated and locked
}

/**
 *  API to configure the device for both DEEP_SLEEP and SLEEP modes, and on-wake mode
 *  i.e., before entering the sleep, the device should be programmed for TX or RX, then upon "waking up" the TX/RX settings
 *  will be preserved and the device can immediately perform the desired action TX/RX.
 *
 * NOTE: e.g. Tag operation - after deep sleep, the device needs to just load the TX buffer and send the frame.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return void
 */
void
dw1000_dev_configure_sleep(dw1000_dev_instance_t * inst)
{    
    uint16_t reg = dw1000_read_reg(inst, AON_ID, AON_WCFG_OFFSET, sizeof(uint16_t));
    reg |= AON_WCFG_ONW_L64P | AON_WCFG_ONW_LDC;

    if (inst->uwb_dev.status.LDE_enabled)
        reg |= AON_WCFG_ONW_LLDE;
    else
        reg &= ~AON_WCFG_ONW_LLDE;

    if (inst->uwb_dev.status.LDO_enabled)
        reg |= AON_WCFG_ONW_LLDO;
    else
        reg &= ~AON_WCFG_ONW_LLDO;

    if (inst->uwb_dev.config.wakeup_rx_enable)
        reg |= AON_WCFG_ONW_RX;
    else
        reg &= ~AON_WCFG_ONW_RX;
        
    dw1000_write_reg(inst, AON_ID, AON_WCFG_OFFSET, reg, sizeof(uint16_t));
    reg = dw1000_read_reg(inst, AON_ID, AON_CFG0_OFFSET, sizeof(uint16_t));
    reg |= AON_CFG0_WAKE_SPI | AON_CFG0_WAKE_PIN; 

    inst->uwb_dev.status.sleep_enabled = inst->uwb_dev.config.sleep_enable;
    if (inst->uwb_dev.status.sleep_enabled)
        reg |= AON_CFG0_WAKE_CNT | AON_CFG0_SLEEP_EN;
    else
        reg &= ~(AON_CFG0_WAKE_CNT | AON_CFG0_SLEEP_EN);
    dw1000_write_reg(inst, AON_ID, AON_CFG0_OFFSET, reg, sizeof(uint16_t));
}

/**
 * API to enter device into sleep mode.
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 * @return struct uwb_dev_status
 */
struct uwb_dev_status
dw1000_dev_enter_sleep(dw1000_dev_instance_t * inst)
{
    // Critical region, atomic lock with mutex
    dpl_error_t err = dpl_mutex_pend(&inst->mutex, DPL_WAIT_FOREVER);
    assert(err == DPL_OK);
    
    /* Upload always on array configuration and enter sleep */
    dw1000_write_reg(inst, AON_ID, AON_CTRL_OFFSET, 0x0, sizeof(uint16_t));
    dw1000_write_reg(inst, AON_ID, AON_CTRL_OFFSET, AON_CTRL_SAVE, sizeof(uint16_t));
    inst->uwb_dev.status.sleeping = 1;

    // Critical region, unlock mutex
    err = dpl_mutex_release(&inst->mutex);
    assert(err == DPL_OK);
    return inst->uwb_dev.status;
}

/**
 * API to wakeup device from sleep to init.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @return struct uwb_dev_status
 */
struct uwb_dev_status
dw1000_dev_wakeup(dw1000_dev_instance_t * inst)
{
    int timeout=5;
    uint32_t devid;
    // Critical region, atomic lock with mutex
    dpl_error_t err = dpl_mutex_pend(&inst->mutex, DPL_WAIT_FOREVER);
    assert(err == DPL_OK);

    devid = dw1000_read_reg(inst, DEV_ID_ID, 0, sizeof(uint32_t));

    while (devid != 0xDECA0130 && --timeout)
    {
        hal_dw1000_wakeup(inst);
        devid = dw1000_read_reg(inst, DEV_ID_ID, 0, sizeof(uint32_t));
    }
    inst->uwb_dev.status.sleeping = (devid != DWT_DEVICE_ID);
    dw1000_write_reg(inst, SYS_STATUS_ID, 0, SYS_STATUS_SLP2INIT, sizeof(uint32_t));
    dw1000_write_reg(inst, SYS_STATUS_ID, 0, SYS_STATUS_ALL_RX_ERR, sizeof(uint32_t));

    /* Antenna delays lost in deep sleep ? */
    dw1000_phy_set_rx_antennadelay(inst, inst->uwb_dev.rx_antenna_delay);
    dw1000_phy_set_tx_antennadelay(inst, inst->uwb_dev.tx_antenna_delay);

    // Critical region, unlock mutex
    err = dpl_mutex_release(&inst->mutex);
    assert(err == DPL_OK);

    /* In case dw1000 was instructed to sleep directly after tx
     * we may need to release the tx sem */
    if(dpl_sem_get_count(&inst->tx_sem) == 0) {
        dpl_sem_release(&inst->tx_sem);
    }

    return inst->uwb_dev.status;
}


/** 
 * API to set the auto TX to sleep bit. This means that after a frame
 * transmission the device will enter deep sleep mode. The dev_configure_sleep() function
 * needs to be called before this to configure the on-wake settings.
 *
 * NOTE: the IRQ line has to be low/inactive (i.e. no pending events)
 *
 * @param inst       Pointer to dw1000_dev_instance_t.
 * @param enable     1 to configure the device to enter deep sleep after TX, 0 to disables the configuration.
 * @return struct uwb_dev_status
 */
struct uwb_dev_status
dw1000_dev_enter_sleep_after_tx(dw1000_dev_instance_t * inst, uint8_t enable)
{

    inst->control.sleep_after_tx = enable;
    uint32_t reg = dw1000_read_reg(inst, PMSC_ID, PMSC_CTRL1_OFFSET, sizeof(uint32_t));
    if(inst->control.sleep_after_tx)
        reg |= PMSC_CTRL1_ATXSLP;
    else
        reg &= ~(PMSC_CTRL1_ATXSLP);
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL1_OFFSET, reg, sizeof(uint32_t));

    return inst->uwb_dev.status;
}

/**
 *  Sets the auto RX to sleep bit. This means that after a frame
 *  received the device will enter deep sleep mode. The dev_configure_sleep() function
 *  needs to be called before this to configure the on-wake settings.
 *
 * NOTE: the IRQ line has to be low/inactive (i.e. no pending events).
 * @param inst     Pointer to dw1000_dev_instance_t.
 * @param enable   1 to configure the device to enter deep sleep after TX, 0 to disables the configuration
 * @return struct uwb_dev_status
 */
struct uwb_dev_status
dw1000_dev_enter_sleep_after_rx(dw1000_dev_instance_t * inst, uint8_t enable)
{        
    inst->control.sleep_after_rx = enable;
    uint32_t reg = dw1000_read_reg(inst, PMSC_ID, PMSC_CTRL1_OFFSET, sizeof(uint32_t));
    if(inst->control.sleep_after_rx)
        reg |= PMSC_CTRL1_ARXSLP;
    else
        reg &= ~(PMSC_CTRL1_ARXSLP);

    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL1_OFFSET, reg, sizeof(uint32_t));

    return inst->uwb_dev.status;
}


/*******************************************************/

inline static struct uwb_dev_status
uwb_dw1000_mac_config(struct uwb_dev *dev, struct uwb_dev_config *config)
{
    return dw1000_mac_config((dw1000_dev_instance_t *)dev, config);
}

inline static void
uwb_dw1000_txrf_config(struct uwb_dev *dev, struct uwb_dev_txrf_config *config)
{
    dw1000_phy_config_txrf((dw1000_dev_instance_t *)dev, config);
}

inline static void
uwb_dw1000_sleep_config(struct uwb_dev *dev)
{
    dw1000_dev_configure_sleep((dw1000_dev_instance_t *)dev);
}

inline static struct uwb_dev_status
uwb_dw1000_enter_sleep(struct uwb_dev *dev)
{
    return dw1000_dev_enter_sleep((dw1000_dev_instance_t *)dev);
}

inline static struct uwb_dev_status
uwb_dw1000_enter_sleep_after_tx(struct uwb_dev *dev, uint8_t enable)
{
    return dw1000_dev_enter_sleep_after_tx((dw1000_dev_instance_t *)dev, enable);
}

inline static struct uwb_dev_status
uwb_dw1000_enter_sleep_after_rx(struct uwb_dev *dev, uint8_t enable)
{
    return dw1000_dev_enter_sleep_after_rx((dw1000_dev_instance_t *)dev, enable);
}

inline static struct uwb_dev_status
uwb_dw1000_wakeup(struct uwb_dev *dev)
{
    return dw1000_dev_wakeup((dw1000_dev_instance_t *)dev);
}

inline static struct uwb_dev_status
uwb_dw1000_set_dblrxbuf(struct uwb_dev *dev, bool enable)
{
    return dw1000_set_dblrxbuff((dw1000_dev_instance_t *)dev, enable);
}

inline static struct uwb_dev_status
uwb_dw1000_set_rx_timeout(struct uwb_dev *dev, uint32_t timeout)
{
    return dw1000_set_rx_timeout((dw1000_dev_instance_t *)dev, timeout);
}

inline static struct uwb_dev_status
uwb_dw1000_adj_rx_timeout(struct uwb_dev *dev, uint32_t timeout)
{
    return dw1000_adj_rx_timeout((dw1000_dev_instance_t *)dev, timeout);
}

inline static struct uwb_dev_status
uwb_dw1000_set_delay_start(struct uwb_dev *dev, uint64_t dx_time)
{
    return dw1000_set_delay_start((dw1000_dev_instance_t *)dev, dx_time);
}

inline static struct uwb_dev_status
uwb_dw1000_start_tx(struct uwb_dev *dev)
{
    return dw1000_start_tx((dw1000_dev_instance_t *)dev);
}

inline static struct uwb_dev_status
uwb_dw1000_start_rx(struct uwb_dev *dev)
{
    return dw1000_start_rx((dw1000_dev_instance_t *)dev);
}

inline static struct uwb_dev_status
uwb_dw1000_stop_rx(struct uwb_dev *dev)
{
    return dw1000_stop_rx((dw1000_dev_instance_t *)dev);
}

inline static struct uwb_dev_status
uwb_dw1000_write_tx(struct uwb_dev* dev, uint8_t *tx_frame_bytes,
                    uint16_t tx_buffer_offset, uint16_t tx_frame_length)
{
    return dw1000_write_tx((dw1000_dev_instance_t *)dev, tx_frame_bytes,
                           tx_buffer_offset, tx_frame_length);
}

inline static void
uwb_dw1000_write_tx_fctrl(struct uwb_dev* dev, uint16_t tx_frame_length,
                          uint16_t tx_buffer_offset)
{
    dw1000_write_tx_fctrl((dw1000_dev_instance_t *)dev, tx_frame_length,
                          tx_buffer_offset);
}

inline static dpl_error_t
uwb_dw1000_hal_noblock_wait(struct uwb_dev* dev, dpl_time_t timeout)
{
    return hal_dw1000_rw_noblock_wait((dw1000_dev_instance_t *)dev, timeout);
}

inline static struct uwb_dev_status
uwb_dw1000_set_wait4resp(struct uwb_dev *dev, bool enable)
{
    return dw1000_set_wait4resp((dw1000_dev_instance_t *)dev, enable);
}

inline static struct uwb_dev_status
uwb_dw1000_set_wait4resp_delay(struct uwb_dev *dev, uint32_t delay)
{
    return dw1000_set_wait4resp_delay((dw1000_dev_instance_t *)dev, delay);
}

inline static struct uwb_dev_status
uwb_dw1000_set_rxauto_disable(struct uwb_dev *dev, bool disable)
{
    return dw1000_set_rxauto_disable((dw1000_dev_instance_t *)dev, disable);
}

inline static uint64_t
uwb_dw1000_read_systime(struct uwb_dev* dev)
{
    return dw1000_read_systime((dw1000_dev_instance_t *)dev);
}

inline static uint32_t
uwb_dw1000_read_systime_lo32(struct uwb_dev* dev)
{
    return dw1000_read_systime_lo((dw1000_dev_instance_t *)dev);
}

inline static uint64_t
uwb_dw1000_read_rxtime(struct uwb_dev* dev)
{
    return dw1000_read_rxtime((dw1000_dev_instance_t *)dev);
}

inline static uint32_t
uwb_dw1000_read_rxtime_lo32(struct uwb_dev* dev)
{
    return dw1000_read_rxtime_lo((dw1000_dev_instance_t *)dev);
}

inline static uint64_t
uwb_dw1000_read_txtime(struct uwb_dev* dev)
{
    return dw1000_read_txtime((dw1000_dev_instance_t *)dev);
}

inline static uint32_t
uwb_dw1000_read_txtime_lo32(struct uwb_dev* dev)
{
    return dw1000_read_txtime_lo((dw1000_dev_instance_t *)dev);
}

inline static uint16_t
uwb_dw1000_phy_frame_duration(struct uwb_dev* dev, uint16_t nlen)
{
    return dw1000_phy_frame_duration(&dev->attrib, nlen);
}

inline static uint16_t
uwb_dw1000_phy_SHR_duration(struct uwb_dev* dev)
{
    return dw1000_phy_SHR_duration(&dev->attrib);
}

inline static void
uwb_dw1000_phy_forcetrxoff(struct uwb_dev* dev)
{
    return dw1000_phy_forcetrxoff((dw1000_dev_instance_t *)dev);
}

inline static void
uwb_dw1000_phy_rx_reset(struct uwb_dev* dev)
{
    return dw1000_phy_rx_reset((dw1000_dev_instance_t *)dev);
}

inline static struct uwb_dev_status
uwb_dw1000_set_on_error_continue(struct uwb_dev * dev, bool enable)
{
    return dw1000_set_on_error_continue((dw1000_dev_instance_t *)dev, enable);
}

inline static void
uwb_dw1000_set_panid(struct uwb_dev * dev, uint16_t pan_id)
{
    return dw1000_set_panid((dw1000_dev_instance_t *)dev, pan_id);
}

inline static float
uwb_dw1000_calc_clock_offset_ratio(struct uwb_dev * dev, int32_t integrator_val)
{
    return dw1000_calc_clock_offset_ratio((dw1000_dev_instance_t *)dev, integrator_val);
}

inline static float
uwb_dw1000_get_rssi(struct uwb_dev * dev)
{
    return dw1000_get_rssi((dw1000_dev_instance_t *)dev);
}

inline static float
uwb_dw1000_get_fppl(struct uwb_dev * dev)
{
    return dw1000_get_fppl((dw1000_dev_instance_t *)dev);
}

inline static float
uwb_dw1000_calc_rssi(struct uwb_dev * dev, struct uwb_dev_rxdiag * diag)
{
    return dw1000_calc_rssi((dw1000_dev_instance_t *)dev, (struct _dw1000_dev_rxdiag_t*)diag);
}

inline static float
uwb_dw1000_calc_fppl(struct uwb_dev * dev, struct uwb_dev_rxdiag * diag)
{
    return dw1000_calc_fppl((dw1000_dev_instance_t *)dev, (struct _dw1000_dev_rxdiag_t*)diag);
}

inline static float
uwb_dw1000_estimate_los(struct uwb_dev * dev, float rssi, float fppl)
{
    return dw1000_estimate_los(rssi, fppl);
}

static const struct uwb_driver_funcs dw1000_uwb_funcs = {
    .uf_mac_config = uwb_dw1000_mac_config,
    .uf_txrf_config = uwb_dw1000_txrf_config,
    .uf_sleep_config = uwb_dw1000_sleep_config,
    .uf_enter_sleep = uwb_dw1000_enter_sleep,
    .uf_enter_sleep_after_tx = uwb_dw1000_enter_sleep_after_tx,
    .uf_enter_sleep_after_rx = uwb_dw1000_enter_sleep_after_rx,
    .uf_wakeup = uwb_dw1000_wakeup,
    .uf_set_dblrxbuf = uwb_dw1000_set_dblrxbuf,
    .uf_set_rx_timeout = uwb_dw1000_set_rx_timeout,
    .uf_adj_rx_timeout = uwb_dw1000_adj_rx_timeout,
    .uf_set_delay_start = uwb_dw1000_set_delay_start,
    .uf_start_tx = uwb_dw1000_start_tx,
    .uf_start_rx = uwb_dw1000_start_rx,
    .uf_stop_rx = uwb_dw1000_stop_rx,
    .uf_write_tx = uwb_dw1000_write_tx,
    .uf_write_tx_fctrl = uwb_dw1000_write_tx_fctrl,
    .uf_hal_noblock_wait = uwb_dw1000_hal_noblock_wait,
    .uf_set_wait4resp = uwb_dw1000_set_wait4resp,
    .uf_set_wait4resp_delay = uwb_dw1000_set_wait4resp_delay,
    .uf_set_rxauto_disable = uwb_dw1000_set_rxauto_disable,
    .uf_read_systime = uwb_dw1000_read_systime,
    .uf_read_systime_lo32 = uwb_dw1000_read_systime_lo32,
    .uf_read_rxtime = uwb_dw1000_read_rxtime,
    .uf_read_rxtime_lo32 = uwb_dw1000_read_rxtime_lo32,
    .uf_read_txtime = uwb_dw1000_read_txtime,
    .uf_read_txtime_lo32 = uwb_dw1000_read_txtime_lo32,
    .uf_phy_frame_duration = uwb_dw1000_phy_frame_duration,
    .uf_phy_SHR_duration = uwb_dw1000_phy_SHR_duration,
    .uf_phy_forcetrxoff = uwb_dw1000_phy_forcetrxoff,
    .uf_phy_rx_reset = uwb_dw1000_phy_rx_reset,
    .uf_set_on_error_continue = uwb_dw1000_set_on_error_continue,
    .uf_set_panid = uwb_dw1000_set_panid,
    .uf_calc_clock_offset_ratio = uwb_dw1000_calc_clock_offset_ratio,
    .uf_get_rssi = uwb_dw1000_get_rssi,
    .uf_get_fppl = uwb_dw1000_get_fppl,
    .uf_calc_rssi = uwb_dw1000_calc_rssi,
    .uf_calc_fppl = uwb_dw1000_calc_fppl,
    .uf_estimate_los = uwb_dw1000_estimate_los,
};

/**
 * API to initialize a dw1000_dev_instance_t structure from the os device initialization callback.  
 *
 * @param odev  Pointer to struct os_dev. 
 * @param arg   Argument to set as pointer to struct dw1000_dev_cfg.
 * @return OS_OK on success
 */
int 
dw1000_dev_init(struct os_dev *odev, void *arg)
{
#if MYNEWT_VAL(DW1000_PKG_INIT_LOG)
    DIAGMSG("{\"utime\": %lu,\"msg\": \"dw1000_dev_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
#endif
    /* TODO, replace with open anch close function pointers below */
    OS_DEV_SETHANDLERS(odev, 0, 0);

    struct dw1000_dev_cfg *cfg = (struct dw1000_dev_cfg*)arg;
    struct uwb_dev *udev = (struct uwb_dev*)odev;
    dw1000_dev_instance_t *inst = (dw1000_dev_instance_t *)odev;

    /* Setup common uwb interface */
    udev->uw_funcs = &dw1000_uwb_funcs;
    udev->rxdiag = (struct uwb_dev_rxdiag*)&inst->rxdiag;
    udev->rxdiag->rxd_len = sizeof(inst->rxdiag);
    /* Check size requirements */
    assert(sizeof(inst->rxdiag) <= MYNEWT_VAL(UWB_DEV_RXDIAG_MAXLEN));
    
    if (inst == NULL ) {
        inst = (dw1000_dev_instance_t *) malloc(sizeof(dw1000_dev_instance_t));
        assert(inst);
        memset(inst,0,sizeof(dw1000_dev_instance_t));
        inst->uwb_dev.status.selfmalloc = 1;
    }

    inst->spi_sem = cfg->spi_sem;
    inst->spi_num = cfg->spi_num;

    dpl_error_t err = dpl_mutex_init(&inst->mutex);
    assert(err == DPL_OK);
    err = dpl_sem_init(&inst->tx_sem, 0x1); 
    assert(err == DPL_OK);
    err = dpl_sem_init(&inst->spi_nb_sem, 0x1);
    assert(err == DPL_OK);

    SLIST_INIT(&inst->uwb_dev.interface_cbs);

    return OS_OK;
}
