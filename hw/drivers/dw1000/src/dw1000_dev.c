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
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_phy.h>


dw1000_dev_status_t 
dw1000_read(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, uint8_t * buffer, uint16_t length){
    assert(reg <= 0x3F); // Record number is limited to 6-bits.
    assert((subaddress <= 0x7FFF) && ((subaddress + length) <= 0x7FFF)); // Index and sub-addressable area are limited to 15-bits.

    dw1000_cmd_t cmd = {
        .reg = reg,
        .subindex = subaddress != 0,
        .operation = 0, //Read
        .extended = subaddress > 128,
        .subaddress = subaddress
    };

    uint8_t header[] = {
        [0] = cmd.operation << 7 | cmd.subindex << 6 | cmd.reg,
        [1] = cmd.extended << 7 | (uint8_t) (subaddress),
        [2] = (uint8_t) (subaddress >> 7)
    };

    uint8_t len = cmd.subaddress?(cmd.extended?3:2):1;
    hal_dw1000_read(inst, header, len, buffer, length);  // result is stored in the buffer

    return inst->status;
}

dw1000_dev_status_t 
dw1000_write(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, uint8_t * buffer, uint16_t length)
{
    assert(reg <= 0x3F); // Record number is limited to 6-bits.
    assert((subaddress <= 0x7FFF) && ((subaddress + length) <= 0x7FFF)); // Index and sub-addressable area are limited to 15-bits.

  dw1000_cmd_t cmd = {
        .reg = reg,
        .subindex = subaddress != 0,
        .operation = 1, //Write
        .extended = subaddress > 128,
        .subaddress = subaddress
    };

    uint8_t header[] = {
        [0] = cmd.operation << 7 | cmd.subindex << 6 | cmd.reg,
        [1] = cmd.extended << 7 | (uint8_t) (subaddress),
        [2] = (uint8_t) (subaddress >> 7)
    };

    uint8_t len = cmd.subaddress?(cmd.extended?3:2):1; 
    hal_dw1000_write(inst, header, len, buffer, length); 

    return inst->status;
}

uint64_t 
dw1000_read_reg(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, size_t nbytes)
{
    union _buffer{
        uint8_t array[sizeof(uint64_t)];
        uint64_t value;
    } __attribute__((__packed__)) buffer;
    
    assert(nbytes <= sizeof(uint64_t));
    dw1000_read(inst, reg, subaddress, buffer.array, nbytes); // Read nbytes register into buffer

    return buffer.value;
} 

void 
dw1000_write_reg(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, uint64_t val, size_t nbytes)
{
     union _buffer{
        uint8_t array[sizeof(uint64_t)];
        uint64_t value;
    } __attribute__((__packed__))  buffer;

    buffer.value = val;
    assert(nbytes <= sizeof(uint64_t));
    dw1000_write(inst, reg, subaddress, buffer.array, nbytes); 
} 

void 
dw1000_softreset(dw1000_dev_instance_t * inst)
{
    // Set system clock to XTI
    uint8_t reg = (uint8_t) dw1000_read_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, sizeof(uint8_t));
    reg &= (uint8_t)~PMSC_CTRL0_SYSCLKS_19M & (uint8_t)~PMSC_CTRL0_SYSCLKS_125M;
    reg |= (uint8_t) PMSC_CTRL0_SYSCLKS_19M;
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, reg, sizeof(uint8_t));
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL1_OFFSET, PMSC_CTRL1_PKTSEQ_DISABLE, sizeof(uint16_t)); // Disable PMSC ctrl of RF and RX clk blocks
    dw1000_write_reg(inst, AON_ID, AON_WCFG_OFFSET, 0x0, sizeof(uint16_t)); // Clear any AON auto download bits (as reset will trigger AON download)
    dw1000_write_reg(inst, AON_ID, AON_CFG0_OFFSET, 0x0, sizeof(uint8_t));  // Clear the wake-up configuration    
    // Uploads always-on (AON) data array and configuration
    dw1000_write_reg(inst, AON_ID, AON_CTRL_OFFSET, 0x0, sizeof(uint16_t)); // Clear the register
    dw1000_write_reg(inst, AON_ID, AON_CTRL_OFFSET, AON_CTRL_SAVE, sizeof(uint16_t));
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_ALL, sizeof(uint8_t));// Reset HIF, TX, RX and PMSC

    // DW1000 needs a 10us sleep to let clk PLL lock after reset - the PLL will automatically lock after the reset
    os_cputime_delay_usecs(10);

    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_CLEAR, sizeof(uint8_t)); // Clear reset
}

/**
 * Callback to initialize an dw1000_dev_instance_t structure from the os device
 * initialization callback.  
 *
 * @param1 os device ptr
 * @param2 struct dw1000_dev_cfg ptr
 * @return OS_OK on success
 */
int 
dw1000_dev_init(struct os_dev *odev, void *arg)
{
    struct dw1000_dev_cfg *cfg = (struct dw1000_dev_cfg*)arg;
    dw1000_dev_instance_t *inst = (dw1000_dev_instance_t *)odev;
    
    if (inst == NULL ) {
        inst = (dw1000_dev_instance_t *) malloc(sizeof(dw1000_dev_instance_t));
        assert(inst);
        memset(inst,0,sizeof(dw1000_dev_instance_t));
        inst->status.selfmalloc = 1;
    }

    inst->spi_mutex = cfg->spi_mutex;
    inst->spi_num  = cfg->spi_num;

    os_error_t err = os_mutex_init(&inst->mutex);
    assert(err == OS_OK);

    err = os_sem_init(&inst->sem, 0x1); 
    assert(err == OS_OK);
    
    return OS_OK;
}

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
    rc = hal_spi_enable(inst->spi_num);
    assert(rc == 0);

    inst->device_id = dw1000_read_reg(inst, DEV_ID_ID, 0, sizeof(uint32_t));
    inst->status.initialized = (inst->device_id == DWT_DEVICE_ID);
    if (!inst->status.initialized && --timeout)
    {
        /* In case dw1000 was sleeping */
        dw1000_dev_wakeup(inst);
        goto retry;
    }

    if(!inst->status.initialized)
    {
        return OS_TIMEOUT;
    }
    inst->timestamp = (uint64_t) dw1000_read_reg(inst, SYS_TIME_ID, SYS_TIME_OFFSET, SYS_TIME_LEN);

    inst->spi_settings.baudrate = MYNEWT_VAL(DW1000_DEVICE_BAUDRATE_HIGH);
    rc = hal_spi_disable(inst->spi_num);
    assert(rc == 0);
    rc = hal_spi_config(inst->spi_num, &inst->spi_settings);
    assert(rc == 0);
    rc = hal_spi_enable(inst->spi_num);
    assert(rc == 0);

    return OS_OK;
}

void 
dw1000_dev_free(dw1000_dev_instance_t * inst){
    assert(inst);  
    hal_spi_disable(inst->spi_num);  

    if (inst->status.selfmalloc)
        free(inst);
    else
        inst->status.initialized = 0;
}


/*! 
 * @fn dw1000_mac_configure_sleep()
 *
 *  @brief configures the device for both DEEP_SLEEP and SLEEP modes, and on-wake mode
 *  I.e. before entering the sleep, the device should be programmed for TX or RX, then upon "waking up" the TX/RX settings
 *  will be preserved and the device can immediately perform the desired action TX/RX
 *
 * NOTE: e.g. Tag operation - after deep sleep, the device needs to just load the TX buffer and send the frame
 *
 *
 *      mode: the array and LDE code (OTP/ROM) and LDO tune, and set sleep persist
 *      DWT_LOADLDO      0x1000 - load LDO tune value from OTP
 *      DWT_LOADUCODE    0x0800 - load ucode from OTP
 *      DWT_PRESRV_SLEEP 0x0100 - preserve sleep
 *      DWT_LOADOPSET    0x0080 - load operating parameter set on wakeup
 *      DWT_CONFIG       0x0040 - download the AON array into the HIF (configuration download)
 *      DWT_LOADEUI      0x0008
 *      DWT_GOTORX       0x0002
 *      DWT_TANDV        0x0001
 *
 *      wake: wake up parameters
 *      DWT_XTAL_EN      0x10 - keep XTAL running during sleep
 *      DWT_WAKE_SLPCNT  0x8 - wake up after sleep count
 *      DWT_WAKE_CS      0x4 - wake up on chip select
 *      DWT_WAKE_WK      0x2 - wake up on WAKEUP PIN
 *      DWT_SLP_EN       0x1 - enable sleep/deep sleep functionality
 *
 * input parameters
 * @param mode - config on-wake parameters
 * @param wake - config wake up parameters
 *
 * output parameters
 *
 * no return value
 */
void
dw1000_dev_configure_sleep(dw1000_dev_instance_t * inst, uint16_t mode, uint8_t wake)
{
    inst->sleep_mode = mode;
    dw1000_write_reg(inst, AON_ID, AON_WCFG_OFFSET, mode, sizeof(uint16_t));
    dw1000_write_reg(inst, AON_ID, AON_CFG0_OFFSET, wake, sizeof(uint16_t));
}


dw1000_dev_status_t
dw1000_dev_enter_sleep(dw1000_dev_instance_t * inst)
{
    // Critical region, atomic lock with mutex
    os_error_t err = os_mutex_pend(&inst->mutex, OS_WAIT_FOREVER);
    assert(err == OS_OK);
    
    /* Upload always on array configuration and enter sleep */
    dw1000_write_reg(inst, AON_ID, AON_CTRL_OFFSET, 0x0, sizeof(uint16_t));
    dw1000_write_reg(inst, AON_ID, AON_CTRL_OFFSET, AON_CTRL_SAVE, sizeof(uint16_t));
    inst->status.sleeping = 1;

    // Critical region, unlock mutex
    err = os_mutex_release(&inst->mutex);
    assert(err == OS_OK);
    return inst->status;
}


dw1000_dev_status_t
dw1000_dev_wakeup(dw1000_dev_instance_t * inst)
{
    int timeout=5;
    uint32_t devid;
    // Critical region, atomic lock with mutex
    os_error_t err = os_mutex_pend(&inst->mutex, OS_WAIT_FOREVER);
    assert(err == OS_OK);

    devid = dw1000_read_reg(inst, DEV_ID_ID, 0, sizeof(uint32_t));

    while (devid != 0xDECA0130 && --timeout)
    {
        hal_dw1000_wakeup(inst);
        devid = dw1000_read_reg(inst, DEV_ID_ID, 0, sizeof(uint32_t));
    }
    inst->status.sleeping = (devid != DWT_DEVICE_ID);
    dw1000_write_reg(inst, SYS_STATUS_ID, 0, SYS_STATUS_SLP2INIT, sizeof(uint32_t));
    dw1000_write_reg(inst, SYS_STATUS_ID, 0, SYS_STATUS_ALL_RX_ERR, sizeof(uint32_t));

    /* Antenna delays lost in deep sleep ? */
    dw1000_phy_set_rx_antennadelay(inst, inst->rx_antenna_delay);
    dw1000_phy_set_tx_antennadelay(inst, inst->tx_antenna_delay);
    
    // Critical region, unlock mutex
    err = os_mutex_release(&inst->mutex);
    assert(err == OS_OK);
    //hal_gpio_irq_enable(inst->irq_pin);
    return inst->status;
}


/*! 
 * @fn dw1000_dev_enter_sleep_after_tx(int enable)
 *
 *  @brief sets the auto TX to sleep bit. This means that after a frame
 *  transmission the device will enter deep sleep mode. The dwt_setdeepsleep() function
 *  needs to be called before this to configure the on-wake settings
 *
 * NOTE: the IRQ line has to be low/inactive (i.e. no pending events)
 *
 * input parameters
 * @param enable - 1 to configure the device to enter deep sleep after TX, 0 - disables the configuration
 *
 * output parameters
 *
 * no return value
 */
void
dw1000_dev_enter_sleep_after_tx(dw1000_dev_instance_t * inst, int enable)
{
    uint32_t reg = dw1000_read_reg(inst, PMSC_ID, PMSC_CTRL1_OFFSET, sizeof(uint32_t));

    //set the auto TX -> sleep bit
    if(enable)
    {
        reg |= PMSC_CTRL1_ATXSLP;
    }
    else
    {
        reg &= ~(PMSC_CTRL1_ATXSLP);
    }
    dw1000_write(inst, PMSC_ID, PMSC_CTRL1_OFFSET, (uint8_t*)&reg, sizeof(uint32_t));
}


