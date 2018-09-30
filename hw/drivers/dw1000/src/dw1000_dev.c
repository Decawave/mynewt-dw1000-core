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
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_phy.h>


static dw1000_extension_callbacks_t* dw1000_new_extension_callbacks(dw1000_dev_instance_t* inst);

/**
 * API to perform dw1000_read from given address.
 *
 * @param inst          Pointer to dw1000_dev_instance_t. 
 * @param reg           Member of dw1000_cmd_t structure. 
 * @param subaddress    Member of dw1000_cmd_t structure. 
 * @param buffer        Result is stored in buffer.
 * @param length        Represents buffer length.
 * @return dw1000_dev_status_t
 */

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
    if (length < 8) {
        hal_dw1000_read(inst, header, len, buffer, length);
    } else {
        hal_dw1000_read_noblock(inst, header, len, buffer, length);
    }

    return inst->status;
}

/**
 * API to performs dw1000_write into given address.
 *
 * @param inst          Pointer to dw1000_dev_instance_t. 
 * @param reg           Member of dw1000_cmd_t structure. 
 * @param subaddress    Member of dw1000_cmd_t structure. 
 * @param buffer        Result is stored in buffer.
 * @param length        Represents buffer length.
 * @return dw1000_dev_status_t
 */

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
    /* Only use non-blocking write if the length of the write justifies it */
    if (len+length < 4) {
        hal_dw1000_write(inst, header, len, buffer, length);
    } else {
        hal_dw1000_write_noblock(inst, header, len, buffer, length);
    }

    return inst->status;
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
        .extended = subaddress > 128,
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
        .extended = subaddress > 128,
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
 * API to initialize a dw1000_dev_instance_t structure from the os device initialization callback.  
 *
 * @param odev  Pointer to struct os_dev. 
 * @param arg   Argument to set as pointer to struct dw1000_dev_cfg.
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

    inst->spi_sem = cfg->spi_sem;
    inst->spi_num  = cfg->spi_num;

    os_error_t err = os_mutex_init(&inst->mutex);
    assert(err == OS_OK);

    err = os_sem_init(&inst->sem, 0x1); 
    assert(err == OS_OK);

    return OS_OK;
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

    dw1000_phy_init(inst, NULL);

    /* It's now safe to increase the SPI baudrate > 4M */
    inst->spi_settings.baudrate = MYNEWT_VAL(DW1000_DEVICE_BAUDRATE_HIGH);
    rc = hal_spi_disable(inst->spi_num);
    assert(rc == 0);
    rc = hal_spi_config(inst->spi_num, &inst->spi_settings);
    assert(rc == 0);
    rc = hal_spi_enable(inst->spi_num);
    assert(rc == 0);

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

    if (inst->status.selfmalloc)
        free(inst);
    else
        inst->status.initialized = 0;
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

    if (inst->status.LDE_enabled)
        reg |= AON_WCFG_ONW_LLDE;
    else
        reg &= ~AON_WCFG_ONW_LLDE;

    if (inst->status.LDO_enabled)
        reg |= AON_WCFG_ONW_LLDO;
    else
        reg &= ~AON_WCFG_ONW_LLDO;

    if (inst->config.wakeup_rx_enable)
        reg |= AON_WCFG_ONW_RX;
    else
        reg &= ~AON_WCFG_ONW_RX;
        
    dw1000_write_reg(inst, AON_ID, AON_WCFG_OFFSET, reg, sizeof(uint16_t));
    reg = dw1000_read_reg(inst, AON_ID, AON_CFG0_OFFSET, sizeof(uint16_t));
    reg |= AON_CFG0_WAKE_SPI | AON_CFG0_WAKE_PIN; 

    inst->status.sleep_enabled = inst->config.sleep_enable;
    if (inst->status.sleep_enabled)
        reg |= AON_CFG0_WAKE_CNT | AON_CFG0_SLEEP_EN;
    else
        reg &= ~(AON_CFG0_WAKE_CNT | AON_CFG0_SLEEP_EN);
    dw1000_write_reg(inst, AON_ID, AON_CFG0_OFFSET, reg, sizeof(uint16_t));
}

/**
 * API to enter device into sleep mode.
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 * @return dw1000_dev_status_t
 */
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

/**
 * API to wakeup device from sleep to init.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @return dw1000_dev_status_t 
 */
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
 
    return inst->status;
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
 * @return dw1000_dev_status_t
 */
dw1000_dev_status_t
dw1000_dev_enter_sleep_after_tx(dw1000_dev_instance_t * inst, uint8_t enable)
{

    inst->control.sleep_after_tx = enable;
    uint32_t reg = dw1000_read_reg(inst, PMSC_ID, PMSC_CTRL1_OFFSET, sizeof(uint32_t));
    if(inst->control.sleep_after_tx)
        reg |= PMSC_CTRL1_ATXSLP;
    else
        reg &= ~(PMSC_CTRL1_ATXSLP);
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL1_OFFSET, reg, sizeof(uint32_t));

    return inst->status;
}

/**
 *  Sets the auto RX to sleep bit. This means that after a frame
 *  received the device will enter deep sleep mode. The dev_configure_sleep() function
 *  needs to be called before this to configure the on-wake settings.
 *
 * NOTE: the IRQ line has to be low/inactive (i.e. no pending events).
 * @param inst     Pointer to dw1000_dev_instance_t.
 * @param enable   1 to configure the device to enter deep sleep after TX, 0 to disables the configuration
 * @return dw1000_dev_status_t
 */
dw1000_dev_status_t
dw1000_dev_enter_sleep_after_rx(dw1000_dev_instance_t * inst, uint8_t enable)
{        
    inst->control.sleep_after_rx = enable;
    uint32_t reg = dw1000_read_reg(inst, PMSC_ID, PMSC_CTRL1_OFFSET, sizeof(uint32_t));
    if(inst->control.sleep_after_rx)
        reg |= PMSC_CTRL1_ARXSLP;
    else
        reg &= ~(PMSC_CTRL1_ARXSLP);

    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL1_OFFSET, reg, sizeof(uint32_t));

    return inst->status;
}

/**
 * API to set sleep callback.
 *
 * @param inst             Pointer to dw1000_dev_instance_t.
 * @param sleep_timer_cb   Callback to set sleep time.
 *
 */
inline void 
dw1000_dev_set_sleep_callback(dw1000_dev_instance_t * inst,  dw1000_dev_cb_t sleep_timer_cb){
    inst->sleep_timer_cb = sleep_timer_cb;
}
/**
 * API to register extension  callbacks for different services.
 *
 * @param inst       Pointer to dw1000_dev_instance_t.
 * @param callbacks  callback instance.
 * @return void
 */
void
dw1000_add_extension_callbacks(dw1000_dev_instance_t* inst, dw1000_extension_callbacks_t callbacks){
    assert(inst);
    dw1000_extension_callbacks_t* prev_cbs = NULL;
    dw1000_extension_callbacks_t* cur_cbs = NULL;
    dw1000_extension_callbacks_t* new_cbs = dw1000_new_extension_callbacks(inst);
    assert(new_cbs);
    memcpy(new_cbs,&callbacks,sizeof(dw1000_extension_callbacks_t));
    if(!(SLIST_EMPTY(&inst->extension_cbs))){
        SLIST_FOREACH(cur_cbs, &inst->extension_cbs, cbs_next) {
            prev_cbs = cur_cbs;
        }
        SLIST_INSERT_AFTER(prev_cbs, new_cbs, cbs_next);
    }else
        SLIST_INSERT_HEAD(&inst->extension_cbs, new_cbs, cbs_next);
}

/**
 * API to assign memory for new callbacks.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @return new callbacks
 */
static dw1000_extension_callbacks_t*
dw1000_new_extension_callbacks(dw1000_dev_instance_t* inst){
    assert(inst);
    dw1000_extension_callbacks_t* new_cbs = (dw1000_extension_callbacks_t*)malloc(sizeof(dw1000_extension_callbacks_t));
    memset(new_cbs, 0, sizeof(dw1000_extension_callbacks_t));
    return new_cbs;
}

/**
 * API to remove specified callbacks.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @param id    ID of the service.
 * @return void
 */
void
dw1000_remove_extension_callbacks(dw1000_dev_instance_t* inst, dw1000_extension_id_t id){
    dw1000_extension_callbacks_t* temp;
    SLIST_FOREACH(temp, &inst->extension_cbs, cbs_next) {
        if(temp->id == id){
            SLIST_REMOVE(&inst->extension_cbs, temp, _dw1000_extension_callback_t, cbs_next);
            break;
        }
    }
}
