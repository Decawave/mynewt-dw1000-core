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
 * @file dw1000_phy.c
 * @author paul kettle
 * @date 2018
 * @brief physical layer
 *
 * @details This is the phy base class which utilises the functions to set the clocks,initializes phy layer and configures the required 
 * parameters. 
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <os/os.h>
#include <math.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include <dw1000/dw1000_phy.h>

static inline void _dw1000_phy_load_microcode(struct _dw1000_dev_instance_t * inst);

/**
 * API that force system clock to be the 19.2 MHz XTI clock.
 * 
 * @param inst  Pointer to dw1000_dev_instance_t. 
 * @return void
 */
inline void dw1000_phy_sysclk_XTAL(struct _dw1000_dev_instance_t * inst){
 uint8_t reg = (uint8_t) dw1000_read_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, sizeof(uint8_t));
    reg &= (uint8_t)~PMSC_CTRL0_SYSCLKS_19M & (uint8_t)~PMSC_CTRL0_SYSCLKS_125M;
    reg |= (uint8_t) PMSC_CTRL0_SYSCLKS_19M;
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, reg, sizeof(uint8_t));
}

/**
 * API that force system clock to be the 125 MHz PLL clock. 
 *
 * @param inst  Pointer to dw1000_dev_instance_t. 
 * @return void
 */
inline void dw1000_phy_sysclk_PLL(struct _dw1000_dev_instance_t * inst){
 uint8_t reg = (uint8_t) dw1000_read_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, sizeof(uint8_t));
    reg &= (uint8_t)~PMSC_CTRL0_SYSCLKS_19M & (uint8_t)~PMSC_CTRL0_SYSCLKS_125M;
    reg |= (uint8_t) PMSC_CTRL0_SYSCLKS_125M;
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, reg, sizeof(uint8_t));
}

/**
 * API to enable running of the LDE algorithm.
 *
 * @param inst  Pointer to dw1000_dev_instance_t. 
 * @return void
 */
void dw1000_phy_sysclk_LDE(struct _dw1000_dev_instance_t * inst){
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, 0x01, sizeof(uint8_t));
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET + 1 , 0x03, sizeof(uint8_t));
}

/**
 * API to enable PLL2 on/off sequencing by SNIFF mode.
 *
 * @param inst  Pointer to dw1000_dev_instance_t. 
 * @return void
 */
inline void dw1000_phy_sysclk_SEQ(struct _dw1000_dev_instance_t * inst){
    uint8_t reg = (uint8_t) dw1000_read_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, sizeof(uint8_t));
    reg &= (uint8_t)~PMSC_CTRL0_SYSCLKS_19M & (uint8_t)~PMSC_CTRL0_SYSCLKS_125M;
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, reg, sizeof(uint8_t));
}

/**
 * API to enable PLL2 on/off sequencing by SNIFF mode through masking of pmsc_ctrl_lo and pmsc_ctrl_hi.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @param mode   Switch to the case specified. 
 * @return void
 */
inline void dw1000_phy_sysclk_ACC(struct _dw1000_dev_instance_t * inst, uint8_t mode){

    uint8_t pmsc_ctrl_lo = (uint8_t) dw1000_read_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, sizeof(uint8_t));
    uint8_t pmsc_ctrl_hi = (uint8_t) dw1000_read_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET + 1, sizeof(uint8_t));

    switch(mode){
        case true:
            pmsc_ctrl_lo  =  0x48 | (pmsc_ctrl_lo & 0xb3);
            pmsc_ctrl_hi  =  0x80 | pmsc_ctrl_hi;
            break;
        default:
            pmsc_ctrl_lo  =  pmsc_ctrl_lo & 0xb3;
            pmsc_ctrl_hi  =  0x7f & pmsc_ctrl_hi ;
            break;
    }
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_ctrl_lo, sizeof(uint8_t));
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET+1, pmsc_ctrl_hi, sizeof(uint8_t));
}


/**
 * API to disable PMSC control of analog RF subsystems.
 *
 * @param inst  Pointer to dw1000_dev_instance_t. 
 * @return void
 */
void dw1000_phy_disable_sequencing(struct _dw1000_dev_instance_t * inst){ 
    dw1000_phy_sysclk_XTAL(inst);
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL1_OFFSET, PMSC_CTRL1_PKTSEQ_DISABLE, sizeof(uint16_t)); // Disable PMSC ctrl of RF and RX clk blocks
}

/**
 * API to initialise the phy layer.
 *
 * @param inst         Pointer to dw1000_dev_instance_t.
 * @param txrf_config  Pointer to dw1000_dev_txrf_config_t.
 * @return dw1000_dev_status_t
 */
dw1000_dev_status_t dw1000_phy_init(struct _dw1000_dev_instance_t * inst, dw1000_dev_txrf_config_t * txrf_config){

    if (txrf_config == NULL)
         txrf_config = &inst->config.txrf;
    else
        memcpy(&inst->config.txrf, txrf_config, sizeof(dw1000_dev_txrf_config_t));

    dw1000_softreset(inst);
    dw1000_phy_sysclk_XTAL(inst);
    dw1000_gpio_config_leds(inst, DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    // Configure the CPLL lock detect
    uint8_t reg = dw1000_read_reg(inst, EXT_SYNC_ID, EC_CTRL_OFFSET, sizeof(uint8_t));
    reg |= EC_CTRL_PLLLCK;
    dw1000_write_reg(inst, EXT_SYNC_ID, EC_CTRL_OFFSET, reg, sizeof(uint8_t));

    // Read OTP revision number
    uint32_t otp_addr = (uint32_t) _dw1000_otp_read(inst, OTP_XTRIM_ADDRESS) & 0xffff;    // Read 32 bit value, XTAL trim val is in low octet-0 (5 bits)
    inst->otp_rev = (otp_addr >> 8) & 0xff;                                               // OTP revision is next byte

    // Load LDO tune from OTP and kick it if there is a value actually programmed.
    uint32_t ldo_tune = _dw1000_otp_read(inst, OTP_LDOTUNE_ADDRESS);
    if((ldo_tune & 0xFF) != 0){
        dw1000_write_reg(inst, OTP_IF_ID, OTP_SF, OTP_SF_LDO_KICK, sizeof(uint8_t)); // Set load LDE kick bit
        inst->status.LDO_enabled = 1; // LDO tune must be kicked at wake-up
    }
    // Load Part and Lot ID from OTP
    inst->partID = _dw1000_otp_read(inst, OTP_PARTID_ADDRESS);
    inst->lotID = _dw1000_otp_read(inst, OTP_LOTID_ADDRESS);

    // Load vbat and vtemp from OTP
    inst->otp_vbat = _dw1000_otp_read(inst, OTP_VBAT_ADDRESS);
    inst->otp_temp = _dw1000_otp_read(inst, OTP_VTEMP_ADDRESS);
    
    // XTAL trim value is set in OTP for DW1000 module and EVK/TREK boards but that might not be the case in a custom design
    if (otp_addr & 0x1F) // A value of 0 means that the crystal has not been trimmed
        inst->xtal_trim = otp_addr & 0x1F;
    else
        inst->xtal_trim = FS_XTALT_MIDRANGE ; // Set to mid-range if no calibration value inside
    // The 3 MSb in this 8-bit register must be kept to 0b011 to avoid any malfunction.

    reg = (3 << 5) | (inst->xtal_trim & FS_XTALT_MASK);
    dw1000_write_reg(inst, FS_CTRL_ID, FS_XTALT_OFFSET, reg, sizeof(uint8_t));

    if(inst->config.LDE_enable)
        _dw1000_phy_load_microcode(inst);

    dw1000_phy_sysclk_SEQ(inst);    // Enable clocks for sequencing

    // The 3 bits in AON CFG1 register must be cleared to ensure proper operation of the DW1000 in DEEPSLEEP mode.
    reg = dw1000_read_reg(inst, AON_ID, AON_CFG1_OFFSET, sizeof(uint8_t));
    reg |= ~AON_CFG1_SMXX;
    dw1000_write_reg(inst, AON_ID, AON_CFG1_OFFSET, reg, sizeof(uint8_t));

    // Enable Temp & Vbat SAR onwake mode
    dw1000_write_reg(inst, AON_ID, AON_WCFG_OFFSET , AON_WCFG_ONW_RADC, sizeof(uint16_t));

    // Apply default antenna delay value. See NOTE 2 below. */
    dw1000_phy_set_rx_antennadelay(inst, inst->rx_antenna_delay);
    dw1000_phy_set_tx_antennadelay(inst, inst->tx_antenna_delay);

    // Apply tx power settings */
    dw1000_phy_config_txrf(inst, txrf_config);

    // Read system register / store local copy
    inst->sys_cfg_reg = dw1000_read_reg(inst, SYS_CFG_ID, 0, sizeof(uint32_t)) ; // Read sysconfig register

    return inst->status;
}

/**
 * API to load LDE microcode.  
 *
 * @param inst   Pointer to _dw1000_dev_instance_t.
 * @return void
 */
void _dw1000_phy_load_microcode(struct _dw1000_dev_instance_t * inst)
{
    // Set up clocks
    dw1000_phy_sysclk_LDE(inst);

    // Kick off the LDE Load
    dw1000_write_reg(inst, OTP_IF_ID, OTP_CTRL, OTP_CTRL_LDELOAD, sizeof(uint16_t)); // Set load LDE kick bit
    os_cputime_delay_usecs(120); // Allow time for code to upload (should take up to 120 us)
    dw1000_phy_sysclk_SEQ(inst); // Enable clocks for sequencing
    inst->status.LDE_enabled = 1;
}

/**
 * API to Configure LDE algorithm parameters.
 * 
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @param prf   This is the PRF index (0 or 1) 0 corresponds to 16 and 1 to 64 PRF.
 * @return void
 */
void dw1000_phy_config_lde(struct _dw1000_dev_instance_t * inst, int prfIndex)
{
    dw1000_write_reg(inst, LDE_IF_ID, LDE_CFG1_OFFSET, LDE_PARAM1, sizeof(uint8_t)); // 8-bit configuration register

    if(prfIndex)
        dw1000_write_reg(inst, LDE_IF_ID, LDE_CFG2_OFFSET, LDE_PARAM3_64, sizeof(uint16_t)); // 16-bit LDE configuration tuning register
    else
        dw1000_write_reg(inst, LDE_IF_ID, LDE_CFG2_OFFSET, LDE_PARAM3_16, sizeof(uint16_t));
}


/**
 * API for the configuration of the TX spectrum
 * including the power and pulse generator delay. The input is a pointer to the data structure
 * of type dwt_txconfig_t that holds all the configurable items.
 *
 * @param inst      Pointer to dw1000_dev_instance_t.
 * @param config    Pointer to dw1000_dev_txrf_config_t.
 * @return void
 */
void dw1000_phy_config_txrf(struct _dw1000_dev_instance_t * inst, dw1000_dev_txrf_config_t *config)
{
    // Configure RF TX PG_DELAY
    dw1000_write_reg(inst, TX_CAL_ID, TC_PGDELAY_OFFSET, config->PGdly, sizeof(uint8_t));
    // Configure TX power
    dw1000_write_reg(inst, TX_POWER_ID, 0, config->power, sizeof(uint32_t));
}



/**
 * API to read the temperature of the DW1000 that was sampled
 * on waking from Sleep/Deepsleep. They are not current values, but read on last
 * wakeup if DWT_TANDV bit is set in mode parameter of dwt_configuresleep.
 *
 * @param inst    Pointer to dw1000_dev_instance_t. 
 * @return float  value for temperature sensor in SI units (Degrees C).
 */
float dw1000_phy_read_wakeuptemp_SI(struct _dw1000_dev_instance_t * inst)
{
   return 1.14 * (dw1000_phy_read_wakeuptemp(inst) - inst->otp_temp) + 23;
}

/**
 * API to read the battery voltage of the DW1000 that was sampled
 * on waking from Sleep/Deepsleep. They are not current values, but read on last
 * wakeup if DWT_TANDV bit is set in mode parameter of dwt_configure_sleep.
 *
 * @param inst    Pointer to dw1000_dev_instance_t. 
 * @return float  Value of battery voltage sensor in SI units (Volts).
 */
float dw1000_phy_read_read_wakeupvbat_SI(struct _dw1000_dev_instance_t * inst)
{
    return (1.0/173) * (dw1000_phy_read_wakeupvbat(inst) - inst->otp_vbat) + 3.3;
}

/**
 * API to reset the receiver of the DW1000.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @return void
 */
void dw1000_phy_rx_reset(struct _dw1000_dev_instance_t * inst)
{
    os_error_t err = os_mutex_pend(&inst->mutex, OS_WAIT_FOREVER);
    assert(err == OS_OK);

    // Set RX reset
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_RX, sizeof(uint8_t));
    // Clear RX reset
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_CLEAR, sizeof(uint8_t));

    err = os_mutex_release(&inst->mutex);
    assert(err == OS_OK);
}


/**
 * API to turn off the transceiver.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @return void
 */
void dw1000_phy_forcetrxoff(struct _dw1000_dev_instance_t * inst)
{
    uint32_t mask = dw1000_read_reg(inst, SYS_MASK_ID, 0 , sizeof(uint32_t)) ; // Read set interrupt mask

    // Need to beware of interrupts occurring in the middle of following read modify write cycle
    // We can disable the radio, but before the status is cleared an interrupt can be set (e.g. the
    // event has just happened before the radio was disabled)
    // thus we need to disable interrupt during this operation

    os_error_t err = os_mutex_pend(&inst->mutex, OS_WAIT_FOREVER);
    assert(err == OS_OK);

    dw1000_write_reg(inst, SYS_MASK_ID, 0, 0, sizeof(uint32_t)) ; // Clear interrupt mask - so we don't get any unwanted events
    dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint16_t)SYS_CTRL_TRXOFF, sizeof(uint16_t)) ; // Disable the radio
    // Forcing Transceiver off - so we do not want to see any new events that may have happened
    dw1000_write_reg(inst, SYS_STATUS_ID, 0, (SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_GOOD), sizeof(uint32_t));
    dw1000_sync_rxbufptrs(inst);
    dw1000_write_reg(inst, SYS_MASK_ID, 0, mask, sizeof(uint32_t)); // Restore mask to what it was
    
    // Enable/restore interrupts again...
    err = os_mutex_release(&inst->mutex);
    assert(err == OS_OK);

    inst->control.wait4resp_enabled = 0;

    /* Reset semaphore if needed */
    if (inst->sem.sem_tokens == 0) {
        os_error_t err = os_sem_release(&inst->sem);
        assert(err == OS_OK);
        inst->status.sem_force_released = 1;
    }
}

/**
 * API to enable the specified events to trigger an interrupt.
 * The following events can be enabled:
 * DWT_INT_TFRS         0x00000080          // frame sent
 * DWT_INT_RFCG         0x00004000          // frame received with good CRC
 * DWT_INT_RPHE         0x00001000          // receiver PHY header error
 * DWT_INT_RFCE         0x00008000          // receiver CRC error
 * DWT_INT_RFSL         0x00010000          // receiver sync loss error
 * DWT_INT_RFTO         0x00020000          // frame wait timeout
 * DWT_INT_RXPTO        0x00200000          // preamble detect timeout
 * DWT_INT_SFDT         0x04000000          // SFD timeout
 * DWT_INT_ARFE         0x20000000          // frame rejected (due to frame filtering configuration)
 *
 * @param inst      Pointer to dw1000_dev_instance_t.
 * @param bitmask   Sets the events which generates interrupt.
 * @param enable    If set, the interrupts are enabled else they are cleared.
 * @return void
 */
void dw1000_phy_interrupt_mask(struct _dw1000_dev_instance_t * inst, uint32_t bitmask, uint8_t enable)
{
    // Critical region, atomic lock with mutex
    os_error_t err = os_mutex_pend(&inst->mutex, OS_WAIT_FOREVER);
    assert(err == OS_OK);

    uint32_t mask = dw1000_read_reg(inst, SYS_MASK_ID, 0, sizeof(uint32_t)) ; // Read register

    if(enable)
        mask |= bitmask ;
    else
        mask &= ~bitmask ; // Clear the bit
    
    dw1000_write_reg(inst, SYS_MASK_ID, 0, mask, sizeof(uint32_t));

    // Critical region, unlock mutex
    err = os_mutex_release(&inst->mutex);
    assert(err == OS_OK);
}

/**
 * API to synchronise DW1000 with external clocks or events or with other DW1000’s. 
 * For example, this would be required in a TDOA RTLS system employing wired clock synchronisation of the 
 * anchor nodes or AoA node for phase measurement.
 * 
 * @param inst      Pointer to dw1000_dev_instance_t. 
 * @param delay     To configure DW1000 for OSTR mode, the delay value is set to the desired delay value.
 * @param enable    True/false. 
 * @return void
 */
void dw1000_phy_external_sync(struct _dw1000_dev_instance_t * inst, uint8_t delay, bool enable){

    uint16_t reg = dw1000_read_reg(inst, EXT_SYNC_ID, EC_CTRL_OFFSET, sizeof(uint16_t));
    if (enable) {
        reg &= ~EC_CTRL_WAIT_MASK; //clear timer value, clear OSTRM
        reg |= EC_CTRL_OSTRM;      //External timebase reset mode enable
        reg |= ((((uint16_t) delay) & 0xff) << 3); //set new timer value

    }else {
        reg &= ~(EC_CTRL_WAIT_MASK | EC_CTRL_OSTRM); //clear timer value, clear OSTRM
    }    
    dw1000_write_reg(inst, EXT_SYNC_ID, EC_CTRL_OFFSET, reg, sizeof(uint16_t));
}



/**
 * API to calculate the SHR (Preamble + SFD) duration. This is used to calculate the correct rx_timeout.
 * @param attrib    Pointer to _phy_attributes_t * struct. The phy attritubes are part of the IEEE802.15.4-2011 standard. 
 * Note the morphology of the frame depends on the mode of operation, see the dw1000_hal.c for the default behaviour
 * @param nlen      The length of the frame to be transmitted/received excluding crc
 * @return uint16_t duration in usec
 */
inline uint16_t dw1000_phy_SHR_duration(struct _phy_attributes_t * attrib){

    uint16_t duration = ceilf(attrib->Tpsym * (attrib->nsync + attrib->nsfd));
    return duration; 
}

/**
 * API to calculate the frame duration (airtime). 
 * @param attrib    Pointer to _phy_attributes_t * struct. The phy attritubes are part of the IEEE802.15.4-2011 standard. 
 * Note the morphology of the frame depends on the mode of operation, see the dw1000_hal.c for the default behaviour
 * @param nlen      The length of the frame to be transmitted/received excluding crc
 * @return uint16_t duration in usec
 */
inline uint16_t dw1000_phy_frame_duration(struct _phy_attributes_t * attrib, uint16_t nlen){

    uint16_t duration = dw1000_phy_SHR_duration(attrib)  
            + ceilf(attrib->Tbsym * attrib->nphr + attrib->Tdsym * (nlen + 2) * 8);  // + 2 accounts for CRC
    return duration; 
}
