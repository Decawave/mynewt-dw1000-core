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
#include <dw1000/dw1000_gpio.h>
#include <dw1000/dw1000_otp.h>

static inline void _dw1000_phy_load_microcode(dw1000_dev_instance_t * inst);

// Set system clock to XTAL
inline void dw1000_phy_sysclk_XTAL(dw1000_dev_instance_t * inst){
 uint8_t reg = (uint8_t) dw1000_read_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, sizeof(uint8_t));
    reg &= (uint8_t)~PMSC_CTRL0_SYSCLKS_19M & (uint8_t)~PMSC_CTRL0_SYSCLKS_125M;
    reg |= (uint8_t) PMSC_CTRL0_SYSCLKS_19M;
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, reg, sizeof(uint8_t));
}

// Set system clock to PLL
inline void dw1000_phy_sysclk_PLL(dw1000_dev_instance_t * inst){
 uint8_t reg = (uint8_t) dw1000_read_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, sizeof(uint8_t));
    reg &= (uint8_t)~PMSC_CTRL0_SYSCLKS_19M & (uint8_t)~PMSC_CTRL0_SYSCLKS_125M;
    reg |= (uint8_t) PMSC_CTRL0_SYSCLKS_125M;
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, reg, sizeof(uint8_t));
}

// Set system clock to LDE
void dw1000_phy_sysclk_LDE(dw1000_dev_instance_t * inst){
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, 0x01, sizeof(uint8_t));
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET + 1 , 0x03, sizeof(uint8_t));
}

// Set system clock to SEQ All
inline void dw1000_phy_sysclk_SEQ(dw1000_dev_instance_t * inst){
    uint8_t reg = (uint8_t) dw1000_read_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, sizeof(uint8_t));
    reg &= (uint8_t)~PMSC_CTRL0_SYSCLKS_19M & (uint8_t)~PMSC_CTRL0_SYSCLKS_125M;
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, reg, sizeof(uint8_t));
}

// Set system clock to SEQ All
inline void dw1000_phy_sysclk_ACC(dw1000_dev_instance_t * inst, uint8_t mode){

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


// Disable sequencing and go to state "INIT"
void dw1000_phy_disable_sequencing(dw1000_dev_instance_t * inst){ 
    dw1000_phy_sysclk_XTAL(inst);
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL1_OFFSET, PMSC_CTRL1_PKTSEQ_DISABLE, sizeof(uint16_t)); // Disable PMSC ctrl of RF and RX clk blocks
}


dw1000_dev_status_t dw1000_phy_init(dw1000_dev_instance_t * inst, dw1000_phy_txrf_config_t * txrf_config){

    dw1000_softreset(inst);
    dw1000_gpio_config_leds(inst, DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    // Configure the CPLL lock detect
    dw1000_write_reg(inst, EXT_SYNC_ID, EC_CTRL_OFFSET, EC_CTRL_PLLLCK, sizeof(uint8_t));

    // Read OTP revision number
    uint32_t otp_addr = (uint32_t) _dw1000_otp_read(inst, OTP_XTRIM_ADDRESS) & 0xffff;    // Read 32 bit value, XTAL trim val is in low octet-0 (5 bits)
    inst->otp_rev = (otp_addr >> 8) & 0xff;                                               // OTP revision is next byte

    // Load LDO tune from OTP and kick it if there is a value actually programmed.
    uint32_t ldo_tune = _dw1000_otp_read(inst, OTP_LDOTUNE_ADDRESS);
    if((ldo_tune & 0xFF) != 0){
        dw1000_write_reg(inst, OTP_IF_ID, OTP_SF, OTP_SF_LDO_KICK, sizeof(uint8_t)); // Set load LDE kick bit
        inst->status.wakeup_LLDO = 1; // LDO tune must be kicked at wake-up
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

    uint8_t reg_val = (3 << 5) | (inst->xtal_trim & FS_XTALT_MASK);
    dw1000_write_reg(inst, FS_CTRL_ID, FS_XTALT_OFFSET, reg_val, sizeof(uint8_t));

    _dw1000_phy_load_microcode(inst);
    inst->status.wakeup_LLDE = 1;   // Microcode must be loaded at wake-up
    dw1000_phy_sysclk_SEQ(inst);    // Enable clocks for sequencing

    // The 3 bits in AON CFG1 register must be cleared to ensure proper operation of the DW1000 in DEEPSLEEP mode.
    dw1000_write_reg(inst, AON_ID, AON_CFG1_OFFSET, 0x0, sizeof(uint8_t));

    // Enable Temp & Vbat SAR onwake mode
    dw1000_write_reg(inst, AON_ID, AON_WCFG_OFFSET , AON_WCFG_ONW_RADC, sizeof(uint16_t));

    // Apply default antenna delay value. See NOTE 2 below. */
    dw1000_phy_set_rx_antennadelay(inst, inst->rx_antenna_delay);
    dw1000_phy_set_tx_antennadelay(inst, inst->tx_antenna_delay);

    // Apply tx power settings */
    if (txrf_config != NULL)  
        dw1000_phy_config_txrf(inst, txrf_config);

    // Read system register / store local copy
    inst->sys_cfg_reg = dw1000_read_reg(inst, SYS_CFG_ID, 0, sizeof(uint32_t)) ; // Read sysconfig register

    return inst->status;
}


void _dw1000_phy_load_microcode(dw1000_dev_instance_t * inst)
{
    // Set up clocks
    dw1000_phy_sysclk_LDE(inst);

    // Kick off the LDE Load
    dw1000_write_reg(inst, OTP_IF_ID, OTP_CTRL, OTP_CTRL_LDELOAD, sizeof(uint16_t)); // Set load LDE kick bit
    os_cputime_delay_usecs(120); // Allow time for code to upload (should take up to 120 us)
    dw1000_phy_sysclk_SEQ(inst); // Enable clocks for sequencing
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dw1000_config_lde()
 *
 * @brief configure LDE algorithm parameters
 *
 * input parameters
 * @param prf   -   this is the PRF index (0 or 1) 0 corresponds to 16 and 1 to 64 PRF
 *
 * output parameters
 *
 * no return value
 */
void dw1000_phy_config_lde(dw1000_dev_instance_t * inst, int prfIndex)
{
    dw1000_write_reg(inst, LDE_IF_ID, LDE_CFG1_OFFSET, LDE_PARAM1, sizeof(uint8_t)); // 8-bit configuration register

    if(prfIndex)
        dw1000_write_reg(inst, LDE_IF_ID, LDE_CFG2_OFFSET, LDE_PARAM3_64, sizeof(uint16_t)); // 16-bit LDE configuration tuning register
    else
        dw1000_write_reg(inst, LDE_IF_ID, LDE_CFG2_OFFSET, LDE_PARAM3_16, sizeof(uint16_t));
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_config_txrf()
 *
 * @brief This function provides the API for the configuration of the TX spectrum
 * including the power and pulse generator delay. The input is a pointer to the data structure
 * of type dwt_txconfig_t that holds all the configurable items.
 *
 * input parameters
 * @param config    -   pointer to the txrf configuration structure, which contains the tx rf config data
 *
 * output parameters`
 *
 * no return value
 */
void dw1000_phy_config_txrf(dw1000_dev_instance_t * inst, dw1000_phy_txrf_config_t *config)
{
    // Configure RF TX PG_DELAY
    dw1000_write_reg(inst, TX_CAL_ID, TC_PGDELAY_OFFSET, config->PGdly, sizeof(uint8_t));
    // Configure TX power
    dw1000_write_reg(inst, TX_POWER_ID, 0, config->power, sizeof(uint32_t));
}



/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_phy_read_wakeuptemp_SI(inst)
 *
 * @brief this function reads the temperature of the DW1000 that was sampled
 * on waking from Sleep/Deepsleep. They are not current values, but read on last
 * wakeup if DWT_TANDV bit is set in mode parameter of dwt_configuresleep
 *
 * input parameters:
 *
 * output parameters:
 *
 * returns: float value for temperature sensor in SI units (Degrees C)
 */
float dw1000_phy_read_wakeuptemp_SI(dw1000_dev_instance_t * inst)
{
   return 1.14 * (dw1000_phy_read_wakeuptemp(inst) - inst->otp_temp) + 23;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_phy_read_read_wakeupvbat_SI()
 *
 * @brief this function reads the battery voltage of the DW1000 that was sampled
 * on waking from Sleep/Deepsleep. They are not current values, but read on last
 * wakeup if DWT_TANDV bit is set in mode parameter of dwt_configuresleep
 *
 * input parameters:
 *
 * output parameters:
 *
 * returns: float battery voltage sensor in SI units (Volts)
 */
float dw1000_phy_read_read_wakeupvbat_SI(dw1000_dev_instance_t * inst)
{
    return (1.0/173) * (dw1000_phy_read_wakeupvbat(inst) - inst->otp_vbat) + 3.3;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_phy_rx_reset()
 *
 * @brief this function resets the receiver of the DW1000
 *
 * input parameters:
 *
 * output parameters
 *
 * no return value
 */
void dw1000_phy_rx_reset(dw1000_dev_instance_t * inst)
{
    // Set RX reset
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_RX, sizeof(uint8_t));
    // Clear RX reset
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_CLEAR, sizeof(uint8_t));
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn  dw1000_phy_forcetrxoff()
 *
 * @brief This is used to turn off the transceiver
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void dw1000_phy_forcetrxoff(dw1000_dev_instance_t * inst)
{
    uint32_t mask;

    mask = dw1000_read_reg(inst, SYS_MASK_ID, 0 , sizeof(uint32_t)) ; // Read set interrupt mask

    // Need to beware of interrupts occurring in the middle of following read modify write cycle
    // We can disable the radio, but before the status is cleared an interrupt can be set (e.g. the
    // event has just happened before the radio was disabled)
    // thus we need to disable interrupt during this operation

    os_error_t err = os_mutex_pend(&inst->mutex, OS_WAIT_FOREVER);
    assert(err == OS_OK);

    dw1000_write_reg(inst, SYS_MASK_ID, 0, 0, sizeof(uint32_t)) ; // Clear interrupt mask - so we don't get any unwanted events
    dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8_t)SYS_CTRL_TRXOFF, sizeof(uint8_t)) ; // Disable the radio
    // Forcing Transceiver off - so we do not want to see any new events that may have happened
    dw1000_write_reg(inst, SYS_STATUS_ID, 0, (SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_GOOD), sizeof(uint32_t));
    dw1000_sync_rxbufptrs(inst);
    dw1000_write_reg(inst, SYS_MASK_ID, 0, mask, sizeof(uint32_t)); // Restore mask to what it was
    
    // Enable/restore interrupts again...
    err = os_mutex_release(&inst->mutex);
    assert(err == OS_OK);

    inst->control.wait4resp_enabled = 0;

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn void dw1000_phy_interrupt_mask()
 *
 * @brief This function enables the specified events to trigger an interrupt.
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
 *
 * input parameters:
 * @param bitmask - sets the events which will generate interrupt
 * @param enable - if set the interrupts are enabled else they are cleared
 *
 * output parameters
 *
 * no return value
 */
void dw1000_phy_interrupt_mask(dw1000_dev_instance_t * inst, uint32_t bitmask, uint8_t enable)
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

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_phy_external_sync(dw1000_dev_instance_t * inst, uint8_t delay, bool enable)
 *
 * @brief This feature is used to synchronise DW1000 with external clocks or events or with other DW1000’s. 
 * For example, this would be required in a TDOA RTLS system employing wired clock synchronisation of the 
 * anchor nodes or AoA node for phase measurement.
 * 
 * @param inst    -  dw1000_dev_instance_t pointer
 * @param delay   -  To configure DW1000 for OSTR mode, the delay value is set to the desired delay value
 * @param enable  -  true/false 
 * 
 * no return value
 */
void dw1000_phy_external_sync(dw1000_dev_instance_t * inst, uint8_t delay, bool enable){

    uint16_t reg = dw1000_read_reg(inst, EXT_SYNC_ID, EC_CTRL_OFFSET, sizeof(uint16_t));
    if (enable) {
        reg &= EC_CTRL_WAIT_MASK; //clear timer value, clear OSTRM
        reg |= EC_CTRL_OSTRM;
        reg |= ((((uint16_t) delay) & 0xff) << 3); //set new timer value

    }else {
        reg &= EC_CTRL_WAIT_MASK ; //clear timer value, clear OSTRM
    }    
    dw1000_write_reg(inst, EXT_SYNC_ID, EC_CTRL_OFFSET, reg, sizeof(uint16_t));
}
