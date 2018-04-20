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
#include <math.h>
#include <os/os.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>

#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_mac.h>

static void dw1000_interrupt_task(void *arg);
static void dw1000_interrupt_ev_cb(struct os_event *ev);
static void dw1000_irq(void *arg);

#define NUM_BR 3
#define NUM_PRF 2
#define NUM_PACS 4
#define NUM_BW 2            //2 bandwidths are supported
#define NUM_SFD 2 

//-----------------------------------------
// map the channel number to the index in the configuration arrays below
// 0th element is chan 1, 1st is chan 2, 2nd is chan 3, 3rd is chan 4, 4th is chan 5, 5th is chan 7
const uint8_t chan_idx[] = {0, 0, 1, 2, 3, 4, 0, 5};

//-----------------------------------------
const uint32_t tx_config[] =
{
    RF_TXCTRL_CH1,
    RF_TXCTRL_CH2,
    RF_TXCTRL_CH3,
    RF_TXCTRL_CH4,
    RF_TXCTRL_CH5,
    RF_TXCTRL_CH7,
};

//Frequency Synthesiser - PLL configuration
const uint32_t fs_pll_cfg[] =
{
    FS_PLLCFG_CH1,
    FS_PLLCFG_CH2,
    FS_PLLCFG_CH3,
    FS_PLLCFG_CH4,
    FS_PLLCFG_CH5,
    FS_PLLCFG_CH7
};

//Frequency Synthesiser - PLL tuning
const uint8_t fs_pll_tune[] =
{
    FS_PLLTUNE_CH1,
    FS_PLLTUNE_CH2,
    FS_PLLTUNE_CH3,
    FS_PLLTUNE_CH4,
    FS_PLLTUNE_CH5,
    FS_PLLTUNE_CH7
};

//bandwidth configuration
const uint8_t rx_config[] =
{
    RF_RXCTRLH_NBW,
    RF_RXCTRLH_WBW
};

typedef struct {
    uint32_t lo32;
    uint16_t target[NUM_PRF];
} agc_cfg_struct ;


const agc_cfg_struct agc_config =
{
    AGC_TUNE2_VAL,
    { AGC_TUNE1_16M , AGC_TUNE1_64M }  //adc target
};

//DW non-standard SFD length for 110k, 850k and 6.81M
const uint8_t dwnsSFDlen[] =
{
    DW_NS_SFD_LEN_110K,
    DW_NS_SFD_LEN_850K,
    DW_NS_SFD_LEN_6M8
};

// SFD Threshold
const uint16_t sftsh[NUM_BR][NUM_SFD] =
{
    {
        DRX_TUNE0b_110K_STD,
        DRX_TUNE0b_110K_NSTD
    },
    {
        DRX_TUNE0b_850K_STD,
        DRX_TUNE0b_850K_NSTD
    },
    {
        DRX_TUNE0b_6M8_STD,
        DRX_TUNE0b_6M8_NSTD
    }
};

const uint16_t dtune1[] =
{
    DRX_TUNE1a_PRF16,
    DRX_TUNE1a_PRF64
};

const uint32_t digital_bb_config[NUM_PRF][NUM_PACS] =
{
    {
        DRX_TUNE2_PRF16_PAC8,
        DRX_TUNE2_PRF16_PAC16,
        DRX_TUNE2_PRF16_PAC32,
        DRX_TUNE2_PRF16_PAC64
    },
    {
        DRX_TUNE2_PRF64_PAC8,
        DRX_TUNE2_PRF64_PAC16,
        DRX_TUNE2_PRF64_PAC32,
        DRX_TUNE2_PRF64_PAC64
    }
};

const uint16_t lde_replicaCoeff[] =
{
    0, // No preamble code 0
    LDE_REPC_PCODE_1,
    LDE_REPC_PCODE_2,
    LDE_REPC_PCODE_3,
    LDE_REPC_PCODE_4,
    LDE_REPC_PCODE_5,
    LDE_REPC_PCODE_6,
    LDE_REPC_PCODE_7,
    LDE_REPC_PCODE_8,
    LDE_REPC_PCODE_9,
    LDE_REPC_PCODE_10,
    LDE_REPC_PCODE_11,
    LDE_REPC_PCODE_12,
    LDE_REPC_PCODE_13,
    LDE_REPC_PCODE_14,
    LDE_REPC_PCODE_15,
    LDE_REPC_PCODE_16,
    LDE_REPC_PCODE_17,
    LDE_REPC_PCODE_18,
    LDE_REPC_PCODE_19,
    LDE_REPC_PCODE_20,
    LDE_REPC_PCODE_21,
    LDE_REPC_PCODE_22,
    LDE_REPC_PCODE_23,
    LDE_REPC_PCODE_24
};

const double txpwr_compensation[] = {
    0.0,
    0.035,
    0.0,
    0.0,
    0.065,
    0.0
};


dw1000_dev_status_t dw1000_mac_init(dw1000_dev_instance_t * inst, dwt_config_t *config)
{
    uint8_t nsSfd_result  = 0;
    uint8_t useDWnsSFD = 0;
    uint8_t chan = config->chan ;
    uint8_t prfIndex = config->prf - DWT_PRF_16M;
    uint8_t bw = ((chan == 4) || (chan == 7)) ? 1 : 0 ; // Select wide or narrow band
    uint16_t reg16 = lde_replicaCoeff[config->rxCode];

#ifdef DW1000_API_ERROR_CHECK
    assert(config->dataRate <= DWT_BR_6M8);
    assert(config->rxPAC <= DWT_PAC64);
    assert((chan >= 1) && (chan <= 7) && (chan != 6));
    assert(((config->prf == DWT_PRF_64M) && (config->txCode >= 9) && (config->txCode <= 24))
           || ((config->prf == DWT_PRF_16M) && (config->txCode >= 1) && (config->txCode <= 8)));
    assert(((config->prf == DWT_PRF_64M) && (config->rxCode >= 9) && (config->rxCode <= 24))
           || ((config->prf == DWT_PRF_16M) && (config->rxCode >= 1) && (config->rxCode <= 8)));
    assert((config->txPreambLength == DWT_PLEN_64) || (config->txPreambLength == DWT_PLEN_128) || (config->txPreambLength == DWT_PLEN_256)
           || (config->txPreambLength == DWT_PLEN_512) || (config->txPreambLength == DWT_PLEN_1024) || (config->txPreambLength == DWT_PLEN_1536)
           || (config->txPreambLength == DWT_PLEN_2048) || (config->txPreambLength == DWT_PLEN_4096));
    assert((config->phrMode == DWT_PHRMODE_STD) || (config->phrMode == DWT_PHRMODE_EXT));
#endif

    /* Keep a copy of the mac_config */
    memcpy(&inst->mac_config, config, sizeof(dwt_config_t));
    
    // For 110 kbps we need a special setup
    if(config->dataRate == DWT_BR_110K){
        inst->sys_cfg_reg |= SYS_CFG_RXM110K;
        reg16 >>= 3; // lde_replicaCoeff must be divided by 8
    }else{
        inst->sys_cfg_reg &= (~SYS_CFG_RXM110K);
    }

    inst->longFrames = config->phrMode;
    inst->sys_cfg_reg &= ~SYS_CFG_PHR_MODE_11;
    inst->sys_cfg_reg |= (SYS_CFG_PHR_MODE_11 & (config->phrMode << SYS_CFG_PHR_MODE_SHFT));
    if (inst->config.rxauto_enable) 
        inst->sys_cfg_reg |=SYS_CFG_RXAUTR; 

    dw1000_write_reg(inst, SYS_CFG_ID, 0, inst->sys_cfg_reg, sizeof(uint32_t));
    dw1000_write_reg(inst, LDE_IF_ID, LDE_REPC_OFFSET, reg16, sizeof(uint16_t)); // Set the lde_replicaCoeff 

    dw1000_phy_config_lde(inst, prfIndex);

    // Configure PLL2/RF PLL block CFG/TUNE (for a given channel)
    dw1000_write_reg(inst, FS_CTRL_ID, FS_PLLCFG_OFFSET, fs_pll_cfg[chan_idx[chan]], sizeof(uint32_t));
    dw1000_write_reg(inst, FS_CTRL_ID, FS_PLLTUNE_OFFSET, fs_pll_tune[chan_idx[chan]], sizeof(uint8_t));

    // Configure RF RX blocks (for specified channel/bandwidth)
    dw1000_write_reg(inst, RF_CONF_ID, RF_RXCTRLH_OFFSET, rx_config[bw], sizeof(uint8_t));

    // Configure RF TX blocks (for specified channel and PRF)
    // Configure RF TX control
    dw1000_write_reg(inst, RF_CONF_ID, RF_TXCTRL_OFFSET, tx_config[chan_idx[chan]], sizeof(uint32_t));

    // Configure the baseband parameters (for specified PRF, bit rate, PAC, and SFD settings)
    // DTUNE0
    dw1000_write_reg(inst, DRX_CONF_ID, DRX_TUNE0b_OFFSET, sftsh[config->dataRate][config->nsSFD], sizeof(uint16_t));
    // DTUNE1
    dw1000_write_reg(inst, DRX_CONF_ID, DRX_TUNE1a_OFFSET, dtune1[prfIndex], sizeof(uint16_t));

    if(config->dataRate == DWT_BR_110K){
        dw1000_write_reg(inst, DRX_CONF_ID, DRX_TUNE1b_OFFSET, DRX_TUNE1b_110K, sizeof(uint16_t));
    }else{
        if(config->txPreambLength == DWT_PLEN_64){
            dw1000_write_reg(inst, DRX_CONF_ID, DRX_TUNE1b_OFFSET, DRX_TUNE1b_6M8_PRE64, sizeof(uint16_t));
            dw1000_write_reg(inst, DRX_CONF_ID, DRX_TUNE4H_OFFSET, DRX_TUNE4H_PRE64, sizeof(uint16_t));
        }else{
            dw1000_write_reg(inst, DRX_CONF_ID, DRX_TUNE1b_OFFSET, DRX_TUNE1b_850K_6M8, sizeof(uint16_t));
            dw1000_write_reg(inst, DRX_CONF_ID, DRX_TUNE4H_OFFSET, DRX_TUNE4H_PRE128PLUS, sizeof(uint16_t));
        }
    }

    // DTUNE2
    dw1000_write_reg(inst, DRX_CONF_ID, DRX_TUNE2_OFFSET, digital_bb_config[prfIndex][config->rxPAC], sizeof(uint16_t));

    // DTUNE3 (SFD timeout)
    // Don't allow 0 - SFD timeout will always be enabled
    if(config->sfdTO == 0)
        config->sfdTO = DWT_SFDTOC_DEF;
    
    dw1000_write_reg(inst, DRX_CONF_ID, DRX_SFDTOC_OFFSET, config->sfdTO, sizeof(uint16_t));

    // Configure AGC parameters
    dw1000_write_reg(inst, AGC_CTRL_ID, AGC_TUNE2_OFFSET, agc_config.lo32, sizeof(uint32_t));
    dw1000_write_reg(inst, AGC_CTRL_ID, AGC_TUNE1_OFFSET, agc_config.target[prfIndex], sizeof(uint32_t));

    // Set (non-standard) user SFD for improved performance,
    if(config->nsSFD){
        // Write non standard (DW) SFD length
        dw1000_write_reg(inst, USR_SFD_ID, 0x0, dwnsSFDlen[config->dataRate], sizeof(uint8_t));
        nsSfd_result = 3 ;
        useDWnsSFD = 1 ;
    }
    uint32_t regval =  (CHAN_CTRL_TX_CHAN_MASK & (chan << CHAN_CTRL_TX_CHAN_SHIFT)) | // Transmit Channel
              (CHAN_CTRL_RX_CHAN_MASK & (chan << CHAN_CTRL_RX_CHAN_SHIFT)) | // Receive Channel
              (CHAN_CTRL_RXFPRF_MASK & (config->prf << CHAN_CTRL_RXFPRF_SHIFT)) | // RX PRF
              ((CHAN_CTRL_TNSSFD|CHAN_CTRL_RNSSFD) & (nsSfd_result << CHAN_CTRL_TNSSFD_SHIFT)) | // nsSFD enable RX&TX
              (CHAN_CTRL_DWSFD & (useDWnsSFD << CHAN_CTRL_DWSFD_SHIFT)) | // Use DW nsSFD
              (CHAN_CTRL_TX_PCOD_MASK & (config->txCode << CHAN_CTRL_TX_PCOD_SHIFT)) | // TX Preamble Code
              (CHAN_CTRL_RX_PCOD_MASK & (config->rxCode << CHAN_CTRL_RX_PCOD_SHIFT)) ; // RX Preamble Code

    dw1000_write_reg(inst, CHAN_CTRL_ID, 0, regval, sizeof(uint32_t)) ;

    // Set up TX Preamble Size, PRF and Data Rate
    inst->tx_fctrl = ((config->txPreambLength | config->prf) << TX_FCTRL_TXPRF_SHFT) | (config->dataRate << TX_FCTRL_TXBR_SHFT);
    dw1000_write_reg(inst, TX_FCTRL_ID, 0, inst->tx_fctrl, sizeof(uint32_t));
    // The SFD transmit pattern is initialised by the DW1000 upon a user TX request, but (due to an IC issue) it is not done for an auto-ACK TX. The
    // SYS_CTRL write below works around this issue, by simultaneously initiating and aborting a transmission, which correctly initialises the SFD
    // after its configuration or reconfiguration.
    // This issue is not documented at the time of writing this code. It should be in next release of DW1000 User Manual (v2.09, from July 2016).
    dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, SYS_CTRL_TXSTRT | SYS_CTRL_TRXOFF, sizeof(uint8_t)); // Request TX start and TRX off at the same time
    dw1000_tasks_init(inst);

    return inst->status;
} 


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_write_tx()
 *
 * @brief This API function writes the supplied TX data into the DW1000's
 * TX buffer.  The input parameters are the data length in bytes and a pointer
 * to those data bytes.
 *
 * input parameters
 * @param txFrameLength  - This is the total frame length, including the two byte CRC.
 *                         Note: this is the length of TX message (including the 2 byte CRC) - max is 1023
 *                         standard PHR mode allows up to 127 bytes
 *                         if > 127 is programmed, DWT_PHRMODE_EXT needs to be set in the phrMode configuration
 *                         see dwt_configure function
 * @param txFrameBytes   - Pointer to the user�s buffer containing the data to send.
 * @param txBufferOffset - This specifies an offset in the DW1000�s TX Buffer at which to start writing data.
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */

dw1000_dev_status_t dw1000_write_tx(dw1000_dev_instance_t * inst,  uint8_t * txFrameBytes, uint16_t txBufferOffset, uint16_t txFrameLength)
{
#ifdef DW1000_API_ERROR_CHECK
    assert((pdw1000local->longFrames && (txFrameLength <= 1023)) || (txFrameLength <= 127));
    assert((txBufferOffset + txFrameLength) <= 1024);
#endif

    os_error_t err = os_sem_pend(&inst->sem,  OS_TIMEOUT_NEVER); // Released by a SYS_STATUS_TXFRS event
    assert(err == OS_OK);

    if ((txBufferOffset + txFrameLength) <= 1024){
        dw1000_write(inst, TX_BUFFER_ID, txBufferOffset,  txFrameBytes, txFrameLength);
        for (uint8_t i = 0; i< sizeof(inst->fctrl); i++)
            inst->fctrl_array[i] =  txFrameBytes[i];
        inst->status.tx_frame_error = 0;
    }
    else
        inst->status.tx_frame_error = 1;

    err = os_sem_release(&inst->sem); 
    assert(err == OS_OK); 
    
    return inst->status;
}

/* @fn dw1000_write_tx_fctrl()
 *
 * @brief This API function configures the TX frame control register before the transmission of a frame
 *
 * input parameters:
 * @param txFrameLength - this is the length of TX message (excluding the 2 byte CRC) - max is 1023
 *                              NOTE: standard PHR mode allows up to 127 bytes
 *                              if > 127 is programmed, DWT_PHRMODE_EXT needs to be set in the phrMode configuration
 *                              see dwt_configure function
 * @param txBufferOffset - the offset in the tx buffer to start writing the data
 * @param ranging - 1 if this is a ranging frame, else 0
 *
 * output parameters
 *
 * no return value
 */
inline void dw1000_write_tx_fctrl(dw1000_dev_instance_t * inst, uint16_t txFrameLength, uint16_t txBufferOffset, bool ranging)
{
#ifdef DW1000_API_ERROR_CHECK
    assert((inst->longFrames && ((txFrameLength + 2) <= 1023)) || ((txFrameLength +2) <= 127));
#endif

    os_error_t err = os_sem_pend(&inst->sem,  OS_TIMEOUT_NEVER); // Released by a SYS_STATUS_TXFRS event
    assert(err == OS_OK);

    // Write the frame length to the TX frame control register
    uint32_t tx_fctrl_reg = inst->tx_fctrl | (txFrameLength + 2)  | (txBufferOffset << TX_FCTRL_TXBOFFS_SHFT) | ((ranging)?(TX_FCTRL_TR):0);
    inst->status.tx_ranging_frame = ranging;
    dw1000_write_reg(inst, TX_FCTRL_ID, 0, tx_fctrl_reg, sizeof(uint32_t));
 
    err = os_sem_release(&inst->sem); 
    assert(err == OS_OK);  
} 

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_start_tx()
 *
 * @brief This call initiates the transmission, input parameter indicates which TX mode is used see below
 *
 */
dw1000_dev_status_t dw1000_start_tx(dw1000_dev_instance_t * inst)
{
    os_error_t err = os_sem_pend(&inst->sem,  OS_TIMEOUT_NEVER); // Released by a SYS_STATUS_TXFRS event
    assert(err == OS_OK);

    inst->status.rx_error = inst->status.rx_timeout_error = 0;

    if (inst->control.wait4resp_enabled) // Undocumented ANONMALY::This should not be required
        dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8_t)SYS_CTRL_WAIT4RESP, sizeof(uint8_t));
        
    inst->sys_ctrl_reg = SYS_CTRL_TXSTRT;
    if (inst->control.wait4resp_enabled)
        inst->sys_ctrl_reg |= SYS_CTRL_WAIT4RESP; 
    if (inst->control.delay_start_enabled)
        inst->sys_ctrl_reg |= SYS_CTRL_TXDLYS; 

    if (inst->control.delay_start_enabled){
        dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8_t) inst->sys_ctrl_reg, sizeof(uint8_t));
        uint16_t sys_status_reg = dw1000_read_reg(inst, SYS_STATUS_ID, 3, sizeof(uint16_t)); // Read at offset 3 to get the upper 2 bytes out of 5
        inst->status.start_tx_error = (sys_status_reg & ((SYS_STATUS_HPDWARN | SYS_STATUS_TXPUTE) >> 24)) != 0;
        if (inst->status.start_tx_error){
            /*
            * Half Period Delay Warning (HPDWARN) OR Power Up error (TXPUTE). This event status bit relates to the 
            * use of delayed transmit and delayed receive functionality. It indicates the delay is more than half 
            * a period of the system clock. There is enough time to send but not to power up individual blocks.
            * Typically when the HPDWARN event is detected the host controller will abort the delayed TX/RX by issuing 
            * a TRXOFF transceiver off command and then take whatever remedial action is deemed appropriate for the application.
            * Remedial action is cancle send and report error
            */
            inst->sys_ctrl_reg = SYS_CTRL_TRXOFF; // This assumes the bit is in the lowest byte
            dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8_t) inst->sys_ctrl_reg, sizeof(uint8_t));
            os_sem_release(&inst->sem); 
        }
    }else{
        dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, inst->sys_ctrl_reg, sizeof(uint8_t));
        inst->status.start_tx_error = 0;
    }

    inst->control_tx_context = inst->control;
    inst->control = (dw1000_dev_control_t){
        .wait4resp_enabled=0,
        .wait4resp_delay_enabled=0,
        .delay_start_enabled=0,
        .autoack_delay_enabled=0,
        .start_rx_syncbuf_enabled=0,
        .rx_timeout_enabled=0
    };
    return inst->status;
} 


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_set_delayed_start()
 *
 * 
 * input parameters
 * @param delay -  The Delayed Send or Receive Time, is used to specify a time in the future to either turn on the 
 * receiver to be ready to receive a frame, or to turn on the transmitter and send a frame. The low-order 9-bits of 
 * this register are ignored. The delay is in UWB microseconds. 
 * 
 * output parameters
 */
inline dw1000_dev_status_t dw1000_set_delay_start(dw1000_dev_instance_t * inst, uint64_t delay)
{
    os_error_t err = os_sem_pend(&inst->sem,  OS_TIMEOUT_NEVER); // Released by a SYS_STATUS_TXFRS event
    assert(err == OS_OK);

    inst->control.delay_start_enabled = (delay >> 8) > 0;

    if (inst->control.delay_start_enabled)
         dw1000_write_reg(inst, DX_TIME_ID, 1, delay >> 8, DX_TIME_LEN-1);

    err = os_sem_release(&inst->sem); 
    assert(err == OS_OK); 
    return inst->status;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_start_rx()
 *
 * @brief 
 *
 * input parameters
 * 
 */

dw1000_dev_status_t dw1000_start_rx(dw1000_dev_instance_t * inst)
{
    inst->status.rx_error = inst->status.rx_timeout_error = 0;

    inst->sys_ctrl_reg = SYS_CTRL_RXENAB;
    if (inst->control.start_rx_syncbuf_enabled)
        dw1000_sync_rxbufptrs(inst);
    if (inst->control.delay_start_enabled) 
        inst->sys_ctrl_reg |= SYS_CTRL_RXDLYE;

    os_error_t err = os_sem_pend(&inst->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, inst->sys_ctrl_reg, sizeof(uint16_t));
    if (inst->control.delay_start_enabled){ // check for errors
        uint8_t sys_status_reg = dw1000_read_reg(inst, SYS_STATUS_ID, 3, sizeof(uint8_t));  // Read 1 byte at offset 3 to get the 4th byte out of 5
        inst->status.start_rx_error = (sys_status_reg & (SYS_STATUS_HPDWARN >> 24)) != 0;   
        if (inst->status.start_rx_error){   // if delay has passed do immediate RX on unless DWT_IDLE_ON_DLY_ERR is true
            dw1000_phy_forcetrxoff(inst);   // turn the delayed receive off
            dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, inst->sys_ctrl_reg, sizeof(uint16_t)); // turn on receiver        
        }
    }else
        inst->status.start_rx_error = 0;

    inst->control_rx_context = inst->control;
    inst->control = (dw1000_dev_control_t){
        .wait4resp_enabled=0,
        .wait4resp_delay_enabled=0,
        .delay_start_enabled=0,
        .autoack_delay_enabled=0,
        .start_rx_syncbuf_enabled=0,
        .rx_timeout_enabled=0
    };

    err = os_sem_release(&inst->sem); 
    assert(err == OS_OK); 
    return inst->status;
} 


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_restart_rx()
 *
 * @brief This function attempts to restore the transceiver to the state prior 
 * to the recent interrupt. Restart_rx  differs from start_rx insofar as errors 
 * are not cleared. 
 *
 * input parameters
 * 
 */
dw1000_dev_status_t dw1000_restart_rx(dw1000_dev_instance_t * inst, dw1000_dev_control_t control)
{
    uint32_t sys_ctrl_reg = SYS_CTRL_RXENAB;
    if (control.start_rx_syncbuf_enabled)
        dw1000_sync_rxbufptrs(inst);
    if (control.delay_start_enabled) 
        sys_ctrl_reg |= SYS_CTRL_RXDLYE;

    os_error_t err = os_sem_pend(&inst->sem,  OS_TIMEOUT_NEVER); // Block if request pending
    assert(err == OS_OK);

    dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, sys_ctrl_reg, sizeof(uint16_t));
    if (control.delay_start_enabled){ // check for errors
        uint8_t sys_status_reg = dw1000_read_reg(inst, SYS_STATUS_ID, 3, sizeof(uint8_t));  // Read 1 byte at offset 3 to get the 4th byte out of 5
        inst->status.start_rx_error = (sys_status_reg & (SYS_STATUS_HPDWARN >> 24)) != 0;   
        if (inst->status.start_rx_error){   // if delay has passed do immediate RX on unless DWT_IDLE_ON_DLY_ERR is true
            dw1000_phy_forcetrxoff(inst);   // turn the delayed receive off
            dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, inst->sys_ctrl_reg, sizeof(uint16_t)); // turn on receiver
        }
    }else
        inst->status.start_rx_error = 0;

    err = os_sem_release(&inst->sem); 
    assert(err == OS_OK); 

    return inst->status;
} 

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_set_wait4resp()
 *
 * @brief enable wait4for feature
 * 
 * output parameters
 *
 */
inline dw1000_dev_status_t dw1000_set_wait4resp(dw1000_dev_instance_t * inst, bool enable)
{
    inst->control.wait4resp_enabled = enable;
    return inst->status;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_set_rx_timeout()
 *
 * @brief The Receive Frame Wait Timeout period is a 16-bit field. The units for this parameter are roughly 1μs, 
 * (the exact unit is 512counts of the fundamental 499.2 MHz UWB clock, or 1.026 μs). When employing the frame wait timeout, 
 * RXFWTO should be set to a value greater than the expected RX frame duration and include an allowance for any uncertainly 
 * attaching to the expected transmission start time of the awaited frame. 
 *
 * input parameters
 * @param timeout - how long the receiver remains on from the RX enable command The time parameter used here is in 1.0256 
 *                  us (512/499.2MHz) units If set to 0 the timeout is disabled.
 *
 */

dw1000_dev_status_t dw1000_set_rx_timeout(dw1000_dev_instance_t * inst, uint16_t timeout)
{

    os_error_t err = os_sem_pend(&inst->sem,  OS_TIMEOUT_NEVER); // Block if request pending
    assert(err == OS_OK);
    inst->status.rx_timeout_error = 0;

    err = os_mutex_pend(&inst->mutex, OS_WAIT_FOREVER); // Read modify write critical section enter 
    assert(err == OS_OK);

    inst->sys_cfg_reg = dw1000_read_reg(inst, SYS_CFG_ID, 0, sizeof(uint32_t));  
    inst->control.rx_timeout_enabled = timeout > 0;
    if(inst->control.rx_timeout_enabled){  
        dw1000_write_reg(inst, RX_FWTO_ID, RX_FWTO_OFFSET, timeout, sizeof(uint16_t));
        inst->sys_cfg_reg |= SYS_CFG_RXWTOE;
        dw1000_write_reg(inst, SYS_CFG_ID, 0, inst->sys_cfg_reg, sizeof(uint32_t));
    }else{
        inst->sys_cfg_reg &= ~SYS_CFG_RXWTOE;
        dw1000_write_reg(inst, SYS_CFG_ID, 0, inst->sys_cfg_reg, sizeof(uint32_t));
    }
          
    err = os_mutex_release(&inst->mutex);       // // Read modify write critical section leave
    assert(err == OS_OK);
    err = os_sem_release(&inst->sem);  
    assert(err == OS_OK);

    return inst->status;
} 

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_sync_rxbufptrs()
 *
 * @brief this function synchronizes rx buffer pointers
 * need to make sure that the host/IC buffer pointers are aligned before starting RX
 *
 * input parameters:
 *
 * output parameters
 *
 * no return value
 */
dw1000_dev_status_t dw1000_sync_rxbufptrs(dw1000_dev_instance_t * inst)
{
    uint8_t  buff;
    os_error_t err = os_sem_pend(&inst->sem,  OS_TIMEOUT_NEVER); // Block if request pending
    assert(err == OS_OK);
    
    inst->control.start_rx_syncbuf_enabled = 1;
    // Need to make sure that the host/IC buffer pointers are aligned before starting RX
    buff = dw1000_read_reg(inst, SYS_STATUS_ID, 3, sizeof(uint8_t)); // Read 1 byte at offset 3 to get the 4th byte out of 5
    
    if((buff & (SYS_STATUS_ICRBP >> 24)) !=     // IC side Receive Buffer Pointer
       ((buff & (SYS_STATUS_HSRBP>>24)) << 1) ) // Host Side Receive Buffer Pointer
        dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET , 0x01, sizeof(uint8_t)) ; // We need to swap RX buffer status reg (write one to toggle internally)
    
    err = os_sem_release(&inst->sem);  
    assert(err == OS_OK);
    return inst->status;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_read_accdata()
 *
 * @brief This is used to read the data from the Accumulator buffer, from an offset location give by offset parameter
 *
 * NOTE: Because of an internal memory access delay when reading the accumulator the first octet output is a dummy octet
 *       that should be discarded. This is true no matter what sub-index the read begins at.
 *
 * input parameters
 * @param buffer - the buffer into which the data will be read
 * @param length - the length of data to read (in bytes)
 * @param accOffset - the offset in the acc buffer from which to read the data
 *
 * output parameters
 *
 */
dw1000_dev_status_t dw1000_read_accdata(dw1000_dev_instance_t * inst, uint8_t *buffer, uint16_t accOffset, uint16_t len)
{
    os_error_t err = os_sem_pend(&inst->sem,  OS_TIMEOUT_NEVER); // Block if request pending
    assert(err == OS_OK);

    // Force on the ACC clocks if we are sequenced
    dw1000_phy_sysclk_ACC(inst, true);
    dw1000_read(inst, ACC_MEM_ID, accOffset, buffer, len) ;
    dw1000_phy_sysclk_ACC(inst, false);
    
    err = os_sem_release(&inst->sem);  
    assert(err == OS_OK);
    return inst->status;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_mac_framefilter()
 *
 * @brief This is used to enable the frame filtering - (the default option is to
 * accept any data and ACK frames with correct destination address
 *
 * input parameters
 * @param - bitmask - enables/disables the frame filtering options according to
 *      DWT_FF_NOTYPE_EN        0x000   no frame types allowed
 *      DWT_FF_COORD_EN         0x002   behave as coordinator (can receive frames with no destination address (PAN ID has to match))
 *      DWT_FF_BEACON_EN        0x004   beacon frames allowed
 *      DWT_FF_DATA_EN          0x008   data frames allowed
 *      DWT_FF_ACK_EN           0x010   ack frames allowed
 *      DWT_FF_MAC_EN           0x020   mac control frames allowed
 *      DWT_FF_RSVD_EN          0x040   reserved frame types allowed
 *
 * output parameters
 *
 */

dw1000_dev_status_t dw1000_mac_framefilter(dw1000_dev_instance_t * inst, uint16_t enable)
{
    os_error_t err = os_sem_pend(&inst->sem,  OS_TIMEOUT_NEVER); // Block if request pending
    assert(err == OS_OK);

    inst->sys_cfg_reg = SYS_CFG_MASK & dw1000_read_reg(inst, SYS_CFG_ID, 0, sizeof(uint32_t)) ; // Read sysconfig register

    inst->config.framefilter_enabled = enable > 0;
    if(inst->config.framefilter_enabled){   // Enable frame filtering and configure frame types
        inst->sys_cfg_reg &= ~(SYS_CFG_FF_ALL_EN);  // Clear all
        inst->sys_cfg_reg |= (enable & SYS_CFG_FF_ALL_EN) | SYS_CFG_FFE;
    }else
        inst->sys_cfg_reg &= ~(SYS_CFG_FFE);

    dw1000_write_reg(inst, SYS_CFG_ID,0, inst->sys_cfg_reg, sizeof(uint32_t)); 
    err = os_sem_release(&inst->sem);  
    assert(err == OS_OK);

    return inst->status;
}



/*! ------------------------------------------------------------------------------------------------------------------
 * @fn  dw1000_set_autoack()
 *
 * @brief This call enables the auto-ACK feature. If the delay (parameter) is 0, the ACK will be sent as-soon-as-possable
 * otherwise it will be sent with a programmed delay (in symbols), max is 255.
 * NOTE: needs to have frame filtering enabled as well
 *
 * input parameters
 * @param responseDelayTime - if non-zero the ACK is sent after this delay, max is 255.
 *
 * output parameters
 *
 */
dw1000_dev_status_t dw1000_set_autoack(dw1000_dev_instance_t * inst, bool enable)
{
    assert(inst->config.framefilter_enabled);

    os_error_t err = os_sem_pend(&inst->sem,  OS_TIMEOUT_NEVER); // Block if request pending
    assert(err == OS_OK);

    err = os_mutex_pend(&inst->mutex, OS_WAIT_FOREVER); // Read modify write critical section enter
    assert(err == OS_OK);


    if (inst->config.framefilter_enabled == 0) // The assumption is that this is never called
        inst->sys_cfg_reg = SYS_CFG_MASK & dw1000_read_reg(inst, SYS_CFG_ID, 0, sizeof(uint32_t)); // Read sysconfig register

    inst->config.autoack_enabled = enable > 0;    
    if(inst->config.autoack_enabled){
        inst->sys_cfg_reg |= SYS_CFG_AUTOACK;
        dw1000_write_reg(inst, SYS_CFG_ID,0, inst->sys_cfg_reg, sizeof(uint32_t));
    }else{
        inst->sys_cfg_reg &= ~SYS_CFG_AUTOACK;
        dw1000_write_reg(inst, SYS_CFG_ID,0, inst->sys_cfg_reg, sizeof(uint32_t));
    }

    err = os_mutex_release(&inst->mutex);       // // Read modify write critical section exit
    assert(err == OS_OK);
    err = os_sem_release(&inst->sem);  
    assert(err == OS_OK);
    return inst->status;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn  dw1000_set_autoack_delay()
 *
 * @brief This call enables the auto-ACK feature. If the delay (parameter) is 0, the ACK will be sent as-soon-as-possable
 * otherwise it will be sent with a programmed delay (in symbols), max is 255.
 * NOTE: needs to have frame filtering enabled as well
 *
 * input parameters
 * @param responseDelayTime - if non-zero the ACK is sent after this delay, max is 255.
 *
 * output parameters
 *
 */
dw1000_dev_status_t dw1000_set_autoack_delay(dw1000_dev_instance_t * inst, uint8_t delay)
{
    assert(inst->config.framefilter_enabled);

    os_error_t err = os_sem_pend(&inst->sem,  OS_TIMEOUT_NEVER); // Block if request pending
    assert(err == OS_OK);

    if (inst->config.framefilter_enabled == 0) // This is here for completness but should never execute because of the assert above
        inst->sys_cfg_reg = SYS_CFG_MASK & dw1000_read_reg(inst, SYS_CFG_ID, 0, sizeof(uint32_t)); // Read sysconfig register

    inst->config.autoack_delay_enabled = delay > 0;
    if (inst->control.autoack_delay_enabled)
        dw1000_write_reg(inst, ACK_RESP_T_ID, ACK_RESP_T_ACK_TIM_OFFSET, delay, sizeof(uint8_t)); // In symbols

    err = os_sem_release(&inst->sem);  
    assert(err == OS_OK);

    dw1000_set_autoack(inst, true);

    return inst->status;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_set_wait4resp_delay()  
 *
 * @brief Wait-for-Response turn-around Time. This 20-bit field is used to configure the turn-around time between TX complete 
 * and RX enable when the wait for response function is being used. This function is enabled by the WAIT4RESP control in 
 * Register file: 0x0D – System Control Register. The time specified by this W4R_TIM parameter is in units of approximately 1 μs, 
 * or 128 system clock cycles. This configuration may be used to save power by delaying the turn-on of the receiver, 
 * to align with the response time of the remote system, rather than turning on the receiver immediately after transmission completes.
 * 
 * input parameters
 * @param delay - (20 bits) - the delay is in UWB microseconds
 *
 * output parameters
 *
 */
dw1000_dev_status_t dw1000_set_wait4resp_delay(dw1000_dev_instance_t * inst, uint32_t delay)
{
    os_error_t err = os_sem_pend(&inst->sem,  OS_TIMEOUT_NEVER); // Block if request pending
    assert(err == OS_OK);

    inst->control.wait4resp_delay_enabled = delay > 0;
    if (inst->control.wait4resp_delay_enabled){

        err = os_mutex_pend(&inst->mutex, OS_WAIT_FOREVER); // Read modify write critical section enter
        assert(err == OS_OK);

        uint32_t ack_resp_reg = dw1000_read_reg(inst, ACK_RESP_T_ID, 0, sizeof(uint32_t)) ; // Read ACK_RESP_T_ID register
        ack_resp_reg &= ~(ACK_RESP_T_W4R_TIM_MASK) ;        // Clear the timer (19:0)
        ack_resp_reg |= (delay & ACK_RESP_T_W4R_TIM_MASK) ; // In UWB microseconds (e.g. turn the receiver on 20uus after TX)
        dw1000_write_reg(inst, ACK_RESP_T_ID, 0, ack_resp_reg, sizeof(uint32_t));
        
        err = os_mutex_release(&inst->mutex);       // // Read modify write critical section exit
        assert(err == OS_OK);
    }
    err = os_sem_release(&inst->sem);  
    assert(err == OS_OK);
    return inst->status;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_set_dblrxbuff()
 *
 * @brief This call enables the double receive buffer mode
 *
 * input parameters
 * @param enable - 1 to enable, 0 to disable the double buffer mode
 *
 * output parameters
 *
 */
dw1000_dev_status_t dw1000_set_dblrxbuff(dw1000_dev_instance_t * inst, bool enable)
{
    os_error_t err = os_sem_pend(&inst->sem,  OS_TIMEOUT_NEVER); // Block if request pending
    assert(err == OS_OK);
    err = os_mutex_pend(&inst->mutex, OS_WAIT_FOREVER); // Read modify write critical section enter
    assert(err == OS_OK);

    inst->sys_cfg_reg = SYS_CFG_MASK & dw1000_read_reg(inst, SYS_CFG_ID, 0, sizeof(uint32_t)); 

    inst->config.dblbuffon_enabled = enable;
    if(inst->config.dblbuffon_enabled)
        inst->sys_cfg_reg &= ~SYS_CFG_DIS_DRXB;
    else
        inst->sys_cfg_reg |= SYS_CFG_DIS_DRXB;
    dw1000_write_reg(inst, SYS_CFG_ID, 0, inst->sys_cfg_reg, sizeof(uint32_t));
    
    err = os_mutex_release(&inst->mutex);       // // Read modify write critical section exit
    assert(err == OS_OK);
    err = os_sem_release(&inst->sem);  
    assert(err == OS_OK);

    return inst->status;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_read_rxdiag()
 *
 * @brief this function reads the RX signal quality diagnostic data
 *
 * input parameters
 * @param diagnostics - diagnostic structure pointer, this will contain the diagnostic data read from the DW1000
 *
 * output parameters
 *
 * no return value
 */
void dw1000_read_rxdiag(dw1000_dev_instance_t * inst, dw1000_dev_rxdiag_t * diag)
{  
    // Read the HW FP index
    diag->fp_idx = dw1000_read_reg(inst, RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET, sizeof(uint16_t));
    diag->fp_amp = dw1000_read_reg(inst, RX_TIME_ID, RX_TIME_FP_AMPL1_OFFSET, sizeof(uint16_t));
    diag->rx_std  = dw1000_read_reg(inst, RX_FQUAL_ID, 0, sizeof(uint16_t));
    diag->fp_amp2 = dw1000_read_reg(inst, RX_FQUAL_ID, 2, sizeof(uint16_t));
    diag->fp_amp3 = dw1000_read_reg(inst, RX_FQUAL_ID, 4, sizeof(uint16_t));
    diag->max_growth_cir = dw1000_read_reg(inst, RX_FQUAL_ID, 6, sizeof(uint16_t));
    diag->preamble_cnt =  (dw1000_read_reg(inst, RX_FINFO_ID, 0, sizeof(uint32_t)) & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_mac_tasks_init()
 *
 * @brief The DW1000 processing of interrupts in a task context instead of the interrupt context such that other interrupts 
 * and high priority tasks are not blocked waiting for the interrupt handler to complete processing. 
 * This dw1000 softstack needs to coexists with other stacks and sensors interfaces. Use directive DW1000_DEV_TASK_PRIO to defined 
 * the priority of the dw1000_softstack at compiletime
 *
 * output parameters
 *
 */
void dw1000_tasks_init(dw1000_dev_instance_t * inst)
{
    /* Check if the tasks are already initiated */
    if (!os_eventq_inited(&inst->interrupt_eventq))
    {
        /* Use a dedicate event queue for timer and interrupt events */
        os_eventq_init(&inst->interrupt_eventq);
        /*
         * Create the task to process timer and interrupt events from the
         * my_timer_interrupt_eventq event queue.
         */
        inst->interrupt_ev.ev_cb = dw1000_interrupt_ev_cb;
        inst->interrupt_ev.ev_arg = (void *)inst;

        os_task_init(&inst->interrupt_task_str, "dw1000_irq",
                     dw1000_interrupt_task,
                     (void *) inst,
                     inst->interrupt_task_prio, OS_WAIT_FOREVER,
                     inst->interrupt_task_stack,
                     DW1000_DEV_TASK_STACK_SZ);

        hal_gpio_irq_init(inst->irq_pin, dw1000_irq, inst, HAL_GPIO_TRIG_RISING, HAL_GPIO_PULL_UP);
        hal_gpio_irq_enable(inst->irq_pin);
    }    
    dw1000_phy_interrupt_mask(inst, SYS_MASK_MRXFCG | SYS_MASK_MTXFRS | SYS_MASK_ALL_RX_TO | SYS_MASK_ALL_RX_ERR, true);
}

static void dw1000_irq(void *arg)
{
    dw1000_dev_instance_t * inst = arg;
    os_eventq_put(&inst->interrupt_eventq, &inst->interrupt_ev);
//    dw1000_interrupt_ev_cb(NULL);
}


static void dw1000_interrupt_task(void *arg)
{
    dw1000_dev_instance_t * inst = arg;
    while (1) {
        os_eventq_run(&inst->interrupt_eventq);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_set_callbacks()
 *
 * @brief This function is used to register the different callbacks called when one of the corresponding event occurs.
 *
 * NOTE: Callbacks can be undefined (set to NULL). In this case, dwt_isr() will process the event as usual but the 'null'
 * callback will not be called.
 *
 * input parameters
 * @param tx_complete_cb - the pointer to the TX confirmation event callback function
 * @param tx_complete_cb - the pointer to the RX good frame event callback function
 * @param rx_timeout_cb - the pointer to the RX timeout events callback function
 * @param rx_error_cb - the pointer to the RX error events callback function
 *
 * output parameters
 *
 */
void dw1000_set_callbacks(dw1000_dev_instance_t * inst,  dw1000_dev_cb_t tx_complete_cb,  dw1000_dev_cb_t rx_complete_cb,  dw1000_dev_cb_t rx_timeout_cb,  dw1000_dev_cb_t rx_error_cb)
{
    inst->tx_complete_cb = tx_complete_cb;
    inst->rx_complete_cb = rx_complete_cb;
    inst->rx_timeout_cb = rx_timeout_cb;
    inst->rx_error_cb = rx_error_cb;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_interrupt_ev_cb()
 *
 * @brief This is the DW1000's general Interrupt Service Routine. It will process/report the following events:
 *          - RXFCG (through rx_complete_cb callback)
 *          - TXFRS (through tx_complete_cb callback)
 *          - RXRFTO/RXPTO (through rx_timeout_cb callback)
 *          - RXPHE/RXFCE/RXRFSL/RXSFDTO/AFFREJ/LDEERR (through rx_error_cb cbRxErr)
 *        For all events, corresponding interrupts are cleared and necessary resets are performed. In addition, in the RXFCG case,
 *        received frame information and frame control are read before calling the callback. If double buffering is activated, it
 *        will also toggle between reception buffers once the reception callback processing has ended.
 *
 *        /!\ This version of the ISR supports double buffering but does not support automatic RX re-enabling!
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */

static void dw1000_interrupt_ev_cb(struct os_event *ev)
{
    dw1000_dev_instance_t * inst = ev->ev_arg;

    inst->sys_status = dw1000_read_reg(inst, SYS_STATUS_ID, 0, sizeof(uint32_t)); // Read status register low 32bits

    // Handle TX confirmation event
    if(inst->sys_status & SYS_STATUS_TXFRS){
        //printf("SYS_STATUS_TXFRS %08lX\n", inst->sys_status);
        dw1000_write_reg(inst, SYS_STATUS_ID, 0, SYS_STATUS_ALL_TX, sizeof(uint32_t)); // Clear TX event bits

        // In the case where this TXFRS interrupt is due to the automatic transmission of an ACK solicited by a response (with ACK request bit set)
        // that we receive through using wait4resp to a previous TX (and assuming that the IRQ processing of that TX has already been handled), then
        // we need to handle the IC issue which turns on the RX again in this situation (i.e. because it is wrongly applying the wait4resp after the
        // ACK TX).
        // See section "Transmit and automatically wait for response" in DW1000 User Manual
        os_error_t err = os_sem_release(&inst->sem);  // unblock dw1000_start_tx
        assert(err == OS_OK);

        if((inst->sys_status & SYS_STATUS_AAT) && inst->control.wait4resp_enabled){
            dw1000_phy_forcetrxoff(inst);   // Turn the RX off
            dw1000_phy_rx_reset(inst);      // Reset in case we were late and a frame was already being received
        }
        // Call the corresponding callback if present
        if(inst->rng_tx_complete_cb != NULL && inst->status.tx_ranging_frame)
            inst->rng_tx_complete_cb(inst);
        if(inst->tx_complete_cb != NULL)
            inst->tx_complete_cb(inst);
    }

    // Handle RX good frame event
    if(inst->sys_status & SYS_STATUS_RXFCG){
        //printf("SYS_STATUS_RXFCG %08lX\n", inst->sys_status);
        dw1000_write_reg(inst, SYS_STATUS_ID, 0, SYS_STATUS_ALL_RX_GOOD, sizeof(uint32_t));     // Clear all receive status bits
        uint16_t finfo = dw1000_read_reg(inst, RX_FINFO_ID, RX_FINFO_OFFSET, sizeof(uint16_t)); // Read frame info - Only the first two bytes of the register are used here.
        inst->frame_len = (finfo & RX_FINFO_RXFL_MASK_1023) - 2;          // Report frame length - Standard frame length up to 127, extended frame length up to 1023 bytes
        inst->status.rx_ranging_frame = (finfo & RX_FINFO_RNG) !=0; // Report ranging bit
        inst->fctrl = dw1000_read_reg(inst, RX_BUFFER_ID, MAC_FFORMAT_FCTRL, MAC_FFORMAT_FCTRL_LEN);// Report frame control - First bytes of the received frame.
        
        // Because of a previous frame not being received properly, AAT bit can be set upon the proper reception of a frame not requesting for
        // acknowledgement (ACK frame is not actually sent though). If the AAT bit is set, check ACK request bit in frame control to confirm (this
        // implementation works only for IEEE802.15.4-2011 compliant frames).
        // This issue is not documented at the time of writing this code. It should be in next release of DW1000 User Manual (v2.09, from July 2016).

        if((inst->sys_status & SYS_STATUS_AAT) && ((inst->fctrl & MAC_FTYPE_ACK) == 0)){
            dw1000_write_reg(inst, SYS_STATUS_ID, 0, SYS_STATUS_AAT, sizeof(uint32_t));     // Clear AAT status bit in register
            inst->sys_status &= ~SYS_STATUS_AAT; // Clear AAT status bit in callback data register copy
        }

        // Call the corresponding ranging frame services callback if present
        if(inst->rng_rx_complete_cb != NULL && inst->status.rx_ranging_frame) 
            inst->rng_rx_complete_cb(inst);
        // Call the corresponding non-ranging frame callback if present
        else if(inst->rx_complete_cb != NULL)
            inst->rx_complete_cb(inst);        
        // Collect RX Frame Quality diagnositics
        if(inst->config.rxdiag_enable)  
            dw1000_read_rxdiag(inst, &inst->rxdiag);
        // Toggle the Host side Receive Buffer Pointer
        if (inst->config.dblbuffon_enabled)
            dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, 1, sizeof(uint8_t));
    }

    // Handle frame reception/preamble detect timeout events
    inst->status.rx_timeout_error = (inst->sys_status & SYS_STATUS_ALL_RX_TO) !=0;
    if(inst->status.rx_timeout_error){
        // printf("SYS_STATUS_ALL_RX_TO %08lX\n", inst->sys_status);
        dw1000_write_reg(inst, SYS_STATUS_ID, 0, SYS_STATUS_ALL_RX_TO, sizeof(uint32_t)); // Clear RX timeout event bits        
        // Because of an issue with receiver restart after error conditions, an RX reset must be applied 
        // after any error or timeout event to ensure the next good frame's timestamp is computed correctly.
        // See section "RX Message timestamp" in DW1000 User Manual.
        dw1000_phy_forcetrxoff(inst);
        dw1000_phy_rx_reset(inst);

        // Call the corresponding ranging frame services callback if present
        if(inst->rng_rx_timeout_cb != NULL )
            inst->rng_rx_timeout_cb(inst);
        if(inst->rng_rx_timeout_extension_cb != NULL)
            inst->rng_rx_timeout_extension_cb(inst);     
        if(inst->rx_timeout_cb != NULL)
            inst->rx_timeout_cb(inst); 
    }

    // Handle RX errors events
    inst->status.rx_error = (inst->sys_status & SYS_STATUS_ALL_RX_ERR) !=0 ;
    if(inst->status.rx_error){
        //printf("SYS_STATUS_ALL_RX_ERR %08lX\n", inst->sys_status);
        dw1000_write_reg(inst, SYS_STATUS_ID, 0, SYS_STATUS_ALL_RX_ERR, sizeof(uint32_t)); // Clear RX error event bits
        // Because of an issue with receiver restart after error conditions, an RX reset must be applied after any error or timeout event to ensure
        // the next good frame's timestamp is computed correctly.
        // See section "RX Message timestamp" in DW1000 User Manual.
        dw1000_phy_forcetrxoff(inst);
        dw1000_phy_rx_reset(inst);

        // Call the corresponding ranging frame services callback if present
        if(inst->rng_rx_error_cb != NULL )
            inst->rng_rx_error_cb(inst);
        if(inst->rng_rx_error_extension_cb != NULL)
            inst->rng_rx_error_extension_cb(inst);       
        if(inst->rx_error_cb != NULL)
            inst->rx_error_cb(inst);
    }
}


/*! 
 * @fn dw1000_get_rssi(inst, *rssi)
 *
 * @brief calculates rssi from last RX in dBm
 *
 * Needs config.rxdiag_enable to be set
 *
 * Returns 0 on success
 */
int
dw1000_get_rssi(dw1000_dev_instance_t * inst, float *rssi)
{
    /* Check if we're reading the diagnostics */
    if (!inst->config.rxdiag_enable) {
        return 1;
    }
    if (!rssi) {
        return 1;
    }
    
    float A = (inst->mac_config.prf == DWT_PRF_16M) ? 115.72 : 122.74;
    if (!inst->rxdiag.max_growth_cir) {
        *rssi = -A;
        return 0;
    }

    float N = inst->rxdiag.preamble_cnt;
    float C = inst->rxdiag.max_growth_cir;
    float v = C*0x20000/(N*N);
    *rssi = 10.0*log10f(v) - A;
    return 0;
}
