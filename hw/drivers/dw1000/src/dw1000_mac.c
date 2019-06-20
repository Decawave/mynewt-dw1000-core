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
 * @file dw1000_mac.c
 * @author Paul Kettle
 * @date 2018
 * @brief Mac initialization  
 *
 * @details This is the mac base class which utilizes the functions to do the configurations related to mac layer based on dependencies.
 *
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <os/os.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include <stats/stats.h>

#include <dw1000/dw1000_ftypes.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_stats.h>
#include <dw1000/dw1000_mac.h>

#if MYNEWT_VAL(CCP_ENABLED)
#include <ccp/ccp.h>
#endif
#if MYNEWT_VAL(RNG_ENABLED)
#include <rng/rng.h>
#endif
#if MYNEWT_VAL(WCS_ENABLED)
#include <wcs/wcs.h>
#endif

#if MYNEWT_VAL(DW1000_MAC_STATS)
STATS_NAME_START(mac_stat_section)
    STATS_NAME(mac_stat_section, tx_bytes)
    STATS_NAME(mac_stat_section, rx_bytes)
    STATS_NAME(mac_stat_section, DFR_cnt)
    STATS_NAME(mac_stat_section, RTO_cnt)
    STATS_NAME(mac_stat_section, ROV_err)
    STATS_NAME(mac_stat_section, TFG_cnt)
    STATS_NAME(mac_stat_section, LDE_err)
    STATS_NAME(mac_stat_section, RX_err)
    STATS_NAME(mac_stat_section, TXBUF_err)
STATS_NAME_END(mac_stat_section)

#define MAC_STATS_INC(__X) STATS_INC(inst->stat, __X)
#define MAC_STATS_INCN(__X, __Y) STATS_INCN(inst->stat, __X, __Y)
#else
#define MAC_STATS_INC(__X) {}
#define MAC_STATS_INCN(__X, __Y) {}
#endif

int dw1000_cli_register(void);
static void dw1000_interrupt_task(void *arg);
static void dw1000_interrupt_ev_cb(struct os_event *ev);
static void dw1000_irq(void *arg);

//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

#define NUM_BR 3            //!< Bit Rate
#define NUM_PRF 2           //!< Pulse repitition frequency
#define NUM_PACS 4          //!< Preamble Acquisition Chunk
#define NUM_BW 2            //!< Bandwidths
#define NUM_SFD 2           //!< Start of frame delimiter


//! map the channel number to the index in the configuration arrays below.
//! 0th element is chan 1, 1st is chan 2, 2nd is chan 3, 3rd is chan 4, 4th is chan 5, 5th is chan 7.
const uint8_t chan_idx[] = {0, 0, 1, 2, 3, 4, 0, 5};

//! Transmit config parameters
const uint32_t tx_config[] =
{
    RF_TXCTRL_CH1,
    RF_TXCTRL_CH2,
    RF_TXCTRL_CH3,
    RF_TXCTRL_CH4,
    RF_TXCTRL_CH5,
    RF_TXCTRL_CH7,
};

//! Frequency Synthesiser - PLL configuration
const uint32_t fs_pll_cfg[] =
{
    FS_PLLCFG_CH1,
    FS_PLLCFG_CH2,
    FS_PLLCFG_CH3,
    FS_PLLCFG_CH4,
    FS_PLLCFG_CH5,
    FS_PLLCFG_CH7
};

//! Frequency Synthesiser - PLL tuning
const uint8_t fs_pll_tune[] =
{
    FS_PLLTUNE_CH1,
    FS_PLLTUNE_CH2,
    FS_PLLTUNE_CH3,
    FS_PLLTUNE_CH4,
    FS_PLLTUNE_CH5,
    FS_PLLTUNE_CH7
};

//! Bandwidth configuration
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
    { AGC_TUNE1_16M , AGC_TUNE1_64M }  //!< adc target
};

//! DW non-standard SFD length for 110k, 850k and 6.81M
const uint8_t dwnsSFDlen[] =
{
    DW_NS_SFD_LEN_110K,
    DW_NS_SFD_LEN_850K,
    DW_NS_SFD_LEN_6M8
};

//! SFD Threshold
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


/**
 * API to configure the mac layer in dw1000
 * @param inst     Pointer to _dw1000_dev_instance_t.
 * @param config   Pointer to dw1000_dev_config_t.
 * @return dw1000_dev_status_t 
 *
 */
struct _dw1000_dev_status_t dw1000_mac_config(struct _dw1000_dev_instance_t * inst,
                                              dw1000_dev_config_t * config)
{
    if (config == NULL) {
        config = &inst->config;
    } else {
        memcpy(&inst->config, config, sizeof(dw1000_dev_config_t));
    }
    
    uint8_t nsSfd_result  = 0;
    uint8_t useDWnsSFD = 0;
    uint8_t chan = config->channel;
    uint8_t prfIndex = config->prf - DWT_PRF_16M;
    uint8_t bw = ((chan == 4) || (chan == 7)) ? 1 : 0 ; // Select wide or narrow band
    uint16_t reg16 = lde_replicaCoeff[config->rx.preambleCodeIndex];

#ifdef DW1000_API_ERROR_CHECK
    assert(config->dataRate <= DWT_BR_6M8);
    assert(config->rx.pacLength <= DWT_PAC64);
    assert((chan >= 1) && (chan <= 7) && (chan != 6));
    
    assert(((config->prf == DWT_PRF_64M) && (config->tx.preambleCodeIndex >= 9) &&
            (config->tx.preambleCodeIndex <= 24)) ||
           ((config->prf == DWT_PRF_16M) && (config->tx.preambleCodeIndex >= 1) &&
            (config->tx.preambleCodeIndex <= 8)));

    assert(((config->prf == DWT_PRF_64M) && (config->rx.preambleCodeIndex >= 9) &&
            (config->rx.preambleCodeIndex <= 24)) ||
           ((config->prf == DWT_PRF_16M) && (config->rx.preambleCodeIndex >= 1) &&
            (config->rx.preambleCodeIndex <= 8)));

    assert((config->tx.preambleLength == DWT_PLEN_64) || (config->tx.preambleLength == DWT_PLEN_128) ||
           (config->tx.preambleLength == DWT_PLEN_256)|| (config->tx.preambleLength == DWT_PLEN_512) ||
           (config->tx.preambleLength == DWT_PLEN_1024) ||
           (config->tx.preambleLength == DWT_PLEN_1536) ||
           (config->tx.preambleLength == DWT_PLEN_2048) ||
           (config->tx.preambleLength == DWT_PLEN_4096));

    assert((config->rx.phrMode == DWT_PHRMODE_STD) || (config->rx.phrMode == DWT_PHRMODE_EXT));
#endif
    
    /* For 110 kbps we need a special setup */
    if(config->dataRate == DWT_BR_110K){
        inst->sys_cfg_reg |= SYS_CFG_RXM110K;
        reg16 >>= 3; // lde_replicaCoeff must be divided by 8
    }else{
        inst->sys_cfg_reg &= (~SYS_CFG_RXM110K);
    }

    inst->sys_cfg_reg &= ~SYS_CFG_PHR_MODE_11;
    inst->sys_cfg_reg |= (SYS_CFG_PHR_MODE_11 & (((uint32_t)config->rx.phrMode) << SYS_CFG_PHR_MODE_SHFT));
    
    if (inst->config.rxauto_enable) 
        inst->sys_cfg_reg |=SYS_CFG_RXAUTR; 
    
    dw1000_write_reg(inst, SYS_CFG_ID, 0, inst->sys_cfg_reg, sizeof(uint32_t));
    /* Set the lde_replicaCoeff */
    dw1000_write_reg(inst, LDE_IF_ID, LDE_REPC_OFFSET, reg16, sizeof(uint16_t));

    dw1000_phy_config_lde(inst, prfIndex);

    /* Configure PLL2/RF PLL block CFG/TUNE (for a given channel) */
    dw1000_write_reg(inst, FS_CTRL_ID, FS_PLLCFG_OFFSET, fs_pll_cfg[chan_idx[chan]], sizeof(uint32_t));
    dw1000_write_reg(inst, FS_CTRL_ID, FS_PLLTUNE_OFFSET, fs_pll_tune[chan_idx[chan]], sizeof(uint8_t));

    /* Configure RF RX blocks (for specified channel/bandwidth) */
    dw1000_write_reg(inst, RF_CONF_ID, RF_RXCTRLH_OFFSET, rx_config[bw], sizeof(uint8_t));

    /* Configure RF TX blocks (for specified channel and PRF)
     * Configure RF TX control */
    dw1000_write_reg(inst, RF_CONF_ID, RF_TXCTRL_OFFSET, tx_config[chan_idx[chan]], sizeof(uint32_t));

    /* Configure the baseband parameters (for specified PRF, bit rate, PAC, and SFD settings) */
    /* DTUNE0 */
    dw1000_write_reg(inst, DRX_CONF_ID, DRX_TUNE0b_OFFSET, sftsh[config->dataRate][config->rx.sfdType], sizeof(uint16_t));
    /* DTUNE1 */
    dw1000_write_reg(inst, DRX_CONF_ID, DRX_TUNE1a_OFFSET, dtune1[prfIndex], sizeof(uint16_t));

    if(config->dataRate == DWT_BR_110K){
        dw1000_write_reg(inst, DRX_CONF_ID, DRX_TUNE1b_OFFSET, DRX_TUNE1b_110K, sizeof(uint16_t));
    }else{
        if(config->tx.preambleLength == DWT_PLEN_64){
            dw1000_write_reg(inst, DRX_CONF_ID, DRX_TUNE1b_OFFSET, DRX_TUNE1b_6M8_PRE64, sizeof(uint16_t));
            dw1000_write_reg(inst, DRX_CONF_ID, DRX_TUNE4H_OFFSET, DRX_TUNE4H_PRE64, sizeof(uint16_t));
        }else{
            dw1000_write_reg(inst, DRX_CONF_ID, DRX_TUNE1b_OFFSET, DRX_TUNE1b_850K_6M8, sizeof(uint16_t));
            dw1000_write_reg(inst, DRX_CONF_ID, DRX_TUNE4H_OFFSET, DRX_TUNE4H_PRE128PLUS, sizeof(uint16_t));
        }
    }

    /* DTUNE2 */
    dw1000_write_reg(inst, DRX_CONF_ID, DRX_TUNE2_OFFSET,
                     digital_bb_config[prfIndex][config->rx.pacLength], sizeof(uint32_t));

    /* DTUNE3 (SFD timeout) */
    /* Don't allow 0 - SFD timeout will always be enabled */
    if(config->rx.sfdTimeout == 0)
        config->rx.sfdTimeout= DWT_SFDTOC_DEF;
    
    dw1000_write_reg(inst, DRX_CONF_ID, DRX_SFDTOC_OFFSET, config->rx.sfdTimeout, sizeof(uint16_t));

    /* Configure AGC parameters */
    dw1000_write_reg(inst, AGC_CTRL_ID, AGC_TUNE2_OFFSET, agc_config.lo32, sizeof(uint32_t));
    dw1000_write_reg(inst, AGC_CTRL_ID, AGC_TUNE1_OFFSET, agc_config.target[prfIndex], sizeof(uint16_t));

    /* Set (non-standard) user SFD for improved performance, */
    if(config->rx.sfdType){
        /* Write non standard (DW) SFD length */
        dw1000_write_reg(inst, USR_SFD_ID, 0x0, dwnsSFDlen[config->dataRate], sizeof(uint8_t));
        nsSfd_result = 3 ;
        useDWnsSFD = 1 ;
    }
    uint32_t regval =  (CHAN_CTRL_TX_CHAN_MASK & (((uint32_t)chan) << CHAN_CTRL_TX_CHAN_SHIFT)) |          // Transmit Channel
        (CHAN_CTRL_RX_CHAN_MASK & (((uint32_t)chan) << CHAN_CTRL_RX_CHAN_SHIFT)) |                         // Receive Channel
        (CHAN_CTRL_RXFPRF_MASK & (((uint32_t)config->prf) << CHAN_CTRL_RXFPRF_SHIFT)) |                    // RX PRF
        ((CHAN_CTRL_TNSSFD|CHAN_CTRL_RNSSFD) & (((uint32_t)nsSfd_result) << CHAN_CTRL_TNSSFD_SHIFT)) |     // nsSFD enable RX&TX
        (CHAN_CTRL_DWSFD & (((uint32_t)useDWnsSFD) << CHAN_CTRL_DWSFD_SHIFT)) |                            // Use DW nsSFD
        (CHAN_CTRL_TX_PCOD_MASK & (((uint32_t)config->tx.preambleCodeIndex) << CHAN_CTRL_TX_PCOD_SHIFT)) | // TX Preamble Code
        (CHAN_CTRL_RX_PCOD_MASK & (((uint32_t)config->rx.preambleCodeIndex) << CHAN_CTRL_RX_PCOD_SHIFT)) ; // RX Preamble Code

    dw1000_write_reg(inst, CHAN_CTRL_ID, 0, regval, sizeof(uint32_t)) ;

    /* Set up TX Preamble Size, PRF and Data Rate */
    inst->tx_fctrl = (((uint32_t)(config->tx.preambleLength | config->prf)) << TX_FCTRL_TXPRF_SHFT) |
        (((uint32_t)config->dataRate) << TX_FCTRL_TXBR_SHFT);
    dw1000_write_reg(inst, TX_FCTRL_ID, 0, inst->tx_fctrl, sizeof(uint32_t));
    /* The SFD transmit pattern is initialised by the DW1000 upon a user TX request,
     * but (due to an IC issue) it is not done for an auto-ACK TX.
     * The SYS_CTRL write below works around this issue, by simultaneously initiating
     * and aborting a transmission, which correctly initialises the SFD
     * after its configuration or reconfiguration. */
    /* Request TX start and TRX off at the same time */
    dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, SYS_CTRL_TXSTRT | SYS_CTRL_TRXOFF, sizeof(uint8_t));

#if MYNEWT_VAL(DW1000_MAC_FILTERING)
    if(inst->config.framefilter_enabled){
        dw1000_mac_framefilter(inst, DWT_FF_BEACON_EN | DWT_FF_DATA_EN | DWT_FF_RSVD_EN );
    }
#endif
    
    if (inst->config.rxauto_enable)     
        assert(inst->config.trxoff_enable);
        
    if(inst->config.dblbuffon_enabled)
        dw1000_set_dblrxbuff(inst, true);

    return inst->status;
}


/**
 * API to initialize the mac layer.
 * @param inst     Pointer to _dw1000_dev_instance_t.
 * @param config   Pointer to dw1000_dev_config_t.
 * @return dw1000_dev_status_t
 *
 */
struct _dw1000_dev_status_t dw1000_mac_init(struct _dw1000_dev_instance_t * inst, dw1000_dev_config_t * config)
{
    /* Configure DW1000 */
    dw1000_mac_config(inst, config);

    dw1000_tasks_init(inst);

#if MYNEWT_VAL(DW1000_MAC_STATS)
    int rc = stats_init(
        STATS_HDR(inst->stat),
        STATS_SIZE_INIT_PARMS(inst->stat, STATS_SIZE_32),
        STATS_NAME_INIT_PARMS(mac_stat_section));
    assert(rc == 0);

#if  MYNEWT_VAL(DW1000_DEVICE_0) && !MYNEWT_VAL(DW1000_DEVICE_1)
    rc = stats_register("mac", STATS_HDR(inst->stat));
#elif  MYNEWT_VAL(DW1000_DEVICE_0) && MYNEWT_VAL(DW1000_DEVICE_1)
    if (inst == hal_dw1000_inst(0))
        rc |= stats_register("mac0", STATS_HDR(inst->stat));
    else
        rc |= stats_register("mac1", STATS_HDR(inst->stat));
#endif
    assert(rc == 0);
#endif

#if MYNEWT_VAL(DW1000_CLI)
    dw1000_cli_register();
#endif

    return inst->status;
} 

/**
 * API to read the supplied RX data from the DW1000's
 * TX buffer.The input parameters are the data length in bytes and a pointer
 * to those data bytes.
 *
 * @param inst              Pointer to _dw1000_dev_instance_t.
 * @param rxFrameLength     This is the total frame length, including the two byte CRC.
 * Note: This is the length of RX message (including the 2 byte CRC) - max is 1023 standard PHR mode allows up to 127 bytes
 * if > 127 is programmed, DWT_PHRMODE_EXT needs to be set in the phrMode configuration.
 *
 * @param rxFrameBytes      Pointer to the user buffer containing the data to send.
 * @param rxBufferOffset    This specifies an offset in the DW1000s TX Buffer where writing of data starts.
 * @return dw1000_dev_status_t
 */
struct _dw1000_dev_status_t dw1000_read_rx(struct _dw1000_dev_instance_t * inst,  uint8_t * rxFrameBytes, uint16_t rxBufferOffset, uint16_t rxFrameLength)
{
#ifdef DW1000_API_ERROR_CHECK
    assert((config->rx.phrMode && (txFrameLength <= 1023)) || (txFrameLength <= 127));
    assert((txBufferOffset + txFrameLength) <= 1024);
#endif
    MAC_STATS_INCN(rx_bytes, rxFrameLength);

    os_error_t err = os_mutex_pend(&inst->mutex,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    dw1000_read(inst, RX_BUFFER_ID, rxBufferOffset, rxFrameBytes, rxFrameLength);

    err = os_mutex_release(&inst->mutex); 
    assert(err == OS_OK); 
    
    return inst->status;
}

/**
 * API to write the supplied TX data into the DW1000's
 * TX buffer.The input parameters are the data length in bytes and a pointer
 * to those data bytes.
 *
 * @param inst              Pointer to _dw1000_dev_instance_t.
 * @param txFrameLength     This is the total frame length, including the two byte CRC.
 * Note: This is the length of TX message (including the 2 byte CRC) - max is 1023 standard PHR mode allows up to 127 bytes
 * if > 127 is programmed, DWT_PHRMODE_EXT needs to be set in the phrMode configuration.
 *
 * @param txFrameBytes      Pointer to the user buffer containing the data to send.
 * @param txBufferOffset    This specifies an offset in the DW1000s TX Buffer where writing of data starts.
 * @return dw1000_dev_status_t
 */
struct _dw1000_dev_status_t dw1000_write_tx(struct _dw1000_dev_instance_t * inst,  uint8_t * txFrameBytes, uint16_t txBufferOffset, uint16_t txFrameLength)
{
#ifdef DW1000_API_ERROR_CHECK
    assert((config->rx.phrMode && (txFrameLength <= 1023)) || (txFrameLength <= 127));
    assert((txBufferOffset + txFrameLength) <= 1024);
#endif
    MAC_STATS_INCN(tx_bytes, txFrameLength);

    os_error_t err = os_mutex_pend(&inst->mutex,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    if ((txBufferOffset + txFrameLength) <= 1024){
        dw1000_write(inst, TX_BUFFER_ID, txBufferOffset,  txFrameBytes, txFrameLength);
        /* This is only valid if the offset is 0, and not always then either  */
        if (txBufferOffset == 0) {
            for (uint8_t i = 0; i< sizeof(inst->fctrl); i++)
                inst->fctrl_array[i] =  txFrameBytes[i];
        }
        inst->status.tx_frame_error = 0;
    }
    else
        inst->status.tx_frame_error = 1;

    err = os_mutex_release(&inst->mutex); 
    assert(err == OS_OK); 
    
    return inst->status;
}

/**
 * API to configure the TX frame control register before the transmission of a frame.
 *
 * @param inst              pointer to _dw1000_dev_instance_t.
 * @param txFrameLength     This is the length of TX message (excluding the 2 byte CRC) - max is 1023
 * NOTE: standard PHR mode allows up to 127 bytes.
 * if > 127 is programmed, DWT_PHRMODE_EXT needs to be set in the phrMode configuration.          
 *
 * @param txBufferOffset    The offset in the tx buffer to start writing the data.
 * @return void
 */
inline void dw1000_write_tx_fctrl(struct _dw1000_dev_instance_t * inst, uint16_t txFrameLength, uint16_t txBufferOffset)
{
#ifdef DW1000_API_ERROR_CHECK
    assert((inst->longFrames && ((txFrameLength + 2) <= 1023)) || ((txFrameLength +2) <= 127));
#endif
    os_error_t err = os_mutex_pend(&inst->mutex,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    // Write the frame length to the TX frame control register
    uint32_t tx_fctrl_reg = inst->tx_fctrl | (txFrameLength + 2)  | (((uint32_t)txBufferOffset) << TX_FCTRL_TXBOFFS_SHFT);
    dw1000_write_reg(inst, TX_FCTRL_ID, 0, tx_fctrl_reg, sizeof(uint32_t));
 
    err = os_mutex_release(&inst->mutex); 
    assert(err == OS_OK);  
} 

/**
 * API to start transmission.
 *
 * @param inst  pointer to _dw1000_dev_instance_t.
 * @return dw1000_dev_status_t
 */
struct _dw1000_dev_status_t dw1000_start_tx(struct _dw1000_dev_instance_t * inst)
{

    os_error_t err = os_sem_pend(&inst->tx_sem,  OS_TIMEOUT_NEVER); // Released by a SYS_STATUS_TXFRS event
    assert(err == OS_OK);

    dw1000_dev_control_t control = inst->control;
    dw1000_dev_config_t config = inst->config;

    if (config.trxoff_enable){ // force return to idle state
        dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8_t) SYS_CTRL_TRXOFF, sizeof(uint8_t)); 
    }    
    
    uint32_t sys_ctrl_reg = SYS_CTRL_TXSTRT;
    if (control.wait4resp_enabled){
        dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8_t) SYS_CTRL_WAIT4RESP, sizeof(uint8_t));
        sys_ctrl_reg |= SYS_CTRL_WAIT4RESP; 
    }
    if (control.delay_start_enabled)
        sys_ctrl_reg |= SYS_CTRL_TXDLYS; 

    if (control.delay_start_enabled){
        dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8_t) sys_ctrl_reg, sizeof(uint8_t));
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
            dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8_t) SYS_CTRL_TRXOFF, sizeof(uint8_t)); 
            err = os_sem_release(&inst->tx_sem);
            assert(err == OS_OK);
        }
    }else{
        dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, sys_ctrl_reg, sizeof(uint8_t));
        inst->status.start_tx_error = 0;

        /* If dw1000 is instructed to sleep after tx, release
         * the sem as there will not be a TXDONE irq */
        if(inst->control.sleep_after_tx) {
            inst->status.sleeping = 1;
            err = os_sem_release(&inst->tx_sem);
        }
    }

    inst->control = (dw1000_dev_control_t){
        .wait4resp_enabled=0,
        .wait4resp_delay_enabled=0,
        .delay_start_enabled=0,
        .autoack_delay_enabled=0,
        .on_error_continue_enabled=0
    };

    return inst->status;
} 


/**
 * API to specify a time in future to either turn on the receiver to be ready to receive a frame, 
 * or to turn on the transmitter and send a frame. The low-order 9-bits of this register are ignored. The delay is in UWB microseconds.  
 * 
 * @param inst     Pointer to _dw1000_dev_instance_t.
 * @param delay    Delayed Send or receive Time. 
 * @return dw1000_dev_status_t
 */
inline struct _dw1000_dev_status_t dw1000_set_delay_start(struct _dw1000_dev_instance_t * inst, uint64_t dx_time)
{
    os_error_t err = os_mutex_pend(&inst->mutex,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    inst->control.delay_start_enabled = true;
    dw1000_write_reg(inst, DX_TIME_ID, 1, dx_time >> 8, DX_TIME_LEN-1);

    err = os_mutex_release(&inst->mutex); 
    assert(err == OS_OK); 

    return inst->status;
}


/**
 * API to keep the transceiver in reception mode to keep on receiving the data.
 *
 * @param inst  pointer to _dw1000_dev_instance_t.
 * @return dw1000_dev_status_t
 * 
 */
struct _dw1000_dev_status_t dw1000_start_rx(struct _dw1000_dev_instance_t * inst)
{
    os_error_t err = os_mutex_pend(&inst->mutex,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    dw1000_dev_control_t control = inst->control;
    dw1000_dev_config_t config = inst->config;

    if (config.trxoff_enable){ // force return to idle state, if in RX state
        uint8_t state = (uint8_t) dw1000_read_reg(inst, SYS_STATE_ID, PMSC_STATE_OFFSET, sizeof(uint8_t));
        if(state != PMSC_STATE_IDLE )    
            dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8_t) SYS_CTRL_TRXOFF, sizeof(uint8_t)); 
    }    
    
    uint16_t sys_ctrl = SYS_CTRL_RXENAB;
    if (config.dblbuffon_enabled)
        dw1000_sync_rxbufptrs(inst);
    if (control.delay_start_enabled) 
        sys_ctrl |= SYS_CTRL_RXDLYE;

    dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, sys_ctrl, sizeof(uint16_t));
    if (control.delay_start_enabled){   // check for errors    
        uint8_t sys_status = dw1000_read_reg(inst, SYS_STATUS_ID, 3, sizeof(uint8_t));  // Read 1 byte at offset 3 to get the 4th byte out of 5
        inst->status.start_rx_error = (sys_status & (SYS_STATUS_HPDWARN >> 24)) != 0;   
        if (inst->status.start_rx_error){   // if delay has passed do immediate RX on unless DWT_IDLE_ON_DLY_ERR is true
            dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8_t) SYS_CTRL_TRXOFF, sizeof(uint8_t)); // return to idle state
            if (control.on_error_continue_enabled){
                sys_ctrl &= ~SYS_CTRL_RXDLYE;
                dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, sys_ctrl, sizeof(uint16_t)); // turn on receiver 
            }       
        }
    }else{
        inst->status.start_rx_error = 0;
    }

    inst->control = (dw1000_dev_control_t){
        .wait4resp_enabled=0,
        .wait4resp_delay_enabled=0,
        .delay_start_enabled=0,
        .autoack_delay_enabled=0,
        .start_rx_syncbuf_enabled=0,
        .rx_timeout_enabled=0,
        .on_error_continue_enabled=0
    };

    err = os_mutex_release(&inst->mutex); 
    assert(err == OS_OK); 

    return inst->status;
} 

/**
 * API to gracefully turnoff reception mode.
 *
 * @param inst  pointer to _dw1000_dev_instance_t.
 * @return dw1000_dev_status_t
 * 
 */
struct _dw1000_dev_status_t dw1000_stop_rx(struct _dw1000_dev_instance_t * inst)
{
    os_error_t err = os_mutex_pend(&inst->mutex,  OS_WAIT_FOREVER);
    assert(err == OS_OK);

    dw1000_set_rx_timeout(inst, 0);

    uint32_t mask = dw1000_read_reg(inst, SYS_MASK_ID, 0 , sizeof(uint32_t)) ; // Read set interrupt mask
    dw1000_write_reg(inst, SYS_MASK_ID, 0, 0, sizeof(uint32_t)) ; // Clear interrupt mask - so we don't get any unwanted events        
    dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8_t) SYS_CTRL_TRXOFF, sizeof(uint8_t)); // return to idle state
    dw1000_write_reg(inst, SYS_STATUS_ID, 0, (SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_GOOD), sizeof(uint32_t));
    dw1000_write_reg(inst, SYS_MASK_ID, 0, mask, sizeof(uint32_t)); // Restore mask to what it was
    
    err = os_mutex_release(&inst->mutex); 
    assert(err == OS_OK); 

    return inst->status;
} 

/**
 * API to enable wait for response feature.
 * 
 * @param inst     Pointer to _dw1000_dev_instance_t.
 * @param enable   Enables/disables the wait for response feature.
 * @return dw1000_dev_status_t
 */
inline struct _dw1000_dev_status_t 
dw1000_set_wait4resp(struct _dw1000_dev_instance_t * inst, bool enable)
{
    inst->control.wait4resp_enabled = enable;
    return inst->status;
}
/**
 * API to enable wait for feature.
 * 
 * @param inst    Pointer to _dw1000_dev_instance_t.
 * @param enable  Enables wait for feature.
 *
 */
inline struct _dw1000_dev_status_t 
dw1000_set_on_error_continue(struct _dw1000_dev_instance_t * inst, bool enable)
{
    inst->control.on_error_continue_enabled = enable;
    return inst->status;
}


/**
 * API to set Wait Timeout period.
 *
 * @param inst      pointer to _dw1000_dev_instance_t.
 * @param timeout   Indicates how long the receiver remains on from the RX enable command.The time parameter used here is in 1.0256 
 * us (512/499.2MHz) units If set to 0 the timeout is disabled.
 * @return dw1000_dev_status_t
 * @brief The Receive Frame Wait Timeout period is a 16-bit field. The units for this parameter are roughly 1μs, 
 * (the exact unit is 512 counts of the fundamental 499.2 MHz UWB clock, or 1.026 μs). When employing the frame wait timeout, 
 * RXFWTO should be set to a value greater than the expected RX frame duration and include an allowance for any uncertainly 
 * attaching to the expected transmission start time of the awaited frame. 
 * When using .rxauto_enable feature it is important to understand the role of rx_timeout, in this situation it is the timeout 
 * that actually turns-off the receiver and returns the transeiver to the idle state. 
 */
struct _dw1000_dev_status_t 
dw1000_set_rx_timeout(struct _dw1000_dev_instance_t * inst, uint16_t timeout)
{
    dw1000_dev_control_t control = inst->control;
    os_error_t err = os_mutex_pend(&inst->mutex,  OS_TIMEOUT_NEVER); // Block if request pending
    assert(err == OS_OK);

    inst->status.rx_timeout_error = 0;

    uint32_t  sys_cfg_reg = SYS_CFG_MASK & dw1000_read_reg(inst, SYS_CFG_ID, 0, sizeof(uint32_t)); 

    control.rx_timeout_enabled = timeout > 0;
    if(control.rx_timeout_enabled){  
        dw1000_write_reg(inst, RX_FWTO_ID, RX_FWTO_OFFSET, (uint16_t)ceilf(dw1000_usecs_to_dwt_usecs(timeout)), sizeof(uint16_t));
        sys_cfg_reg |= SYS_CFG_RXWTOE;
        dw1000_write_reg(inst, SYS_CFG_ID, 0, sys_cfg_reg, sizeof(uint32_t));
    }else{
        control.on_error_continue_enabled = 1; 
        sys_cfg_reg &= ~SYS_CFG_RXWTOE;
        dw1000_write_reg(inst, SYS_CFG_ID, 0, sys_cfg_reg, sizeof(uint32_t));
    }
          
    err = os_mutex_release(&inst->mutex);  
    assert(err == OS_OK);

    return inst->status;
} 

/**
 * API to synchronize rx buffer pointers to make sure that the host/IC buffer pointers are aligned before starting RX.
 *
 * @param inst  pointer to _dw1000_dev_instance_t.
 * @return dw1000_dev_status_t
 */
inline struct _dw1000_dev_status_t 
dw1000_sync_rxbufptrs(struct _dw1000_dev_instance_t * inst)
{
    uint8_t  buff;

    inst->control.start_rx_syncbuf_enabled = 1;
    // Need to make sure that the host/IC buffer pointers are aligned before starting RX
    buff = dw1000_read_reg(inst, SYS_STATUS_ID, 3, sizeof(uint8_t)); // Read 1 byte at offset 3 to get the 4th byte out of 5
    
    if((buff & (SYS_STATUS_ICRBP >> 24)) !=         // IC side Receive Buffer Pointer
       ((buff & (SYS_STATUS_HSRBP >> 24)) << 1) )   // Host Side Receive Buffer Pointer
        dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, 0x01, sizeof(uint8_t)); // We need to swap RX buffer status reg (write one to toggle internally)

    return inst->status;
}

/**
 * API to read the data from the Accumulator buffer, from an offset location give by offset parameter.
 *
 * NOTE: Because of an internal memory access delay when reading the accumulator the first octet output is a dummy octet
 *       that should be discarded. This is true no matter what sub-index the read begins at.
 *
 * @param inst       Pointer to _dw1000_dev_instance_t.
 * @param buffer     The buffer into which the data will be read.
 * @param length     The length of data to read (in bytes).
 * @param accOffset  The offset in the acc buffer from which to read the data.
 * @return dw1000_dev_status_t
 *
 */
struct _dw1000_dev_status_t 
dw1000_read_accdata(struct _dw1000_dev_instance_t * inst, uint8_t *buffer, uint16_t accOffset, uint16_t len)
{
    os_error_t err = os_mutex_pend(&inst->mutex,  OS_TIMEOUT_NEVER); // Block if request pending
    assert(err == OS_OK);

    // Force on the ACC clocks if we are sequenced
    dw1000_phy_sysclk_ACC(inst, true);
    dw1000_read(inst, ACC_MEM_ID, accOffset, buffer, len) ;
    dw1000_phy_sysclk_ACC(inst, false);
    
    err = os_mutex_release(&inst->mutex);  
    assert(err == OS_OK);
    return inst->status;
}


/**
 * API to enable the frame filtering - (the default option is to
 * accept any data and ACK frames with correct destination address.
 *
 * @param inst     Pointer to structure _dw1000_dev_instance_t.
 * @param bitmask  Enables/disables the frame filtering options according to
 *      DWT_FF_NOTYPE_EN        0x000   no frame types allowed
 *      DWT_FF_COORD_EN         0x002   behave as coordinator (can receive frames with no destination address (PAN ID has to match))
 *      DWT_FF_BEACON_EN        0x004   beacon frames allowed
 *      DWT_FF_DATA_EN          0x008   data frames allowed
 *      DWT_FF_ACK_EN           0x010   ack frames allowed
 *      DWT_FF_MAC_EN           0x020   mac control frames allowed
 *      DWT_FF_RSVD_EN          0x040   reserved frame types allowed
 * @return dw1000_dev_status_t
 *
 */
struct _dw1000_dev_status_t 
dw1000_mac_framefilter(struct _dw1000_dev_instance_t * inst, uint16_t enable)
{
    os_error_t err = os_mutex_pend(&inst->mutex,  OS_TIMEOUT_NEVER); // Block if request pending
    assert(err == OS_OK);

    uint32_t sys_cfg_reg = SYS_CFG_MASK & dw1000_read_reg(inst, SYS_CFG_ID, 0, sizeof(uint32_t)) ; // Read sysconfig register

    inst->config.framefilter_enabled = enable > 0;
    if(inst->config.framefilter_enabled){   // Enable frame filtering and configure frame types
        sys_cfg_reg &= ~(SYS_CFG_FF_ALL_EN);  // Clear all
        sys_cfg_reg |= (enable & SYS_CFG_FF_ALL_EN) | SYS_CFG_FFE;
    }else
        sys_cfg_reg &= ~(SYS_CFG_FFE);

    dw1000_write_reg(inst, SYS_CFG_ID,0, sys_cfg_reg, sizeof(uint32_t)); 
    err = os_mutex_release(&inst->mutex);  
    assert(err == OS_OK);

    return inst->status;
}



/**
 * API to enable the auto-ACK feature. If the delay (parameter) is 0, the ACK will be sent as-soon-as-possable
 * otherwise it will be sent with a programmed delay (in symbols), max is 255.
 * NOTE: needs to have frame filtering enabled as well.
 *
 * @param inst    Pointer to _dw1000_dev_instance_t.
 * @param enable  If non-zero the ACK is sent after this delay, max is 255.
 * @return dw1000_dev_status_t
 *
 */
struct _dw1000_dev_status_t 
dw1000_set_autoack(struct _dw1000_dev_instance_t * inst, bool enable)
{
    assert(inst->config.framefilter_enabled);

    os_error_t err = os_mutex_pend(&inst->mutex,  OS_TIMEOUT_NEVER); // Block if request pending
    assert(err == OS_OK);

    uint32_t sys_cfg_reg = SYS_CFG_MASK & dw1000_read_reg(inst, SYS_CFG_ID, 0, sizeof(uint32_t)); // Read sysconfig register
    inst->config.autoack_enabled = enable > 0;    
    if(inst->config.autoack_enabled){
        sys_cfg_reg |= SYS_CFG_AUTOACK;
        dw1000_write_reg(inst, SYS_CFG_ID,0, sys_cfg_reg, sizeof(uint32_t));
    } else {
        sys_cfg_reg &= ~SYS_CFG_AUTOACK;
        dw1000_write_reg(inst, SYS_CFG_ID,0, sys_cfg_reg, sizeof(uint32_t));
    }

    err = os_mutex_release(&inst->mutex);  
    assert(err == OS_OK);

    return inst->status;
}


/**
 * API to enable the auto-ACK feature. If the delay (parameter) is 0, the ACK will be sent as-soon-as-possable
 * otherwise it will be sent with a programmed delay (in symbols), max is 255.
 * NOTE: needs to have frame filtering enabled as well.
 *
 * @param inst   Pointer to _dw1000_dev_instance_t.
 * @param delay  If non-zero the ACK is sent after this delay, max is 255.
 * @return dw1000_dev_status_t
 *
 */
struct _dw1000_dev_status_t 
dw1000_set_autoack_delay(struct _dw1000_dev_instance_t * inst, uint8_t delay)
{
    assert(inst->config.framefilter_enabled);

    os_error_t err = os_mutex_pend(&inst->mutex,  OS_TIMEOUT_NEVER); // Block if request pending
    assert(err == OS_OK);

    inst->config.autoack_delay_enabled = delay > 0;
    if (inst->control.autoack_delay_enabled)
        dw1000_write_reg(inst, ACK_RESP_T_ID, ACK_RESP_T_ACK_TIM_OFFSET, delay, sizeof(uint8_t)); // In symbols

    err = os_mutex_release(&inst->mutex);  
    assert(err == OS_OK);

    dw1000_set_autoack(inst, true);

    return inst->status;
}


/**
 * Wait-for-Response turn-around Time. This 20-bit field is used to configure the turn-around time between TX complete 
 * and RX enable when the wait for response function is being used. This function is enabled by the WAIT4RESP control in 
 * Register file: 0x0D – System Control Register. The time specified by this W4R_TIM parameter is in units of approximately 1 μs, 
 * or 128 system clock cycles. This configuration may be used to save power by delaying the turn-on of the receiver, 
 * to align with the response time of the remote system, rather than turning on the receiver immediately after transmission completes.
 * 
 * @param inst   Pointer to _dw1000_dev_instance_t.
 * @param delay  The delay is in UWB microseconds.
 *
 * @return dw1000_dev_status_t
 *
 */
struct _dw1000_dev_status_t 
dw1000_set_wait4resp_delay(struct _dw1000_dev_instance_t * inst, uint32_t delay)
{    
    dw1000_dev_control_t control = inst->control;
    os_error_t err = os_mutex_pend(&inst->mutex,  OS_TIMEOUT_NEVER); // Block if request pending
    assert(err == OS_OK);
    
    control.wait4resp_delay_enabled = delay > 0;
    if (control.wait4resp_delay_enabled){
        uint32_t ack_resp_reg = dw1000_read_reg(inst, ACK_RESP_T_ID, 0, sizeof(uint32_t)) ; // Read ACK_RESP_T_ID register
        ack_resp_reg &= ~(ACK_RESP_T_W4R_TIM_MASK) ;        // Clear the timer (19:0)
        ack_resp_reg |= (delay & ACK_RESP_T_W4R_TIM_MASK) ; // In UWB microseconds (e.g. turn the receiver on 20uus after TX)
        dw1000_write_reg(inst, ACK_RESP_T_ID, 0, ack_resp_reg, sizeof(uint32_t));
    }
    err = os_mutex_release(&inst->mutex);  
    assert(err == OS_OK);
    
    return inst->status;
}


/**
 * API to enable the double receive buffer mode.
 *
 * @param inst    Pointer to _dw1000_dev_instance_t.
 * @param enable  1 to enable, 0 to disable the double buffer mode.
 *
 * @return dw1000_dev_status_t
 *
 */
struct _dw1000_dev_status_t 
dw1000_set_dblrxbuff(struct _dw1000_dev_instance_t * inst, bool enable)
{
    os_error_t err = os_mutex_pend(&inst->mutex,  OS_TIMEOUT_NEVER); // Block if request pending
    assert(err == OS_OK);

    uint32_t sys_cfg_reg = SYS_CFG_MASK & dw1000_read_reg(inst, SYS_CFG_ID, 0, sizeof(uint32_t)); 

    inst->config.dblbuffon_enabled = enable;
    if(inst->config.dblbuffon_enabled)
        sys_cfg_reg &= ~SYS_CFG_DIS_DRXB;
    else
        sys_cfg_reg |= SYS_CFG_DIS_DRXB;
    dw1000_write_reg(inst, SYS_CFG_ID, 0, sys_cfg_reg, sizeof(uint32_t));
    
    dw1000_sync_rxbufptrs(inst);
    
    err = os_mutex_release(&inst->mutex);       // Read modify write critical section exit
    assert(err == OS_OK);
    
    return inst->status;
}

/**
 * API for reading carrier integrator value
 *
 * @brief This is used to read the RX carrier integrator value 
 * (relating to the frequency offset of the TX node)
 *
 * NOTE: This is a 21-bit signed quantity, the function sign extends the most 
 *       significant bit, which is bit #20 (numbering from bit zero) to return 
 *       a 32-bit signed integer value.
 *
 * @param inst          Pointer to _dw1000_dev_instance_t.
 *
 * @return int32_t the signed carrier integrator value.
 *                 A positive value means the local RX clock is running faster than the remote TX device.
 */
int32_t
dw1000_read_carrier_integrator(struct _dw1000_dev_instance_t * inst)
{
#define B20_SIGN_EXTEND_TEST (0x00100000UL)
#define B20_SIGN_EXTEND_MASK (0xFFF00000UL)
    uint32_t  regval=0;
    /* Read 3 bytes (21-bit quantity) */
    regval = dw1000_read_reg(inst, DRX_CONF_ID, DRX_CARRIER_INT_OFFSET, DRX_CARRIER_INT_LEN);

    /* Check for a negative number */
    if (regval & B20_SIGN_EXTEND_TEST) {
        /* sign extend bit #20 to whole word */
        regval |= B20_SIGN_EXTEND_MASK;
    } else {
        /* make sure upper bits are clear if not sign extending */
        regval &= DRX_CARRIER_INT_MASK;
    }
    /* cast unsigned value to signed quantity */
    return (int32_t) regval;
}

/**
 * API for calculating the clock offset ratio from the carrior integrator value
 *
 * @param inst Pointer to _dw1000_dev_instance_t.
 * @param integrator_val carrier integrator value
 *
 * @return float   the relative clock offset ratio
 */
float
dw1000_calc_clock_offset_ratio(struct _dw1000_dev_instance_t * inst, int32_t integrator_val)
{
    float fom = DWT_FREQ_OFFSET_MULTIPLIER;
    float hz_to_ppm;
    if (inst->config.dataRate == DWT_BR_110K) {
        fom = DWT_FREQ_OFFSET_MULTIPLIER_110KB;
    }
    
    switch ( inst->config.channel ) {
    case 1: hz_to_ppm = DWT_HZ_TO_PPM_MULTIPLIER_CHAN_1;break;
    case 2: hz_to_ppm = DWT_HZ_TO_PPM_MULTIPLIER_CHAN_2;break;
    case 3: hz_to_ppm = DWT_HZ_TO_PPM_MULTIPLIER_CHAN_3;break;
    case 4: hz_to_ppm = DWT_HZ_TO_PPM_MULTIPLIER_CHAN_4;break;
    case 5: hz_to_ppm = DWT_HZ_TO_PPM_MULTIPLIER_CHAN_5;break;
    case 7: hz_to_ppm = DWT_HZ_TO_PPM_MULTIPLIER_CHAN_7;break;
    default: assert(0);
    }
    
    return integrator_val * (fom * hz_to_ppm / 1.0e6);
}

/**
 * API to read the RX signal quality diagnostic data.
 *
 * @param inst          Pointer to _dw1000_dev_instance_t.
 * @param diagnostics   Diagnostic structure pointer, this will contain the diagnostic data read from the DW1000.
 * @return void
 */
void 
dw1000_read_rxdiag(struct _dw1000_dev_instance_t * inst, struct _dw1000_dev_rxdiag_t * diag)
{  
    /* Read several of the diag parameters together, requires that the struct parameters are in the 
     * same order as the registers */
    dw1000_read(inst, RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET, (uint8_t*)&diag->rx_time, sizeof(diag->rx_time));
    dw1000_read(inst, RX_FQUAL_ID, 0, (uint8_t*)&diag->rx_fqual, sizeof(diag->rx_fqual));
    diag->pacc_cnt =  (dw1000_read_reg(inst, RX_FINFO_ID, 0, sizeof(uint32_t)) & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT;
    // diag->pacc_cnt_nosat =  (dw1000_read_reg(inst, DRX_CONF_ID, RPACC_NOSAT_OFFSET, sizeof(uint16_t)) & RPACC_NOSAT_MASK);
}


/**
 * The DW1000 processing of interrupts in a task context instead of the interrupt context such that other interrupts 
 * and high priority tasks are not blocked waiting for the interrupt handler to complete processing. 
 * This dw1000 softstack needs to coexists with other stacks and sensors interfaces. Use directive DW1000_DEV_TASK_PRIO to defined 
 * the priority of the dw1000_softstack at compile time.
 *
 * @param inst  Pointer to _dw1000_dev_instance_t.
 *
 * @return void
 */
void 
dw1000_tasks_init(struct _dw1000_dev_instance_t * inst)
{
    /* Check if the tasks are already initiated */
    if (!os_eventq_inited(&inst->eventq))
    {
        /* Use a dedicate event queue for timer and interrupt events */
        os_eventq_init(&inst->eventq);
        /*
         * Create the task to process timer and interrupt events from the
         * my_timer_interrupt_eventq event queue.
         */
        inst->interrupt_ev.ev_cb = dw1000_interrupt_ev_cb;
        inst->interrupt_ev.ev_arg = (void *)inst;

        os_task_init(&inst->task_str, "dw1000_irq",
                     dw1000_interrupt_task,
                     (void *) inst,
                     inst->task_prio, OS_WAIT_FOREVER,
                     inst->task_stack,
                     DW1000_DEV_TASK_STACK_SZ);
        /* Enable pull-down on IRQ to not get spurious interrupts when dw1000 is sleeping */
        hal_gpio_irq_init(inst->irq_pin, dw1000_irq, inst, HAL_GPIO_TRIG_RISING, HAL_GPIO_PULL_DOWN);
        hal_gpio_irq_enable(inst->irq_pin);
    }    
    dw1000_phy_interrupt_mask(inst,          SYS_MASK_MCPLOCK | SYS_MASK_MRXDFR | SYS_MASK_MLDEERR |  SYS_MASK_MTXFRS  | SYS_MASK_ALL_RX_TO   | SYS_MASK_ALL_RX_ERR | SYS_MASK_MTXBERR, false);
    dw1000_write_reg(inst, SYS_STATUS_ID, 0, SYS_STATUS_CPLOCK| SYS_STATUS_RXDFR | SYS_STATUS_LDEERR | SYS_STATUS_TXFRS | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXBERR, sizeof(uint32_t)); // Clear SLP2INIT event bits
    dw1000_phy_interrupt_mask(inst,          SYS_MASK_MCPLOCK | SYS_MASK_MRXDFR | SYS_MASK_MLDEERR | SYS_MASK_MTXFRS  | SYS_MASK_ALL_RX_TO   | SYS_MASK_ALL_RX_ERR | SYS_MASK_MTXBERR, true);
}


/**
 * API for the interrupt request. 
 *
 * @param arg  Pointer to the queue of interrupts.
 * @return void
 */
static void 
dw1000_irq(void *arg){
    dw1000_dev_instance_t * inst = arg;
    os_eventq_put(&inst->eventq, &inst->interrupt_ev);   
}

/**
 * API to execute each of the interrupt in queue.
 *
 * @param arg  Pointer to the queue of interrupts.
 * @return void
 */
static void 
dw1000_interrupt_task(void *arg)
{
    dw1000_dev_instance_t * inst = arg;
    while (1) {
        os_eventq_run(&inst->eventq);
    }
}


/**
 * API to register extension  callbacks for different services.
 *
 * @param inst       Pointer to dw1000_dev_instance_t.
 * @param callbacks  callback instance.
 * @return void
 */
void
dw1000_mac_append_interface(dw1000_dev_instance_t* inst, dw1000_mac_interface_t * cbs){
    assert(inst);

    if (cbs == NULL){
        cbs = (dw1000_mac_interface_t * )malloc(sizeof(dw1000_mac_interface_t));
        assert(cbs);
        memset(cbs, 0, sizeof(dw1000_mac_interface_t));
        cbs->status.selfmalloc = true;
    }else{
        cbs->status.initialized = true;
    }

    if(!(SLIST_EMPTY(&inst->interface_cbs))){
        dw1000_mac_interface_t * prev_cbs = NULL;
        dw1000_mac_interface_t * cur_cbs = NULL;
        SLIST_FOREACH(cur_cbs, &inst->interface_cbs, next){
            prev_cbs = cur_cbs;
        }
        SLIST_INSERT_AFTER(prev_cbs, cbs, next);
    }else
        SLIST_INSERT_HEAD(&inst->interface_cbs, cbs, next);
}


/**
 * API to remove specified callbacks.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @param id    ID of the service.
 * @return void
 */
void
dw1000_mac_remove_interface(dw1000_dev_instance_t * inst, dw1000_extension_id_t id){
    assert(inst);
    dw1000_mac_interface_t * cbs = NULL;
    SLIST_FOREACH(cbs, &inst->interface_cbs, next){
        if(cbs->id == id){
            SLIST_REMOVE(&inst->interface_cbs, cbs, _dw1000_mac_interface_t, next);
            break;
        }
    }
    if(cbs != NULL && cbs->status.selfmalloc)
        free(cbs); 
}

/**
 * API to return specified callbacks.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @param id    ID of the service.
 * @return dw1000_mac_interface_t * cbs
 */
dw1000_mac_interface_t *
dw1000_mac_get_interface(dw1000_dev_instance_t * inst, dw1000_extension_id_t id){
    assert(inst);
    dw1000_mac_interface_t * cbs = NULL;
    SLIST_FOREACH(cbs, &inst->interface_cbs, next){
        if(cbs->id == id){
            break;
        }
    }
    return cbs;
}
 
/**
 * This is the DW1000's general Interrupt Service Routine. It will process/report the following events:
 *          - RXFCG (through rx_complete_cb callback)
 *          - TXFRS (through tx_complete_cb callback)
 *          - RXRFTO/RXPTO (through rx_timeout_cb callback)
 *          - RXPHE/RXFCE/RXRFSL/RXSFDTO/AFFREJ/LDEERR (through rx_error_cb cbRxErr)
 * For all events, corresponding interrupts are cleared and necessary resets are performed. In addition, in the RXFCG case,
 * received frame information and frame control are read before calling the callback. If double buffering is activated, it
 * will also toggle between reception buffers once the reception callback processing has ended.
 *
 * @param ev  Pointer to the queue of events.
 * @return void
 * 
 */
static void 
dw1000_interrupt_ev_cb(struct os_event *ev)
{
    dw1000_dev_instance_t * inst = ev->ev_arg;

    inst->sys_status = dw1000_read_reg(inst, SYS_STATUS_ID, 0, sizeof(uint32_t)); // Read status register low 32bits
    //printf("inst->sys_status= %lX\n",inst->sys_status);

    // Set status flags
    inst->status.rx_error = (inst->sys_status & SYS_STATUS_ALL_RX_ERR) !=0;
    inst->status.rx_timeout_error = (inst->sys_status & SYS_STATUS_ALL_RX_TO) !=0;
    inst->status.lde_error = (inst->sys_status & SYS_STATUS_LDEDONE) == 0;
    inst->status.overrun_error = (inst->sys_status & SYS_STATUS_RXOVRR) != 0;
    inst->status.txbuf_error = (inst->sys_status & SYS_STATUS_TXBERR) != 0;

    if(os_sem_get_count(&inst->tx_sem) == 0){
            os_error_t err = os_sem_release(&inst->tx_sem);  
            assert(err == OS_OK); 
    }
    
      // leading edge detection complete
    if((inst->sys_status & SYS_STATUS_RXFCG)){
        MAC_STATS_INC(DFR_cnt);

        uint16_t finfo = dw1000_read_reg(inst, RX_FINFO_ID, RX_FINFO_OFFSET, sizeof(uint16_t));     // Read frame info - Only the first two bytes of the register are used here.
        inst->frame_len = (finfo & RX_FINFO_RXFL_MASK_1023) - 2;          // Report frame length - Standard frame length up to 127, extended frame length up to 1023 bytes
        
        if (inst->status.overrun_error){
            MAC_STATS_INC(ROV_err);
            /* Overrun flag has been set */
            dw1000_write_reg(inst, SYS_STATUS_ID, 0, SYS_STATUS_RXOVRR, sizeof(uint32_t));
            dw1000_phy_forcetrxoff(inst);
            dw1000_phy_rx_reset(inst);
            if (inst->control.on_error_continue_enabled) 
                dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, SYS_CTRL_RXENAB, sizeof(uint16_t));
            return;
        }

        // The DW1000 has a bug that render the hardware auto_enable feature useless when used in conjunction with the double buffering. 
        // Consequently, we reenable the transeiver in the MAC-layer as early as possable. Note: The default behavior of MAC-Layer 
        // is that the transceiver only returns to the IDLE state with a timeout event occured. The MAC-layer should otherwise reenable.

        if (inst->config.rxauto_enable == 0 && inst->config.dblbuffon_enabled) 
            dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, SYS_CTRL_RXENAB, sizeof(uint16_t));
        
        assert(inst->frame_len < sizeof(inst->rxbuf));
        if (inst->frame_len < sizeof(inst->rxbuf))
            dw1000_read_rx(inst, inst->rxbuf, 0, inst->frame_len);            // Read the whole frame
        
        inst->fctrl = ((ieee_rng_request_frame_t * ) inst->rxbuf)->fctrl; 

        if (inst->status.lde_error) // retest lde_error condition
            inst->status.lde_error = (dw1000_read_reg(inst, SYS_STATUS_ID, 1, sizeof(uint8_t))  & (SYS_STATUS_LDEDONE >> 8)) == 0;
        if (inst->status.lde_error) // LDE eror or LDE late
            MAC_STATS_INC(LDE_err);
        
        inst->rxtimestamp = dw1000_read_rxtime(inst);  
       
        // Because of a previous frame not being received properly, AAT bit can be set upon the proper reception of a frame not requesting for
        // acknowledgement (ACK frame is not actually sent though). If the AAT bit is set, check ACK request bit in frame control to confirm (this
        // implementation works only for IEEE802.15.4-2011 compliant frames).
        // This issue is not documented at the time of writing this code. It should be in next release of DW1000 User Manual (v2.09, from July 2016).

        if((inst->sys_status & SYS_STATUS_AAT) && ((inst->fctrl & MAC_FTYPE_ACK) == 0)){
            dw1000_write_reg(inst, SYS_STATUS_ID, 0, SYS_STATUS_AAT, sizeof(uint8_t));     // Clear AAT status bit in register
            inst->sys_status &= ~SYS_STATUS_AAT; // Clear AAT status bit in callback data register copy
        }
        // Collect RX Frame Quality diagnositics
        if(inst->config.rxdiag_enable)  
            dw1000_read_rxdiag(inst, &inst->rxdiag);
        
          // Toggle the Host side Receive Buffer Pointer
        if (inst->config.dblbuffon_enabled) {
            inst->status.overrun_error = dw1000_checkoverrun(inst);
            if (inst->status.overrun_error == 0){ 
                 uint8_t mask = dw1000_read_reg(inst, SYS_MASK_ID, 1 , sizeof(uint8_t)) ;  
                 dw1000_write_reg(inst, SYS_MASK_ID, 1, 0, sizeof(uint8_t));       
                 dw1000_write_reg(inst, SYS_STATUS_ID, 1, (SYS_STATUS_LDEDONE | SYS_STATUS_RXDFR | SYS_STATUS_RXFCG | SYS_STATUS_RXFCE | SYS_STATUS_RXDFR)>>8, sizeof(uint8_t)); 
                 dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET , 0b1, sizeof(uint8_t)); 
                 dw1000_write_reg(inst, SYS_MASK_ID, 1, mask, sizeof(uint8_t)); 
            }else{
                MAC_STATS_INC(ROV_err);
                /* Overrun flag has been set */
                dw1000_write_reg(inst, SYS_STATUS_ID, 0, SYS_STATUS_RXOVRR, sizeof(uint32_t));
                dw1000_phy_forcetrxoff(inst);
                dw1000_phy_rx_reset(inst);
                if (inst->control.on_error_continue_enabled) 
                    dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, SYS_CTRL_RXENAB, sizeof(uint16_t));
            }
        }else{
            // carrier_integrator only avialble while in single buffer mode.
            inst->carrier_integrator = dw1000_read_carrier_integrator(inst);
#if MYNEWT_VAL(CIR_ENABLED) || MYNEWT_VAL(PMEM_ENABLED) 
            // Call CIR complete calbacks if present
            dw1000_mac_interface_t * cbs = NULL;
            if(!(SLIST_EMPTY(&inst->interface_cbs))){ 
                SLIST_FOREACH(cbs, &inst->interface_cbs, next){    
                if (cbs != NULL && cbs->cir_complete_cb) 
                    if(cbs->cir_complete_cb(inst,cbs)) break;
                }   
            }  
#endif
            dw1000_write_reg(inst, SYS_STATUS_ID, 0, (SYS_STATUS_LDEDONE | SYS_STATUS_RXDFR | SYS_STATUS_RXFCG | SYS_STATUS_RXFCE | SYS_STATUS_RXDFR), sizeof(uint16_t)); 
            dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, SYS_CTRL_RXENAB, sizeof(uint16_t));
        }
        
        // Call the corresponding frame services callback if present
        dw1000_mac_interface_t * cbs = NULL;
        if(!(SLIST_EMPTY(&inst->interface_cbs))){ 
            SLIST_FOREACH(cbs, &inst->interface_cbs, next){    
            if (cbs != NULL && cbs->rx_complete_cb) 
                if(cbs->rx_complete_cb(inst,cbs)) break;
            }   
        }  
    }

    // Handle TX confirmation event
    if(inst->sys_status & SYS_STATUS_TXFRS){
        MAC_STATS_INC(TFG_cnt);
        dw1000_write_reg(inst, SYS_STATUS_ID, 0, SYS_STATUS_ALL_TX, sizeof(uint32_t)); // Clear TX event bits
        // In the case where this TXFRS interrupt is due to the automatic transmission of an ACK solicited by a response (with ACK request bit set)
        // that we receive through using wait4resp to a previous TX (and assuming that the IRQ processing of that TX has already been handled), then
        // we need to handle the IC issue which turns on the RX again in this situation (i.e. because it is wrongly applying the wait4resp after the
        // ACK TX).
        // See section "Transmit and automatically wait for response" in DW1000 User Manual

        if((inst->sys_status & SYS_STATUS_AAT) && inst->control.wait4resp_enabled){
            dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8_t) SYS_CTRL_TRXOFF, sizeof(uint8_t)); // return to idle state
            dw1000_phy_forcetrxoff(inst);
            dw1000_phy_rx_reset(inst);      // Reset in case we were late and a frame was already being received
        }

        if(os_sem_get_count(&inst->tx_sem) == 0){
            os_error_t err = os_sem_release(&inst->tx_sem);  
            assert(err == OS_OK); 
        }
        
        // Call the corresponding callback if present
        dw1000_mac_interface_t * cbs = NULL;
        if(!(SLIST_EMPTY(&inst->interface_cbs))){ 
            SLIST_FOREACH(cbs, &inst->interface_cbs, next){    
            if (cbs!=NULL && cbs->tx_complete_cb) 
                if(cbs->tx_complete_cb(inst,cbs)) break;
            }   
        }          
    }
    // Tx buffer error
    if(inst->status.txbuf_error){
        MAC_STATS_INC(TXBUF_err);
        dw1000_write_reg(inst, SYS_STATUS_ID, 0, SYS_STATUS_TXBERR, sizeof(uint32_t)); 
        if(os_sem_get_count(&inst->tx_sem) == 0){
            os_error_t err = os_sem_release(&inst->tx_sem);  
            assert(err == OS_OK); 
        }
    }

    // leading edge detection complete
    if(inst->sys_status & SYS_STATUS_LDEERR){
        MAC_STATS_INC(LDE_err);
        dw1000_write_reg(inst, SYS_STATUS_ID, 0, SYS_STATUS_LDEERR, sizeof(uint32_t)); 
    }

    // Handle frame reception/preamble detect timeout events
    if(inst->status.rx_timeout_error){
        MAC_STATS_INC(RTO_cnt);
        dw1000_write_reg(inst, SYS_STATUS_ID, 0, SYS_STATUS_ALL_RX_TO, sizeof(uint32_t)); // Clear RX timeout event bits        
        // Because of an issue with receiver restart after error conditions, an RX reset must be applied 
        // after any error or timeout event to ensure the next good frame's timestamp is computed correctly.
        // See section "RX Message timestamp" in DW1000 User Manual.
        dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint16_t)SYS_CTRL_TRXOFF, sizeof(uint16_t)) ; // Disable the radio
        dw1000_phy_rx_reset(inst);

        // Call the corresponding frame services callback if present
        dw1000_mac_interface_t * cbs = NULL;
        if(!(SLIST_EMPTY(&inst->interface_cbs))){ 
            SLIST_FOREACH(cbs, &inst->interface_cbs, next){    
            if (cbs!=NULL && cbs->rx_timeout_cb) 
                if(cbs->rx_timeout_cb(inst,cbs)) continue; 
            }   
        }      
    }

    // Handle RX errors events
    if(inst->status.rx_error){
        MAC_STATS_INC(RX_err);
        dw1000_write_reg(inst, SYS_STATUS_ID, 0, (SYS_STATUS_ALL_RX_ERR), sizeof(uint32_t)); // Clear RX error event bits
        // Because of an issue with receiver restart after error conditions, an RX reset must be applied after any error or timeout event to ensure
        // the next good frame's timestamp is computed correctly.
        // See section "RX Message timestamp" in DW1000 User Manual.
        dw1000_phy_forcetrxoff(inst);
        dw1000_phy_rx_reset(inst);

        // Restart the receiver in the event if rxauto is not enabled. Timeout remain active if set.
        if (inst->config.rxauto_enable == 0) //&& (inst->sys_status & SYS_STATUS_RXPHE))
            dw1000_write_reg(inst, SYS_CTRL_ID, SYS_CTRL_OFFSET, SYS_CTRL_RXENAB, sizeof(uint16_t));

        // Call the corresponding frame services callback if present
        dw1000_mac_interface_t * cbs = NULL;
        if(!(SLIST_EMPTY(&inst->interface_cbs))){ 
            SLIST_FOREACH(cbs, &inst->interface_cbs, next){    
            if (cbs!=NULL && cbs->rx_error_cb) 
                if(cbs->rx_error_cb(inst,cbs)) continue;         
            }   
        }      
    }

     // Handle sleep timer event
    if(inst->sys_status & SYS_STATUS_CLKPLL_LL){
        dw1000_write_reg(inst, SYS_STATUS_ID, 0, SYS_STATUS_CLKPLL_LL, sizeof(uint32_t)); // Clear SLP2INIT event bits
    }
    // Handle sleep timer event
    if(inst->sys_status & SYS_MASK_MCPLOCK){
        dw1000_write_reg(inst, SYS_STATUS_ID, 0, SYS_MASK_MCPLOCK, sizeof(uint32_t)); // Clear SLP2INIT event bits

        // restore antenna delay value, these are not preserved during sleep/deepsleep */
        dw1000_phy_set_rx_antennadelay(inst, inst->rx_antenna_delay);
        dw1000_phy_set_tx_antennadelay(inst, inst->tx_antenna_delay);

        // Call the corresponding callback if present
        inst->status.sleeping = 0;
        dw1000_mac_interface_t * cbs = NULL;
        if(!(SLIST_EMPTY(&inst->interface_cbs))){ 
            SLIST_FOREACH(cbs, &inst->interface_cbs, next){    
            if (cbs!=NULL && cbs->sleep_cb) 
                if (cbs->sleep_cb(inst,cbs)) continue; 
            }   
        }         
        return;
    }
}


/**
 * API to calculate First Path Power Level (fppl) from an rxdiag structure
 *
 * @param inst  Pointer to _dw1000_dev_instance_t.
 * @param diag  Pointer to _dw1000_dev_rxdiag_t.
 *
 * @return fppl on success
 */
float
dw1000_calc_fppl(struct _dw1000_dev_instance_t * inst,
                 struct _dw1000_dev_rxdiag_t * diag)
{
    if (diag->pacc_cnt == 0 ||
        (!diag->fp_amp && !diag->fp_amp2 && !diag->fp_amp3)) {
        return -INFINITY;
    }
    float A = (inst->config.prf == DWT_PRF_16M) ? 113.77 : 121.74;

    float N = diag->pacc_cnt;
    float v = (float)diag->fp_amp*diag->fp_amp +
        (float)diag->fp_amp2*diag->fp_amp2 +
        (float)diag->fp_amp3*diag->fp_amp3;
    v /= N*N;
    return 10.0*log10f(v) - A;
}

/**
 * API to calculate First Path Power Level from last RX in dBm,
 * which needs config.rxdiag_enable to be set.
 *
 * @param inst  Pointer to _dw1000_dev_instance_t.
 *
 * @return rssi on success
 */
float
dw1000_get_fppl(struct _dw1000_dev_instance_t * inst)
{
    if (!inst->config.rxdiag_enable)
        return -INFINITY;
    return dw1000_calc_fppl(inst, &inst->rxdiag);
}

/**
 * API to calculate rssi from an rxdiag structure
 *
 * @param inst  Pointer to _dw1000_dev_instance_t.
 * @param diag  Pointer to _dw1000_dev_rxdiag_t.
 *
 * @return rssi on success
 */
float
dw1000_calc_rssi(struct _dw1000_dev_instance_t * inst,
                 struct _dw1000_dev_rxdiag_t * diag)
{
    if (diag->cir_pwr == 0 || diag->pacc_cnt == 0) {
        return -INFINITY;
    }
    float rssi = 10.0f * log10f(diag->cir_pwr * 0x20000/(diag->pacc_cnt * diag->pacc_cnt))
        - ((inst->config.prf == DWT_PRF_16M) ? 115.72 : 122.74);
    return rssi;
}

/**
 * API to calculate rssi from last RX in dBm, which needs config.rxdiag_enable to be set.
 *
 * @param inst  Pointer to _dw1000_dev_instance_t.
 *
 * @return rssi on success
 */
float
dw1000_get_rssi(struct _dw1000_dev_instance_t * inst)
{
    if (!inst->config.rxdiag_enable) 
        return -INFINITY;
    return dw1000_calc_rssi(inst, &inst->rxdiag);
}

/**
 * API to give a rough estimate of how likely the received packet is
 * line of sight (LOS). Taken from 4.7 of DW1000 manual.
 *
 * @param rssi rssi as calculated by dw1000_calc_rssi
 * @param fppl fppl as calculated by dw1000_calc_fppl
 *
 * @return 1.0 for likely LOS, 0.0 for non-LOS, with a sliding scale in between.
 */
float
dw1000_estimate_los(float rssi, float fppl)
{
    float d = fabs(rssi-fppl);
    if (d<6)  return 1.0;       /* Less than 6dB difference - LOS */
    if (d>10) return 0.0;       /* More than 10dB difference - NLOS */
    return 1.0 - (d-6)/4.0;
}


/**
 * With ADAPTIVE_TIMESCALE_ENABLED all time local clock are adjusted to master clock frequency. 
 * The compensated local clock value is offset from the master clock, but the frequency is adjusted 
 * such that any derived time differences values will be the same. WCS_ENABLE && ADAPTIVE_TIMESCALE_ENABLED  
 * is usefull for TDOA and Single-Sided TWR applciaitons. 
 *
 * @param inst  Pointer to _dw1000_dev_instance_t. 
 * @return time
 */
inline uint64_t dw1000_read_systime(struct _dw1000_dev_instance_t * inst){
    uint64_t time = ((uint64_t) dw1000_read_reg(inst, SYS_TIME_ID, SYS_TIME_OFFSET, SYS_TIME_LEN)) & 0x0FFFFFFFFFFULL;
    return time;
}


/**
 * API to read system time at lower offset address.
 *
 * @param inst  Pointer to _dw1000_dev_instance_t.
 * 
 * @return time 
 */
inline uint32_t dw1000_read_systime_lo(struct _dw1000_dev_instance_t * inst){
    uint32_t time = (uint32_t) dw1000_read_reg(inst, SYS_TIME_ID, SYS_TIME_OFFSET, sizeof(uint32_t));    
    return time;
}

/**
 * API to read receive time.
 *
 * @param inst  Pointer to _dw1000_dev_instance_t.
 *
 * @return time
 */

inline uint64_t dw1000_read_rxtime(struct _dw1000_dev_instance_t * inst){
    uint64_t time = (uint64_t)  dw1000_read_reg(inst, RX_TIME_ID, RX_TIME_RX_STAMP_OFFSET, RX_TIME_RX_STAMP_LEN) & 0x0FFFFFFFFFFULL;
    return time;
}


/**
 * API to read receive time at lower offset address.
 *
 * @param inst  Pointer to _dw1000_dev_instance_t.
 *
 * @return time
 */
inline uint32_t dw1000_read_rxtime_lo(struct _dw1000_dev_instance_t * inst){
    uint64_t time = (uint32_t) dw1000_read_reg(inst, RX_TIME_ID, RX_TIME_RX_STAMP_OFFSET, sizeof(uint32_t));
    return time;
}

/**
 * API to read transmission time.
 *
 * @param inst  Pointer to _dw1000_dev_instance_t. 
 *
 * @return time
 * 
 */
inline uint64_t dw1000_read_txrawst(struct _dw1000_dev_instance_t * inst){
    uint64_t time = (uint64_t) dw1000_read_reg(inst, TX_TIME_ID, TX_TIME_TX_RAWST_OFFSET, TX_TIME_TX_STAMP_LEN) & 0x0FFFFFFFFFFULL;
    return time;
}

/**
 * API to read transmission time.
 *
 * @param inst  Pointer to _dw1000_dev_instance_t. 
 *
 * @return time
 * 
 */
inline uint64_t dw1000_read_txtime(struct _dw1000_dev_instance_t * inst){
    uint64_t time = (uint64_t) dw1000_read_reg(inst, TX_TIME_ID, TX_TIME_TX_STAMP_OFFSET, TX_TIME_TX_STAMP_LEN) & 0x0FFFFFFFFFFULL;
    return time;
}

/**
 * API to read transmit time at lower offset address.
 *
 * @param inst  Pointer to _dw1000_dev_instance_t.
 *
 * @return time
 */
inline uint32_t dw1000_read_txtime_lo(struct _dw1000_dev_instance_t * inst){
    uint32_t time = (uint32_t) dw1000_read_reg(inst, TX_TIME_ID, TX_TIME_TX_STAMP_OFFSET, sizeof(uint32_t));
    return time;
}

/**
 * @fn dwt_configcwmode()
 *
 * @brief this function sets the DW1000 to transmit cw signal at specific channel
 * frequency.
 *
 * input parameters:
 * @param chan - specifies the operating channel (e.g. 1, 2, 3, 4, 5, 6 or 7)
 *
 */
void
dw1000_configcwmode(struct _dw1000_dev_instance_t * inst, uint8_t chan)
{
    if ((chan < 1) || (chan > 7) || (6 == chan))
    {
        assert(0);
    }

    /* disable TX/RX RF block sequencing (needed for cw frame mode) */
    dw1000_phy_sysclk_XTAL(inst);
    dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL1_OFFSET,
                     PMSC_CTRL1_PKTSEQ_DISABLE, sizeof(uint16_t));

    /* config RF pll (for a given channel) */
    /* configure PLL2/RF PLL block CFG */
    dw1000_write_reg(inst, FS_CTRL_ID, FS_PLLCFG_OFFSET,
                     fs_pll_cfg[chan_idx[chan]], sizeof(uint32_t));

    /* Configure RF TX blocks (for specified channel and prf) */
    /* Config RF TX control */
    dw1000_write_reg(inst, RF_CONF_ID, RF_TXCTRL_OFFSET,
                     tx_config[chan_idx[chan]], sizeof(uint32_t));

    /* enable RF PLL */
    dw1000_write_reg(inst, RF_CONF_ID, 0, RF_CONF_TXPLLPOWEN_MASK, sizeof(uint32_t));
    dw1000_write_reg(inst, RF_CONF_ID, 0, RF_CONF_TXALLEN_MASK, sizeof(uint32_t));

    /* configure TX clocks */
    dw1000_write_reg(inst, PMSC_ID,PMSC_CTRL0_OFFSET, 0x22, 1);
    dw1000_write_reg(inst, PMSC_ID, 0x1, 0x07, 1);

    /* disable fine grain TX seq */
    dw1000_write_reg(inst, PMSC_ID, PMSC_TXFINESEQ_OFFSET,
                     PMSC_TXFINESEQ_DISABLE, sizeof(uint16_t));

    /* configure CW mode */
    dw1000_write_reg(inst, TX_CAL_ID, TC_PGTEST_OFFSET,
                     TC_PGTEST_CW, TC_PGTEST_LEN);
}
