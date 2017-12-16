/**
 * Copyright (C) 2017-2018, Decawave Limited, All Rights Reserved
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

#ifndef _DW1000_PHY_H_
#define _DW1000_PHY_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/hal_spi.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_otp.h>
#include <dw1000/dw1000_mac.h>

#define PEAK_MULTPLIER  (0x60) //3 -> (0x3 * 32) & 0x00E0
#define N_STD_FACTOR    (13)
#define LDE_PARAM1      (PEAK_MULTPLIER | N_STD_FACTOR)
#define LDE_PARAM3_16 (0x1607)
#define LDE_PARAM3_64 (0x0607)
#define MIXER_GAIN_STEP (0.5)
#define DA_ATTN_STEP    (2.5)

typedef struct _dw1000_phy_txconfig_t {
    uint8_t   PGdly;
    //TX POWER
    //31:24     BOOST_0.125ms_PWR
    //23:16     BOOST_0.25ms_PWR-TX_SHR_PWR
    //15:8      BOOST_0.5ms_PWR-TX_PHR_PWR
    //7:0       DEFAULT_PWR-TX_DATA_PWR
    uint32_t  power;
}dw1000_phy_txconfig_t;

typedef struct _dw1000_phy_rxdiag_t{
    uint16_t    fp_idx;             // First path index (10.6 bits fixed point integer)
    uint16_t    fp_amp;             // Amplitude at floor(index FP) + 1
    uint16_t    rx_std;             // Standard deviation of noise
    uint16_t    preamble_cnt;       // Count of preamble symbols accumulated
}dw1000_phy_rxdiag_t;

dw1000_dev_status_t dw1000_phy_init(dw1000_dev_instance_t * inst);
void dw1000_phy_sysclk_XTAL(dw1000_dev_instance_t * inst);
void dw1000_phy_sysclk_PLL(dw1000_dev_instance_t * inst);
void dw1000_phy_sysclk_SEQ(dw1000_dev_instance_t * inst);
void dw1000_phy_sysclk_ACC(dw1000_dev_instance_t * inst, uint8_t mode);
void dw1000_phy_disable_sequencing(dw1000_dev_instance_t * inst);
void dw1000_phy_delayed_txrxtime(dw1000_dev_instance_t * inst, uint32_t starttime);
void dw1000_phy_config_lde(dw1000_dev_instance_t * inst, int prfIndex);
void dw1000_phy_config_txrf(dw1000_dev_instance_t * inst, dw1000_phy_txconfig_t *config);
void dw1000_phy_read_rxdiag(dw1000_dev_instance_t * inst, dw1000_phy_rxdiag_t * diag);
void dw1000_phy_rx_reset(dw1000_dev_instance_t * inst);
void dw1000_phy_forcetrxoff(dw1000_dev_instance_t * inst);
void dw1000_phy_interrupt_mask(dw1000_dev_instance_t * inst, uint32_t bitmask, uint8_t enable);


#define dw1000_phy_set_rx_antennadelay(inst, rxDelay) dw1000_write_reg(inst, LDE_IF_ID, LDE_RXANTD_OFFSET, rxDelay, sizeof(uint16_t)) // Set the RX antenna delay for auto TX timestamp adjustment
#define dw1000_phy_set_tx_antennadelay(inst, txDelay) dw1000_write_reg(inst, TX_ANTD_ID, TX_ANTD_OFFSET, txDelay, sizeof(uint16_t)) // Set the TX antenna delay for auto TX timestamp adjustment

#define dw1000_phy_read_wakeuptemp(inst) ((uint8_t) dw1000_read_reg(inst, TX_CAL_ID, TC_SARL_SAR_LTEMP_OFFSET, sizeof(uint8_t)))
#define dw1000_phy_read_wakeupvbat(inst) ((uint8_t) dw1000_read_reg(inst, TX_CAL_ID, TC_SARL_SAR_LVBAT_OFFSET, sizeof(uint8_t)))

float dw1000_phy_read_wakeuptemp_SI(dw1000_dev_instance_t * inst);
float dw1000_phy_read_read_wakeupvbat_SI(dw1000_dev_instance_t * inst);

#ifdef __cplusplus
}
#endif

#endif /* _DW1000_PHY_H_ */
