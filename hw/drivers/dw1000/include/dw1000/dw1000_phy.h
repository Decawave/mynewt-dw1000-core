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
 * @file dw1000_phy.h
 * @author paul kettle
 * @date 2018
 * @brief physical layer
 *
 * @details This is the phy base class which utilises the functions to set the clocks,initializes phy layer and configures the required 
 * parameters. 
 */

#ifndef _DW1000_PHY_H_
#define _DW1000_PHY_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/hal_spi.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_gpio.h>
#include <dw1000/dw1000_otp.h>

#define PEAK_MULTPLIER  (0x60) //!<  Peak Multiplier 
#define N_STD_FACTOR    (13)   //!< Number of standard deviation factor
#define LDE_PARAM1      (PEAK_MULTPLIER | N_STD_FACTOR) //!< LDE masking for 8-bit configuration
#define LDE_PARAM3_16 (0x1607)   //!<  LDE masking for 16-bit configuration
#define LDE_PARAM3_64 (0x0607)   //!< LDE masking for 64-bit configuration
#define MIXER_GAIN_STEP (0.5)    //!< TODO
#define DA_ATTN_STEP    (2.5)    //!< TODO

//! Enum of txrf config parameters
typedef enum {
    DW1000_txrf_config_18db = 0,   //!< txrf_config_18db for 18db power gain
    DW1000_txrf_config_15db,       //!< txrf_config_15db for 15db power gain 
    DW1000_txrf_config_12db,       //!< txrf_config_12db for 12db power gain 
    DW1000_txrf_config_9db,        //!< txrf_config_9db for 9db power gain 
    DW1000_txrf_config_6db,        //!< txrf_config_6db for 6db power gain 
    DW1000_txrf_config_3db,        //!< txrf_config_3db for 3db power gain 
    DW1000_txrf_config_0db,        //!< txrf_config_0db for 0db power gain 
    DW1000_txrf_config_off         //!< txrf_config_off
}coarse_power_levels_t;

#define dw1000_power_value(COARSE,FINE) ((COARSE<<5) + FINE)    //!< To configure power values

struct _dw1000_dev_status_t dw1000_phy_init(struct _dw1000_dev_instance_t * inst, struct _dw1000_dev_txrf_config_t * txrf_config);
void dw1000_phy_sysclk_XTAL(struct _dw1000_dev_instance_t * inst);
void dw1000_phy_sysclk_PLL(struct _dw1000_dev_instance_t * inst);
void dw1000_phy_sysclk_SEQ(struct _dw1000_dev_instance_t * inst);
void dw1000_phy_sysclk_ACC(struct _dw1000_dev_instance_t * inst, uint8_t mode);
void dw1000_phy_disable_sequencing(struct _dw1000_dev_instance_t * inst);
void dw1000_phy_config_lde(struct _dw1000_dev_instance_t * inst, int prfIndex);
void dw1000_phy_config_txrf(struct _dw1000_dev_instance_t * inst, struct _dw1000_dev_txrf_config_t * config);
void dw1000_phy_rx_reset(struct _dw1000_dev_instance_t * inst);
void dw1000_phy_forcetrxoff(struct _dw1000_dev_instance_t * inst);
void dw1000_phy_interrupt_mask(struct _dw1000_dev_instance_t * inst, uint32_t bitmask, uint8_t enable);

#define dw1000_phy_set_rx_antennadelay(inst, rxDelay) dw1000_write_reg(inst, LDE_IF_ID, LDE_RXANTD_OFFSET, rxDelay, sizeof(uint16_t)) //!< Set the RX antenna delay for auto TX timestamp adjustment
#define dw1000_phy_set_tx_antennadelay(inst, txDelay) dw1000_write_reg(inst, TX_ANTD_ID, TX_ANTD_OFFSET, txDelay, sizeof(uint16_t)) //!< Set the TX antenna delay for auto TX timestamp adjustment

#define dw1000_phy_read_wakeuptemp(inst) ((uint8_t) dw1000_read_reg(inst, TX_CAL_ID, TC_SARL_SAR_LTEMP_OFFSET, sizeof(uint8_t))) //!< Read the temperature level of the DW1000 that was sampled on waking from Sleep/Deepsleep

#define dw1000_phy_read_wakeupvbat(inst) ((uint8_t) dw1000_read_reg(inst, TX_CAL_ID, TC_SARL_SAR_LVBAT_OFFSET, sizeof(uint8_t))) //!< Read the battery voltage of the DW1000 that was sampled on waking from Sleep/Deepsleep


float dw1000_phy_read_wakeuptemp_SI(struct _dw1000_dev_instance_t * inst);
float dw1000_phy_read_read_wakeupvbat_SI(struct _dw1000_dev_instance_t * inst);
void dw1000_phy_external_sync(struct _dw1000_dev_instance_t * inst, uint8_t delay, bool enable);

uint16_t dw1000_phy_SHR_duration(struct _phy_attributes_t * attrib);
uint16_t dw1000_phy_frame_duration(struct _phy_attributes_t * attrib, uint16_t nlen);

#ifdef __cplusplus
}
#endif

#endif /* _DW1000_PHY_H_ */
