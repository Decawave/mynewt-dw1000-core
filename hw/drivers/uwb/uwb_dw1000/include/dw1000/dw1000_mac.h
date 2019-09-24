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
 * @file dw1000_mac.h
 * @author Paul Kettle
 * @date 2018
 * @brief Mac initialization  
 *      
 * @details This is the mac base class which utilizes the functions to do the configurations related to mac layer based on dependencies.
 *        
 */

#ifndef _DW1000_MAC_H_
#define _DW1000_MAC_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/hal_spi.h>
#include <stats/stats.h>
#include <dw1000/dw1000_dev.h>

#define DWT_DEVICE_ID   (0xDECA0130)        //!< DW1000 MP device ID

//! constants for selecting the bit rate for data TX (and RX).
//! These are defined for write (with just a shift) the TX_FCTRL register.
#define DWT_BR_110K     0   //!< UWB bit rate 110 kbits/s
#define DWT_BR_850K     1   //!< UWB bit rate 850 kbits/s
#define DWT_BR_6M8      2   //!< UWB bit rate 6.8 Mbits/s

//! constants for specifying the (Nominal) mean Pulse Repetition Frequency.
//! These are defined for direct write (with a shift if necessary) to CHAN_CTRL and TX_FCTRL regs.
#define DWT_PRF_16M     1   //!< UWB PRF 16 MHz
#define DWT_PRF_64M     2   //!< UWB PRF 64 MHz

//! constants for specifying Preamble Acquisition Chunk (PAC) Size in symbols.
#define DWT_PAC8        0   //!< PAC  8 (recommended for RX of preamble length  128 and below
#define DWT_PAC16       1   //!< PAC 16 (recommended for RX of preamble length  256
#define DWT_PAC32       2   //!< PAC 32 (recommended for RX of preamble length  512
#define DWT_PAC64       3   //!< PAC 64 (recommended for RX of preamble length 1024 and up

//! constants for specifying TX Preamble length in symbols.
//! These are defined to allow them be directly written into byte 2 of the TX_FCTRL register.
//! (i.e. a four bit value destined for bits 20..18 but shifted left by 2 for byte alignment)
#define DWT_PLEN_4096   0x0C    //! Standard preamble length 4096 symbols
#define DWT_PLEN_2048   0x28    //! Non-standard preamble length 2048 symbols
#define DWT_PLEN_1536   0x18    //! Non-standard preamble length 1536 symbols
#define DWT_PLEN_1024   0x08    //! Standard preamble length 1024 symbols
#define DWT_PLEN_512    0x34    //! Non-standard preamble length 512 symbols
#define DWT_PLEN_256    0x24    //! Non-standard preamble length 256 symbols
#define DWT_PLEN_128    0x14    //! Non-standard preamble length 128 symbols
#define DWT_PLEN_64     0x04    //! Standard preamble length 64 symbols

#define DWT_SFDTOC_DEF              0x1041  //!< Default SFD timeout value
#define DWT_PHRMODE_STD             0x0     //!< standard PHR mode
#define DWT_PHRMODE_EXT             0x3     //!< DW proprietary extended frames PHR mode

//! Multiplication factors to convert carrier integrator value to a frequency offset in Hz
#define DWT_FREQ_OFFSET_MULTIPLIER          (998.4e6/2.0/1024.0/131072.0)
#define DWT_FREQ_OFFSET_MULTIPLIER_110KB    (998.4e6/2.0/8192.0/131072.0)

//! Multiplication factors to convert frequency offset in Hertz to PPM crystal offset
// NB: also changes sign so a positive value means the local RX clock is
// running slower than the remote TX device.
#define DWT_HZ_TO_PPM_MULTIPLIER_CHAN_1     (-1.0e6/3494.4e6)
#define DWT_HZ_TO_PPM_MULTIPLIER_CHAN_2     (-1.0e6/3993.6e6)
#define DWT_HZ_TO_PPM_MULTIPLIER_CHAN_3     (-1.0e6/4492.8e6)
#define DWT_HZ_TO_PPM_MULTIPLIER_CHAN_4     (-1.0e6/4492.8e6)
#define DWT_HZ_TO_PPM_MULTIPLIER_CHAN_5     (-1.0e6/6489.6e6)
#define DWT_HZ_TO_PPM_MULTIPLIER_CHAN_7     (-1.0e6/6489.6e6)

//! Frame filtering configuration options.
#define DWT_FF_NOTYPE_EN            0x000           //!< No frame types allowed (FF disabled)
#define DWT_FF_COORD_EN             0x002           //!< Behave as coordinator (can receive frames with no dest address (PAN ID has to match))
#define DWT_FF_BEACON_EN            0x004           //!< Beacon frames allowed
#define DWT_FF_DATA_EN              0x008           //!< Data frames allowed
#define DWT_FF_ACK_EN               0x010           //!< Ack frames allowed
#define DWT_FF_MAC_EN               0x020           //!< MAC control frames allowed
#define DWT_FF_RSVD_EN              0x040           //!< Reserved frame types allowed


//! DW1000 SLEEP and WAKEUP configuration parameters.
#define DWT_LOADLDO      0x1000                      //!< Load LDO tune value from OTP
#define DWT_LOADUCODE    0x0800                      //!< Load ucode from OTP
#define DWT_PRESRV_SLEEP 0x0100                      //!< PRES_SLEEP  on wakeup preserve sleep bit
#define DWT_LOADOPSET    0x0080                      //!< ONW_L64P    on wakeup load operating parameter set for 64 PSR
#define DWT_CONFIG       0x0040                      //!< ONW_LDC   on wakeup restore (load) the saved configurations (from AON array into HIF)
#define DWT_LOADEUI      0x0008                      //!< ONW_LEUI  on wakeup load EUI
#define DWT_RX_EN        0x0002                      //!< ONW_RX  on wakeup activate reception
#define DWT_TANDV        0x0001                      //!< ONW_RADC  on wakeup run ADC to sample temperature and voltage sensor values

#define DWT_XTAL_EN      0x10                       //!< Keep XTAL running during sleep
#define DWT_WAKE_SLPCNT  0x8                        //!< Wake up after sleep count
#define DWT_WAKE_CS      0x4                        //!< Wake up on chip select
#define DWT_WAKE_WK      0x2                        //!< Wake up on WAKEUP PIN
#define DWT_SLP_EN       0x1                        //!< Enable sleep/deep sleep functionality

//! DW1000 OTP operating parameter set selection.
#define DWT_OPSET_64LEN   0x0          //!< DW1000 OTP operating parameter set selection
#define DWT_OPSET_TIGHT   0x1          //!< DW1000 OTP operating parameter set selection 
#define DWT_OPSET_DEFLT   0x2          //!< DW1000 OTP operating parameter set selection

//! MAC frame format.
#define MAC_FFORMAT_FCTRL 0x0         //!<  MAC frame format - Control parameter selection
#define MAC_FFORMAT_FCTRL_LEN 0x2     //!<  MAC frame format - Length parameter selection
#define MAC_FFORMAT_FTYPE 0           //!<  MAC frame format - Frame type parameter selection
#define MAC_FTYPE_BEACON  0x0         //!<  MAC frame format - BEACON parameter selection
#define MAC_FTYPE_DATA    0x1         //!<  MAC frame format - DATA parameter selection
#define MAC_FTYPE_ACK     0x2         //!<  MAC frame format - ACK parameter selection
#define MAC_FTYPE_COMMAND 0x3         //!<  MAC frame format - COMMAND parameter selection


//! Callback data of mac.
typedef struct _dw1000_mac_cb_data_t {
    uint32_t status;      //!< Initial value of register as ISR is entered
    uint16_t datalength;  //!< Length of frame
    uint8_t  fctrl[2];    //!< Frame control bytes
    uint8_t  rx_flags;    //!< RX frame flags
} dw1000_mac_cb_data_t;

//! Callback type for all events.
typedef void (*dw1000_mac_cb_t)(struct _dw1000_dev_instance_t * inst, const dw1000_mac_cb_data_t *);

//! Mac device parameters.
typedef struct _dw1000_mac_deviceentcnts_t{
    uint16_t PHE ;                    //!< Number of received header errors
    uint16_t RSL ;                    //!< Number of received frame sync loss events
    uint16_t CRCG ;                   //!< Number of good CRC received frames
    uint16_t CRCB ;                   //!< Number of bad CRC (CRC error) received frames
    uint16_t ARFE ;                   //!< Number of address filter errors
    uint16_t OVER ;                   //!< Number of receiver overflows (used in double buffer mode)
    uint16_t SFDTO ;                  //!< SFD timeouts
    uint16_t PTO ;                    //!< Preamble timeouts
    uint16_t RTO ;                    //!< RX frame wait timeouts
    uint16_t TXF ;                    //!< Number of transmitted frames
    uint16_t HPW ;                    //!< Half period warn
    uint16_t TXW ;                    //!< Power up warn
} dw1000_mac_deviceentcnts_t ;


struct uwb_dev_status dw1000_mac_init(struct _dw1000_dev_instance_t * inst, struct uwb_dev_config * config);
struct uwb_dev_status dw1000_mac_config(struct _dw1000_dev_instance_t * inst, struct uwb_dev_config * config);
void dw1000_tasks_init(struct _dw1000_dev_instance_t * inst);
struct uwb_dev_status dw1000_mac_framefilter(struct _dw1000_dev_instance_t * inst, uint16_t enable);
struct uwb_dev_status dw1000_write_tx(struct _dw1000_dev_instance_t * inst,  uint8_t *txFrameBytes, uint16_t txBufferOffset, uint16_t txFrameLength);
struct uwb_dev_status dw1000_read_rx(struct _dw1000_dev_instance_t * inst,  uint8_t *rxFrameBytes, uint16_t rxBufferOffset, uint16_t rxFrameLength);
struct uwb_dev_status dw1000_start_tx(struct _dw1000_dev_instance_t * inst);
struct uwb_dev_status dw1000_set_delay_start(struct _dw1000_dev_instance_t * inst, uint64_t dx_time);
struct uwb_dev_status dw1000_set_wait4resp(struct _dw1000_dev_instance_t * inst, bool enable);
struct uwb_dev_status dw1000_set_wait4resp_delay(struct _dw1000_dev_instance_t * inst, uint32_t delay);
struct uwb_dev_status dw1000_set_on_error_continue(struct _dw1000_dev_instance_t * inst, bool enable);
struct uwb_dev_status dw1000_set_rxauto_disable(struct _dw1000_dev_instance_t * inst, bool disable);
struct uwb_dev_status dw1000_start_rx(struct _dw1000_dev_instance_t * inst);
struct uwb_dev_status dw1000_stop_rx(struct _dw1000_dev_instance_t * inst);
void dw1000_write_tx_fctrl(struct _dw1000_dev_instance_t * inst, uint16_t txFrameLength, uint16_t txBufferOffset);
struct uwb_dev_status dw1000_sync_rxbufptrs(struct _dw1000_dev_instance_t * inst);
struct uwb_dev_status dw1000_read_accdata(struct _dw1000_dev_instance_t * inst, uint8_t *buffer, uint16_t len, uint16_t accOffset);
struct uwb_dev_status dw1000_enable_autoack(struct _dw1000_dev_instance_t * inst, uint8_t delay);
struct uwb_dev_status dw1000_set_dblrxbuff(struct _dw1000_dev_instance_t * inst, bool flag);
void dw1000_set_callbacks(struct _dw1000_dev_instance_t * inst, dw1000_dev_cb_t cb_TxDone, dw1000_dev_cb_t cb_RxOk, dw1000_dev_cb_t cb_RxTo, dw1000_dev_cb_t cb_RxErr);
struct uwb_dev_status dw1000_set_rx_timeout(struct _dw1000_dev_instance_t * inst, uint16_t timeout);
struct uwb_dev_status dw1000_adj_rx_timeout(struct _dw1000_dev_instance_t * inst, uint16_t timeout);

float dw1000_calc_rssi(struct _dw1000_dev_instance_t * inst, struct _dw1000_dev_rxdiag_t * diag);
float dw1000_get_rssi(struct _dw1000_dev_instance_t * inst);
float dw1000_calc_fppl(struct _dw1000_dev_instance_t * inst, struct _dw1000_dev_rxdiag_t * diag);
float dw1000_get_fppl(struct _dw1000_dev_instance_t * inst);
float dw1000_estimate_los(float rssi, float fppl);
    
int32_t dw1000_read_carrier_integrator(struct _dw1000_dev_instance_t * inst);
float dw1000_calc_clock_offset_ratio(struct _dw1000_dev_instance_t * inst, int32_t integrator_val);
int32_t dw1000_read_time_tracking_offset(struct _dw1000_dev_instance_t * inst);
float dw1000_calc_clock_offset_ratio_ttco(struct _dw1000_dev_instance_t * inst, int32_t ttcko);

void dw1000_read_rxdiag(struct _dw1000_dev_instance_t * inst, struct _dw1000_dev_rxdiag_t * diag);
#define dw1000_set_preamble_timeout(counts) dw1000_write_reg(inst, DRX_CONF_ID, DRX_PRETOC_OFFSET, counts, sizeof(uint16_t))
#define dw1000_set_panid(inst, pan_id) dw1000_write_reg(inst, PANADR_ID, PANADR_PAN_ID_OFFSET, pan_id, sizeof(uint16_t))
#define dw1000_set_address16(inst, shortAddress) dw1000_write_reg(inst ,PANADR_ID, PANADR_SHORT_ADDR_OFFSET, shortAddress, sizeof(uint16_t))
#define dw1000_set_eui(inst, eui64) dw1000_write_reg(inst, EUI_64_ID, EUI_64_OFFSET, eui64, EUI_64_LEN)
#define dw1000_get_eui(inst) (uint64_t) dw1000_read_reg(inst, EUI_64_ID, EUI_64_OFFSET, EUI_64_LEN)

uint64_t dw1000_read_systime(struct _dw1000_dev_instance_t * inst);
uint32_t dw1000_read_systime_lo(struct _dw1000_dev_instance_t * inst);
uint64_t dw1000_read_rxtime(struct _dw1000_dev_instance_t * inst);
uint64_t dw1000_read_rawrxtime(struct _dw1000_dev_instance_t * inst);
uint64_t dw1000_read_txrawst(struct _dw1000_dev_instance_t * inst);
uint32_t dw1000_read_rxtime_lo(struct _dw1000_dev_instance_t * inst);
uint64_t dw1000_read_txtime(struct _dw1000_dev_instance_t * inst);
uint32_t dw1000_read_txtime_lo(struct _dw1000_dev_instance_t * inst);

#define dw1000_get_irqstatus(inst) ((uint8_t) dw1000_read_reg(inst, SYS_STATUS_ID, SYS_STATUS_OFFSET,sizeof(uint8_t)) & (uint8_t) SYS_STATUS_IRQS)

void dw1000_configcwmode(struct _dw1000_dev_instance_t * inst, uint8_t chan);

#ifdef __cplusplus
}
#endif
#endif /* _DW1000_MAC_H_ */
