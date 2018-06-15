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

#ifndef _DW1000_DEV_H_
#define _DW1000_DEV_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <os/os_dev.h>
#include <os/os_mutex.h>
#include <hal/hal_spi.h>
#include <dw1000/dw1000_regs.h>

#define DWT_DEVICE_ID   (0xDECA0130) 
#define DWT_SUCCESS (0)
#define DWT_ERROR   (-1)
#define DWT_TIME_UNITS          (1.0/499.2e6/128.0)

#define DW1000_DEV_TASK_PRIO        MYNEWT_VAL(DW1000_DEV_TASK_PRIO)
#define DW1000_DEV_TASK_STACK_SZ    MYNEWT_VAL(DW1000_DEV_TASK_STACK_SZ)

#define BROADCAST_ADDRESS          0xffff

typedef enum _dw1000_dev_modes_t{
    DWT_BLOCKING,
    DWT_NONBLOCKING
}dw1000_dev_modes_t;

typedef enum _dw1000_dev_role_t{
    NODE_0,
    NODE,
    TAG
}dw1000_dev_role_t;

typedef struct _dw1000_cmd{
    uint32_t reg:6;
    uint32_t subindex:1;
    uint32_t operation:1;
    uint32_t extended:1;
    uint32_t subaddress:15;    
}dw1000_cmd_t;

typedef struct _dw1000_dev_status_t{
    uint32_t selfmalloc:1;
    uint32_t initialized:1;
    uint32_t start_tx_error:1;
    uint32_t start_rx_error:1;
    uint32_t tx_frame_error:1;
    uint32_t rx_error:1;
    uint32_t rx_timeout_error:1;
    uint32_t spi_error:1;
    uint32_t wakeup_LLDE:1;
    uint32_t wakeup_LLDO:1;
    uint32_t rx_ranging_frame:1;     //Range Request bit set for inbound frame
    uint32_t tx_ranging_frame:1;     //Range Request bit set for outbound frame
    uint32_t sleeping:1;
}dw1000_dev_status_t;

typedef struct _dw1000_dev_control_t{
    uint32_t wait4resp_enabled:1;
    uint32_t wait4resp_delay_enabled:1;
    uint32_t delay_start_enabled:1;
    uint32_t autoack_delay_enabled:1;
    uint32_t start_rx_syncbuf_enabled:1;
    uint32_t rx_timeout_enabled:1;
    uint32_t wakeup_LLDE:1;
    uint32_t wakeup_LLDO:1;
}dw1000_dev_control_t;

typedef struct _dw1000_dev_config_t{
    uint32_t autoack_enabled:1;
    uint32_t autoack_delay_enabled:1;
    uint32_t dblbuffon_enabled:1;
    uint32_t framefilter_enabled:1;
    uint32_t rxdiag_enable:1;
    uint32_t rxauto_enable:1;
}dw1000_dev_config_t;

typedef struct _dwt_config_t {
    uint8_t chan ;           //!< channel number {1, 2, 3, 4, 5, 7 }
    uint8_t prf ;            //!< Pulse Repetition Frequency {DWT_PRF_16M or DWT_PRF_64M}
    uint8_t txPreambLength ; //!< DWT_PLEN_64..DWT_PLEN_4096
    uint8_t rxPAC ;          //!< Acquisition Chunk Size (Relates to RX preamble length)
    uint8_t txCode ;         //!< TX preamble code
    uint8_t rxCode ;         //!< RX preamble code
    uint8_t nsSFD ;          //!< Boolean should we use non-standard SFD for better performance
    uint8_t dataRate ;       //!< Data Rate {DWT_BR_110K, DWT_BR_850K or DWT_BR_6M8}
    uint8_t phrMode ;        //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
    uint16_t sfdTO ;         //!< SFD timeout value (in symbols)
} dwt_config_t ;
    
typedef struct _dw1000_dev_rxdiag_t{
    uint16_t    fp_idx;             // First path index (10.6 bits fixed point integer)
    uint16_t    fp_amp;             // Amplitude at floor(index FP) + 1
    uint16_t    fp_amp2;            // Amplitude at floor(index FP) + 2
    uint16_t    fp_amp3;            // Amplitude at floor(index FP) + 3
    uint16_t    rx_std;             // Standard deviation of noise
    uint16_t    preamble_cnt;       // Count of preamble symbols accumulated
    uint16_t    max_growth_cir;     // Channel Impulse Response max growth CIR
} dw1000_dev_rxdiag_t;


typedef struct _dw1000_dev_instance_t{
    struct os_dev uwb_dev;     /** Has to be here for cast in create_dev to work */
    struct os_mutex *spi_mutex;  /** Pointer to global spi mutex if available  */
    struct os_sem sem;  // semphore for low level mac/phy functions. 

    void (* tx_complete_cb) (struct _dw1000_dev_instance_t *);
    void (* rx_complete_cb) (struct _dw1000_dev_instance_t *);
    void (* rx_timeout_cb) (struct _dw1000_dev_instance_t *);
    void (* rx_error_cb) (struct _dw1000_dev_instance_t *);
/*
    struct _dw1000_mac_callbacks_t;
    struct _dw1000_rng_callbacks_t;
    struct _dw1000_rng_callbacks_extension_t;
    struct _dw1000_lwip_callbacks_t;
 */   
    void (* rng_tx_complete_cb) (struct _dw1000_dev_instance_t *);
    void (* rng_rx_complete_cb) (struct _dw1000_dev_instance_t *);
    void (* rng_rx_timeout_cb) (struct _dw1000_dev_instance_t *);
    void (* rng_rx_timeout_extension_cb) (struct _dw1000_dev_instance_t *);
    void (* rng_rx_error_cb) (struct _dw1000_dev_instance_t *);
    void (* rng_rx_error_extension_cb) (struct _dw1000_dev_instance_t *);
    void (* rng_tx_final_cb) (struct _dw1000_dev_instance_t *);
    void (* rng_interface_extension_cb) (struct _dw1000_dev_instance_t *);
    void (* rng_complete_cb) (struct _dw1000_dev_instance_t *);
  
#if MYNEWT_VAL(DW1000_LWIP)
    void (* lwip_tx_complete_cb) (struct _dw1000_dev_instance_t *);
    void (* lwip_rx_complete_cb) (struct _dw1000_dev_instance_t *);
    void (* lwip_rx_timeout_cb) (struct _dw1000_dev_instance_t *);
    void (* lwip_rx_error_cb) (struct _dw1000_dev_instance_t *);
#endif

#if MYNEWT_VAL(DW1000_CLOCK_CALIBRATION)
    void (* ccp_rx_complete_cb) (struct _dw1000_dev_instance_t *);
    void (* ccp_tx_complete_cb) (struct _dw1000_dev_instance_t *);
#endif

#if MYNEWT_VAL(DW1000_PAN)
    void (* pan_rx_complete_cb) (struct _dw1000_dev_instance_t *);
    void (* pan_tx_complete_cb) (struct _dw1000_dev_instance_t *);
    void (* pan_rx_timeout_cb) (struct _dw1000_dev_instance_t *);    
#endif
#if MYNEWT_VAL(DW1000_RANGE)
    void (* range_complete_cb) (struct _dw1000_dev_instance_t *);
    void (* range_error_cb) (struct _dw1000_dev_instance_t *);
#endif
#if MYNEWT_VAL(DW1000_PROVISION)
    void (* provision_tx_complete_cb) (struct _dw1000_dev_instance_t *);
    void (* provision_rx_complete_cb) (struct _dw1000_dev_instance_t *);
    void (* provision_rx_timeout_cb) (struct _dw1000_dev_instance_t *);
    void (* provision_rx_error_cb) (struct _dw1000_dev_instance_t *);
#endif
    union {
        uint16_t fctrl;                         // Reported frame control 
        uint8_t fctrl_array[sizeof(uint16_t)];  //endianness safe interface
    };
    uint16_t frame_len;      // Reported frame length
    uint8_t spi_num;
    uint8_t irq_pin;        
    uint8_t ss_pin;
    uint8_t rst_pin;
    uint32_t device_id;
    uint16_t my_short_address;
    uint64_t my_long_address;
#if MYNEWT_VAL(DW1000_CLOCK_CALIBRATION)
    uint64_t clock_master;
#endif
    uint64_t timestamp;
    uint64_t rxtimestamp;
    uint64_t txtimestamp;
    uint16_t PANID;
    uint16_t slot_id;
    uint32_t partID;
    uint32_t lotID;
    uint16_t otp_rev;
    uint8_t otp_vbat;
    uint8_t otp_temp;
    uint16_t sleep_mode;
    uint8_t xtal_trim;
    uint32_t sys_cfg_reg;
    uint32_t sys_ctrl_reg;
    uint8_t longFrames;
    uint32_t tx_fctrl;
    uint32_t sys_status;    // SYS_STATUS_ID for current event
    uint16_t rx_antenna_delay;
    uint16_t tx_antenna_delay;
    struct os_mutex mutex;
    struct hal_spi_settings spi_settings;
    struct os_eventq interrupt_eventq;
    struct os_event interrupt_ev;
    struct os_task interrupt_task_str;
    uint8_t interrupt_task_prio;
    os_stack_t interrupt_task_stack[DW1000_DEV_TASK_STACK_SZ]
        __attribute__((aligned(OS_STACK_ALIGNMENT)));
    struct _dw1000_rng_instance_t * rng;
#if MYNEWT_VAL(DW1000_LWIP)
    struct _dw1000_lwip_instance_t * lwip;
#endif
#if MYNEWT_VAL(DW1000_PROVISION)
    struct _dw1000_provision_instance_t *provision;
#endif
#if MYNEWT_VAL(DW1000_CLOCK_CALIBRATION)
    struct _dw1000_ccp_instance_t * ccp;
#endif
#if MYNEWT_VAL(DW1000_PAN)
    struct _dw1000_pan_instance_t * pan;
#endif
#if MYNEWT_VAL(DW1000_RANGE)
    struct _dw1000_range_instance_t * range;
#endif
    dw1000_dev_rxdiag_t rxdiag;
    dw1000_dev_config_t config;
    dw1000_dev_control_t control;
    dw1000_dev_control_t control_rx_context;
    dw1000_dev_control_t control_tx_context; 
    dw1000_dev_status_t status; 
    dw1000_dev_role_t dev_type;
    dwt_config_t mac_config;
}dw1000_dev_instance_t;

/* Used to pass data to init function from bsp_hal */
struct dw1000_dev_cfg {
    struct os_mutex *spi_mutex;
    int spi_num;
};

typedef void (* dw1000_dev_cb_t)(dw1000_dev_instance_t * inst);
int dw1000_dev_init(struct os_dev *odev, void *arg);
int dw1000_dev_config(dw1000_dev_instance_t * inst);
void dw1000_softreset(dw1000_dev_instance_t * inst);
dw1000_dev_status_t dw1000_read(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, uint8_t * buffer, uint16_t length);
dw1000_dev_status_t dw1000_write(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, uint8_t * buffer, uint16_t length);
uint64_t dw1000_read_reg(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, size_t nsize);
void dw1000_write_reg(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, uint64_t val, size_t nsize);

void dw1000_dev_configure_sleep(dw1000_dev_instance_t * inst, uint16_t mode, uint8_t wake);
dw1000_dev_status_t dw1000_dev_enter_sleep(dw1000_dev_instance_t * inst);
dw1000_dev_status_t dw1000_dev_wakeup(dw1000_dev_instance_t * inst);
void dw1000_dev_enter_sleep_after_tx(dw1000_dev_instance_t * inst, int enable);
    
#ifdef __cplusplus
}
#endif

#endif /* _DW1000_DEV_H_ */
