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
 * @file dw1000_dev.h
 * @author paul kettle  
 * @date 2018 
 * @brief Device file
 *
 * @details This is the dev base class which utilises the functions to perform initialization and necessary configurations on device.
 *
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
#include <dw1000/dw1000_stats.h>
#if MYNEWT_VAL(CIR_ENABLED)
#include <cir/cir.h>
#endif

#define DWT_DEVICE_ID   (0xDECA0130) //!< Decawave Device ID 
#define DWT_SUCCESS (0)              //!< DWT Success
#define DWT_ERROR   (-1)             //!< DWT Error
#define DWT_TIME_UNITS          (1.0/499.2e6/128.0) //!< DWT time units calculation

#define DW1000_DEV_TASK_PRIO        MYNEWT_VAL(DW1000_DEV_TASK_PRIO)   //!< Priority for DW1000 Dev Task
#define DW1000_DEV_TASK_STACK_SZ    MYNEWT_VAL(DW1000_DEV_TASK_STACK_SZ) //!< Stack size for DW1000 Dev Task

#define BROADCAST_ADDRESS          0xffff  //!< Broad cast addresss

//! IDs for blocking/non-blocking mode .
typedef enum _dw1000_dev_modes_t{
    DWT_BLOCKING,                    //!< Blocking mode of DW1000
    DWT_NONBLOCKING                  //!< Non-blocking mode of DW1000
}dw1000_dev_modes_t;
 
//! Device Role
typedef enum _dw1000_dev_role_t{
    NODE_0,                         //!< Device type of NODE_0   
    NODE,                           //!< Device type of NODE
    TAG                             //!< Device type of tag
}dw1000_dev_role_t;

//! Extension ids for services.
typedef enum _dw1000_extension_id_t{
    DW1000_CCP=1,                            //!< Clock calibration packet
    DW1000_WCS,                              //!< Wireless Clock Synchronization services
    DW1000_TDMA,                             //!< TDMA services
    DW1000_RNG,                              //!< Ranging
    DW1000_RNG_SS,                           //!< Ranging
    DW1000_RNG_SS_EXT,                       //!< Ranging
    DW1000_RNG_DS,                           //!< Ranging
    DW1000_RNG_DS_EXT,                       //!< Ranging
    DW1000_RANGE,                            //!< Ranging
    DW1000_NRNG,                             //!< Nrng
    DW1000_NRNG_SS,                          //!< Nrng
    DW1000_NRNG_SS_EXT,                      //!< Nrng
    DW1000_NRNG_DS,                          //!< Nrng
    DW1000_NRNG_DS_EXT,                      //!< Nrng
    DW1000_LWIP,
    DW1000_PAN,                              //!< Personal area network
    DW1000_PROVISION,                        //!< Provisioning
    DW1000_NMGR_UWB,                         //!< UWB transport layer
    DW1000_NMGR_CMD,                         //!< UWB command support
    DW1000_CIR,                              //!< Channel impulse response 
    DW1000_OT,                               //!< Openthread
    DW1000_RTDOA,                            //!< RTDoA
    DW1000_SURVEY,
    DW1000_APP0 = 1024, 
    DW1000_APP1, 
    DW1000_APP2
}dw1000_extension_id_t;

//! Structure of DW1000 attributes.
typedef struct _dw1000_cmd{
    uint32_t reg:6;                   //!< Indicates the register to be read or write into 
    uint32_t subindex:1;              //!< Indicates offset address of the register 
    uint32_t operation:1;             //!< Read or Write operation 
    uint32_t extended:1;              //!< If subaddress is morethan 128 
    uint32_t subaddress:15;           //!< Indicates subaddress of register 
}dw1000_cmd_t;

//! Structure of DW1000 device status.
typedef struct _dw1000_dev_status_t{
    uint32_t selfmalloc:1;            //!< Internal flag for memory garbage collection 
    uint32_t initialized:1;           //!< Instance allocated 
    uint32_t start_tx_error:1;        //!< Start transmit error 
    uint32_t start_rx_error:1;        //!< Start receive error
    uint32_t tx_frame_error:1;        //!< Transmit frame error
    uint32_t txbuf_error:1;           //!< Tx buffer error
    uint32_t rx_error:1;              //!< Receive error
    uint32_t rx_timeout_error:1;      //!< Receive timeout error
    uint32_t lde_error:1;             //!< LDE error
    uint32_t spi_error:1;             //!< SPI error
    uint32_t LDE_enabled:1;           //!< Load LDE microcode on wake up
    uint32_t LDO_enabled:1;           //!< Load the LDO tune value on wake up
    uint32_t sleep_enabled:1;         //!< Indicates sleep_enabled bit is set
    uint32_t sleeping:1;              //!< Indicates sleeping state
    uint32_t sem_force_released:1;    //!< Semaphore was released in forcetrxoff
    uint32_t overrun_error:1;         //!< Dblbuffer overrun detected
}dw1000_dev_status_t;

//! Device control status bits.
typedef struct _dw1000_dev_control_t{
    uint32_t wait4resp_enabled:1;           //!< Wait for the response
    uint32_t wait4resp_delay_enabled:1;     //!< Wait for the delayed response
    uint32_t delay_start_enabled:1;         //!< Transmit after delayed start
    uint32_t autoack_delay_enabled:1;       //!< Enables automatic acknowledgement feature with delay
    uint32_t start_rx_syncbuf_enabled:1;    //!< Enables receive syncbuffer
    uint32_t rx_timeout_enabled:1;          //!< Enables receive timeout
    uint32_t on_error_continue_enabled:1;   //!< Enables on_error_continue
    uint32_t sleep_after_tx:1;              //!< Enables to load LDE microcode on wake up
    uint32_t sleep_after_rx:1;              //!< Enables to load LDO tune value on wake up
}dw1000_dev_control_t;

//! DW1000 receiver configuration parameters.
typedef struct _dw1000_dev_rx_config_t{
    uint8_t pacLength;                      //!< Acquisition Chunk Size DWT_PAC8..DWT_PAC64 (Relates to RX preamble length)
    uint8_t preambleCodeIndex;              //!< RX preamble code
    uint8_t sfdType;                        //!< Boolean should we use non-standard SFD for better performance
    uint8_t phrMode;                        //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
    uint16_t sfdTimeout;                    //!< SFD timeout value (in symbols) (preamble length + 1 + SFD length - PAC size). 
}dw1000_dev_rx_config_t;

//! DW1000 transmitter configuration parameters.
typedef struct _dw1000_dev_tx_config_t{
    uint8_t preambleCodeIndex;              //!< TX preamble code
    uint8_t preambleLength;                 //!< DWT_PLEN_64..DWT_PLEN_4096
}dw1000_dev_tx_config_t;

//! DW1000 transmitter power configuration parameters.
typedef struct _dw1000_dev_txrf_config_t {
    uint8_t   PGdly; 
    union _power {   
        struct _smart{ 
            uint8_t BOOSTNORM;      //!< PWR_TX_DATA_PWR
            uint8_t BOOSTP500;      //!< PWR_TX_PHR_PWR
            uint8_t BOOSTP250;      //!< PWR_TX_SHR_PWR
            uint8_t BOOSTP125;      //!< T0D0
         };
         struct _manual { 
            uint8_t _NA1;           //!< TODO  
            uint8_t TXPOWPHR;       //!< TODO
            uint8_t TXPOWSD;        //!< TODO
            uint8_t _NA4;           //!< TODO   
         };
        uint32_t power;             //!< TODO
    };
}dw1000_dev_txrf_config_t;

//! DW1000 device configuration parameters.
typedef struct _dw1000_dev_config_t{
    uint8_t channel;                        //!< channel number {1, 2, 3, 4, 5, 7 }
    uint8_t dataRate;                       //!< Data Rate {DWT_BR_110K, DWT_BR_850K or DWT_BR_6M8}
    uint8_t prf;                            //!< Pulse Repetition Frequency {DWT_PRF_16M or DWT_PRF_64M}
    struct _dw1000_dev_rx_config_t rx;      //!< DW1000 receiver configuration parameters.
    struct _dw1000_dev_tx_config_t tx;      //!< DW1000 transmitterr configuration parameters.
    struct _dw1000_dev_txrf_config_t txrf;  //!< DW1000 transmitter power configuration parameters.
    uint32_t autoack_enabled:1;             //!< Enables automatic acknowledgement
    uint32_t autoack_delay_enabled:1;       //!< Enables automatic acknowledgement feature with delay
    uint32_t dblbuffon_enabled:1;           //!< Enables double buffer
    uint32_t framefilter_enabled:1;         //!< Enables frame fileter
    uint32_t trxoff_enable:1;               //!< Enables forced TRXOFF in start_tx and start_tx interface 
    uint32_t rxdiag_enable:1;               //!< Enables receive diagnostics parameters 
    uint32_t rxauto_enable:1;               //!< Enables auto receive parameter
    uint32_t bias_correction_enable:1;      //!< Enables bias correction ploynomial
    uint32_t LDE_enable:1;                  //!< Enables LDE
    uint32_t LDO_enable:1;                  //!< Enables LDO
    uint32_t wakeup_rx_enable:1;            //!< Enables wakeup_rx_enable 
    uint32_t sleep_enable:1;                //!< Enables sleep_enable bit
    uint32_t cir_enable:1;                  //!< Enables reading CIR as default
    uint32_t pmem_enable:1;                 //!< Enables reading Preamble memory as default behaviour
}dw1000_dev_config_t;

//! DW1000 receiver diagnostics parameters.
typedef struct _dw1000_dev_rxdiag_t{
    union {
        struct _rx_time {
            uint32_t    fp_idx:16;          //!< First path index (10.6 bits fixed point integer)
            uint32_t    fp_amp:16;          //!<  Amplitude at floor(index FP) + 1
        };
        uint32_t rx_time;
    };
    union {
        struct _rx_fqual {
            uint64_t    rx_std:16;          //!<  Standard deviation of noise
            uint64_t    fp_amp2:16;         //!<  Amplitude at floor(index FP) + 2
            uint64_t    fp_amp3:16;         //!<  Amplitude at floor(index FP) + 3
            uint64_t    cir_pwr:16;         //!<  Channel Impulse Response max growth CIR
        };
        uint64_t rx_fqual;
    };
    uint16_t    pacc_cnt;                   //!<  Count of preamble symbols accumulated
} __attribute__((packed, aligned(1))) dw1000_dev_rxdiag_t;

//! physical attributes per IEEE802.15.4-2011 standard, Table 101
typedef struct _phy_attributes_t{
    float Tpsym;
    float Tbsym;
    float Tdsym;
    uint8_t nsfd;  
    uint8_t nphr;    
    uint16_t nsync;  
} phy_attributes_t;

struct _dw1000_dev_instance_t;

//! Structure of extension callbacks structure common for mac layer.
typedef struct _dw1000_mac_interface_t dw1000_mac_interface_t;
typedef struct _dw1000_mac_interface_t {
    struct _status{
        uint16_t selfmalloc:1;            //!< Internal flag for memory garbage collection 
        uint16_t initialized:1;           //!< Instance allocated          
    } status;
    uint16_t id;
    bool (* tx_complete_cb) (struct _dw1000_dev_instance_t *, struct _dw1000_mac_interface_t *);    //!< Transmit complete callback
    bool (* rx_complete_cb) (struct _dw1000_dev_instance_t *, struct _dw1000_mac_interface_t *);    //!< Receive complete callback
    bool (* cir_complete_cb)(struct _dw1000_dev_instance_t *, struct _dw1000_mac_interface_t *);    //!< CIR complete callback, prior to RXEN
    bool (* rx_timeout_cb)  (struct _dw1000_dev_instance_t *, struct _dw1000_mac_interface_t *);    //!< Receive timeout callback
    bool (* rx_error_cb)    (struct _dw1000_dev_instance_t *, struct _dw1000_mac_interface_t *);    //!< Receive error callback
    bool (* tx_error_cb)    (struct _dw1000_dev_instance_t *, struct _dw1000_mac_interface_t *);    //!< Transmit error callback  
    bool (* reset_cb)       (struct _dw1000_dev_instance_t *, struct _dw1000_mac_interface_t *);    //!< Reset interface callback  
    bool (* final_cb)       (struct _dw1000_dev_instance_t *, struct _dw1000_mac_interface_t *);    //!< Final frame preperation interface callback  
    bool (* complete_cb)    (struct _dw1000_dev_instance_t *, struct _dw1000_mac_interface_t *);    //!< Completion event interface callback  
    bool (* sleep_cb)       (struct _dw1000_dev_instance_t *, struct _dw1000_mac_interface_t *);    //!< Wakeup event interface callback  
    bool (* start_tx_error_cb) (struct _dw1000_dev_instance_t *, struct _dw1000_mac_interface_t *);    //!< Start error event interface callback  
    SLIST_ENTRY(_dw1000_mac_interface_t) next;                    //!< Next callback in the list
}dw1000_mac_interface_t;

//! Device instance parameters.
typedef struct _dw1000_dev_instance_t{
    struct os_dev uwb_dev;                     //!< Has to be here for cast in create_dev to work 
    struct os_sem *spi_sem;                    //!< Pointer to global spi bus semaphore
    struct os_sem spi_nb_sem;                  //!< Semaphore for nonblocking rd/wr operations
    struct os_sem tx_sem;                         //!< semphore for low level mac/phy functions
    struct os_mutex mutex;                     //!< os_mutex
    uint32_t epoch; 
    uint8_t idx;                               //!< instance number number {0, 1, 2 etc}

    SLIST_HEAD(,_dw1000_mac_interface_t) interface_cbs;

#if MYNEWT_VAL(DW1000_LWIP)
    void (* lwip_rx_complete_cb) (struct _dw1000_dev_instance_t *);
#endif

    union {
        uint16_t fctrl;                         //!< Reported frame control 
        uint8_t fctrl_array[sizeof(uint16_t)];  //!< Endianness safe interface
    };

#if MYNEWT_VAL(DW1000_MAC_STATS)
    STATS_SECT_DECL(mac_stat_section) stat;
#endif
    uint16_t frame_len;            //!< Reported frame length
    uint8_t spi_num;               //!< SPI number
    uint8_t irq_pin;               //!< Interrupt request pin
    uint8_t ss_pin;                //!< Slave select pin
    uint8_t rst_pin;               //!< Reset pin
    uint32_t device_id;            //!< Device id  
    uint16_t my_short_address;     //!< Short address of tag/node
    union {
        uint64_t my_long_address;  //!< Long address of tag/node
        uint64_t euid;             //!< Extended Unique Identifier
    };
    uint64_t timestamp;            //!< Timestamp
    uint64_t rxtimestamp;          //!< Receive timestamp
    uint64_t txtimestamp;          //!< Transmit timestamp
    int32_t carrier_integrator;
    uint16_t PANID;                //!< personal network inetrface id
    uint16_t slot_id;              //!< Slot id 
    uint16_t cell_id;              //!< Cell id  
    uint32_t partID;               //!< Identifier of a particular part design
    uint32_t lotID;                //!< Identification number assigned to a particular quantity
    uint16_t otp_rev;              //!< OTP parameter revision
    uint8_t otp_vbat;              //!< OTP parameter for voltage 
    uint8_t otp_temp;              //!< OTP parameter for temperature
    uint8_t xtal_trim;             //!< Crystal trim
    uint32_t sys_cfg_reg;          //!< System config register
    uint32_t tx_fctrl;             //!< Transmit frame control register parameter 
    uint32_t sys_status;           //!< SYS_STATUS_ID for current event
    uint16_t rx_antenna_delay;     //!< Receive antenna delay
    uint16_t tx_antenna_delay;     //!< Transmit antenna delay  
    
    struct hal_spi_settings spi_settings;  //!< Structure of SPI settings in hal layer 
    struct os_eventq eventq;     //!< Structure of os_eventq that has event queue 
    struct os_event interrupt_ev;          //!< Structure of os_event that tirgger interrupts 
    struct os_task task_str;     //!< Structure of os_task that has interrupt task 
    uint8_t task_prio;           //!< Priority of the interrupt task  
    os_stack_t task_stack[DW1000_DEV_TASK_STACK_SZ]  //!< Stack of the interrupt task 
        __attribute__((aligned(OS_STACK_ALIGNMENT)));
    uint8_t rxbuf[RX_BUFFER_LEN];            //!< local rxbuf  
    struct _dw1000_rng_instance_t * rng;     //!< DW1000 rng instance 
#if MYNEWT_VAL(LWIP_ENABLED) 
    struct _dw1000_lwip_instance_t * lwip;   //!< DW1000 lwip instance
#endif
#if MYNEWT_VAL(PROVISION_ENABLED)
    struct _dw1000_provision_instance_t * provision; //!< DW1000 provision instance
#endif 
#if MYNEWT_VAL(CCP_ENABLED)
    struct _dw1000_ccp_instance_t * ccp;           //!< DW1000 ccp instance
#endif
#if MYNEWT_VAL(PAN_ENABLED)
    struct _dw1000_pan_instance_t * pan;           //!< DW1000 pan instance
#endif
#if MYNEWT_VAL(DW1000_RANGE)
    struct _dw1000_range_instance_t * range;       //!< DW1000 range instance
#endif
#if MYNEWT_VAL(TDMA_ENABLED)
    struct _tdma_instance_t * tdma;               //!< DW1000 tdma instance
#endif
#if MYNEWT_VAL(NRNG_ENABLED)
    struct _dw1000_nrng_instance_t * nrng;
#endif
#if MYNEWT_VAL(RTDOA_ENABLED)
    struct _dw1000_rtdoa_instance_t * rtdoa;
#endif
#if MYNEWT_VAL(CIR_ENABLED)
    struct _cir_instance_t * cir;                  //!< CIR instance
#endif
#if MYNEWT_VAL(OT_ENABLED)
    struct _ot_instance_t* ot;                     //!< Openthread Instance
#endif
#if MYNEWT_VAL(SURVEY_ENABLED)
    struct _survey_instance_t * survey;            //!< AutoSite Survey instance
#endif
#if MYNEWT_VAL(NMGR_UWB_ENABLED)
    struct _nmgr_uwb_instance_t* nmgruwb;
#endif
    dw1000_dev_rxdiag_t rxdiag;                    //!< DW1000 receive diagnostics
    dw1000_dev_config_t config;                    //!< DW1000 device configurations  
    dw1000_dev_control_t control;                  //!< DW1000 device control parameters      
    dw1000_dev_status_t status;                    //!< DW1000 device status 
    dw1000_dev_role_t dev_type;                    //!< Type of the device (tag/node)
    struct _phy_attributes_t attrib;
    
}dw1000_dev_instance_t;

//! SPI parameters
struct dw1000_dev_cfg {
    struct os_sem *spi_sem;                        //!< Pointer to os_sem structure to lock spi bus
    int spi_num;                                   //!< SPI number
};

typedef void (* dw1000_dev_cb_t)(dw1000_dev_instance_t * inst);
int dw1000_dev_init(struct os_dev *odev, void *arg);
int dw1000_dev_config(dw1000_dev_instance_t * inst);
void dw1000_softreset(dw1000_dev_instance_t * inst);
dw1000_dev_status_t dw1000_read(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, uint8_t * buffer, uint16_t length);
dw1000_dev_status_t dw1000_write(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, uint8_t * buffer, uint16_t length);
uint64_t dw1000_read_reg(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, size_t nsize);
void dw1000_write_reg(dw1000_dev_instance_t * inst, uint16_t reg, uint16_t subaddress, uint64_t val, size_t nsize);
void dw1000_dev_set_sleep_timer(dw1000_dev_instance_t * inst, uint16_t count);
void dw1000_dev_configure_sleep(dw1000_dev_instance_t * inst);
dw1000_dev_status_t dw1000_dev_enter_sleep(dw1000_dev_instance_t * inst);
dw1000_dev_status_t dw1000_dev_wakeup(dw1000_dev_instance_t * inst);
dw1000_dev_status_t dw1000_dev_enter_sleep_after_tx(dw1000_dev_instance_t * inst, uint8_t enable);
dw1000_dev_status_t dw1000_dev_enter_sleep_after_rx(dw1000_dev_instance_t * inst, uint8_t enable);
    
#define dw1000_dwt_usecs_to_usecs(_t) (double)( (_t) * (0x10000UL/(128*499.2)))
#define dw1000_usecs_to_dwt_usecs(_t) (double)( (_t) / dw1000_dwt_usecs_to_usecs(1.0))

#ifdef __cplusplus
}
#endif

#endif /* _DW1000_DEV_H_ */
