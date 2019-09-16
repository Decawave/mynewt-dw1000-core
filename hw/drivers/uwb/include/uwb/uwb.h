/*
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

#ifndef __UWB_H__
#define __UWB_H__

#include <os/os_dev.h>
#include <dpl/dpl.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_DEV_TASK_STACK_SZ (MYNEWT_VAL(UWB_DEV_TASK_STACK_SZ))

//! Extension ids for services.
typedef enum uwb_extension_id {
    UWBEXT_CCP=1,                            //!< Clock Calibration Packet
    UWBEXT_WCS,                              //!< Wireless Clock Synchronization services
    UWBEXT_TDMA,                             //!< TDMA services
    UWBEXT_RNG,                              //!< Ranging
    UWBEXT_RNG_SS,                           //!< Ranging
    UWBEXT_RNG_SS_EXT,                       //!< Ranging
    UWBEXT_RNG_DS,                           //!< Ranging
    UWBEXT_RNG_DS_EXT,                       //!< Ranging
    UWBEXT_RANGE,                            //!< Ranging
    UWBEXT_NRNG,                             //!< Nrng
    UWBEXT_NRNG_SS,                          //!< Nrng
    UWBEXT_NRNG_SS_EXT,                      //!< Nrng
    UWBEXT_NRNG_DS,                          //!< Nrng
    UWBEXT_NRNG_DS_EXT,                      //!< Nrng
    UWBEXT_LWIP,                             //!< LWIP
    UWBEXT_PAN,                              //!< Personal area network
    UWBEXT_PROVISION,                        //!< Provisioning
    UWBEXT_NMGR_UWB,                         //!< UWB transport layer
    UWBEXT_NMGR_CMD,                         //!< UWB command support
    UWBEXT_CIR,                              //!< Channel impulse response
    UWBEXT_OT = 0x30,                        //!< Openthread
    UWBEXT_RTDOA = 0x40,                     //!< RTDoA
    UWBEXT_RTDOA_BH,                         //!< RTDoA Backhaul
    UWBEXT_SURVEY = 0x50,                    //!< Survey
    UWBEXT_APP0 = 1024,
    UWBEXT_APP1,
    UWBEXT_APP2
} uwb_extension_id_t;

//! Device Roles
#define UWB_ROLE_CCP_MASTER   (0x0001)
#define UWB_ROLE_PAN_MASTER   (0x0002)
#define UWB_ROLE_ANCHOR       (0x0004)

//! IDs for blocking/non-blocking mode .
typedef enum _uwb_dev_modes_t{
    UWB_BLOCKING,                    //!< Blocking mode 
    UWB_NONBLOCKING                  //!< Non-blocking mode
}uwb_dev_modes_t;


//! Structure of UWB device status.
struct uwb_dev_status {
    uint32_t selfmalloc:1;            //!< Internal flag for memory garbage collection
    uint32_t initialized:1;           //!< Instance allocated
    uint32_t start_tx_error:1;        //!< Start transmit error
    uint32_t start_rx_error:1;        //!< Start receive error
    uint32_t tx_frame_error:1;        //!< Transmit frame error
    uint32_t txbuf_error:1;           //!< Tx buffer error
    uint32_t rx_error:1;              //!< Receive error
    uint32_t rx_timeout_error:1;      //!< Receive timeout error
    uint32_t lde_error:1;             //!< Error in Leading Edge Detection
    uint32_t spi_error:1;             //!< SPI error
    uint32_t LDE_enabled:1;           //!< Load LDE microcode on wake up
    uint32_t LDO_enabled:1;           //!< Load the LDO tune value on wake up
    uint32_t sleep_enabled:1;         //!< Indicates sleep_enabled bit is set
    uint32_t sleeping:1;              //!< Indicates sleeping state
    uint32_t sem_force_released:1;    //!< Semaphore was released in forcetrxoff
    uint32_t overrun_error:1;         //!< Dblbuffer overrun detected
    uint32_t rx_restarted:1;          //!< RX restarted since last received packet
};

/** API parent structure for rx diagnostics. Used to store values for later
 * processing if needed in a device agnostic way.
 */
struct uwb_dev_rxdiag {
    uint16_t rxd_len;                 //!< length of rxdiag structure
    uint32_t features;                //!< Features covered by rxdiag structure
} __attribute__((__packed__, aligned(1)));

//! physical attributes per IEEE802.15.4-2011 standard, Table 101
struct uwb_phy_attributes {
    float Tpsym;
    float Tbsym;
    float Tdsym;
    uint8_t nsfd;  
    uint8_t nphr;    
    uint16_t nsync;  
};
    
struct uwb_dev;

//! Structure of extension callbacks structure common for mac layer.
struct uwb_mac_interface {
    struct {
        uint16_t initialized:1;           //!< Instance initialized
    } status;
    uint16_t id;                          //!< Identifier
    void *inst_ptr;                       //!< Pointer to instance owning this interface
    bool (* tx_complete_cb) (struct uwb_dev *, struct uwb_mac_interface *);    //!< Transmit complete callback
    bool (* rx_complete_cb) (struct uwb_dev *, struct uwb_mac_interface *);    //!< Receive complete callback
    bool (* cir_complete_cb)(struct uwb_dev *, struct uwb_mac_interface *);    //!< CIR complete callback, prior to RXEN
    bool (* rx_timeout_cb)  (struct uwb_dev *, struct uwb_mac_interface *);    //!< Receive timeout callback
    bool (* rx_error_cb)    (struct uwb_dev *, struct uwb_mac_interface *);    //!< Receive error callback
    bool (* tx_error_cb)    (struct uwb_dev *, struct uwb_mac_interface *);    //!< Transmit error callback
    bool (* reset_cb)       (struct uwb_dev *, struct uwb_mac_interface *);    //!< Reset interface callback
    bool (* final_cb)       (struct uwb_dev *, struct uwb_mac_interface *);    //!< Final frame preperation interface callback
    bool (* complete_cb)    (struct uwb_dev *, struct uwb_mac_interface *);    //!< Completion event interface callback
    bool (* sleep_cb)       (struct uwb_dev *, struct uwb_mac_interface *);    //!< Wakeup event interface callback
    bool (* start_tx_error_cb) (struct uwb_dev *, struct uwb_mac_interface *); //!< TX Start error event interface callback
    SLIST_ENTRY(uwb_mac_interface) next;                                       //!< Next callback in the list
};

//! DW1000 receiver configuration parameters.
struct uwb_dev_rx_config {
    uint8_t pacLength;                      //!< Acquisition Chunk Size DWT_PAC8..DWT_PAC64 (Relates to RX preamble length)
    uint8_t preambleCodeIndex;              //!< RX preamble code
    uint8_t sfdType;                        //!< Boolean should we use non-standard SFD for better performance
    uint8_t phrMode;                        //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
    uint16_t sfdTimeout;                    //!< SFD timeout value (in symbols) (preamble length + 1 + SFD length - PAC size).
};

//! UWB transmitter configuration parameters.
struct uwb_dev_tx_config {
    uint8_t preambleCodeIndex;              //!< TX preamble code
    uint8_t preambleLength;                 //!< DWT_PLEN_64..DWT_PLEN_4096
};

//! UWB transmitter power configuration parameters.
struct uwb_dev_txrf_config {
    uint8_t   PGdly;
    union _power {
        struct _smart{
            uint8_t BOOSTNORM;      //!< PWR_TX_DATA_PWR
            uint8_t BOOSTP500;      //!< PWR_TX_PHR_PWR
            uint8_t BOOSTP250;      //!< PWR_TX_SHR_PWR
            uint8_t BOOSTP125;      //!< TODO
         };
         struct _manual {
            uint8_t _NA1;           //!< TODO
            uint8_t TXPOWPHR;       //!< TODO
            uint8_t TXPOWSD;        //!< TODO
            uint8_t _NA4;           //!< TODO
         };
        uint32_t power;             //!< TODO
    };
};

//! DW1000 device configuration parameters.
struct uwb_dev_config{
    uint8_t channel;                        //!< channel number {1, 2, 3, 4, 5, 7, 9}
    uint8_t dataRate;                       //!< Data Rate {DWT_BR_110K, DWT_BR_850K or DWT_BR_6M8}
    uint8_t prf;                            //!< Pulse Repetition Frequency {DWT_PRF_16M or DWT_PRF_64M}
    struct uwb_dev_rx_config rx;            //!< DW1000 receiver configuration parameters.
    struct uwb_dev_tx_config tx;            //!< DW1000 transmitter configuration parameters.
    struct uwb_dev_txrf_config txrf;        //!< DW1000 transmitter power configuration parameters.
    uint32_t autoack_enabled:1;             //!< Enables automatic acknowledgement
    uint32_t autoack_delay_enabled:1;       //!< Enables automatic acknowledgement feature with delay
    uint32_t dblbuffon_enabled:1;           //!< Enables double buffer
    uint32_t framefilter_enabled:1;         //!< Enables frame fileter
    uint32_t trxoff_enable:1;               //!< Enables forced TRXOFF in start_tx and start_tx interface
    uint32_t rxdiag_enable:1;               //!< Enables receive diagnostics parameters
    uint32_t rxttcko_enable:1;              //!< Enables reading of time tracking integrator (used in dblbuffer only as carrier integrator isn't available)
    uint32_t rxauto_enable:1;               //!< Enables auto receive parameter
    uint32_t bias_correction_enable:1;      //!< Enables bias correction ploynomial
    uint32_t LDE_enable:1;                  //!< Enables LDE
    uint32_t LDO_enable:1;                  //!< Enables LDO
    uint32_t wakeup_rx_enable:1;            //!< Enables wakeup_rx_enable
    uint32_t sleep_enable:1;                //!< Enables sleep_enable bit
    uint32_t cir_enable:1;                  //!< Enables reading CIR as default
    uint32_t pmem_enable:1;                 //!< Enables reading Preamble memory as default behaviour
    uint32_t cir_pdoa_slave:1;              //!< This instance is acting as a pdoa slave
};

/**
 * Configure the mac layer
 * @param inst     Pointer to struct uwb_dev.
 * @param config   Pointer to struct uwb_dev_config.
 * @return struct uwb_dev_status
 *
 */
typedef struct uwb_dev_status (*uwb_mac_config_func_t)(struct uwb_dev * dev, struct uwb_dev_config * config);

/**
 * Configuration of the uwb transmitter
 * including the power and pulse generator delay. The input is a pointer to the data structure
 * of type struct uwb_dev_txrf_config that holds all the configurable items.
 *
 * @param inst      Pointer to struct uwb_dev
 * @param config    Pointer to struct uwb_dev_txrf_config.
 * @return void
 */
typedef void (*uwb_txrf_config_func_t)(struct uwb_dev * dev, struct uwb_dev_txrf_config *config);

/**
 *  Configure the device for both DEEP_SLEEP and SLEEP modes, and on-wake mode
 *  i.e., before entering the sleep, the device can be programmed for TX or RX, then upon "waking up" the TX/RX settings
 *  will be preserved and the device can immediately perform the desired action TX/RX.
 *
 * NOTE: e.g. Tag operation - after deep sleep, the device needs to just load the TX buffer and send the frame.
 *
 * @param inst  Pointer to struct uwb_dev.
 *
 * @return void
 */
typedef void (*uwb_sleep_config_func_t)(struct uwb_dev * dev);

/**
 * API to enter device into sleep mode.
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 * @return struct uwb_dev_status
 */
typedef struct uwb_dev_status (*uwb_enter_sleep_func_t)(struct uwb_dev * dev);

/** 
 * Set the auto TX to sleep bit. This means that after a frame
 * transmission the device will enter deep sleep mode. The uwb_sleep_config() function
 * needs to be called before this to configure the on-wake settings.
 *
 * NOTE: the IRQ line has to be low/inactive (i.e. no pending events)
 *
 * @param inst       Pointer to struct uwb_dev.
 * @param enable     1 to configure the device to enter deep sleep after TX, 
 *                   0 to disables the configuration.
 * @return struct uwb_dev_status
 */
typedef struct uwb_dev_status (*uwb_enter_sleep_after_tx_func_t)(struct uwb_dev * dev, uint8_t enable);

/**
 *  Sets the auto RX to sleep bit. This means that after a frame
 *  received the device will enter deep sleep mode. The dev_configure_sleep() function
 *  needs to be called before this to configure the on-wake settings.
 *
 * NOTE: the IRQ line has to be low/inactive (i.e. no pending events).
 * @param inst       Pointer to struct uwb_dev.
 * @param enable     1 to configure the device to enter deep sleep after TX, 
 *                   0 to disables the configuration
 * @return struct uwb_dev_status
 */
typedef struct uwb_dev_status (*uwb_enter_sleep_after_rx_func_t)(struct uwb_dev * dev, uint8_t enable);

/**
 * Wakeup device from sleep to init.
 *
 * @param inst  struct uwb_dev.
 * @return struct uwb_dev_status
 */
typedef struct uwb_dev_status (*uwb_wakeup_func_t)(struct uwb_dev * dev);

/**
 * Set Receive Wait Timeout period.
 *
 * @param dev       pointer to struct uwb_dev.
 * @param timeout   Indicates how long the receiver remains on from the RX enable command.
 *                  The time parameter used here is in 1.0256 * us (512/499.2MHz) units.
 *                  If set to 0 the timeout is disabled.
 * @return struct uwb_dev_status
 *
 * @brief The Receive Frame Wait Timeout period is a 32-bit field. The units for this parameter are roughly 1μs,
 * (the exact unit is 512 counts of the fundamental 499.2 MHz UWB clock, or 1.026 μs). When employing the frame wait timeout,
 * RXFWTO should be set to a value greater than the expected RX frame duration and include an allowance for any uncertainly
 * attaching to the expected transmission start time of the awaited frame.
 * When using .rxauto_enable feature it is important to understand the role of rx_timeout, in this situation it is the timeout
 * that actually turns-off the receiver and returns the transeiver to the idle state.
 *
 * NOTE: On dw1000 the timeout is a 16bit field only.
 */
typedef struct uwb_dev_status (*uwb_set_rx_timeout_func_t)(struct uwb_dev *dev, uint32_t timeout);

/**
 * Adjust RX Wait Timeout period.
 *
 * @param inst      pointer to struct uwb_dev.
 * @param timeout   Indicates how long the receiver remains on from the RX enable command. The time parameter used here is in 1.0256 
 * us (512/499.2MHz) units If set to 0 the timeout is disabled.
 * @return struct uwb_dev_status
 * @brief The Receive Frame Wait Timeout period is a 32-bit field. The units for this parameter are roughly 1μs, 
 * (the exact unit is 512 counts of the fundamental 499.2 MHz UWB clock, or 1.026 μs). When employing the frame wait timeout, 
 * RXFWTO should be set to a value greater than the expected RX frame duration and include an allowance for any uncertainly 
 * attaching to the expected transmission start time of the awaited frame. 
 * When using .rxauto_enable feature it is important to understand the role of rx_timeout, in this situation it is the timeout 
 * that actually turns-off the receiver and returns the transeiver to the idle state.
 *
 * OBSERVE: This only works if the receiver has been setup to use a timeout with uwb_set_rx_timeout first.
 *
 * NOTE: On dw1000 the timeout is a 16bit field only.
 */
typedef struct uwb_dev_status (*uwb_adj_rx_timeout_func_t)(struct uwb_dev *dev, uint32_t timeout);

/**
 * To specify a time in future to either turn on the receiver to be ready to receive a frame,
 * or to turn on the transmitter and send a frame.
 * The low-order 9-bits of this register are ignored.
 * The delay is in UWB microseconds.
 *
 * @param dev     Pointer to struct uwb_dev.
 * @param dx_time Delayed Send or receive Time, in UWB microseconds.
 * @return uwb_dev_status
 */
typedef struct uwb_dev_status (*uwb_set_delay_start_func_t)(struct uwb_dev *dev, uint64_t dx_time);

/**
 * Start transmission.
 *
 * @param inst  pointer to struct uwb_dev.
 * @return struct uwb_dev_status
 */
typedef struct uwb_dev_status (*uwb_start_tx_func_t)(struct uwb_dev * dev);

/**
 * Activate reception mode (rx).
 *
 * @param inst  pointer to struct uwb_dev.
 * @return struct uwb_dev_status
 *
 */
typedef struct uwb_dev_status (*uwb_start_rx_func_t)(struct uwb_dev * dev);

/**
 * Gracefully turn off reception mode.
 *
 * @param inst  pointer to struct uwb_dev
 * @return struct uwb_dev_status
 * 
 */
typedef struct uwb_dev_status (*uwb_stop_rx_func_t)(struct uwb_dev *dev);

/**
 * Write the supplied TX data into the tranceiver's
 * TX buffer.The input parameters are the data length in bytes and a pointer
 * to those data bytes.
 *
 ***** NIKLAS TODO: This text should be fixed, no need to mention CRC bytes here?
 * Note: This is the length of TX message (including the 2 byte CRC) - max is 1023 standard PHR mode allows up to 127 bytes
 * if > 127 is programmed, DWT_PHRMODE_EXT needs to be set in the phrMode configuration.
 *
 * @param dev                 Pointer to uwb_dev.
 * @param tx_frame_bytes      Pointer to the user buffer containing the data to send.
 * @param tx_buffer_offset    This specifies an offset in the tranceiver's TX Buffer where writing of data starts.
 * @param tx_frame_length     This is the total frame length, including the two byte CRC.
 * @return struct uwb_dev_status
 */
typedef struct uwb_dev_status (*uwb_write_tx_func_t)(struct uwb_dev* dev, uint8_t *tx_frame_bytes,
                                                uint16_t tx_buffer_offset, uint16_t tx_frame_length);

/**
 * Configure the TX frame control register before the transmission of a frame.
 *
 * @param dev               Pointer to struct uwb_dev.
 * @param tx_frame_length   This is the length of TX message (excluding the 2 byte CRC) - max is 1023
 *                          NOTE: standard PHR mode allows up to 127 bytes.
 * @param tx_buffer_offset  The offset in the tx buffer to start writing the data.
 * @return void
 */
typedef void (*uwb_write_tx_fctrl_func_t)(struct uwb_dev* dev, uint16_t tx_frame_length, uint16_t tx_buffer_offset);

/**
 * Enable wait for response feature.
 * 
 * @param dev               Pointer to struct uwb_dev.
 * @param enable            Enables/disables the wait for response feature.
 * @return struct uwb_dev_status
 */
typedef struct uwb_dev_status (*uwb_set_wait4resp_func_t)(struct uwb_dev *dev, bool enable);

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
 * @return struct uwb_dev_status
 *
 */
typedef struct uwb_dev_status (*uwb_set_wait4resp_delay_func_t)(struct uwb_dev * dev, uint32_t delay);

/**
 * Set rxauto disable 
 * 
 * @param dev      Pointer to struct uwb_dev.
 * @param disable  Disable mac-layer auto rx-reenable feature. The default behavior is rxauto enable, this API overrides default behavior
 * on an individual transaction such as in dw1000_rng_request or dw1000_rng_listen
 *
 */
typedef struct uwb_dev_status (*uwb_set_rxauto_disable_func_t)(struct uwb_dev * dev, bool disable);

/**
 * Read the current system time of the uwb tranceiver
 *
 * @param dev     Pointer to struct uwb_dev.
 * @return time   64bit uwt usecs
 */
typedef uint64_t (*uwb_read_systime_func_t)(struct uwb_dev* dev);

/**
 * Read lower 32bit of system time.
 *
 * @param dev      Pointer to struct uwb_dev.
 *
 * @return time
 */
typedef uint32_t (*uwb_read_systime_lo32_func_t)(struct uwb_dev* dev);

/**
 * Read receive time. (As adjusted by the LDE)
 *
 * @param inst  Pointer to struct uwb_dev.
 *
 * @return time
 */
typedef uint64_t (*uwb_read_rxtime_func_t)(struct uwb_dev* dev);

/**
 * Read lower 32bit of receive time.
 *
 * @param dev      Pointer to struct uwb_dev.
 *
 * @return time
 */
typedef uint32_t (*uwb_read_rxtime_lo32_func_t)(struct uwb_dev* dev);

/**
 * Read transmit time.
 *
 * @param dev      Pointer to struct uwb_dev.
 *
 * @return time
 */
typedef uint64_t (*uwb_read_txtime_func_t)(struct uwb_dev* dev);

/**
 * Read lower 32bit of transmit time.
 *
 * @param dev      Pointer to struct uwb_dev.
 *
 * @return time
 */
typedef uint32_t (*uwb_read_txtime_lo32_func_t)(struct uwb_dev* dev);

/**
 * Calculate the frame duration (airtime) in usecs (not uwb usecs).
 * @param attrib    Pointer to struct uwb_phy_attributes_t *. The phy attritubes are part of the IEEE802.15.4-2011 standard. 
 * Note the morphology of the frame depends on the mode of operation, see the dw1000_hal.c for the default behaviour
 * @param nlen      The length of the frame to be transmitted/received excluding crc
 * @return uint16_t duration in usec (not uwb usec)
 */
typedef uint16_t (*uwb_phy_frame_duration_func_t)(struct uwb_dev* dev, uint16_t nlen);

/**
 * API to calculate the SHR (Preamble + SFD) duration. This is used to calculate the correct rx_timeout.
 * @param attrib    Pointer to struct uwb_phy_attributes *. The phy attritubes are part of the IEEE802.15.4-2011 standard. 
 * Note the morphology of the frame depends on the mode of operation, see the dw1000_hal.c for the default behaviour
 * @param nlen      The length of the frame to be transmitted/received excluding crc
 * @return uint16_t duration in usec (not uwb usec)
 */
typedef uint16_t (*uwb_phy_SHR_duration_func_t)(struct uwb_dev* dev);

/**
 * Turn off the transceiver.
 *
 * @param inst  Pointer to struct uwb_dev.
 * @return void
 */
typedef void (*uwb_phy_forcetrxoff_func_t)(struct uwb_dev* dev);

/**
 * Enable rx regardless of hpdwarning
 *
 * @param inst    Pointer to struct uwb_dev.
 * @param enable  weather to continue with rx regardless of error
 *
 */
typedef struct uwb_dev_status (*uwb_set_on_error_continue_func_t)(struct uwb_dev * dev, bool enable);

/**
 * Update PAN ID in device
 *
 * @param inst    Pointer to struct uwb_dev.
 * @param uint16_t pan id
 *
 */
typedef void (*uwb_set_panid_func_t)(struct uwb_dev * dev, uint16_t pan_id);

/**
 * Calculate rssi from last RX in dBm. 
 * Note that config.rxdiag_enable needs to be set.
 *
 * @param inst  Pointer to struct uwb_dev.
 *
 * @return rssi (dBm) on success
 */
typedef float (*uwb_get_rssi_func_t)(struct uwb_dev * dev);

/**
 * Calculate First Path Power Level from last RX in dBm,
 * Note that config.rxdiag_enable needs to be set.
 *
 * @param inst  Pointer to struct uwb_dev.
 *
 * @return fppl (dBm) on success
 */
typedef float (*uwb_get_fppl_func_t)(struct uwb_dev * inst);

/**
 * Calculate rssi from an rxdiag structure
 *
 * @param inst  Pointer to struct uwb_dev.
 * @param diag  Pointer to struct uwb_dev_rxdiag.
 *
 * @return rssi (dBm) on success
 */
typedef float (*uwb_calc_rssi_func_t)(struct uwb_dev * dev, struct uwb_dev_rxdiag * diag);

/**
 * Calculate First Path Power Level (fppl) from an rxdiag structure
 *
 * @param inst  Pointer to struct uwb_dev.
 * @param diag  Pointer to struct uwb_dev_rxdiag.
 *
 * @return fppl (dBm) on success
 */
typedef float (*uwb_calc_fppl_func_t)(struct uwb_dev * dev, struct uwb_dev_rxdiag * diag);

/**
 * Give a rough estimate of how likely the received packet is
 * line of sight (LOS).
 *
 * @param rssi rssi as calculated by uwb_calc_rssi
 * @param fppl fppl as calculated by uwb_calc_fppl
 *
 * @return 1.0 for likely LOS, 0.0 for non-LOS, with a sliding scale in between.
 */
typedef float (*uwb_estimate_los_func_t)(struct uwb_dev * dev, float rssi, float fppl);

struct uwb_driver_funcs {
    uwb_mac_config_func_t uf_mac_config;
    uwb_txrf_config_func_t uf_txrf_config;
    uwb_sleep_config_func_t uf_sleep_config;
    uwb_enter_sleep_func_t uf_enter_sleep;
    uwb_enter_sleep_after_tx_func_t uf_enter_sleep_after_tx;
    uwb_enter_sleep_after_rx_func_t uf_enter_sleep_after_rx;
    uwb_wakeup_func_t uf_wakeup;
    uwb_set_rx_timeout_func_t uf_set_rx_timeout;
    uwb_adj_rx_timeout_func_t uf_adj_rx_timeout;
    uwb_set_delay_start_func_t uf_set_delay_start;
    uwb_start_tx_func_t uf_start_tx;
    uwb_start_rx_func_t uf_start_rx;
    uwb_stop_rx_func_t uf_stop_rx;
    uwb_write_tx_func_t uf_write_tx;
    uwb_write_tx_fctrl_func_t uf_write_tx_fctrl;
    uwb_set_wait4resp_func_t uf_set_wait4resp;
    uwb_set_wait4resp_delay_func_t uf_set_wait4resp_delay;
    uwb_set_rxauto_disable_func_t uf_set_rxauto_disable;
    uwb_read_systime_func_t uf_read_systime;
    uwb_read_systime_lo32_func_t uf_read_systime_lo32;
    uwb_read_rxtime_func_t uf_read_rxtime;
    uwb_read_rxtime_lo32_func_t uf_read_rxtime_lo32;
    uwb_read_txtime_func_t uf_read_txtime;
    uwb_read_txtime_lo32_func_t uf_read_txtime_lo32;
    uwb_phy_frame_duration_func_t uf_phy_frame_duration;
    uwb_phy_SHR_duration_func_t uf_phy_SHR_duration;
    uwb_phy_forcetrxoff_func_t uf_phy_forcetrxoff;
    uwb_set_on_error_continue_func_t uf_set_on_error_continue;
    uwb_set_panid_func_t uf_set_panid;
    uwb_get_rssi_func_t uf_get_rssi;
    uwb_get_fppl_func_t uf_get_fppl;
    uwb_calc_rssi_func_t uf_calc_rssi;
    uwb_calc_fppl_func_t uf_calc_fppl;
    uwb_estimate_los_func_t uf_estimate_los;
};

struct uwb_dev {
    struct os_dev uw_dev;
    const struct uwb_driver_funcs *uw_funcs;

    /* Interrupt handling */
    uint8_t task_prio;                          //!< Priority of the interrupt task
    struct dpl_eventq eventq;                   //!< Structure of dpl_eventq that has event queue
    struct dpl_event interrupt_ev;              //!< Structure of dpl_event that trigger interrupts
    struct dpl_task task_str;                   //!< Structure of dpl_task that has interrupt task
    dpl_stack_t task_stack[UWB_DEV_TASK_STACK_SZ] //!< Stack of the interrupt task
        __attribute__((aligned(DPL_STACK_ALIGNMENT)));

    /* RX data from latest frame received */
    union {
        uint16_t fctrl;                         //!< Reported frame control
        uint8_t fctrl_array[sizeof(uint16_t)];  //!< Endianness safe interface
    };
    uint16_t frame_len;                         //!< Reported frame length
    uint64_t rxtimestamp;                       //!< Receive timestamp
    int32_t carrier_integrator;                 //!< Carrier integrator
    struct uwb_dev_rxdiag *rxdiag;              //!< Pointer to rx diagnostics structure
    uint8_t rxbuf[MYNEWT_VAL(UWB_RX_BUFFER_SIZE)]; //!< Local receive buffer

    /* Device parameters */
    uint8_t idx;                                //!< instance number number {0, 1, 2 etc}
    uint16_t role;                              //!< Roles for this device
    union {
        uint16_t my_short_address;              //!< Short address of tag/node
        uint16_t uid;
    };
    union {
        uint64_t my_long_address;               //!< Long address of tag/node
        uint64_t euid;                          //!< Extended Unique Identifier
    };
    uint16_t pan_id;                            //!< Private network interface id
    uint16_t slot_id;                           //!< Slot id
    uint16_t cell_id;                           //!< Cell id
    uint32_t device_id;                         //!< Device id
    uint16_t rx_antenna_delay;                  //!< Receive antenna delay
    uint16_t tx_antenna_delay;                  //!< Transmit antenna delay
    int32_t ext_clock_delay;                    //!< External clock delay

    struct uwb_dev_status status;               //!< Device status
    struct uwb_dev_config config;               //!< Device configuration
    SLIST_HEAD(, uwb_mac_interface) interface_cbs;
    struct uwb_phy_attributes attrib;
};

#if 0
    hal_dw1000_rw_noblock_wait
    dw1000_read_rawrxtime ??? (only nmgr_uwb)
    dw1000_set_dblrxbuff 
    dw1000_calc_clock_offset_ratio
    rxttcko
#endif

/**
 * Configure the mac layer
 * @param inst     Pointer to struct uwb_dev.
 * @param config   Pointer to struct uwb_dev_config.
 * @return struct uwb_dev_status
 *
 */
static inline struct uwb_dev_status
uwb_mac_config(struct uwb_dev * dev, struct uwb_dev_config * config)
{
    return (dev->uw_funcs->uf_mac_config(dev, config));
}

/**
 * Configuration of the uwb transmitter
 * including the power and pulse generator delay. The input is a pointer to the data structure
 * of type struct uwb_dev_txrf_config that holds all the configurable items.
 *
 * @param inst      Pointer to struct uwb_dev
 * @param config    Pointer to struct uwb_dev_txrf_config.
 * @return void
 */
static inline void
uwb_txrf_config(struct uwb_dev * dev, struct uwb_dev_txrf_config *config)
{
    return (dev->uw_funcs->uf_txrf_config(dev, config));
}

/**
 *  Configure the device for both DEEP_SLEEP and SLEEP modes, and on-wake mode
 *  i.e., before entering the sleep, the device can be programmed for TX or RX, then upon "waking up" the TX/RX settings
 *  will be preserved and the device can immediately perform the desired action TX/RX.
 *
 * NOTE: e.g. Tag operation - after deep sleep, the device needs to just load the TX buffer and send the frame.
 *
 * @param inst  Pointer to struct uwb_dev.
 *
 * @return void
 */
static inline void uwb_sleep_config(struct uwb_dev * dev)
{
    return (dev->uw_funcs->uf_sleep_config(dev));
}

/**
 * API to enter device into sleep mode.
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 * @return struct uwb_dev_status
 */
static inline struct uwb_dev_status
uwb_enter_sleep(struct uwb_dev * dev)
{
    return (dev->uw_funcs->uf_enter_sleep(dev));
}

/** 
 * Set the auto TX to sleep bit. This means that after a frame
 * transmission the device will enter deep sleep mode. The uwb_sleep_config() function
 * needs to be called before this to configure the on-wake settings.
 *
 * NOTE: the IRQ line has to be low/inactive (i.e. no pending events)
 *
 * @param inst       Pointer to struct uwb_dev.
 * @param enable     1 to configure the device to enter deep sleep after TX, 
 *                   0 to disables the configuration.
 * @return struct uwb_dev_status
 */
static inline struct uwb_dev_status
uwb_enter_sleep_after_tx(struct uwb_dev * dev, uint8_t enable)
{
    return (dev->uw_funcs->uf_enter_sleep_after_tx(dev, enable));
}

/**
 *  Sets the auto RX to sleep bit. This means that after a frame
 *  received the device will enter deep sleep mode. The dev_configure_sleep() function
 *  needs to be called before this to configure the on-wake settings.
 *
 * NOTE: the IRQ line has to be low/inactive (i.e. no pending events).
 * @param inst       Pointer to struct uwb_dev.
 * @param enable     1 to configure the device to enter deep sleep after TX, 
 *                   0 to disables the configuration
 * @return struct uwb_dev_status
 */
static inline struct uwb_dev_status
uwb_enter_sleep_after_rx(struct uwb_dev * dev, uint8_t enable)
{
    return (dev->uw_funcs->uf_enter_sleep_after_rx(dev, enable));
}

/**
 * Wakeup device from sleep to init.
 *
 * @param inst  struct uwb_dev.
 * @return struct uwb_dev_status
 */
static inline struct uwb_dev_status
uwb_wakeup(struct uwb_dev * dev)
{
    return (dev->uw_funcs->uf_wakeup(dev));
}

/**
 * Set Receive Wait Timeout period.
 *
 * @param inst      pointer to struct uwb_dev.
 * @param timeout   Indicates how long the receiver remains on from the RX enable command. 
 *                  The time parameter used here is in 1.0256 * us (512/499.2MHz) units.
 *                  If set to 0 the timeout is disabled.
 * @return struct uwb_dev_status
 *
 * @brief The Receive Frame Wait Timeout period is a 32-bit field. The units for this parameter are roughly 1μs, 
 * (the exact unit is 512 counts of the fundamental 499.2 MHz UWB clock, or 1.026 μs). When employing the frame wait timeout, 
 * RXFWTO should be set to a value greater than the expected RX frame duration and include an allowance for any uncertainly 
 * attaching to the expected transmission start time of the awaited frame. 
 * When using .rxauto_enable feature it is important to understand the role of rx_timeout, in this situation it is the timeout 
 * that actually turns-off the receiver and returns the transeiver to the idle state. 
 *
 * NOTE: On dw1000 the timeout is a 16bit field only.
 */
static inline struct uwb_dev_status
uwb_set_rx_timeout(struct uwb_dev *dev, uint32_t to)
{
    return (dev->uw_funcs->uf_set_rx_timeout(dev, to));
}

/**
 * Adjust RX Wait Timeout period.
 *
 * @param inst      pointer to struct uwb_dev.
 * @param timeout   Indicates how long the receiver remains on from the RX enable command. The time parameter used here is in 1.0256 
 * us (512/499.2MHz) units If set to 0 the timeout is disabled.
 * @return struct uwb_dev_status
 * @brief The Receive Frame Wait Timeout period is a 32-bit field. The units for this parameter are roughly 1μs, 
 * (the exact unit is 512 counts of the fundamental 499.2 MHz UWB clock, or 1.026 μs). When employing the frame wait timeout, 
 * RXFWTO should be set to a value greater than the expected RX frame duration and include an allowance for any uncertainly 
 * attaching to the expected transmission start time of the awaited frame. 
 * When using .rxauto_enable feature it is important to understand the role of rx_timeout, in this situation it is the timeout 
 * that actually turns-off the receiver and returns the transeiver to the idle state.
 *
 * OBSERVE: This only works if the receiver has been setup to use a timeout with uwb_set_rx_timeout first.
 *
 * NOTE: On dw1000 the timeout is a 16bit field only.
 */
static inline struct uwb_dev_status
uwb_adj_rx_timeout(struct uwb_dev *dev, uint32_t to)
{
    return (dev->uw_funcs->uf_adj_rx_timeout(dev, to));
}

/**
 * To specify a time in future to either turn on the receiver to be ready to receive a frame, 
 * or to turn on the transmitter and send a frame. 
 * The low-order 9-bits of this register are ignored. 
 * The delay is in UWB microseconds.  
 * 
 * @param inst     Pointer to uwb_dev.
 * @param delay    Delayed Send or receive Time, in UWB microseconds.
 * @return uwb_dev_status
 */
static inline struct uwb_dev_status
uwb_set_delay_start(struct uwb_dev *dev, uint64_t dx_time)
{
    return (dev->uw_funcs->uf_set_delay_start(dev, dx_time));
}

/**
 * Start transmission.
 *
 * @param inst  pointer to struct uwb_dev.
 * @return struct uwb_dev_status
 */
static inline struct uwb_dev_status uwb_start_tx(struct uwb_dev * dev)
{
    return (dev->uw_funcs->uf_start_tx(dev));
}

/**
 * Activate reception mode (rx).
 *
 * @param inst  pointer to struct uwb_dev.
 * @return struct uwb_dev_status
 * 
 */
static inline struct uwb_dev_status uwb_start_rx(struct uwb_dev * dev)
{
    return (dev->uw_funcs->uf_start_rx(dev));
}

/**
 * Gracefully turn off reception mode.
 *
 * @param inst  pointer to struct uwb_dev
 * @return struct uwb_dev_status
 * 
 */
static inline struct uwb_dev_status uwb_stop_rx(struct uwb_dev *dev)
{
    return (dev->uw_funcs->uf_stop_rx(dev));
}

/**
 * Write the supplied TX data into the tranceiver's
 * TX buffer.The input parameters are the data length in bytes and a pointer
 * to those data bytes.
 *
 ***** NIKLAS TODO: This text should be fixed, no need to mention CRC bytes here?
 * Note: This is the length of TX message (including the 2 byte CRC) - max is 1023 standard PHR mode allows up to 127 bytes
 * if > 127 is programmed, DWT_PHRMODE_EXT needs to be set in the phrMode configuration.
 *
 * @param dev                 Pointer to uwb_dev.
 * @param tx_frame_bytes      Pointer to the user buffer containing the data to send.
 * @param tx_buffer_offset    This specifies an offset in the tranceiver's TX Buffer where writing of data starts.
 * @param tx_frame_length     This is the total frame length, including the two byte CRC.
 * @return struct uwb_dev_status
 */
static inline struct uwb_dev_status uwb_write_tx(struct uwb_dev* dev, uint8_t *tx_frame_bytes,
                                                 uint16_t tx_buffer_offset, uint16_t tx_frame_length)
{
    return (dev->uw_funcs->uf_write_tx(dev, tx_frame_bytes, tx_buffer_offset, tx_frame_length));
}
    
/**
 * Configure the TX frame control register before the transmission of a frame.
 *
 * @param dev               Pointer to struct uwb_dev.
 * @param tx_frame_length   This is the length of TX message (excluding the 2 byte CRC) - max is 1023
 *                          NOTE: standard PHR mode allows up to 127 bytes.
 * @param tx_buffer_offset  The offset in the tx buffer to start writing the data.
 * @return void
 */
static inline void uwb_write_tx_fctrl(struct uwb_dev* dev, uint16_t tx_frame_length, uint16_t tx_buffer_offset)
{
    return (dev->uw_funcs->uf_write_tx_fctrl(dev, tx_frame_length, tx_buffer_offset));
}

/**
 * Enable wait for response feature.
 * 
 * @param dev               Pointer to struct uwb_dev.
 * @param enable            Enables/disables the wait for response feature.
 * @return struct uwb_dev_status
 */
static inline struct uwb_dev_status uwb_set_wait4resp(struct uwb_dev *dev, bool enable)
{
    return (dev->uw_funcs->uf_set_wait4resp(dev, enable));
}

/**
 * Wait-for-Response turn-around Time. This 20-bit field is used to configure the turn-around time between TX complete 
 * and RX enable when the wait for response function is being used. This function is enabled by the WAIT4RESP control in 
 * Register file: 0x0D – System Control Register. The time specified by this W4R_TIM parameter is in units of approximately 1 μs, 
 * or 128 system clock cycles. This configuration may be used to save power by delaying the turn-on of the receiver, 
 * to align with the response time of the remote system, rather than turning on the receiver immediately after transmission completes.
 * 
 * @param inst   Pointer to struct uwb_dev.
 * @param delay  The delay is in UWB microseconds.
 *
 * @return struct uwb_dev_status
 *
 */
static inline struct uwb_dev_status uwb_set_wait4resp_delay(struct uwb_dev * dev, uint32_t delay)
{
    return (dev->uw_funcs->uf_set_wait4resp_delay(dev, delay));
}

/**
 * Set rxauto disable 
 * 
 * @param inst   Pointer to struct uwb_dev.
 * @param disable  Disable mac-layer auto rx-reenable feature. The default behavior is rxauto enable, this API overrides default behavior
 * on an individual transaction such as in dw1000_rng_request or dw1000_rng_listen
 *
 */
static inline struct uwb_dev_status uwb_set_rxauto_disable(struct uwb_dev * dev, bool disable)
{
    return (dev->uw_funcs->uf_set_rxauto_disable(dev, disable));
}

/**
 * Read the current system time of the uwb tranceiver
 *
 * @param dev     Pointer to struct uwb_dev.
 * @return time   64bit uwt usecs
 */
static inline uint64_t uwb_read_systime(struct uwb_dev* dev)
{
    return (dev->uw_funcs->uf_read_systime(dev));
}

/**
 * Read lower 32bit of system time.
 *
 * @param dev      Pointer to struct uwb_dev.
 *
 * @return time
 */
static inline uint32_t uwb_read_systime_lo32(struct uwb_dev* dev)
{
    return (dev->uw_funcs->uf_read_systime_lo32(dev));
}

/**
 * Read receive time. (As adjusted by the LDE)
 *
 * @param inst  Pointer to struct uwb_dev.
 *
 * @return time
 */
static inline uint64_t uwb_read_rxtime(struct uwb_dev* dev)
{
    return (dev->uw_funcs->uf_read_rxtime(dev));
}

/**
 * Read lower 32bit of receive time.
 *
 * @param dev      Pointer to struct uwb_dev.
 *
 * @return time
 */
static inline uint32_t uwb_read_rxtime_lo32(struct uwb_dev* dev)
{
    return (dev->uw_funcs->uf_read_rxtime_lo32(dev));
}

/**
 * Read transmit time.
 *
 * @param dev      Pointer to struct uwb_dev.
 *
 * @return time
 */
static inline uint64_t uwb_read_txtime(struct uwb_dev* dev)
{
    return (dev->uw_funcs->uf_read_txtime(dev));
}

/**
 * Read lower 32bit of transmit time.
 *
 * @param dev      Pointer to struct uwb_dev.
 *
 * @return time
 */
static inline uint32_t uwb_read_txtime_lo32(struct uwb_dev* dev)
{
    return (dev->uw_funcs->uf_read_txtime_lo32(dev));
}

/**
 * Calculate the frame duration (airtime) in usecs (not uwb usecs). 
 * @param attrib    Pointer to struct uwb_phy_attributes_t * struct. The phy attritubes are part of the IEEE802.15.4-2011 standard. 
 * Note the morphology of the frame depends on the mode of operation, see the dw1000_hal.c for the default behaviour
 * @param nlen      The length of the frame to be transmitted/received excluding crc
 * @return uint16_t duration in usec (not uwb usecs)
 */
static inline uint16_t uwb_phy_frame_duration(struct uwb_dev* dev, uint16_t nlen)
{
    return (dev->uw_funcs->uf_phy_frame_duration(dev, nlen));
}

/**
 * API to calculate the SHR (Preamble + SFD) duration. This is used to calculate the correct rx_timeout.
 * @param attrib    Pointer to struct uwb_phy_attributes *. The phy attritubes are part of the IEEE802.15.4-2011 standard. 
 * Note the morphology of the frame depends on the mode of operation, see the dw1000_hal.c for the default behaviour
 * @param nlen      The length of the frame to be transmitted/received excluding crc
 * @return uint16_t duration in usec (not uwb usec)
 */
static inline uint16_t uwb_phy_SHR_duration(struct uwb_dev* dev)
{
    return (dev->uw_funcs->uf_phy_SHR_duration(dev));
}

/**
 * Turn off the transceiver.
 *
 * @param inst  Pointer to struct uwb_dev.
 * @return void
 */
static inline void uwb_phy_forcetrxoff(struct uwb_dev* dev)
{
    return (dev->uw_funcs->uf_phy_forcetrxoff(dev));
}

/**
 * Enable rx regardless of hpdwarning
 * TODO: also for tx?
 * @param inst    Pointer to struct uwb_dev.
 * @param enable  weather to continue with rx regardless of error
 *
 */
static inline struct uwb_dev_status uwb_set_on_error_continue(struct uwb_dev * dev, bool enable)
{
    return (dev->uw_funcs->uf_set_on_error_continue(dev, enable));
}

/**
 * Update PAN ID in device
 *
 * @param inst    Pointer to struct uwb_dev.
 * @param uint16_t pan id
 *
 */
static inline void uwb_set_panid(struct uwb_dev * dev, uint16_t pan_id)
{
    return (dev->uw_funcs->uf_set_panid(dev, pan_id));
}

/**
 * Calculate rssi from last RX in dBm. 
 * Note that config.rxdiag_enable needs to be set.
 *
 * @param inst  Pointer to struct uwb_dev.
 *
 * @return rssi (dBm) on success
 */
static inline float uwb_get_rssi(struct uwb_dev * dev)
{
    return (dev->uw_funcs->uf_get_rssi(dev));
}

/**
 * Calculate First Path Power Level from last RX in dBm,
 * Note that config.rxdiag_enable needs to be set.
 *
 * @param inst  Pointer to struct uwb_dev.
 *
 * @return fppl (dBm) on success
 */
static inline float uwb_get_fppl(struct uwb_dev * dev)
{
    return (dev->uw_funcs->uf_get_fppl(dev));
}

/**
 * Calculate rssi from an rxdiag structure
 *
 * @param inst  Pointer to struct uwb_dev.
 * @param diag  Pointer to struct uwb_dev_rxdiag.
 *
 * @return rssi (dBm) on success
 */
static inline float uwb_calc_rssi(struct uwb_dev * dev, struct uwb_dev_rxdiag * diag)
{
    return (dev->uw_funcs->uf_calc_rssi(dev, diag));
}

/**
 * Calculate First Path Power Level (fppl) from an rxdiag structure
 *
 * @param inst  Pointer to struct uwb_dev.
 * @param diag  Pointer to struct uwb_dev_rxdiag.
 *
 * @return fppl (dBm) on success
 */
static inline float uwb_calc_fppl(struct uwb_dev * dev, struct uwb_dev_rxdiag * diag)
{
    return (dev->uw_funcs->uf_calc_fppl(dev, diag));
}

/**
 * Give a rough estimate of how likely the received packet is
 * line of sight (LOS).
 *
 * @param rssi rssi as calculated by uwb_calc_rssi
 * @param fppl fppl as calculated by uwb_calc_fppl
 *
 * @return 1.0 for likely LOS, 0.0 for non-LOS, with a sliding scale in between.
 */
static inline float uwb_estimate_los(struct uwb_dev * dev, float rssi, float fppl)
{
    return (dev->uw_funcs->uf_estimate_los(dev, rssi, fppl));
}

#define uwb_dwt_usecs_to_usecs(_t) (double)( (_t) * (0x10000UL/(128*499.2)))
#define uwb_usecs_to_dwt_usecs(_t) (double)( (_t) / uwb_dwt_usecs_to_usecs(1.0))

struct uwb_mac_interface *uwb_mac_append_interface(struct uwb_dev* dev, struct uwb_mac_interface * cbs);
void uwb_mac_remove_interface(struct uwb_dev* dev, uwb_extension_id_t id);
void* uwb_mac_find_cb_inst_ptr(struct uwb_dev *dev, uint16_t id);
struct uwb_mac_interface *uwb_mac_get_interface(struct uwb_dev* dev, uwb_extension_id_t id);
    
struct uwb_dev* uwb_dev_idx_lookup(int idx);

void uwb_task_init(struct uwb_dev * inst, void (*irq_ev_cb)(struct dpl_event*));

#ifdef __cplusplus
}
#endif

#endif /* __UWB_H__ */
