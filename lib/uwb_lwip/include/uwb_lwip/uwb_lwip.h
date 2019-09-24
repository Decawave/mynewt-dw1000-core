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
 * @file uwb_lwip.h
 * @author paul kettle
 * @date 2018
 * 
 * @brief lwip service
 * @details This is the lwip base class which utilizes the functions to do the configurations related to lwip layer based on dependencies.
 *
 */

#ifndef _UWB_LWIP_H_
#define _UWB_LWIP_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/hal_spi.h>
#include <uwb/uwb.h>
#include <uwb/uwb_ftypes.h>
#include <lwip/pbuf.h>
#include <lwip/ip_addr.h>
#include <lwip/netif.h>
#include <lwip/raw.h>

//! Lwip config parameters.
typedef struct _uwb_lwip_config_t{
   uint16_t poll_resp_delay;    //!< Delay between frames, in UWB microseconds.
   uint16_t resp_timeout;       //!< Receive response timeout, in UWB microseconds.
   uint32_t uwbtime_to_systime; //!< UWB time to system time
}uwb_lwip_config_t;

//! Lwip modes. 
typedef enum _uwb_lwip_modes_t{
    LWIP_BLOCKING,              //!< lwip blocking mode
    LWIP_NONBLOCKING            //!< lwip non-blocking mode
}uwb_lwip_modes_t;

//! Lwip status parameters.
typedef struct _uwb_lwip_status_t{
    uint32_t selfmalloc:1;             //!< Internal flag for memory garbage collection 
    uint32_t initialized:1;            //!< Instance allocated 
    uint32_t start_tx_error:1;         //!< Set for start transmit error 
    uint32_t start_rx_error:1;         //!< Set for start receive error 
    uint32_t tx_frame_error:1;         //!< Set transmit frame error
    uint32_t rx_error:1;               //!< Set for receive error
    uint32_t rx_timeout_error:1;       //!< Set for receive timeout error 
    uint32_t request_timeout:1;        //!< Set for request timeout
}uwb_lwip_status_t;

//! Lwip instance parameters.
typedef struct _uwb_lwip_instance_t{
    struct uwb_dev * dev_inst;             //!< Structure for uwb instance 
    struct uwb_mac_interface cbs;
    struct os_sem sem;                     //!< Structure for OS semaphores
    struct os_sem data_sem;                //!< Structure for data of semaphores

#if MYNEWT_VAL(UWB_LWIP_P2P)
    struct _uwb_lwip_p2p_instance_t * lwip_p2p;
#endif

    uwb_lwip_config_t * config;         //!< lwip config parameters 
    uwb_lwip_status_t status;           //!< lwip status
    uint16_t nframes;                      //!< Number of buffers defined to store the lwip data  
    uint16_t buf_idx;                      //!< Indicates number of buffer instances for the chosen bsp 
    uint16_t buf_len;                      //!< Indicates buffer length 
    uint16_t dst_addr;                     //!< Destination address    
    struct netif lwip_netif;               //!< Network interface
    struct raw_pcb * pcb;                  //!< Pointer to raw_pcb structure                       
    void * payload_ptr;                    //!< Pointer to payload 
    char * data_buf[];                     //!< Data buffers 
}uwb_lwip_instance_t;

//! Lwip callback. 
typedef struct _uwb_lwip_cb_t{
   void (*recv)(uwb_lwip_instance_t * lwip, uint16_t timeout);  //!< Keep tracks of lwip tx/rx status
}uwb_lwip_cb_t;

//! Lwip context parameters. 
typedef struct _uwb_lwip_context_t{
   uwb_lwip_cb_t rx_cb;    //!< DW1000 lwip receive callback
}uwb_lwip_context_t;

/**
 * [uwb_lwip_init Function to initialize the lwip service]
 * @param  inst    [Device/Parent instance]
 * @param  config  [Structure containing the configuration values]
 * @param  nframes [Number of frames to allocate memory for]
 * @param  buf_len [Buffer length of each frame]
 * @return         [Structure pointer to lwip]
 */
uwb_lwip_instance_t *
uwb_lwip_init(struct uwb_dev *inst, uwb_lwip_config_t * config, uint16_t nframes, uint16_t buf_len);

/**
 * [uwb_pcb_init Function to initialize a PCB for raw lwip]
 * @param   inst    [LWIP instance]
 */
void 
uwb_pcb_init(uwb_lwip_instance_t * lwip);

/**
 * [uwb_lwip_free Function to mark lwip serveice as free]
 * @param inst [Device/Parent Instance]
 */
void
uwb_lwip_free(uwb_lwip_instance_t * inst);



/**
 * [uwb_lwip_write Function to send lwIP buffer to radio]
 * @param  inst [LWIP instance]
 * @param  p    [lwIP Buffer to be sent to radio]
 * @param  mode [Blocking : Wait for Tx to complete]
 *              [Non-Blocking : Don't wait for Tx to complete]
 * @return      [Device Status]
 */
struct uwb_dev_status
uwb_lwip_write(uwb_lwip_instance_t * lwip, struct pbuf *p, uwb_lwip_modes_t code);

/**
 * [uwb_low_level_init Radio Low level initialization function]
 * @param inst        [Device/Parent instance]
 * @param txrf_config [Radio Tx and Rx configuration structure]
 * @param mac_config  [Radio MAC configuration structure]
 */
void
uwb_low_level_init( struct uwb_dev * inst, 
                       struct uwb_dev_txrf_config * txrf_config,
                       struct uwb_dev_config * mac_config);

/**
 * [uwb_netif_config lwIP network interface configuration function]
 * @param inst         [Device/Parent instance]
 * @param uwb_netif [Network interface structure to be configured]
 * @param my_ip_addr   [IP address of radio]
 * @param rx_status    [Defalut mode status]
 */
void
uwb_netif_config(uwb_lwip_instance_t * lwip, struct netif *netif, ip_addr_t *my_ip_addr, bool rx_status);

/**
 * [uwb_netif_init Network interface initialization function]
 * @param  uwb_netif [Network interface structure to be initialized]
 * @return              [Error status : Default ERR_OK]
 */
err_t
uwb_netif_init( struct netif * uwb_netif);

/**
 * [uwb_lwip_send function to pass the payload to lwIP stack]
 * @param inst         [Device/Parent instance]
 * @param payload_size [Size of the payload to be sent]
 * @param payload      [Pointer to the payload]
 * @param ipaddr       [Pointer to the IP address of target device]
 */
void 
uwb_lwip_send(uwb_lwip_instance_t *lwip, uint16_t payload_size, char * payload, ip_addr_t * ipaddr);

/**
 * [uwb_ll_output Low Level output function, acts as a brigde between 6lowPAN and radio]
 * @param  uwb_netif [Network interface]
 * @param  p            [Buffer to be sent to the radio]
 * @return              [Error status]
 */
err_t
uwb_ll_output(struct netif * uwb_netif, struct pbuf *p);

/**
 * [uwb_ll_input Low level input function, acts as a bridge between radio input and 6lowPAN]
 * @param  pt           [Pointer to received buffer from radio]
 * @param  uwb_netif [Network interface]
 * @return              [Error status]
 */
err_t
uwb_ll_input(struct pbuf *p, struct netif *uwb_netif);

/**
 * [uwb_lwip_start_rx Function to put the radio in Receive mode]
 * @param inst    [LWIP instance]
 * @param timeout [Timeout value for radio in receive mode]
 */
void
uwb_lwip_start_rx(uwb_lwip_instance_t * lwip, uint16_t timeout);

/**
 * [print_error Function to print error status and type]
 * @param error [Error Type]
 */
void print_error(err_t error);

/**
 * [lwip_rx_cb Received payload is fetched to this function after lwIP stack]
 * @param  arg  [User defined argument]
 * @param  pcb  [Pointer to PCB]
 * @param  p    [Payload pointer]
 * @param  addr [Device IP address]
 * @return      [1: Signifies that the payload is received successfully]
 */
uint8_t 
lwip_rx_cb(void *arg, struct raw_pcb *pcb, struct pbuf *p, const ip_addr_t *addr);

#ifdef __cplusplus
}
#endif
#endif /* _UWB_LWIP_H_ */
