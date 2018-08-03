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

#ifndef _DW1000_LWIP_H_
#define _DW1000_LWIP_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/hal_spi.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_ftypes.h>
#include <dw1000/dw1000_phy.h>
#include <lwip/pbuf.h>
#include <lwip/ip_addr.h>
#include <lwip/netif.h>
#include <lwip/raw.h>


typedef struct _dw1000_lwip_config_t{
   uint16_t poll_resp_delay;    // Delay between frames, in UWB microseconds.
   uint16_t resp_timeout;       // Receive response timeout, in UWB microseconds.
   uint32_t uwbtime_to_systime;
}dw1000_lwip_config_t;

typedef enum _dw1000_lwip_modes_t{
    LWIP_BLOCKING,
    LWIP_NONBLOCKING
}dw1000_lwip_modes_t;

typedef struct _dw1000_lwip_status_t{
    uint32_t selfmalloc:1;
    uint32_t initialized:1;
    uint32_t start_tx_error:1;
    uint32_t start_rx_error:1;
    uint32_t tx_frame_error:1;
    uint32_t rx_error:1;
    uint32_t rx_timeout_error:1;
    uint32_t request_timeout:1;
}dw1000_lwip_status_t;

typedef struct _dw1000_lwip_instance_t{
    struct _dw1000_dev_instance_t * dev;
    struct os_sem sem;
    struct os_sem data_sem;

#if MYNEWT_VAL(DW1000_LWIP_P2P)
    struct _dw1000_lwip_p2p_instance_t * lwip_p2p;
#endif
    void (* ext_complete_cb) (struct _dw1000_dev_instance_t *); 
    void (* ext_tx_complete_cb) (struct _dw1000_dev_instance_t *);
    void (* ext_rx_complete_cb) (struct _dw1000_dev_instance_t *);
    void (* ext_rx_timeout_cb) (struct _dw1000_dev_instance_t *);
    void (* ext_rx_error_cb) (struct _dw1000_dev_instance_t *);

    dw1000_lwip_config_t * config;
    dw1000_lwip_status_t status;
    uint16_t nframes;
    uint16_t buf_idx;
    uint16_t buf_len;
    uint16_t dst_addr;
    struct netif lwip_netif;
    struct raw_pcb * pcb;
    void * payload_ptr;
    char * data_buf[];
}dw1000_lwip_instance_t;

typedef struct _dw1000_lwip_cb_t{
   void (*recv)(dw1000_dev_instance_t * inst, uint16_t timeout);
}dw1000_lwip_cb_t;

typedef struct _dw1000_lwip_context_t{
   dw1000_lwip_cb_t rx_cb;
}dw1000_lwip_context_t;

/**
 * [dw1000_lwip_config Function to assign the config parameters]
 * @param  inst   [Device/Parent instance]
 * @param  config [Sctructure containing the configuration values]
 * @return        [Device Status]
 */
dw1000_lwip_config_t *
dw1000_config(dw1000_dev_instance_t * inst);

/**
 * [dw1000_lwip_init Function to initialize the lwip service]
 * @param  inst    [Device/Parent instance]
 * @param  config  [Structure containing the configuration values]
 * @param  nframes [Number of frames to allocate memory for]
 * @param  buf_len [Buffer length of each frame]
 * @return         [Structure pointer to lwip]
 */
dw1000_lwip_instance_t *
dw1000_lwip_init(dw1000_dev_instance_t * inst, dw1000_lwip_config_t * config, uint16_t nframes, uint16_t buf_len);

void
dw1000_pcb_init(dw1000_dev_instance_t * inst);

/**
 * [dw1000_lwip_free Function to mark lwip serveice as free]
 * @param inst [Device/Parent Instance]
 */
void
dw1000_lwip_free(dw1000_lwip_instance_t * inst);

/**
 * [dw1000_lwip_set_callbacks Function to assign lwip callbacks]
 * @param inst           [Device/Parent instance]
 * @param tx_complete_cb [Transmit complete callback function]
 * @param rx_complete_cb [Receive complete callback function]
 * @param rx_timeout_cb  [Receive timeout callback function]
 * @param rx_error_cb    [Receive error callback function]
 */
void
dw1000_lwip_set_callbacks(dw1000_dev_instance_t * inst, dw1000_dev_cb_t tx_complete_cb, 
                            dw1000_dev_cb_t lwip_rx_complete_cb, dw1000_dev_cb_t rx_complete_cb, 
                            dw1000_dev_cb_t rx_timeout_cb, dw1000_dev_cb_t rx_error_cb);

//dw1000_lwip_set_callbacks(dw1000_dev_instance_t * inst, dw1000_dev_cb_t lwip_tx_complete_cb, 
//    dw1000_dev_cb_t lwip_rx_complete_cb,  dw1000_dev_cb_t lwip_timeout_cb,  dw1000_dev_cb_t lwip_error_cb);

/**
 * [dw1000_lwip_write Function to send lwIP buffer to radio]
 * @param  inst [Device/Parent instance]
 * @param  p    [lwIP Buffer to be sent to radio]
 * @param  mode [Blocking : Wait for Tx to complete]
 *              [Non-Blocking : Don't wait for Tx to complete]
 * @return      [Device Status]
 */
dw1000_dev_status_t
dw1000_lwip_write(dw1000_dev_instance_t * inst, struct pbuf *p, dw1000_lwip_modes_t code);

/**
 * [dw1000_low_level_init Radio Low level initialization function]
 * @param inst        [Device/Parent instance]
 * @param txrf_config [Radio Tx and Rx configuration structure]
 * @param mac_config  [Radio MAC configuration structure]
 */
void
dw1000_low_level_init( dw1000_dev_instance_t * inst, 
			dw1000_dev_txrf_config_t * txrf_config,
			dw1000_dev_config_t * mac_config);

/**
 * [dw1000_netif_config lwIP network interface configuration function]
 * @param inst         [Device/Parent instance]
 * @param dw1000_netif [Network interface structure to be configured]
 * @param my_ip_addr   [IP address of radio]
 * @param rx_status    [Defalut mode status]
 */
void
dw1000_netif_config( dw1000_dev_instance_t * inst, struct netif *netif, ip_addr_t *my_ip_addr, bool rx_status);

/**
 * [dw1000_netif_init Network interface initialization function]
 * @param  dw1000_netif [Network interface structure to be initialized]
 * @return              [Error status : Default ERR_OK]
 */
err_t
dw1000_netif_init( struct netif * dw1000_netif);

void 
dw1000_lwip_send(dw1000_dev_instance_t * inst, uint16_t payload_size, char * payload, ip_addr_t * ipaddr);
//dw1000_lwip_send(struct raw_pcb *pcb, struct pbuf *p, const ip_addr_t *ipaddr);

/**
 * [dw1000_ll_output Low Level output function, acts as a brigde between 6lowPAN and radio]
 * @param  dw1000_netif [Network interface]
 * @param  p            [Buffer to be sent to the radio]
 * @return              [Error status]
 */
err_t
dw1000_ll_output(struct netif * dw1000_netif, struct pbuf *p);

/**
 * [dw1000_ll_input Low level input function, acts as a bridge between radio input and 6lowPAN]
 * @param  pt           [Pointer to received buffer from radio]
 * @param  dw1000_netif [Network interface]
 * @return              [Error status]
 */
err_t
dw1000_ll_input(struct pbuf *p, struct netif *dw1000_netif);

/**
 * [dw1000_lwip_start_rx Function to put the radio in Receive mode]
 * @param inst    [Device/Parent instance]
 * @param timeout [Timeout value for radio in receive mode]
 */
void
dw1000_lwip_start_rx(dw1000_dev_instance_t * inst, uint16_t timeout);

/**
 * [print_error Function to print error status and type]
 * @param error [Error Type]
 */
void print_error(err_t error);

uint8_t lwip_rx_cb(void *arg, struct raw_pcb *pcb, struct pbuf *p, const ip_addr_t *addr);

#ifdef __cplusplus
}
#endif
#endif /* _DW1000_LWIP_H_ */
