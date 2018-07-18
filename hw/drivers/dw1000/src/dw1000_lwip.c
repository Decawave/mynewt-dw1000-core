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
#include <os/os.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include "bsp/bsp.h"

#if MYNEWT_VAL(DW1000_LWIP)

#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_ftypes.h>
#include <dw1000/dw1000_lwip.h>

#include <dw1000/dw1000_phy.h>
#include "sysinit/sysinit.h"


#include <lwip/pbuf.h>
#include <lwip/netif.h>
#include <netif/lowpan6.h>
#include <lwip/ethip6.h>
#include <lwip/icmp.h>
#include <lwip/inet_chksum.h>


// TODOs::This file is a place holder for the lwip project

static void rx_complete_cb(dw1000_dev_instance_t * inst);
static void tx_complete_cb(dw1000_dev_instance_t * inst);
static void rx_timeout_cb(dw1000_dev_instance_t * inst);
static void rx_error_cb(dw1000_dev_instance_t * inst);
dw1000_lwip_context_t cntxt;


dw1000_dev_status_t
dw1000_lwip_config(dw1000_dev_instance_t * inst, dw1000_lwip_config_t * config){

	assert(inst);
	assert(config);

	inst->lwip->config = config;
	return inst->status;
}


dw1000_lwip_instance_t *
dw1000_lwip_init(dw1000_dev_instance_t * inst, dw1000_lwip_config_t * config, uint16_t nframes, uint16_t buf_len){

	assert(inst);
	if (inst->lwip == NULL ){
		inst->lwip  = (dw1000_lwip_instance_t *) malloc(sizeof(dw1000_lwip_instance_t) + nframes * sizeof(char *));
		assert(inst->lwip);
		memset(inst->lwip,0,sizeof(dw1000_lwip_instance_t) + nframes * sizeof(char *));
		inst->lwip->status.selfmalloc = 1;
		inst->lwip->nframes = nframes;
		inst->lwip->buf_len = buf_len;
		inst->lwip->buf_idx = 0;

		for(uint16_t i=0 ; i < nframes ; ++i){
			inst->lwip->data_buf[i]  = (char *) malloc(sizeof(char)*buf_len);
			assert(inst->lwip->data_buf[i]);
		}
	}
	os_error_t err = os_sem_init(&inst->lwip->sem, 0x01);
	assert(err == OS_OK);
	err = os_sem_init(&inst->lwip->data_sem, nframes);
	assert(err == OS_OK);

	if (config != NULL){
		inst->lwip->config = config;
		dw1000_lwip_config(inst, config);
	}

	dw1000_lwip_set_callbacks(inst, tx_complete_cb, rx_complete_cb, rx_timeout_cb, rx_error_cb);

	inst->lwip->status.initialized = 1;

	return inst->lwip;
}


void 
dw1000_lwip_free(dw1000_lwip_instance_t * inst){

	assert(inst);
	if (inst->status.selfmalloc)
		free(inst);
	else
		inst->status.initialized = 0;
}


void
dw1000_lwip_set_callbacks( dw1000_dev_instance_t * inst, dw1000_dev_cb_t tx_complete_cb, 
				dw1000_dev_cb_t rx_complete_cb,	dw1000_dev_cb_t rx_timeout_cb, 
				dw1000_dev_cb_t rx_error_cb){

	inst->tx_complete_cb = tx_complete_cb;
	inst->rx_complete_cb = rx_complete_cb;
	inst->rx_timeout_cb = rx_timeout_cb;
	inst->rx_error_cb = rx_error_cb;
}


dw1000_dev_status_t
dw1000_lwip_write(dw1000_dev_instance_t * inst, struct pbuf *p, dw1000_lwip_modes_t mode){

	/* Semaphore lock for multi-threaded applications */
	os_error_t err = os_sem_pend(&inst->lwip->sem, OS_TIMEOUT_NEVER);
	assert(err == OS_OK);
	assert(p != NULL);

	dw1000_write_tx(inst, (uint8_t *) p, 0, inst->lwip->buf_len);
	dw1000_write_tx_fctrl(inst, inst->lwip->buf_len, 0, false);
	inst->lwip->netif->flags = 5 ;
	inst->lwip->status.start_tx_error = dw1000_start_tx(inst).start_tx_error;

	if( mode == LWIP_BLOCKING )
		err = os_sem_pend(&inst->lwip->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions units os_clicks
	else
		err = os_sem_pend(&inst->lwip->sem, 500); // Wait for completion of transactions units os_clicks

	os_sem_release(&inst->lwip->sem);
	return inst->status;
}


void
dw1000_lwip_start_rx(dw1000_dev_instance_t * inst, uint16_t timeout){

	os_error_t err = os_sem_pend(&inst->lwip->data_sem, OS_TIMEOUT_NEVER);
	assert(err == OS_OK);

	dw1000_set_rx_timeout(inst, timeout);
	dw1000_start_rx(inst);
}


static void 
rx_complete_cb(dw1000_dev_instance_t * inst){


	uint16_t buf_idx = (inst->lwip->buf_idx++) % inst->lwip->nframes;
	char *data_buf = inst->lwip->data_buf[ buf_idx];

	dw1000_read_rx(inst, (uint8_t *) data_buf, 0, inst->lwip->buf_len);
	inst->lwip->netif->input((struct pbuf *)data_buf, inst->lwip->netif);
	os_error_t err = os_sem_release(&inst->lwip->data_sem);
	assert(err == OS_OK);

}

static void 
tx_complete_cb(dw1000_dev_instance_t * inst){

	os_error_t err = os_sem_release(&inst->lwip->sem);
	assert(err == OS_OK);
}


static void 
rx_timeout_cb(dw1000_dev_instance_t * inst){

	os_error_t err = os_sem_release(&inst->lwip->data_sem);
	assert(err == OS_OK);

	inst->lwip->status.rx_timeout_error = 1;
}


void
rx_error_cb(dw1000_dev_instance_t * inst){

	os_error_t err = os_sem_release(&inst->lwip->data_sem);
	assert(err == OS_OK);

	inst->lwip->status.rx_error = 1;
}


void
dw1000_low_level_init( dw1000_dev_instance_t * inst, 
			dw1000_dev_txrf_config_t * txrf_config,
			dw1000_dev_config_t * mac_config){

	dw1000_phy_init(inst, txrf_config);
	dw1000_mac_init(inst, mac_config) ;
}


void
dw1000_netif_config(dw1000_dev_instance_t *inst, struct netif *dw1000_netif, ip_addr_t *my_ip_addr, bool rx_status){

	netif_add(dw1000_netif, NULL, dw1000_netif_init, ip6_input);
	IP_ADDR6_HOST(dw1000_netif->ip6_addr, 	my_ip_addr->addr[0], my_ip_addr->addr[1],
						my_ip_addr->addr[2], my_ip_addr->addr[3]);

	dw1000_netif->ip6_addr_state[0] = IP6_ADDR_VALID;

	netif_set_default(dw1000_netif);
	netif_set_link_up(dw1000_netif);
	netif_set_up(dw1000_netif);

	inst->lwip->netif = netif_default;
	cntxt.rx_cb.recv = dw1000_lwip_start_rx; 
	inst->lwip->netif->state = (void*)&cntxt;
	
	if(rx_status)
		dw1000_lwip_start_rx(inst, 0xffff);
}


err_t
dw1000_netif_init(struct netif *dw1000_netif){

	LWIP_ASSERT("netif != NULL", (dw1000_netif != NULL));

	dw1000_netif->hostname = "twr_lwip";
	dw1000_netif->name[0] = 'D';
	dw1000_netif->name[1] = 'W';
	dw1000_netif->hwaddr_len = 2;
	dw1000_netif->input = dw1000_ll_input;
	dw1000_netif->linkoutput = dw1000_ll_output;

	return ERR_OK;
}


err_t
dw1000_ll_output(struct netif *dw1000_netif, struct pbuf *p){

	dw1000_dev_instance_t * inst = hal_dw1000_inst(0);
	dw1000_lwip_write(inst, p, LWIP_BLOCKING);

	err_t error = ERR_OK;

	if (inst->lwip->status.request_timeout)
		error = ERR_INPROGRESS;

	if (inst->lwip->status.rx_timeout_error)
		error = ERR_TIMEOUT;

	return error;
}


err_t
dw1000_ll_input(struct pbuf *pt, struct netif *dw1000_netif){

	err_t error = ERR_OK;
	pt->payload = pt + sizeof(struct pbuf)/sizeof(struct pbuf);
	error = lowpan6_input(pt, dw1000_netif);
	print_error(error);

	return error;
}


void 
print_error(err_t error){

	switch(error){
		case ERR_MEM :
			printf("[Memory Error]\n");
			break;
		case ERR_BUF :
			printf("[Buffer Error]\n");
			break;
		case ERR_TIMEOUT :
			printf("[Timeout Error]\n");
			break;
		case ERR_RTE :
			printf("[Routing Error]\n");
			break;
		case ERR_INPROGRESS :
			printf("[Inprogress Error]\n");
			break;
		case ERR_OK :
		default :
			break;
	}
}

#endif  /* End MYNEWT_VAL(DW1000_LWIP) */
