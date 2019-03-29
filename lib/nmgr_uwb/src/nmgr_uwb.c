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
 * @file nmgr_uwb.c
 * @author paul kettle
 * @date 2018
 * @brief Range 
 *
 * @details UWB Transport Layer for NewtMgr
 *
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <os/os.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include "bsp/bsp.h"
#include <stats/stats.h>

#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_ftypes.h>
#include <mgmt/mgmt.h>

#include <newtmgr/newtmgr.h>
#include <nmgr_uwb/nmgr_uwb.h>

//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

struct nmgr_uwb_usr_hdr{
    uint8_t inst_idx;
    struct _ieee_std_frame_t uwb_hdr;
}__attribute__((__packed__,aligned(1)));

static bool rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static int nmgr_resp_cb(struct nmgr_transport *nt, struct os_mbuf *m);

struct nmgr_transport uwb_transport;
struct os_sem sem;

static dw1000_mac_interface_t g_cbs[] = {
        [0] = {
            .id = DW1000_NMGR_UWB,
            .rx_complete_cb = rx_complete_cb,
            .tx_complete_cb = tx_complete_cb,
            .rx_timeout_cb = rx_timeout_cb,
        },
#if MYNEWT_VAL(DW1000_DEVICE_1)
        [1] = {
            .id = DW1000_NMGR_UWB,
            .rx_complete_cb = rx_complete_cb,
            .tx_complete_cb = tx_complete_cb,
            .rx_timeout_cb = rx_timeout_cb,
        },
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
        [2] = {
            .id = DW1000_NMGR_UWB,
            .rx_complete_cb = rx_complete_cb,
            .tx_complete_cb = tx_complete_cb,
            .rx_timeout_cb = rx_timeout_cb,
        }
#endif
};


static uint16_t
nmgr_uwb_mtu(struct os_mbuf *m)
{    
    return NMGR_UWB_MTU_STD;
}

dw1000_nmgr_uwb_instance_t*
dw1000_nmgr_uwb_init(dw1000_dev_instance_t* inst)
{
    assert(inst != NULL);
    if(inst->nmgruwb == NULL){
        dw1000_nmgr_uwb_instance_t* nmgruwb = (dw1000_nmgr_uwb_instance_t*)malloc(sizeof(dw1000_nmgr_uwb_instance_t));
        assert(nmgruwb);
        nmgruwb->parent = inst;
        inst->nmgruwb = nmgruwb;
    }
    os_sem_init(&sem, 0x1);
    return inst->nmgruwb;
}

/**
 * API to initialise the rng package.
 *
 *
 * @return void
 */
void nmgr_uwb_pkg_init(void)
{
    printf("{\"utime\": %lu,\"msg\": \"nmgr_uwb_init\"}\n", os_cputime_ticks_to_usecs(os_cputime_get32()));

#if MYNEWT_VAL(DW1000_DEVICE_0)
    SYSINIT_ASSERT_ACTIVE();

    nmgr_transport_init(&uwb_transport, nmgr_resp_cb, nmgr_uwb_mtu);
    dw1000_nmgr_uwb_init(hal_dw1000_inst(0));
    dw1000_mac_append_interface(hal_dw1000_inst(0), &g_cbs[0]);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    SYSINIT_ASSERT_ACTIVE();

    nmgr_transport_init(&uwb_transport, nmgr_resp_cb, nmgr_uwb_mtu);
    dw1000_mac_append_interface(hal_dw1000_inst(1), &g_cbs[1]);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    SYSINIT_ASSERT_ACTIVE();

    nmgr_transport_init(&uwb_transport, nmgr_resp_cb, nmgr_uwb_mtu);
    dw1000_mac_append_interface(hal_dw1000_inst(2), &g_cbs[2]);
#endif
}

/**
 * API for receive timeout callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
static bool 
rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    //For future use
    return false;
}

/** 
 * API for forming mngr response data
 *
 * @param nt   Pointer to newt manager transport structure
 * @param m    pointer to mbuf where the response data will be available
 * @return true on sucess
 */
static int
nmgr_resp_cb(struct nmgr_transport *nt, struct os_mbuf *m)
{
    uint8_t usrlen = OS_MBUF_USRHDR_LEN(m);
    uint8_t pktlen = OS_MBUF_PKTLEN(m);
    uint8_t totlen = usrlen + pktlen;
    uint8_t buf[64];
    int mbuf_offset = 0;
    int device_offset;
    os_sem_pend(&sem, OS_TIMEOUT_NEVER);

    /* Prepare header and write to device */
    struct nmgr_uwb_usr_hdr *hdr = (struct nmgr_uwb_usr_hdr*)OS_MBUF_USRHDR(m);
    dw1000_dev_instance_t* inst = hal_dw1000_inst(hdr->inst_idx);
    struct _ieee_std_frame_t *frame = &hdr->uwb_hdr;

    frame->code = NMGR_CMD_STATE_RSP;
    frame->dst_address = frame->src_address;
    frame->src_address = inst->my_short_address;
    dw1000_write_tx(inst, (uint8_t*)frame, 0, sizeof(struct _ieee_std_frame_t));
    dw1000_write_tx_fctrl(inst, totlen, 0);
    device_offset = sizeof(struct _ieee_std_frame_t);

    /* Copy the mbuf payload data to the device to be sent */
    while (mbuf_offset < OS_MBUF_PKTLEN(m)) {
        int cpy_len = OS_MBUF_PKTLEN(m) - mbuf_offset > sizeof(buf);
        cpy_len = (cpy_len) ? sizeof(buf) : cpy_len;
        printf("uwb_nmgr_resp: mbuf_offs:%d dev_offs:%d cpylen:%d\n",
               mbuf_offset, device_offset, cpy_len);

        os_mbuf_copydata(m, mbuf_offset, cpy_len, buf);
        dw1000_write_tx(inst, buf, device_offset, cpy_len);
        mbuf_offset += cpy_len;
        device_offset += cpy_len;
    }
    os_mbuf_free_chain(m);

    dw1000_set_wait4resp(inst, true);
    if(dw1000_start_tx(inst).start_tx_error){
        os_sem_release(&sem);
        printf("UWB NMGR: Tx Error \n");
    }
    os_sem_pend(&sem, OS_TIMEOUT_NEVER);
    os_sem_release(&sem);

    return 0;
}

/**
 * API for receive complete callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
static bool 
rx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    struct os_mbuf * mbuf;
    if(strncmp((char *)&inst->fctrl, "NM",2))
        return false;

    //Read the buffer
    struct _ieee_std_frame_t *frame = (struct _ieee_std_frame_t*)inst->rxbuf;
    if(frame->dst_address != inst->my_short_address &&
       frame->dst_address != 0xffff) {
        return false;
    }

    if(frame->code == NMGR_CMD_STATE_RSP) {
        //Let the nmgr_cmds.c take responsibility of this
        return false;
    } else {
        mbuf = os_msys_get_pkthdr(inst->frame_len - sizeof(struct _ieee_std_frame_t),
                                  sizeof(struct nmgr_uwb_usr_hdr));
        if (mbuf != NULL) {
            printf("Err, nomem\n");
            return true;
        }

        /* Copy the instance index and UWB header info so that we can use
         * it during sending the response */
        struct nmgr_uwb_usr_hdr *hdr = (struct nmgr_uwb_usr_hdr*)OS_MBUF_USRHDR(mbuf);
        hdr->inst_idx = inst->idx;
        memcpy(&hdr->uwb_hdr, inst->rxbuf, sizeof(struct _ieee_std_frame_t));

        /* Copy the nmgr hdr & payload */
        os_mbuf_copyinto(mbuf, 0, inst->rxbuf + sizeof(struct _ieee_std_frame_t),
                         (inst->frame_len - sizeof(struct _ieee_std_frame_t)));

        nmgr_rx_req(&uwb_transport, mbuf);
        return true;
    }
    
    return true;
}

/**
 * API for transmission complete callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
static bool
tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    if(strncmp((char *)&inst->fctrl, "NM",2)) {
        return false;
    }
    if(os_sem_get_count(&sem) == 0) {
        os_sem_release(&sem);
    }
    return true;
}

int
nmgr_uwb_tx(uint16_t dst_addr, struct os_mbuf *m, uint64_t dx_time)
{
    dw1000_dev_instance_t* inst = hal_dw1000_inst(0);
    struct _ieee_std_frame_t uwb_hdr;

    uint8_t usrlen = OS_MBUF_USRHDR_LEN(m);
    uint8_t pktlen = OS_MBUF_PKTLEN(m);
    uint8_t buf[32];
    int mbuf_offset = 0;
    int device_offset;
    printf("uwb_nmgr tx: usrhdr %d, pktlen %d\n", usrlen, pktlen);
    os_sem_pend(&sem, OS_TIMEOUT_NEVER);

    /* Prepare header and write to device */
    uwb_hdr.src_address = inst->my_short_address;
    uwb_hdr.code = NMGR_CMD_STATE_SEND;
    uwb_hdr.dst_address = dst_addr;
    uwb_hdr.seq_num = inst->nmgruwb->frame_seq_num++;
    uwb_hdr.PANID = 0xDECA;
    /* TODO:BELOW IS UGLY */
    strncpy((char*)&uwb_hdr.fctrl, "NM", 2);

    dw1000_write_tx(inst, (uint8_t*)&uwb_hdr, 0, sizeof(struct _ieee_std_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(struct _ieee_std_frame_t) + OS_MBUF_PKTLEN(m), 0);
    device_offset = sizeof(struct _ieee_std_frame_t);

    /* Copy the mbuf payload data to the device to be sent */
    while (mbuf_offset < OS_MBUF_PKTLEN(m)) {
        int cpy_len = OS_MBUF_PKTLEN(m) - mbuf_offset > sizeof(buf);
        cpy_len = (cpy_len) ? sizeof(buf) : cpy_len;
        printf("uwb_nmgr_tx: mbuf_offs:%d dev_offs:%d cpylen:%d\n",
               mbuf_offset, device_offset, cpy_len);

        os_mbuf_copydata(m, mbuf_offset, cpy_len, buf);
        dw1000_write_tx(inst, buf, device_offset, cpy_len);
        mbuf_offset += cpy_len;
        device_offset += cpy_len;
    }
    os_mbuf_free_chain(m);

    dw1000_set_wait4resp(inst, true);
    if(dw1000_start_tx(inst).start_tx_error){
        os_sem_release(&sem);
        printf("UWB NMGR_tx: Tx Error \n");
    }
    os_sem_pend(&sem, OS_TIMEOUT_NEVER);
    os_sem_release(&sem);

    return 0;
}
