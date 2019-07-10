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
    nmgr_uwb_frame_header_t uwb_hdr;
}__attribute__((__packed__,aligned(1)));

static bool rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static int nmgr_resp_cb(struct nmgr_transport *nt, struct os_mbuf *m);

static struct nmgr_transport uwb_transport_0;
#if MYNEWT_VAL(DW1000_DEVICE_1)
static struct nmgr_transport uwb_transport_1;
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
static struct nmgr_transport uwb_transport_2;
#endif

static struct nmgr_transport*
uwb_transport(int idx) {
    if (idx == 0) return &uwb_transport_0;
#if MYNEWT_VAL(DW1000_DEVICE_1)
    if (idx == 1) return &uwb_transport_1;
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    if (idx == 2) return &uwb_transport_2;
#endif
    return 0;
}

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


uint16_t
nmgr_uwb_mtu(struct os_mbuf *m, int idx)
{
    dw1000_dev_instance_t* inst = hal_dw1000_inst(idx);
    return (inst->config.rx.phrMode==DWT_PHRMODE_STD) ? NMGR_UWB_MTU_STD : NMGR_UWB_MTU_EXT;
}

static uint16_t
nmgr_uwb_mtu_0(struct os_mbuf *m)
{
    return nmgr_uwb_mtu(m, 0);
}


#if MYNEWT_VAL(DW1000_DEVICE_1)
static uint16_t
nmgr_uwb_mtu_1(struct os_mbuf *m, int idx)
{
    return nmgr_uwb_mtu(struct os_mbuf *m, 1);
}
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
static uint16_t
nmgr_uwb_mtu_2(struct os_mbuf *m, int idx)
{
    return nmgr_uwb_mtu(struct os_mbuf *m, 2);
}
#endif

nmgr_uwb_instance_t*
nmgr_uwb_init(dw1000_dev_instance_t* inst)
{
    assert(inst != NULL);
    if(inst->nmgruwb == NULL){
        nmgr_uwb_instance_t* nmgruwb = (nmgr_uwb_instance_t*)malloc(sizeof(nmgr_uwb_instance_t));
        assert(nmgruwb);
        nmgruwb->parent = inst;
        inst->nmgruwb = nmgruwb;
    }
    os_sem_init(&inst->nmgruwb->sem, 0x1);
    os_mqueue_init(&inst->nmgruwb->tx_q, NULL, NULL);

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
    SYSINIT_ASSERT_ACTIVE();
#if MYNEWT_VAL(DW1000_PKG_INIT_LOG)
    printf("{\"utime\": %lu,\"msg\": \"nmgr_uwb_init\"}\n", os_cputime_ticks_to_usecs(os_cputime_get32()));
#endif

#if MYNEWT_VAL(DW1000_DEVICE_0)
    nmgr_transport_init(uwb_transport(0), nmgr_resp_cb, nmgr_uwb_mtu_0);
    nmgr_uwb_init(hal_dw1000_inst(0));
    dw1000_mac_append_interface(hal_dw1000_inst(0), &g_cbs[0]);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    nmgr_uwb_init(hal_dw1000_inst(1));
    nmgr_transport_init(uwb_transport(1), nmgr_resp_cb, nmgr_uwb_mtu_1);
    dw1000_mac_append_interface(hal_dw1000_inst(1), &g_cbs[1]);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    nmgr_uwb_init(hal_dw1000_inst(2));
    nmgr_transport_init(uwb_transport(2), nmgr_resp_cb, nmgr_uwb_mtu_2);
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
    if(os_sem_get_count(&inst->nmgruwb->sem) == 0){
        os_sem_release(&inst->nmgruwb->sem);
        return true;
    }
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
    int rc;
    /* Prepare header and write to device */
    if (OS_MBUF_USRHDR_LEN(m) != sizeof(struct nmgr_uwb_usr_hdr)) {
        rc = os_mbuf_free_chain(m);
        return rc;
    }
    struct nmgr_uwb_usr_hdr *hdr = (struct nmgr_uwb_usr_hdr*)OS_MBUF_USRHDR(m);
    dw1000_dev_instance_t* inst = hal_dw1000_inst(hdr->inst_idx);
    assert(inst);
    nmgr_uwb_frame_header_t *frame = &hdr->uwb_hdr;

    if (hdr->uwb_hdr.dst_address == BROADCAST_ADDRESS) {
        rc = os_mbuf_free_chain(m);
        assert(rc==0);
        goto early_exit;
    }

    if (uwb_nmgr_queue_tx(inst, frame->src_address, NMGR_CMD_STATE_RSP, m) != 0) {
        rc = os_mbuf_free_chain(m);
        assert(rc==0);
    }

early_exit:
    if(os_sem_get_count(&inst->nmgruwb->sem) == 0){
        rc = os_sem_release(&inst->nmgruwb->sem);
        assert(rc==0);
    }
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
    bool ret = false;
    struct os_mbuf * mbuf;
    static uint16_t last_rpt_src=0;
    static uint8_t last_rpt_seq_num=0;

    if(strncmp((char *)&inst->fctrl, "NM",2)) {
        goto early_ret;
    }

    nmgr_uwb_frame_header_t *frame = (nmgr_uwb_frame_header_t*)inst->rxbuf;

    /* If this packet should be repeated, repeat it (unless already repeated) */
    if (frame->rpt_count < frame->rpt_max &&
        frame->dst_address != inst->my_short_address &&
        frame->src_address != inst->my_short_address &&
        !(frame->src_address = last_rpt_src && frame->seq_num != last_rpt_seq_num)
        ) {
        /* Avoid repeating more than once */
        last_rpt_src = frame->src_address;
        last_rpt_seq_num = frame->seq_num;
        frame->rpt_count++;

        dw1000_set_wait4resp(inst, true);
        dw1000_write_tx_fctrl(inst, inst->frame_len, 0);
        if (dw1000_start_tx(inst).start_tx_error) {
            /* Fail silently */
        } else {
            dw1000_write_tx(inst, inst->rxbuf, 0, inst->frame_len);
        }
    }

    if(frame->dst_address != inst->my_short_address && frame->dst_address != 0xffff) {
        goto early_ret;
    }

    switch(frame->code) {
        case NMGR_CMD_STATE_RSP: {
            /* Don't process responses here */
            break;
        }
        case NMGR_CMD_STATE_SEND: {
            ret = true;
            mbuf = os_msys_get_pkthdr(inst->frame_len - sizeof(nmgr_uwb_frame_header_t),
                                      sizeof(struct nmgr_uwb_usr_hdr));
            if (!mbuf) {
                printf("ERRMEM %d\n", inst->frame_len - sizeof(nmgr_uwb_frame_header_t) +
                       sizeof(struct nmgr_uwb_usr_hdr));
                break;
            }

            /* Copy the instance index and UWB header info so that we can use
             * it during sending the response */
            struct nmgr_uwb_usr_hdr *hdr = (struct nmgr_uwb_usr_hdr*)OS_MBUF_USRHDR(mbuf);
            hdr->inst_idx = inst->idx;
            memcpy(&hdr->uwb_hdr, inst->rxbuf, sizeof(nmgr_uwb_frame_header_t));

            /* Copy the nmgr hdr & payload */
            int rc = os_mbuf_copyinto(mbuf, 0, inst->rxbuf + sizeof(nmgr_uwb_frame_header_t),
                                      (inst->frame_len - sizeof(nmgr_uwb_frame_header_t)));
            if (rc == 0) {
                nmgr_rx_req(&uwb_transport_0, mbuf);
            } else {
                os_mbuf_free_chain(mbuf);
            }
            break;
        }
        default: {
            break;
        }
    }

early_ret:
    /* TODO: Check and reduce slot timeout */
    if(os_sem_get_count(&inst->nmgruwb->sem) == 0) {
        os_sem_release(&inst->nmgruwb->sem);
    }
    
    return ret;
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
    if(os_sem_get_count(&inst->nmgruwb->sem) == 0) {
        os_sem_release(&inst->nmgruwb->sem);
        return true;
    }
    return false;
}

/**
 * Listen for an incoming newtmgr data
 *
 * @param inst Pointer to dw1000_dev_instance_t.
 * @param mode DWT_BLOCKING or DWT_NONBLOCKING
 * @param inst Pointer to dw1000_dev_instance_t.
 *
 * @return dw1000_dev_status_t
 */
dw1000_dev_status_t
nmgr_uwb_listen(dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode, uint64_t delay, uint16_t timeout)
{
    os_error_t err;
    os_sem_pend(&inst->nmgruwb->sem, OS_TIMEOUT_NEVER);

    /* TODO: Persist listening until finished */
    dw1000_set_rx_timeout(inst, timeout);

    if (delay) {
        dw1000_set_delay_start(inst, delay);
    }

    if(dw1000_start_rx(inst).start_rx_error){
        err = os_sem_release(&inst->nmgruwb->sem);
        assert(err == OS_OK);
    }

    if (mode == DWT_BLOCKING){
        err = os_sem_pend(&inst->nmgruwb->sem, OS_TIMEOUT_NEVER);
        assert(err == OS_OK);
        err = os_sem_release(&inst->nmgruwb->sem);
        assert(err == OS_OK);
    }
    return inst->status;
}


int
nmgr_uwb_tx(dw1000_dev_instance_t* inst, uint16_t dst_addr, uint16_t code,
            struct os_mbuf *m, uint64_t dx_time)
{
    nmgr_uwb_frame_header_t uwb_hdr;

    uint8_t buf[32];
    int mbuf_offset = 0;
    int device_offset;
    os_sem_pend(&inst->nmgruwb->sem, OS_TIMEOUT_NEVER);

    /* Prepare header and write to device */
    uwb_hdr.src_address = inst->my_short_address;
    uwb_hdr.code = code;
    uwb_hdr.dst_address = dst_addr;
    uwb_hdr.seq_num = inst->nmgruwb->frame_seq_num++;
    uwb_hdr.PANID = 0xDECA;
    uwb_hdr.rpt_count = 0;
    uwb_hdr.rpt_max = MYNEWT_VAL(CCP_MAX_CASCADE_RPTS);

    /* TODO:BELOW IS UGLY, change to use code as identifier instead */
    strncpy((char*)&uwb_hdr.fctrl, "NM", 2);

    /* If fx_time provided, delay until then with tx */
    if (dx_time) {
        dw1000_set_delay_start(inst, dx_time);
    }

    dw1000_write_tx(inst, (uint8_t*)&uwb_hdr, 0, sizeof(nmgr_uwb_frame_header_t));
    device_offset = sizeof(nmgr_uwb_frame_header_t);

    /* Copy the mbuf payload data to the device to be sent */
    while (mbuf_offset < OS_MBUF_PKTLEN(m)) {
        int cpy_len = OS_MBUF_PKTLEN(m) - mbuf_offset;
        cpy_len = (cpy_len > sizeof(buf)) ? sizeof(buf) : cpy_len;

        /* The dw1000_write_tx can do a dma transfer, make sure we wait
         * until that's finished before updating the buffer */
        hal_dw1000_rw_noblock_wait(inst, OS_TIMEOUT_NEVER);
        os_mbuf_copydata(m, mbuf_offset, cpy_len, buf);
        dw1000_write_tx(inst, buf, device_offset, cpy_len);
        mbuf_offset += cpy_len;
        device_offset += cpy_len;
    }

    dw1000_write_tx_fctrl(inst, sizeof(nmgr_uwb_frame_header_t) + OS_MBUF_PKTLEN(m), 0);

    if(dw1000_start_tx(inst).start_tx_error){
        os_sem_release(&inst->nmgruwb->sem);
        printf("UWB NMGR_tx: Tx Error \n");
    }

    os_sem_pend(&inst->nmgruwb->sem, OS_TIMEOUT_NEVER);
    if(os_sem_get_count(&inst->nmgruwb->sem) == 0) {
        os_sem_release(&inst->nmgruwb->sem);
    }

    os_mbuf_free_chain(m);
    return 0;
}

int
uwb_nmgr_process_tx_queue(dw1000_dev_instance_t* inst, uint64_t dx_time)
{
    int rc;
    uint16_t dst_addr = 0;
    uint16_t code = 0;
    struct os_mbuf *om;

    if ((om = os_mqueue_get(&inst->nmgruwb->tx_q)) != NULL) {
        /* Extract dest address and code */
        rc = os_mbuf_copydata(om, OS_MBUF_PKTLEN(om)-4, sizeof(dst_addr), &dst_addr);
        assert(rc==0);
        rc = os_mbuf_copydata(om, OS_MBUF_PKTLEN(om)-2, sizeof(code), &code);
        assert(rc==0);
        os_mbuf_adj(om, -4);
        /* nmgr_uwb_tx consumes the mbuf */
        nmgr_uwb_tx(inst, dst_addr, code, om, dx_time);
        return true;
    }
    return false;
}

int
uwb_nmgr_queue_tx(dw1000_dev_instance_t* inst, uint16_t dst_addr, uint16_t code, struct os_mbuf *om)
{
#if MYNEWT_VAL(NMGR_UWB_LOOPBACK)
    nmgr_rx_req(&uwb_transport_0, om);
#else
    int rc;
    if (code==0) {
         code = NMGR_CMD_STATE_SEND;
    }

    /* Append the code and address to the end of the mbuf */
    uint16_t *p = os_mbuf_extend(om, sizeof(uint16_t)*2);
    if (!p) {
        printf("##### ERROR uwb_nmgr_q ext_failed\n");
        rc = os_mbuf_free_chain(om);
        return OS_EINVAL;
    }
    p[0] = dst_addr;
    p[1] = code;

    /* Enqueue the packet for sending at the next slot */
    rc = os_mqueue_put(&inst->nmgruwb->tx_q, NULL, om);
    if (rc != 0) {
        printf("##### ERROR uwb_nmgr_q rc:%d\n", rc);
        rc = os_mbuf_free_chain(om);
        return OS_EINVAL;
    }
#endif
    return 0;
}
