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
 * @file dw1000_rng.c
 * @author paul kettle
 * @date 2018
 * @brief Range 
 *
 * @details This is the rng base class which utilises the functions to enable/disable the configurations related to rng.
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

static bool rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static int nmgr_resp_cb(struct nmgr_transport *nt, struct os_mbuf *m);
static void post_process(struct os_event *ev);

static struct os_callout rx_uwb_post;
struct nmgr_transport uwb_transport;
static uint8_t repeat_mode = 0;
static uint16_t rem_len = 0;
struct os_sem sem;

#if MYNEWT_VAL(DW1000_DEVICE_0)
static nmgr_uwb_frame_t g_nmgr_uwb_0 = {
    .PANID = MYNEWT_VAL(PANID),                 // PAN ID (0xDECA)
};
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
static twr_frame_t g_twr_1 = {
    .PANID = MYNEWT_VAL(PANID),                 // PAN ID (0xDECA)
};
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
static twr_frame_t g_twr_2 = {
    .PANID = MYNEWT_VAL(PANID),                 // PAN ID (0xDECA)
};
#endif

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
    return NMGR_UWB_MTU;
}

dw1000_nmgr_uwb_instance_t*
dw1000_nmgr_uwb_init(dw1000_dev_instance_t* inst){
    assert(inst != NULL);
    if(inst->nmgruwb == NULL){
        dw1000_nmgr_uwb_instance_t* nmgruwb = (dw1000_nmgr_uwb_instance_t*)malloc(sizeof(dw1000_nmgr_uwb_instance_t));
        assert(nmgruwb);
        nmgruwb->frame = &g_nmgr_uwb_0;
        nmgruwb->parent = inst;
        inst->nmgruwb = nmgruwb;
    }
    os_sem_init(&sem, 0x1);
    os_callout_init(&rx_uwb_post, os_eventq_dflt_get(), post_process, (void*)hal_dw1000_inst(0));
    return inst->nmgruwb;
}

/**
 * API to initialise the rng package.
 *
 *
 * @return void
 */
void nmgr_uwb_pkg_init(void){

    printf("{\"utime\": %lu,\"msg\": \"uwb_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

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
nmgr_resp_cb(struct nmgr_transport *nt, struct os_mbuf *m){

    //TODO: Need to find an alternative rather than initializing the instance this way
    dw1000_dev_instance_t* inst = hal_dw1000_inst(0);
    nmgr_uwb_frame_t *frame = inst->nmgruwb->frame;
    uint8_t usrlen = OS_MBUF_USRHDR_LEN(m);
    uint8_t pktlen = OS_MBUF_PKTLEN(m);
    uint8_t totlen = usrlen + pktlen;
    os_sem_pend(&sem, OS_TIMEOUT_NEVER);
    //Clear the entire frame. Just to make sure no
    //left overs from last transactions are going out
    memset(frame->array, 0x00, sizeof(frame->payload));
    //Copy the mbuf data to the frame to be sent
    os_mbuf_copydata(m, 0, pktlen, &frame->array[sizeof(struct _ieee_std_frame_t)]);
    memcpy(&frame->array[0], OS_MBUF_USRHDR(m), usrlen);
    if(repeat_mode == 0){
        if(htons(frame->hdr.nh_len) > NMGR_UWB_MTU){
            repeat_mode = 1;
            rem_len = htons(frame->hdr.nh_len) - (pktlen - sizeof(struct nmgr_hdr));
        }else{
            dw1000_set_wait4resp(inst, true);
            dw1000_set_rx_timeout(inst, 0);
        }
    }else{
        rem_len = rem_len - pktlen;
        if(rem_len == 0){
            repeat_mode = 0;
        }
        dw1000_set_wait4resp(inst, true);
        dw1000_set_rx_timeout(inst, 0);
        uint64_t request_timestamp = dw1000_read_systime(inst);
        uint64_t response_tx_delay = request_timestamp + ((uint64_t)(dw1000_phy_frame_duration(&inst->attrib, 128) + 0x400) << 16);
        dw1000_set_delay_start(inst, response_tx_delay); 
    }
    //Add the required UWB header info
    frame->code = NMGR_CMD_STATE_RSP;
    frame->dst_address = frame->src_address;
    frame->src_address = inst->my_short_address;
    dw1000_write_tx(inst, frame->array, 0, totlen);
    dw1000_write_tx_fctrl(inst, totlen, 0, true);
    if(dw1000_start_tx(inst).start_tx_error){
        os_sem_release(&sem);
        printf("Tx Error \n");
    }
    os_sem_pend(&sem, OS_TIMEOUT_NEVER);
    os_sem_release(&sem);
    if(repeat_mode == 0) 
       os_mbuf_free_chain(m);

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
    if(strncmp((char *)&inst->fctrl, "NM",2))
        return false;

    dw1000_nmgr_uwb_instance_t *nmgr = inst->nmgruwb;
    nmgr_uwb_frame_t *frame = nmgr->frame;
    //Read the buffer
    memset(frame, 0x00, sizeof(nmgr_uwb_frame_t));
    memcpy(&frame->array[0], inst->rxbuf, sizeof(nmgr_uwb_frame_t));

    if(frame->dst_address == inst->my_short_address){
        if(frame->code == NMGR_CMD_STATE_RSP)
            //Let the nmgr_cmds.c take responsibility of this
            return false;
        else{
            os_eventq_put(os_eventq_dflt_get(), &rx_uwb_post.c_ev);
            return true;
        }
    }else{
        dw1000_start_rx(inst);
    }
    return true;
    
}

static void post_process(struct os_event *ev){
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);
    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;

    dw1000_nmgr_uwb_instance_t *nmgr = inst->nmgruwb;
    nmgr_uwb_frame_t *frame = nmgr->frame;
    struct os_mbuf * mbuf = os_msys_get_pkthdr(inst->frame_len - sizeof(struct _ieee_std_frame_t), sizeof(struct _ieee_std_frame_t));
    assert(mbuf != NULL);
    assert(mbuf->om_data != NULL);
    //Copy the nmgr hdr & payload as the pkthdr. The UWB header will be stored as the usrhdr
    os_mbuf_copyinto(mbuf, 0, &frame->array[sizeof(struct _ieee_std_frame_t)], (inst->frame_len - sizeof(struct _ieee_std_frame_t)));
    //Copy the UWB header info also so that we can use it during sending the response
    memcpy(OS_MBUF_USRHDR(mbuf), &frame->array[0], sizeof(struct _ieee_std_frame_t));

    nmgr_rx_req(&uwb_transport, mbuf);
    return ;
}
/**
 * API for transmission complete callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
static bool
tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    if(strncmp((char *)&inst->fctrl, "NM",2))
        return false;
    if(os_sem_get_count(&sem) == 0)
        os_sem_release(&sem);
    return true;
}
