/**
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

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "sysinit/sysinit.h"
#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#include "hal/hal_bsp.h"
#ifdef ARCH_sim
#include "mcu/mcu_sim.h"
#endif

#include "dw1000/dw1000_dev.h"
#include "dw1000/dw1000_hal.h"
#include "dw1000/dw1000_phy.h"
#include "dw1000/dw1000_mac.h"
#include "dw1000/dw1000_rng.h"
#include <dw1000/dw1000_lwip.h>
#include "dw1000/dw1000_ftypes.h"

static dwt_config_t config = {
    .chan = 5,                          /* Channel number. */
    .prf = DWT_PRF_64M,                 /* Pulse repetition frequency. */
    .txPreambLength = DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    .rxPAC = DWT_PAC8,                  /* Preamble acquisition chunk size. Used in RX only. */
    .txCode = 8,                        /* TX preamble code. Used in TX only. */
    .rxCode = 9,                        /* RX preamble code. Used in RX only. */
    .nsSFD = 0,                         /* 0 to use standard SFD, 1 to use non-standard SFD. */
    .dataRate = DWT_BR_6M8,             /* Data rate. */
    .phrMode = DWT_PHRMODE_STD,         /* PHY header mode. */
    .sfdTO = (129 + 8 - 8)              /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

static dw1000_rng_config_t rng_config = {
    .wait4resp_delay = 0x0600,         // Delayed Send or Receive Time in usec.
    .rx_timeout_period = 0            // Receive response timeout in usec.
};

ss_twr_range_t ss_twr_range = {
    .request = {
        .fctrl = 0x8841,            // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = 0xDECA             // PAN ID (0xDECA)
    }
};


/* The timer callout */
static struct os_callout blinky_callout;

/*
 * Event callback function for timer events. It toggles the led pin.
*/
static void timer_ev_cb(struct os_event *ev) {
    assert(ev != NULL);

    hal_gpio_toggle(LED_BLINK_PIN);
    dw1000_dev_instance_t * inst = hal_dw1000_inst(0);
    ss_twr_range_t * ss_twr  = inst->rng->ss_twr;  

#if 0
    if (dw1000_rng_request(inst, NULL, NULL, DW1000_SS_TWR).rx_error)
        printf("timer_ev_cb:rng_request failed [status.mac_error]\n");
#else
    if (inst->status.rx_error)
            printf("timer_ev_cb:rng_request failed [rx_error]\n");
    if (inst->status.start_rx_error)
            printf("timer_ev_cb:rng_request failed [start_rx_error]\n");
    if (inst->status.start_tx_error){
            printf("timer_ev_cb:rng_request failed [start_tx_error]\n");
            inst->status.start_tx_error = 0;
            dw1000_start_rx(inst); 
    }
    if (inst->status.start_tx_error || inst->status.rx_error || inst->status.range_request_timeout ||  inst->status.rx_timeout_error)
        dw1000_start_rx(inst); 
#endif

            printf("{\n\tfctrl:0x%04X,\n", ss_twr->response.fctrl);
            printf("\tseq_num:0x%02X,\n", ss_twr->response.seq_num);
            printf("\tPANID:0x%04X,\n", ss_twr->response.PANID);
            printf("\tdst_address:0x%04X,\n", ss_twr->response.dst_address);
            printf("\tsrc_address:0x%04X,\n", ss_twr->response.src_address);
            printf("\tcode:0x%04X,\n", ss_twr->response.code);
            printf("\treception_timestamp:0x%08lX,\n", ss_twr->response.reception_timestamp); 
            printf("\ttransmission_timestamp:0x%08lX,\n", ss_twr->response.transmission_timestamp); 
            printf("\trequest_timestamp:0x%08lX,\n", ss_twr->request_timestamp); 
            printf("\tresponse_timestamp:0x%08lX\n}\n", ss_twr->response_timestamp);

    os_callout_reset(&blinky_callout, OS_TICKS_PER_SEC);
}

static void init_timer(void) {
    /*
     * Initialize the callout for a timer event.
     */
    os_callout_init(&blinky_callout, os_eventq_dflt_get(), timer_ev_cb, NULL);
    os_callout_reset(&blinky_callout, OS_TICKS_PER_SEC);
}

int main(int argc, char **argv)
{
    int rc;

#ifdef ARCH_sim
    mcu_sim_parse_args(argc, argv);
#endif
    
    sysinit();
    dw1000_dev_instance_t * inst = hal_dw1000_inst(0);
    dw1000_dev_init(inst, MYNEWT_VAL(DW1000_DEVICE_0_SPI_IDX));
    
    hal_gpio_init_out(LED_BLINK_PIN, 1);
    hal_gpio_init_out(LED_1, 1);
    hal_gpio_init_out(LED_3, 1);

    inst->PANID = 0xDECA;
    inst->my_short_address = 0x4321;

    init_timer();
    dw1000_softreset(inst);
    dw1000_write_reg(inst, PANADR_ID, PANADR_PAN_ID_OFFSET, inst->PANID, sizeof(inst->PANID));
    
    printf("device_id=%lX\n",inst->device_id);
    printf("PANID=%X\n",inst->PANID);
    printf("partID =%lX\n",inst->partID);
    printf("lotID =%lX\n",inst->lotID);
    printf("xtal_trim =%X\n",inst->xtal_trim);

    dw1000_mac_init(inst, &config);
    dw1000_rng_init(inst, &rng_config);
    dw1000_rng_set_frames(inst, &ss_twr_range);
    dw1000_start_rx(inst); 

    while (1) {
        os_eventq_run(os_eventq_dflt_get());   
    }

    assert(0);

    return rc;
}

