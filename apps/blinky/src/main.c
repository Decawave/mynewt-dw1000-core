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
    .txPreambLength = DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    .rxPAC = DWT_PAC8,                  /* Preamble acquisition chunk size. Used in RX only. */
    .txCode = 9,                        /* TX preamble code. Used in TX only. */
    .rxCode = 9,                        /* RX preamble code. Used in RX only. */
    .nsSFD = 0,                         /* 0 to use standard SFD, 1 to use non-standard SFD. */
    .dataRate = DWT_BR_6M8,             /* Data rate. */
    .phrMode = DWT_PHRMODE_STD,         /* PHY header mode. */
    .sfdTO = (129 + 8 - 8)             /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};


static ieee_blink_frame_t tx_frame = {
    .fctrl = 0xC5,              // frame control (0xC5 to indicate a data frame using 16-bit addressing).
    .seq_num = 0,               // sequence number, incremented for each new frame.
    .ext_address = 0x4556415741434544UL
};

/*
static ieee_blink_frame_t tx_frame = {
    .array = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0, 0},
};
*/
/* The timer callout */
static struct os_callout blinky_callout;

/*
 * Event callback function for timer events. It toggles the led pin.
 */

static void timer_ev_cb(struct os_event *ev) {
    assert(ev != NULL);

    hal_gpio_toggle(LED_BLINK_PIN);

    dw1000_dev_instance_t * inst = hal_dw1000_inst(0);
  
    dw1000_write_tx(inst, (uint8_t *)&tx_frame, 0, sizeof(ieee_blink_frame_t)); /* Zero offset in TX buffer. */
    dw1000_write_tx_fctrl(inst, sizeof(ieee_blink_frame_t), 0, false); /* Zero offset in TX buffer, no ranging. */
    dw1000_set_wait4resp(inst, true);
    dw1000_set_wait4resp_delay(inst, 3000);
    dw1000_set_rx_timeout(inst, 0);
    dw1000_start_tx(inst);

    tx_frame.seq_num++;
    os_callout_reset(&blinky_callout, OS_TICKS_PER_SEC/10);
}

static void init_timer(void) {
    /*
     * Initialize the callout for a timer event.
     */
    os_callout_init(&blinky_callout, os_eventq_dflt_get(), timer_ev_cb, NULL);
    os_callout_reset(&blinky_callout, OS_TICKS_PER_SEC/10);
}

void rx_complete_cb(dw1000_dev_instance_t * inst){

    /* Clear local RX buffer to avoid having leftovers from previous receptions. This is not necessary but is included here to aid reading the RX
     * buffer. 
     * */

    hal_gpio_toggle(LED_1);

    ieee_blink_frame_t rx_frame;
    if (inst->frame_len <= sizeof(ieee_blink_frame_t))
        dw1000_read_rx(inst, (uint8_t *)&rx_frame, 0, inst->frame_len);

    printf("[%02X:", rx_frame.fctrl);
    printf("%02X:", rx_frame.seq_num);
    printf("%lX", (uint32_t) (rx_frame.ext_address >> 32)); printf("%lX]\n", (uint32_t) (rx_frame.ext_address));

//    dw1000_phy_rxdiag_t rxdiag;
//    dw1000_phy_read_rxdiag(inst, &rxdiag);
//        printf("fp_idx=%X,fp_amp=%X,rx_std=%X,preamble_cnt=%X,wakeuptemp=%X\n",
//            rxdiag.fp_idx, rxdiag.fp_amp, rxdiag.rx_std,rxdiag.preamble_cnt, (uint16_t) dw1000_phy_read_wakeuptemp_SI(inst));
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

    init_timer();
    
    dw1000_softreset(inst);
   
    dw1000_write_reg(inst, PANADR_ID, PANADR_PAN_ID_OFFSET, 0xDECA, sizeof(inst->PANID));
    inst->PANID = dw1000_read_reg(inst, PANADR_ID, PANADR_PAN_ID_OFFSET, sizeof(inst->PANID));
    
    printf("device_id=%lX\n",inst->device_id);
    printf("PANID=%X\n",inst->PANID);
    printf("otp_rev =%X\n",inst->otp_rev);
    printf("partID =%lX\n",inst->partID);
    printf("lotID =%lX\n",inst->lotID);
    printf("xtal_trim =%X\n",inst->xtal_trim);
 
    //printf(".fctrl=%x\n",tx_msg.frame.fctrl);
    //dw1000_phy_txconfig_t txconfig={0xC0,0x25456585};
    //dw1000_phy_config_txrf(inst, &txconfig);
    
    dw1000_set_callbacks(inst, NULL, rx_complete_cb,  NULL,  NULL);
    dw1000_mac_init(inst, &config);
//    dw1000_set_wait4resp_delay(inst, 100UL);
//    dw1000_set_rx_timeout(inst, 0UL);
//    dw1000_set_wait4resp(inst, false);

//   dw1000_set_rxtx_delay(inst, 140UL);
//   dw1000_set_wait4resp_delay(inst, 140UL);

    while (1) {
        os_eventq_run(os_eventq_dflt_get());
    }
    assert(0);

    return rc;
}

