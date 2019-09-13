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
 * @file twr_ss.c
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

#include <stats/stats.h>
#include <uwb/uwb.h>
#include <uwb/uwb_ftypes.h>
#include <rng/rng.h>
#include <dsp/polyval.h>

#if MYNEWT_VAL(UWB_WCS_ENABLED)
#include <uwb_wcs/uwb_wcs.h>
#endif

#if MYNEWT_VAL(RNG_VERBOSE)
#define DIAGMSG(s,u) printf(s,u)
#endif
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

static bool rx_complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs);
static bool reset_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs);
static bool start_tx_error_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs);

static struct uwb_mac_interface g_cbs[] = {
        [0] = {
            .id = UWBEXT_RNG_SS,
            .rx_complete_cb = rx_complete_cb,
            .start_tx_error_cb = start_tx_error_cb,
            .reset_cb = reset_cb
        },
#if MYNEWT_VAL(DW1000_DEVICE_1) ||  MYNEWT_VAL(DW1000_DEVICE_2)
        [1] = {
            .id = UWBEXT_RNG_SS,
            .rx_complete_cb = rx_complete_cb,
            .start_tx_error_cb = start_tx_error_cb,
            .reset_cb = reset_cb
        },
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
        [2] = {
            .id = UWBEXT_RNG_SS,
            .rx_complete_cb = rx_complete_cb,
            .start_tx_error_cb = start_tx_error_cb,
            .reset_cb = reset_cb
        }
#endif
};

STATS_SECT_START(twr_ss_stat_section)
    STATS_SECT_ENTRY(complete)
    STATS_SECT_ENTRY(tx_error)
    STATS_SECT_ENTRY(reset)
STATS_SECT_END

STATS_NAME_START(twr_ss_stat_section)
    STATS_NAME(twr_ss_stat_section, complete)
    STATS_NAME(twr_ss_stat_section, tx_error)
    STATS_NAME(twr_ss_stat_section, reset)
STATS_NAME_END(twr_ss_stat_section)

static STATS_SECT_DECL(twr_ss_stat_section) g_stat;

static dw1000_rng_config_t g_config = {
    .tx_holdoff_delay = MYNEWT_VAL(TWR_SS_TX_HOLDOFF),         // Send Time delay in usec.
    .rx_timeout_delay = MYNEWT_VAL(TWR_SS_RX_TIMEOUT)       // Receive response timeout in usec
};

static struct rng_config_list g_rng_cfgs[] = {
    [0] = {
        .rng_code = DWT_SS_TWR,
        .config = &g_config
    },
#if MYNEWT_VAL(DW1000_DEVICE_1) ||  MYNEWT_VAL(DW1000_DEVICE_2)
    [1] = {
        .rng_code = DWT_SS_TWR,
        .config = &g_config
    },
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    [2] = {
        .rng_code = DWT_SS_TWR,
        .config = &g_config
    },
#endif
};


/**
 * @fn twr_ss_pkg_init(void)
 * @brief API to initialise the rng_ss package.
 *
 * @return void
 */
void
twr_ss_pkg_init(void){

    printf("{\"utime\": %lu,\"msg\": \"twr_ss_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

    struct uwb_dev *udev;
#if MYNEWT_VAL(DW1000_DEVICE_0)
    udev = uwb_dev_idx_lookup(0);
    g_cbs[0].inst_ptr = (dw1000_rng_instance_t*)uwb_mac_find_cb_inst_ptr(udev, UWBEXT_RNG);
    uwb_mac_append_interface(udev, &g_cbs[0]);
    dw1000_rng_append_config(g_cbs[0].inst_ptr, &g_rng_cfgs[0]);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    udev = uwb_dev_idx_lookup(1);
    g_cbs[1].inst_ptr = (dw1000_rng_instance_t*)uwb_mac_find_cb_inst_ptr(udev, UWBEXT_RNG);
    uwb_mac_append_interface(udev, &g_cbs[1]);
    dw1000_rng_append_config(g_cbs[1].inst_ptr, &g_rng_cfgs[1]);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    udev = uwb_dev_idx_lookup(2);
    g_cbs[2].inst_ptr = (dw1000_rng_instance_t*)uwb_mac_find_cb_inst_ptr(udev, UWBEXT_RNG);
    uwb_mac_append_interface(udev, &g_cbs[2]);
    dw1000_rng_append_config(g_cbs[2].inst_ptr, &g_rng_cfgs[2]);
#endif

    int rc = stats_init(
        STATS_HDR(g_stat),
        STATS_SIZE_INIT_PARMS(g_stat, STATS_SIZE_32),
        STATS_NAME_INIT_PARMS(twr_ss_stat_section));
    assert(rc == 0);

    rc |= stats_register("twr_ss", STATS_HDR(g_stat));
    assert(rc == 0);
}

/**
 * @fn twr_ss_free(dw1000_dev_instance_t * inst)
 * @brief API to free the allocated resources.
 *
 * @param inst  Pointer to dw1000_rng_instance_t.
 *
 * @return void
 */
void
twr_ss_free(struct uwb_dev * dev)
{
    assert(dev);
    uwb_mac_remove_interface(dev, UWBEXT_RNG_SS);
}

/**
 * @fn start_tx_error_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
 * @brief API for start tx error callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @param cbs   Pointer to struct uwb_mac_interface.
 *
 * @return true on sucess
 */
static bool
start_tx_error_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
{
    STATS_INC(g_stat, tx_error);
    return true;
}

/**
 * @fn reset_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
 * @brief API for reset_cb of rng interface
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @param cbs    Pointer to struct uwb_mac_interface.
 *
 * @return true on sucess
 */
static bool
reset_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
{
    dw1000_rng_instance_t * rng = (dw1000_rng_instance_t *)cbs->inst_ptr;
    assert(rng);
    if(dpl_sem_get_count(&rng->sem) == 0){
        STATS_INC(g_stat, reset);
        dpl_error_t err = dpl_sem_release(&rng->sem);
        assert(err == DPL_OK);
        return true;
    }
    else
       return false;
}

/**
 * @fn rx_complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
 * @brief API for receive complete callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @param cbs   Pointer to dw1000_mac_interace_t.
 *
 * @return true on sucess
 */
static bool
rx_complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
{
    if (inst->fctrl != FCNTL_IEEE_RANGE_16)
        return false;

    dw1000_rng_instance_t * rng = (dw1000_rng_instance_t *)cbs->inst_ptr;
    assert(rng);
    if(dpl_sem_get_count(&rng->sem) == 1) // unsolicited inbound
        return false;

    twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes]; // Frame already read within loader layers.

    switch(rng->code){
        case DWT_SS_TWR:
            {
                // This code executes on the device that is responding to a request

                uint64_t request_timestamp = inst->rxtimestamp;
                uint64_t response_tx_delay = request_timestamp + ((uint64_t) g_config.tx_holdoff_delay << 16);
                uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;

#if MYNEWT_VAL(UWB_WCS_ENABLED)
                struct uwb_wcs_instance * wcs = rng->ccp_inst->wcs;
                frame->reception_timestamp = (uint32_t)(uwb_wcs_local_to_master(wcs, request_timestamp)) & 0xFFFFFFFFULL;
                frame->transmission_timestamp = (uint32_t)(uwb_wcs_local_to_master(wcs, response_timestamp)) & 0xFFFFFFFFULL;
#else
                frame->reception_timestamp = request_timestamp & 0xFFFFFFFFULL;
                frame->transmission_timestamp = response_timestamp & 0xFFFFFFFFULL;
#endif

                frame->dst_address = frame->src_address;
                frame->src_address = inst->my_short_address;
                frame->code = DWT_SS_TWR_T1;

#if MYNEWT_VAL(UWB_WCS_ENABLED)
                frame->carrier_integrator  = 0.0l;
#else
                frame->carrier_integrator  = - inst->carrier_integrator;
#endif
                // Write the second part of the response
                uwb_write_tx(inst, frame->array ,0 ,sizeof(ieee_rng_response_frame_t));
                uwb_write_tx_fctrl(inst, sizeof(ieee_rng_response_frame_t), 0);
                uwb_set_wait4resp(inst, true); 
                 
                uint16_t frame_duration = uwb_phy_frame_duration(inst,sizeof(ieee_rng_response_frame_t));
                uint16_t shr_duration  = uwb_phy_SHR_duration(inst);
                uint16_t data_duration = frame_duration - shr_duration;
                uwb_set_wait4resp_delay(inst, g_config.tx_holdoff_delay - data_duration - shr_duration);
                uwb_set_delay_start(inst, response_tx_delay);
                uwb_set_rx_timeout(inst, frame_duration + g_config.rx_timeout_delay);

                // Disable default behavor, do not RXENAB on RXFCG thereby avoiding rx timeout events on sucess  
                uwb_set_rxauto_disable(inst, true);

                if (uwb_start_tx(inst).start_tx_error){
                    STATS_INC(g_stat, tx_error);
                    dpl_sem_release(&rng->sem);
                    if (cbs!=NULL && cbs->start_tx_error_cb)
                        cbs->start_tx_error_cb(inst, cbs);
                }
                break;
            }
        case DWT_SS_TWR_T1:
            {
                // This code executes on the device that initiated a request, and is now preparing the final timestamps
                if (inst->frame_len != sizeof(ieee_rng_response_frame_t))
                    break;
                if(inst->status.lde_error)
                    break;

                uint64_t response_timestamp = inst->rxtimestamp;
#if MYNEWT_VAL(UWB_WCS_ENABLED)
                struct uwb_wcs_instance * wcs = rng->ccp_inst->wcs;
                frame->request_timestamp = uwb_wcs_local_to_master(wcs, uwb_read_txtime(inst)) & 0xFFFFFFFFULL;
                frame->response_timestamp = uwb_wcs_local_to_master(wcs, response_timestamp) & 0xFFFFFFFFULL;
#else
                frame->request_timestamp = uwb_read_txtime_lo32(inst) & 0xFFFFFFFFULL;
                frame->response_timestamp  = (uint32_t)(response_timestamp & 0xFFFFFFFFULL);
#endif
                frame->dst_address = frame->src_address;
                frame->src_address = inst->my_short_address;
                frame->code = DWT_SS_TWR_FINAL;
#if MYNEWT_VAL(UWB_WCS_ENABLED)
                frame->carrier_integrator  = 0.0l;
#else
                frame->carrier_integrator  = inst->carrier_integrator;
#endif
                // Transmit timestamp final report
                uwb_write_tx(inst, frame->array, 0,  sizeof(twr_frame_final_t));
                uwb_write_tx_fctrl(inst, sizeof(twr_frame_final_t), 0);
                uint64_t final_tx_delay = response_timestamp + ((uint64_t) g_config.tx_holdoff_delay << 16);
                uwb_set_delay_start(inst, final_tx_delay);
                
                if (uwb_start_tx(inst).start_tx_error){
                    dpl_sem_release(&rng->sem);
                    if (cbs!=NULL && cbs->start_tx_error_cb)
                        cbs->start_tx_error_cb(inst, cbs);
                }
                else{
                    STATS_INC(g_stat, complete);
                    dpl_sem_release(&rng->sem);
                    struct uwb_mac_interface * cbs = NULL;
                    if(!(SLIST_EMPTY(&inst->interface_cbs))){
                        SLIST_FOREACH(cbs, &inst->interface_cbs, next){
                            if (cbs!=NULL && cbs->complete_cb)
                                if(cbs->complete_cb(inst, cbs)) continue;
                            }
                    }
                }
                break;
            }
        case  DWT_SS_TWR_FINAL:
            {
                // This code executes on the device that responded to the original request, and has now receive the response final timestamp.
                // This marks the completion of the single-size-two-way request. This final 4th message is perhaps optional in some applicaiton.

                if (inst->frame_len != sizeof(twr_frame_final_t))
                   break;

                STATS_INC(g_stat, complete);
                dpl_sem_release(&rng->sem);
                struct uwb_mac_interface * cbs = NULL;
                if(!(SLIST_EMPTY(&inst->interface_cbs))){
                    SLIST_FOREACH(cbs, &inst->interface_cbs, next){
                    if (cbs!=NULL && cbs->complete_cb)
                        if(cbs->complete_cb(inst, cbs)) continue;
                    }
                }
                break;
            }
        default:
                return false;
                break;
    }

    return true;
}
