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
 * @file rng.c
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
#include <uwb/uwb_mac.h>
#include <dsp/polyval.h>

#if MYNEWT_VAL(UWB_RNG_ENABLED)
#include <uwb_rng/uwb_rng.h>
#include <uwb_rng/rng_encode.h>
#endif
#if MYNEWT_VAL(UWB_WCS_ENABLED)
#include <uwb_wcs/uwb_wcs.h>
#endif
#if MYNEWT_VAL(CIR_ENABLED)
#include <cir/cir.h>
#endif


#if MYNEWT_VAL(RNG_STATS)
STATS_NAME_START(rng_stat_section)
    STATS_NAME(rng_stat_section, rng_request)
    STATS_NAME(rng_stat_section, rng_listen)
    STATS_NAME(rng_stat_section, tx_complete)
    STATS_NAME(rng_stat_section, rx_complete)
    STATS_NAME(rng_stat_section, rx_unsolicited)
    STATS_NAME(rng_stat_section, rx_error)
    STATS_NAME(rng_stat_section, tx_error)
    STATS_NAME(rng_stat_section, rx_timeout)
    STATS_NAME(rng_stat_section, reset)
STATS_NAME_END(rng_stat_section)

#define RNG_STATS_INC(__X) STATS_INC(rng->stat, __X)
#else
#define RNG_STATS_INC(__X) {}
#endif

//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

static bool rx_complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs);
static bool tx_complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs);
static bool reset_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs);
static bool rx_timeout_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs);
#if MYNEWT_VAL(RNG_VERBOSE)
static bool complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs);
#endif

/*
% From APS011 Table 2
rls = [-61,-63,-65,-67,-69,-71,-73,-75,-77,-79,-81,-83,-85,-87,-89,-91,-93];
bias = [-11,-10.4,-10.0,-9.3,-8.2,-6.9,-5.1,-2.7,0,2.1,3.5,4.2,4.9,6.2,7.1,7.6,8.1]./100;
p=polyfit(rls,bias,3)
mat2c(p,'rng_bias_poly_PRF64')
bias = [-19.8,-18.7,-17.9,-16.3,-14.3,-12.7,-10.9,-8.4,-5.9,-3.1,0,3.6,6.5,8.4,9.7,10.6,11.0]./100;
p=polyfit(rls,bias,3)
mat2c(p,'rng_bias_poly_PRF16')
*/
static float rng_bias_poly_PRF64[] ={
        1.404476e-03, 3.208478e-01, 2.349322e+01, 5.470342e+02,
     	};
static float rng_bias_poly_PRF16[] ={
        1.754924e-05, 4.106182e-03, 3.061584e-01, 7.189425e+00,
     	};

static struct uwb_rng_config g_config = {
    .tx_holdoff_delay = MYNEWT_VAL(RNG_TX_HOLDOFF),       // Send Time delay in usec.
    .rx_timeout_delay = MYNEWT_VAL(RNG_RX_TIMEOUT)       // Receive response timeout in usec
};

#if MYNEWT_VAL(UWB_DEVICE_0)
static twr_frame_t g_twr_0[] = {
    [0] = {
        .fctrl = FCNTL_IEEE_RANGE_16,               // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = MYNEWT_VAL(PANID),                 // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID
    },
    [1] = {
        .fctrl = FCNTL_IEEE_RANGE_16,               // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = MYNEWT_VAL(PANID),                 // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID,
    },
    [2] = {
        .fctrl = FCNTL_IEEE_RANGE_16,               // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = MYNEWT_VAL(PANID),                 // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID
    },
    [3] = {
        .fctrl = FCNTL_IEEE_RANGE_16,                // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = MYNEWT_VAL(PANID),                  // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID,
    }
};
#endif
#if MYNEWT_VAL(UWB_DEVICE_1)
static twr_frame_t g_twr_1[] = {
    [0] = {
        .fctrl = FCNTL_IEEE_RANGE_16,                // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = MYNEWT_VAL(PANID),                  // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID
    },
    [1] = {
        .fctrl = FCNTL_IEEE_RANGE_16,                // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = MYNEWT_VAL(PANID),                  // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID,
    },
    [2] = {
        .fctrl = FCNTL_IEEE_RANGE_16,                // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = MYNEWT_VAL(PANID),                  // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID
    },
    [3] = {
        .fctrl = FCNTL_IEEE_RANGE_16,                // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = MYNEWT_VAL(PANID),                  // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID,
    }
};
#endif
#if MYNEWT_VAL(UWB_DEVICE_2)
static twr_frame_t g_twr_2[] = {
    [0] = {
        .fctrl = FCNTL_IEEE_RANGE_16,                // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = MYNEWT_VAL(PANID),                 // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID
    },
    [1] = {
        .fctrl = FCNTL_IEEE_RANGE_16,                // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = MYNEWT_VAL(PANID),                 // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID,
    },
    [2] = {
        .fctrl = FCNTL_IEEE_RANGE_16,                // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = MYNEWT_VAL(PANID),                  // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID
    },
    [3] = {
        .fctrl = FCNTL_IEEE_RANGE_16,                // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = MYNEWT_VAL(PANID),                  // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID,
    }

};
#endif

static struct uwb_mac_interface g_cbs[] = {
        [0] = {
            .id = UWBEXT_RNG,
            .rx_complete_cb = rx_complete_cb,
            .tx_complete_cb = tx_complete_cb,
            .rx_timeout_cb = rx_timeout_cb,
#if MYNEWT_VAL(RNG_VERBOSE)
            .complete_cb  = complete_cb,
#endif
            .reset_cb = reset_cb
        },
#if MYNEWT_VAL(UWB_DEVICE_1)
        [1] = {
            .id = UWBEXT_RNG,
            .rx_complete_cb = rx_complete_cb,
            .tx_complete_cb = tx_complete_cb,
            .rx_timeout_cb = rx_timeout_cb,
#if MYNEWT_VAL(RNG_VERBOSE)
            .complete_cb  = complete_cb,
#endif
            .reset_cb = reset_cb
        },
#endif
#if MYNEWT_VAL(UWB_DEVICE_2)
        [2] = {
            .id = UWBEXT_RNG,
            .rx_complete_cb = rx_complete_cb,
            .tx_complete_cb = tx_complete_cb,
            .rx_timeout_cb = rx_timeout_cb,
#if MYNEWT_VAL(RNG_VERBOSE)
            .complete_cb  = complete_cb,
#endif
            .reset_cb = reset_cb
        }
#endif
};

/**
 * @fn uwb_rng_init(struct uwb_dev * inst, struct uwb_rng_config * config, uint16_t nframes)
 * @brief API to initialise the ranging by setting all the required configurations and callbacks.
 *
 * @param inst      Pointer to struct uwb_dev.
 * @param config    Pointer to the structure struct uwb_rng_config.
 * @param nframes   Number of buffers defined to store the ranging data.
 *
 * @return struct uwb_rng_instance
 */
struct uwb_rng_instance *
uwb_rng_init(struct uwb_dev * dev, struct uwb_rng_config * config, uint16_t nframes)
{
    assert(dev);

    struct uwb_rng_instance *rng = (struct uwb_rng_instance*)uwb_mac_find_cb_inst_ptr(dev, UWBEXT_RNG);
    if (rng == NULL ) {
        rng = (struct uwb_rng_instance *) malloc(sizeof(struct uwb_rng_instance) + nframes * sizeof(twr_frame_t *)); // struct + flexible array member
        assert(rng);
        memset(rng, 0, sizeof(struct uwb_rng_instance));
        rng->status.selfmalloc = 1;
        rng->nframes = nframes;
    }
    rng->dev_inst = dev;
#if MYNEWT_VAL(UWB_WCS_ENABLED)
    rng->ccp_inst = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(dev, UWBEXT_CCP);
    assert(rng->ccp_inst);
#endif
    dpl_error_t err = dpl_sem_init(&rng->sem, 0x1);
    assert(err == DPL_OK);

    if (config != NULL ) {
        uwb_rng_config(rng, config);
    }

    rng->control = (uwb_rng_control_t){
        .delay_start_enabled = 0,
    };
    rng->idx = 0xFFFF;
    rng->status.initialized = 1;
    
#if MYNEWT_VAL(RNG_STATS)
    int rc = stats_init(
                    STATS_HDR(rng->stat),
                    STATS_SIZE_INIT_PARMS(rng->stat, STATS_SIZE_32),
                    STATS_NAME_INIT_PARMS(rng_stat_section)
            );

#if  MYNEWT_VAL(UWB_DEVICE_0) && !MYNEWT_VAL(UWB_DEVICE_1)
        rc |= stats_register("rng", STATS_HDR(rng->stat));
#elif  MYNEWT_VAL(UWB_DEVICE_0) && MYNEWT_VAL(UWB_DEVICE_1)
    if (dev->idx == 0)
        rc |= stats_register("rng0", STATS_HDR(rng->stat));
    else
        rc |= stats_register("rng1", STATS_HDR(rng->stat));
#endif
    assert(rc == 0);
#endif
    return rng;
}

/**
 * @fn uwb_rng_free(struct uwb_rng_instance * inst)
 * @brief API to free the allocated resources.
 *
 * @param inst  Pointer to struct uwb_rng_instance.
 *
 * @return void
 */
void
uwb_rng_free(struct uwb_rng_instance * rng){

    assert(rng);
    if (rng->status.selfmalloc)
        free(rng);
    else
        rng->status.initialized = 0;
}

/**
 * @fn uwb_rng_pkg_init(void)
 * @brief API to initialise the rng package.
 *
 * @return void
 */

void
uwb_rng_pkg_init(void)
{
#if MYNEWT_VAL(UWB_PKG_INIT_LOG)
    printf("{\"utime\": %lu,\"msg\": \"rng_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
#endif

    struct uwb_rng_instance *rng;
    struct uwb_dev *udev;
#if MYNEWT_VAL(UWB_DEVICE_0)
    udev = uwb_dev_idx_lookup(0);
    g_cbs[0].inst_ptr = rng = uwb_rng_init(udev, &g_config, sizeof(g_twr_0)/sizeof(twr_frame_t));
    uwb_rng_set_frames(rng, g_twr_0, sizeof(g_twr_0)/sizeof(twr_frame_t));
    uwb_mac_append_interface(udev, &g_cbs[0]);
#endif
#if MYNEWT_VAL(UWB_DEVICE_1)
    udev = uwb_dev_idx_lookup(1);
    g_cbs[1].inst_ptr = rng = uwb_rng_init(udev, &g_config, sizeof(g_twr_1)/sizeof(twr_frame_t));
    uwb_rng_set_frames(rng, g_twr_1, sizeof(g_twr_1)/sizeof(twr_frame_t));
    uwb_mac_append_interface(udev, &g_cbs[1]);
#endif
#if MYNEWT_VAL(UWB_DEVICE_2)
    udev = uwb_dev_idx_lookup(2);
    g_cbs[2].inst_ptr = rng = uwb_rng_init(udev, &g_config, sizeof(g_twr_2)/sizeof(twr_frame_t));
    uwb_rng_set_frames(rng, g_twr_2, sizeof(g_twr_2)/sizeof(twr_frame_t));
    uwb_mac_append_interface(udev, &g_cbs[2]);
#endif

}

/**
 * @fn uwb_rng_set_frames(struct uwb_rng_instance * inst, twr_frame_t twr[], uint16_t nframes)
 * @brief API to set the pointer to the twr buffers.
 *
 * @param inst      Pointer to struct uwb_rng_instance.
 * @param twr[]     Pointer to twr buffers.
 * @param nframes   Number of buffers defined to store the ranging data.
 *
 * @return void
 */
inline void
uwb_rng_set_frames(struct uwb_rng_instance * rng, twr_frame_t twr[], uint16_t nframes){
    assert(nframes <= rng->nframes);
    for (uint16_t i = 0; i < nframes; i++)
        rng->frames[i] = &twr[i];
}

/**
 * @fn uwb_rng_config(struct uwb_rng_instance * inst, struct uwb_rng_config * config)
 * @brief API to assign the config parameters to range instance.
 *
 * @param inst    Pointer to struct uwb_rng_instance.
 * @param config  Pointer to struct uwb_rng_config.
 *
 * @return struct uwb_dev_status
 */
struct uwb_dev_status
uwb_rng_config(struct uwb_rng_instance * rng, struct uwb_rng_config * config){
    assert(config);

    memcpy(&rng->config, config, sizeof(struct uwb_rng_config));
    return rng->dev_inst->status;
}

/**
 * @fn uwb_rng_get_config(struct uwb_rng_instance * inst, uwb_rng_modes_t code)
 * @brief API to get configuration using uwb_rng_modes_t.
 *
 * @param inst          Pointer to struct uwb_rng_instance.
 * @param code          Represents mode of ranging DWT_SS_TWR enables single sided two way ranging DWT_DS_TWR enables double sided
 * two way ranging DWT_DS_TWR_EXT enables double sided two way ranging with extended frame.
 *
 * @return struct uwb_rng_config
 */
struct uwb_rng_config *
uwb_rng_get_config(struct uwb_rng_instance * rng, uwb_rng_modes_t code)
{
    struct rng_config_list * cfgs;

    if(!(SLIST_EMPTY(&rng->rng_configs))){ 
        SLIST_FOREACH(cfgs, &rng->rng_configs, next){    
            if (cfgs != NULL && cfgs->rng_code == code) {
                return cfgs->config;
            }
        }
    }
    return &g_config;
}

/**
 * Add config extension different rng services.
 *
 * @param rng        Pointer to struct uwb_rng_instance.
 * @param callbacks  callback instance.
 * @return void
 */
void
uwb_rng_append_config(struct uwb_rng_instance * rng, struct rng_config_list *cfgs)
{
    assert(rng);

    if(!(SLIST_EMPTY(&rng->rng_configs))) {
        struct rng_config_list * prev_cfgs = NULL;
        struct rng_config_list * cur_cfgs = NULL;
        SLIST_FOREACH(cur_cfgs, &rng->rng_configs, next){
            prev_cfgs = cur_cfgs;
        }
        SLIST_INSERT_AFTER(prev_cfgs, cfgs, next);
    } else {
        SLIST_INSERT_HEAD(&rng->rng_configs, cfgs, next);
    }
}


/**
 * API to remove specified callbacks.
 *
 * @param inst  Pointer to struct uwb_rng_instance.
 * @param id    ID of the service.
 * @return void
 */
void
uwb_rng_remove_config(struct uwb_rng_instance * rng, uwb_rng_modes_t code)
{
    assert(rng);
    struct rng_config_list * cfgs = NULL;
    SLIST_FOREACH(cfgs, &rng->rng_configs, next){
        if(cfgs->rng_code == code){
            SLIST_REMOVE(&rng->rng_configs, cfgs, rng_config_list, next);
            break;
        }
    }
}


/**
 * @fn uwb_rng_request(struct uwb_rng_instance * inst, uint16_t dst_address, uwb_rng_modes_t code)
 * @brief API to initialise range request.
 *
 * @param inst          Pointer to struct uwb_rng_instance.
 * @param dst_address   Address of the receiver to whom range request to be sent.
 * @param code          Represents mode of ranging DWT_SS_TWR enables single sided two way ranging DWT_DS_TWR enables double sided
 * two way ranging DWT_DS_TWR_EXT enables double sided two way ranging with extended frame.
 *
 * @return struct uwb_dev_status
 */
struct uwb_dev_status
uwb_rng_request(struct uwb_rng_instance * rng, uint16_t dst_address, uwb_rng_modes_t code)
{
    // This function executes on the device that initiates a request 
    RNG_STATS_INC(rng_request);
    dpl_error_t err = dpl_sem_pend(&rng->sem,  DPL_TIMEOUT_NEVER);
    assert(err == DPL_OK);

    struct uwb_dev * inst = rng->dev_inst;
    struct uwb_rng_config * config = uwb_rng_get_config(rng, code);

    twr_frame_t * frame  = rng->frames[(rng->idx+1)%rng->nframes];

    if (code == DWT_SS_TWR || code == DWT_SS_TWR_EXT)
        rng->seq_num+=1;
    else
        rng->seq_num+=2;

    frame->seq_num = rng->seq_num;
    frame->code = code;
    frame->src_address = inst->my_short_address;
    frame->dst_address = dst_address;

    // Download the CIR on the response
#if MYNEWT_VAL(CIR_ENABLED)
    cir_enable(inst->cir, true);
#endif

    uwb_write_tx(inst, frame->array, 0, sizeof(ieee_rng_request_frame_t));
    uwb_write_tx_fctrl(inst, sizeof(ieee_rng_request_frame_t), 0);
    uwb_set_wait4resp(inst, true); 
    
    uint16_t frame_duration = uwb_phy_frame_duration(inst, sizeof(ieee_rng_response_frame_t));
    uint16_t shr_duration  = uwb_phy_SHR_duration(inst);
    uint16_t data_duration = frame_duration - shr_duration;
 
    // The wait for response counter starts on the completion of the entire outgoing frame. 
    // To relate the delay to the RMARKER we remove the data-duration of the outbound frame
    // and start the receiver in time for the preamble by subtracting the SHR duration.

    uwb_set_wait4resp_delay(inst, config->tx_holdoff_delay - data_duration - shr_duration);

    // The timeout counter starts when the receiver in re-enabled. The timeout event 
    // should occuring just after the inbound frame is received
    if (code == DWT_SS_TWR_EXT) {
        uint16_t frame_duration = uwb_phy_frame_duration(inst,sizeof(twr_frame_t));
        uwb_set_rx_timeout(inst, frame_duration + config->rx_timeout_delay);
    } else {
        uwb_set_rx_timeout(inst, frame_duration + config->rx_timeout_delay);
    }
    uwb_set_rxauto_disable(inst, true);

    if (rng->control.delay_start_enabled)
        uwb_set_delay_start(inst, rng->delay);

    if (uwb_start_tx(inst).start_tx_error && inst->status.rx_timeout_error == 0){
        dpl_sem_release(&rng->sem);
        RNG_STATS_INC(tx_error);
    }

    err = dpl_sem_pend(&rng->sem, DPL_TIMEOUT_NEVER); // Wait for completion of transactions
    assert(err == DPL_OK);
    err = dpl_sem_release(&rng->sem);
    assert(err == DPL_OK);

   return inst->status;
}

/**
 * @fn uwb_rng_listen(struct uwb_rng_instance * inst, uwb_dev_modes_t mode)
 * @brief API to listen rng request.
 *
 * @param inst          Pointer to struct uwb_rng_instance.
 * @param mode          uwb_dev_modes_t of UWB_BLOCKING and UWB_NONBLOCKING
 *
 * @return struct uwb_dev_status
 */
struct uwb_dev_status
uwb_rng_listen(struct uwb_rng_instance * rng, uwb_dev_modes_t mode)
{
    dpl_error_t err = dpl_sem_pend(&rng->sem,  DPL_TIMEOUT_NEVER);
    assert(err == DPL_OK);

    // Download the CIR on the response    
#if MYNEWT_VAL(CIR_ENABLED)   
    cir_enable(rng->dev_inst->cir, true);
#endif 
    uwb_set_rxauto_disable(rng->dev_inst, true);

    RNG_STATS_INC(rng_listen);
    if(uwb_start_rx(rng->dev_inst).start_rx_error){
        err = dpl_sem_release(&rng->sem);
        assert(err == DPL_OK);
        RNG_STATS_INC(rx_error);
    }

    if (mode == UWB_BLOCKING){
        err = dpl_sem_pend(&rng->sem, DPL_TIMEOUT_NEVER); // Wait for completion of transactions
        assert(err == DPL_OK);
        err = dpl_sem_release(&rng->sem);
        assert(err == DPL_OK);
    }

   return rng->dev_inst->status;
}

/**
 * @fn uwb_rng_request_delay_start(struct uwb_rng_instance * inst, uint16_t dst_address, uint64_t delay, uwb_rng_modes_t code)
 * @brief API to configure dw1000 to start transmission after certain delay.
 *
 * @param inst          Pointer to struct uwb_rng_instance.
 * @param dst_address   Address of the receiver to whom range request to be sent.
 * @param delay         Time until which request has to be resumed.
 * @param code          Represents mode of ranging DWT_SS_TWR enables single sided two way ranging DWT_DS_TWR enables double sided
 * two way ranging DWT_DS_TWR_EXT enables double sided two way ranging with extended frame.
 *
 * @return struct uwb_dev_status
 */
struct uwb_dev_status
uwb_rng_request_delay_start(struct uwb_rng_instance * rng, uint16_t dst_address, uint64_t delay, uwb_rng_modes_t code)
{
    rng->control.delay_start_enabled = 1;
    rng->delay = delay;
    uwb_rng_request(rng, dst_address, code);
    rng->control.delay_start_enabled = 0;

   return rng->dev_inst->status;
}

/**
 * @fn uwb_rng_path_loss(float Pt, float G, float fc, float R)
 * @brief calculate rng path loss using range parameters and return signal level.
 * and allocated PANIDs and SLOTIDs.
 *
 * @param Pt      Transmit power in dBm.
 * @param G       Antenna Gain in dB.
 * @param Fc      Centre frequency in Hz.
 * @param R       Range in meters.
 *
 * @return Pr received signal level dBm
 */
float
uwb_rng_path_loss(float Pt, float G, float fc, float R){
    float Pr = Pt + 2 * G + 20 * log10(299792458.0l/1.000293l) - 20 * log10(4 * M_PI * fc * R);
    return Pr;
}

/**
 * @fn uwb_rng_bias_correction(struct uwb_dev * inst, float Pr)
 * @brief API for bias correction polynomial.
 *
 * @param inst   Pointer to struct uwb_dev.
 * @param pr     Variable that calculates range path loss.
 *
 * @return Bias value
 */
float
uwb_rng_bias_correction(struct uwb_dev * dev, float Pr){
    float bias;
    switch(dev->config.prf){
        case DWT_PRF_16M:
            bias = polyval(rng_bias_poly_PRF16, Pr, sizeof(rng_bias_poly_PRF16)/sizeof(float));
            break;
        case DWT_PRF_64M:
            bias = polyval(rng_bias_poly_PRF64, Pr, sizeof(rng_bias_poly_PRF64)/sizeof(float));
            break;
        default:
            assert(0);
    }
    return bias;
}


/**
 * @fn uwb_rng_twr_to_tof(struct uwb_rng_instance * rng, uint16_t idx)
 * @brief API to calculate time of flight based on type of ranging.
 *
 * @param rng  Pointer to struct uwb_rng_instance.
 * @param idx  Position of rng frame
 *
 * @return Time of flight in float
 */
float
uwb_rng_twr_to_tof(struct uwb_rng_instance * rng, uint16_t idx)
{
    float ToF = 0;
    uint64_t T1R, T1r, T2R, T2r;
    int64_t nom,denom;

    struct uwb_dev * inst = rng->dev_inst;

    twr_frame_t * first_frame = rng->frames[(uint16_t)(idx-1)%rng->nframes];
    twr_frame_t * frame = rng->frames[(idx)%rng->nframes];

    switch(frame->code){
        case DWT_SS_TWR ... DWT_SS_TWR_END:
        case DWT_SS_TWR_EXT ... DWT_SS_TWR_EXT_END:{
#if MYNEWT_VAL(UWB_WCS_ENABLED)
            struct uwb_ccp_instance *ccp = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_CCP);
            struct uwb_wcs_instance * wcs = ccp->wcs;
            float skew = wcs->skew;
#else
            float skew = uwb_calc_clock_offset_ratio(inst, first_frame->carrier_integrator,
                                                     UWB_CR_CARRIER_INTEGRATOR);
#endif
            ToF = ((frame->response_timestamp - frame->request_timestamp)
                    -  (frame->transmission_timestamp - frame->reception_timestamp) * (1.0f - skew))/2.;
            }
            break;
        case DWT_DS_TWR ... DWT_DS_TWR_END:
        case DWT_DS_TWR_EXT ... DWT_DS_TWR_EXT_END:
            T1R = (first_frame->response_timestamp - first_frame->request_timestamp);
            T1r = (first_frame->transmission_timestamp  - first_frame->reception_timestamp);
            T2R = (frame->response_timestamp - frame->request_timestamp);
            T2r = (frame->transmission_timestamp - frame->reception_timestamp);
            nom = T1R * T2R  - T1r * T2r;
            denom = T1R + T2R  + T1r + T2r;
            ToF = (float) (nom) / denom;
            break;
        default: break;
    }
    return ToF;
}

/**
 * @fn uwb_rng_tof_to_meters(float ToF)
 * @brief API to calculate range in meters from time-of-flight based on type of ranging.
 *
 * @param ToF Time of flight in float.
 *
 * @return range in meters
 */
float
uwb_rng_tof_to_meters(float ToF) {
    return (float)(ToF * (299792458.0l/1.000293l) * (1.0/499.2e6/128.0)); //!< Converts time of flight to meters.
}

/**
 * @fn uwb_rng_twr_to_tof_sym(twr_frame_t twr[], uwb_rng_modes_t code)
 * @brief API to calculate time of flight for symmetric type of ranging.
 *
 * @param twr[]  Pointer to twr buffers.
 * @param code   Represents mode of ranging DWT_SS_TWR enables single sided two way ranging DWT_DS_TWR enables double sided
 * two way ranging DWT_DS_TWR_EXT enables double sided two way ranging with extended frame.
 *
 * @return Time of flight
 */
uint32_t
uwb_rng_twr_to_tof_sym(twr_frame_t twr[], uwb_rng_modes_t code){
    uint32_t ToF = 0;
    uint64_t T1R, T1r, T2R, T2r;

    switch(code){
        case DWT_SS_TWR:
            ToF = ((twr[0].response_timestamp - twr[0].request_timestamp)
                    -  (twr[0].transmission_timestamp - twr[0].reception_timestamp))/2.;
        break;
        case DWT_DS_TWR:
            T1R = (twr[0].response_timestamp - twr[0].request_timestamp);
            T1r = (twr[0].transmission_timestamp  - twr[0].reception_timestamp);
            T2R = (twr[1].response_timestamp - twr[1].request_timestamp);
            T2r = (twr[1].transmission_timestamp  - twr[1].reception_timestamp);
            ToF = (T1R - T1r + T2R - T2r) >> 2;
        break;
        default: break;
    }
    return ToF;
}

/**
 * @fn rx_timeout_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
 * @brief API for receive timeout callback.
 *
 * @param inst  Pointer to struct uwb_dev.
 * @param cbs   Pointer to struct uwb_mac_interface.
 *
 * @return true on sucess
 */
static bool 
rx_timeout_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
{
    struct uwb_rng_instance * rng = (struct uwb_rng_instance *)cbs->inst_ptr;
    if(dpl_sem_get_count(&rng->sem) == 1)
        return false;

    if(dpl_sem_get_count(&rng->sem) == 0){
        dpl_error_t err = dpl_sem_release(&rng->sem);
        assert(err == DPL_OK);
        RNG_STATS_INC(rx_timeout);
        switch(rng->code){
            case DWT_SS_TWR ... DWT_DS_TWR_EXT_FINAL:
                {
                    RNG_STATS_INC(rx_timeout);
                    return true;
                }
                break;    
            default:
                return false;
        }
    }
    return false;
}

/**
 * @fn reset_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
 * @brief API for reset_cb of rng interface
 *
 * @param inst   Pointer to struct uwb_dev.
 * @param cbs    Pointer to struct uwb_mac_interface.
 *
 * @return true on sucess
 */
static bool
reset_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
{
    struct uwb_rng_instance * rng = (struct uwb_rng_instance *)cbs->inst_ptr;
    if(dpl_sem_get_count(&rng->sem) == 0){
        dpl_error_t err = dpl_sem_release(&rng->sem);
        assert(err == DPL_OK);
        RNG_STATS_INC(reset);
        return true;
    }
    else
        return false;
}

/**
 * @fn rx_complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
 * @brief API for receive complete callback.
 *
 * @param inst  Pointer to struct uwb_dev.
 * @param cbs   Pointer to struct uwb_mac_interface.
 *
 * @return true on sucess
 */
static bool
rx_complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
{
    if (inst->fctrl != FCNTL_IEEE_RANGE_16)
        return false;

    struct uwb_rng_instance * rng = (struct uwb_rng_instance *)cbs->inst_ptr;
    if(dpl_sem_get_count(&rng->sem) == 1){
        // unsolicited inbound
        RNG_STATS_INC(rx_unsolicited);
        return false;
    }

    if (inst->frame_len < sizeof(ieee_rng_request_frame_t))
       return false;

    rng->code = ((ieee_rng_request_frame_t * ) inst->rxbuf)->code;
    switch(rng->code) {
        case DWT_SS_TWR ... DWT_DS_TWR_EXT_END:
            {
                twr_frame_t * frame = rng->frames[(rng->idx+1)%rng->nframes]; // speculative frame advance
                if (inst->frame_len <= sizeof(frame->array))
                    memcpy(frame->array, inst->rxbuf, inst->frame_len);
                else
                    break;
                // IEEE 802.15.4 standard ranging frames, software MAC filtering
                if (inst->config.framefilter_enabled == false && frame->dst_address != inst->my_short_address){
                    return true;
                }else{
                    RNG_STATS_INC(rx_complete); 
                    rng->idx++;     // confirmed frame advance  
                    return false;   // Allow sub extensions to handle event
                }
            }
            break;
        default:
            return false;
    }
    return false;
}

/**
 * @fn tx_complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
 * @brief API for transmission complete callback.
 *
 * @param inst  Pointer to struct uwb_dev.
 * @param cbs   Pointer to struct uwb_mac_interface.
 *
 * @return true on sucess
 */
static bool
tx_complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
{
    if (inst->fctrl != FCNTL_IEEE_RANGE_16)
        return false;
    
    struct uwb_rng_instance * rng = (struct uwb_rng_instance *)cbs->inst_ptr;
    if(dpl_sem_get_count(&rng->sem) == 1) {
        // unsolicited inbound
        return false;
    }

    switch(rng->code) {
        case DWT_SS_TWR ... DWT_DS_TWR_EXT_END:
            RNG_STATS_INC(tx_complete);
            return true;
            break;
        default:
            return false;
    }
}

#if MYNEWT_VAL(RNG_VERBOSE)

/**
 * @fn complete_ev_cb(struct os_event *ev)
 * @brief API for rng complete event callback and print rng logs into json format.
 *
 * @param ev    Pointer to os_event.
 *
 * @return true on sucess
 */
static void
complete_ev_cb(struct dpl_event *ev) {
    assert(ev != NULL);
    assert(dpl_event_get_arg(ev));

    struct uwb_rng_instance * rng = (struct uwb_rng_instance *)dpl_event_get_arg(ev);
    rng_encode(rng);
}

static struct dpl_event rng_event;

/**
 * @fn complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
 * @brief API for rng complete callback and put complete_event_cb in queue.
 *
 * @param inst   Pointer to struct uwb_dev.
 * @param cbs    Pointer to struct uwb_mac_interface.
 *
 * @return true on sucess
 */
static bool
complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
{
    struct uwb_rng_instance * rng = (struct uwb_rng_instance *)cbs->inst_ptr;
    if (inst->fctrl != FCNTL_IEEE_RANGE_16)
        return false;

    rng->idx_current = (rng->idx)%rng->nframes;
    if (!dpl_event_get_arg(&rng_event)) {
        dpl_event_init(&rng_event, complete_ev_cb, (void*) rng);
    }
    dpl_eventq_put(dpl_eventq_dflt_get(), &rng_event);
    return false;
}
#endif
