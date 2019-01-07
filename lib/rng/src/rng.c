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
#include "bsp/bsp.h"
#include <stats/stats.h>

#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_ftypes.h>
#include <dw1000/dw1000_stats.h>
#include <dsp/polyval.h>

#if MYNEWT_VAL(RNG_ENABLED)
#include <rng/rng.h>
#include <rng/rng_encode.h>
#include <rng/nrng.h>
#endif
#if MYNEWT_VAL(TWR_DS_EXT_ENABLED)
#include <twr_ds_ext/twr_ds_ext.h>
#endif
#if MYNEWT_VAL(TWR_DS_ENABLED)
#include <twr_ds/twr_ds.h>
#endif
#if MYNEWT_VAL(TWR_SS_ENABLED)
#include <twr_ss/twr_ss.h>
#endif
#if MYNEWT_VAL(WCS_ENABLED)
#include <wcs/wcs.h>
#endif
#if MYNEWT_VAL(CIR_ENABLED)
#include <cir/cir.h>
#endif

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

//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

static bool rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
#if MYNEWT_VAL(RNG_VERBOSE)
static bool complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
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

static dw1000_rng_config_t g_config = {
    .tx_holdoff_delay = MYNEWT_VAL(RNG_TX_HOLDOFF),       // Send Time delay in usec.
    .rx_timeout_delay = MYNEWT_VAL(RNG_RX_TIMEOUT)       // Receive response timeout in usec
};

#if MYNEWT_VAL(DW1000_DEVICE_0)
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
#if MYNEWT_VAL(DW1000_DEVICE_1)
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
#if MYNEWT_VAL(DW1000_DEVICE_2)
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
    }
};
#endif

static dw1000_mac_interface_t g_cbs[] = {
        [0] = {
            .id = DW1000_RNG,
            .rx_complete_cb = rx_complete_cb,
            .tx_complete_cb = tx_complete_cb,
            .rx_timeout_cb = rx_timeout_cb,
#if MYNEWT_VAL(RNG_VERBOSE)
            .complete_cb  = complete_cb,
#endif
            .reset_cb = reset_cb
        },
#if MYNEWT_VAL(DW1000_DEVICE_1)
        [1] = {
            .id = DW1000_RNG,
            .rx_complete_cb = rx_complete_cb,
            .tx_complete_cb = tx_complete_cb,
            .rx_timeout_cb = rx_timeout_cb,
#if MYNEWT_VAL(RNG_VERBOSE)
            .complete_cb  = complete_cb,
#endif
            .reset_cb = reset_cb
        },
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
        [2] = {
            .id = DW1000_RNG,
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
 * API to initialise the ranging by setting all the required configurations and callbacks.
 *
 * @param inst      Pointer to dw1000_dev_instance_t. 
 * @param config    Pointer to the structure dw1000_rng_config_t.
 * @param nframes   Number of buffers defined to store the ranging data. 
 *
 * @return dw1000_rng_instance_t
 */
dw1000_rng_instance_t * 
dw1000_rng_init(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config, uint16_t nframes){

    assert(inst);
    if (inst->rng == NULL ) {
        inst->rng = (dw1000_rng_instance_t *) malloc(sizeof(dw1000_rng_instance_t) + nframes * sizeof(twr_frame_t *)); // struct + flexible array member
        assert(inst->rng);
        memset(inst->rng, 0, sizeof(dw1000_rng_instance_t));
        inst->rng->status.selfmalloc = 1;
        inst->rng->nframes = nframes;
    }
    inst->rng->parent = inst;
    os_error_t err = os_sem_init(&inst->rng->sem, 0x1); 
    assert(err == OS_OK);

    if (config != NULL ){
        dw1000_rng_config(inst, config);
    }

    inst->rng->control = (dw1000_rng_control_t){
        .delay_start_enabled = 0,
    };
    inst->rng->idx = 0xFFFF;
    inst->rng->status.initialized = 1;
    
    int rc = stats_init(
                    STATS_HDR(inst->rng->stat),
                    STATS_SIZE_INIT_PARMS(inst->rng->stat, STATS_SIZE_32),
                    STATS_NAME_INIT_PARMS(rng_stat_section)
            );
   
#if  MYNEWT_VAL(DW1000_DEVICE_0) && !MYNEWT_VAL(DW1000_DEVICE_1)
        rc |= stats_register("rng", STATS_HDR(inst->rng->stat));
#elif  MYNEWT_VAL(DW1000_DEVICE_0) && MYNEWT_VAL(DW1000_DEVICE_1)
    if (inst == hal_dw1000_inst(0))
        rc |= stats_register("rng0", STATS_HDR(inst->rng->stat));
    else
        rc |= stats_register("rng1", STATS_HDR(inst->rng->stat));
#endif
    assert(rc == 0);
    return inst->rng;
}


/**
 * API to free the allocated resources.
 *
 * @param inst  Pointer to dw1000_rng_instance_t.
 *
 * @return void 
 */
void 
dw1000_rng_free(dw1000_rng_instance_t * inst){
   
    assert(inst);  
    if (inst->status.selfmalloc)
        free(inst);
    else
        inst->status.initialized = 0;
}

/**
 * API to initialise the rng package.
 *
 *
 * @return void
 */

void 
rng_pkg_init(void){

    printf("{\"utime\": %lu,\"msg\": \"rng_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

#if MYNEWT_VAL(DW1000_DEVICE_0)
    dw1000_rng_init(hal_dw1000_inst(0), &g_config, sizeof(g_twr_0)/sizeof(twr_frame_t));
    dw1000_rng_set_frames(hal_dw1000_inst(0), g_twr_0, sizeof(g_twr_0)/sizeof(twr_frame_t));
    dw1000_mac_append_interface(hal_dw1000_inst(0), &g_cbs[0]);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    dw1000_rng_init(hal_dw1000_inst(1), &g_config, sizeof(g_twr_1)/sizeof(twr_frame_t));
    dw1000_rng_set_frames(hal_dw1000_inst(1), g_twr_1, sizeof(g_twr_1)/sizeof(twr_frame_t));
    dw1000_mac_append_interface(hal_dw1000_inst(1), &g_cbs[1]);

#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    dw1000_rng_init(hal_dw1000_inst(2), &g_config, sizeof(g_twr_2)/sizeof(twr_frame_t));
    dw1000_rng_set_frames(hal_dw1000_inst(2), g_twr_2, sizeof(g_twr_2)/sizeof(twr_frame_t));
    dw1000_mac_append_interface(hal_dw1000_inst(2), &g_cbs[2]);
#endif
  
}

/**
 * API to set the pointer to the twr buffers.
 *
 * @param inst      Pointer to dw1000_dev_instance_t.
 * @param twr[]     Pointer to twr buffers.
 * @param nframes   Number of buffers defined to store the ranging data.
 *
 * @return void
 */
inline void 
dw1000_rng_set_frames(dw1000_dev_instance_t * inst, twr_frame_t twr[], uint16_t nframes){
        assert(nframes <= inst->rng->nframes);
        for (uint16_t i = 0; i < nframes; i++)
            inst->rng->frames[i] = &twr[i];
}

/**
 * API to assign the config parameters to range instance.
 *
 * @param inst    Pointer to dw1000_dev_instance_t. 
 * @param config  Pointer to dw1000_rng_config_t.
 *
 * @return dw1000_dev_status_t 
 */
dw1000_dev_status_t 
dw1000_rng_config(dw1000_dev_instance_t * inst, dw1000_rng_config_t * config){
    assert(inst);
    assert(config);

    memcpy(&inst->rng->config, config, sizeof(dw1000_rng_config_t));
    return inst->status;
}

/**
 * API to configure dw1000 to start transmission after certain delay.
 *
 * @param inst          Pointer to dw1000_dev_instance_t. 
 * @param dst_address   Address of the receiver to whom range request to be sent. 
 * @param delay         Time until which request has to be resumed. 
 * @param code          Represents mode of ranging DWT_SS_TWR enables single sided two way ranging DWT_DS_TWR enables double sided 
 * two way ranging DWT_DS_TWR_EXT enables double sided two way ranging with extended frame.
 *
 * @return dw1000_dev_status_t
 */
dw1000_rng_config_t *
dw1000_rng_get_config(dw1000_dev_instance_t * inst, dw1000_rng_modes_t code){

    dw1000_rng_config_t * config;

    switch (code){
#if MYNEWT_VAL(TWR_SS_ENABLED) 
        case  DWT_SS_TWR:                     //!< Single sided TWR 
            config = twr_ss_config(inst);
            break;  
#endif
#if MYNEWT_VAL(TWR_DS_ENABLED) 
        case  DWT_DS_TWR:                     //!< Double sided TWR 
            config = twr_ds_config(inst);
            break;  
#endif
#if MYNEWT_VAL(TWR_DS_EXT_ENABLED) 
        case DWT_DS_TWR_EXT:                  //!< Double sided TWR in extended mode 
            config = twr_ds_ext_config(inst);
            break;  
#endif
        default:
            config = &g_config;
    }
    return config;
}

/**
 * API to initialise range request.
 *
 * @param inst          Pointer to dw1000_dev_instance_t.
 * @param dst_address   Address of the receiver to whom range request to be sent.
 * @param code          Represents mode of ranging DWT_SS_TWR enables single sided two way ranging DWT_DS_TWR enables double sided 
 * two way ranging DWT_DS_TWR_EXT enables double sided two way ranging with extended frame. 
 *
 * @return dw1000_dev_status_t 
 */
dw1000_dev_status_t 
dw1000_rng_request(dw1000_dev_instance_t * inst, uint16_t dst_address, dw1000_rng_modes_t code){

    // This function executes on the device that initiates a request 
    STATS_INC(inst->rng->stat, rng_request);
    os_error_t err = os_sem_pend(&inst->rng->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    dw1000_rng_config_t * config = dw1000_rng_get_config(inst, code);
    
    dw1000_rng_instance_t * rng = inst->rng;                            
    twr_frame_t * frame  = inst->rng->frames[(rng->idx+1)%rng->nframes];    

    if (code == DWT_SS_TWR)
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

    dw1000_write_tx(inst, frame->array, 0, sizeof(ieee_rng_request_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_request_frame_t), 0, true); 
    dw1000_set_wait4resp(inst, true);    
   // dw1000_set_wait4resp_delay(inst, config->tx_holdoff_delay - dw1000_phy_SHR_duration(&inst->attrib));
    uint16_t timeout = dw1000_phy_frame_duration(&inst->attrib, sizeof(ieee_rng_response_frame_t)) 
                    + config->rx_timeout_delay // At least 2 * ToF, 1us ~= 300m
                    + config->tx_holdoff_delay;

    dw1000_set_rx_timeout(inst, timeout); 
   
    if (rng->control.delay_start_enabled) 
        dw1000_set_delay_start(inst, rng->delay);

    if (dw1000_start_tx(inst).start_tx_error && inst->status.rx_timeout_error == 0){
        os_sem_release(&inst->rng->sem);
        STATS_INC(inst->rng->stat, tx_error);
    }
     
    err = os_sem_pend(&inst->rng->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions 
    assert(err == OS_OK);
    err = os_sem_release(&inst->rng->sem);
    assert(err == OS_OK);
    
   return inst->status;
}

/**
 * API to initialise range request.
 *
 * @param inst          Pointer to dw1000_dev_instance_t.
 *
 * @return dw1000_dev_status_t 
 */
dw1000_dev_status_t 
dw1000_rng_listen(dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode){

    os_error_t err = os_sem_pend(&inst->rng->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    // Set fcntl in the event of a timeout
    inst->fctrl = FCNTL_IEEE_RANGE_16;

    // Download the CIR on the response    
#if MYNEWT_VAL(CIR_ENABLED)   
    cir_enable(inst->cir, true);
#endif 
    
    STATS_INC(inst->rng->stat, rng_listen);
    if(dw1000_start_rx(inst).start_rx_error){
        err = os_sem_release(&inst->rng->sem);
        assert(err == OS_OK);
        STATS_INC(inst->rng->stat, rx_error);
    }
      
    if (mode == DWT_BLOCKING){
        err = os_sem_pend(&inst->rng->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions 
        assert(err == OS_OK);
        err = os_sem_release(&inst->rng->sem);
        assert(err == OS_OK);
    }
    
   return inst->status;
}

/**
 * API to configure dw1000 to start transmission after certain delay.
 *
 * @param inst          Pointer to dw1000_dev_instance_t. 
 * @param dst_address   Address of the receiver to whom range request to be sent. 
 * @param delay         Time until which request has to be resumed. 
 * @param code          Represents mode of ranging DWT_SS_TWR enables single sided two way ranging DWT_DS_TWR enables double sided 
 * two way ranging DWT_DS_TWR_EXT enables double sided two way ranging with extended frame.
 *
 * @return dw1000_dev_status_t
 */
dw1000_dev_status_t 
dw1000_rng_request_delay_start(dw1000_dev_instance_t * inst, uint16_t dst_address, uint64_t delay, dw1000_rng_modes_t code){
    
    dw1000_rng_instance_t * rng = inst->rng;    
  
    rng->control.delay_start_enabled = 1;
    rng->delay = delay;
    dw1000_rng_request(inst, dst_address, code);
    rng->control.delay_start_enabled = 0;
    
   return inst->status;
}



/**
 * This is a template which should be replaced by the pan_master by a event that tracks UUIDs 
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
dw1000_rng_path_loss(float Pt, float G, float fc, float R){
    float Pr = Pt + 2 * G + 20 * log10(299792458.0l/1.000293l) - 20 * log10(4 * M_PI * fc * R);
    return Pr;
}

/**
 * API for bias correction polynomial.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @param pr     Variable that calculates range path loss.
 *
 * @return Bias value
 */
float 
dw1000_rng_bias_correction(dw1000_dev_instance_t * inst, float Pr){
    float bias;
    switch(inst->config.prf){
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

#if MYNEWT_VAL(DW1000_RANGE)

/**
 * API to calculate time of flight based on type of ranging.
 *
 * @param fframe   Pointer to the first twr frame. 
 * @param nframe   Poinetr to the second twr frame.
 *
 * @return Time of flight in float
 */
float
dw1000_rng_twr_to_tof(twr_frame_t *fframe, twr_frame_t *nframe){
    float ToF = 0;
    uint64_t T1R, T1r, T2R, T2r;
    int64_t nom,denom;

    assert(fframe != NULL);
    assert(nframe != NULL);

    twr_frame_t * first_frame = fframe;
    twr_frame_t * frame = nframe;

    switch(frame->code){
        case DWT_SS_TWR ... DWT_SS_TWR_END:
            ToF = ((first_frame->response_timestamp - first_frame->request_timestamp)
                    -  (first_frame->transmission_timestamp - first_frame->reception_timestamp))/2.;
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
#else

/**
 * API to calculate time of flight based on type of ranging.
 *
 * @param rng  Pointer to dw1000_rng_instance_t.
 *
 * @return Time of flight in float
 */
float 
dw1000_rng_twr_to_tof(dw1000_rng_instance_t * rng, uint16_t idx){

    float ToF = 0;
    uint64_t T1R, T1r, T2R, T2r;
    int64_t nom,denom;

    dw1000_dev_instance_t * inst = rng->parent;

    twr_frame_t * first_frame = rng->frames[(uint16_t)(idx-1)%rng->nframes];
    twr_frame_t * frame = rng->frames[(idx)%rng->nframes];

    switch(frame->code){
        case DWT_SS_TWR ... DWT_SS_TWR_END:{
#if MYNEWT_VAL(WCS_ENABLED)
            wcs_instance_t * wcs = inst->ccp->wcs;
            float skew = wcs->skew;
#else
            float skew = dw1000_calc_clock_offset_ratio(inst, first_frame->carrier_integrator);
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
#endif


/**
 * API to calculate range in meters from time-of-flight based on type of ranging.
 *
 * @param rng  Pointer to dw1000_rng_instance_t.
 *
 * @return range in meters
 */
float 
dw1000_rng_tof_to_meters(float ToF) {
    return (float)(ToF * (299792458.0l/1.000293l) * (1.0/499.2e6/128.0)); //!< Converts time of flight to meters.
}

/**
 * API to calculate time of flight for symmetric type of ranging.
 *
 * @param twr[]  Pointer to twr buffers. 
 * @param code   Represents mode of ranging DWT_SS_TWR enables single sided two way ranging DWT_DS_TWR enables double sided 
 * two way ranging DWT_DS_TWR_EXT enables double sided two way ranging with extended frame.
 *
 * @return Time of flight
 */
uint32_t 
dw1000_rng_twr_to_tof_sym(twr_frame_t twr[], dw1000_rng_modes_t code){
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
 * API for receive timeout callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */

static bool 
rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){

    if(inst->fctrl != FCNTL_IEEE_RANGE_16){
        return false;
    }

    dw1000_rng_instance_t * rng = inst->rng;
    if(os_sem_get_count(&rng->sem) == 0){
        os_error_t err = os_sem_release(&rng->sem);
        assert(err == OS_OK);
        STATS_INC(inst->rng->stat, rx_timeout);
        switch(rng->code){
            case DWT_SS_TWR ... DWT_DS_TWR_EXT_FINAL:
                {
                    STATS_INC(inst->rng->stat, rx_timeout);
                    return true;
                }
                break;    
                   
            case DWT_SS_TWR_NRNG ... DWT_SS_TWR_NRNG_FINAL:
                {
                    // In the case of a NRNG timeout is used to mark the end of the request and is used to call the completion callback  
                    STATS_INC(inst->rng->stat, rx_complete);
                    if(!(SLIST_EMPTY(&inst->interface_cbs))){
                        SLIST_FOREACH(cbs, &inst->interface_cbs, next){
                            if (cbs!=NULL && cbs->complete_cb)
                                if(cbs->complete_cb(inst, cbs)) continue;
                        }
                    }
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
 * API for reset_cb of rng interface
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 * @return true on sucess
 */
static bool
reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){

    if(os_sem_get_count(&inst->rng->sem) == 0){
        os_error_t err = os_sem_release(&inst->rng->sem);  
        assert(err == OS_OK);
        STATS_INC(inst->rng->stat, reset);
        return true;
    }
    else 
        return false;

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
    if (inst->fctrl != FCNTL_IEEE_RANGE_16)
        return false;
  
    if(os_sem_get_count(&inst->rng->sem) == 1){ 
        // unsolicited inbound
        STATS_INC(inst->rng->stat, rx_unsolicited);
        return false;
    }

    dw1000_rng_instance_t * rng = inst->rng; 
#if MYNEWT_VAL(NRNG_ENABLED)
    dw1000_nrng_instance_t * nrng = inst->nrng; 
#endif

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
                    STATS_INC(rng->stat, rx_complete); 
                    rng->idx++;     // confirmed frame advance  
                    return false;   // Allow sub extensions to handle event
                }
            }
            break;
#if MYNEWT_VAL(NRNG_ENABLED)
        case DWT_SS_TWR_NRNG ... DWT_DS_TWR_NRNG_EXT_END:
            {   
                nrng_frame_t * frame = (nrng_frame_t *) inst->rxbuf; 
                if (inst->frame_len < sizeof(nrng_request_frame_t)) 
                    return false;
                if (frame->dst_address != inst->my_short_address && (frame->dst_address != BROADCAST_ADDRESS || nrng->device_type == DWT_NRNG_INITIATOR)){
                    return true;
                }else{
                    STATS_INC(rng->stat, rx_complete); 
                    return false;   // Allow sub extensions to handle event
                }
            }
            break;
#endif //MYNEWT_VAL(NRNG_ENABLED)
        default: 
            return false;
    }
    return false;
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

    if (inst->fctrl != FCNTL_IEEE_RANGE_16)
        return false;

    switch(inst->rng->code) {
        case DWT_SS_TWR ... DWT_DS_TWR_EXT_END:
            STATS_INC(inst->rng->stat, tx_complete);
            return true;
            break;
        default: 
            return false;
    }
}


#if MYNEWT_VAL(RNG_VERBOSE)

static void
complete_ev_cb(struct os_event *ev) {
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    rng_encode(inst->rng);
}


struct os_callout rng_callout;
static bool
complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
        if (inst->fctrl != FCNTL_IEEE_RANGE_16)
        return false;

        os_callout_init(&rng_callout, os_eventq_dflt_get(), complete_ev_cb, inst);
        os_eventq_put(os_eventq_dflt_get(), &rng_callout.c_ev);
        return false;
}


#endif
