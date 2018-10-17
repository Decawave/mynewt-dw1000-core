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

#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_ftypes.h>
#include <rng/rng.h>
#include <dsp/polyval.h>

#if MYNEWT_VAL(TWR_DS_EXT_ENABLED)
#include <twr_ds_ext/twr_ds_ext.h>
#endif
#if MYNEWT_VAL(TWR_DS_ENABLED)
#include <twr_ds/twr_ds.h>
#endif
#if MYNEWT_VAL(TWR_SS_ENABLED)
#include <twr_ss/twr_ss.h>
#endif


//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

static bool rx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool tx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);

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
    .tx_holdoff_delay = MYNEWT_VAL(RNG_TX_HOLDOFF),         // Send Time delay in usec.
    .rx_timeout_period = MYNEWT_VAL(RNG_RX_TIMEOUT)       // Receive response timeout in usec
};

#if MYNEWT_VAL(DW1000_DEVICE_0)
static twr_frame_t g_twr_0[] = {
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
        .PANID = MYNEWT_VAL(PANID),                 // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID
    },
    [3] = {
        .fctrl = FCNTL_IEEE_RANGE_16,                // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = MYNEWT_VAL(PANID),                 // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID,
    }
};
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
static twr_frame_t g_twr_1[] = {
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
            .tx_complete_cb = tx_complete_cb
        },
#if MYNEWT_VAL(DW1000_DEVICE_1)
        [1] = {
            .id = DW1000_RNG,
            .rx_complete_cb = rx_complete_cb,
            .tx_complete_cb = tx_complete_cb
        },
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
        [2] = {
            .id = DW1000_RNG,
            .rx_complete_cb = rx_complete_cb,
            .tx_complete_cb = tx_complete_cb
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
 * API to initialise the rng_ss package.
 *
 *
 * @return void
 */

void rng_pkg_init(void){

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
    
    os_error_t err = os_sem_pend(&inst->rng->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    dw1000_rng_config_t * config = dw1000_rng_get_config(inst, code);
    
    dw1000_rng_instance_t * rng = inst->rng;                            
    twr_frame_t * frame  = inst->rng->frames[(rng->idx+1)%rng->nframes];    

    frame->seq_num++;
    frame->code = code;
    frame->src_address = inst->my_short_address;
    frame->dst_address = dst_address;
   
    dw1000_write_tx(inst, frame->array, 0, sizeof(ieee_rng_request_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_request_frame_t), 0, true);     
    dw1000_set_wait4resp(inst, true);  
     
    uint16_t timeout = dw1000_phy_frame_duration(&inst->attrib, sizeof(ieee_rng_response_frame_t)) 
                    + config->rx_timeout_period         // At least 2 * ToF, 1us ~= 300m
                    + config->tx_holdoff_delay;         // Remote side turn arroud time. 
    dw1000_set_rx_timeout(inst, timeout); 
   
    if (rng->control.delay_start_enabled) 
        dw1000_set_delay_start(inst, rng->delay);
   
    if (dw1000_start_tx(inst).start_tx_error){
        os_sem_release(&inst->rng->sem);
    }
    err = os_sem_pend(&inst->rng->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions 
    os_sem_release(&inst->rng->sem);
    
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
    float Pr = Pt + 2 * G + 20 * log10(299792458) - 20 * log10(4 * M_PI * fc * R);
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
dw1000_rng_twr_to_tof(dw1000_rng_instance_t * rng){
    float ToF = 0;
    uint64_t T1R, T1r, T2R, T2r;
    int64_t nom,denom;

    twr_frame_t * first_frame = rng->frames[(rng->idx-1)%rng->nframes];
    twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];

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
#endif

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

    DIAGMSG("{\"utime\": %lu,\"msg\": \"rx_complete_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

    dw1000_rng_instance_t * rng = inst->rng; 
    twr_frame_t * frame = rng->frames[(++rng->idx)%rng->nframes]; // advance to next frame 
    
    if (inst->frame_len >= sizeof(ieee_rng_request_frame_t))
        dw1000_read_rx(inst, frame->array, 0, sizeof(ieee_rng_request_frame_t));
    else 
        return false;
        
    inst->rng->code = frame->code;
    switch(frame->code) {
        case DWT_SS_TWR ... DWT_DS_TWR_EXT_END:
            if (inst->config.framefilter_enabled == false && frame->dst_address != inst->my_short_address){  
                // IEEE 802.15.4 standard ranging frames, software MAC filtering
                DIAGMSG("{\"utime\": %lu,\"msg\": \"software MAC filtering\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
                rng->idx--; // Rewind
                inst->control = inst->control_rx_context;
                dw1000_restart_rx(inst, inst->control);             
                return true;
            }  
            break;
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
            DIAGMSG("{\"utime\": %lu,\"msg\": \"tx_complete_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
            return true;
            break;
        default: 
            return false;
    }
}
