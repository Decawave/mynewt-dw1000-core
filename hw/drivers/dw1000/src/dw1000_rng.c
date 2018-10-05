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
#include <dw1000/dw1000_rng.h>

#include <dsp/polyval.h>

//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
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

static void rng_tx_complete_cb(dw1000_dev_instance_t * inst);
static void rng_rx_complete_cb(dw1000_dev_instance_t * inst);
static void rng_rx_timeout_cb(dw1000_dev_instance_t * inst);
static void rng_rx_error_cb(dw1000_dev_instance_t * inst);
static void rng_tx_final_cb(dw1000_dev_instance_t * inst);

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
        inst->rng->config = config;
        dw1000_rng_config(inst, config);
    }

    dw1000_rng_set_callbacks(inst, rng_tx_complete_cb, rng_rx_complete_cb, rng_rx_timeout_cb, rng_rx_error_cb);
    dw1000_rng_set_tx_final_cb(inst, rng_tx_final_cb);
    dw1000_rng_set_complete_cb(inst, 0);

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
 * API to set the ranging callbacks.
 *
 * @param inst                  Pointer to dw1000_dev_instance_t.
 * @param rng_tx_complete_cb    Pointer to TX confirmation event callback function.
 * @param rx_complete_cb        Pointer to RX good frame event callback function.
 * @param rx_timeout_cb         Pointer to RX timeout events callback function.
 * @param rx_error_cb           Pointer to RX error events callback function. 
 *
 * @return void
 */
void 
dw1000_rng_set_callbacks(dw1000_dev_instance_t * inst,  dw1000_dev_cb_t rng_tx_complete_cb, dw1000_dev_cb_t rng_rx_complete_cb,  dw1000_dev_cb_t rng_rx_timeout_cb,  dw1000_dev_cb_t rng_rx_error_cb){
    inst->rng_tx_complete_cb = rng_tx_complete_cb;
    inst->rng_rx_complete_cb = rng_rx_complete_cb;
    inst->rng_rx_timeout_cb = rng_rx_timeout_cb;
    inst->rng_rx_error_cb = rng_rx_error_cb;
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

    inst->rng->config = config;
    return inst->status;
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
    
    dw1000_rng_instance_t * rng = inst->rng;                            
    twr_frame_t * frame  = inst->rng->frames[(++rng->idx)%rng->nframes];    
    dw1000_rng_config_t * config = inst->rng->config;

    frame->seq_num++;
    frame->code = code;
    frame->src_address = inst->my_short_address;
    frame->dst_address = dst_address;
   
    dw1000_write_tx(inst, frame->array, 0, sizeof(ieee_rng_request_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_request_frame_t), 0, true);     
    dw1000_set_wait4resp(inst, true);  
     
    uint16_t timeout = dw1000_phy_frame_duration(&inst->attrib, sizeof(ieee_rng_response_frame_t)) 
                    + config->rx_timeout_period         // 2 * ToF, 1us ~= 300m
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
 * API for final transmission to calculate range.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 *
 * @return void
 */
static void 
rng_tx_final_cb(dw1000_dev_instance_t * inst){

#ifdef DS_TWR_EXT_ENABLE
    dw1000_rng_instance_t * rng = inst->rng; 
    twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];

    frame->cartesian.x = MYNEWT_VAL(LOCAL_COORDINATE_X);
    frame->cartesian.y = MYNEWT_VAL(LOCAL_COORDINATE_Y);
    frame->cartesian.z = MYNEWT_VAL(LOCAL_COORDINATE_Z);
  
#if MYNEWT_VAL(DW1000_BIAS_CORRECTION_ENABLED)
    if (inst->config.bias_correction_enable){ 
        float range = dw1000_rng_tof_to_meters(dw1000_rng_twr_to_tof(rng)); 
        float bias = 2 * dw1000_rng_bias_correction(inst, 
                    dw1000_rng_path_loss(
                        MYNEWT_VAL(DW1000_DEVICE_TX_PWR),
                        MYNEWT_VAL(DW1000_DEVICE_ANT_GAIN),
                        MYNEWT_VAL(DW1000_DEVICE_FREQ),
                        range)
                    );
        frame->spherical.range = range - bias;
    }
#else
    frame->spherical.range = dw1000_rng_tof_to_meters(dw1000_rng_twr_to_tof(rng));
#endif
    frame->spherical_variance.range = MYNEWT_VAL(RANGE_VARIANCE);
    frame->spherical_variance.azimuth = -1;
    frame->spherical_variance.zenith = -1;
    frame->utime = os_cputime_ticks_to_usecs(os_cputime_get32());//dw1000_read_systime(inst)/128;
#endif
}

/**
 * API for transmission complete callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return void
 */
static void 
rng_tx_complete_cb(dw1000_dev_instance_t * inst)
{
    bool status = false;
    if(!(SLIST_EMPTY(&inst->extension_cbs))){
        dw1000_extension_callbacks_t *temp = NULL;
        SLIST_FOREACH(temp, &inst->extension_cbs, cbs_next){
            if(temp != NULL && temp->tx_complete_cb != NULL)
                status |= temp->tx_complete_cb(inst);
        }
    }
/*
    dw1000_rng_instance_t * rng = inst->rng; 
    if (status == false && inst->fctrl == FCNTL_IEEE_RANGE_16){
        twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];
        if (frame->code == DWT_SS_TWR_FINAL || frame->code == DWT_SS_TWR_T1){
            os_sem_release(&inst->rng->sem);  
        }else{ 
            twr_frame_t * frame = rng->frames[(rng->idx+1)%rng->nframes];
            if (frame->code ==  DWT_DS_TWR_FINAL || frame->code ==  DWT_DS_TWR_EXT_FINAL){
                os_sem_release(&inst->rng->sem);  
            }
        }
    }
*/
}


/**
 * API for receive timeout callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return void
 */
static void 
rng_rx_timeout_cb(dw1000_dev_instance_t * inst){

    bool status = false;
    if(!(SLIST_EMPTY(&inst->extension_cbs))){
        dw1000_extension_callbacks_t *temp = NULL;
        SLIST_FOREACH(temp, &inst->extension_cbs, cbs_next){
            if(temp != NULL && temp->rx_timeout_cb != NULL)
                status |= temp->rx_timeout_cb(inst);
        }
    }
    if(os_sem_get_count(&inst->rng->sem) == 0){
        printf("{\"utime\": %lu,\"log\": \"rng_rx_timeout_cb\",\"%s\":%d}\n",os_cputime_ticks_to_usecs(os_cputime_get32()),__FILE__, __LINE__); 
        os_error_t err = os_sem_release(&inst->rng->sem);
        assert(err == OS_OK);
    }
}

/**
 * API for receive error callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return void
 */
static void 
rng_rx_error_cb(dw1000_dev_instance_t * inst){

    bool status = false;
    if(!(SLIST_EMPTY(&inst->extension_cbs))){
        dw1000_extension_callbacks_t *temp = NULL;
        SLIST_FOREACH(temp, &inst->extension_cbs, cbs_next){
            if(temp != NULL && temp->rx_error_cb != NULL)
                status |= temp->rx_error_cb(inst);
        }
    }

    if(os_sem_get_count(&inst->rng->sem) == 0){
        printf("{\"utime\": %lu,\"log\": \"rng_rx_error_cb\",\"%s\":%d}\n",os_cputime_ticks_to_usecs(os_cputime_get32()),__FILE__, __LINE__); 
        os_error_t err = os_sem_release(&inst->rng->sem);   
        assert(err == OS_OK);
    }
}

/**
 * API for receive complete callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return void
 */
static void 
rng_rx_complete_cb(dw1000_dev_instance_t * inst)
{
    uint16_t code, dst_address; 
    dw1000_rng_config_t * config = inst->rng->config;
    dw1000_dev_control_t control = inst->control_rx_context;
    if (inst->fctrl == FCNTL_IEEE_RANGE_16){

        dw1000_read_rx(inst, (uint8_t *) &code, offsetof(ieee_rng_request_frame_t,code), sizeof(uint16_t));
        dw1000_read_rx(inst, (uint8_t *) &dst_address, offsetof(ieee_rng_request_frame_t,dst_address), sizeof(uint16_t));

    }else if(!(SLIST_EMPTY(&inst->extension_cbs))){
        bool status = false;
        dw1000_extension_callbacks_t * temp = NULL;
        SLIST_FOREACH(temp, &inst->extension_cbs, cbs_next){
            if(temp->rx_complete_cb != NULL)
                status |= temp->rx_complete_cb(inst);
        } 
        if (status == false){
            //No extension callbacks in place. So just return to receive mode again
            DIAGMSG("{\"utime\": %lu,\"msg\": \"No extension callbacks\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
            inst->control = inst->control_rx_context;
            if (dw1000_restart_rx(inst, control).start_rx_error)
                inst->rng_rx_error_cb(inst);
            return;
        }else{
            DIAGMSG("{\"utime\": %lu,\"msg\": \"extension callback found\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
            return;
        }    
    }
    
    if (dst_address != inst->my_short_address){  
        // IEEE 802.15.4 standard ranging frames, software MAC filtering
        DIAGMSG("{\"utime\": %lu,\"msg\": \"software MAC filtering\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
        inst->control = inst->control_rx_context;
        if (dw1000_restart_rx(inst, control).start_rx_error)  
            inst->rng_rx_error_cb(inst);    
        return;
    }  

        // IEEE 802.15.4 standard ranging frames
#if MYNEWT_VAL(DW1000_RNG_INDICATE_LED)
    hal_gpio_toggle(LED_1);
#endif

    switch (code){
#ifdef SS_TWR_ENABLE
        case DWT_SS_TWR ... DWT_SS_TWR_FINAL:
            switch(code){
                case DWT_SS_TWR:
                    {
                        // This code executes on the device that is responding to a request
                        DIAGMSG("{\"utime\": %lu,\"msg\": \"DWT_SS_TWR\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

                        dw1000_rng_instance_t * rng = inst->rng; 
                        twr_frame_t * frame = rng->frames[(++rng->idx)%rng->nframes];
                        if (inst->frame_len >= sizeof(ieee_rng_request_frame_t))
                            dw1000_read_rx(inst, frame->array, 0, sizeof(ieee_rng_request_frame_t));
                        else 
                            break; 
                    
                        uint64_t request_timestamp = dw1000_read_rxtime(inst);  
                        uint64_t response_tx_delay = request_timestamp + ((uint64_t)config->tx_holdoff_delay << 16);
                        uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;
        
                        frame->reception_timestamp = request_timestamp;
                        frame->transmission_timestamp = response_timestamp;
                        frame->dst_address = frame->src_address;
                        frame->src_address = inst->my_short_address;
                        frame->code = DWT_SS_TWR_T1;

                        dw1000_write_tx(inst, frame->array, 0, sizeof(ieee_rng_response_frame_t));
                        dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_response_frame_t), 0, true); 
                        dw1000_set_wait4resp(inst, true);    
                        dw1000_set_delay_start(inst, response_tx_delay);
                        uint16_t timeout = dw1000_phy_frame_duration(&inst->attrib, sizeof(ieee_rng_response_frame_t)) 
                            + config->rx_timeout_period        
                            + config->tx_holdoff_delay;         // Remote side turn arroud time. 
                        dw1000_set_rx_timeout(inst, timeout); 

                        if (dw1000_start_tx(inst).start_tx_error)
                            os_sem_release(&rng->sem);  
                        break;
                    }
                case DWT_SS_TWR_T1:
                    {
                        // This code executes on the device that initiated a request, and is now preparing the final timestamps
                        DIAGMSG("{\"utime\": %lu,\"msg\": \"DWT_SS_TWR_T1\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

                        dw1000_rng_instance_t * rng = inst->rng; 
                        twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];
                        if (inst->frame_len >= sizeof(ieee_rng_response_frame_t))
                            dw1000_read_rx(inst, frame->array, 0, sizeof(ieee_rng_response_frame_t));
                        else 
                            break;

                        frame->request_timestamp = dw1000_read_txtime_lo(inst);   // This corresponds to when the original request was actually sent
                        frame->response_timestamp = dw1000_read_rxtime_lo(inst);  // This corresponds to the response just received            
                        frame->dst_address = frame->src_address;
                        frame->src_address = inst->my_short_address;
                        frame->code = DWT_SS_TWR_FINAL;
                    
                        // Transmit timestamp final report
                        dw1000_write_tx(inst, frame->array, 0, sizeof(twr_frame_final_t));
                        dw1000_write_tx_fctrl(inst, sizeof(twr_frame_final_t), 0, true);
                        if (dw1000_start_tx(inst).start_tx_error)
                            os_sem_release(&rng->sem);  

                        if (inst->rng_complete_cb)
                            inst->rng_complete_cb(inst);

                        bool status = false;
                        if(!(SLIST_EMPTY(&inst->extension_cbs))){
                            dw1000_extension_callbacks_t *temp = NULL;
                            SLIST_FOREACH(temp, &inst->extension_cbs, cbs_next){
                                if(temp != NULL && temp->rx_complete_cb != NULL)
                                    status |= temp->rx_complete_cb(inst);
                                }
                        }
                        os_sem_release(&rng->sem);  
                        break;
                    }
                case  DWT_SS_TWR_FINAL:
                    {
                        // This code executes on the device that responded to the original request, and has now receive the response final timestamp. 
                        // This marks the completion of the single-size-two-way request. This final 4th message is perhaps optional in some applicaiton. 
                        DIAGMSG("{\"utime\": %lu,\"msg\": \"DWT_SS_TWR_FINAL\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

                        dw1000_rng_instance_t * rng = inst->rng; 
                        twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];
                        if (inst->frame_len >= sizeof(twr_frame_final_t))
                            dw1000_read_rx(inst, frame->array, 0, sizeof(twr_frame_final_t));
                        os_sem_release(&rng->sem);
                        
                        if (inst->rng_complete_cb) 
                            inst->rng_complete_cb(inst);

                        bool status = false;
                        if(!(SLIST_EMPTY(&inst->extension_cbs))){
                            dw1000_extension_callbacks_t *temp = NULL;
                            SLIST_FOREACH(temp, &inst->extension_cbs, cbs_next){
                                if(temp != NULL && temp->rx_complete_cb != NULL)
                                    status |= temp->rx_complete_cb(inst);
                                }
                        }
                        os_sem_release(&rng->sem);  
                        break;
                    }
                default: 
                    break;
             }
             break;
#endif //SS_TWR_ENABLE
#ifdef DS_TWR_ENABLE
        case DWT_DS_TWR ... DWT_DS_TWR_FINAL:
            switch(code){
                    case DWT_DS_TWR:
                        {
                            // This code executes on the device that is responding to a original request
                            DIAGMSG("{\"utime\": %lu,\"msg\": \"DWT_DS_TWR\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

                            dw1000_rng_instance_t * rng = inst->rng; 
                            twr_frame_t * frame = rng->frames[(++rng->idx)%rng->nframes];
                            if (inst->frame_len >= sizeof(ieee_rng_request_frame_t))
                                dw1000_read_rx(inst, frame->array, 0, sizeof(ieee_rng_request_frame_t));
                            else 
                                break; 

                            uint64_t request_timestamp = dw1000_read_rxtime(inst);
                            uint64_t response_tx_delay = request_timestamp + ((uint64_t)config->tx_holdoff_delay << 16);
                            uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;
            
                            frame->reception_timestamp =  request_timestamp;
                            frame->transmission_timestamp =  response_timestamp;
                            frame->dst_address = frame->src_address;
                            frame->src_address = inst->my_short_address;
                            frame->code = DWT_DS_TWR_T1;

                            dw1000_write_tx(inst, frame->array, 0, sizeof(ieee_rng_response_frame_t));
                            dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_response_frame_t), 0, true); 
                            dw1000_set_wait4resp(inst, true);    
                            dw1000_set_delay_start(inst, response_tx_delay); 

                            uint16_t timeout = dw1000_phy_frame_duration(&inst->attrib, sizeof(ieee_rng_response_frame_t)) 
                                + config->rx_timeout_period        
                                + config->tx_holdoff_delay;         // Remote side turn arroud time. 
                            dw1000_set_rx_timeout(inst, timeout); 

                            if (dw1000_start_tx(inst).start_tx_error)
                                os_sem_release(&rng->sem);
                            
                            break;
                        }
                    case DWT_DS_TWR_T1:
                        {
                            // This code executes on the device that initiated the original request, and is now preparing the next series of timestamps
                            // The 1st frame now contains a local copy of the initial first side of the double sided scheme. 
                            DIAGMSG("{\"utime\": %lu,\"msg\": \"DWT_DS_TWR_T1\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

                            dw1000_rng_instance_t * rng = inst->rng; 
                            twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];
                            twr_frame_t * next_frame = rng->frames[(++rng->idx)%rng->nframes];

                            if (inst->frame_len >= sizeof(ieee_rng_response_frame_t))
                                dw1000_read_rx(inst, frame->array, 0, sizeof(ieee_rng_response_frame_t));
                            else 
                                break;

                            // This corresponds to when the original request was actually sent
                            frame->request_timestamp = next_frame->request_timestamp = dw1000_read_txtime_lo(inst);
                            // This corresponds to the response just received
                            frame->response_timestamp = next_frame->response_timestamp = dw1000_read_rxtime_lo(inst);
                      
                            uint16_t src_address = frame->src_address; 
                            uint8_t seq_num = frame->seq_num; 

                             // Note:: Advance to next frame 
                            frame = next_frame;                            
                            frame->dst_address = src_address;
                            frame->src_address = inst->my_short_address;
                            frame->seq_num = seq_num + 1;
                            frame->code = DWT_DS_TWR_T2;

                            uint64_t request_timestamp = dw1000_read_rxtime(inst);  
                            uint64_t response_tx_delay = request_timestamp + ((uint64_t)config->tx_holdoff_delay << 16);
                            uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;
                            
                            frame->reception_timestamp = request_timestamp;
                            frame->transmission_timestamp = response_timestamp;

                            dw1000_write_tx(inst, frame->array, 0, sizeof(twr_frame_final_t));
                            dw1000_write_tx_fctrl(inst, sizeof(twr_frame_final_t), 0, true);
                            dw1000_set_wait4resp(inst, true);
                            dw1000_set_delay_start(inst, response_tx_delay);

                            uint16_t timeout = dw1000_phy_frame_duration(&inst->attrib, sizeof(twr_frame_final_t)) 
                                + config->rx_timeout_period        
                                + config->tx_holdoff_delay;         // Remote side turn around time. 
                            dw1000_set_rx_timeout(inst, timeout); 
                            
                            if (dw1000_start_tx(inst).start_tx_error)
                                os_sem_release(&rng->sem);  
							
                            break; 
                        }

                    case DWT_DS_TWR_T2:
                        {
                            // This code executes on the device that responded to the original request, and is now preparing the final timestamps
                            DIAGMSG("{\"utime\": %lu,\"msg\": \"DWT_DS_TWR_T2\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

                            dw1000_rng_instance_t * rng = inst->rng; 
                            twr_frame_t * previous_frame = rng->frames[(rng->idx++)%rng->nframes];
                            twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];

                            if (inst->frame_len >= sizeof(twr_frame_final_t))
                                dw1000_read_rx(inst,  frame->array, 0, sizeof(twr_frame_final_t));
                            else 
                                break;

                            previous_frame->request_timestamp = frame->request_timestamp;
                            previous_frame->response_timestamp = frame->response_timestamp;

                            frame->request_timestamp = dw1000_read_txtime_lo(inst);   // This corresponds to when the original request was actually sent
                            frame->response_timestamp = dw1000_read_rxtime_lo(inst);  // This corresponds to the response just received            
                            frame->dst_address = frame->src_address;
                            frame->src_address = inst->my_short_address;
                            frame->code = DWT_DS_TWR_FINAL;

                            // Transmit timestamp final report
                            dw1000_write_tx(inst, frame->array, 0, sizeof(twr_frame_final_t));
                            dw1000_write_tx_fctrl(inst, sizeof(twr_frame_final_t), 0, true); 

                            if (dw1000_start_tx(inst).start_tx_error)
                                os_sem_release(&rng->sem);    
                            
                            if (inst->rng_complete_cb) 
                                inst->rng_complete_cb(inst);
                            
                            bool status = false;
                            if(!(SLIST_EMPTY(&inst->extension_cbs))){
                                dw1000_extension_callbacks_t *temp = NULL;
                                SLIST_FOREACH(temp, &inst->extension_cbs, cbs_next){
                                    if(temp != NULL && temp->rx_complete_cb != NULL)
                                        status |= temp->rx_complete_cb(inst);
                                    }
                            }
                            
                            os_sem_release(&rng->sem);
                            break;
                        }
                    case  DWT_DS_TWR_FINAL:
                        {
                            // This code executes on the device that initialed the original request, and has now receive the final response timestamp. 
                            // This marks the completion of the double-single-two-way request. 
                            DIAGMSG("{\"utime\": %lu,\"msg\": \"DWT_DS_TWR_FINAL\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

                            dw1000_rng_instance_t * rng = inst->rng; 
                            twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];
                            if (inst->frame_len >= sizeof(twr_frame_final_t))
                                dw1000_read_rx(inst, frame->array, 0, sizeof(twr_frame_final_t));
                           
                            if (inst->rng_complete_cb) 
                                inst->rng_complete_cb(inst);
                            
                            bool status = false;
                            if(!(SLIST_EMPTY(&inst->extension_cbs))){
                                dw1000_extension_callbacks_t *temp = NULL;
                                SLIST_FOREACH(temp, &inst->extension_cbs, cbs_next){
                                    if(temp != NULL && temp->rx_complete_cb != NULL)
                                        status |= temp->rx_complete_cb(inst);
                                    }
                            }

                            os_sem_release(&rng->sem);
                            break;
                        }
                    default: 
                        break;
                }
            break;
#endif //DS_TWR_ENABLE
#ifdef DS_TWR_EXT_ENABLE
        case DWT_DS_TWR_EXT ... DWT_DS_TWR_EXT_FINAL:
            switch(code){
                    case DWT_DS_TWR_EXT:
                        {
                            // This code executes on the device that is responding to a original request
                            DIAGMSG("{\"utime\": %lu,\"msg\": \"DWT_DS_TWR_EXT\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

                            dw1000_rng_instance_t * rng = inst->rng; 
                            twr_frame_t * frame = rng->frames[(++rng->idx)%rng->nframes];
                            if (inst->frame_len >= sizeof(ieee_rng_request_frame_t))
                                dw1000_read_rx(inst, frame->array, 0, sizeof(ieee_rng_request_frame_t));
                            else 
                                break; 

                            uint64_t request_timestamp = dw1000_read_rxtime(inst);  
                            uint64_t response_tx_delay = request_timestamp + ((uint64_t)config->tx_holdoff_delay << 16); 
                            uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;
            
                            frame->reception_timestamp = request_timestamp;
                            frame->transmission_timestamp = response_timestamp;

                            frame->dst_address = frame->src_address;
                            frame->src_address = inst->my_short_address;
                            frame->code = DWT_DS_TWR_EXT_T1;

                            dw1000_write_tx(inst, frame->array, 0, sizeof(ieee_rng_response_frame_t));
                            dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_response_frame_t), 0, true); 
                            dw1000_set_wait4resp(inst, true);    
                            dw1000_set_delay_start(inst, response_tx_delay);   
                            uint16_t timeout = dw1000_phy_frame_duration(&inst->attrib, sizeof(ieee_rng_response_frame_t)) 
                                + config->rx_timeout_period        
                                + config->tx_holdoff_delay;         // Remote side turn arroud time. 
                            dw1000_set_rx_timeout(inst, timeout); 

                            if (dw1000_start_tx(inst).start_tx_error)
                                os_sem_release(&rng->sem);

                            break;
                        }
                    case DWT_DS_TWR_EXT_T1:
                        {
                            // This code executes on the device that initiated the original request, and is now preparing the next series of timestamps
                            // The 1st frame now contains a local copy of the initial first side of the double sided scheme. 
                            DIAGMSG("{\"utime\": %lu,\"msg\": \"DWT_DS_TWR_T1\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

                            dw1000_rng_instance_t * rng = inst->rng; 
                            twr_frame_t * frame = rng->frames[(rng->idx++)%rng->nframes];
                            twr_frame_t * next_frame = rng->frames[(rng->idx)%rng->nframes];

                            if (inst->frame_len >= sizeof(ieee_rng_response_frame_t))
                                dw1000_read_rx(inst, frame->array, 0, sizeof(ieee_rng_response_frame_t));
                            else 
                                break;

                            frame->request_timestamp = next_frame->request_timestamp = dw1000_read_txtime_lo(inst);    // This corresponds to when the original request was actually sent
                            frame->response_timestamp = next_frame->response_timestamp = dw1000_read_rxtime_lo(inst);  // This corresponds to the response just received      
                        
                            uint16_t src_address = frame->src_address; 
                            uint8_t seq_num = frame->seq_num; 

                            // Note:: Advance to next frame 
                            frame = next_frame;                            
                            frame->dst_address = src_address;
                            frame->src_address = inst->my_short_address;
                            frame->seq_num = seq_num + 1;
                            frame->code = DWT_DS_TWR_EXT_T2;

                            uint64_t request_timestamp = dw1000_read_rxtime(inst);  
                            uint64_t response_tx_delay = request_timestamp + ((uint64_t)config->tx_holdoff_delay << 16); 
                            uint64_t response_timestamp = (response_tx_delay & 0xFFFFFFFE00UL) + inst->tx_antenna_delay;
                            
                            frame->reception_timestamp = request_timestamp;
                            frame->transmission_timestamp = response_timestamp;

                            // Final callback, prior to transmission, use this callback to populate the EXTENDED_FRAME fields.
                            if (inst->rng_tx_final_cb != NULL)
                                inst->rng_tx_final_cb(inst);

                            dw1000_write_tx(inst, frame->array, 0, sizeof(twr_frame_t));
                            dw1000_write_tx_fctrl(inst, sizeof(twr_frame_t), 0, true); 
                            dw1000_set_wait4resp(inst, true);    
                            dw1000_set_delay_start(inst, response_tx_delay);   
                            uint16_t timeout = dw1000_phy_frame_duration(&inst->attrib, sizeof(twr_frame_t)) 
                                + config->rx_timeout_period        
                                + config->tx_holdoff_delay;         // Remote side turn arroud time. 
                            dw1000_set_rx_timeout(inst, timeout); 

                        
                            if (dw1000_start_tx(inst).start_tx_error)
                                os_sem_release(&rng->sem);  
							
                            break; 
                        }

                    case DWT_DS_TWR_EXT_T2:
                        {
                            // This code executes on the device that responded to the original request, and is now preparing the final timestamps
                            DIAGMSG("{\"utime\": %lu,\"msg\": \"DWT_DS_TWR_T2\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

                            dw1000_rng_instance_t * rng = inst->rng; 
                            twr_frame_t * previous_frame = rng->frames[(rng->idx++)%rng->nframes];
                            twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];

                            if (inst->frame_len >= sizeof(twr_frame_t))
                                dw1000_read_rx(inst, frame->array, 0, sizeof(twr_frame_t));
                            else 
                                break;

                            previous_frame->request_timestamp = frame->request_timestamp;
                            previous_frame->response_timestamp = frame->response_timestamp;

                            frame->request_timestamp = dw1000_read_txtime_lo(inst);   // This corresponds to when the original request was actually sent
                            frame->response_timestamp = dw1000_read_rxtime_lo(inst);  // This corresponds to the response just received            
                            frame->dst_address = frame->src_address;
                            frame->src_address = inst->my_short_address;
                            frame->code = DWT_DS_TWR_EXT_FINAL;

                            // Final callback, prior to transmission, use this callback to populate the EXTENDED_FRAME fields.
                            if (inst->rng_tx_final_cb != NULL)
                                inst->rng_tx_final_cb(inst);

                            // Transmit timestamp final report
                            dw1000_write_tx(inst, frame->array, 0, sizeof(twr_frame_t));
                            dw1000_write_tx_fctrl(inst, sizeof(twr_frame_t), 0, true); 
                         
                            if (dw1000_start_tx(inst).start_tx_error)
                                os_sem_release(&rng->sem);

                            bool status = false;
                            if(!(SLIST_EMPTY(&inst->extension_cbs))){
                                dw1000_extension_callbacks_t *temp = NULL;
                                SLIST_FOREACH(temp, &inst->extension_cbs, cbs_next){
                                    if(temp != NULL && temp->rx_complete_cb != NULL)
                                        status |= temp->rx_complete_cb(inst);
                                    }
                            }
                            
                            os_sem_release(&rng->sem);
                            if (inst->rng_complete_cb) {
                                inst->rng_complete_cb(inst);
                            }
                            break;
                        }
                    case  DWT_DS_TWR_EXT_FINAL:
                        {
                            // This code executes on the device that initialed the original request, and has now receive the final response timestamp. 
                            // This marks the completion of the double-single-two-way request. 
                            DIAGMSG("{\"utime\": %lu,\"msg\": \"DWT_DS_TWR_FINAL\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

                            dw1000_rng_instance_t * rng = inst->rng; 
                            twr_frame_t * frame = inst->rng->frames[(rng->idx)%rng->nframes];
                            if (inst->frame_len >= sizeof(twr_frame_t))
                                dw1000_read_rx(inst, frame->array, 0, sizeof(twr_frame_t));
                            os_sem_release(&rng->sem);

                            bool status = false;
                            if(!(SLIST_EMPTY(&inst->extension_cbs))){
                                dw1000_extension_callbacks_t *temp = NULL;
                                SLIST_FOREACH(temp, &inst->extension_cbs, cbs_next){
                                    if(temp != NULL && temp->rx_complete_cb != NULL)
                                        status |= temp->rx_complete_cb(inst);
                                    }
                            }

                            if (inst->rng_complete_cb) 
                                inst->rng_complete_cb(inst);
                            
                            break;
                        }
                    default: 
                        break;
                }
            break;
#endif //DS_TWR_EXT_ENABLE
        default: 
            // Use this callback to extend interface and ranging services
            if(!(SLIST_EMPTY(&inst->extension_cbs))){
                dw1000_extension_callbacks_t *temp = NULL;
                SLIST_FOREACH(temp, &inst->extension_cbs, cbs_next){
                    if(temp != NULL)
                        if(temp->rx_complete_cb != NULL)
                            if(temp->rx_complete_cb(inst) == true)
                                break;
                }
            }
            break;
    }

}



