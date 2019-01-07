/**
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
 * @file wcs.c
 * @author Paul.Kettle@decawave.com
 * @date Oct 20 2018
 * @brief Wireless Clock Synchronization
 *
 * @details This is a top-level package for managing Clock Calibration using Clock Calibration Packet (CCP). 
 * In an RTLS system the Clock Master send a periodic blink which is received by the anchor nodes. The device driver model on the node
 * handles the ccp frame and schedules a callback for post-processing of the event. The Clock Calibration herein is an example 
 * of this post-processing. In TDOA-base RTLS system clock synchronization is essential, this can be either wired or wireless depending on the requirements.
 * In the case of wireless clock synchronization clock skew is estimated from the CCP packets. Depending on the accuracy required and the 
 * available computational resources two model is available; timescale or linear interpolation.  
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
#include <ccp/ccp.h>
#include <wcs/wcs.h>
#include <timescale/timescale.h>

//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

#undef TICTOC

static void wcs_postprocess(struct os_event * ev);

/*! 
 * @fn wcs_init(wcs_instance_t * inst,  dw1000_ccp_instance_t * ccp)
 *
 * @brief Allocate resources for the clkcal calibration tasks. Binds resources 
 * to dw1000_ccp interface and instantiate timescale instance if in use.      
 *
 * input parameters
 * @param inst - clkcal_instance_t *
 * @param ccp - dw1000_ccp_instance_t *,
 *
 * output parameters
 *
 * returns wcs_instance_t * 
 */

wcs_instance_t * 
wcs_init(wcs_instance_t * inst, dw1000_ccp_instance_t * ccp){

    if (inst == NULL ) {
        inst = (wcs_instance_t *) malloc(sizeof(wcs_instance_t)); 
        assert(inst);
        memset(inst, 0, sizeof(wcs_instance_t));
        inst->status.selfmalloc = 1;
    }
    inst->ccp = ccp;    
    double x0[TIMESCALE_N] = {0};
    double q[] = {MYNEWT_VAL(TIMESCALE_QVAR) * 1.0, MYNEWT_VAL(TIMESCALE_QVAR) * 0.1, MYNEWT_VAL(TIMESCALE_QVAR) * 0.01};
    double T = 1e-6l * MYNEWT_VAL(CCP_PERIOD);  // peroid in sec

    inst->timescale = timescale_init(NULL, x0, q, T); 
    inst->timescale->status.initialized = 0; //Ignore X0 values, until we get first event
    inst->status.initialized = 0;

    wcs_set_postprocess(inst, &wcs_postprocess);      // Using default process
    
    return inst;
}

/*! 
 * @fn wcs_free(wcs_instance_t * inst)
 *
 * @brief Free resources and restore default behaviour. 
 *
 * input parameters
 * @param inst - wcs_instance_t * inst
 *
 * output parameters
 *
 * returns none
 */
void 
wcs_free(wcs_instance_t * inst){
    assert(inst);  
    timescale_free(inst->timescale);
    if (inst->status.selfmalloc)
        free(inst);
    else
        inst->status.initialized = 0;
}


/*! 
 * @fn clkcal_update_cb(struct os_event * ev)
 *
 * @brief This function serves as a placeholder for clkcal updates based on ccp observation. The clkcal thread uses timescale 
 * if available otherwise defaults to linear interpolation. Once complete a post-process event is invoked.     
 *
 * input parameters
 * @param inst - struct os_event * ev * 
 *
 * output parameters
 *
 * returns none 
 */
void wcs_update_cb(struct os_event * ev){
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);
    dw1000_ccp_instance_t * ccp = (dw1000_ccp_instance_t *)ev->ev_arg;
    wcs_instance_t * inst = ccp->wcs;

    DIAGMSG("{\"utime\": %lu,\"msg\": \"wcs_update_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

    if(ccp->status.valid){ 
        ccp_frame_t * previous_frame = ccp->frames[(uint16_t)(ccp->idx-1)%ccp->nframes]; 
        ccp_frame_t * frame = ccp->frames[(ccp->idx)%ccp->nframes]; 
        inst->nT = (int16_t)frame->seq_num - (int16_t)previous_frame->seq_num;
        inst->nT = (inst->nT < 0)?0x100+inst->nT:inst->nT;
       
        timescale_instance_t * timescale = inst->timescale; 
        timescale_states_t * states = (timescale_states_t *) (inst->timescale->eke->x); 

        inst->master_epoch = ccp->epoch_master; //->transmission_timestamp;
        inst->local_epoch = ccp->epoch;//frame->reception_timestamp;
        
        if (inst->status.initialized == 0 ){
            double skew = (double ) dw1000_calc_clock_offset_ratio(ccp->parent, frame->carrier_integrator);
            states->time = (double)inst->local_epoch;
            states->skew = (1.0l - skew) * ((uint64_t)1 << 16)/1e-6l; 
            inst->status.initialized = 1;
        }else{
            double skew = 1.0l + (double ) dw1000_calc_clock_offset_ratio(ccp->parent, frame->carrier_integrator);
            double T = 1e-6l * frame->transmission_interval * inst->nT;   // peroid in seconds referenced to master
            double q[] = {MYNEWT_VAL(TIMESCALE_QVAR) * 1.0, MYNEWT_VAL(TIMESCALE_QVAR) * 0.1, MYNEWT_VAL(TIMESCALE_QVAR) * 0.01};
            double r[] = {MYNEWT_VAL(TIMESCALE_RVAR), MYNEWT_VAL(TIMESCALE_RVAR) * 1e10};
            double z[] = {(double)inst->local_epoch, skew};
            inst->status.valid  = timescale_main(timescale, z, q, r, T).valid;
        }
/*
        if(timescale->status.illconditioned || timescale->status.NotPositiveDefinitive ){
            inst->status.valid = 0;
            inst->status.initialized = 0;
            double x0[TIMESCALE_N] = {0};
            double q[] = {MYNEWT_VAL(TIMESCALE_QVAR) * 1.0, MYNEWT_VAL(TIMESCALE_QVAR) * 0.1, MYNEWT_VAL(TIMESCALE_QVAR) * 0.01};
            double T = 1e-6l * MYNEWT_VAL(CCP_PERIOD);  // peroid in sec
            timescale_init(inst->timescale, x0, q, T);
        }
*/
        inst->status.valid |= fabs(1.0l - states->skew * (1e-6l/((uint64_t)1UL << 16))) < 1e-5;
        if (inst->status.valid)
            inst->skew = 1.0l - states->skew * (1e-6l/((uint64_t)1UL << 16));
        else {
            inst->skew = 0.0l;
        }

    if(inst->config.postprocess == true)
        os_eventq_put(os_eventq_dflt_get(), &inst->postprocess_ev);
    }
}


/*! 
 * @fn wcs_postprocess(struct os_event * ev)
 *
 * @brief This function serves as a placeholder for timescale processing and by default creates json string for the event
 *
 * input parameters
 * @param inst - struct os_event *  
 *
 * output parameters
 *
 * returns none 
 */
static void 
wcs_postprocess(struct os_event * ev){
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

#if MYNEWT_VAL(WCS_VERBOSE)
    wcs_instance_t * inst = (wcs_instance_t *)ev->ev_arg;
    dw1000_ccp_instance_t * ccp = (void *)inst->ccp; 
    ccp_frame_t * previous_frame = ccp->frames[(uint16_t)(ccp->idx-1)%ccp->nframes]; 
    ccp_frame_t * frame = ccp->frames[(ccp->idx)%ccp->nframes]; 
//    wcs_instance_t * wcs = ccp->wcs;
//    timescale_states_t * states = (timescale_states_t *) (wcs->timescale->eke->x); 
    printf("{\"utime\": %lu,\"wcs\": [%llu,%llu,%llu],\"skew\": %llu,\"nT\": [%d,%d,%d]}\n", 
        os_cputime_ticks_to_usecs(os_cputime_get32()),
        (uint64_t) inst->master_epoch,
        (uint64_t) inst->local_epoch,
        (uint64_t) wcs_local_to_master(ccp->parent,  inst->local_epoch + frame->transmission_interval) - inst->master_epoch - frame->transmission_interval,
        *(uint64_t *)&(inst->skew),
        inst->nT,
        frame->seq_num,
        previous_frame->seq_num
    );
#endif
}

/*! 
 * @fn wcs_set_postprocess(wcs_instance_t *  inst * inst, os_event_fn * ccp_postprocess)
 *
 * @brief Overrides the default post-processing behaviors, replacing the JSON stream with an alternative 
 * or an advanced timescale processing algorithm.
 * 
 * input parameters
 * @param inst - wcs_instance_t *
 *
 * returns none
 */
void 
wcs_set_postprocess(wcs_instance_t * inst, os_event_fn * postprocess){
    inst->postprocess_ev.ev_cb = postprocess;
    inst->postprocess_ev.ev_arg = (void *)inst;
    inst->config.postprocess = true;
}

/**
 * API to compensate for local frequency to reference frequency clock skew
 *
 * @param inst  Pointer to _dw1000_dev_instance_t. 
 * @param dtu_time uint64_t time in decawave transeiver units of time (dtu)
 *
 * @return time
 * 
 */
inline uint64_t wcs_dtu_time_adjust(struct _dw1000_dev_instance_t * inst, uint64_t dtu_time){
    wcs_instance_t * wcs = inst->ccp->wcs;
    assert(wcs);
    
    if (wcs->status.valid)
       dtu_time = (uint64_t) roundl(dtu_time * wcs_dtu_time_correction(inst));

    return dtu_time & 0x00FFFFFFFFFFUL;
}

inline double wcs_dtu_time_correction(struct _dw1000_dev_instance_t * inst){
    wcs_instance_t * wcs = inst->ccp->wcs;
    assert(wcs);
    
    timescale_states_t * states = (timescale_states_t *) (wcs->timescale->eke->x); 

    double correction = 1.0l;
    if (wcs->status.valid)
       correction = (double) roundl(states->skew * (1e-6l/(1UL << 16)));

    return correction;
}


/**
 * API compensate for clock skew and offset relative to master clock
 *
 * @param inst  Pointer to _dw1000_dev_instance_t. 
 *
 * @return time
 * 
 */

inline uint64_t wcs_local_to_master(struct _dw1000_dev_instance_t * inst, uint64_t dtu_time){
    wcs_instance_t * wcs = inst->ccp->wcs;
    assert(wcs);
    timescale_states_t * states = (timescale_states_t *) (wcs->timescale->eke->x); 
    uint64_t delta = (dtu_time - wcs->local_epoch) & 0x0FFFFFFFFFFUL;

    if (wcs->status.valid)
        dtu_time = wcs->master_epoch + (uint64_t) round(delta / (states->skew * (1e-6l/(1UL << 16))));    

    return dtu_time & 0x0FFFFFFFFFFUL;
}


/**
 * 
 * With WCS_ENABLED the adj_ timer API compensates all local timestamps values for local local drift. 
 * This simplifies the TWR problem by mitigiating the need for double sided exchanges.  
 *
 * @param inst  Pointer to _dw1000_dev_instance_t. 
 * @return time
 */

inline uint64_t wcs_read_systime(struct _dw1000_dev_instance_t * inst){
    return wcs_dtu_time_adjust(inst, dw1000_read_systime(inst));
}

/**
 * API to read system time at lower offset address.
 *
 * @param inst  Pointer to _dw1000_dev_instance_t.
 * 
 * @return time 
 */

inline uint32_t wcs_read_systime_lo(struct _dw1000_dev_instance_t * inst){
    return (uint32_t) (wcs_dtu_time_adjust(inst, dw1000_read_systime_lo(inst)) & 0xFFFFFFFFUL);
}

/**
 * API to read receive time.
 *
 * @param inst  Pointer to _dw1000_dev_instance_t.
 *
 * @return time
 */


inline uint64_t wcs_read_rxtime(struct _dw1000_dev_instance_t * inst){
    return wcs_dtu_time_adjust(inst, dw1000_read_rxtime(inst));
}


/**
 * API to read receive time at lower offset address.
 *
 * @param inst  Pointer to _dw1000_dev_instance_t.
 *
 * @return time
 */

inline uint32_t wcs_read_rxtime_lo(struct _dw1000_dev_instance_t * inst){
    return (uint32_t) (wcs_dtu_time_adjust(inst, dw1000_read_rxtime_lo(inst)) & 0xFFFFFFFFUL);
}

/**
 * API to read transmission time.
 *
 * @param inst  Pointer to _dw1000_dev_instance_t. 
 *
 * @return time
 * 
 */

inline uint64_t wcs_read_txtime(struct _dw1000_dev_instance_t * inst){
    return wcs_dtu_time_adjust(inst, dw1000_read_txtime(inst));
}

/**
 * API to read transmit time at lower offset address
 *
 * @param inst  Pointer to _dw1000_dev_instance_t.
 *
 * @return time
 */

inline uint32_t wcs_read_txtime_lo(struct _dw1000_dev_instance_t * inst){
    return (uint32_t) (wcs_dtu_time_adjust(inst, dw1000_read_txtime_lo(inst)) & 0xFFFFFFFFUL);
}

/**
 * 
 * With WCS_ENABLED the wsc_ timer API transforms all local timestamps to the master clock timedomain, effectivly compensating for offset and drift. 
 * This simplifies the TDOA multilateration problem by referencing all times to a shared timedomain. Note all local timer event are still in the local timedomain 
 * and as such all dx_delay calcaultion should use the dw1000_ or adj_ api.  
 *
 * @param inst  Pointer to _dw1000_dev_instance_t. 
 * @return time
 */

inline uint64_t wcs_read_systime_master(struct _dw1000_dev_instance_t * inst){
    return wcs_local_to_master(inst, dw1000_read_systime(inst));
}


/**
 * API to read system time at lower offset address.
 *
 * @param inst  Pointer to _dw1000_dev_instance_t.
 * 
 * @return time 
 */

inline uint32_t wcs_read_systime_lo_master(struct _dw1000_dev_instance_t * inst){
       return (uint32_t) (wcs_local_to_master(inst, dw1000_read_systime_lo(inst)) & 0xFFFFFFFFUL);
}

/**
 * API to read receive time.
 *
 * @param inst  Pointer to _dw1000_dev_instance_t.
 *
 * @return time
 */


inline uint64_t wcs_read_rxtime_master(struct _dw1000_dev_instance_t * inst){
    return wcs_local_to_master(inst, dw1000_read_rxtime(inst));
}


/**
 * API to read receive time at lower offset address.
 *
 * @param inst  Pointer to _dw1000_dev_instance_t.
 *
 * @return time
 */

inline uint32_t wcs_read_rxtime_lo_master(struct _dw1000_dev_instance_t * inst){
    return (uint32_t) (wcs_local_to_master(inst, dw1000_read_rxtime_lo(inst)) & 0xFFFFFFFFUL);
}

/**
 * API to read transmission time.
 *
 * @param inst  Pointer to _dw1000_dev_instance_t. 
 *
 * @return time
 * 
 */

inline uint64_t wcs_read_txtime_master(struct _dw1000_dev_instance_t * inst){
    return wcs_local_to_master(inst, dw1000_read_txtime(inst));
}

/**
 * API to read transmit time at lower offset address.
 *
 * @param inst  Pointer to _dw1000_dev_instance_t.
 *
 * @return time
 */

inline uint32_t wcs_read_txtime_lo_master(struct _dw1000_dev_instance_t * inst){
    return (uint32_t) (wcs_local_to_master(inst, dw1000_read_txtime_lo(inst))& 0xFFFFFFFFUL);
}





