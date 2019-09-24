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
#include <os/os_dev.h>

#include <uwb/uwb.h>
#include <uwb_ccp/uwb_ccp.h>
#include <uwb_wcs/uwb_wcs.h>
#include <dw1000/dw1000_mac.h>
#include <timescale/timescale.h>

#if MYNEWT_VAL(UWB_WCS_ENABLED)

//#define DIAGMSG(s,u) printf(s,u)
#define WCS_DTU MYNEWT_VAL(UWB_WCS_DTU)

#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

#undef TICTOC

static void wcs_postprocess(struct dpl_event * ev);

static const double g_x0[TIMESCALE_N] = {0};
static const double g_q[] = { MYNEWT_VAL(TIMESCALE_QVAR) * 1.0l, MYNEWT_VAL(TIMESCALE_QVAR) * 0.1l, MYNEWT_VAL(TIMESCALE_QVAR) * 0.01l};
static const double g_T = 1e-6l * MYNEWT_VAL(UWB_CCP_PERIOD);  // peroid in sec

/*! 
 * @fn uwb_wcs_init(struct uwb_wcs_instance * inst,  struct uwb_ccp_instance * ccp)
 *
 * @brief Allocate resources for the clkcal calibration tasks. Binds resources 
 * to uwb_ccp interface and instantiate timescale instance if in use.
 *
 * input parameters
 * @param inst - clkcal_instance_t *
 * @param ccp - struct uwb_ccp_instance *,
 *
 * output parameters
 *
 * returns struct uwb_wcs_instance * 
 */

struct uwb_wcs_instance * 
uwb_wcs_init(struct uwb_wcs_instance * inst, struct uwb_ccp_instance * ccp){

    if (inst == NULL ) {
        inst = (struct uwb_wcs_instance *) malloc(sizeof(struct uwb_wcs_instance)); 
        assert(inst);
        memset(inst, 0, sizeof(struct uwb_wcs_instance));
        inst->status.selfmalloc = 1;
    }
    inst->ccp = ccp;    

    inst->timescale = timescale_init(NULL, g_x0, g_q, g_T);
    inst->timescale->status.initialized = 0; //Ignore X0 values, until we get first event
    inst->status.initialized = 0;

    uwb_wcs_set_postprocess(inst, &wcs_postprocess);      // Using default process

    return inst;
}

/*! 
 * @fn uwb_wcs_free(struct uwb_wcs_instance * inst)
 *
 * @brief Free resources and restore default behaviour. 
 *
 * input parameters
 * @param inst - struct uwb_wcs_instance * inst
 *
 * output parameters
 *
 * returns none
 */
void 
uwb_wcs_free(struct uwb_wcs_instance * inst){
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
void uwb_wcs_update_cb(struct dpl_event * ev)
{
    assert(ev != NULL);
    assert(dpl_event_get_arg(ev) != NULL);

    struct uwb_ccp_instance * ccp = (struct uwb_ccp_instance *)dpl_event_get_arg(ev);
    struct uwb_wcs_instance * wcs = ccp->wcs;
    timescale_instance_t * timescale = wcs->timescale;
    timescale_states_t * states = (timescale_states_t *) (timescale->eke->x);

    DIAGMSG("{\"utime\": %lu,\"msg\": \"uwb_wcs_update_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

    if(ccp->status.valid){
        uwb_ccp_frame_t * frame = ccp->frames[(ccp->idx)%ccp->nframes];

        wcs->observed_interval = (ccp->local_epoch - wcs->local_epoch.lo) & 0x0FFFFFFFFFFUL; // Observed ccp interval        
        wcs->master_epoch.timestamp = ccp->master_epoch.timestamp; 
        wcs->local_epoch.timestamp += wcs->observed_interval;

        if (wcs->status.initialized == 0){
            timescale = timescale_init(timescale, g_x0, g_q, g_T);
            /* Update pointer in case realloc happens in timescale_init */
            states = (timescale_states_t *) (timescale->eke->x);
            states->time = (double) wcs->master_epoch.lo;
            states->skew = (1.0l + (double ) uwb_calc_clock_offset_ratio(
                                ccp->dev_inst, frame->carrier_integrator,
                                UWB_CR_CARRIER_INTEGRATOR)) * MYNEWT_VAL(UWB_WCS_DTU);
            wcs->status.valid = wcs->status.initialized = 1;
        }else{
            double skew = (1.0l + (double ) uwb_calc_clock_offset_ratio(
                               ccp->dev_inst, frame->carrier_integrator,
                               UWB_CR_CARRIER_INTEGRATOR)) * MYNEWT_VAL(UWB_WCS_DTU);
            double T = wcs->observed_interval / WCS_DTU ; // observed interval in seconds, master reference
            //previous_frame->transmission_timestamp = (frame->transmission_timestamp + ((uint64_t)inst->ccp->period << 16)) & 0x0FFFFFFFFFFUL;
            //double T = 1e-6l * frame->transmission_interval * wcs->nT;   // interval in seconds referenced to master
            double q[] = {MYNEWT_VAL(TIMESCALE_QVAR) * 1.0, MYNEWT_VAL(TIMESCALE_QVAR) * 0.1, MYNEWT_VAL(TIMESCALE_QVAR) * 0.01};
            double r[] = {MYNEWT_VAL(TIMESCALE_RVAR),  WCS_DTU * 1e20};
            double z[] = {(double) wcs->master_epoch.lo, skew};
            wcs->status.valid = timescale_main(timescale, z, q, r, T).valid;
        }

        if (wcs->status.valid)
            wcs->skew = 1.0l - states->skew / WCS_DTU;
        else
            wcs->skew = 0.0l;

        if(wcs->config.postprocess == true)
            dpl_eventq_put(dpl_eventq_dflt_get(), &wcs->postprocess_ev);
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
wcs_postprocess(struct dpl_event * ev){
    assert(ev != NULL);
    assert(dpl_event_get_arg(ev) != NULL);

#if MYNEWT_VAL(UWB_WCS_VERBOSE)
    struct uwb_wcs_instance * wcs = (struct uwb_wcs_instance *) dpl_event_get_arg(ev);
    timescale_instance_t * timescale = wcs->timescale; 
    timescale_states_t * x = (timescale_states_t *) (timescale->eke->x); 

        printf("{\"utime\": %llu, \"wcs\": [%llu,%llu,%llu,%llu], \"skew\": %llu}\n",
        uwb_wcs_read_systime_master64(wcs->ccp->dev_inst),
        (uint64_t) wcs->master_epoch.timestamp,
        (uint64_t) uwb_wcs_local_to_master(wcs, wcs->local_epoch.lo),
        (uint64_t) wcs->local_epoch.timestamp,
        (uint64_t) x->time,
       *(uint64_t *)&(wcs->skew)
    );
#endif
}

/*! 
 * @fn uwb_wcs_set_postprocess(struct uwb_wcs_instance *  inst * inst, os_event_fn * ccp_postprocess)
 *
 * @brief Overrides the default post-processing behaviors, replacing the JSON stream with an alternative 
 * or an advanced timescale processing algorithm.
 * 
 * input parameters
 * @param inst - struct uwb_wcs_instance *
 *
 * returns none
 */
void 
uwb_wcs_set_postprocess(struct uwb_wcs_instance * inst, dpl_event_fn * postprocess)
{
    dpl_event_init(&inst->postprocess_ev, postprocess, (void *)inst);
    inst->config.postprocess = true;
}

/**
 * API to compensate for local frequency to reference frequency clock skew
 *
 * @param inst  Pointer to struct uwb_wcs_instance * wcs.
 * @param dtu_time uint64_t time in decawave transeiver units of time (dtu)
 *
 * @return time
 * 
 */
inline uint64_t uwb_wcs_dtu_time_adjust(struct uwb_wcs_instance * wcs, uint64_t dtu_time){
    
    if (wcs->status.valid)
       dtu_time = (uint64_t) roundl(dtu_time * uwb_wcs_dtu_time_correction(wcs));

    return dtu_time & 0x00FFFFFFFFFFUL;
}


inline double uwb_wcs_dtu_time_correction(struct uwb_wcs_instance * wcs){
    assert(wcs);

    timescale_states_t * x = (timescale_states_t *) (wcs->timescale->eke->x);

    double correction = 1.0l;
    if (wcs->status.valid)
       correction = (double) x->skew / WCS_DTU;

    return correction;
}


/**
 * API compensate for clock skew and offset relative to master clock
 *
 * @param wcs pointer to struct uwb_wcs_instance
 * @param dtu_time local observed timestamp
 * @return time
 * 
 */
uint64_t uwb_wcs_local_to_master64(struct uwb_wcs_instance * wcs, uint64_t dtu_time){
    timescale_instance_t * timescale = wcs->timescale; 

    double delta = ((dtu_time & 0x0FFFFFFFFFFUL) - wcs->local_epoch.lo) & 0x0FFFFFFFFFFUL;
    uint64_t master_lo40;
    if (wcs->status.valid) {
        /* No need to take special care of 40bit overflow as the timescale forward returns 
         * a double value that can exceed the 40bit. */
        master_lo40 = (uint64_t) round(timescale_forward(timescale, delta / WCS_DTU));
    } else {
        master_lo40 = wcs->master_epoch.lo + delta;
    }

    return (wcs->master_epoch.timestamp & 0xFFFFFF0000000000UL) + master_lo40;
}

/**
 * API compensate for clock skew and offset relative to master clock
 *
 * @param wcs pointer to struct uwb_wcs_instance
 * @param dtu_time local observed timestamp
 * @return time
 *
 */

uint64_t uwb_wcs_local_to_master(struct uwb_wcs_instance * wcs, uint64_t dtu_time){
    return uwb_wcs_local_to_master64(wcs, dtu_time) & 0x0FFFFFFFFFFUL;
}


/**
 *
 * With UWB_WCS_ENABLED the adjust_timer API compensates all local timestamps values for local local drift.
 * This simplifies the TWR problem by mitigiating the need for double sided exchanges.
 *
 * @param inst  Pointer to struct uwb_dev * inst.
 * @return time
 */

inline uint64_t uwb_wcs_read_systime(struct uwb_dev * inst){
    struct uwb_ccp_instance *ccp = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_CCP);
    struct uwb_wcs_instance * wcs = ccp->wcs;
    return uwb_wcs_dtu_time_adjust(wcs, uwb_read_systime(inst));
}

/**
 * API to read system time at lower offset address.
 *
 * @param inst  Pointer to struct uwb_dev * inst.
 * 
 * @return time 
 */

inline uint32_t uwb_wcs_read_systime_lo(struct uwb_dev * inst){
    struct uwb_ccp_instance *ccp = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_CCP);
    struct uwb_wcs_instance * wcs = ccp->wcs;
    return (uint32_t) (uwb_wcs_dtu_time_adjust(wcs, uwb_read_systime_lo32(inst)) & 0xFFFFFFFFUL);
}

/**
 * API to read receive time.
 *
 * @param inst  Pointer to struct uwb_dev * inst.
 *
 * @return time
 */


uint64_t uwb_wcs_read_rxtime(struct uwb_dev * inst){
    struct uwb_ccp_instance *ccp = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_CCP);
    struct uwb_wcs_instance * wcs = ccp->wcs;
    return uwb_wcs_dtu_time_adjust(wcs, uwb_read_rxtime(inst));
}


/**
 * API to read receive time at lower offset address.
 *
 * @param inst  Pointer to struct uwb_dev * inst.
 *
 * @return time
 */

uint32_t uwb_wcs_read_rxtime_lo(struct uwb_dev * inst){
    struct uwb_ccp_instance *ccp = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_CCP);
    struct uwb_wcs_instance * wcs = ccp->wcs;
    return (uint32_t) uwb_wcs_dtu_time_adjust(wcs, uwb_read_rxtime_lo32(inst)) & 0xFFFFFFFFUL;
}

/**
 * API to read transmission time.
 *
 * @param inst  Pointer to struct uwb_dev * inst. 
 *
 * @return time
 * 
 */

uint64_t uwb_wcs_read_txtime(struct uwb_dev * inst){
    struct uwb_ccp_instance *ccp = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_CCP);
    struct uwb_wcs_instance * wcs = ccp->wcs;
    return uwb_wcs_dtu_time_adjust(wcs, uwb_read_txtime(inst));
}

/**
 * API to read transmit time at lower offset address
 *
 * @param inst  Pointer to struct uwb_dev * inst.
 *
 * @return time
 */

uint32_t uwb_wcs_read_txtime_lo(struct uwb_dev * inst){
    struct uwb_ccp_instance *ccp = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_CCP);
    struct uwb_wcs_instance * wcs = ccp->wcs;
    return (uint32_t) (uwb_wcs_dtu_time_adjust(wcs, uwb_read_txtime_lo32(inst)) & 0xFFFFFFFFUL);
}

/**
 * 
 * With UWB_WCS_ENABLED the wsc_ timer API transforms all local timestamps to the master clock timedomain, effectivly compensating for offset and drift. 
 * This simplifies the TDOA multilateration problem by referencing all times to a shared timedomain. Note all local timer event are still in the local timedomain 
 * and as such all dx_delay calcaultion should use the uwb_ or adj_ api.
 *
 * @param inst  Pointer to struct uwb_dev * inst. 
 * @return time
 */

uint64_t uwb_wcs_read_systime_master(struct uwb_dev * inst){
    struct uwb_ccp_instance *ccp = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_CCP);
    struct uwb_wcs_instance * wcs = ccp->wcs;
    return uwb_wcs_local_to_master(wcs, uwb_read_systime(inst));
}


/**
 * 
 * With UWB_WCS_ENABLED the wsc_ timer API transforms all local systime to the master clock timedomain, effectivly compensating for offset and drift. 
 *
 * @param inst  Pointer to struct uwb_dev. 
 * @return time
 */

uint64_t uwb_wcs_read_systime_master64(struct uwb_dev * inst){
    struct uwb_ccp_instance *ccp = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_CCP);
    struct uwb_wcs_instance * wcs = ccp->wcs;
    return uwb_wcs_local_to_master64(wcs, uwb_read_systime(inst));
}

/**
 * API to read system time at lower offset address.
 *
 * @param inst  Pointer to struct uwb_dev * inst.
 * 
 * @return time 
 */

uint32_t uwb_wcs_read_systime_lo_master(struct uwb_dev * inst){
    struct uwb_ccp_instance *ccp = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_CCP);
    struct uwb_wcs_instance * wcs = ccp->wcs;
    return (uint32_t) (uwb_wcs_local_to_master(wcs, uwb_read_systime_lo32(inst)) & 0xFFFFFFFFUL);
}

/**
 * API to read receive time.
 *
 * @param inst  Pointer to struct uwb_dev * inst.
 *
 * @return time
 */


uint64_t uwb_wcs_read_rxtime_master(struct uwb_dev * inst){
    struct uwb_ccp_instance *ccp = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_CCP);
    struct uwb_wcs_instance * wcs = ccp->wcs;
    return uwb_wcs_local_to_master(wcs, uwb_read_rxtime(inst));
}


/**
 * API to read receive time at lower offset address.
 *
 * @param inst  Pointer to struct uwb_dev * inst.
 *
 * @return time
 */

uint32_t uwb_wcs_read_rxtime_lo_master(struct uwb_dev * inst){
    struct uwb_ccp_instance *ccp = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_CCP);
    struct uwb_wcs_instance * wcs = ccp->wcs;
    return (uint32_t) (uwb_wcs_local_to_master(wcs, uwb_read_rxtime_lo32(inst)) & 0xFFFFFFFFUL);
}

/**
 * API to read transmission time.
 *
 * @param inst  Pointer to struct uwb_dev. 
 *
 * @return time
 * 
 */

uint64_t uwb_wcs_read_txtime_master(struct uwb_dev * inst){
    struct uwb_ccp_instance *ccp = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_CCP);
    struct uwb_wcs_instance * wcs = ccp->wcs;
    return uwb_wcs_local_to_master(wcs, uwb_read_txtime(inst));
}

/**
 * API to read transmit time at lower offset address.
 *
 * @param inst  Pointer to struct uwb_dev.
 *
 * @return time
 */

uint32_t uwb_wcs_read_txtime_lo_master(struct uwb_dev * inst){
    struct uwb_ccp_instance *ccp = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_CCP);
    struct uwb_wcs_instance * wcs = ccp->wcs;
    return (uint32_t) (uwb_wcs_local_to_master(wcs, uwb_read_txtime_lo32(inst))& 0xFFFFFFFFUL);
}

#endif  /* MYNEWT_VAL(UWB_WCS_ENABLED) */
