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

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <os/os.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include "bsp/bsp.h"

#if MYNEWT_VAL(DW1000_CCP_ENABLED)
#include <dw1000/dw1000_ccp.h>

#if MYNEWT_VAL(CLOCK_CALIBRATION_ENABLED)
#include <clkcal/clkcal.h>
#endif

#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED) 

/*	%Lowpass design
 Fs = 1
 Fc = 0.2
 [z,p,k] = cheby2(6,80,Fc/Fs);
 [sos]=zp2sos(z,p,k);
 fvtool(sos);
 info = stepinfo(zp2tf(p,z,k));
 sprintf("#define FS_XTALT_SETTLINGTIME %d", 2*ceil(info.SettlingTime))
 sos2c(sos,'g_fs_xtalt')
*/
#define FS_XTALT_SETTLINGTIME 17
static float g_fs_xtalt_b[] ={
     	2.160326e-04, 9.661246e-05, 2.160326e-04, 
     	1.000000e+00, -1.302658e+00, 1.000000e+00, 
     	1.000000e+00, -1.593398e+00, 1.000000e+00, 
     	};
static float g_fs_xtalt_a[] ={
     	1.000000e+00, -1.555858e+00, 6.083635e-01, 
     	1.000000e+00, -1.661260e+00, 7.136943e-01, 
     	1.000000e+00, -1.836731e+00, 8.911796e-01, 
     	};
/*
% From Figure 29 PPM vs Crystal Trim
p=polyfit([30,20,0,-18],[0,5,17,30],2) 
mat2c(p,'g_fs_xtalt_poly')
*/
static float g_fs_xtalt_poly[] ={
     	3.252948e-03, -6.641957e-01, 1.699287e+01, 
     	};
#endif


static void ccp_rx_complete_cb(struct _dw1000_dev_instance_t * inst);
static void ccp_tx_complete_cb(struct _dw1000_dev_instance_t * inst);
static void ccp_rx_error_cb(dw1000_dev_instance_t * inst);
static void ccp_rx_timeout_cb(dw1000_dev_instance_t * inst);
static void ccp_tx_error_cb(dw1000_dev_instance_t * inst);
static struct _dw1000_ccp_status_t dw1000_ccp_blink(struct _dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode);
#if MYNEWT_VAL(CLOCK_CALIBRATION_ENABLED) !=1
static void ccp_postprocess(struct os_event * ev);
#endif
/*! 
 * @fn ccp_timer_ev_cb(struct os_event *ev)
 *
 * @brief The OS scheduler is not accurate enough for the timing requirement of an RTLS system.  
 * Instead, the OS is used to schedule events in advance of the actual event.  
 * The DW1000 delay start mechanism then takes care of the actual event. This removes the non-deterministic 
 * latencies of the OS implementation.  
 * 
 * input parameters
 * @param inst - struct os_event *
 *    
 * output parameters
 *
 * returns none
 */
static void 
ccp_timer_ev_cb(struct os_event *ev) {
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_ccp_instance_t * ccp = inst->ccp; 
    if(dw1000_ccp_blink(inst, DWT_BLOCKING).start_tx_error)
      os_callout_reset(&ccp->callout_timer, OS_TICKS_PER_SEC * (ccp->period - MYNEWT_VAL(OS_LATENCY)) * 1e-6);
}

/*! 
 * @fn ccp_timer_init(dw1000_dev_instance_t * inst)
 *
 * @brief The default eventq is used. 
 * 
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 *    
 * output parameters
 *
 * returns none
 */
static void 
ccp_timer_init(struct _dw1000_dev_instance_t * inst) {

    dw1000_ccp_instance_t * ccp = inst->ccp; 
    os_callout_init(&ccp->callout_timer, os_eventq_dflt_get(), ccp_timer_ev_cb, (void *) inst);
    os_callout_reset(&ccp->callout_timer, OS_TICKS_PER_SEC/100);
    ccp->status.timer_enabled = true;
}


/*! 
 * @fn dw1000_ccp_init(dw1000_dev_instance_t * inst, uint16_t nframes, uint64_t clock_master)
 *
 * @brief Precise timing is achieved by adding a fixed period to the transmission time of the previous frame. 
 * This approach solves the non-deterministic latencies caused by the OS. The OS, however, is used to schedule 
 * the next transmission event, but the DW1000 controls the actual next transmission time using the dw1000_set_delay_start.
 * This function allocates all the required resources. In the case of a large scale deployment multiple instances 
 * can be uses to track multiple clock domains. 
 * 
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 * @param nframes - Nominally set to 2 frames for the simple use case. But depending on the interpolation 
 * algorithm this should be set accordingly. For example, four frames are required or bicubic interpolation. 
 * @param clock_master - UUID address of the system clock_master all other masters are rejected.  
 * output parameters
 *
 * returns dw1000_ccp_instance_t * 
 */
dw1000_ccp_instance_t * 
dw1000_ccp_init(struct _dw1000_dev_instance_t * inst, uint16_t nframes, uint64_t clock_master){
    assert(inst);
    assert(nframes > 1);
    dw1000_extension_callbacks_t ccp_cbs;

    if (inst->ccp == NULL ) {
        dw1000_ccp_instance_t * ccp = (dw1000_ccp_instance_t *) malloc(sizeof(dw1000_ccp_instance_t) + nframes * sizeof(ccp_frame_t *)); 
        assert(ccp);
        memset(ccp, 0, sizeof(dw1000_ccp_instance_t));
        ccp->status.selfmalloc = 1;
        ccp->nframes = nframes; 
        ccp_frame_t ccp_default = {
            .fctrl = FCNTL_IEEE_BLINK_CCP_64,    // frame control (FCNTL_IEEE_BLINK_64 to indicate a data frame using 16-bit addressing).
            .correction_factor = 1.0f,
            .seq_num = 0xFF
        };
        for (uint16_t i = 0; i < ccp->nframes; i++){
            ccp->frames[i] = (ccp_frame_t *) malloc(sizeof(ccp_frame_t)); 
            memcpy(ccp->frames[i], &ccp_default, sizeof(ccp_frame_t));
            ccp->frames[i]->seq_num -= nframes - i + 1;
        }
        ccp->parent = inst;
        inst->ccp = ccp;
    }else{
        assert(inst->ccp->nframes == nframes);
    }
    inst->ccp->period = MYNEWT_VAL(CCP_PERIOD);
    inst->ccp->config = (dw1000_ccp_config_t){
        .postprocess = false,
#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED) 
        .fs_xtalt_autotune = true,
#endif
    };
    inst->clock_master = clock_master;

    os_error_t err = os_sem_init(&inst->ccp->sem, 0x1); 
    assert(err == OS_OK);

#if MYNEWT_VAL(CLOCK_CALIBRATION_ENABLED)
    inst->ccp->clkcal = clkcal_init(NULL, inst->ccp);      // Using clkcal process
#else
    dw1000_ccp_set_postprocess(inst->ccp, &ccp_postprocess);            // Using default process
#endif

    ccp_cbs.tx_complete_cb = ccp_tx_complete_cb;
    ccp_cbs.rx_complete_cb = ccp_rx_complete_cb;
    ccp_cbs.rx_timeout_cb = ccp_rx_timeout_cb;
    ccp_cbs.rx_error_cb = ccp_rx_error_cb;
    ccp_cbs.tx_error_cb = ccp_tx_error_cb;
    
    dw1000_ccp_set_ext_callbacks(inst, ccp_cbs);

    dw1000_ccp_instance_t * ccp = inst->ccp; 
    ccp_frame_t * frame = ccp->frames[(ccp->idx)%ccp->nframes]; 

#if MYNEWT_VAL(CLOCK_CALIBRATION)
    frame->transmission_timestamp = _dw1000_read_systime(inst);
#else
    frame->transmission_timestamp = dw1000_read_systime(inst);
#endif

#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED) 
    inst->ccp->xtalt_sos = sosfilt_init(NULL, sizeof(g_fs_xtalt_b)/sizeof(float)/BIQUAD_N);
#endif

    inst->ccp->status.initialized = 1;
    return inst->ccp;
}

/*! 
 * @fn dw1000_ccp_free(dw1000_ccp_instance_t * inst)
 *
 * @brief Deconstructor
 * 
 * input parameters
 * @param inst - dw1000_cpp_instance_t * 
 *
 * returns none
 */
void 
dw1000_ccp_free(dw1000_ccp_instance_t * inst){
    assert(inst);  
#if MYNEWT_VAL(CLOCK_CALIBRATION_ENABLED)
    clkcal_free(inst->clkcal);
#endif         
#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED) 
    sosfilt_free(inst->xtalt_sos);
#endif   
    if (inst->status.selfmalloc){
        for (uint16_t i = 0; i < inst->nframes; i++)
            free(inst->frames[i]);
        free(inst);
    }
    else
        inst->status.initialized = 0;
}

void dw1000_ccp_set_ext_callbacks(dw1000_dev_instance_t * inst, dw1000_extension_callbacks_t ccp_cbs){
    ccp_cbs.id = DW1000_CCP;
    dw1000_add_extension_callbacks(inst , ccp_cbs);
}


/*! 
 * @fn dw1000_ccp_set_postprocess(dw1000_dev_instance_t * inst, os_event_fn * ccp_postprocess)
 *
 * @briefOverrides the default post-processing behaviors, replacing the JSON stream with an alternative 
 * or an advanced timescale processing algorithm.
 * 
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 *
 * returns none
 */
void 
dw1000_ccp_set_postprocess(dw1000_ccp_instance_t * inst, os_event_fn * ccp_postprocess){
    os_callout_init(&inst->callout_postprocess, os_eventq_dflt_get(), ccp_postprocess, (void *) inst);
    inst->config.postprocess = true;
}

#if MYNEWT_VAL(CLOCK_CALIBRATION_ENABLED) !=1
/*! 
 * @fn ccp_postprocess(dw1000_dev_instance_t * inst)
 *
 * @brief This serves as a place holder for timescale processing and by default is creates json string for the event
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 *
 * output parameters
 *
 * returns none 
 */
static void ccp_postprocess(struct os_event * ev){
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);
    dw1000_ccp_instance_t * ccp = (dw1000_ccp_instance_t *)ev->ev_arg;
    ccp_frame_t * previous_frame = ccp->frames[(ccp->idx-1)%ccp->nframes]; 
    ccp_frame_t * frame = ccp->frames[(ccp->idx)%ccp->nframes]; 

    printf("{\"utime\":%lu,\"ccp\":[%llu,%llu],\"correction\":%lu,\"seq_num\":%d}\n", 
        os_cputime_ticks_to_usecs(os_cputime_get32()),
        frame->reception_timestamp,
        (uint64_t)((uint64_t)(frame->reception_timestamp) - (uint64_t)(previous_frame->reception_timestamp)) & 0xFFFFFFFFF,
        *(uint32_t *)&frame->correction_factor,
        frame->seq_num
    );
    dw1000_dev_instance_t* inst = ccp->parent;
    inst->control = inst->control_rx_context;
    dw1000_restart_rx(inst,inst->control_rx_context);
}
#endif


/*! 
 * @fn ccp_rx_complete_cb(dw1000_dev_instance_t * inst)
 *
 * @brief Precise timing is achieved using the reception_timestamp and tracking intervals along with  
 * the correction factor. For timescale processing, a postprocessing 
 * callback is placed in the eventq
 * 
 * This callback with FS_XTALT_AUTOTUNE_ENABLED set, uses the RX_TTCKO_ID register to compensate for crystal offset and drift. This is an
 * adaptive loop with a time constant of minutes. By aligning the crystals within the network RF TX power is optimum. 
 * Note: Precise RTLS timing still relies on timescale algorithm. 
 * 
 * The fs_xtalt adjustments align crystals to 1us (1PPM) while timescale processing resolves timestamps to sub 1ns.
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 *
 * output parameters
 *
 * returns none 
 */
static void 
ccp_rx_complete_cb(struct _dw1000_dev_instance_t * inst){
    if (inst->fctrl_array[0] == FCNTL_IEEE_BLINK_CCP_64){
        // CCP Packet Received
        uint64_t clock_master;
        dw1000_read_rx(inst, (uint8_t *) &clock_master, offsetof(ieee_blink_frame_t,long_address), sizeof(uint64_t));
        if(inst->clock_master != clock_master){
            dw1000_restart_rx(inst, inst->control_rx_context);
            return;
       }
    }else{
        //The packet is not intended for the CCP. So pass it on to next item on list
        //If there is not cb in the list go to receive mode again as it is a spurious packet
        //inst->extension_cb = (dw1000_extension_callbacks_t*)(inst->extension_cb->next);
        if(inst->extension_cb->next != NULL){
            inst->extension_cb = inst->extension_cb->next;
            if(inst->extension_cb->rx_complete_cb != NULL)
                inst->extension_cb->rx_complete_cb(inst);
        }
        //For the range service the fctrl is same as FCNTL_IEEE_RANGE_16
        //In he range case the decision is always taken by application
        //So put it back to receive only if the intended packet doesn't match
        //any of the reserved or range packet
        else if(inst->fctrl != FCNTL_IEEE_RANGE_16){
           dw1000_restart_rx(inst, inst->control_rx_context);
        }
        return;
    }
    dw1000_ccp_instance_t * ccp = inst->ccp; 
    ccp_frame_t * frame = ccp->frames[(++ccp->idx)%ccp->nframes];
    
    dw1000_read_rx(inst, frame->array, 0, sizeof(ieee_blink_frame_t));

#if MYNEWT_VAL(ADAPTIVE_TIMESCALE_ENABLED) 
    frame->reception_timestamp = _dw1000_read_rxtime_raw(inst); 
#else
    frame->reception_timestamp = dw1000_read_rxtime(inst);
#endif

    int32_t tracking_interval = (int32_t) dw1000_read_reg(inst, RX_TTCKI_ID, 0, sizeof(int32_t));
    int32_t tracking_offset = (int32_t)(((uint32_t) dw1000_read_reg(inst, RX_TTCKO_ID, 0, sizeof(uint32_t)) & RX_TTCKO_RXTOFS_MASK) << 13) >> 13;
    frame->correction_factor = 1.0f +((float)tracking_offset) / tracking_interval;

    ccp->status.valid |= ccp->idx > ccp->nframes;
    if (ccp->config.postprocess && ccp->status.valid) 
        os_eventq_put(os_eventq_dflt_get(), &ccp->callout_postprocess.c_ev);
   else{
        inst->control = inst->control_rx_context;
        dw1000_restart_rx(inst,inst->control_rx_context);
    }
#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED) 
    if (ccp->config.fs_xtalt_autotune && ccp->status.valid){  
        float fs_xtalt_offset = sosfilt(ccp->xtalt_sos,  1e6 * ((float)tracking_offset) / tracking_interval, g_fs_xtalt_b, g_fs_xtalt_a);  
        if(ccp->xtalt_sos->clk % FS_XTALT_SETTLINGTIME == 0){ 
            int8_t reg = dw1000_read_reg(inst, FS_CTRL_ID, FS_XTALT_OFFSET, sizeof(uint8_t)) & FS_XTALT_MASK;
            int8_t trim_code = (int8_t) roundf(polyval(g_fs_xtalt_poly, fs_xtalt_offset, sizeof(g_fs_xtalt_poly)/sizeof(float)) 
                                - polyval(g_fs_xtalt_poly, 0, sizeof(g_fs_xtalt_poly)/sizeof(float)));
            if(reg - trim_code < 0)
                reg = 0;
            else if(reg - trim_code > FS_XTALT_MASK)
                reg = FS_XTALT_MASK;
            else
                reg = ((reg - trim_code) & FS_XTALT_MASK);

//            printf("{\"utime\":%lu,\"xtalt_trim\": [%d,%d],\"ppm\": %lu}\n", 
//                os_cputime_ticks_to_usecs(os_cputime_get32()),
//                reg,
//                trim_code,
//                *(uint32_t *)&fs_xtalt_offset
//            );
            dw1000_write_reg(inst, FS_CTRL_ID, FS_XTALT_OFFSET,  (3 << 5) | reg, sizeof(uint8_t));
        }
    }
#endif


}



/*! 
 * @fn ccp_tx_complete_cb(dw1000_dev_instance_t * inst)
 *
 * @brief Precise timing is achieved by adding a fixed period to the transmission time of the previous frame. This static 
 * function is called on successful transmission of a CCP packet, and this advances the frame index point. Circular addressing is used 
 * for the frame addressing. The next os_event is scheduled to occur in (MYNEWT_VAL(CCP_PERIOD) - MYNEWT_VAL(CCP_OS_LATENCY)) usec 
 * from now. This provided a context switch guard zone. The assumption is that the underlying OS will have sufficient time start 
 * the call dw1000_set_delay_start within dw1000_ccp_blink. 
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 *
 * output parameters
 *
 * returns none 
 */
static void 
ccp_tx_complete_cb(struct _dw1000_dev_instance_t * inst){
    if (inst->fctrl_array[0] != FCNTL_IEEE_BLINK_CCP_64){
        //The packet is not intended for the CCP. So pass it on to next item on list
        //If there is not cb in the list go to receive mode again as it is a spurious packet
        //inst->extension_cb = (dw1000_extension_callbacks_t*)(inst->extension_cb->next);
        if(inst->extension_cb->next != NULL){
            inst->extension_cb = inst->extension_cb->next;
            if(inst->extension_cb->tx_complete_cb != NULL)
                inst->extension_cb->tx_complete_cb(inst);
        }
        return;
    }
    //Advance frame idx 
    dw1000_ccp_instance_t * ccp = inst->ccp; 
    ccp_frame_t * previous_frame = ccp->frames[(ccp->idx)%ccp->nframes]; 
    ccp_frame_t * frame = ccp->frames[(++ccp->idx)%ccp->nframes];

    printf("{\"utime\":%lu,\"ccp_tx\":[\"%llX\",\"%llX\"],\"seq_num\":%d}\n", 
        os_cputime_ticks_to_usecs(os_cputime_get32()),
        frame->transmission_timestamp,
        (uint64_t)((uint64_t)(frame->transmission_timestamp) - (uint64_t)(previous_frame->transmission_timestamp)) & 0xFFFFFFFFF,
        frame->seq_num
    );
    if (ccp->status.timer_enabled) 
        os_callout_reset(&ccp->callout_timer, OS_TICKS_PER_SEC * (ccp->period - MYNEWT_VAL(OS_LATENCY)) * 1e-6);    
    os_sem_release(&inst->ccp->sem);  
}

/*! 
 * @fn ccp_rx_error_cb(dw1000_dev_instance_t * inst)
 *
 * A place holder for rx_error_cb for ccp.
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 *
 * output parameters
 *
 * returns none 
 */
static void
ccp_rx_error_cb(struct _dw1000_dev_instance_t * inst){
    /* Place holder */
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_CCP_64){
        if(inst->extension_cb->next != NULL){
            inst->extension_cb = inst->extension_cb->next;
            if(inst->extension_cb->rx_error_cb != NULL)
                inst->extension_cb->rx_error_cb(inst);
        }
    }
}

/*! 
 * @fn ccp_tx_error_cb(dw1000_dev_instance_t * inst)
 *
 * A place holder for tx_error_cb for ccp.
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 *
 * output parameters
 *
 * returns none 
 */
static void
ccp_tx_error_cb(struct _dw1000_dev_instance_t * inst){
    /* Place holder */
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_CCP_64){
        if(inst->extension_cb->next != NULL){
            inst->extension_cb = inst->extension_cb->next;
            if(inst->extension_cb->tx_error_cb != NULL)
                inst->extension_cb->tx_error_cb(inst);
        }
    }
}

/*! 
 * @fn ccp_rx_timeout_cb(dw1000_dev_instance_t * inst)
 *
 * A place holder for rx_error_cb for ccp.
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 *
 * output parameters
 *
 * returns none 
 */
static void
ccp_rx_timeout_cb(struct _dw1000_dev_instance_t * inst){
    /* Place holder */
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_CCP_64){
        if(inst->extension_cb->next != NULL){
            inst->extension_cb = inst->extension_cb->next;
            if(inst->extension_cb->rx_timeout_cb != NULL)
                inst->extension_cb->rx_timeout_cb(inst);
        }
    }
}

/*! 
 * @fn dw1000_ccp_blink(dw1000_dev_instance_t * inst, dw1000_ccp_modes_t mode)
 *
 * @brief Start clock calibration packets (CCP) blinks  with a pulse repetition period of MYNEWT_VAL(CCP_PERIOD). 
 * Precise timing is achieved by adding a fixed period to the transmission time of the previous frame. 
 * This removes the need to explicitly read the systime register and the assiciated non-deterministic latencies. 
 * This function is static function for internl use. It will force a Half Period Delay Warning is called at 
 * out of sequence.   
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 * @param mode - dw1000_ccp_modes_t for CCP_BLOCKING, CCP_NONBLOCKING modes.
 *
 * output parameters
 *
 * returns dw1000_ccp_status_t 
 */
static dw1000_ccp_status_t 
dw1000_ccp_blink(struct _dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode){

    os_error_t err = os_sem_pend(&inst->ccp->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    dw1000_ccp_instance_t * ccp = inst->ccp; 
    ccp_frame_t * previous_frame = ccp->frames[(ccp->idx-1)%ccp->nframes];
    ccp_frame_t * frame = ccp->frames[(ccp->idx)%ccp->nframes];

    frame->transmission_timestamp = previous_frame->transmission_timestamp + 2 * ((uint64_t)inst->ccp->period << 15);
    frame->seq_num += inst->ccp->nframes;
    frame->long_address = inst->my_short_address;

    dw1000_write_tx(inst, frame->array, 0, sizeof(ieee_blink_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(ieee_blink_frame_t), 0, true); 
    dw1000_set_wait4resp(inst, false);    
    dw1000_set_delay_start(inst, frame->transmission_timestamp); 
   
    ccp->status.start_tx_error = dw1000_start_tx(inst).start_tx_error;
    if (ccp->status.start_tx_error){
        // Half Period Delay Warning occured try for the next epoch
        // Use seq_num to detect this on receiver size
        previous_frame->transmission_timestamp += ((uint64_t)inst->ccp->period << 15);
        os_sem_release(&inst->ccp->sem);
    }  
    else if(mode == DWT_BLOCKING){
        err = os_sem_pend(&inst->ccp->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions 
        os_sem_release(&inst->ccp->sem);
    }
   return ccp->status;
}


/*! 
 * @fn dw1000_ccp_start(dw1000_dev_instance_t * inst)
 *
 * @brief Start clock calibration packets (CCP) blinks. 
 * With a pulse repetition period of MYNEWT_VAL(CCP_PERIOD).   
 *
 * input parameters
 * @param dw1000_dev_instance_t * 
 *
 * output parameters
 *
 * no return value
 */
void 
dw1000_ccp_start(struct _dw1000_dev_instance_t * inst){
    // Initialise frame timestamp to current time
    dw1000_ccp_instance_t * ccp = inst->ccp; 
    ccp->idx = 0x0;  
    ccp->status.valid = false;
    ccp_frame_t * frame = ccp->frames[(ccp->idx)%ccp->nframes]; 
    frame->transmission_timestamp = dw1000_read_systime(inst);
    ccp_timer_init(inst);
}

/*! 
 * @fn dw1000_ccp_stop(dw1000_dev_instance_t * inst)
 *
 * @brief Stop clock calibration packets (CCP) blinks.   
 *
 * input parameters
 * @param dw1000_dev_instance_t * 
 *
 * output parameters
 *
 * no return value
 */
void 
dw1000_ccp_stop(dw1000_dev_instance_t * inst){
    dw1000_ccp_instance_t * ccp = inst->ccp; 
    os_callout_stop(&ccp->callout_timer);
}

#endif /* MYNEWT_VAL(DW1000_CCP_ENABLED) */
