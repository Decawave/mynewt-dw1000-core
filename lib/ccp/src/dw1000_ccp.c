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
 * @file dw1000_ccp.c
 * @author paul kettle
 * @date 2018
 * @brief clock calibration packets
 *
 * @details This is the ccp base class which utilises the functions to enable/disable the configurations related to ccp. 
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <os/os.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include "bsp/bsp.h"

#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_phy.h>
#include <ccp/dw1000_ccp.h>

#if MYNEWT_VAL(CLOCK_CALIBRATION_ENABLED)
#include <clkcal/clkcal.h>
#endif

//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
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


static bool ccp_rx_complete_cb(struct _dw1000_dev_instance_t * inst);
static bool ccp_tx_complete_cb(struct _dw1000_dev_instance_t * inst);
static bool ccp_rx_error_cb(dw1000_dev_instance_t * inst);
static bool ccp_rx_timeout_cb(dw1000_dev_instance_t * inst);
static bool ccp_tx_error_cb(dw1000_dev_instance_t * inst);
static bool ccp_reset_cb(struct _dw1000_dev_instance_t * inst);

static struct _dw1000_ccp_status_t dw1000_ccp_send(struct _dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode);
static struct _dw1000_ccp_status_t dw1000_ccp_receive(struct _dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode);

static void ccp_tasks_init(struct _dw1000_ccp_instance_t * inst);
static void ccp_postprocess(struct os_event * ev);
static void ccp_timer_irq(void * arg);
static void ccp_master_timer_ev_cb(struct os_event *ev);
static void ccp_slave_timer_ev_cb(struct os_event *ev);

/**
 * API to initiate timer for ccp.
 * 
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @return void 
 */
static void 
ccp_timer_init(struct _dw1000_dev_instance_t * inst, dw1000_ccp_role_t role) {

    dw1000_ccp_instance_t * ccp = inst->ccp; 
    ccp->status.timer_enabled = true;

    os_cputime_timer_init(&ccp->timer, ccp_timer_irq, (void *) inst);

if (role == CCP_ROLE_MASTER) 
    os_callout_init(&ccp->event_cb, &ccp->eventq, ccp_master_timer_ev_cb, (void *) inst);
else if (role == CCP_ROLE_SLAVE)
    os_callout_init(&ccp->event_cb, &ccp->eventq, ccp_slave_timer_ev_cb, (void *) inst);
else
    assert(0);

    os_cputime_timer_relative(&ccp->timer, 0);

}

/**
 * ccp_timer_event is in the interrupr context and schedules and tasks on the event queue
 *
 * @param arg   Pointer to  dw1000_dev_instance_t.
 * @return void 
 */
static void 
ccp_timer_irq(void * arg){
   assert(arg);
 
    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *) arg;
    dw1000_ccp_instance_t * ccp = inst->ccp; 
    os_eventq_put(&ccp->eventq, &ccp->event_cb.c_ev);
}



/** 
 * The OS scheduler is not accurate enough for the timing requirement of an RTLS system.  
 * Instead, the OS is used to schedule events in advance of the actual event.  
 * The DW1000 delay start mechanism then takes care of the actual event. This removes the non-deterministic 
 * latencies of the OS implementation.  
 * 
 * @param ev  Pointer to os_events.
 * @return void
 */
static void 
ccp_master_timer_ev_cb(struct os_event *ev) {
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    DIAGMSG("{\"utime\": %lu,\"msg\": \"ccp_master_timer_ev_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_ccp_instance_t * ccp = inst->ccp; 
    
    dw1000_ccp_send(inst, DWT_BLOCKING);
    hal_timer_start_at(&ccp->timer, ccp->os_epoch
        + os_cputime_usecs_to_ticks((uint32_t)dw1000_dwt_usecs_to_usecs(ccp->period))
    );
}

/** 
 * The OS scheduler is not accurate enough for the timing requirement of an RTLS system.  
 * Instead, the OS is used to schedule events in advance of the actual event.  
 * The DW1000 delay start mechanism then takes care of the actual event. This removes the non-deterministic 
 * latencies of the OS implementation.  
 * 
 * @param ev  Pointer to os_events.
 * @return void
 */
static void 
ccp_slave_timer_ev_cb(struct os_event *ev) {
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    DIAGMSG("{\"utime\": %lu,\"msg\": \"ccp_slave_timer_ev_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_ccp_instance_t * ccp = inst->ccp; 
    
    if(dw1000_ccp_receive(inst, DWT_BLOCKING).start_rx_error){
        // Sync lost, switch to always on mode
        // DIAGMSG("{\"utime\": %lu,\"msg\": \"ccp_slave_timer_ev_cb:start_rx_error\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
        os_error_t err = os_sem_pend(&ccp->sem, OS_TIMEOUT_NEVER);
        assert(err == OS_OK);
   
        dw1000_set_rx_timeout(inst, ccp->period);
        dw1000_start_rx(inst);

        err = os_sem_pend(&ccp->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions 
        os_sem_release(&ccp->sem);
    }
    // Schedule next event
    hal_timer_start_at(&ccp->timer, ccp->os_epoch 
        + os_cputime_usecs_to_ticks(
            - MYNEWT_VAL(OS_LATENCY)
            + (uint32_t)dw1000_dwt_usecs_to_usecs(ccp->period)
            - dw1000_phy_frame_duration(&inst->attrib, sizeof(ieee_blink_frame_t))
            )
        );
}


static void 
ccp_task(void *arg)
{
    dw1000_ccp_instance_t * inst = arg;
    while (1) {
        os_eventq_run(&inst->eventq);
    }
}

/**
 * The default eventq is used. 
 * 
 * @param inst Pointer to dw1000_dev_instance_t * 
 * @return void
 */
static void 
ccp_tasks_init(struct _dw1000_ccp_instance_t * inst)
{
    /* Check if the tasks are already initiated */
    if (!os_eventq_inited(&inst->eventq))
    {
        /* Use a dedicate event queue for tdma events */
        os_eventq_init(&inst->eventq);
        os_task_init(&inst->task_str, "dw1000_ccp",
                     ccp_task,
                     (void *) inst,
                     inst->task_prio, OS_WAIT_FOREVER,
                     inst->task_stack,
                     DW1000_DEV_TASK_STACK_SZ);
    }       
}


/**
 * Precise timing is achieved by adding a fixed period to the transmission time of the previous frame. 
 * This approach solves the non-deterministic latencies caused by the OS. The OS, however, is used to schedule 
 * the next transmission event, but the DW1000 controls the actual next transmission time using the dw1000_set_delay_start.
 * This function allocates all the required resources. In the case of a large scale deployment multiple instances 
 * can be uses to track multiple clock domains. 
 * 
 * @param inst     Pointer to dw1000_dev_instance_t
 * @param nframes  Nominally set to 2 frames for the simple use case. But depending on the interpolation 
 * algorithm this should be set accordingly. For example, four frames are required or bicubic interpolation. 
 * @param clock_master  UUID address of the system clock_master all other masters are rejected.  
 *
 * @return dw1000_ccp_instance_t * 
 */
dw1000_ccp_instance_t * 
dw1000_ccp_init(struct _dw1000_dev_instance_t * inst, uint16_t nframes, uint64_t clock_master){
    assert(inst);
    assert(nframes > 1);
    
    DIAGMSG("{\"utime\": %lu,\"msg\": \"dw1000_ccp_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

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
            ccp->frames[i]->seq_num = i;
        }

        ccp->parent = inst;
        inst->ccp = ccp;
        ccp->task_prio = inst->task_prio + 1;

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

    dw1000_ccp_set_postprocess(inst->ccp, &ccp_postprocess);    // Using default process

#if MYNEWT_VAL(CLOCK_CALIBRATION_ENABLED)
    inst->ccp->clkcal = clkcal_init(NULL, inst->ccp);           // Using clkcal process
#endif

    dw1000_extension_callbacks_t ccp_cbs = {
        .id = DW1000_CCP,
        .tx_complete_cb = ccp_tx_complete_cb,
        .rx_complete_cb = ccp_rx_complete_cb,
        .rx_timeout_cb = ccp_rx_timeout_cb,
        .rx_error_cb = ccp_rx_error_cb,
        .tx_error_cb = ccp_tx_error_cb,
        .reset_cb = ccp_reset_cb
    };

    dw1000_ccp_set_ext_callbacks(inst, ccp_cbs);
    ccp_tasks_init(inst->ccp);
    inst->ccp->os_epoch = os_cputime_get32();

#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED) 
    inst->ccp->xtalt_sos = sosfilt_init(NULL, sizeof(g_fs_xtalt_b)/sizeof(float)/BIQUAD_N);
#endif
    inst->ccp->status.initialized = 1;
    
    return inst->ccp;
}

/** 
 * Deconstructor
 * 
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @return void
 */
void 
dw1000_ccp_free(dw1000_ccp_instance_t * inst){
    assert(inst);  
    os_sem_release(&inst->sem);
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

/**
 * API to register extension callbacks in ccp.
 *
 * @param inst     Pointer to dw1000_dev_instance_t
 * @param ccp_cbs  Callbacks of ccp.
 * @return void
 */
void dw1000_ccp_set_ext_callbacks(dw1000_dev_instance_t * inst, dw1000_extension_callbacks_t ccp_cbs){
    ccp_cbs.id = DW1000_CCP;
    dw1000_add_extension_callbacks(inst , ccp_cbs);
}


/** 
 * API that overrides the default post-processing behaviors, replacing the JSON stream with an alternative 
 * or an advanced timescale processing algorithm.
 * 
 * @param inst              Pointer to dw1000_dev_instance_t. 
 * @param ccp_postprocess   Pointer to os_events.
 * @return void
 */
void 
dw1000_ccp_set_postprocess(dw1000_ccp_instance_t * inst, os_event_fn * postprocess){
    os_callout_init(&inst->callout_postprocess, os_eventq_dflt_get(), postprocess, (void *) inst);
    inst->config.postprocess = true;
}


/** 
 * API that serves as a place holder for timescale processing and by default is creates json string for the event.
 *
 * @param ev   pointer to os_events.
 * @return void 
 */
static void 
ccp_postprocess(struct os_event * ev){
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);
    dw1000_ccp_instance_t * ccp = (dw1000_ccp_instance_t *)ev->ev_arg;

    ccp_frame_t * previous_frame = ccp->frames[(ccp->idx-2)%ccp->nframes]; 
    ccp_frame_t * frame = ccp->frames[(ccp->idx-1)%ccp->nframes]; 

#if  MYNEWT_VAL(DW1000_CCP_MASTER_ENABLED) 
    uint64_t delta = (frame->transmission_timestamp - previous_frame->transmission_timestamp);
#else 
    uint64_t delta = (frame->reception_timestamp - previous_frame->reception_timestamp);
#endif
    delta = delta & ((uint64_t)1<<63)?delta & 0xFFFFFFFFFF :delta;

    printf("{\"utime\": %lu,\"ccp\":[%llX,%llX],\"correction\":%lu,\"seq_num\":%d}\n", 
        os_cputime_ticks_to_usecs(os_cputime_get32()),   
#if  MYNEWT_VAL(DW1000_CCP_MASTER_ENABLED) 
        frame->transmission_timestamp,
#else
        frame->reception_timestamp,       
#endif
        delta,
        *(uint32_t *)&frame->correction_factor,
        frame->seq_num
    );
    dw1000_dev_instance_t* inst = ccp->parent;
    inst->control = inst->control_rx_context;
    dw1000_restart_rx(inst,inst->control_rx_context);
}



/** 
 * Precise timing is achieved using the reception_timestamp and tracking intervals along with  
 * the correction factor. For timescale processing, a postprocessing  callback is placed in the eventq.
 * This callback with FS_XTALT_AUTOTUNE_ENABLED set, uses the RX_TTCKO_ID register to compensate for crystal offset and drift. This is an
 * adaptive loop with a time constant of minutes. By aligning the crystals within the network RF TX power is optimum. 
 * Note: Precise RTLS timing still relies on timescale algorithm. 
 * 
 * The fs_xtalt adjustments align crystals to 1us (1PPM) while timescale processing resolves timestamps to sub 1ns.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.  
 * @return void 
 */
static bool 
ccp_rx_complete_cb(struct _dw1000_dev_instance_t * inst){

    if (inst->fctrl_array[0] == FCNTL_IEEE_BLINK_CCP_64){
        // CCP Packet Received
        uint64_t clock_master;
        dw1000_read_rx(inst, (uint8_t *) &clock_master, offsetof(ieee_blink_frame_t,long_address), sizeof(uint64_t));
        if(inst->clock_master != clock_master)
            return false;
    }else {
        return false;
    }
    
    dw1000_ccp_instance_t * ccp = inst->ccp; 
    ccp_frame_t * frame = ccp->frames[(ccp->idx++)%ccp->nframes];
    ccp->os_epoch = os_cputime_get32();
 
    DIAGMSG("{\"utime\": %lu,\"msg\": \"ccp_rx_complete_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
  
    dw1000_read_rx(inst, frame->array, 0, sizeof(ieee_blink_frame_t));

#if MYNEWT_VAL(ADAPTIVE_TIMESCALE_ENABLED) 
    ccp->epoch = frame->reception_timestamp = _dw1000_read_rxtime_raw(inst); 
#else
    ccp->epoch = frame->reception_timestamp = dw1000_read_rxtime(inst);
#endif

    int32_t tracking_interval = (int32_t) dw1000_read_reg(inst, RX_TTCKI_ID, 0, sizeof(int32_t));
    int32_t tracking_offset = (int32_t)(((uint32_t) dw1000_read_reg(inst, RX_TTCKO_ID, 0, sizeof(uint32_t)) & RX_TTCKO_RXTOFS_MASK) << 13) >> 13;
    frame->correction_factor = 1.0f +((float)tracking_offset) / tracking_interval;
    ccp->status.valid |= ccp->idx > 1;

    if (ccp->config.postprocess && ccp->status.valid) 
        os_eventq_put(os_eventq_dflt_get(), &ccp->callout_postprocess.c_ev);
    
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
            dw1000_write_reg(inst, FS_CTRL_ID, FS_XTALT_OFFSET,  (3 << 5) | reg, sizeof(uint8_t));
        }
    }
#endif
    os_sem_release(&inst->ccp->sem);
    return true;
}

/** 
 * Precise timing is achieved by adding a fixed period to the transmission time of the previous frame. This static 
 * function is called on successful transmission of a CCP packet, and this advances the frame index point. Circular addressing is used 
 * for the frame addressing. The next os_event is scheduled to occur in (MYNEWT_VAL(CCP_PERIOD) - MYNEWT_VAL(CCP_OS_LATENCY)) usec 
 * from now. This provided a context switch guard zone. The assumption is that the underlying OS will have sufficient time start 
 * the call dw1000_set_delay_start within dw1000_ccp_blink. 
 *
 * @param inst    Pointer to dw1000_dev_instance_t  
 * @return bool
 */
static bool 
ccp_tx_complete_cb(struct _dw1000_dev_instance_t * inst){

    if (inst->fctrl_array[0] != FCNTL_IEEE_BLINK_CCP_64)
        return false;

    dw1000_ccp_instance_t * ccp = inst->ccp; 
    ccp_frame_t * frame = ccp->frames[(ccp->idx++)%ccp->nframes];
    ccp->os_epoch = os_cputime_get32();

    ccp->epoch = frame->transmission_timestamp = dw1000_read_txtime(inst); 

    DIAGMSG("{\"utime\": %lu,\"msg\": \"ccp_tx_complete_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
    
    if (ccp->status.timer_enabled){
        hal_timer_start_at(&ccp->timer, ccp->os_epoch 
            - os_cputime_usecs_to_ticks(MYNEWT_VAL(OS_LATENCY)) 
            + os_cputime_usecs_to_ticks(dw1000_dwt_usecs_to_usecs(ccp->period))
        );
    }

    ccp->status.valid |= ccp->idx > 1;

    // Postprocess for tx_complete is used to generate tdma events on the clock master node. 
    if (ccp->config.postprocess && ccp->status.valid) 
        os_eventq_put(os_eventq_dflt_get(), &ccp->callout_postprocess.c_ev);
    
    os_sem_release(&inst->ccp->sem);  
    return true;
}

/** 
 * API for rx_error_cb of ccp.
 *
 * @param inst   pointer to dw1000_dev_instance_t.  
 * @return void
 */
static bool
ccp_rx_error_cb(struct _dw1000_dev_instance_t * inst){

    if(os_sem_get_count(&inst->ccp->sem) == 0){
        printf("{\"utime\": %lu,\"log\": \"ccp_rx_error_cb\",\"%s\":%d}\n",os_cputime_ticks_to_usecs(os_cputime_get32()),__FILE__, __LINE__); 
        os_sem_release(&inst->ccp->sem);  
	    return false; 
    }
    return false;
}

/** 
 * API for tx_error_cb of ccp.
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 * @return bool 
 */
static bool
ccp_tx_error_cb(struct _dw1000_dev_instance_t * inst){

    if(inst->fctrl_array[0] == FCNTL_IEEE_BLINK_CCP_64){
        printf("{\"utime\": %lu,\"log\": \"ccp_tx_error_cb\",\"%s\":%d}\n",os_cputime_ticks_to_usecs(os_cputime_get32()),__FILE__, __LINE__); 
        os_sem_release(&inst->ccp->sem);  
        return true;
    }
    return false;
}

/** 
 * API for rx_timeout_cb of ccp.
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 * @return void 
 */
static bool
ccp_rx_timeout_cb(struct _dw1000_dev_instance_t * inst){
    /* Place holder */
    if (os_sem_get_count(&inst->ccp->sem) == 0){
        printf("{\"utime\": %lu,\"log\": \"ccp_rx_timeout_cb\",\"%s\":%d}\n",os_cputime_ticks_to_usecs(os_cputime_get32()),__FILE__, __LINE__); 
        os_sem_release(&inst->ccp->sem);  
        return true;
    }
    return false;
}


/** 
 * API for ccp_reset_cb of ccp.
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 * @return void 
 */
static bool
ccp_reset_cb(struct _dw1000_dev_instance_t * inst){
    /* Place holder */
        os_sem_release(&inst->ccp->sem);  
    return true;
}
 

/**
 * @fn dw1000_ccp_send(dw1000_dev_instance_t * inst, dw1000_ccp_modes_t mode)
 * API that start clock calibration packets (CCP) blinks  with a pulse repetition period of MYNEWT_VAL(CCP_PERIOD). 
 * Precise timing is achieved by adding a fixed period to the transmission time of the previous frame. 
 * This removes the need to explicitly read the systime register and the assiciated non-deterministic latencies. 
 * This function is static function for internl use. It will force a Half Period Delay Warning is called at 
 * out of sequence.   
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 * @param mode   dw1000_ccp_modes_t for CCP_BLOCKING, CCP_NON_BLOCKING modes.
 * @return dw1000_ccp_status_t 
 */
static dw1000_ccp_status_t 
dw1000_ccp_send(struct _dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode){

    DIAGMSG("{\"utime\": %lu,\"msg\": \"dw1000_ccp_send\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
    dw1000_ccp_instance_t * ccp = inst->ccp; 

    os_error_t err = os_sem_pend(&ccp->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    ccp_frame_t * previous_frame = ccp->frames[(ccp->idx-1)%ccp->nframes];
    ccp_frame_t * frame = ccp->frames[(ccp->idx)%ccp->nframes];

    frame->transmission_timestamp = previous_frame->transmission_timestamp + ((uint64_t)inst->ccp->period << 16);
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
        DIAGMSG("{\"utime\": %lu,\"msg\": \"dw1000_ccp_send:start_tx_error\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
        previous_frame->transmission_timestamp += ((uint64_t)inst->ccp->period << 16);
        os_sem_release(&ccp->sem);
    }else if(mode == DWT_BLOCKING){
        err = os_sem_pend(&ccp->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions 
        os_sem_release(&ccp->sem);
    }
   return ccp->status;
}

/*! 
 * @fn dw1000_ccp_receive(dw1000_dev_instance_t * inst, dw1000_ccp_modes_t mode)
 *
 * @brief Explisit entry function for reveicing a ccp frame.  
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
dw1000_ccp_receive(struct _dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode){

    DIAGMSG("{\"utime\": %lu,\"msg\": \"dw1000_ccp_receive\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
    dw1000_ccp_instance_t * ccp = inst->ccp; 

    os_error_t err = os_sem_pend(&ccp->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    uint64_t dx_time = ccp->epoch 
            + ((uint64_t)inst->ccp->period << 16) 
            - ((uint64_t)ceilf(dw1000_usecs_to_dwt_usecs(dw1000_phy_SHR_duration(&inst->attrib))) << 16);

    uint16_t timeout = dw1000_phy_frame_duration(&inst->attrib, sizeof(ieee_blink_frame_t)) + MYNEWT_VAL(XTALT_GUARD);
                       
    dw1000_set_rx_timeout(inst, timeout); 
    dw1000_set_delay_start(inst, dx_time);
    ccp->status.start_rx_error = dw1000_start_rx(inst).start_rx_error;
    if (ccp->status.start_rx_error){
        // Half Period Delay Warning occured try for the next epoch
        // Use seq_num to detect this on receiver size
        // DIAGMSG("{\"utime\": %lu,\"msg\": \"dw1000_ccp_receive:start_rx_error\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
        os_sem_release(&ccp->sem);
    }else if(mode == DWT_BLOCKING){
        err = os_sem_pend(&ccp->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions 
        os_sem_release(&ccp->sem);
    }
   return ccp->status;
}


/** 
 * API to start clock calibration packets (CCP) blinks. 
 * With a pulse repetition period of MYNEWT_VAL(CCP_PERIOD).   
 *
 * @param inst   Pointer to dw1000_dev_instance_t. 
 * @return void
 */
void 
dw1000_ccp_start(struct _dw1000_dev_instance_t * inst, dw1000_ccp_role_t role){
    // Initialise frame timestamp to current time
    DIAGMSG("{\"utime\": %lu,\"msg\": \"dw1000_ccp_start\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
    dw1000_ccp_instance_t * ccp = inst->ccp; 
    ccp->idx = 0x0;  
    ccp->status.valid = false;
    ccp_frame_t * frame = ccp->frames[(ccp->idx)%ccp->nframes]; 

    if (role == CCP_ROLE_MASTER)
        frame->transmission_timestamp = dw1000_read_systime(inst);
    else if (role == CCP_ROLE_SLAVE)
        frame->reception_timestamp = dw1000_read_systime(inst);
    else
        assert(0);

    ccp_timer_init(inst, role);
}

/**
 * API to stop clock calibration packets (CCP) blinks.   
 *
 * @param inst   Pointer to  dw1000_dev_instance_t. 
 * @return void
 */
void 
dw1000_ccp_stop(dw1000_dev_instance_t * inst){
    dw1000_ccp_instance_t * ccp = inst->ccp; 
   os_cputime_timer_stop(&ccp->timer);
}

