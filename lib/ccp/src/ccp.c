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
#include <stats/stats.h>
#include <bsp/bsp.h>

#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_hal.h>
#if MYNEWT_VAL(CCP_ENABLED)
#include <ccp/ccp.h>
#endif
#if MYNEWT_VAL(WCS_ENABLED)
#include <wcs/wcs.h>
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


STATS_NAME_START(ccp_stat_section)
    STATS_NAME(ccp_stat_section, master_cnt)
    STATS_NAME(ccp_stat_section, slave_cnt)
    STATS_NAME(ccp_stat_section, send)
    STATS_NAME(ccp_stat_section, listen)
    STATS_NAME(ccp_stat_section, slave_cnt)
    STATS_NAME(ccp_stat_section, tx_complete)
    STATS_NAME(ccp_stat_section, rx_complete)
    STATS_NAME(ccp_stat_section, rx_relayed)
    STATS_NAME(ccp_stat_section, rx_unsolicited)
    STATS_NAME(ccp_stat_section, rx_error)
    STATS_NAME(ccp_stat_section, tx_start_error)
    STATS_NAME(ccp_stat_section, tx_relay_error)
    STATS_NAME(ccp_stat_section, tx_relay_ok)
    STATS_NAME(ccp_stat_section, rx_timeout)
    STATS_NAME(ccp_stat_section, reset)
STATS_NAME_END(ccp_stat_section)


static bool rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool ccp_tx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool ccp_rx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool ccp_rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool ccp_tx_error_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool ccp_reset_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);

static struct _dw1000_ccp_status_t dw1000_ccp_send(struct _dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode);
static struct _dw1000_ccp_status_t dw1000_ccp_listen(struct _dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode);

static void ccp_tasks_init(struct _dw1000_ccp_instance_t * inst);
static void ccp_timer_irq(void * arg);
static void ccp_master_timer_ev_cb(struct os_event *ev);
static void ccp_slave_timer_ev_cb(struct os_event *ev);

#if !MYNEWT_VAL(WCS_ENABLED)
static void ccp_postprocess(struct os_event * ev);
#endif

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
    else
        os_callout_init(&ccp->event_cb, &ccp->eventq, ccp_slave_timer_ev_cb, (void *) inst);
    
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

    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_ccp_instance_t * ccp = inst->ccp; 
    
    STATS_INC(inst->ccp->stat, master_cnt);

    if (dw1000_ccp_send(inst, DWT_BLOCKING).start_tx_error){
        hal_timer_start_at(&ccp->timer, ccp->os_epoch
            + os_cputime_usecs_to_ticks((uint32_t)dw1000_dwt_usecs_to_usecs(ccp->period) << 1)
        );    
    }else{
        hal_timer_start_at(&ccp->timer, ccp->os_epoch
            + os_cputime_usecs_to_ticks((uint32_t)dw1000_dwt_usecs_to_usecs(ccp->period))
        );
    }
}

/** 
 * Help function to calculate the delay between cascading ccp relays
 * 
 * @param inst Pointer to dw1000_dev_instance_t * 
 * @param rx_slot 0 for master, and increasing
 * @param my_slot my_slot should be inst->slot_id - 1, master having slot_id=1 usually
 * @return void
 */
static uint32_t
usecs_to_response(dw1000_dev_instance_t * inst, int rx_slot, int my_slot)
{
    uint32_t ccp_duration = dw1000_phy_frame_duration(&inst->attrib, sizeof(ccp_blink_frame_t));
    uint32_t ret = ((uint32_t)inst->ccp->config.tx_guard_dly + ccp_duration);
    /* Master has slot 0 */
    if (rx_slot == 0) {
        ret *= my_slot - 1;
        ret += inst->ccp->config.tx_holdoff_dly;
    } else {
        ret *= my_slot - rx_slot;
    }
    return ret;
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

    DIAGMSG("{\"utime\": %lu,\"msg\": \"ccp_slave_timer_ev_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_ccp_instance_t * ccp = inst->ccp; 

    STATS_INC(inst->ccp->stat, slave_cnt);

    uint64_t dx_time = ccp->epoch 
            + ((uint64_t)inst->ccp->period << 16) 
            - ((uint64_t)ceilf(dw1000_usecs_to_dwt_usecs(dw1000_phy_SHR_duration(&inst->attrib))) << 16);

    uint16_t timeout = dw1000_phy_frame_duration(&inst->attrib, sizeof(ccp_blink_frame_t)) 
                        + MYNEWT_VAL(XTALT_GUARD);
                        
#if MYNEWT_VAL(CCP_NUM_RELAYING_ANCHORS) != 0
    /* Adjust timeout if we're using cascading ccp in anchors */
    timeout += usecs_to_response(inst, 0, MYNEWT_VAL(CCP_NUM_RELAYING_ANCHORS));
#endif
    dw1000_set_rx_timeout(inst, timeout); 
    dw1000_set_delay_start(inst, dx_time);

    dw1000_ccp_status_t status = dw1000_ccp_listen(inst, DWT_BLOCKING);
    if(status.start_rx_error){
        /* Sync lost, set a long rx timeout */
        dw1000_set_rx_timeout(inst, (uint16_t) 0xffff);
        dw1000_ccp_listen(inst, DWT_BLOCKING);
    }
    // Schedule event
    hal_timer_start_at(&ccp->timer, ccp->os_epoch 
        + os_cputime_usecs_to_ticks(
            - MYNEWT_VAL(OS_LATENCY)
            + (uint32_t)dw1000_dwt_usecs_to_usecs(ccp->period)
            - dw1000_phy_frame_duration(&inst->attrib, sizeof(ccp_blink_frame_t))
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
dw1000_ccp_init(struct _dw1000_dev_instance_t * inst, uint16_t nframes, uint64_t uuid){
    assert(inst);
    assert(nframes > 1);
    
    if (inst->ccp == NULL ) {
        dw1000_ccp_instance_t * ccp = (dw1000_ccp_instance_t *) malloc(sizeof(dw1000_ccp_instance_t) + nframes * sizeof(ccp_frame_t *)); 
        assert(ccp);
        memset(ccp, 0, sizeof(dw1000_ccp_instance_t));
        ccp->status.selfmalloc = 1;
        ccp->nframes = nframes; 
        ccp_frame_t ccp_default = {
            .fctrl = FCNTL_IEEE_BLINK_CCP_64,    // frame control (FCNTL_IEEE_BLINK_64 to indicate a data frame using 16-bit addressing).
            .seq_num = 0xFF
        };
        
        for (uint16_t i = 0; i < ccp->nframes; i++){
            ccp->frames[i] = (ccp_frame_t *) malloc(sizeof(ccp_frame_t)); 
            assert(ccp->frames[i]);
            memcpy(ccp->frames[i], &ccp_default, sizeof(ccp_frame_t));
            ccp->frames[i]->seq_num = 0;
        }

        ccp->parent = inst;
        inst->ccp = ccp;
        ccp->task_prio = inst->task_prio - 0x4;

    }else{
        assert(inst->ccp->nframes == nframes);
    }
    inst->ccp->period = MYNEWT_VAL(CCP_PERIOD);
    inst->ccp->config = (dw1000_ccp_config_t){
        .postprocess = false,
#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED)
        .fs_xtalt_autotune = true,
#endif
        .tx_holdoff_dly = 0x300,
        .tx_guard_dly = 0x10,
    };
    inst->ccp->uuid = uuid;

    os_error_t err = os_sem_init(&inst->ccp->sem, 0x1); 
    assert(err == OS_OK);

#if MYNEWT_VAL(WCS_ENABLED)
    inst->ccp->wcs = wcs_init(NULL, inst->ccp);                 // Using wcs process
    dw1000_ccp_set_postprocess(inst->ccp, &wcs_update_cb);      // Using default process
#else
    dw1000_ccp_set_postprocess(inst->ccp, &ccp_postprocess);    // Using default process
#endif

    inst->ccp->cbs = (dw1000_mac_interface_t){
        .id = DW1000_CCP,
        .tx_complete_cb = ccp_tx_complete_cb,
        .rx_complete_cb = rx_complete_cb,
        .rx_timeout_cb = ccp_rx_timeout_cb,
        .rx_error_cb = ccp_rx_error_cb,
        .tx_error_cb = ccp_tx_error_cb,
        .reset_cb = ccp_reset_cb
    };
    dw1000_mac_append_interface(inst, &inst->ccp->cbs);

    ccp_tasks_init(inst->ccp);
    inst->ccp->os_epoch = os_cputime_get32();

#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED) 
    inst->ccp->xtalt_sos = sosfilt_init(NULL, sizeof(g_fs_xtalt_b)/sizeof(float)/BIQUAD_N);
#endif
    inst->ccp->status.initialized = 1;

    int rc = stats_init(
                STATS_HDR(inst->ccp->stat),
                STATS_SIZE_INIT_PARMS(inst->ccp->stat, STATS_SIZE_32),
                STATS_NAME_INIT_PARMS(ccp_stat_section)
            );
    assert(rc == 0);

#if  MYNEWT_VAL(DW1000_DEVICE_0) && !MYNEWT_VAL(DW1000_DEVICE_1)
    rc = stats_register("ccp", STATS_HDR(inst->ccp->stat));
#elif  MYNEWT_VAL(DW1000_DEVICE_0) && MYNEWT_VAL(DW1000_DEVICE_1)
    if (inst->idx == 0)
        rc |= stats_register("ccp0", STATS_HDR(inst->ccp->stat));
    else
        rc |= stats_register("ccp1", STATS_HDR(inst->ccp->stat));
#endif
    assert(rc == 0);

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

#if MYNEWT_VAL(WCS_ENABLED)    
    wcs_free(inst->wcs); 
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
 * API to initialise the package, only one ccp service required in the system.
 *
 * @return void
 */

void ccp_pkg_init(void){

    printf("{\"utime\": %lu,\"msg\": \"ccp_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

#if MYNEWT_VAL(DW1000_DEVICE_0)
    dw1000_ccp_init(hal_dw1000_inst(0), 2, MYNEWT_VAL(UUID_CCP_MASTER));
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    dw1000_ccp_init(hal_dw1000_inst(1), 2, MYNEWT_VAL(UUID_CCP_MASTER));
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    dw1000_ccp_init(hal_dw1000_inst(2), 2, MYNEWT_VAL(UUID_CCP_MASTER));
#endif

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
//    os_callout_init(&inst->callout_postprocess, &inst->eventq,  postprocess, (void *) inst);
    inst->config.postprocess = true;
}

#if !MYNEWT_VAL(WCS_ENABLED)
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

    ccp_frame_t * previous_frame = ccp->frames[(uint16_t)(ccp->idx-1)%ccp->nframes]; 
    ccp_frame_t * frame = ccp->frames[(ccp->idx)%ccp->nframes]; 
    uint64_t delta = 0;

    if (ccp->config.role == CCP_ROLE_MASTER){
        delta = (frame->transmission_timestamp - previous_frame->transmission_timestamp);
    } else {
        delta = (frame->reception_timestamp - previous_frame->reception_timestamp);
    }
    delta = delta & ((uint64_t)1<<63)?delta & 0xFFFFFFFFFF :delta;

#if MYNEWT_VAL(CCP_VERBOSE)
    float clock_offset = dw1000_calc_clock_offset_ratio(ccp->parent, frame->carrier_integrator);
        os_cputime_ticks_to_usecs(os_cputime_get32()),   
        frame->transmission_timestamp,
        delta,
        *(uint32_t *)&clock_offset,
        frame->seq_num
    );
#endif
}
#endif


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
rx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    if (inst->fctrl_array[0] != FCNTL_IEEE_BLINK_CCP_64){     
        if(os_sem_get_count(&inst->ccp->sem) == 0){
            dw1000_set_rx_timeout(inst, (uint16_t) 0xffff);
            return true;
        }
        return false;
    }
 
    if(os_sem_get_count(&inst->ccp->sem) != 0){ 
        //unsolicited inbound
        STATS_INC(inst->ccp->stat, rx_unsolicited);
        return false;
    }

    if (inst->ccp->config.role == CCP_ROLE_MASTER) {
        return true;
    }
    DIAGMSG("{\"utime\": %lu,\"msg\": \"ccp:rx_complete_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
    
    dw1000_ccp_instance_t * ccp = inst->ccp; 
    ccp_frame_t * frame = ccp->frames[(ccp->idx+1)%ccp->nframes];  // speculative frame advance

    if (inst->frame_len >= sizeof(ccp_blink_frame_t) && inst->frame_len <= sizeof(frame->array)) 
        memcpy(frame->array, inst->rxbuf, sizeof(ccp_blink_frame_t));
    else 
        return false;

    /* Mask off the last 8 bits and compare to our ccp->uuid master id */
    if((inst->ccp->uuid & 0xffffffffffffff00UL) != (frame->long_address & 0xffffffffffffff00UL)) {
        return false;
    }

    if (inst->status.lde_error) 
        return false;

    /* A good ccp packet has been received, stop the receiver */
    dw1000_stop_rx(inst); //Prevent timeout event
    
    ccp->idx++; // confirmed frame advance  

    ccp->os_epoch = os_cputime_get32();
    STATS_INC(inst->ccp->stat, rx_complete);

    ccp->epoch_master = frame->transmission_timestamp;
    ccp->epoch = frame->reception_timestamp = inst->rxtimestamp;
    ccp->period = frame->transmission_interval;
    frame->carrier_integrator = inst->carrier_integrator;
    ccp->status.valid |= ccp->idx > 1;

    /* Compensate if not receiving the master ccp packet directly */
    int rx_slot = (frame->long_address & 0xff);
    if (rx_slot != 0x00) {
        STATS_INC(inst->ccp->stat, rx_relayed);
        /* Assume ccp intervals are a multiple of 0x10000 us */
        uint32_t master_interval = ((frame->transmission_interval/0x10000+1)*0x10000);
        ccp->epoch_master -= (master_interval - frame->transmission_interval) << 16;
        ccp->epoch -= (master_interval - frame->transmission_interval) << 16;
        ccp->os_epoch -= os_cputime_usecs_to_ticks(master_interval - frame->transmission_interval);
    }

    /* Cascade relay of ccp packet */
    if (ccp->config.role == CCP_ROLE_RELAY && ccp->status.valid &&
        rx_slot < (inst->slot_id-1) && inst->slot_id != 0xffff) {
        ccp_frame_t tx_frame;
        memcpy(tx_frame.array, frame->array, sizeof(ccp_frame_t));
        uint64_t tx_timestamp = frame->reception_timestamp + (((uint64_t)usecs_to_response(inst, rx_slot, inst->slot_id-1))<<16);
        tx_timestamp &= 0x0FFFFFFFFFFUL;
        dw1000_set_delay_start(inst, tx_timestamp);

        /* Need to add antenna delay and tof compensation */
        tx_timestamp += inst->tx_antenna_delay + inst->ccp->config.tof_compensation;

#if MYNEWT_VAL(WCS_ENABLED)
        tx_frame.transmission_timestamp = wcs_local_to_master(inst, tx_timestamp);
#else
        tx_frame.transmission_timestamp = frame->transmission_timestamp + tx_timestamp - frame->reception_timestamp;
#endif

        tx_frame.long_address = inst->ccp->uuid | (inst->slot_id-1);
        tx_frame.transmission_interval = frame->transmission_interval - ((tx_frame.transmission_timestamp - frame->transmission_timestamp)>>16);

        dw1000_write_tx(inst, tx_frame.array, 0, sizeof(ccp_blink_frame_t));
        dw1000_write_tx_fctrl(inst, sizeof(ccp_blink_frame_t), 0, true); 
        ccp->status.start_tx_error = dw1000_start_tx(inst).start_tx_error;
        if (ccp->status.start_tx_error){
            STATS_INC(inst->ccp->stat, tx_relay_error);
        } else {
            STATS_INC(inst->ccp->stat, tx_relay_ok);
        }
    }

    if (ccp->config.postprocess && ccp->status.valid) 
        // os_eventq_put(&ccp->eventq, &ccp->callout_postprocess.c_ev);
        os_eventq_put(os_eventq_dflt_get(), &ccp->callout_postprocess.c_ev);
    
#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED) 
    if (ccp->config.fs_xtalt_autotune && ccp->status.valid){
//        float fs_xtalt_offset = sosfilt(ccp->xtalt_sos,  1e6 * ((float)tracking_offset) / tracking_interval, g_fs_xtalt_b, g_fs_xtalt_a);  
        float fs_xtalt_offset = sosfilt(ccp->xtalt_sos,  -1e6 * ccp->wcs->skew, g_fs_xtalt_b, g_fs_xtalt_a);
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
    return false;
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
ccp_tx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){

    if (inst->fctrl_array[0] != FCNTL_IEEE_BLINK_CCP_64)
        return false;

    STATS_INC(inst->ccp->stat, tx_complete);
    if (inst->ccp->config.role != CCP_ROLE_MASTER)
        return false;

    dw1000_ccp_instance_t * ccp = inst->ccp; 
    ccp_frame_t * frame = ccp->frames[(++ccp->idx)%ccp->nframes];
    
    ccp->os_epoch = os_cputime_get32();
    ccp->epoch = frame->transmission_timestamp = dw1000_read_txrawst(inst); 
    ccp->epoch_master = frame->transmission_timestamp;
    ccp->period = frame->transmission_interval;

    if (ccp->status.timer_enabled){
        hal_timer_start_at(&ccp->timer, ccp->os_epoch 
            - os_cputime_usecs_to_ticks(MYNEWT_VAL(OS_LATENCY)) 
            + os_cputime_usecs_to_ticks(dw1000_dwt_usecs_to_usecs(ccp->period))
        );
    }
    ccp->status.valid |= ccp->idx > 1;
    // Postprocess for tx_complete is used to generate tdma events on the clock master node. 
    if (ccp->config.postprocess && ccp->status.valid) 
        //os_eventq_put(&ccp->eventq, &ccp->callout_postprocess.c_ev);
        os_eventq_put(os_eventq_dflt_get(), &ccp->callout_postprocess.c_ev);
       
    if(os_sem_get_count(&inst->ccp->sem) == 0){
        os_error_t err = os_sem_release(&inst->ccp->sem);
        assert(err == OS_OK);
    }
    return false;
}

/** 
 * API for rx_error_cb of ccp.
 *
 * @param inst   pointer to dw1000_dev_instance_t.  
 * @return void
 */
static bool
ccp_rx_error_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){

    dw1000_ccp_instance_t * ccp = inst->ccp; 
    STATS_INC(inst->ccp->stat, rx_error);

    if (ccp->config.role == CCP_ROLE_MASTER) 
        return false;

    // Release semaphore if rxauto enable is not set. 
    if(inst->config.rxauto_enable)
        return false;
        
    else if(os_sem_get_count(&inst->ccp->sem) == 0){
        os_error_t err = os_sem_release(&inst->ccp->sem); 
        assert(err == OS_OK); 
	    return true;
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
ccp_tx_error_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    
    dw1000_ccp_instance_t * ccp = inst->ccp; 
    if (ccp->config.role == CCP_ROLE_SLAVE) 
        return false;

    if(inst->fctrl_array[0] == FCNTL_IEEE_BLINK_CCP_64){
        STATS_INC(inst->ccp->stat, tx_start_error);
        if(os_sem_get_count(&inst->ccp->sem) == 0){
            os_error_t err = os_sem_release(&inst->ccp->sem);  
            assert(err == OS_OK);
        return true;    
        }
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
ccp_rx_timeout_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){

    dw1000_ccp_instance_t * ccp = inst->ccp; 
    if (ccp->config.role == CCP_ROLE_MASTER) 
        return false;

    if (os_sem_get_count(&ccp->sem) == 0){
        os_error_t err = os_sem_release(&ccp->sem);
        assert(err == OS_OK); 
        DIAGMSG("{\"utime\": %lu,\"msg\": \"ccp:rx_timeout_cb\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

        STATS_INC(inst->ccp->stat, rx_timeout);
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
ccp_reset_cb(struct _dw1000_dev_instance_t * inst,  dw1000_mac_interface_t * cbs){
    /* Place holder */
    if(os_sem_get_count(&inst->ccp->sem) == 0){
        os_error_t err = os_sem_release(&inst->ccp->sem);
        assert(err == OS_OK);   
        STATS_INC(inst->ccp->stat, reset);
        return true;    
    }
    return false;   // CCP is an observer and should not return true
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
dw1000_ccp_send(struct _dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode)
{

    STATS_INC(inst->ccp->stat,send);
    dw1000_ccp_instance_t * ccp = inst->ccp; 
    os_error_t err = os_sem_pend(&ccp->sem, OS_TIMEOUT_NEVER);
    assert(err == OS_OK);
    
    ccp_frame_t * previous_frame = ccp->frames[(uint16_t)(ccp->idx)%ccp->nframes];
    ccp_frame_t * frame = ccp->frames[(ccp->idx+1)%ccp->nframes];
    
    frame->transmission_timestamp = (previous_frame->transmission_timestamp
                                    + ((uint64_t)inst->ccp->period << 16)
                                    ) & 0x0FFFFFFFFFFUL;
    dw1000_set_delay_start(inst, frame->transmission_timestamp);
    frame->transmission_timestamp += inst->tx_antenna_delay;

    frame->seq_num = previous_frame->seq_num + 1;
    frame->long_address = inst->ccp->uuid;
    frame->transmission_interval = inst->ccp->period;

    dw1000_write_tx(inst, frame->array, 0, sizeof(ccp_blink_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(ccp_blink_frame_t), 0, true); 
    dw1000_set_wait4resp(inst, false);    
    ccp->status.start_tx_error = dw1000_start_tx(inst).start_tx_error;
    if (ccp->status.start_tx_error ){
        STATS_INC(inst->ccp->stat, tx_start_error);
        previous_frame->transmission_timestamp = (frame->transmission_timestamp + ((uint64_t)inst->ccp->period << 16)) & 0x0FFFFFFFFFFUL;
        ccp->idx++;
        err =  os_sem_release(&ccp->sem);
        assert(err == OS_OK); 

    }else if(mode == DWT_BLOCKING){
        err = os_sem_pend(&ccp->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions 
        assert(err == OS_OK); 
        err =  os_sem_release(&ccp->sem);
        assert(err == OS_OK); 
    }
    return ccp->status;
}


/*! 
 * @fn dw1000_ccp_receive(dw1000_dev_instance_t * inst, dw1000_ccp_modes_t mode)
 *
 * @brief Explicit entry function for reveicing a ccp frame.
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
dw1000_ccp_listen(struct _dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode)
{
    DIAGMSG("{\"utime\": %lu,\"msg\": \"dw1000_ccp_listen\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

    dw1000_ccp_instance_t * ccp = inst->ccp; 
    os_error_t err = os_sem_pend(&ccp->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    STATS_INC(inst->ccp->stat,listen);

    ccp->status = (dw1000_ccp_status_t){
        .rx_timeout_error = 0,
        .start_rx_error = 0
    };
    ccp->status.start_rx_error = dw1000_start_rx(inst).start_rx_error;
    if (ccp->status.start_rx_error){
        err = os_sem_release(&ccp->sem);
        assert(err == OS_OK); 
    }else if(mode == DWT_BLOCKING){
        err = os_sem_pend(&ccp->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions 
        assert(err == OS_OK); 
        err = os_sem_release(&ccp->sem);
        assert(err == OS_OK); 
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
    ccp->config.role = role;

    if (ccp->config.role == CCP_ROLE_MASTER)
        ccp->epoch = frame->transmission_timestamp = dw1000_read_systime(inst);
    else {
        ccp->epoch = frame->reception_timestamp = dw1000_read_systime(inst);
        /* Temporarily override period to start listening for the first
         * ccp packet sooner */
        ccp->period = 5000;
    }

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




