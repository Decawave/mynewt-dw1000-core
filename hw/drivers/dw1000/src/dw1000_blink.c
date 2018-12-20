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
 * @file dw1000_blink.c
 * @author paul kettle
 * @date 2018
 * @brief clock calibration packets
 *
 * @details This is the blink base class which utilises the functions to enable/disable the configurations related to blink.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <os/os.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include "bsp/bsp.h"

#if MYNEWT_VAL(DW1000_BLINK_ENABLED)
#include <dw1000/dw1000_blink.h>

#if MYNEWT_VAL(CLOCK_CALIBRATION_ENABLED)
#include <clkcal/clkcal.h>
#endif

#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED)

/*    %Lowpass design
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


static bool blink_rx_complete_cb(struct _dw1000_dev_instance_t * inst);
static bool blink_tx_complete_cb(struct _dw1000_dev_instance_t * inst);
static bool blink_rx_error_cb(dw1000_dev_instance_t * inst);
static bool blink_rx_timeout_cb(dw1000_dev_instance_t * inst);
static bool blink_tx_error_cb(dw1000_dev_instance_t * inst);
static struct _dw1000_blink_status_t dw1000_blink_blink(struct _dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode);
#if MYNEWT_VAL(CLOCK_CALIBRATION_ENABLED) !=1
static void blink_postprocess(struct os_event * ev);
#endif


static void blink_timer_irq(void * arg);
static void blink_timer_ev_cb(struct os_event *ev);

/**
 * API to initiate timer for blink.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 * @return void
 */
static void
blink_timer_init(struct _dw1000_dev_instance_t * inst) {

    dw1000_blink_instance_t * blink = inst->blink;
    blink->status.timer_enabled = true;

    os_cputime_timer_init(&blink->timer, blink_timer_irq, (void *) inst);
    os_callout_init(&blink->event_cb, &blink->eventq, blink_timer_ev_cb, (void *) inst);
    os_cputime_timer_relative(&blink->timer, MYNEWT_VAL(OS_LATENCY));
}

/**
 * blink_timer_event is in the interrupr context and schedules and tasks on the event queue
 *
 * @param arg   Pointer to  dw1000_dev_instance_t.
 * @return void
 */
static void
blink_timer_irq(void * arg){
   assert(arg);

    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *) arg;
    dw1000_blink_instance_t * blink = inst->blink;
//    os_eventq_put(&blink->eventq, &blink->event_cb.c_ev);
    blink_timer_ev_cb(&blink->event_cb.c_ev);
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
blink_timer_ev_cb(struct os_event *ev) {
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_blink_instance_t * blink = inst->blink;

    uint32_t cputime = os_cputime_get32() - os_cputime_usecs_to_ticks(MYNEWT_VAL(OS_LATENCY));
    if(dw1000_blink_blink(inst, DWT_BLOCKING).start_tx_error)
      hal_timer_start_at(&blink->timer, cputime + os_cputime_usecs_to_ticks(dw1000_dwt_usecs_to_usecs(blink->period)));
}

static void
blink_task(void *arg)
{
    dw1000_blink_instance_t * inst = arg;
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
blink_tasks_init(struct _dw1000_blink_instance_t * inst)
{
    /* Check if the tasks are already initiated */
    if (!os_eventq_inited(&inst->eventq))
    {
        /* Use a dedicate event queue for tdma events */
        os_eventq_init(&inst->eventq);
        os_task_init(&inst->task_str, "dw1000_blink",
                     blink_task,
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
 * @return dw1000_blink_instance_t *
 */
dw1000_blink_instance_t *
dw1000_blink_init(struct _dw1000_dev_instance_t * inst, uint16_t nframes, uint64_t clock_master){
    assert(inst);
    assert(nframes > 1);
    dw1000_extension_callbacks_t blink_cbs;

    if (inst->blink == NULL ) {
        dw1000_blink_instance_t * blink = (dw1000_blink_instance_t *) malloc(sizeof(dw1000_blink_instance_t) + nframes * sizeof(blink_frame_t *));
        assert(blink);
        memset(blink, 0, sizeof(dw1000_blink_instance_t));
        blink->status.selfmalloc = 1;
        blink->nframes = nframes;
        blink_frame_t blink_default = {
            .fctrl = FCNTL_IEEE_BLINK_TAG_64,    // frame control (FCNTL_IEEE_BLINK_64 to indicate a data frame using 16-bit addressing).
            .correction_factor = 1.0f,
            .seq_num = 0xFF
        };

        for (uint16_t i = 0; i < blink->nframes; i++){
            blink->frames[i] = (blink_frame_t *) malloc(sizeof(blink_frame_t));
            memcpy(blink->frames[i], &blink_default, sizeof(blink_frame_t));
            blink->frames[i]->seq_num = i-1;
        }

        blink->parent = inst;
        inst->blink = blink;
        blink->task_prio = inst->task_prio + 1;

    }else{
        assert(inst->blink->nframes == nframes);
    }
    inst->blink->period = MYNEWT_VAL(BLINK_PERIOD);
    inst->blink->config = (dw1000_blink_config_t){
        .postprocess = false,
#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED)
        .fs_xtalt_autotune = true,
#endif
    };
    //inst->clock_master = clock_master;
    (void)clock_master;
    os_error_t err = os_sem_init(&inst->blink->sem, 0x1);
    assert(err == OS_OK);

#if MYNEWT_VAL(CLOCK_CALIBRATION_ENABLED)
    inst->blink->clkcal = clkcal_init(NULL, inst->blink);      // Using clkcal process
#else
    dw1000_blink_set_postprocess(inst->blink, &blink_postprocess);            // Using default process
#endif

    blink_cbs.tx_complete_cb = blink_tx_complete_cb;
    blink_cbs.rx_complete_cb = blink_rx_complete_cb;
    blink_cbs.rx_timeout_cb = blink_rx_timeout_cb;
    blink_cbs.rx_error_cb = blink_rx_error_cb;
    blink_cbs.tx_error_cb = blink_tx_error_cb;

    dw1000_blink_set_ext_callbacks(inst, blink_cbs);

    dw1000_blink_instance_t * blink = inst->blink;
    blink_frame_t * frame = blink->frames[(blink->idx)%blink->nframes];

#if MYNEWT_VAL(CLOCK_CALIBRATION)
    frame->transmission_timestamp = _dw1000_read_systime(inst);
#else
    frame->transmission_timestamp = dw1000_read_systime(inst);
#endif

#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED)
    inst->blink->xtalt_sos = sosfilt_init(NULL, sizeof(g_fs_xtalt_b)/sizeof(float)/BIQUAD_N);
#endif

    //blink_tasks_init(inst->blink);
    (void)blink_tasks_init;
    inst->blink->status.initialized = 1;
    return inst->blink;
}

/**
 * Deconstructor
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @return void
 */
void
dw1000_blink_free(dw1000_blink_instance_t * inst){
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

/**
 * API to register extension callbacks in blink.
 *
 * @param inst     Pointer to dw1000_dev_instance_t
 * @param blink_cbs  Callbacks of blink.
 * @return void
 */
void dw1000_blink_set_ext_callbacks(dw1000_dev_instance_t * inst, dw1000_extension_callbacks_t blink_cbs){
    blink_cbs.id = DW1000_BLINK;
    dw1000_add_extension_callbacks(inst , blink_cbs);
}

/**
 * API that overrides the default post-processing behaviors, replacing the JSON stream with an alternative
 * or an advanced timescale processing algorithm.
 *
 * @param inst              Pointer to dw1000_dev_instance_t.
 * @param blink_postprocess   Pointer to os_events.
 * @return void
 */
void
dw1000_blink_set_postprocess(dw1000_blink_instance_t * inst, os_event_fn * blink_postprocess){
    os_callout_init(&inst->callout_postprocess, os_eventq_dflt_get(), blink_postprocess, (void *) inst);
    inst->config.postprocess = true;
}

#if MYNEWT_VAL(CLOCK_CALIBRATION_ENABLED) !=1
/**
 * API that serves as a place holder for timescale processing and by default is creates json string for the event.
 *
 * @param ev   pointer to os_events.
 * @return void
 */
static void
blink_postprocess(struct os_event * ev){
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);
    dw1000_blink_instance_t * blink = (dw1000_blink_instance_t *)ev->ev_arg;
    printf("blink-rx\n");
#if 0
    blink_frame_t * previous_frame = blink->frames[(blink->idx-1)%blink->nframes];
    blink_frame_t * frame = blink->frames[(blink->idx)%blink->nframes];

    printf("{\"utime\":%lu,\"blink\":[%llu,%llu],\"correction\":%lu,\"seq_num\":%d}\n",
        os_cputime_ticks_to_usecs(os_cputime_get32()),
        frame->reception_timestamp,
        (uint64_t)((uint64_t)(frame->reception_timestamp) - (uint64_t)(previous_frame->reception_timestamp)) & 0xFFFFFFFFF,
        *(uint32_t *)&frame->correction_factor,
        frame->seq_num
    );
#endif
    dw1000_dev_instance_t* inst = blink->parent;
    inst->control = inst->control_rx_context;
    dw1000_restart_rx(inst,inst->control_rx_context);
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
uint8_t blink_time[5];
uint8_t blink_seq_number = 0;
uint8_t blink_rx = 0;
uint16_t blink_firstPath = 0;
static struct os_callout blink_send_callout;
void blink_send(struct os_event *ev);

static bool
blink_rx_complete_cb(struct _dw1000_dev_instance_t * inst){
    if (inst->fctrl_array[0] == FCNTL_IEEE_BLINK_TAG_64){
        // BLINK Packet Received
    #if 0
        uint64_t clock_master;
        dw1000_read_rx(inst, (uint8_t *) &clock_master, offsetof(ieee_blink_frame_t,long_address), sizeof(uint64_t));
        if(inst->clock_master != clock_master){
            dw1000_restart_rx(inst, inst->control_rx_context);
            return true;
       }
    #endif
    }else{
        return false;
    }
    dw1000_blink_instance_t * blink = inst->blink;
    blink_frame_t * frame = blink->frames[(++blink->idx)%blink->nframes];

    dw1000_read_rx(inst, frame->array, 0, sizeof(ieee_blink_frame_t));

#if MYNEWT_VAL(ADAPTIVE_TIMESCALE_ENABLED)
    frame->reception_timestamp = _dw1000_read_rxtime_raw(inst);
#else
    frame->reception_timestamp = dw1000_read_rxtime(inst);
#endif
    memcpy(blink_time,&frame->reception_timestamp,5);
    blink_seq_number = frame->seq_num;
    blink_firstPath = dw1000_read_reg(inst, RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET, 2);
    blink_rx = 1;
#if 0
    int32_t tracking_interval = (int32_t) dw1000_read_reg(inst, RX_TTCKI_ID, 0, sizeof(int32_t));
    int32_t tracking_offset = (int32_t)(((uint32_t) dw1000_read_reg(inst, RX_TTCKO_ID, 0, sizeof(uint32_t)) & RX_TTCKO_RXTOFS_MASK) << 13) >> 13;
    frame->correction_factor = 1.0f +((float)tracking_offset) / tracking_interval;
#endif

    blink->status.valid |= blink->idx > blink->nframes;
    if (blink->config.postprocess && blink->status.valid)
        os_eventq_put(os_eventq_dflt_get(), &blink->callout_postprocess.c_ev);
   else{
        inst->control = inst->control_rx_context;
        dw1000_restart_rx(inst,inst->control_rx_context);
    }
#if MYNEWT_VAL(FS_XTALT_AUTOTUNE_ENABLED)
    if (blink->config.fs_xtalt_autotune && blink->status.valid){
        float fs_xtalt_offset = sosfilt(blink->xtalt_sos,  1e6 * ((float)tracking_offset) / tracking_interval, g_fs_xtalt_b, g_fs_xtalt_a);
        if(blink->xtalt_sos->clk % FS_XTALT_SETTLINGTIME == 0){
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
    return true;

}

/**
 * Precise timing is achieved by adding a fixed period to the transmission time of the previous frame. This static
 * function is called on successful transmission of a BLINK packet, and this advances the frame index point. Circular addressing is used
 * for the frame addressing. The next os_event is scheduled to occur in (MYNEWT_VAL(BLINK_PERIOD) - MYNEWT_VAL(CCP_OS_LATENCY)) usec
 * from now. This provided a context switch guard zone. The assumption is that the underlying OS will have sufficient time start
 * the call dw1000_set_delay_start within dw1000_blink_blink.
 *
 * @param inst    Pointer to dw1000_dev_instance_t
 * @return bool
 */
static bool
blink_tx_complete_cb(struct _dw1000_dev_instance_t * inst){
    if (inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        return false;
    }
    //Advance frame idx
    dw1000_blink_instance_t * blink = inst->blink;
    //blink_frame_t * frame = blink->frames[(++blink->idx)%blink->nframes];
    printf("blink-tx-cb\n");
#if 0
    blink_frame_t * previous_frame = blink->frames[(blink->idx)%blink->nframes];
    blink_frame_t * frame = blink->frames[(++blink->idx)%blink->nframes];
    printf("{\"utime\":%lu,\"blink_tx\":[\"%llX\",\"%llX\"],\"seq_num\":%d}\n",
        os_cputime_ticks_to_usecs(os_cputime_get32()),
        frame->transmission_timestamp,
        (uint64_t)((uint64_t)(frame->transmission_timestamp) - (uint64_t)(previous_frame->transmission_timestamp)) & 0xFFFFFFFFF,
        frame->seq_num
    );
    if (blink->status.timer_enabled){
        uint32_t cputime = os_cputime_get32() - os_cputime_usecs_to_ticks(MYNEWT_VAL(OS_LATENCY));
        hal_timer_start_at(&blink->timer, cputime + os_cputime_usecs_to_ticks(dw1000_dwt_usecs_to_usecs(blink->period - MYNEWT_VAL(OS_LATENCY))));
    }
#endif

    os_callout_reset(&blink_send_callout, OS_TICKS_PER_SEC/4);
    os_sem_release(&blink->sem);
    return true;
}

/**
 * API for rx_error_cb of blink.
 *
 * @param inst   pointer to dw1000_dev_instance_t.
 * @return void
 */
static bool
blink_rx_error_cb(struct _dw1000_dev_instance_t * inst){
    /* Place holder */
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
            return false;
    }
    return true;
}

/**
 * API for tx_error_cb of blink.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @return bool
 */
static bool
blink_tx_error_cb(struct _dw1000_dev_instance_t * inst){
    /* Place holder */
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        return false;
    }
    return true;
}

/**
 * API for rx_timeout_cb of blink.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @return void
 */
static bool
blink_rx_timeout_cb(struct _dw1000_dev_instance_t * inst){
    /* Place holder */
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        return false;
    }
    return true;
}

/**
 * API that start clock calibration packets (BLINK) blinks  with a pulse repetition period of MYNEWT_VAL(CCP_PERIOD).
 * Precise timing is achieved by adding a fixed period to the transmission time of the previous frame.
 * This removes the need to explicitly read the systime register and the assiciated non-deterministic latencies.
 * This function is static function for internl use. It will force a Half Period Delay Warning is called at
 * out of sequence.
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @param mode   dw1000_blink_modes_t for BLINK_BLOCKING, CCP_NON_BLOCKING modes.
 * @return dw1000_blink_status_t
 */
static dw1000_blink_status_t
dw1000_blink_blink(struct _dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode){

    os_error_t err = os_sem_pend(&inst->blink->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    dw1000_blink_instance_t * blink = inst->blink;
    blink_frame_t * previous_frame = blink->frames[(blink->idx-1)%blink->nframes];
    blink_frame_t * frame = blink->frames[(blink->idx)%blink->nframes];

    //frame->transmission_timestamp = previous_frame->transmission_timestamp + 2 * ((uint64_t)inst->blink->period << 15);
    frame->seq_num += inst->blink->nframes;
    frame->long_address = inst->my_short_address;

    dw1000_write_tx(inst, frame->array, 0, sizeof(ieee_blink_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(ieee_blink_frame_t), 0, true);
    dw1000_set_wait4resp(inst, false);
    //dw1000_set_delay_start(inst, frame->transmission_timestamp);

    blink->status.start_tx_error = dw1000_start_tx(inst).start_tx_error;
    if (blink->status.start_tx_error){
        // Half Period Delay Warning occured try for the next epoch
        // Use seq_num to detect this on receiver size
        previous_frame->transmission_timestamp += ((uint64_t)inst->blink->period << 15);
        os_sem_release(&inst->blink->sem);
    }
    else if(mode == DWT_BLOCKING){
        err = os_sem_pend(&inst->blink->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions
        os_sem_release(&inst->blink->sem);
    }
   return blink->status;
}

/**
 * API to start clock calibration packets (BLINK) blinks.
 * With a pulse repetition period of MYNEWT_VAL(BLINK_PERIOD).
 *
 * @param inst   Pointer to dw1000_dev_instance_t.
 * @return void
 */

void blink_send(struct os_event *ev) {
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
   // dw1000_blink_instance_t * blink = inst->blink;

   if(dw1000_blink_blink(inst, DWT_BLOCKING).start_tx_error)
       os_callout_reset(&blink_send_callout, OS_TICKS_PER_SEC/4);
}

void
dw1000_blink_start(struct _dw1000_dev_instance_t * inst){
    // Initialise frame timestamp to current time
    dw1000_blink_instance_t * blink = inst->blink;
    blink->idx = 0x0;
    blink->status.valid = false;
    //blink_frame_t * frame = blink->frames[(blink->idx)%blink->nframes];
    //frame->transmission_timestamp = dw1000_read_systime(inst) + (MYNEWT_VAL(OS_LATENCY) << 15);
    //blink_timer_init(inst);
    (void)blink_timer_init;
    os_callout_init(&blink_send_callout, os_eventq_dflt_get(), blink_send, (void *) inst);
    os_callout_reset(&blink_send_callout, OS_TICKS_PER_SEC/2);
}

/**
 * API to stop clock calibration packets (BLINK) blinks.
 *
 * @param inst   Pointer to  dw1000_dev_instance_t.
 * @return void
 */
void
dw1000_blink_stop(dw1000_dev_instance_t * inst){
    dw1000_blink_instance_t * blink = inst->blink;
   os_cputime_timer_stop(&blink->timer);
}
#endif /* MYNEWT_VAL(DW1000_BLINK_ENABLED) */
