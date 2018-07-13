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
#include <assert.h>
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

#if MYNEWT_VAL(DW1000_PAN)
#include <dw1000/dw1000_pan.h>

static pan_frame_t frames[] = {
    [0] = {
        .fctrl = FCNTL_IEEE_BLINK_TAG_64,    // frame control (FCNTL_IEEE_BLINK_64 to indicate a data frame using 16-bit addressing).
        .seq_num = 0x0,
    },
    [1] = {
        .fctrl = FCNTL_IEEE_BLINK_TAG_64,    // frame control (FCNTL_IEEE_BLINK_64 to indicate a data frame using 16-bit addressing).
    }
};

static void pan_rx_complete_cb(dw1000_dev_instance_t * inst);
static void pan_tx_complete_cb(dw1000_dev_instance_t * inst);
static void pan_rx_timeout_cb(dw1000_dev_instance_t * inst);
static void pan_rx_error_cb(dw1000_dev_instance_t * inst);
static void pan_tx_error_cb(dw1000_dev_instance_t * inst);
static dw1000_pan_status_t dw1000_pan_blink(dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode);
static void pan_postprocess(struct os_event * ev);
static struct os_callout g_pan_callout_timer;
static struct os_callout g_pan_callout_postprocess;

static void 
pan_timer_ev_cb(struct os_event *ev) {
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_pan_instance_t * pan = inst->pan; 

    if(dw1000_pan_blink(inst, DWT_BLOCKING).start_tx_error)
        os_callout_reset(&g_pan_callout_timer, OS_TICKS_PER_SEC * (pan->period - MYNEWT_VAL(OS_LATENCY)) * 1e-6);   
}

static void 
pan_timer_init(dw1000_dev_instance_t * inst) {
    dw1000_pan_instance_t * pan = inst->pan; 
    os_callout_init(&g_pan_callout_timer, os_eventq_dflt_get(), pan_timer_ev_cb, (void *) inst);
    os_callout_reset(&g_pan_callout_timer, OS_TICKS_PER_SEC/100);
    pan->status.timer_enabled = true;
}

/*! 
 * @fn dw1000_pan_init(dw1000_pan_instance_t * inst)
 *
 * @brief Allocate resources on pan_master and TAG/ANCHOR for pan discovery behaviour. TAG/ANCHOR and anchor size resources
 * can be freeded on TAG/ANCHOR on once assigment have are valid. .   
 *
 * input parameters
 * @param inst - struct os_event * ev
 *
 * output parameters
 *
 * returns none
 */
dw1000_pan_instance_t * 
dw1000_pan_init(dw1000_dev_instance_t * inst,  dw1000_pan_config_t * config){
    assert(inst);

    uint16_t nframes = sizeof(frames)/sizeof(pan_frame_t);
    dw1000_extension_callbacks_t pan_cbs;
    if (inst->pan == NULL ) {
        inst->pan = (dw1000_pan_instance_t *) malloc(sizeof(dw1000_pan_instance_t) + nframes * sizeof(pan_frame_t *)); 
        assert(inst->pan);
        memset(inst->pan, 0, sizeof(dw1000_pan_instance_t));
        inst->pan->status.selfmalloc = 1;
        inst->pan->nframes = nframes; 
    }

    inst->pan->parent = inst;
    inst->pan->period = MYNEWT_VAL(PAN_PERIOD);
    inst->pan->config = config;
    inst->pan->control = (dw1000_pan_control_t){
        .postprocess = false,
    };

    os_error_t err = os_sem_init(&inst->pan->sem, 0x1); 
    err |= os_sem_init(&inst->pan->sem_waitforsucess, 0x1); 
    assert(err == OS_OK);

    for (uint16_t i = 0; i < inst->pan->nframes; i++)
        inst->pan->frames[i] = &frames[i];

    dw1000_pan_set_postprocess(inst, pan_postprocess);

    pan_cbs.tx_complete_cb = pan_tx_complete_cb;
    pan_cbs.rx_complete_cb = pan_rx_complete_cb;
    pan_cbs.rx_timeout_cb = pan_rx_timeout_cb;
    pan_cbs.rx_error_cb = pan_rx_error_cb;
    pan_cbs.tx_error_cb = pan_tx_error_cb;
    
    dw1000_pan_set_ext_callbacks(inst, pan_cbs);

    dw1000_pan_instance_t * pan = inst->pan; 
    pan_frame_t * frame = pan->frames[(pan->idx)%pan->nframes]; 
    frame->transmission_timestamp = dw1000_read_systime(inst);
    inst->pan->status.initialized = 1;
    return inst->pan;
}

/*! 
 * @fn dw1000_pan_free(dw1000_dev_instance_t * inst)
 *
 * @brief Free resources and restore default behaviour. 
 *
 * input parameters
 * @param inst - dw1000_pan_instance_t * inst
 *
 * output parameters
 *
 * returns none
 */
void 
dw1000_pan_free(dw1000_dev_instance_t * inst){
    assert(inst->pan); 
    dw1000_remove_extension_callbacks(inst, DW1000_PAN); 
    if (inst->status.selfmalloc)
        free(inst->pan);
    else
        inst->status.initialized = 0;
}


/*! 
 * @fn dw1000_pan_set_callbacks(dw1000_dev_instance_t * inst, os_event_fn * pan_postprocess)
 *
 * @brief Replaces default behavor. Not common.
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 * @param inst - dw1000_dev_cb_t  pan_rx_complete_cb
 * @param inst - dw1000_dev_cb_t  pan_tx_complete_cb
 *
 * output parameters
 *
 * returns none
 */
void dw1000_pan_set_ext_callbacks(dw1000_dev_instance_t * inst, dw1000_extension_callbacks_t pan_cbs){
    pan_cbs.id = DW1000_PAN;
    dw1000_add_extension_callbacks(inst , pan_cbs);
}

/*! 
 * @fn dw1000_pan_set_postprocess(dw1000_dev_instance_t * inst, os_event_fn * pan_postprocess)
 *
 * @brief Replace default behavor. Required on pan_master.
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 * @param inst - struct os_event * pan_postprocess
 *
 * output parameters
 *
 * returns none
 */
void 
dw1000_pan_set_postprocess(dw1000_dev_instance_t * inst, os_event_fn * pan_postprocess){
    os_callout_init(&g_pan_callout_postprocess, os_eventq_dflt_get(), pan_postprocess, (void *) inst);
    dw1000_pan_instance_t * pan = inst->pan; 
    pan->control.postprocess = true;
}

/*! 
 * @fn pan_postprocess(struct os_event * ev)
 *
 * @brief This a template which should be replaced by the pan_master by a event that tracks UUIDs 
 * and allocated PANIDs and SLOTIDs. See dw1000_pan_set_postprocess to replace current behavor. On the TAG/ANCHOR size this 
 * template generate a json log of the event.
 *
 * input parameters
 * @param inst - struct os_event * ev
 *
 * output parameters
 *
 * returns none
 */
static void 
pan_postprocess(struct os_event * ev){
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);
    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_pan_instance_t * pan = inst->pan; 
    pan_frame_t * frame = pan->frames[(pan->idx)%pan->nframes]; 
  
    if(pan->status.valid && frame->long_address == inst->my_long_address)
        printf("{\"utime\":%lu,\"UUID\":\"%llX\",\"ID\":\"%X\",\"PANID\":\"%X\",\"slot\":%d}\n", 
            os_cputime_ticks_to_usecs(os_cputime_get32()),
            frame->long_address,
            frame->short_address,            
            frame->pan_id,
            frame->slot_id
        );
    else if (inst->frame_len == sizeof(struct _ieee_blink_frame_t))
        printf("{\"utime\":%lu,\"UUID\":\"%llX\",\"seq_num\":%d}\n", 
            os_cputime_ticks_to_usecs(os_cputime_get32()),
            frame->long_address,
            frame->seq_num
        );    
    else if (inst->frame_len == sizeof(struct _pan_frame_resp_t))
        printf("{\"utime\":%lu,\"UUID\":\"%llX\",\"ID\":\"%X\",\"PANID\":\"%X\",\"slot\":%d}\n", 
            os_cputime_ticks_to_usecs(os_cputime_get32()),
            frame->long_address,
            frame->short_address,            
            frame->pan_id,
            frame->slot_id
        );
}

/*! 
 * @fn pan_rx_complete_cb(dw1000_dev_instance_t * inst)
 *
 * @brief This is an internal static function that executes on both the pan_master Node and the TAG/ANCHOR 
 * that initiated the blink. On the pan_master the postprecess function should allocate a PANID and a SLOTID, 
 * while on the TAG/ANCHOR the returned allocations are assigned and the PAN discover event is stopped. The pan 
 * discovery resources can be released. 
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 *
 * output parameters
 *
 * returns none
 */
static void 
pan_rx_complete_cb(dw1000_dev_instance_t * inst){
     if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        if(inst->extension_cb->next != NULL){
			inst->extension_cb = inst->extension_cb->next;
			if(inst->extension_cb->rx_complete_cb != NULL)
	            inst->extension_cb->rx_complete_cb(inst);
        //For the range service the fctrl is same as FCNTL_IEEE_RANGE_16
        //In he range case the decision is always taken by application
        //So put it back to receive only if the intended packet doesn't match
        //any of the reserved or range packet
        }else if(inst->fctrl != FCNTL_IEEE_RANGE_16){
            dw1000_dev_control_t control = inst->control_rx_context;
            dw1000_restart_rx(inst, control);
        }
        return;
    }else if(inst->pan->status.valid == true){
        dw1000_dev_control_t control = inst->control_rx_context;
        dw1000_restart_rx(inst, control);
        return;
    }
    dw1000_pan_instance_t * pan = inst->pan; 
    pan_frame_t * frame = pan->frames[(pan->idx)%pan->nframes];

    if (inst->frame_len == sizeof(struct _ieee_blink_frame_t)){
        dw1000_read_rx(inst, frame->array, 0, sizeof(struct _ieee_blink_frame_t));
        frame->reception_timestamp = dw1000_read_rxtime(inst); 
        int32_t tracking_interval = (int32_t) dw1000_read_reg(inst, RX_TTCKI_ID, 0, sizeof(int32_t));
        int32_t tracking_offset = (int32_t) dw1000_read_reg(inst, RX_TTCKO_ID, 0, sizeof(int32_t)) & RX_TTCKO_RXTOFS_MASK;
        frame->correction_factor = 1.0f + ((float)tracking_offset) / tracking_interval;
    }
    else if (inst->frame_len == sizeof(struct _pan_frame_resp_t)) { 
        dw1000_read_rx(inst, frame->array, 0, sizeof(struct _pan_frame_resp_t));

        if(frame->long_address == inst->my_long_address){   
            // TAG/ANCHOR side
            inst->my_short_address = frame->short_address;
            inst->PANID = frame->pan_id;
            inst->slot_id = frame->slot_id;
            pan->status.valid = true;
            dw1000_pan_stop(inst);
            os_sem_release(&pan->sem);
            os_sem_release(&pan->sem_waitforsucess);
        }
    }         
    // both pan_master and TAG/ANCHOR
    if (pan->control.postprocess) 
        os_eventq_put(os_eventq_dflt_get(), &g_pan_callout_postprocess.c_ev); 
}

/*! 
 * @fn pan_tx_complete_cb(dw1000_dev_instance_t * inst)
 *
 * @brief This is an internal static function that executes on the TAG/ANCHOR.
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 *
 * output parameters
 *
 * returns none
 */
static void 
pan_tx_complete_cb(dw1000_dev_instance_t * inst){
    //printf("pan_tx_complete_cb\n");
   if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        if(inst->extension_cb->next != NULL){
			inst->extension_cb = inst->extension_cb->next;
			if(inst->extension_cb->tx_complete_cb != NULL)
	            inst->extension_cb->tx_complete_cb(inst);
        }
	return;
    }
    dw1000_pan_instance_t * pan = inst->pan;
    if (pan->status.timer_enabled && pan->status.valid == false)
        os_callout_reset(&g_pan_callout_timer, OS_TICKS_PER_SEC * (pan->period - MYNEWT_VAL(OS_LATENCY)) * 1e-6); 
    os_sem_release(&inst->pan->sem);  
    pan->idx++;
}

/*!
 * @fn pan_rx_error_cb(dw1000_dev_instance_t * inst)
 *
 * @brief This is an internal static function that executes on the TAG/ANCHOR.
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t *
 *
 * output parameters
 *
 *
 * returns none
 */
static void
pan_rx_error_cb(dw1000_dev_instance_t * inst){
    /* Place holder */
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        if(inst->extension_cb->next != NULL){
			inst->extension_cb = inst->extension_cb->next;
			if(inst->extension_cb->rx_error_cb != NULL)
	            inst->extension_cb->rx_error_cb(inst);
        }
        return;
    }
    os_sem_release(&inst->pan->sem);
}

/*!
 * @fn pan_tx_error_cb(dw1000_dev_instance_t * inst)
 *
 * @brief This is an internal static function that executes on the TAG/ANCHOR.
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t *
 *
 * output parameters
 *
 *
 * returns none
 */

static void
pan_tx_error_cb(dw1000_dev_instance_t * inst){
    /* Place holder */
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        if(inst->extension_cb->next != NULL){
			inst->extension_cb = inst->extension_cb->next;
			if(inst->extension_cb->tx_error_cb != NULL)
	            inst->extension_cb->tx_error_cb(inst);
        }
        return;
    }
}
/*! 
 * @fn pan_rx_timeout_cb(dw1000_dev_instance_t * inst)
 *
 * @brief This is an internal static function that executes on the TAG/ANCHOR.
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 *
 * output parameters
 *
 * returns none
 */
static void 
pan_rx_timeout_cb(dw1000_dev_instance_t * inst){
    //printf("pan_rx_timeout_cb\n");  
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        if(inst->extension_cb->next != NULL){
			inst->extension_cb = inst->extension_cb->next;
			if(inst->extension_cb->rx_timeout_cb != NULL)
	            inst->extension_cb->rx_timeout_cb(inst);
        }
        return;
    }
    dw1000_pan_instance_t * pan = inst->pan;  
    if (pan->status.timer_enabled)
        os_callout_reset(&g_pan_callout_timer, OS_TICKS_PER_SEC * (pan->period - MYNEWT_VAL(OS_LATENCY)) * 1e-6);    
    os_sem_release(&inst->pan->sem);  
}

/*! 
 * @fn dw1000_pan_blink(dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode)
 *
 * @brief A Personal Area Network blink request is a discovery phase in which a TAG/ANCHOR seeks to discover 
 * an available PAN Master. The outcome of this process is a PANID and SLOTID assignment.   
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 * @param mode - dw1000_devmodes_t for DWT_BLOCKING, DWT_NONBLOCKING.
 *
 * output parameters
 *
 * returns dw1000_pan_status_t 
 */
static dw1000_pan_status_t 
dw1000_pan_blink(dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode){

    os_error_t err = os_sem_pend(&inst->pan->sem,  OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    //printf("dw1000_pan_blink\n");  

    dw1000_pan_instance_t * pan = inst->pan; 
    pan_frame_t * frame = pan->frames[(pan->idx)%pan->nframes];
    pan_frame_t * previous_frame = pan->frames[(pan->idx-1)%pan->nframes];

    frame->transmission_timestamp = previous_frame->transmission_timestamp 
        + 2 * ((uint64_t)inst->pan->period << 15) 
        + ((uint64_t)inst->pan->config->tx_holdoff_delay << 15); // random component

    frame->seq_num += inst->pan->nframes;
    frame->long_address = inst->my_long_address;

    dw1000_write_tx(inst, frame->array, 0, sizeof(ieee_blink_frame_t));
    dw1000_write_tx_fctrl(inst, sizeof(ieee_blink_frame_t), 0, true); 
    dw1000_set_wait4resp(inst, true);    
    dw1000_set_delay_start(inst, frame->transmission_timestamp);   
    dw1000_set_rx_timeout(inst, pan->config->rx_timeout_period); 
    pan->status.start_tx_error = dw1000_start_tx(inst).start_tx_error;
    if (pan->status.start_tx_error){
        // Half Period Delay Warning occured try for the next epoch
        // Use seq_num to detect this on receiver size
        printf("Half Period Delay Warning\n");
        frame->transmission_timestamp += ((uint64_t)inst->pan->period << 15);
        os_sem_release(&inst->pan->sem);
    }  
    else if(mode == DWT_BLOCKING){
        err = os_sem_pend(&inst->pan->sem, OS_TIMEOUT_NEVER); // Wait for completion of transactions 
        os_sem_release(&inst->pan->sem);
        assert(err == OS_OK);
    }
   return pan->status;
}


/*! 
 * @fn dw1000_pan_start(dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode)
 *
 * @brief A Personal Area Network blink is a discovery phase in which a TAG/ANCHOR seeks to discover 
 * an available PAN Master. This function scheduled this discovery timer event. The pan_master does not 
 * need to call this function.     
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 * @param mode - dw1000_devmodes_t for DWT_BLOCKING, DWT_NONBLOCKING.
 *
 * output parameters
 *
 * returns none
 */
void 
dw1000_pan_start(dw1000_dev_instance_t * inst, dw1000_dev_modes_t mode){
    // Initialise previous frame timestamp to current time
    dw1000_pan_instance_t * pan = inst->pan; 

    os_error_t err = os_sem_pend(&pan->sem_waitforsucess, OS_TIMEOUT_NEVER);
    assert(err == OS_OK);

    pan->idx = 0x0;  
    pan->status.valid = false;
    pan_frame_t * frame = pan->frames[(pan->idx)%pan->nframes]; 
    frame->transmission_timestamp = dw1000_read_systime(inst); 
    pan_timer_init(inst);

    printf("{\"utime\":%lu,\"PAN\":\"%s\"}\n", 
            os_cputime_ticks_to_usecs(os_cputime_get32()),
            "Provisioning"
    );

    if(mode == DWT_BLOCKING){
        os_error_t err = os_sem_pend(&pan->sem_waitforsucess, OS_TIMEOUT_NEVER);
        assert(err == OS_OK);
        os_sem_release(&pan->sem_waitforsucess);
    }
}

/*! 
 * @fn dw1000_pan_stop(dw1000_dev_instance_t * inst)
 *
 * @brief Called once the discovery processs is complete see pan_rx_complete_cb or to abort the discovery event.   
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 *
 * output parameters
 *
 * returns none
 */
void 
dw1000_pan_stop(dw1000_dev_instance_t * inst){
    dw1000_pan_instance_t * pan = inst->pan;   
    pan->status.timer_enabled = false;
    os_callout_stop(&g_pan_callout_timer);
    printf("{\"utime\":%lu,\"PAN\":\"%s\"}\n", 
            os_cputime_ticks_to_usecs(os_cputime_get32()),
            "Stopped"
    );
}

#endif /* MYNEWT_VAL(DW1000_PAN) */
