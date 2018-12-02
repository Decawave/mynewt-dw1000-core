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
 * @file cir.c
 * @author Paul.Kettle@decawave.com
 * @date Nov 29 2019
 * @brief Channel Impulse Response
 *
 * @details 
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
#include <cir/cir.h>
#include <cir/cir_encode.h>

#if MYNEWT_VAL(CIR_VERBOSE)
struct os_callout cir_callout;

static void
cir_complete_ev_cb(struct os_event *ev) {
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;

#if  MYNEWT_VAL(DW1000_DEVICE_0) && !MYNEWT_VAL(DW1000_DEVICE_1)
    cir_encode(inst->cir, "cir", MYNEWT_VAL(CIR_SIZE));
#elif  MYNEWT_VAL(DW1000_DEVICE_0) && MYNEWT_VAL(DW1000_DEVICE_1)
    if (inst->idx == 0)
        cir_encode(inst->cir, "cir0", MYNEWT_VAL(CIR_SIZE));   
    else     
        cir_encode(inst->cir, "cir1", MYNEWT_VAL(CIR_SIZE)); 
#endif
}
#endif


/*! 
 * @fn cir_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
 *
 * @brief Read CIR inadvance of RXENB 
 * 
 * input parameters
 * @param inst - dw1000_dev_instance_t * inst
 *
 * output parameters
 *
 * returns none 
 */


static bool
cir_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){

    if(inst->fctrl != FCNTL_IEEE_RANGE_16){
        return false;
    }

    cir_instance_t * cir = inst->cir;

    if (cir->control.enable || inst->config.cir_enable){
        cir->control.enable = inst->config.cir_enable; // restore defaults behavior

        cir->fp_idx = dw1000_read_reg(inst, RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET, sizeof(uint16_t));
        cir->fp_idx = (uint16_t) roundf( ((float) cir->fp_idx)/64.0f + 0.5f) - 2;

        dw1000_read_accdata(inst, (uint8_t *)&cir->cir, cir->fp_idx * sizeof(cir_complex_t), sizeof(cir_t));

        float _rcphase = (float)((uint8_t)dw1000_read_reg(inst, RX_TTCKO_ID, 4, sizeof(uint8_t)) & 0x7F);
        cir->rcphase = _rcphase * (M_PI/64.0f);
        cir->angle = atan2f((float)cir->cir.array[0].imag, (float)cir->cir.array[0].real);

        cir->status.valid = 1;
    #if MYNEWT_VAL(CIR_VERBOSE)
        os_callout_init(&cir_callout, os_eventq_dflt_get(), cir_complete_ev_cb,inst);
        os_eventq_put(os_eventq_dflt_get(), &cir_callout.c_ev);
    #endif
        return true;        
    }
    return false;

}

/*! 
 * @fn cir_enable(cir_instance_t * inst, bool mode){
 *
 * @brief Enable CIR reading of CIR. 
 * 
 * @param inst - cir_instance_t *
 * @param mode - bool
 * 
 * output parameters
 *
 * returns cir_instance_t * 
 */

cir_instance_t * 
cir_enable(cir_instance_t * inst, bool mode){
    inst->status.valid = 0;
    inst->control.enable = mode;
    return inst;
}


/*! 
 * @fn cir_init(ir_instance_t * ccp)
 *
 * @brief Allocate resources for the CIR instance. 
 * 
 * @param inst - cir_instance_t *
 * 
 * output parameters
 *
 * returns cir_instance_t * 
 */

cir_instance_t * 
cir_init(cir_instance_t * inst){

    if (inst == NULL) {
        inst = (cir_instance_t *) malloc(sizeof(cir_instance_t)); 
        assert(inst);
        memset(inst, 0, sizeof(cir_instance_t));
        inst->status.selfmalloc = 1;
    }
    return inst;
}

/*! 
 * @fn cir_free(cir_instance_t * inst)
 *
 * @brief Free resources and restore default behaviour. 
 *
 * input parameters
 * @param inst - cir_instance_t * inst
 *
 * output parameters
 *
 * returns none
 */
void 
cir_free(cir_instance_t * inst){
    assert(inst);  
    if (inst->status.selfmalloc)
        free(inst);
    else
        inst->status.initialized = 0;
}

dw1000_mac_interface_t cbs[] = {
    [0] = {
            .id =  DW1000_CIR,
            .cir_complete_cb = cir_complete_cb
    },
#if MYNEWT_VAL(DW1000_DEVICE_1)
    [1] = {
            .id =  DW1000_CIR,
            .cir_complete_cb = cir_complete_cb
    },
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    [2] = {
            .id =  DW1000_CIR,
            .cir_complete_cb = cir_complete_cb
    }
#endif
};

/**
 * API to initialise the cip package.
 * @return void
 */

void cir_pkg_init(void){

    printf("{\"utime\": %lu,\"msg\": \"cir_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

    dw1000_dev_instance_t * inst = hal_dw1000_inst(0);
    inst->cir = cir_init(NULL);
    dw1000_mac_append_interface(inst, &cbs[0]);

#if MYNEWT_VAL(DW1000_DEVICE_1)
    inst = hal_dw1000_inst(1);
    inst->cir = cir_init(NULL);
    dw1000_mac_append_interface(inst, &cbs[1]);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    inst = hal_dw1000_inst(1);
    inst->cir = cir_init(NULL);
    dw1000_mac_append_interface(inst, &cbs[2]);
#endif
  
}




