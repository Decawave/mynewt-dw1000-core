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
#include <stats/stats.h>

#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_stats.h>
#include <cir/cir.h>
#include <cir/cir_encode.h>

#if MYNEWT_VAL(CIR_STATS)
STATS_NAME_START(cir_stat_section)
    STATS_NAME(cir_stat_section, complete)
STATS_NAME_END(cir_stat_section)
#define CIR_STATS_INC(__X) STATS_INC(cir->stat, __X)
#else
#define CIR_STATS_INC(__X) {}
#endif

#if MYNEWT_VAL(CIR_VERBOSE) 

struct os_event cir_event;
static void
cir_complete_ev_cb(struct os_event *ev) {
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);
    
    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    cir_t * cir  = &inst->cir->cir;
    if(inst->uwb_dev.config.rxdiag_enable){
        for (uint16_t i=0; i < MYNEWT_VAL(CIR_SIZE); i++){
            cir->array[i].real /= inst->rxdiag.pacc_cnt;
            cir->array[i].imag /= inst->rxdiag.pacc_cnt;
        }
    }
    inst->cir->fp_power = dw1000_get_fppl(inst);

#if  MYNEWT_VAL(DW1000_DEVICE_0) && !MYNEWT_VAL(DW1000_DEVICE_1)
    cir_encode(inst->cir, "cir", MYNEWT_VAL(CIR_SIZE));
#elif  MYNEWT_VAL(DW1000_DEVICE_0) && MYNEWT_VAL(DW1000_DEVICE_1)
    if (inst->uwb_dev.idx == 0)
        cir_encode(inst->cir, "cir0", MYNEWT_VAL(CIR_SIZE));   
    else     
        cir_encode(inst->cir, "cir1", MYNEWT_VAL(CIR_SIZE)); 
#endif
}

#endif //CIR_VERBOSE


/*! 
 * @fn cir_remap_fp_index(cir_instance_t *cir0, cir_instance_t *cir1)
 *
 * @brief Map cir0's fp_idx into cir1 and return it
 * 
 * input parameters
 * @param cir0 - The cir which fp_index will be remapped
 * @param cir1 - The mapping target cir
 *
 * output parameters
 *
 * returns float cir0.fp_idx as it would have been inside cir1
 */
float
cir_remap_fp_index(cir_instance_t *cir0, cir_instance_t *cir1)
{
    /* Correct aligment using raw timestamp and resampler delays */
    double raw_ts_diff = ((int64_t) cir0->raw_ts + ((int64_t)cir0->resampler_delay)*8 -
                           ((int64_t)cir1->raw_ts + ((int64_t)cir1->resampler_delay)*8))/64.0;

    /* Compensate for different clock and rx-antenna delays */
    if (cir0->dev_inst && cir1->dev_inst) {
        raw_ts_diff += ((int32_t)cir0->dev_inst->uwb_dev.ext_clock_delay  - (int32_t)cir1->dev_inst->uwb_dev.ext_clock_delay)/64.0f;
        raw_ts_diff += ((int32_t)cir0->dev_inst->uwb_dev.rx_antenna_delay - (int32_t)cir1->dev_inst->uwb_dev.rx_antenna_delay)/64.0f;
    }

    float fp_idx_0_given_1 = cir0->fp_idx + raw_ts_diff;
    return fp_idx_0_given_1;
}

bool
cir_reread_from_cir(dw1000_dev_instance_t * inst, cir_instance_t *master_cir)
{
    /* CIR-data is lost already if the receiver has been turned back on */
    if (inst->uwb_dev.status.rx_restarted) {
        return false;
    }
    cir_instance_t * cir = inst->cir;

    /* Correct aligment by remapping master_cir's fp_idx into our cir */
    float fp_idx_override  = cir_remap_fp_index(master_cir, cir);

    /* Sanity check, only a fp_index within the accumulator makes sense */
    if(fp_idx_override < MYNEWT_VAL(CIR_OFFSET) || (fp_idx_override + MYNEWT_VAL(CIR_SIZE)) > 1023) {
        /* Can't extract CIR from required offset, abort */
        return false;
    }

    /* Override our local LDE result with the other LDE's result */
    cir->status.lde_override = 1;
    uint16_t fp_idx = floor(fp_idx_override + 0.5f);

    dw1000_read_accdata(inst, (uint8_t *)&cir->cir, (fp_idx - MYNEWT_VAL(CIR_OFFSET)) * sizeof(cir_complex_t), sizeof(cir_t));

    /* No need to re-read rc-phase, it hasn't changed */
    cir->angle = atan2f((float)cir->cir.array[MYNEWT_VAL(CIR_OFFSET)].imag, (float)cir->cir.array[MYNEWT_VAL(CIR_OFFSET)].real);
    cir->status.valid = 1;
    return true;
}

/*! 
 * @fn cir_complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
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
#if MYNEWT_VAL(CIR_ENABLED)
static bool
cir_complete_cb(struct uwb_dev * udev, struct uwb_mac_interface * cbs)
{
    cir_instance_t * cir = (cir_instance_t *)cbs->inst_ptr;
    struct _dw1000_dev_instance_t *inst = (struct _dw1000_dev_instance_t *)udev;

    cir->status.valid = 0;
    CIR_STATS_INC(complete);
    
    cir->raw_ts = dw1000_read_rawrxtime(cir->dev_inst);
    cir->resampler_delay = dw1000_read_reg(inst, RX_TTCKO_ID, 3, sizeof(uint8_t));

    uint16_t fp_idx;
    uint16_t fp_idx_reg = inst->rxdiag.fp_idx;
    if(!inst->uwb_dev.config.rxdiag_enable) {
        fp_idx_reg = dw1000_read_reg(cir->dev_inst, RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET, sizeof(uint16_t));
    }
    cir->fp_idx = (float)fp_idx_reg / 64.0f;
    fp_idx  = (uint16_t)floorf(cir->fp_idx + 0.5f);

    if (inst->uwb_dev.config.cir_pdoa_slave) {
        /* This unit is acting as part of a pdoa network of receivers.
         * instead of trusting the LDE of this unit, use the LDE that detected
         * the earliest first path. This assumes all receivers are within 30mm of
         * each other (or rather their antennas).
         * */
        cir_instance_t * master_cir = hal_dw1000_inst(0)->cir;
        float fp_idx_from_master  = cir_remap_fp_index(master_cir, cir);

        /* Check if our first path comes before the master's first path.
         * If so, reread the master's CIR data if possible */
        if (fp_idx_from_master - cir->fp_idx > MYNEWT_VAL(CIR_PDOA_SLAVE_MAX_LEAD)) {
            bool b = cir_reread_from_cir(hal_dw1000_inst(0), cir);
            if (!b) {
                return true;
            }
        } else {
            /* Override our local LDE with master's LDE */
            cir->status.lde_override = 1;
            fp_idx = (uint16_t)floorf(fp_idx_from_master + 0.5f);
        }
    }

    /* Sanity check, only a fp_index within the accumulator makes sense */
    if(fp_idx < MYNEWT_VAL(CIR_OFFSET) || (fp_idx + MYNEWT_VAL(CIR_SIZE)) > 1023) {
        /* Can't extract CIR from required offset, abort */
        return true;
    }
    dw1000_read_accdata(cir->dev_inst, (uint8_t *)&cir->cir, (fp_idx - MYNEWT_VAL(CIR_OFFSET)) * sizeof(cir_complex_t), sizeof(cir_t));

    float _rcphase = (float)((uint8_t)dw1000_read_reg(cir->dev_inst, RX_TTCKO_ID, 4, sizeof(uint8_t)) & 0x7F);
    cir->rcphase = _rcphase * (M_PI/64.0f);
    cir->angle = atan2f((float)cir->cir.array[MYNEWT_VAL(CIR_OFFSET)].imag, (float)cir->cir.array[MYNEWT_VAL(CIR_OFFSET)].real);
    cir->status.valid = 1;

#if MYNEWT_VAL(CIR_VERBOSE)
    cir_event.ev_cb  = cir_complete_ev_cb;
    cir_event.ev_arg = (void*) inst;
    os_eventq_put(os_eventq_dflt_get(), &cir_event);
#endif
    return false;
}
#endif // MYNEWT_VAL(CIR_ENABLED)

/*! 
 * @fn cir_enable(cir_instance_t * inst, bool mode){
 *
 * @brief Enable CIR reading of CIR. 
 * 
 * @param cir - cir_instance_t *
 * @param mode - bool
 * 
 * output parameters
 *
 * returns void
 */

void  
cir_enable(struct _cir_instance_t * cir, bool mode)
{
#if MYNEWT_VAL(CIR_ENABLED)
    dw1000_dev_instance_t * inst = cir->dev_inst;
    cir->status.valid = 0;
    inst->control.cir_enable = mode;
#endif
}

/*! 
 * @fn cir_get_pdoa(cir_instance_t * master, cir_instance *slave)
 *
 * @brief Calculate the phase difference between receivers
 * 
 * @param master - cir_instance_t *
 * @param slave  - cir_instance_t *
 * 
 * output parameters
 *
 * returns phase_difference - float 
 */
float
cir_get_pdoa(cir_instance_t * master, cir_instance_t *slave)
{
    return fmodf((slave->angle - slave->rcphase) - (master->angle - master->rcphase) + 3*M_PI, 2*M_PI) - M_PI;
}


/*! 
 * @fn cir_calc_aoa(float pdoa, float wavelength, float antenna_separation)
 *
 * @brief Calculate the phase difference between receivers
 * 
 * @param pdoa       - phase difference of arrival in radians
 * @param wavelength - wavelength of the radio channel used in meters
 * @param antenna_separation - distance between centers of antennas in meters
 * 
 * output parameters
 *
 * returns angle of arrival - float, in radians
 */
float
cir_calc_aoa(float pdoa, float wavelength, float antenna_separation)
{
    float pd_dist = pdoa / (2.0f*M_PI) * wavelength;
    return asinf(pd_dist / antenna_separation);
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
cir_init(struct _dw1000_dev_instance_t * inst, struct _cir_instance_t * cir)
{
    if (cir == NULL) {
        cir = (cir_instance_t *) malloc(sizeof(cir_instance_t)); 
        assert(cir);
        memset(cir, 0, sizeof(cir_instance_t));
        cir->status.selfmalloc = 1;
    }
    cir->dev_inst = inst;

#if MYNEWT_VAL(CIR_STATS)
    int rc = stats_init(
                STATS_HDR(cir->stat),
                STATS_SIZE_INIT_PARMS(cir->stat, STATS_SIZE_32),
                STATS_NAME_INIT_PARMS(cir_stat_section)
            );

#if  MYNEWT_VAL(DW1000_DEVICE_0) && !MYNEWT_VAL(DW1000_DEVICE_1)
    rc |= stats_register("cir", STATS_HDR(cir->stat));
#elif  MYNEWT_VAL(DW1000_DEVICE_0) && MYNEWT_VAL(DW1000_DEVICE_1)
    if (inst == hal_dw1000_inst(0))
        rc |= stats_register("cir0", STATS_HDR(cir->stat));
    else
        rc |= stats_register("cir1", STATS_HDR(cir->stat));
#endif
    assert(rc == 0);
#endif
    return cir;
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

#if MYNEWT_VAL(CIR_ENABLED)
struct uwb_mac_interface cbs[] = {
    [0] = {
            .id =  UWBEXT_CIR,
            .cir_complete_cb = cir_complete_cb
    },
#if MYNEWT_VAL(DW1000_DEVICE_1)
    [1] = {
            .id =  UWBEXT_CIR,
            .cir_complete_cb = cir_complete_cb
    },
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    [2] = {
            .id =  UWBEXT_CIR,
            .cir_complete_cb = cir_complete_cb
    }
#endif
};
#endif // MYNEWT_VAL(CIR_ENABLED)


/**
 * API to initialise the cir package.
 * @return void
 */
void cir_pkg_init(void)
{
#if MYNEWT_VAL(CIR_ENABLED)
    printf("{\"utime\": %lu,\"msg\": \"cir_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));

    dw1000_dev_instance_t * inst = hal_dw1000_inst(0);
    cbs[0].inst_ptr = inst->cir = cir_init(inst, NULL);
    uwb_mac_append_interface(&inst->uwb_dev, &cbs[0]);

#if MYNEWT_VAL(DW1000_DEVICE_1)
    inst = hal_dw1000_inst(1);
    cbs[1].inst_ptr = inst->cir = cir_init(inst, NULL);
    uwb_mac_append_interface(&inst->uwb_dev, &cbs[1]);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    inst = hal_dw1000_inst(2);
    cbs[2].inst_ptr = inst->cir = cir_init(inst, NULL);
    uwb_mac_append_interface(&inst->uwb_dev, &cbs[2]);
#endif

#endif // MYNEWT_VAL(CIR_ENABLED)
}



