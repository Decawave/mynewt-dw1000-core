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

#ifndef _CIR_H_
#define _CIR_H_

#include <stdlib.h>
#include <stdint.h>
#include <os/os.h>
#include <stats/stats.h>
#include <dw1000/dw1000_dev.h>

#ifdef __cplusplus
extern "C" {
#endif

#if MYNEWT_VAL(CIR_STATS)
STATS_SECT_START(cir_stat_section)
    STATS_SECT_ENTRY(complete)
STATS_SECT_END
#endif

typedef struct _cir_status_t{
    uint16_t selfmalloc:1;
    uint16_t initialized:1;
    uint16_t valid:1;
}cir_status_t;

typedef union{
    struct  _cir_complex_t{
        int16_t real;
        int16_t imag;
    }__attribute__((__packed__));
    uint8_t array[sizeof(struct _cir_complex_t)];
}cir_complex_t;

typedef struct _cir_t{
    uint8_t dummy;  //Errata
    struct _cir_complex_t array[MYNEWT_VAL(CIR_SIZE)]; 
} __attribute__((packed, aligned(1))) cir_t;


typedef struct _cir_instance_t{
    struct _dw1000_dev_instance_t * dev_inst; //!< Structure of DW1000_dev_instance
#if MYNEWT_VAL(CIR_STATS)
    STATS_SECT_DECL(cir_stat_section) stat; //!< Stats instance
#endif
    cir_status_t status;
    uint16_t fp_amp1;
    float fp_idx;
    float fp_power;
    float rcphase;
    float angle;
    uint64_t raw_ts;
    cir_t cir;
}cir_instance_t; 

cir_instance_t * cir_init(struct _dw1000_dev_instance_t * inst, struct _cir_instance_t * cir);
void cir_enable(struct _cir_instance_t * inst, bool mode);
void cir_free(struct _cir_instance_t * inst);
float cir_get_pdoa(struct _cir_instance_t * master, struct _cir_instance_t *slave);
float cir_calc_aoa(float pdoa, float wavelength, float antenna_separation);

#ifdef __cplusplus
}
#endif

#endif /* _CIR_H_ */
