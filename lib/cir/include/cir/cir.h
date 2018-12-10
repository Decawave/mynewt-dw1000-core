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

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _cir_status_t{
    uint16_t selfmalloc:1;
    uint16_t initialized:1;
    uint16_t valid:1;
}cir_status_t;

typedef struct _cir_control_t{
    uint16_t cir_enable:1;
    uint16_t pmem_enable:1;
}cir_control_t;

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
}cir_t;

typedef struct _pmem_t{
    uint8_t dummy;  //Errata
    struct _cir_complex_t array[MYNEWT_VAL(PMEM_SIZE)]; 
}pmem_t;

typedef struct _cir_instance_t{
    cir_status_t status;
    cir_control_t control;
    uint16_t fp_amp1;
    float fp_idx;
    float fp_power;
    float rcphase;
    float angle;
    cir_t cir;
    pmem_t pmem;
}cir_instance_t; 

cir_instance_t * cir_init(cir_instance_t * inst);
cir_instance_t * cir_enable(cir_instance_t * inst, bool mode);
cir_instance_t * pmem_enable(cir_instance_t * inst, bool mode);
void cir_free(cir_instance_t * inst);

#ifdef __cplusplus
}
#endif

#endif /* _CIR_H_ */
