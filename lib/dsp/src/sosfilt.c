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

#include <assert.h>
#include <dsp/sosfilt.h>

sos_instance_t * sosfilt_init(sos_instance_t * inst, uint16_t nsize) {

    if (inst == NULL){
		inst = (sos_instance_t *) malloc(sizeof(sos_instance_t) + nsize * sizeof(biquad_instance_t *) ); 
        assert(inst);
        memset(inst, 0, sizeof(sos_instance_t));
		inst->status.selfmalloc = 1;
        for (uint8_t i=0;i < nsize; i++){
            inst->biquads[i] = biquad_init(NULL);
        }
	}else{
		assert(inst->nsize == nsize);
    }
    inst->nsize = nsize;
    return inst;
}

void sosfilt_free(sos_instance_t * inst){
    assert(inst);
    for (uint8_t i=0;i < inst->nsize; i++)
        biquad_free(inst->biquads[i]);
}

float sosfilt(sos_instance_t * inst, float x, float b[], float a[]) {
    
	float result=x;
    
	for (uint8_t i=0; i < inst->nsize; i++)
		result = biquad(inst->biquads[i], result, &b[i*BIQUAD_N], &a[i*BIQUAD_N], inst->clk);

    inst->clk++;
    
    return result;
}






