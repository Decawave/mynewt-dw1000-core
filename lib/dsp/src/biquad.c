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

#include <dsp/biquad.h>
#include <stdio.h>
#include <assert.h>

/**
 * Initilize biquad
 */
biquad_instance_t * biquad_init(biquad_instance_t * inst) {

	if (inst == NULL){
		inst = (biquad_instance_t *) malloc(sizeof(biquad_instance_t)); 
        assert(inst);
        memset(inst, 0, sizeof(biquad_instance_t));
		inst->status.selfmalloc = 1;
	}
	return inst;
}

void biquad_free(biquad_instance_t * inst) {
	
	if(inst->status.selfmalloc)
		free(inst);
}


float biquad(biquad_instance_t * inst, float x, float b[], float a[], uint16_t clk) {

	float y = 0;
	// Implementing y(n) = (-a1 y(n-1) -a2 y(n-2) + b0 x(n) + b1 x(n-1) + b2 x(n-2))/a0
	// taps := [y(n), y(n-1), y(n-2), x(n), x(n-1), x(n-2)]
	// b:= [b0, b1, b2]
	// a:= [a0, a1, a2]

	inst->num[clk%BIQUAD_N] = x;

	for (int16_t i=0; i < BIQUAD_N; i++){
		y += b[i] * inst->num[(clk-i+BIQUAD_N)%(BIQUAD_N)];// b0 x(n) + b1 x(n-1) + b2 x(n-2)
	}	

	for (int16_t i=1; i < BIQUAD_N; i++){
		y -= a[i] * inst->den[(clk-i+BIQUAD_N)%(BIQUAD_N)];//         - a1 y(n-1) - a2 y(n-2)
	}
	y = y/a[0];

	inst->den[clk%BIQUAD_N] = y;
    
    return y;
}






