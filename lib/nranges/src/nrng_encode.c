/**
 * Copyright (C) 2017-2018, Decawave Limited, All Rights Reserved
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
#include <string.h>
#include <stdio.h>
#include <rng/rng.h>
#include <dw1000/dw1000_mac.h>
#include <nranges/nrng_encode.h>

#if MYNEWT_VAL(NRNG_VERBOSE)

/*! 
 * @fn nrng_encode struct os_event * ev)
 *
 * @brief JSON encoding of range
 * 
 * input parameters
 * @param inst - struct os_event *  
 * output parameters
 * returns none 
 */
void 
nrng_encode(dw1000_nrng_instance_t * nrng) {

    nrng_frame_t *  frame = (nrng_frame_t * ) nrng->frames[(nrng->idx)%(nrng->nframes/FRAMES_PER_RANGE)][FIRST_FRAME_IDX];
    uint16_t slot_idx = frame->slot_id;
        
    if (frame->code ==  DWT_SS_TWR_NRNG_T1) {
        printf("{\"utime\": %lu,\"slot_id\": [%u,%u],\"reception\": \"%lX\",\"transmission\": \"%lX\"}\n",
            os_cputime_ticks_to_usecs(os_cputime_get32()),
            slot_idx, frame->seq_num,
            frame->reception_timestamp,
            frame->transmission_timestamp
            );
    }
}

#endif



