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
#include <rng/rng_encode.h>

#if MYNEWT_VAL(RNG_VERBOSE)

/*! 
 * @fn rng_encodestruct os_event * ev)
 *
 * @brief JSON encoding of range
 * 
 * input parameters
 * @param inst - struct os_event *  
 * output parameters
 * returns none 
 */
void 
rng_encode(dw1000_rng_instance_t * rng) {

    uint16_t idx = (rng->idx)%rng->nframes;
    twr_frame_t * frame = rng->frames[idx];

    if (frame->code == DWT_SS_TWR_FINAL) {
        float time_of_flight = (float) dw1000_rng_twr_to_tof(rng, idx);
        float range = dw1000_rng_tof_to_meters(time_of_flight);
        printf("{\"utime\": %lu,\"tof\": %lu,\"range\": %lu,\"res_req\": \"%lX\","
                " \"rec_tra\": \"%lX\"}\n",
                os_cputime_ticks_to_usecs(os_cputime_get32()),
                *(uint32_t *)(&time_of_flight), 
                *(uint32_t *)(&range),
                (frame->response_timestamp - frame->request_timestamp),
                (frame->transmission_timestamp - frame->reception_timestamp)
        );
        frame->code = DWT_SS_TWR_END;
    }
    else if (frame->code == DWT_DS_TWR_FINAL) {
        float time_of_flight = dw1000_rng_twr_to_tof(rng, idx);
        float range = dw1000_rng_tof_to_meters(time_of_flight);
        printf("{\"utime\": %lu,\"tof\": %lu,\"range\": %lu,\"azimuth\": %lu,\"res_req\":\"%lX\","
                " \"rec_tra\": \"%lX\"}\n",
                os_cputime_ticks_to_usecs(os_cputime_get32()), 
                *(uint32_t *)(&time_of_flight), 
                *(uint32_t *)(&range),
                *(uint32_t *)(&frame->spherical.azimuth),
                (frame->response_timestamp - frame->request_timestamp),
                (frame->transmission_timestamp - frame->reception_timestamp)
        );
        frame->code = DWT_DS_TWR_END;
    } 
    else if (frame->code == DWT_DS_TWR_EXT_FINAL) {
        float time_of_flight = dw1000_rng_twr_to_tof(rng, idx);
        printf("{\"utime\": %lu,\"tof\": %lu,\"range\": %lu,\"azimuth\": %lu,\"res_req\":\"%lX\","
                " \"rec_tra\": \"%lX\"}\n",
                os_cputime_ticks_to_usecs(os_cputime_get32()), 
                *(uint32_t *)(&time_of_flight), 
                *(uint32_t *)(&frame->spherical.range),
                *(uint32_t *)(&frame->spherical.azimuth),
                (frame->response_timestamp - frame->request_timestamp),
                (frame->transmission_timestamp - frame->reception_timestamp)
        );
        frame->code = DWT_DS_TWR_END;
    } 
}

#endif



