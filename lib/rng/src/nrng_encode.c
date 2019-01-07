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
#include <json/json.h>
#include <dw1000/dw1000_mac.h>
#include <rng/nrng_encode.h>

#if MYNEWT_VAL(NRNG_VERBOSE)

#define JSON_BUF_SIZE (1024)
static char _buf[JSON_BUF_SIZE];
static uint16_t idx=0;

static void
json_fflush(){
    _buf[idx] = '\0';
    printf("%s\n", _buf);
    idx=0;
}

static void
_json_fflush(){
    _buf[idx] = '\0';
    printf("%s", _buf);
    idx=0;
}

static int
json_write(void *buf, char* data, int len) {

    if (idx + len > JSON_BUF_SIZE) 
        _json_fflush();

    for (uint16_t i=0; i< len; i++)
        _buf[i+idx] = data[i];
    idx+=len;

    return len;
}


void 
nrng_encode(dw1000_nrng_instance_t * nrng, uint8_t seq_num, uint16_t base){
 
    struct json_encoder encoder;
    struct json_value value;
    int rc;
    uint32_t utime = os_cputime_ticks_to_usecs(os_cputime_get32());
    uint32_t valid_mask = 0;

    // Workout which slots responded with a valid frames
    for (uint16_t i=0; i < 16; i++){
        if (nrng->slot_mask & 1UL << i){
            uint16_t idx = BitIndex(nrng->slot_mask, 1UL << i, SLOT_POSITION); 
            nrng_frame_t * frame = nrng->frames[(base + idx)%(nrng->nframes/FRAMES_PER_RANGE)][FIRST_FRAME_IDX];
            if (frame->code == DWT_SS_TWR_NRNG_FINAL && frame->seq_num == seq_num){
                valid_mask |= 1UL << i;
            }
        }
    }

    if (valid_mask == 0) 
        return;

    /* reset the state of the internal test */
    memset(&encoder, 0, sizeof(encoder));
    encoder.je_write = json_write;
    encoder.je_arg= NULL;

    rc = json_encode_object_start(&encoder); 
    JSON_VALUE_INT(&value,  utime);
    rc |= json_encode_object_entry(&encoder, "utime", &value);   
    rc |= json_encode_object_key(&encoder, "nrng");
    rc |= json_encode_object_start(&encoder);    
  
    JSON_VALUE_UINT(&value, seq_num);
    rc |= json_encode_object_entry(&encoder, "seq_num", &value);

    JSON_VALUE_UINT(&value, valid_mask);
    rc |= json_encode_object_entry(&encoder, "mask", &value);
    rc |= json_encode_array_name(&encoder, "rng");
    rc |= json_encode_array_start(&encoder);
    for (uint16_t i=0; i < 16; i++){
        if (valid_mask & 1UL << i){
            uint16_t idx = BitIndex(nrng->slot_mask, 1UL << i, SLOT_POSITION); 
            nrng_frame_t * frame = nrng->frames[(base + idx)%(nrng->nframes/FRAMES_PER_RANGE)][FIRST_FRAME_IDX];
            if (frame->code == DWT_SS_TWR_NRNG_FINAL && frame->seq_num == seq_num){
                float range = dw1000_rng_tof_to_meters(dw1000_nrng_twr_to_tof_frames(nrng->parent, frame, frame));
                JSON_VALUE_UINT(&value, *(uint32_t *)&range);
                rc |= json_encode_array_value(&encoder, &value); 
                if (i%64==0) _json_fflush();
            }
        }
    }
    rc |= json_encode_array_finish(&encoder);  
    rc |= json_encode_array_name(&encoder, "tdoa");
    rc |= json_encode_array_start(&encoder);
    for (uint16_t i=0; i < 16; i++){
        if (valid_mask & 1UL << i){
            uint16_t idx = BitIndex(nrng->slot_mask , 1UL << i, SLOT_POSITION); 
            nrng_frame_t * frame = nrng->frames[(base + idx)%(nrng->nframes/FRAMES_PER_RANGE)][FIRST_FRAME_IDX];
            if (frame->code == DWT_SS_TWR_NRNG_FINAL && frame->seq_num == seq_num){
                JSON_VALUE_INT(&value, (int32_t)(frame->reception_timestamp - frame->request_timestamp) );
                rc |= json_encode_array_value(&encoder, &value); 
                if (i%64==0) _json_fflush();
            }
        }
    }
    rc |= json_encode_array_finish(&encoder);    
    rc |= json_encode_object_finish(&encoder);
    rc |= json_encode_object_finish(&encoder);
    assert(rc == 0);
    json_fflush();
}




#endif


