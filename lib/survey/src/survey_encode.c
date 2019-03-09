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
#include <nrng/nrng_encode.h>
#include <survey/survey_encode.h>

#if MYNEWT_VAL(SURVEY_VERBOSE)

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

/**
 * API for verbose JSON logging of survey resultss
 * 
 * @param survey survey_instance_t point
 * @param seq_num survey
 * @return none.
 */
void 
survey_encode(survey_instance_t * survey, uint16_t seq, uint16_t idx){
 
    struct json_encoder encoder;
    struct json_value value;
    int rc;
    uint32_t utime = os_cputime_ticks_to_usecs(os_cputime_get32());
    survey_nrngs_t * nrngs = survey->nrngs[idx%survey->nframes];

    uint32_t mask = 0;
    // Workout which node responded to the request
    for (uint16_t i=0; i < survey->nnodes; i++){
        if (nrngs->nrng[i]->mask){
                mask |= 1UL << i;
        }
    }

    survey->status.empty = NumberOfBits(mask) == 0;
    if (survey->status.empty)
       return;
       
    /* reset the state of the internal test */
    memset(&encoder, 0, sizeof(encoder));
    encoder.je_write = json_write;
    encoder.je_arg= NULL;

    rc = json_encode_object_start(&encoder); 
    JSON_VALUE_INT(&value,  utime);
    rc |= json_encode_object_entry(&encoder, "utime", &value);   
    rc |= json_encode_object_key(&encoder, "survey");
    rc |= json_encode_object_start(&encoder);    
  
    JSON_VALUE_UINT(&value, seq);
    rc |= json_encode_object_entry(&encoder, "seq", &value);
    
    JSON_VALUE_UINT(&value, mask);
    rc |= json_encode_object_entry(&encoder, "mask", &value);
    rc |= json_encode_object_key(&encoder, "nrngs");
    rc |= json_encode_array_start(&encoder);
   
    for (uint16_t i=0; i < survey->nnodes; i++){
        if (nrngs->nrng[i]->mask){
            JSON_VALUE_UINT(&value, nrngs->nrng[i]->mask);
             rc |= json_encode_object_start(&encoder); 
            rc |= json_encode_object_entry(&encoder, "mask", &value);
            rc |= json_encode_array_name(&encoder, "nrng");
            rc |= json_encode_array_start(&encoder);
            for (uint16_t j=0; j < NumberOfBits(nrngs->nrng[i]->mask); j++){
                JSON_VALUE_UINT(&value, *(uint32_t *)&nrngs->nrng[i]->rng[j]);
                rc |= json_encode_array_value(&encoder, &value); 
            }
            rc |= json_encode_array_finish(&encoder); 
            rc |= json_encode_object_finish(&encoder);
        }
    }
    rc |= json_encode_array_finish(&encoder);  
    rc |= json_encode_object_finish(&encoder);
    rc |= json_encode_object_finish(&encoder);
    assert(rc == 0);
    json_fflush();
}

#endif


