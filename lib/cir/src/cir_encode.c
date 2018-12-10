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
#include <cir/cir_encode.h>
#include <cir/cir.h>

#if MYNEWT_VAL(CIR_VERBOSE)

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
    // write(STDOUT_FILENO, data, len);  TODOs: This is the prefered approach

    if (idx + len > JSON_BUF_SIZE) 
        _json_fflush();

    for (uint16_t i=0; i< len; i++)
        _buf[i+idx] = data[i];
    idx+=len;

    return len;
}

void 
cir_encode(cir_instance_t * cir, char * name, uint16_t nsize){

    struct json_encoder encoder;
    struct json_value value;
    int rc;
    uint32_t utime = os_cputime_ticks_to_usecs(os_cputime_get32());

    /* reset the state of the internal test */
    memset(&encoder, 0, sizeof(encoder));
    encoder.je_write = json_write;
    encoder.je_arg= NULL;

    rc = json_encode_object_start(&encoder); 
    JSON_VALUE_INT(&value,  utime);
    rc |= json_encode_object_entry(&encoder, "utime", &value);   
  
    rc |= json_encode_object_key(&encoder, name);
    rc |= json_encode_object_start(&encoder);    

    JSON_VALUE_UINT(&value, *(uint32_t *)&cir->fp_idx);
    rc |= json_encode_object_entry(&encoder, "idx", &value);

    JSON_VALUE_UINT(&value, *(uint32_t *)&cir->fp_power);
    rc |= json_encode_object_entry(&encoder, "power", &value);

    rc |= json_encode_array_name(&encoder, "real");
    rc |= json_encode_array_start(&encoder);
    for (uint16_t i=0; i< nsize; i++){
        JSON_VALUE_INT(&value, cir->cir.array[i].real);
        rc |= json_encode_array_value(&encoder, &value); 
        if (i%32==0) _json_fflush();
    }
    rc |= json_encode_array_finish(&encoder);  

    rc |= json_encode_array_name(&encoder, "imag");
    rc |= json_encode_array_start(&encoder);
    for (uint16_t i=0; i< nsize; i++){
        JSON_VALUE_INT(&value, cir->cir.array[i].imag);
        rc |= json_encode_array_value(&encoder, &value);
        if (i%32==0) _json_fflush();
    }
    rc |= json_encode_array_finish(&encoder);    
    rc |= json_encode_object_finish(&encoder);
    rc |= json_encode_object_finish(&encoder);
    assert(rc == 0);
    json_fflush();
}

void 
pmem_encode(cir_instance_t * cir, char * name, uint16_t nsize){

    struct json_encoder encoder;
    struct json_value value;
    int rc;
    uint32_t utime = os_cputime_ticks_to_usecs(os_cputime_get32());

    /* reset the state of the internal test */
    memset(&encoder, 0, sizeof(encoder));
    encoder.je_write = json_write;
    encoder.je_arg= NULL;

    rc = json_encode_object_start(&encoder); 
    JSON_VALUE_INT(&value,  utime);
    rc |= json_encode_object_entry(&encoder, "utime", &value);   
  
    rc |= json_encode_object_key(&encoder, name);
    rc |= json_encode_object_start(&encoder);    

    rc |= json_encode_array_name(&encoder, "real");
    rc |= json_encode_array_start(&encoder);
    for (uint16_t i=0; i< nsize; i++){
        JSON_VALUE_INT(&value, cir->pmem.array[i].real);
        rc |= json_encode_array_value(&encoder, &value); 
        if (i%32==0) _json_fflush();
    }
    rc |= json_encode_array_finish(&encoder);  

    rc |= json_encode_array_name(&encoder, "imag");
    rc |= json_encode_array_start(&encoder);
    for (uint16_t i=0; i< nsize; i++){
        JSON_VALUE_INT(&value, cir->pmem.array[i].imag);
        rc |= json_encode_array_value(&encoder, &value);
        if (i%32==0) _json_fflush();
    }
    rc |= json_encode_array_finish(&encoder);    
    rc |= json_encode_object_finish(&encoder);
    rc |= json_encode_object_finish(&encoder);
    assert(rc == 0);
    json_fflush();
}


#endif



