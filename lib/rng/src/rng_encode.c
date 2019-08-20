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
#include <json/json.h>
#include <rng/rng.h>
#include <dw1000/dw1000_mac.h>
#include <rng/rng_encode.h>

#if MYNEWT_VAL(WCS_ENABLED)
#include <wcs/wcs.h>
#endif

#if MYNEWT_VAL(RNG_VERBOSE)

#define JSON_BUF_SIZE (128)
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

/*!
 * @fn rng_encodestruct os_event * ev)
 *
 * @brief JSON encoding of range
 *
 * input parameters
 * @param rng     Pointer of dw1000_rng_instance_t.
 * output parameters
 * returns void
 */
void
rng_encode(dw1000_rng_instance_t * rng) {

    struct json_encoder encoder;
    struct json_value value;
    int rc;

    memset(&encoder, 0, sizeof(encoder));
    encoder.je_write = json_write;
    encoder.je_arg= NULL;
    
    uint16_t idx = (rng->idx)%rng->nframes;
    twr_frame_t * frame = rng->frames[idx];
    
    float time_of_flight = dw1000_rng_twr_to_tof(rng, idx);
    frame->spherical.range = dw1000_rng_tof_to_meters(time_of_flight);

    rc = json_encode_object_start(&encoder);
#if MYNEWT_VAL(WCS_ENABLED)
    wcs_instance_t * wcs = (wcs_instance_t *)dw1000_mac_find_cb_inst_ptr(rng->dev_inst, DW1000_WCS);
    JSON_VALUE_UINT(&value, wcs_local_to_master(wcs, frame->reception_timestamp));
#else
    JSON_VALUE_UINT(&value, os_cputime_ticks_to_usecs(os_cputime_get32()));
#endif
    rc |= json_encode_object_entry(&encoder, "utime", &value);
    _json_fflush();
    printf(", ");
    
    frame->code = DWT_SS_TWR_FINAL;
    if (frame->code == DWT_SS_TWR_FINAL) {
        _twr_encode(frame);
        frame->code = DWT_SS_TWR_END;
    }
    else if (frame->code == DWT_DS_TWR_FINAL) {
        _twr_encode(frame);
        frame->code = DWT_DS_TWR_END;
    }
    else if (frame->code == DWT_DS_TWR_EXT_FINAL) {
        _raz_encode(frame);
        frame->code = DWT_DS_TWR_END;
    }

    rc |= json_encode_object_finish(&encoder);
    assert(rc == 0);
    json_fflush();
}

/*!
 * @fn twr_encode(twr_frame_t * frame) {
 *
 * @brief JSON encoding twr_frames support the folowing json objects: 
 * {"twr": {"rng": "1.100","uid": "1234"},"uid": "4321"}
 * input parameters
 * @param frame twr_frame_t *
 * output parameters
 * returns void
 */

void
_twr_encode(twr_frame_t * frame) {
    struct json_encoder encoder;
    struct json_value value;
    int rc;

    memset(&encoder, 0, sizeof(encoder));
    encoder.je_write = json_write;
    encoder.je_arg= NULL;

    rc = json_encode_object_key(&encoder, "twr");  
    rc |= json_encode_object_start(&encoder);  

#if MYNEWT_VAL(FLOAT_USER)
    char float_string[32];
    sprintf(float_string,"%f",frame->spherical.range);
    JSON_VALUE_STRING(&value, float_string);
#else
    JSON_VALUE_UINT(&value, *(uint32_t *)&frame->spherical.range);
#endif
    rc |= json_encode_object_entry(&encoder, "rng", &value);
   
    char uuid[16];
    sprintf(uuid,"%04x",frame->dst_address);
    JSON_VALUE_STRINGN(&value, uuid,4);
    rc |= json_encode_object_entry(&encoder, "uid", &value);
    rc |= json_encode_object_finish(&encoder);

    sprintf(uuid,"%04x",frame->src_address);
    JSON_VALUE_STRINGN(&value, uuid,4);
    rc |= json_encode_object_entry(&encoder, "uid", &value);

    assert(rc == 0);
    _json_fflush();
   
}

void
twr_encode(twr_frame_t * frame){

    struct json_encoder encoder;
    int rc;

    memset(&encoder, 0, sizeof(encoder));
    encoder.je_write = json_write;
    encoder.je_arg= NULL;
        
    rc = json_encode_object_start(&encoder); 
    _twr_encode(frame);
    rc |= json_encode_object_finish(&encoder);
    _json_fflush();
    printf(" \n");
}

/*!
 * @fn raz_encode(twr_frame_t * frame) {
 *
 * @brief JSON encoding twr_frames support the folowing json objects: 
 * {"twr": {"raz": ["1.100","0.000","0.000"],"uid": "1234"},"uid": "4321"}
 * input parameters
 * @param frame twr_frame_t *
 * output parameters
 * returns void
 */

void
_raz_encode(twr_frame_t * frame) {
    struct json_encoder encoder;
    struct json_value value;
    int rc;

    memset(&encoder, 0, sizeof(encoder));
    encoder.je_write = json_write;
    encoder.je_arg= NULL;

    rc = json_encode_object_key(&encoder, "twr");  
    rc |= json_encode_object_start(&encoder);  

#if MYNEWT_VAL(FLOAT_USER)
    char float_string[32];
    sprintf(float_string,"%f",frame->spherical.range);
    JSON_VALUE_STRING(&value, float_string);
#else
    JSON_VALUE_UINT(&value, *(uint32_t *)&frame->spherical.range);
#endif

    rc |= json_encode_array_name(&encoder, "raz");
    rc |= json_encode_array_start(&encoder);

    for (uint16_t i=0; i < 3; i++){
#if MYNEWT_VAL(FLOAT_USER)
        if ((float)frame->spherical.array[i] == 0.0f/0.0f){
            JSON_VALUE_STRING(&value, "null");
        }else{
            char float_string[32];
            sprintf(float_string,"%f",frame->spherical.array[i]);
            JSON_VALUE_STRING(&value, float_string);
        }        
#else
        JSON_VALUE_UINT(&value, *(uint32_t *)&frame->spherical.array[i]);
#endif
        rc |= json_encode_array_value(&encoder, &value);
        if (i%64==0) _json_fflush();
    }
    rc |= json_encode_array_finish(&encoder);

    char uuid[16];
    sprintf(uuid,"%04x",frame->dst_address);
    JSON_VALUE_STRINGN(&value, uuid,4);
    rc |= json_encode_object_entry(&encoder, "uid", &value);
    rc |= json_encode_object_finish(&encoder);

    sprintf(uuid,"%04x",frame->src_address);
    JSON_VALUE_STRINGN(&value, uuid,4);
    rc |= json_encode_object_entry(&encoder, "uid", &value);

    assert(rc == 0);
    _json_fflush();
   
}

void
raz_encode(twr_frame_t * frame){
    struct json_encoder encoder;
    int rc;

    memset(&encoder, 0, sizeof(encoder));
    encoder.je_write = json_write;
    encoder.je_arg= NULL;
        
    rc = json_encode_object_start(&encoder); 
    _raz_encode(frame);
    rc |= json_encode_object_finish(&encoder);
    _json_fflush();
    printf(" \n");
}

#endif
