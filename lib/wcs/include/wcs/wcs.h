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

#ifndef _WCS_H_
#define _wCS_H_

#include <stdlib.h>
#include <stdint.h>
#include <os/os.h>
#include <dw1000/dw1000_dev.h>
#include <ccp/ccp.h>
#include <timescale/timescale.h>        

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _wcs_status_t{
    uint16_t selfmalloc:1;
    uint16_t initialized:1;
    uint16_t valid:1;
}wcs_status_t;

typedef struct _wcs_control_t{
    uint16_t restart:1;
}wcs_control_t;

typedef struct _wcs_config_t{
    uint16_t postprocess:1;
}wcs_config_t;

typedef struct _wcs_instance_t{
    wcs_status_t status;
    wcs_control_t control;
    wcs_config_t config;
    int16_t nT;
    uint64_t master_epoch;
    uint64_t local_epoch;
    double skew;
    struct os_event postprocess_ev;
    struct _dw1000_ccp_instance_t * ccp;
    struct _timescale_instance_t * timescale;
}wcs_instance_t; 

wcs_instance_t * wcs_init(wcs_instance_t * inst, dw1000_ccp_instance_t * ccp);
void wcs_free(wcs_instance_t * inst);
void wcs_update_cb(struct os_event * ev);
void wcs_set_postprocess(wcs_instance_t * inst, os_event_fn * postprocess);

uint64_t wcs_read_systime(struct _dw1000_dev_instance_t * inst);
uint32_t wcs_read_systime_lo(struct _dw1000_dev_instance_t * inst);
uint64_t wcs_read_rxtime(struct _dw1000_dev_instance_t * inst);
uint32_t wcs_read_rxtime_lo(struct _dw1000_dev_instance_t * inst);
uint64_t wcs_read_txtime(struct _dw1000_dev_instance_t * inst);
uint32_t wcs_read_txtime_lo(struct _dw1000_dev_instance_t * inst);
uint64_t wcs_read_systime_master(struct _dw1000_dev_instance_t * inst);
uint32_t wcs_read_systime_lo_master(struct _dw1000_dev_instance_t * inst);
uint64_t wcs_read_rxtime_master(struct _dw1000_dev_instance_t * inst);
uint32_t wcs_read_rxtime_lo_master(struct _dw1000_dev_instance_t * inst);
uint64_t wcs_read_txtime_master(struct _dw1000_dev_instance_t * inst);
uint32_t wcs_read_txtime_lo_master(struct _dw1000_dev_instance_t * inst);

double wcs_dtu_time_correction(struct _dw1000_dev_instance_t * inst);
uint64_t wcs_dtu_time_adjust(struct _dw1000_dev_instance_t * inst, uint64_t dtu_time);
uint64_t wcs_local_to_master(struct _dw1000_dev_instance_t * inst, uint64_t dtu_time);

#ifdef __cplusplus
}
#endif

#endif /* _WCS_H_ */
