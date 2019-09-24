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

#ifndef _UWB_WCS_H_
#define _wCS_H_

#include <stdlib.h>
#include <stdint.h>
#include <os/os.h>
#include <uwb/uwb.h>
#include <uwb_ccp/uwb_ccp.h>
#include <timescale/timescale.h>        

#ifdef __cplusplus
extern "C" {
#endif


typedef struct _uwb_wcs_status_t{
    uint16_t selfmalloc:1;
    uint16_t initialized:1;
    uint16_t valid:1;
}uwb_wcs_status_t;

typedef struct _uwb_wcs_control_t{
    uint16_t restart:1;
}uwb_wcs_control_t;

typedef struct _uwb_wcs_config_t{
    uint16_t postprocess:1;
}uwb_wcs_config_t;

struct uwb_wcs_instance {
    uwb_wcs_status_t status;
    uwb_wcs_control_t control;
    uwb_wcs_config_t config;
    uint64_t observed_interval;
    uwb_ccp_timestamp_t master_epoch;
    uwb_ccp_timestamp_t local_epoch;
    double skew;
    struct dpl_event postprocess_ev;
    struct uwb_ccp_instance * ccp;
    struct _timescale_instance_t * timescale;
};

struct uwb_wcs_instance * uwb_wcs_init(struct uwb_wcs_instance * inst, struct uwb_ccp_instance * ccp);
void uwb_wcs_free(struct uwb_wcs_instance * inst);
void uwb_wcs_update_cb(struct dpl_event * ev);
void uwb_wcs_set_postprocess(struct uwb_wcs_instance * inst, dpl_event_fn * postprocess);

uint64_t uwb_wcs_read_systime(struct uwb_dev * inst);
uint32_t uwb_wcs_read_systime_lo(struct uwb_dev * inst);
uint64_t uwb_wcs_read_rxtime(struct uwb_dev * inst);
uint32_t uwb_wcs_read_rxtime_lo(struct uwb_dev * inst);
uint64_t uwb_wcs_read_txtime(struct uwb_dev * inst);
uint32_t uwb_wcs_read_txtime_lo(struct uwb_dev * inst);
uint64_t uwb_wcs_read_systime_master(struct uwb_dev * inst);
uint32_t uwb_wcs_read_systime_lo_master(struct uwb_dev * inst);
uint64_t uwb_wcs_read_rxtime_master(struct uwb_dev * inst);
uint32_t uwb_wcs_read_rxtime_lo_master(struct uwb_dev * inst);
uint64_t uwb_wcs_read_txtime_master(struct uwb_dev * inst);
uint32_t uwb_wcs_read_txtime_lo_master(struct uwb_dev * inst);

double uwb_wcs_dtu_time_correction(struct uwb_wcs_instance * wcs);
uint64_t uwb_wcs_dtu_time_adjust(struct uwb_wcs_instance * wcs, uint64_t dtu_time);
uint64_t uwb_wcs_local_to_master64(struct uwb_wcs_instance * wcs, uint64_t dtu_time);
uint64_t uwb_wcs_local_to_master(struct uwb_wcs_instance * wcs, uint64_t dtu_time);
uint64_t uwb_wcs_read_systime_master64(struct uwb_dev * inst);

#ifdef __cplusplus
}
#endif

#endif /* _UWB_WCS_H_ */
