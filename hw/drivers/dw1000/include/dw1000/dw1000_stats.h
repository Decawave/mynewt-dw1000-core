/*
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

/** 
 * @file dw1000_stats.h
 * @author paul kettle
 * @date 10/24/18
 * @brief ftypes file
 *
 * @details 
 */

#ifndef _DW1000_STATS_H_
#define _DW1000_STATS_H_

#include <stdlib.h>
#include <stdint.h>
#include <stats/stats.h>

#ifdef __cplusplus
extern "C" {
#endif

STATS_SECT_START(mac_stat_section)
    STATS_SECT_ENTRY(tx_bytes)
    STATS_SECT_ENTRY(rx_bytes)
    STATS_SECT_ENTRY(DFR_cnt)
    STATS_SECT_ENTRY(RTO_cnt)
    STATS_SECT_ENTRY(ROV_err)
    STATS_SECT_ENTRY(TFG_cnt)
    STATS_SECT_ENTRY(LDE_err)
    STATS_SECT_ENTRY(RX_err)
STATS_SECT_END

#ifdef __cplusplus
}
#endif
#endif /* _DW1000_STATS_H_ */
