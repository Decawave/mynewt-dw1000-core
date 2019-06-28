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

#ifndef _DW1000_RTDOA_TAG_H_
#define _DW1000_RTDOA_TAG_H_

#if MYNEWT_VAL(RTDOA_TAG_ENABLED)

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_ftypes.h>
#include <rtdoa/rtdoa.h>

void rtdoa_tag_free(dw1000_dev_instance_t * inst);
dw1000_rng_config_t * rtdoa_tag_config(dw1000_dev_instance_t * inst);

#ifdef __cplusplus
}
#endif

#endif // RTDOA_TAG_ENABLED
#endif //_DW1000_RTDOA_TAG_H_
