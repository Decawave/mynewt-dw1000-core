/*
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

#ifndef _DPL_ATOMIC_H_
#define _DPL_ATOMIC_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dpl/dpl_os_types.h"

#ifdef __cplusplus
extern "C" {
#endif

uint32_t dpl_hw_enter_critical(void);
void dpl_hw_exit_critical(uint32_t ctx);
bool dpl_hw_is_in_critical(void);

#ifdef __cplusplus
}
#endif

#endif  /* _DPL_ATOMIC_H_ */
