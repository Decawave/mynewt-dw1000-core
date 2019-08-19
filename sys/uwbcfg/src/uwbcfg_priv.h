/**
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

#ifndef __UWBCFG_PRIV_H_
#define __UWBCFG_PRIV_H_

#include <log/log.h>
#define LOG_MODULE_UWBCFG (92)
#define UC_INFO(...)     LOG_INFO(&_uwbcfg_log, LOG_MODULE_UWBCFG, __VA_ARGS__)
#define UC_DEBUG(...)    LOG_DEBUG(&_uwbcfg_log, LOG_MODULE_UWBCFG, __VA_ARGS__)
#define UC_WARN(...)     LOG_WARN(&_uwbcfg_log, LOG_MODULE_UWBCFG, __VA_ARGS__)
#define UC_ERR(...)      LOG_ERROR(&_uwbcfg_log, LOG_MODULE_UWBCFG, __VA_ARGS__)

enum {
    CFGSTR_CH=0,
    CFGSTR_PRF,
    CFGSTR_DATARATE,
    CFGSTR_RX_PACLEN,
    CFGSTR_RX_PREAM_CIDX,
    CFGSTR_RX_SFDTYPE,
    CFGSTR_RX_PHRMODE,
    CFGSTR_TX_PREAM_CIDX,
    CFGSTR_TX_PREAM_LEN,
    CFGSTR_TXRF_PWR_COARSE,
    CFGSTR_TXRF_PWR_FINE,
    CFGSTR_RX_ANTDLY,
    CFGSTR_TX_ANTDLY,
    CFGSTR_ROLE,
    CFGSTR_MAX
};


#define CFGSTR_STRLEN (7)
#ifdef __cplusplus
extern "C" {
#endif

extern struct log _uwbcfg_log;
extern const char* _uwbcfg_str[CFGSTR_MAX];
    
char* uwbcfg_internal_get(int idx);
int uwbcfg_internal_set(int idx, char* val);
int uwbcfg_commit(void);
void uwbcfg_nmgr_module_init(void);
int uwbcfg_cli_register(void);

#ifdef __cplusplus
}
#endif

#endif
