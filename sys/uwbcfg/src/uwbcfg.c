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

#include <string.h>
#include <stdio.h>

#include <os/mynewt.h>
#include <config/config.h>

#include <uwbcfg/uwbcfg.h>
#include <dw1000/dw1000_hal.h>

static struct uwbcfg_cbs_head uwbcfg_callbacks;

/* 
 * Default Config 
 */
#define UWB_DEFAULT_CONFIG {        \
        .channel = "5",                         \
            .prf = "64",                        \
            .dataRate = "6m8",                  \
            .rx_paclen= "8",                    \
            .rx_preambleCodeIndex = "9",        \
            .rx_sfdType = "0",                  \
            .rx_phrMode = "s",                  \
            .tx_preambleCodeIndex = "9",        \
            .tx_preambleLength = "128",         \
            .txrf_power_coarse = "15",          \
            .txrf_power_fine = "22",            \
            .rx_antenna_dly = "0x4050",         \
            .tx_antenna_dly = "0x4050",         \
    }

/** Differing from dw1000_power_value in that fine is interpreted as integer steps
 *  Thus to get 15.5dB from fine, set FINE to 31 */
#define power_value(COARSE,FINE) ((COARSE<<5) + FINE)

static char *uwbcfg_get(int argc, char **argv, char *val, int val_len_max);
static int uwbcfg_set(int argc, char **argv, char *val);
static int uwbcfg_commit(void);
static int uwbcfg_export(void (*export_func)(char *name, char *val),
  enum conf_export_tgt tgt);

static struct uwb_config_s {
    char channel[4];
    char prf[4];
    char dataRate[8];
    char rx_paclen[4];
    char rx_preambleCodeIndex[4];
    char rx_sfdType[4];
    char rx_phrMode[4];
    char tx_preambleCodeIndex[4];
    char tx_preambleLength[8];
    char txrf_power_coarse[8];
    char txrf_power_fine[8];
    char rx_antenna_dly[8];
    char tx_antenna_dly[8];
} uwb_config = UWB_DEFAULT_CONFIG;

static struct conf_handler uwbcfg_handler = {
    .ch_name = "uwb",
    .ch_get = uwbcfg_get,
    .ch_set = uwbcfg_set,
    .ch_commit = uwbcfg_commit,
    .ch_export = uwbcfg_export,
};

static char *
uwbcfg_get(int argc, char **argv, char *val, int val_len_max)
{
    if (argc == 1) {
        if (!strcmp(argv[0], "channel"))  return uwb_config.channel;
        else if (!strcmp(argv[0], "prf"))      return uwb_config.prf;
        else if (!strcmp(argv[0], "datarate")) return uwb_config.dataRate;
        
        else if (!strcmp(argv[0], "rx_paclen")) return uwb_config.rx_paclen;
        else if (!strcmp(argv[0], "rx_pream_cidx")) return uwb_config.rx_preambleCodeIndex;
        else if (!strcmp(argv[0], "rx_sfdtype")) return uwb_config.rx_sfdType;
        else if (!strcmp(argv[0], "rx_phrmode")) return uwb_config.rx_phrMode;
        
        else if (!strcmp(argv[0], "tx_pream_cidx")) return uwb_config.tx_preambleCodeIndex;
        else if (!strcmp(argv[0], "tx_pream_len")) return uwb_config.tx_preambleLength;
        else if (!strcmp(argv[0], "txrf_power_coarse")) return uwb_config.txrf_power_coarse;
        else if (!strcmp(argv[0], "txrf_power_fine"))  return uwb_config.txrf_power_fine;
        else if (!strcmp(argv[0], "rx_antdly")) return uwb_config.rx_antenna_dly;
        else if (!strcmp(argv[0], "tx_antdly")) return uwb_config.tx_antenna_dly;
    }
    return NULL;
}

static int
uwbcfg_set(int argc, char **argv, char *val)
{
    if (argc == 1) {
        if (!strcmp(argv[0], "channel")) {
            return CONF_VALUE_SET(val, CONF_STRING, uwb_config.channel);
        } else if (!strcmp(argv[0], "prf")) {
            return CONF_VALUE_SET(val, CONF_STRING, uwb_config.prf);
        } else if (!strcmp(argv[0], "datarate")) {
            return CONF_VALUE_SET(val, CONF_STRING, uwb_config.dataRate);
            /* RX  */
        } else if (!strcmp(argv[0], "rx_paclen")) {
            return CONF_VALUE_SET(val, CONF_STRING, uwb_config.rx_paclen);
        } else if (!strcmp(argv[0], "rx_pream_cidx")) {
            return CONF_VALUE_SET(val, CONF_STRING, uwb_config.rx_preambleCodeIndex);
        } else if (!strcmp(argv[0], "rx_sfdtype")) {
            return CONF_VALUE_SET(val, CONF_STRING, uwb_config.rx_sfdType);
        } else if (!strcmp(argv[0], "rx_phrmode")) {
            return CONF_VALUE_SET(val, CONF_STRING, uwb_config.rx_phrMode);

        } else if (!strcmp(argv[0], "tx_pream_cidx")) {
            return CONF_VALUE_SET(val, CONF_STRING, uwb_config.tx_preambleCodeIndex);
        } else if (!strcmp(argv[0], "tx_pream_len")) {
            return CONF_VALUE_SET(val, CONF_STRING, uwb_config.tx_preambleLength);

        } else if (!strcmp(argv[0], "txrf_power_coarse")) {
            return CONF_VALUE_SET(val, CONF_STRING, uwb_config.txrf_power_coarse);
        } else if (!strcmp(argv[0], "txrf_power_fine")) {
            return CONF_VALUE_SET(val, CONF_STRING, uwb_config.txrf_power_fine);
        } else if (!strcmp(argv[0], "rx_antdly")) {
            return CONF_VALUE_SET(val, CONF_STRING, uwb_config.rx_antenna_dly);
        } else if (!strcmp(argv[0], "tx_antdly")) {
            return CONF_VALUE_SET(val, CONF_STRING, uwb_config.tx_antenna_dly);
        }
    }
    return OS_ENOENT;
}

static int
uwbcfg_commit(void)
{
    uint8_t coarse, fine, txpwr, datarate, paclen;
    dw1000_dev_instance_t * inst = hal_dw1000_inst(0);

    conf_value_from_str(uwb_config.channel, CONF_INT8, (void*)&(inst->config.channel), 0);
    switch (inst->config.channel) {
    case (1): inst->config.txrf.PGdly = TC_PGDELAY_CH1;break;
    case (2): inst->config.txrf.PGdly = TC_PGDELAY_CH2;break;
    case (3): inst->config.txrf.PGdly = TC_PGDELAY_CH3;break;
    case (4): inst->config.txrf.PGdly = TC_PGDELAY_CH4;break;
    case (5): inst->config.txrf.PGdly = TC_PGDELAY_CH5;break;
    case (7): inst->config.txrf.PGdly = TC_PGDELAY_CH7;break;
    default: 
        printf("Warning, invalid channel\n");
        break;
    }

    datarate = DWT_BR_6M8;
    if (!strcmp(uwb_config.dataRate, "6m8")) datarate = DWT_BR_6M8;
    else if (!strcmp(uwb_config.dataRate, "850k")) datarate = DWT_BR_850K;
    else if (!strcmp(uwb_config.dataRate, "110k")) datarate = DWT_BR_110K;
    else {
        printf("Warning, invalid datarate\n");
    }
    inst->config.dataRate = datarate;

    
    conf_value_from_str(uwb_config.rx_paclen, CONF_INT8,
                        (void*)&paclen, 0);
    switch (paclen) {
    case (8):  inst->config.rx.pacLength = DWT_PAC8;break;
    case (16): inst->config.rx.pacLength = DWT_PAC16;break;
    case (32): inst->config.rx.pacLength = DWT_PAC32;break;
    case (64): inst->config.rx.pacLength = DWT_PAC64;break;
    default:
        printf("Warning, invalid datarate\n");
    }
    conf_value_from_str(uwb_config.rx_sfdType, CONF_INT8,
                        (void*)&(inst->config.rx.sfdType), 0);
    if (uwb_config.rx_phrMode[0] == 's') {
        inst->config.rx.phrMode = DWT_PHRMODE_STD;
    } else {
        inst->config.rx.phrMode = DWT_PHRMODE_EXT;
    }

    conf_value_from_str(uwb_config.rx_preambleCodeIndex, CONF_INT8,
                        (void*)&(inst->config.rx.preambleCodeIndex), 0);
    conf_value_from_str(uwb_config.tx_preambleCodeIndex, CONF_INT8,
                        (void*)&(inst->config.tx.preambleCodeIndex), 0);

    conf_value_from_str(uwb_config.txrf_power_coarse, CONF_INT8,
                        (void*)&coarse, 0);
    conf_value_from_str(uwb_config.txrf_power_fine, CONF_INT8, (void*)&fine, 0);

    txpwr = inst->config.txrf.BOOSTNORM;
    switch (coarse) {
    case(18): txpwr = power_value(DW1000_txrf_config_18db, fine);break;
    case(15): txpwr = power_value(DW1000_txrf_config_15db, fine);break;
    case(12): txpwr = power_value(DW1000_txrf_config_12db, fine);break;
    case(9):  txpwr = power_value(DW1000_txrf_config_9db, fine);break;
    case(6):  txpwr = power_value(DW1000_txrf_config_6db, fine);break;
    case(3):  txpwr = power_value(DW1000_txrf_config_3db, fine);break;
    case(0):  txpwr = power_value(DW1000_txrf_config_0db, fine);break;
    default:
        printf("Warning, invalid coarse txpower config\n");
    }
    inst->config.txrf.BOOSTNORM = txpwr;
    inst->config.txrf.BOOSTP500 = txpwr;
    inst->config.txrf.BOOSTP250 = txpwr;
    inst->config.txrf.BOOSTP125 = txpwr;

    /* Antenna dlys will be updated in dw1000 automatically next time 
     * it wakes up */
    conf_value_from_str(uwb_config.rx_antenna_dly, CONF_INT16,
                        (void*)&inst->rx_antenna_delay, 0);
    conf_value_from_str(uwb_config.tx_antenna_dly, CONF_INT16,
                        (void*)&inst->tx_antenna_delay, 0);

    /* Preamble */
    uint16_t plen;
    uint8_t  txP = inst->config.tx.preambleLength;
    uint16_t sfdTO = inst->config.rx.sfdTimeout;
    conf_value_from_str(uwb_config.tx_preambleLength, CONF_INT16,
                        (void*)&plen, 0);

    /* TODO: calculate with proper pac-len etc. */
    switch (plen)
    {
    case (64):   txP = DWT_PLEN_64;   sfdTO = (plen + 1 + 8 - 8);break;
    case (128):  txP = DWT_PLEN_128;  sfdTO = (plen + 1 + 8 - 8);break;
    case (256):  txP = DWT_PLEN_256;  sfdTO = (plen + 1 + 8 - 8);break;
    case (512):  txP = DWT_PLEN_512;  sfdTO = (plen + 1 + 8 - 8);break;
    case (1024): txP = DWT_PLEN_1024; sfdTO = (plen + 1 + 8 - 8);break;
    case (2048): txP = DWT_PLEN_2048; sfdTO = (plen + 1 + 8 - 8);break;
    case (4096): txP = DWT_PLEN_4096; sfdTO = (plen + 1 + 8 - 8);break;
    default:
        printf("Invalid preamb_len\n");
        break;
    }
    
    inst->config.tx.preambleLength = txP;
    inst->config.rx.sfdTimeout = sfdTO;

    /* Callback to allow host application to decide when to update config
       of chip */
    struct uwbcfg_cbs *cb;
    SLIST_FOREACH(cb, &uwbcfg_callbacks, uc_list) {
        if (cb->uc_update) {
            cb->uc_update();
        }
    }
    return 0;
}

static int
uwbcfg_export(void (*export_func)(char *name, char *val),
  enum conf_export_tgt tgt)
{
    export_func("uwb/channel", uwb_config.channel);
    export_func("uwb/prf", uwb_config.prf);
    export_func("uwb/datarate", uwb_config.dataRate);
    export_func("uwb/rx_antdly", uwb_config.rx_antenna_dly);
    export_func("uwb/tx_antdly", uwb_config.tx_antenna_dly);

    export_func("uwb/rx_paclen", uwb_config.rx_paclen);
    export_func("uwb/rx_pream_cidx", uwb_config.rx_preambleCodeIndex);
    export_func("uwb/rx_sfdtype", uwb_config.rx_sfdType);
    export_func("uwb/rx_phrmode", uwb_config.rx_phrMode);

    export_func("uwb/tx_pream_cidx", uwb_config.tx_preambleCodeIndex);
    export_func("uwb/tx_pream_len", uwb_config.tx_preambleLength);
    
    export_func("uwb/txrf_power_coarse", uwb_config.txrf_power_coarse);
    export_func("uwb/txrf_power_fine", uwb_config.txrf_power_fine);
    return 0;
}

int
uwbcfg_register(struct uwbcfg_cbs *handler)
{
    SLIST_INSERT_HEAD(&uwbcfg_callbacks, handler, uc_list);
    return 0;
}

int
uwbcfg_apply(void)
{
    return uwbcfg_commit(); 
}
    
int
uwbcfg_pkg_init()
{
    int rc;
    rc = conf_register(&uwbcfg_handler);
    SYSINIT_PANIC_ASSERT(rc == 0);
    
    SLIST_INIT(&uwbcfg_callbacks);
#if MYNEWT_VAL(UWBCFG_APPLY_AT_INIT)
    uwbcfg_commit();
#endif
    return 0;
}
