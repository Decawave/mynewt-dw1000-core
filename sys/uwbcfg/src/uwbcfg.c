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
#include <log/log.h>

#include <os/mynewt.h>
#include <config/config.h>

#include <uwbcfg/uwbcfg.h>
#include <dw1000/dw1000_hal.h>

#define LOG_MODULE_UWBCFG (92)
#define UC_INFO(...)     LOG_INFO(&_log, LOG_MODULE_UWBCFG, __VA_ARGS__)
#define UC_DEBUG(...)    LOG_DEBUG(&_log, LOG_MODULE_UWBCFG, __VA_ARGS__)
#define UC_WARN(...)     LOG_WARN(&_log, LOG_MODULE_UWBCFG, __VA_ARGS__)
#define UC_ERR(...)      LOG_ERROR(&_log, LOG_MODULE_UWBCFG, __VA_ARGS__)
static struct log _log;

/** Differing from dw1000_power_value in that fine is interpreted as integer steps
 *  Thus to get 15.5dB from fine, set FINE to 31 */
#define power_value(COARSE,FINE) ((COARSE<<5) + FINE)

static struct uwbcfg_cbs_head uwbcfg_callbacks;

static char *uwbcfg_get(int argc, char **argv, char *val, int val_len_max);
static int uwbcfg_set(int argc, char **argv, char *val);
static int uwbcfg_commit(void);
static int uwbcfg_export(void (*export_func)(char *name, char *val),
  enum conf_export_tgt tgt);

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

static char uwb_config[CFGSTR_MAX][7] = {
    MYNEWT_VAL(UWBCFG_DEF_CH),                /* channel */
    MYNEWT_VAL(UWBCFG_DEF_PRF),               /* prf */
    MYNEWT_VAL(UWBCFG_DEF_DATARATE),          /* datarate */
    MYNEWT_VAL(UWBCFG_DEF_PACLEN),            /* rx_paclen */
    MYNEWT_VAL(UWBCFG_DEF_RX_PREAM_CIDX),     /* rx_pream_cidx */
    MYNEWT_VAL(UWBCFG_DEF_RX_SFD_TYPE),       /* rx_sfdType */
    MYNEWT_VAL(UWBCFG_DEF_RX_PHR_MODE),       /* rx_phrMode */
    MYNEWT_VAL(UWBCFG_DEF_TX_PREAM_CIDX),     /* tx_pream_cidx */
    MYNEWT_VAL(UWBCFG_DEF_TX_PREAM_LEN),      /* tx_pream_len */
    MYNEWT_VAL(UWBCFG_DEF_TXRF_POWER_COARSE), /* txrf_power_coarse */
    MYNEWT_VAL(UWBCFG_DEF_TXRF_POWER_FINE),   /* txrf_power_fine */
    MYNEWT_VAL(UWBCFG_DEF_RX_ANTDLY),         /* rx_antdly */
    MYNEWT_VAL(UWBCFG_DEF_TX_ANTDLY),         /* tx_antdly */
    MYNEWT_VAL(UWBCFG_DEF_ROLE),              /* role */
};

static const char* _uwbcfg_str[] = {
    "channel", "prf", "datarate",
    "rx_paclen", "rx_pream_cidx", "rx_sfdtype", "rx_phrmode",
    "tx_pream_cidx", "tx_pream_len", "txrf_power_coarse", "txrf_power_fine",
    "rx_antdly", "tx_antdly", "role"
};

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
        for (int i=0;i<CFGSTR_MAX;i++) {
            if (!strcmp(argv[0], _uwbcfg_str[i])) return uwb_config[i];
        }
    }
    return NULL;
}

static int
uwbcfg_set(int argc, char **argv, char *val)
{
    if (argc == 1) {
        for (int i=0;i<CFGSTR_MAX;i++) {
            if (!strcmp(argv[0], _uwbcfg_str[i]))
                return CONF_VALUE_SET(val, CONF_STRING, uwb_config[i]);
        }
    }
    return OS_ENOENT;
}

static void
check_preamble_code(dw1000_dev_instance_t * inst, uint8_t *arg_code)
{
    int new_code = 0;
    int ch = inst->config.channel;
    int prf = inst->config.prf;
    uint8_t code = *arg_code;
    if (prf == DWT_PRF_16M) {
        if (ch == 1 && code != 1 && code != 2)              new_code = 1;
        if ((ch == 2 || ch==5) && code != 3 && code != 4)   new_code = 3;
        if (ch == 3 && code != 5 && code != 6)              new_code = 5;
        if ((ch == 4 || ch == 7) && code != 7 && code != 8) new_code = 7;
    } else {
        if (ch == 1 || ch==2 || ch==3 || ch==5) {
            if (code < 9 || code > 12)  new_code = 9;
        } else { /* channels 4 and 7 */
            if (code < 17 || code > 20) new_code = 17;
        }
    }
    if (new_code) {
        UC_WARN("inv pream code (using %d)\n", new_code);
        *arg_code = new_code;
    }
}

static int
uwbcfg_commit_to_inst(dw1000_dev_instance_t * inst)
{
    uint8_t coarse, fine, txpwr, paclen;
    int sfd_len=0;

    conf_value_from_str(uwb_config[CFGSTR_CH], CONF_INT8, (void*)&(inst->config.channel), 0);
    switch (inst->config.channel) {
    case (1): inst->config.txrf.PGdly = TC_PGDELAY_CH1;break;
    case (2): inst->config.txrf.PGdly = TC_PGDELAY_CH2;break;
    case (3): inst->config.txrf.PGdly = TC_PGDELAY_CH3;break;
    case (4): inst->config.txrf.PGdly = TC_PGDELAY_CH4;break;
    case (5): inst->config.txrf.PGdly = TC_PGDELAY_CH5;break;
    case (7): inst->config.txrf.PGdly = TC_PGDELAY_CH7;break;
    default: 
        UC_WARN("inv ch\n");
        break;
    }
    
    /* Set the PRF */
    if (!strcmp(uwb_config[CFGSTR_PRF], "16")) {
        inst->config.prf = DWT_PRF_16M;
    } else if (!strcmp(uwb_config[CFGSTR_PRF], "64")) {
        inst->config.prf = DWT_PRF_64M;
    } else {
        UC_WARN("inv prf\n");
    }

    /* Data rate */
    if (!strcmp(uwb_config[CFGSTR_DATARATE], "6m8")) {
        inst->config.dataRate = DWT_BR_6M8;
        sfd_len = 8;
    } else if (!strcmp(uwb_config[CFGSTR_DATARATE], "850k")) {
        inst->config.dataRate = DWT_BR_850K;
        sfd_len = 8;
    } else if (!strcmp(uwb_config[CFGSTR_DATARATE], "110k")) {
        inst->config.dataRate = DWT_BR_110K;
        sfd_len = 64;
    } else {
        UC_WARN("inv datarate\n");
    }
    
    /* PAC length */
    conf_value_from_str(uwb_config[CFGSTR_RX_PACLEN], CONF_INT8, (void*)&paclen, 0);
    switch (paclen) {
    case (8):  inst->config.rx.pacLength = DWT_PAC8;break;
    case (16): inst->config.rx.pacLength = DWT_PAC16;break;
    case (32): inst->config.rx.pacLength = DWT_PAC32;break;
    case (64): inst->config.rx.pacLength = DWT_PAC64;break;
    default:
        UC_WARN("inv paclen\n");
    }

    inst->config.rx.sfdType = (uwb_config[CFGSTR_RX_SFDTYPE][0] == '1');
    inst->config.rx.phrMode = (uwb_config[CFGSTR_RX_PHRMODE][0] == 's')? DWT_PHRMODE_STD : DWT_PHRMODE_EXT;

    /* Check that the rx and tx preamble codes are legal for the ch+prf combo */
    conf_value_from_str(uwb_config[CFGSTR_RX_PREAM_CIDX], CONF_INT8,
                        (void*)&(inst->config.rx.preambleCodeIndex), 0);
    check_preamble_code(inst, &inst->config.rx.preambleCodeIndex);
    conf_value_from_str(uwb_config[CFGSTR_TX_PREAM_CIDX], CONF_INT8,
                        (void*)&(inst->config.tx.preambleCodeIndex), 0);
    check_preamble_code(inst, &inst->config.tx.preambleCodeIndex);

    /* Tx Power */
    conf_value_from_str(uwb_config[CFGSTR_TXRF_PWR_COARSE], CONF_INT8, (void*)&coarse, 0);
    conf_value_from_str(uwb_config[CFGSTR_TXRF_PWR_FINE], CONF_INT8, (void*)&fine, 0);

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
        UC_WARN("inv coarse txpwr\n");
    }
    inst->config.txrf.BOOSTNORM = txpwr;
    inst->config.txrf.BOOSTP500 = txpwr;
    inst->config.txrf.BOOSTP250 = txpwr;
    inst->config.txrf.BOOSTP125 = txpwr;

    /* Antenna dlys will be updated in dw1000 automatically next time it wakes up */
    conf_value_from_str(uwb_config[CFGSTR_RX_ANTDLY], CONF_INT16, (void*)&inst->rx_antenna_delay, 0);
    conf_value_from_str(uwb_config[CFGSTR_TX_ANTDLY], CONF_INT16, (void*)&inst->tx_antenna_delay, 0);

    /* Role */
    conf_value_from_str(uwb_config[CFGSTR_ROLE], CONF_INT16, (void*)&inst->role, 0);

    /* Preamble */
    uint16_t preamble_len;
    uint8_t  txP = inst->config.tx.preambleLength;
    uint16_t sfd_timeout = inst->config.rx.sfdTimeout;
    conf_value_from_str(uwb_config[CFGSTR_TX_PREAM_LEN], CONF_INT16,
                        (void*)&preamble_len, 0);

    switch (preamble_len)
    {
    case (64):   txP = DWT_PLEN_64  ;break;
    case (128):  txP = DWT_PLEN_128 ;break;
    case (256):  txP = DWT_PLEN_256 ;break;
    case (512):  txP = DWT_PLEN_512 ;break;
    case (1024): txP = DWT_PLEN_1024;break;
    case (2048): txP = DWT_PLEN_2048;break;
    case (4096): txP = DWT_PLEN_4096;break;
    default:
        UC_WARN("inv preamb_len\n");
        break;
    }
    /* Calculate the SFD timeout */
    sfd_timeout = (preamble_len + 1 + sfd_len - paclen);
    inst->config.tx.preambleLength = txP;
    inst->config.rx.sfdTimeout = sfd_timeout;
    inst->attrib.nsfd = sfd_len;
    inst->attrib.nsync = preamble_len;

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
uwbcfg_commit(void)
{
    dw1000_dev_instance_t * inst;
#if  MYNEWT_VAL(DW1000_DEVICE_0)
    inst = hal_dw1000_inst(0);
    uwbcfg_commit_to_inst(inst);
#endif
#if  MYNEWT_VAL(DW1000_DEVICE_1)
    inst = hal_dw1000_inst(1);
    uwbcfg_commit_to_inst(inst);
#endif
#if  MYNEWT_VAL(DW1000_DEVICE_2)
    inst = hal_dw1000_inst(2);
    uwbcfg_commit_to_inst(inst);
#endif
    return 0;
}

static int
uwbcfg_export(void (*export_func)(char *name, char *val),
  enum conf_export_tgt tgt)
{
    char b[32];
    for (int i=0;i<CFGSTR_MAX;i++) {
        snprintf(b, sizeof(b), "%s/%s", uwbcfg_handler.ch_name, _uwbcfg_str[i]);
        export_func(b, uwb_config[i]);
    }
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

    /* Init log and Config */
    log_register("uwbcfg", &_log, &log_console_handler,
                 NULL, LOG_SYSLEVEL);
    
    SLIST_INIT(&uwbcfg_callbacks);
#if MYNEWT_VAL(UWBCFG_APPLY_AT_INIT)
    uwbcfg_commit();
#endif
    return 0;
}
