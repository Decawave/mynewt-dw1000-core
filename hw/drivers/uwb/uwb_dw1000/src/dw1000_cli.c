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
 * @file dw1000_cli.c
 * @author Niklas Casaril
 * @date 2018
 * @brief Command debug interface  
 *
 * @details 
 *
 */

#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_regs.h>

#include <shell/shell.h>
#include <console/console.h>
#if MYNEWT_VAL(CCP_ENABLED)
#include <ccp/ccp.h>
#endif
#if MYNEWT_VAL(RNG_ENABLED)
#include <rng/rng.h>
#endif
#if MYNEWT_VAL(NRNG_ENABLED)
#include <nrng/nrng.h>
#endif
#if MYNEWT_VAL(SURVEY_ENABLED)
#include <survey/survey.h>
#endif


#if MYNEWT_VAL(DW1000_CLI)

static int dw1000_cli_cmd(int argc, char **argv);

#if MYNEWT_VAL(SHELL_CMD_HELP)
const struct shell_param cmd_dw1000_param[] = {
    {"dump", "[instance] dump all registers"},
    {NULL,NULL},
};

const struct shell_cmd_help cmd_dw1000_help = {
	"dw1000 dbg", "dw1000 debug", cmd_dw1000_param
};
#endif

static struct shell_cmd shell_dw1000_cmd = {
    .sc_cmd = "dw1000",
    .sc_cmd_func = dw1000_cli_cmd,
#if MYNEWT_VAL(SHELL_CMD_HELP)
    &cmd_dw1000_help
#endif
};

static void
dw1000_dump_registers(struct _dw1000_dev_instance_t * inst)
{
    uint64_t reg = 0;
    int i, l;

    for(i=0; i<0x37; i++)
    {
        if (i==0x05 || i==0x07 || i==0x0B ||
            i==0x16 || i==0x1B || i==0x1C ||
            i==0x20 || i==0x22 || i==0x29 ||
            (i>0x29 && i<0x36)) {
            continue;
        }
        switch (i) {
        case (DEV_ID_ID):
        case (PANADR_ID):
        case (SYS_CFG_ID):
        case (SYS_CTRL_ID):
        case (SYS_MASK_ID):
        case (RX_FINFO_ID):
        case (RX_TTCKI_ID):
        case (ACK_RESP_T_ID):
        case (RX_SNIFF_ID):
        case (TX_POWER_ID):
        case (CHAN_CTRL_ID):
        case (TX_ANTD_ID):
            reg = dw1000_read_reg(inst, i, 0, 4);
            console_printf("{\"reg[%02X]\"=\"0x%08llX\"}\n",i,reg&0xffffffff);
            break;
        case (SYS_TIME_ID):
        case (TX_FCTRL_ID):
        case (DX_TIME_ID):
        case (SYS_STATUS_ID):
        case (RX_TTCKO_ID):
        case (RX_TIME_ID):
        case (TX_TIME_ID):
        case (SYS_STATE_ID):
            reg = dw1000_read_reg(inst, i, 0, 5);
            console_printf("{\"reg[%02X]\"=\"0x%08llX\"}\n",i,reg&0xffffffffffll);
            break;
        default:
            l=8;
            reg = dw1000_read_reg(inst, i, 0, l);
            console_printf("{\"reg[%02X]\"=\"0x%016llX\"}\n",i,
                           reg&0xffffffffffffffffll);
        }
    }
    console_printf("{\"inst->tx_sem\"=\"0x%0X\"}\n", dpl_sem_get_count(&inst->tx_sem));
#if MYNEWT_VAL(RNG_ENABLED)
    dw1000_rng_instance_t *rng = (dw1000_rng_instance_t*)dw1000_mac_find_cb_inst_ptr(inst, DW1000_RNG);
    if (rng)
        console_printf("{\"rng->sem\"=\"0x%0X\"}\n", dpl_sem_get_count(&rng->sem));
#endif
#if MYNEWT_VAL(NRNG_ENABLED)
    dw1000_nrng_instance_t *nrng = (dw1000_nrng_instance_t*)dw1000_mac_find_cb_inst_ptr(inst, DW1000_NRNG);
    if (nrng)
        console_printf("{\"nrng->sem\"=\"0x%0X\"}\n", dpl_sem_get_count(&nrng->sem));
#endif
#if MYNEWT_VAL(CCP_ENABLED)
    dw1000_ccp_instance_t *ccp = (dw1000_ccp_instance_t*)dw1000_mac_find_cb_inst_ptr(inst, DW1000_CCP);
    if (ccp)
        console_printf("{\"ccp->sem\"=\"0x%0X\"}\n", dpl_sem_get_count(&ccp->sem));
#endif
#if MYNEWT_VAL(SURVEY_ENABLED)
    survey_instance_t *survey = (survey_instance_t*)dw1000_mac_find_cb_inst_ptr(inst, DW1000_SURVEY);
    if (survey)
        console_printf("{\"survey->sem\"=\"0x%0X\"}\n", dpl_sem_get_count(&survey->sem));
#endif
}

static void
dw1000_cli_too_few_args(void)
{
    console_printf("Too few args\n");
}

static int
dw1000_cli_cmd(int argc, char **argv)
{
    struct _dw1000_dev_instance_t * inst = 0;
    uint16_t inst_n;
    
    if (argc < 2) {
        dw1000_cli_too_few_args();
        return 0;
    }
    
    if (!strcmp(argv[1], "dump")) {
        if (argc < 3) {
            inst_n=0;
        } else {
            inst_n = strtol(argv[2], NULL, 0);
        }
        inst = hal_dw1000_inst(inst_n);
        dw1000_dump_registers(inst);
    } else {
        console_printf("Unknown cmd\n");
    }

    return 0;
}

#endif


int
dw1000_cli_register(void)
{
#if MYNEWT_VAL(DW1000_CLI)
    return shell_cmd_register(&shell_dw1000_cmd);
#else
    return 0;
#endif
}
