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

#include <os/os_time.h>
#include <syscfg/syscfg.h>
#include <datetime/datetime.h>

#if MYNEWT_VAL(UWBCFG_CLI)

#include <uwbcfg/uwbcfg.h>
#include "uwbcfg_priv.h"

#include "os/mynewt.h"
#include <string.h>

#include <defs/error.h>
#include <hal/hal_bsp.h>
#include <console/console.h>

#include <shell/shell.h>

#include <tinycbor/cbor.h>
#include <tinycbor/cborjson.h>
#include <tinycbor/cbor_mbuf_writer.h>
#include <tinycbor/cbor_mbuf_reader.h>
#include <cborattr/cborattr.h>

#include <nmgr_os/nmgr_os.h>
#include <mgmt/mgmt.h>
#include <newtmgr/newtmgr.h>

#if MYNEWT_VAL(DW1000_DEVICE_0)
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#if MYNEWT_VAL(NMGR_UWB_ENABLED)
#include <nmgr_uwb/nmgr_uwb.h> 
#endif
#endif

static struct nmgr_transport nmgr_mstr_transport;

static uint16_t
nmgr_mstr_get_mtu(struct os_mbuf *m)
{
    return 196;
}

static int
nmgr_mstr_out(struct nmgr_transport *nt, struct os_mbuf *req)
{
    int rc;
    int64_t rc_attr;
    CborError g_err = CborNoError;
    struct mgmt_cbuf n_b;
    struct cbor_mbuf_reader reader;
    
    struct cbor_attr_t attrs[] = {
        [0] = {
            .attribute = "rc",
            .type = CborAttrIntegerType,
            .addr.integer = &rc_attr,
            .nodefault = true
        },
        [1] = {
            .attribute = NULL
        }
    };
    rc = 0;
    cbor_mbuf_reader_init(&reader, req, sizeof(struct nmgr_hdr));
    cbor_parser_init(&reader.r, 0, &n_b.parser, &n_b.it);

    struct mgmt_cbuf *cb = &n_b;

    g_err |= cbor_read_object(&cb->it, attrs);
    if (g_err) {
        console_printf("gerr: '%d\n", g_err);
    }
    if (rc)
    {
        console_printf("nmgr_out: rc=%d\n", (int)(rc_attr&0xffffffff));
    }

#if MYNEWT_VAL(BCAST_OTA_DEBUG)    
    printf("json:\n=========\n");
    CborValue it;
    CborParser p;
    cbor_mbuf_reader_init(&reader, req, sizeof(struct nmgr_hdr));
    cbor_parser_init(&reader.r, 0, &p, &it);

    cbor_value_to_json(stdout, &it,
                       CborConvertRequireMapStringKeys);
    printf("\n===========\n");
#endif

    os_mbuf_free_chain(req);
    return (rc);
}

static int uwbcfg_cli_cmd(int argc, char **argv);

#if MYNEWT_VAL(SHELL_CMD_HELP)
const struct shell_param cmd_uwbcfg_param[] = {
    {"txmycfg", "<addr> [bitfield to include]"},
    {"txcfg", "<addr> <cfg> excfg:channel=1:prf=64:..."},
    {NULL,NULL},
};

const struct shell_cmd_help cmd_uwbcfg_help = {
	"uwbcfg commands", "<txmycfg, txcfg>", cmd_uwbcfg_param
};
#endif


static struct shell_cmd shell_uwbcfg_cmd = {
    .sc_cmd = "uwbcfg",
    .sc_cmd_func = uwbcfg_cli_cmd,
#if MYNEWT_VAL(SHELL_CMD_HELP)
    .help = &cmd_uwbcfg_help
#endif
};

struct os_mbuf*
get_txcfg_mbuf(uint32_t fields, char *cfgstr)
{
    int rc;
    CborEncoder payload_enc;
    struct mgmt_cbuf n_b;
    struct cbor_mbuf_writer writer;
    struct nmgr_hdr *hdr;
    struct os_mbuf *rsp;
    bool do_save = (fields!=0x0);
    int num_fields = 0;
    
    rsp = os_msys_get_pkthdr(0, 0);

    if (!rsp) {
        UC_ERR("could not get mbuf\n");
        return 0;
    }

    hdr = (struct nmgr_hdr *) os_mbuf_extend(rsp, sizeof(struct nmgr_hdr));
    if (!hdr) {
        goto exit_err;
    }
    hdr->nh_len = 0;
    hdr->nh_flags = 0;
    hdr->nh_op = NMGR_OP_WRITE;
    hdr->nh_group = htons(MGMT_GROUP_ID_UWBCFG);
    hdr->nh_seq = 0;
    hdr->nh_id = 0;

    cbor_mbuf_writer_init(&writer, rsp);
    cbor_encoder_init(&n_b.encoder, &writer.enc, 0);
    rc = cbor_encoder_create_map(&n_b.encoder, &payload_enc, CborIndefiniteLength);
    if (rc != 0) {
        goto exit_err;
    }

    struct mgmt_cbuf *cb = &n_b;
    
    CborError g_err = CborNoError;

    struct CborEncoder data_array_enc;
    g_err |= cbor_encode_text_stringz(&payload_enc, "cfgs");
    cbor_encoder_create_array(&payload_enc, &data_array_enc, CborIndefiniteLength);
        
    if (cfgstr) {
        assert(fields==0);
        /* Extract config from string */
        char* rest = cfgstr;
        char* token = strtok_r(cfgstr, ":", &rest);
  
        // Keep printing tokens while one of the 
        // delimiters present in str[]. 
        while (token != NULL) {
            if (strstr(token, "save")) {
                console_printf("  do_save\n");
                do_save = true;
                goto next;
            }
            if (!strchr(token, '=')) {
                goto next;
            }
            char field[16];
            char *p = token, *f = field;
            while(*p != '=' && f-field < sizeof(field)) {
                *f++ = *p++;
            }
            *f = 0;
            p++;
            for (int i=0;i<CFGSTR_MAX;i++) {
                if (!strcmp(_uwbcfg_str[i], field)) {
                    console_printf("  %s -> '%s'\n", _uwbcfg_str[i], p);
                    g_err |= cbor_encode_text_stringz(&data_array_enc, p);
                    fields |= ((uint32_t)1<<i);
                    num_fields++;
                    break;
                }
            }
        next:
            token = strtok_r(rest, ":", &rest);
        }
    } else {
        for (int i=0;i<CFGSTR_MAX;i++) {
            if (fields&((uint32_t)0x1<<i)) {
                console_printf("  %s -> '%s'\n", _uwbcfg_str[i], uwbcfg_internal_get(i));
                g_err |= cbor_encode_text_stringz(&data_array_enc,
                                                  uwbcfg_internal_get(i));
                num_fields++;
            }
        }
    }
    cbor_encoder_close_container(&payload_enc, &data_array_enc);

    g_err |= cbor_encode_text_stringz(&payload_enc, "flds");
    g_err |= cbor_encode_uint(&payload_enc, fields);
    g_err |= cbor_encode_text_stringz(&payload_enc, "save");
    g_err |= cbor_encode_boolean(&payload_enc, do_save);
    
    rc = cbor_encoder_close_container(&cb->encoder, &payload_enc);
    if (rc != 0) {
        goto exit_err;
    }
    hdr->nh_len += cbor_encode_bytes_written(&cb->encoder);
    hdr->nh_len = htons(hdr->nh_len);

    /* If we're not actually sending any fields the other side will not be
     * happy, hence free and return 0 instead  */
    if (num_fields) {
        return rsp;
    }
exit_err:
    console_printf("something went wrong\n");
    os_mbuf_free_chain(rsp);
    return 0;
}


static int
uwbcfg_cli_cmd(int argc, char **argv)
{
    const char* sending = "sending new cfg to %x:\n";
    
    if (argc < 2) {
        console_printf("Too few args\n");
        return 0;
    }
    if (!strcmp(argv[1], "txmycfg")) {
        uint16_t addr;
        uint32_t fields = 0x1fff;
        if (argc < 3) {
            console_printf("pls provide <addr>\n");
            return 0;
        }
        addr = strtol(argv[2], NULL, 0);
        if (argc > 3) {
            fields = strtol(argv[3], NULL, 0);
        }
        console_printf(sending, addr);
        struct os_mbuf *om = get_txcfg_mbuf(fields, 0);
        if (!om) return 0;
        nmgr_uwb_instance_t *nmgruwb = (nmgr_uwb_instance_t*)dw1000_mac_find_cb_inst_ptr(hal_dw1000_inst(0), DW1000_NMGR_UWB);
        uwb_nmgr_queue_tx(nmgruwb, addr, NMGR_CMD_STATE_SEND, om);
    } else if(!strcmp(argv[1], "txcfg")) {
         /* uwbcfg txcfg 0xffff channel=5:prf=64:rx_sfdtype=1 */
        if (argc < 4) {
            console_printf("pls provide: <addr> <cfg>\n");
            return 0;
        }
        uint16_t addr;
        addr = strtol(argv[2], NULL, 0);
        console_printf(sending, addr);
        struct os_mbuf *om = get_txcfg_mbuf(0, argv[3]);
        if (!om) return 0;
        nmgr_uwb_instance_t *nmgruwb = (nmgr_uwb_instance_t*)dw1000_mac_find_cb_inst_ptr(hal_dw1000_inst(0), DW1000_NMGR_UWB);
        uwb_nmgr_queue_tx(nmgruwb, addr, NMGR_CMD_STATE_SEND, om);
        
    } else {
        return -1;
    }
    return 0;
}

int
uwbcfg_cli_register(void)
{
    int rc;
    rc = nmgr_transport_init(&nmgr_mstr_transport, nmgr_mstr_out,
                             nmgr_mstr_get_mtu);
    assert(rc == 0);

    return shell_cmd_register(&shell_uwbcfg_cmd);
}
#endif /* MYNEWT_VAL(UWBCFG_CLI) */
