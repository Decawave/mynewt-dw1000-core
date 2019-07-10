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
 * @file nmgr_cmds.c
 * @author paul kettle
 * @date 2018
 * @brief 
 *
 * @details 
 *
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <os/os.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include "bsp/bsp.h"
#include <stats/stats.h>

#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_ftypes.h>
#include <mgmt/mgmt.h>
#include <newtmgr/newtmgr.h>
#include "console/console.h"
#include <nmgr_os/nmgr_os.h>
#include <mgmt/mgmt.h>
#include <imgmgr/imgmgr.h>
#include <bootutil/image.h>

#include <tinycbor/cbor.h>
#include <tinycbor/cborjson.h>
#include <tinycbor/cbor_mbuf_writer.h>
#include <tinycbor/cbor_mbuf_reader.h>
#include <cborattr/cborattr.h>
#include <nmgr_uwb/nmgr_uwb.h>
#include <nmgr_cmds/nmgr_cmds.h>

#include "shell/shell.h"
#include "base64/hex.h"

#define IMG_UPLOAD_CBOR_OVERHEAD (sizeof(struct nmgr_hdr) + sizeof("\"data\"") + sizeof("\"len\"") \
                                 + sizeof("\"off\"") + sizeof(uint32_t) + sizeof(uint32_t)) //Length of "len" + "off" + "data"
#define RETRY_COUNT 5
#define INITIAL_OS_MBUF_SIZE 256
#define NMGR_CMD_STACK_SIZE 1024

//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif

static bool rx_complete_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);
static bool rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs);

static int nmgr_uwb_img_upload(int argc, char** argv);
static int nmgr_uwb_img_list(int argc, char** argv);
static int nmgr_uwb_img_set_state(int argc, char** argv);
static struct os_mbuf* buf_to_imgmgr_mbuf(uint8_t *buf, uint64_t len, uint64_t off, uint32_t size);

static dw1000_mac_interface_t g_cbs[] = {
        [0] = {
            .id = DW1000_NMGR_CMD,
            .rx_complete_cb = rx_complete_cb,
            .rx_timeout_cb = rx_timeout_cb,
        },
#if MYNEWT_VAL(DW1000_DEVICE_1)
        [1] = {
            .id = DW1000_NMGR_CMD,
            .rx_complete_cb = rx_complete_cb,
            .rx_timeout_cb = rx_timeout_cb,
        },
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
        [2] = {
            .id = DW1000_NMGR_CMD,
            .rx_complete_cb = rx_complete_cb,
            .rx_timeout_cb = rx_timeout_cb,
        }
#endif
};

struct shell_cmd_help help[] = {
    {
        .summary = "Command to upload an image to a slave node",
        .usage = "img_upload slot_num(1/2) dst_add(0x1234)", 
    },
    {
        .summary = "Command to change the flags after an upload",
        .usage = "To change the image to pending state\nimg_set_state test hash(32 bytes) dst_add(0x1234)\nTo make the image permenant\nimg_set_state confirm dst_add(0x1234)",
    },
    {
        .summary = "List the details of the current images in the slave node",
        .usage = "img_list dst_add(0x1234)",
    }
};

struct shell_cmd imgmgr_cli_cmds[] = {
    {
        .sc_cmd = "img_upload",
        .sc_cmd_func = nmgr_uwb_img_upload,
        .help = &help[0], 
    },
    {
        .sc_cmd = "img_set_state",
        .sc_cmd_func = nmgr_uwb_img_set_state,
        .help = &help[1],
    },
    {
        .sc_cmd = "img_list",
        .sc_cmd_func = nmgr_uwb_img_list,
        .help = &help[2],
    },
};

#define CLI_CMD_NUM sizeof(imgmgr_cli_cmds)/sizeof(struct shell_cmd)

/**
 * API to initialise the rng package.
 *
 *
 * @return void
 */
static void rx_post_process(struct os_event* ev);

typedef struct _nmgr_cmd_instance_t {
    struct os_sem cmd_sem;
    struct os_callout rx_callout;
    struct mgmt_cbuf n_b;
    struct cbor_mbuf_writer writer;
    struct cbor_mbuf_reader reader;
    CborEncoder payload_enc;
    struct os_eventq nmgr_eventq;
    struct os_task nmgr_task;
    uint16_t repeat_mode;
    uint16_t rem_len;
    uint16_t cmd_id;
    uint32_t curr_off;
    uint8_t err_status;
    uint8_t frame_seq_num;
    uint8_t nmgr_cmd_seq_num;
    struct os_mbuf *rx_pkt;
    os_stack_t *pstack;
    dw1000_dev_instance_t* parent;
}nmgr_cmd_instance_t;

static nmgr_cmd_instance_t *nmgr_inst = NULL;

static void
nmgr_cmd_task(void *arg);

void nmgr_cmds_pkg_init(void){

    printf("{\"utime\": %lu,\"msg\": \"cmd_pkg_init\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
    nmgr_inst = (nmgr_cmd_instance_t*)malloc(sizeof(nmgr_cmd_instance_t));
    memset(nmgr_inst, 0x00, sizeof(nmgr_cmd_instance_t));    

#if MYNEWT_VAL(DW1000_DEVICE_0)
    SYSINIT_ASSERT_ACTIVE();
    dw1000_mac_append_interface(hal_dw1000_inst(0), &g_cbs[0]);
    nmgr_inst->parent = hal_dw1000_inst(0);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_1)
    SYSINIT_ASSERT_ACTIVE();
    dw1000_mac_append_interface(hal_dw1000_inst(1), &g_cbs[1]);
    nmgr_inst->parent = hal_dw1000_inst(1);
#endif
#if MYNEWT_VAL(DW1000_DEVICE_2)
    SYSINIT_ASSERT_ACTIVE();
    dw1000_mac_append_interface(hal_dw1000_inst(2), &g_cbs[2]);
    nmgr_inst->parent = hal_dw1000_inst(2);
#endif
    for(int i =0; i < CLI_CMD_NUM; i++)
        shell_cmd_register(&imgmgr_cli_cmds[i]);

    os_sem_init(&nmgr_inst->cmd_sem, 0x1);

    os_eventq_init(&nmgr_inst->nmgr_eventq);
    
    nmgr_inst->pstack = malloc(sizeof(os_stack_t)*OS_STACK_ALIGN(NMGR_CMD_STACK_SIZE));
    os_task_init(&nmgr_inst->nmgr_task, "nmgr_cmd_task",
            nmgr_cmd_task,
            NULL,
            8,
            OS_WAIT_FOREVER,
            nmgr_inst->pstack,
            OS_STACK_ALIGN(NMGR_CMD_STACK_SIZE));
    os_callout_init(&nmgr_inst->rx_callout, &nmgr_inst->nmgr_eventq, rx_post_process, (void*)hal_dw1000_inst(0));

}


static void
nmgr_cmd_task(void *arg){
    
    struct os_eventq *nmgr_eventq = &nmgr_inst->nmgr_eventq;
    while(1){
        os_eventq_run(nmgr_eventq);
    }
}

static int
nmgr_uwb_img_upload(int argc, char** argv){

    int slot_num = 0;
    if(argc < 3){
        return -1;
    }
    if(strcmp(argv[1],"slot1") == 0)
        slot_num = 0;
    else if(strcmp(argv[1],"slot2") == 0)
        slot_num = 1;

    assert(nmgr_inst);
    uint16_t dst_add = strtol(argv[2], NULL, 16);
    if (!dst_add) {
        console_printf("Dst addr needed\n");
        return 1;
    }

    dw1000_dev_instance_t * inst = nmgr_inst->parent;
    
    uint8_t retries = 0;
    int  len = 0;
    uint32_t flags =0, off = 0;
    struct os_mbuf *rsp =  NULL;
    uint8_t buf[NMGR_UWB_MTU_STD - IMG_UPLOAD_CBOR_OVERHEAD];
    const struct flash_area *s_fa;
    struct image_version ver;
    int fa_id = flash_area_id_from_image_slot(slot_num);
    
    imgr_read_info(slot_num, &ver, 0, &flags);
    printf("ver: %d.%d.%d.%lu\n", ver.iv_major, ver.iv_minor,
                 ver.iv_revision, ver.iv_build_num);
    flash_area_open(fa_id, &s_fa);
    while(off < s_fa->fa_size){

        os_error_t err = os_sem_pend(&nmgr_inst->cmd_sem, OS_TIMEOUT_NEVER);
        assert(err == OS_OK);
        if (off < s_fa->fa_size) {
            len = (int)s_fa->fa_size - (int)off;
            len = (len>sizeof(buf)) ? sizeof(buf) : len;
            flash_area_read(s_fa, off, buf, len);

            rsp = buf_to_imgmgr_mbuf(buf, len, off, s_fa->fa_size);
            if (rsp == NULL) {
                printf("Could not convert flash data to mbuf\n");
                return 0;
            }
            off += len;
        } else {
            rsp = NULL;
        }

        int rc = uwb_nmgr_queue_tx(inst, dst_add, 0, rsp);
        if (rc) {
            /* Failed to send */
            os_time_delay(OS_TICKS_PER_SEC/10);
        }

        err = os_sem_pend(&nmgr_inst->cmd_sem, OS_TIMEOUT_NEVER);
        err |= os_sem_release(&nmgr_inst->cmd_sem);
        assert(err == OS_OK);
        if(nmgr_inst->err_status != 0){
            if(inst->status.rx_timeout_error){
                retries++;
                inst->status.rx_timeout_error = 0;
                if(retries > RETRY_COUNT){
                    nmgr_inst->curr_off = 0;
                    goto err;
                }else
                    off -= len;
            }else{
                printf("Err %d occured \n", nmgr_inst->err_status);
                printf("Refer to mgmt.h for more info \n");
                nmgr_inst->curr_off = 0;
                goto err;
            }
        }else{ 
            retries = 0;
            //If the offset was wrong then update the offset with the received offset
            off = nmgr_inst->curr_off;
            printf("\b\b\b\b\b\b%ld%%", ((uint32_t)((off * 100)/s_fa->fa_size)));
        }
    }
    printf(" \n");
err:
    return 0; 
}

static int
nmgr_uwb_img_list(int argc, char** argv){

    if(argc < 2){
        return -1;
    }
    struct os_mbuf* om = os_msys_get_pkthdr(10, 40);
    struct nmgr_hdr *hdr = (struct nmgr_hdr *) os_mbuf_extend(om, sizeof(struct nmgr_hdr));
    hdr->nh_len = 0;
    hdr->nh_flags = 0;
    hdr->nh_op = NMGR_OP_READ;
    hdr->nh_group = htons(MGMT_GROUP_ID_IMAGE);
    hdr->nh_seq = nmgr_inst->nmgr_cmd_seq_num++;
    hdr->nh_id = IMGMGR_NMGR_ID_STATE;
    uint16_t dst_addr = strtol(argv[1], NULL, 16);
    if (!dst_addr) {
        console_printf("Dst addr needed\n");
        return 1;
    }

    uwb_nmgr_queue_tx(hal_dw1000_inst(0), dst_addr, 0, om);
    return 0;
}

static int
nmgr_uwb_img_set_state(int argc, char** argv){
    assert(nmgr_inst);

    if(strcmp(argv[1], "test") == 0 && argc < 3){
        printf("Please provide the hash of the image to be tested \n");
        return -1;
    }else if(strcmp(argv[1], "confirm") != 0 && strcmp(argv[1], "test") != 0){
        printf("Unknown state provided \n");
        return -1;
    }
    uint16_t dst_add = 0x0000;

    int rc = 0;
    struct os_mbuf* tx_pkt = os_msys_get_pkthdr(NMGR_UWB_MTU_STD - sizeof(struct nmgr_hdr), 0);
    struct nmgr_hdr *hdr = os_mbuf_extend(tx_pkt, sizeof(struct nmgr_hdr));
    
    hdr->nh_len = 0;
    hdr->nh_flags = 0;
    hdr->nh_op = NMGR_OP_WRITE;
    hdr->nh_group = htons(MGMT_GROUP_ID_IMAGE);
    hdr->nh_seq = nmgr_inst->nmgr_cmd_seq_num++;
    hdr->nh_id = IMGMGR_NMGR_ID_STATE;

    cbor_mbuf_writer_init(&nmgr_inst->writer, tx_pkt);
    cbor_encoder_init(&nmgr_inst->n_b.encoder, &nmgr_inst->writer.enc, 0);
    rc = cbor_encoder_create_map(&nmgr_inst->n_b.encoder, &nmgr_inst->payload_enc, CborIndefiniteLength);
    if (rc != 0) {
        printf("could not create map\n");
        return 0;
    }
    struct mgmt_cbuf *cb = &nmgr_inst->n_b;

    CborError g_err = CborNoError;
    if(strcmp(argv[1], "test") == 0){
        dst_add = strtol(argv[3], NULL, 16);
        uint8_t* hash_bytes[32] = {0};
        hex_parse(argv[2], strlen(argv[2]), hash_bytes, 32);

        g_err |= cbor_encode_text_stringz(&cb->encoder, "hash");
        g_err |= cbor_encode_byte_string(&cb->encoder,(const uint8_t*)hash_bytes, 32);
        g_err |= cbor_encode_text_stringz(&cb->encoder, "confirm");
        g_err |= cbor_encode_boolean(&cb->encoder, false);
    }else{
        dst_add = strtol(argv[2], NULL, 16);
        g_err |= cbor_encode_text_stringz(&cb->encoder, "confirm");
        g_err |= cbor_encode_boolean(&cb->encoder, true);
    }
    rc = cbor_encoder_close_container(&nmgr_inst->n_b.encoder, &nmgr_inst->payload_enc);
    if (rc != 0) {
        printf("could not close container\n");
        return 0;
    }
    hdr->nh_len += cbor_encode_bytes_written(&nmgr_inst->n_b.encoder);
    hdr->nh_len = htons(hdr->nh_len);

    uwb_nmgr_queue_tx(hal_dw1000_inst(0), dst_add, 0, tx_pkt);
    return 0;
}

static struct os_mbuf*
buf_to_imgmgr_mbuf(uint8_t *buf, uint64_t len, uint64_t off, uint32_t size)
{
    int rc;
    struct os_mbuf *tx_pkt = os_msys_get_pkthdr(len, 0);
    if (!tx_pkt){
        printf("could not get mbuf\n");
        return 0;
    }
    
    struct nmgr_hdr *hdr = os_mbuf_extend(tx_pkt, sizeof(struct nmgr_hdr));
    hdr->nh_len = 0;
    hdr->nh_flags = 0;
    hdr->nh_op = NMGR_OP_WRITE;
    hdr->nh_group = htons(MGMT_GROUP_ID_IMAGE);
    hdr->nh_seq = nmgr_inst->nmgr_cmd_seq_num++;
    hdr->nh_id = IMGMGR_NMGR_ID_UPLOAD;

    cbor_mbuf_writer_init(&nmgr_inst->writer, tx_pkt);
    cbor_encoder_init(&nmgr_inst->n_b.encoder, &nmgr_inst->writer.enc, 0);
    rc = cbor_encoder_create_map(&nmgr_inst->n_b.encoder, &nmgr_inst->payload_enc, CborIndefiniteLength);
    if (rc != 0) {
        printf("could not create map\n");
        return 0;
    }

    struct mgmt_cbuf *cb = &nmgr_inst->n_b;
    
    CborError g_err = CborNoError;
    g_err |= cbor_encode_text_stringz(&cb->encoder, "len");
    g_err |= cbor_encode_uint(&cb->encoder, size);
    g_err |= cbor_encode_text_stringz(&cb->encoder, "off");
    g_err |= cbor_encode_uint(&cb->encoder, off);
    g_err |= cbor_encode_text_stringz(&cb->encoder, "data");
    g_err |= cbor_encode_byte_string(&cb->encoder, buf, len);
    
    rc = cbor_encoder_close_container(&nmgr_inst->n_b.encoder, &nmgr_inst->payload_enc);
    if (rc != 0) {
        printf("could not close container\n");
        return 0;
    }
    hdr->nh_len += cbor_encode_bytes_written(&nmgr_inst->n_b.encoder);
    hdr->nh_len = htons(hdr->nh_len);
    
    return tx_pkt;
}


/**
 * API for receive timeout callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
static bool 
rx_timeout_cb(dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs){
    if (strncmp((char*)&inst->fctrl, "NM", 2)){
        return false;
    }
    nmgr_inst->err_status = 1;
    nmgr_inst->repeat_mode = 0;
    if(os_sem_get_count(&nmgr_inst->cmd_sem) == 0)
        os_sem_release(&nmgr_inst->cmd_sem);
    return false;
}

/**
 * API for receive complete callback.
 *
 * @param inst  Pointer to dw1000_dev_instance_t.
 *
 * @return true on sucess
 */
static bool 
rx_complete_cb(struct _dw1000_dev_instance_t * inst, dw1000_mac_interface_t * cbs)
{
    if (strncmp((char*)&inst->fctrl, "NM", 2)){
        return false;
    }
    nmgr_uwb_frame_t *frame = (nmgr_uwb_frame_t*)inst->rxbuf;
    if(inst->my_short_address != frame->dst_address){
        return true;
    }else{
        os_eventq_put(&nmgr_inst->nmgr_eventq, &nmgr_inst->rx_callout.c_ev);
    }
    return true;
}

static void
rx_post_process(struct os_event* ev){
    dw1000_dev_instance_t * inst = hal_dw1000_inst(0);
    nmgr_uwb_frame_t *frame = (nmgr_uwb_frame_t*)inst->rxbuf;
    
    //There is a chance that the response could be split if the total length is more than NMGR_UWB_MTU
    //If so then do the decoding only after the entire packet is received
    if(nmgr_inst->repeat_mode == 0){
        nmgr_inst->rx_pkt = os_msys_get_pkthdr(htons(frame->hdr.nh_len), 0);
        if(nmgr_inst->rx_pkt == NULL){
            nmgr_inst->err_status = -1;
            if(os_sem_get_count(&nmgr_inst->cmd_sem) == 0)
                os_sem_release(&nmgr_inst->cmd_sem);
            return;
        }
        //Trim out the uwb frame header
        int rc = os_mbuf_copyinto(nmgr_inst->rx_pkt, 0, &frame->array[sizeof(struct _nmgr_uwb_header)], inst->frame_len - sizeof(struct _nmgr_uwb_header));
        assert(rc==0);

        if(htons(frame->hdr.nh_len) > NMGR_UWB_MTU_EXT && 0){
            nmgr_inst->repeat_mode = 1;
            nmgr_inst->rem_len = (htons(frame->hdr.nh_len) + sizeof(struct nmgr_hdr)) - OS_MBUF_PKTLEN(nmgr_inst->rx_pkt);
            uint16_t timeout = dw1000_phy_frame_duration(&inst->attrib, 128) + 0x600;
            dw1000_set_rx_timeout(inst, timeout);
            dw1000_start_rx(inst);
        }
        nmgr_inst->cmd_id = frame->hdr.nh_id;
    }
    else{
        nmgr_inst->rem_len -= (inst->frame_len - sizeof(struct _nmgr_uwb_header));
        if(nmgr_inst->rem_len == 0)
            nmgr_inst->repeat_mode = 0;
        else{
            uint16_t timeout = dw1000_phy_frame_duration(&inst->attrib, 128) + 0x600;
            dw1000_set_rx_timeout(inst, timeout);
            dw1000_start_rx(inst);
        }
        uint16_t cur_len = OS_MBUF_PKTLEN(nmgr_inst->rx_pkt);
        os_mbuf_copyinto(nmgr_inst->rx_pkt, cur_len, &frame->array[sizeof(struct _nmgr_uwb_header)], inst->frame_len - sizeof(struct _nmgr_uwb_header));
    }
    //Start decoding
    if(nmgr_inst->repeat_mode == 0){

        cbor_mbuf_reader_init(&nmgr_inst->reader, nmgr_inst->rx_pkt, sizeof(struct nmgr_hdr));
        cbor_parser_init(&nmgr_inst->reader.r, 0, &nmgr_inst->n_b.parser, &nmgr_inst->n_b.it);

        if(nmgr_inst->cmd_id == IMGMGR_NMGR_ID_UPLOAD){
            long long int rc = 0, off = 0;
            char rsn[15] = "\0";
            struct cbor_attr_t attrs[4] = {
                [0] = {
                    .attribute = "rc",
                    .type = CborAttrIntegerType,
                    .addr.integer = &rc,
                    .nodefault = true,
                },
                [1] = {
                    .attribute = "off",
                    .type = CborAttrIntegerType,
                    .addr.integer = &off,
                    .nodefault = true,
                },
                [2] = {
                    .attribute = "rsn",
                    .type = CborAttrTextStringType,
                    .addr.string = rsn,
                    .nodefault = 1,
                    .len = sizeof(rsn),

                },
                [3] = {
                    .attribute = NULL
                }
            };
            rc = cbor_read_object(&nmgr_inst->n_b.it, attrs);
            if(rc != 0){
                nmgr_inst->err_status = (uint8_t)rc;
                
            }else{
                nmgr_inst->err_status = 0;
                nmgr_inst->curr_off = off;
            }
            os_mbuf_free_chain(nmgr_inst->rx_pkt);
            os_sem_release(&nmgr_inst->cmd_sem);
        }else if(nmgr_inst->cmd_id == IMGMGR_NMGR_ID_STATE){
            struct h_obj {
                char hash[IMGMGR_HASH_LEN];
                char version[32];
                long long int rc;
                long long int slot;
                bool bootable;
                bool active;
                bool confirmed;
                bool pending;
                bool permenant;
            } arr_objs[2];
            char hash_str[IMGMGR_HASH_LEN * 2 + 1];
            long long int rc = 0;
            int count = 0;
            memset(&arr_objs[0], 0xff, 2 * sizeof(struct h_obj));
            struct cbor_attr_t attrs[9] = {
                [0] = {
                    .attribute = "hash",
                    .type = CborAttrByteStringType,
                    CBORATTR_STRUCT_OBJECT(struct h_obj, hash),
                    .nodefault = 1,
                    .len = sizeof(arr_objs[0].hash),
                },
                [1] = {
                    .attribute = "slot",
                    .type= CborAttrIntegerType,
                    CBORATTR_STRUCT_OBJECT(struct h_obj, slot),
                    .nodefault = true,
                },
                [2] = {
                    .attribute = "version",
                    .type = CborAttrTextStringType,
                    CBORATTR_STRUCT_OBJECT(struct h_obj, version),
                    .len = sizeof(arr_objs[0].version),
                },
                [3] = {
                    .attribute = "bootable",
                    .type= CborAttrBooleanType,
                    CBORATTR_STRUCT_OBJECT(struct h_obj, bootable),
                    .nodefault = true,
                },
                [4] = {
                    .attribute = "active",
                    .type= CborAttrBooleanType,
                    CBORATTR_STRUCT_OBJECT(struct h_obj, active),
                    .nodefault = true,
                },
                [5] = {
                    .attribute = "confirmed",
                    .type= CborAttrBooleanType,
                    CBORATTR_STRUCT_OBJECT(struct h_obj, confirmed),
                    .nodefault = true,
                },
                [6] = {
                    .attribute = "pending",
                    .type= CborAttrBooleanType,
                    CBORATTR_STRUCT_OBJECT(struct h_obj, pending),
                    .nodefault = true,
                },
                [7] = {
                    .attribute = "permanent",
                    .type= CborAttrBooleanType,
                    CBORATTR_STRUCT_OBJECT(struct h_obj, permenant),
                    .nodefault = true,
                },
                [8] = {
                    NULL,
                },
            };
            struct cbor_attr_t arr[3] = {
                [0]={
                    .attribute = "images",
                    .type = CborAttrArrayType,
                    CBORATTR_STRUCT_ARRAY(arr_objs, attrs, &count)
                },
                [1]={
                    .attribute = "rc",
                    .type = CborAttrIntegerType,
                    .addr.integer = &rc,
                    .nodefault = true,
                },
                [2] = {
                    NULL,
                },
            };
            int err = cbor_read_object(&nmgr_inst->n_b.it, arr);
            if(rc != 0LL || err != 0){
                nmgr_inst->err_status = 1;
                printf("Wrong hash sent %lld %d\n", rc, err);
                goto err;
            }else
                nmgr_inst->err_status = 0;
            for(int i =0; i< count; i++){
                printf("\nSlot = %lld \n", arr_objs[i].slot);
                printf("Version = %s \n", arr_objs[i].version);
                if(arr_objs[i].bootable)
                    printf("bootable : true \n");
                printf("Flags : ");
                if(arr_objs[i].active)
                    printf("active ");
                if(arr_objs[i].confirmed)
                    printf("confirmed ");
                if(arr_objs[i].pending)
                    printf("pending ");
                printf("\n%s\n", hex_format(arr_objs[i].hash, IMGMGR_HASH_LEN, hash_str, sizeof(hash_str)));
            }
err:
            os_mbuf_free_chain(nmgr_inst->rx_pkt);
            
        }else{
            printf("Unrecognized command \n");
            os_mbuf_free_chain(nmgr_inst->rx_pkt);
        }
    }
}

struct os_eventq*
nmgr_cmds_get_eventq()
{
    return &nmgr_inst->nmgr_eventq;
}
