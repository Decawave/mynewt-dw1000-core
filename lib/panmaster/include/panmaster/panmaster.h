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

#ifndef _PANMASTER_H_
#define _PANMASTER_H_

#include <inttypes.h>
#include <bootutil/image.h>
struct image_version;

struct panmaster_node {
    int64_t  first_seen_utc; /*!< When this node was first seen */
    int64_t  euid;           /*!< Unique id, 64bit */
    uint16_t addr;           /*!< Local id, 16bit */
    uint16_t flags:12;       /*!< Flags (unused for now) */
    uint16_t role:4;         /*!< Network role */
    uint8_t  index;          /*!< Index into vector */
    uint8_t  slot_id;        /*!< slot_id */
    struct image_version fw_ver;
} __attribute__((__packed__, aligned(1)));

struct panmaster_node_idx {
    uint16_t addr;           /*!< Local id, 16bit */
    uint8_t slot_id;
    uint8_t role;
    uint32_t lease_ends;
};

struct find_node_s {
    struct panmaster_node find;
    struct panmaster_node *results;
    int is_found;
};


#define PANMASTER_NODE_DEFAULT(N)  {(N).first_seen_utc=0;(N).euid=0;    \
        (N).addr=0xffff;(N).flags=0;(N).index=0;(N).slot_id=0xff;(N).role=0; \
        (N).fw_ver.iv_major=0;(N).fw_ver.iv_minor=0;                    \
        (N).fw_ver.iv_revision=0;(N).fw_ver.iv_build_num=0;}
#define PANMASTER_NODE_IDX_DEFAULT(N)  {(N).addr=0xffff;(N).slot_id=0xff;(N).role=0;}

typedef void (*panm_load_cb)(struct panmaster_node *node, void *cb_arg);

#ifdef __cplusplus
extern "C" {
#endif

void panmaster_pkg_init(void);
int panmaster_find_node(uint64_t euid, uint16_t role, struct panmaster_node **node);
int panmaster_find_node_general(struct find_node_s *fns);
    
int panmaster_load(panm_load_cb cb, void *cb_arg);
void panmaster_save(void);

void panmaster_node_idx(struct panmaster_node_idx **node_idx_arg, int *num_nodes);
int panmaster_clear_list();

void panmaster_add_version(uint64_t euid, struct image_version *ver);
void panmaster_add_node(uint16_t short_addr, uint16_t role, uint8_t *euid_u8);
void panmaster_delete_node(uint64_t euid);

void panmaster_compress();
void panmaster_sort();
uint16_t panmaster_highest_node_addr();

struct os_mbuf* panmaster_cbor_nodes_list(struct os_mbuf_pool *mbuf_pool);
int panmaster_cbor_nodes_list_fa(const struct flash_area *fa, int *fa_offset);
    
#ifdef __cplusplus
}
#endif

#endif /* _PANMASTER_H */
