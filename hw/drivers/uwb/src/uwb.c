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


#include <uwb/uwb.h>
#include <errno.h>
#include <assert.h>
#include <stdio.h>

struct uwb_dev*
uwb_dev_idx_lookup(int idx)
{
    const char base1k[] = "dw1000_%d";
    const char base3k[] = "dw3000_%d";
    char buf[sizeof(base3k) + 2];
    struct os_dev *odev;
    snprintf(buf, sizeof buf, base1k, idx);
    odev = os_dev_lookup(buf);
    if (!odev) {
        snprintf(buf, sizeof buf, base3k, idx);
        odev = os_dev_lookup(buf);
    }

    return (struct uwb_dev*)odev;
}


/**
 * API to register extension  callbacks for different services.
 *
 * @param dev  Pointer to struct uwb_dev
 * @param callbacks  callback instance.
 * @return void
 */
struct uwb_mac_interface *
uwb_mac_append_interface(struct uwb_dev* dev, struct uwb_mac_interface * cbs)
{
    assert(dev);
    assert(cbs);
    cbs->status.initialized = true;

    if(!(SLIST_EMPTY(&dev->interface_cbs))) {
        struct uwb_mac_interface * prev_cbs = NULL;
        struct uwb_mac_interface * cur_cbs = NULL;
        SLIST_FOREACH(cur_cbs, &dev->interface_cbs, next){
            prev_cbs = cur_cbs;
        }
        SLIST_INSERT_AFTER(prev_cbs, cbs, next);
    } else {
        SLIST_INSERT_HEAD(&dev->interface_cbs, cbs, next);
    }

    return cbs;
}


/**
 * API to remove specified callbacks.
 *
 * @param dev  Pointer to struct uwb_dev
 * @param id    ID of the service.
 * @return void
 */
void
uwb_mac_remove_interface(struct uwb_dev* dev, uwb_extension_id_t id)
{
    assert(dev);
    struct uwb_mac_interface * cbs = NULL;
    SLIST_FOREACH(cbs, &dev->interface_cbs, next){
        if(cbs->id == id){
            SLIST_REMOVE(&dev->interface_cbs, cbs, uwb_mac_interface, next);
            break;
        }
    }
}

/**
 * API to return specified callbacks.
 *
 * @param dev  Pointer to struct uwb_dev
 * @param id    ID of the service.
 * @return struct uwb_mac_interface * cbs
 */
struct uwb_mac_interface *
uwb_mac_get_interface(struct uwb_dev* dev, uwb_extension_id_t id)
{
    assert(dev);
    struct uwb_mac_interface * cbs = NULL;
    SLIST_FOREACH(cbs, &dev->interface_cbs, next){
        if(cbs->id == id){
            break;
        }
    }
    return cbs;
}

/**
 *  Finds the first instance pointer to the callback structure 
 *  setup with a specific id.
 *
 * @param dev      Pointer to struct uwb_dev
 * @param id       Corresponding id to find (UWBEXT_CCP,...)
 * @return void pointer to instance, null otherwise
 */
void*
uwb_mac_find_cb_inst_ptr(struct uwb_dev *dev, uint16_t id)
{
    struct uwb_mac_interface * cbs = uwb_mac_get_interface(dev, id);
    if (cbs) {
        return cbs->inst_ptr;
    }
    return 0;
}

/**
 * API to execute each of the interrupt in queue.
 *
 * @param arg  Pointer to the queue of interrupts.
 * @return void
 */
static void *
uwb_interrupt_task(void *arg)
{
    struct uwb_dev * inst = (struct uwb_dev *)arg;
    while (1) {
        dpl_eventq_run(&inst->eventq);
    }
    return NULL;
}

/**
 * The UWB processing of interrupts in a task context instead of the interrupt context such that other interrupts 
 * and high priority tasks are not blocked waiting for the interrupt handler to complete processing. 
 * This uwb softstack needs to coexists with other stacks and sensors interfaces. 
 *
 * @param inst      Pointer to struct uwb_dev.
 * @param irq_ev_cb Pointer to function processing interrupt
 *
 * @return void
 */
void
uwb_task_init(struct uwb_dev * inst, void (*irq_ev_cb)(struct dpl_event*))
{
    /* Check if the task is already initiated */
    if (!dpl_eventq_inited(&inst->eventq))
    {
        /* Use a dedicate event queue for timer and interrupt events */
        dpl_eventq_init(&inst->eventq);
        /*
         * Create the task to process timer and interrupt events from the
         * my_timer_interrupt_eventq event queue.
         */
        dpl_event_init(&inst->interrupt_ev, irq_ev_cb, (void *)inst);
        dpl_task_init(&inst->task_str, "uwb_irq",
                     uwb_interrupt_task,
                     (void *) inst,
                     inst->task_prio, DPL_WAIT_FOREVER,
                     inst->task_stack,
                     UWB_DEV_TASK_STACK_SZ);
    }
}
