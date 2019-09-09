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

#ifndef __UWB_H__
#define __UWB_H__

#include <os/os_dev.h>

#ifdef __cplusplus
extern "C" {
#endif

struct uwb_dev;

/**
 * Types of UWB events passed to the UWB driver.
 */
typedef enum {
    /* This event represents the result of an UWB run. */
    UWB_EVENT_RESULT = 0,
    UWB_EVENT_CALIBRATED
} uwb_event_type_t;

/**
 * Event handler for UWB events that are processed in asynchronous mode.
 *
 * @param The UWB device being processed
 * @param The argument data passed to uwb_set_result_handler()
 * @param The event type
 * @param The buffer containing event data
 * @param The size in bytes of that buffer.
 *
 * @return 0 on success, non-zero error code on failure
 */
typedef int (*uwb_event_handler_func_t)(struct uwb_dev *, void *,
    uwb_event_type_t, void *, int);

/**
 * Configure an UWB channel for this UWB device.  This is implemented
 * by the HW specific drivers.
 *
 * @param The UWB device to configure
 * @param The channel number to configure
 * @param An opaque blob containing HW specific configuration.
 *
 * @return 0 on success, non-zero error code on failure.
 */
typedef int (*uwb_configure_channel_func_t)(struct uwb_dev *dev, uint8_t,
        void *);

/**
 * Trigger a sample on the UWB device asynchronously.  This is implemented
 * by the HW specific drivers.
 *
 * @param The UWB device to sample
 *
 * @return 0 on success, non-zero error code on failure
 */
typedef int (*uwb_sample_func_t)(struct uwb_dev *);

/**
 * Blocking read of an UWB channel.  This is implemented by the HW specific
 * drivers.
 *
 * @param The UWB device to perform the blocking read on
 * @param The channel to read
 * @param The result to put the UWB reading into
 *
 * @return 0 on success, non-zero error code on failure
 */
typedef int (*uwb_read_channel_func_t)(struct uwb_dev *, uint8_t, int *);

/**
 * Set the buffer(s) to read UWB results into for non-blocking reads.  This
 * is implemented by the HW specific drivers.
 *
 * @param The UWB device to read results into
 * @param The primary buffer to read results into
 * @param The secondary buffer to read results into (for cases where
 *        DMA'ing occurs, and secondary buffer can be used while primary
 *        is being processed.)
 * @param The length of both buffers in number of bytes
 *
 * @return 0 on success, non-zero error code on failure.
 */
typedef int (*uwb_buf_set_func_t)(struct uwb_dev *, void *, void *, int);

/**
 * Release a buffer for an UWB device, allowing the driver to re-use it for
 * DMA'ing data.
 *
 * @param The UWB device to release the buffer to
 * @param A pointer to the buffer to release
 * @param The length of the buffer being released.
 *
 * @return 0 on success, non-zero error code on failure.
 */
typedef int (*uwb_buf_release_func_t)(struct uwb_dev *, void *, int);

/**
 * Read the next entry from an UWB buffer as a integer
 *
 * @param The UWB device to read the entry from
 * @param The buffer to read the entry from
 * @param The total length of the buffer
 * @param The entry number to read from the buffer
 * @param The result to put the entry into
 *
 * @return 0 on success, non-zero error code on failure.
 */
typedef int (*uwb_buf_read_func_t)(struct uwb_dev *, void *, int, int, int *);

/**
 * Get the size of a buffer
 *
 * @param The UWB device to get the buffer size from
 * @param The number of channels in the buffer
 * @param The number of samples in the buffer
 *
 * @return The size of the buffer
 */
typedef int (*uwb_buf_size_func_t)(struct uwb_dev *, int, int);

struct uwb_driver_funcs {
    uwb_configure_channel_func_t af_configure_channel;
    uwb_sample_func_t af_sample;
    uwb_read_channel_func_t af_read_channel;
    uwb_buf_set_func_t af_set_buffer;
    uwb_buf_release_func_t af_release_buffer;
    uwb_buf_read_func_t af_read_buffer;
    uwb_buf_size_func_t af_size_buffer;
};

struct uwb_chan_config {
    uint16_t c_refmv;
    uint8_t c_res;
    uint8_t c_configured;
    uint8_t c_cnum;
};

struct uwb_dev {
    struct os_dev ad_dev;
    struct os_mutex ad_lock;
    const struct uwb_driver_funcs *ad_funcs;
    struct uwb_chan_config *ad_chans;
    int ad_chan_count;
    /* Open reference count */
    uint8_t ad_ref_cnt;
    uwb_event_handler_func_t ad_event_handler_func;
    void *ad_event_handler_arg;
};

int uwb_chan_config(struct uwb_dev *, uint8_t, void *);
int uwb_chan_read(struct uwb_dev *, uint8_t, int *);
int uwb_event_handler_set(struct uwb_dev *, uwb_event_handler_func_t,
        void *);

/**
 * Sample the device specified by dev.  This is used in non-blocking mode
 * to generate samples into the event buffer.
 *
 * @param dev The device to sample
 *
 * @return 0 on success, non-zero on failure
 */
static inline int
uwb_sample(struct uwb_dev *dev)
{
    return (dev->ad_funcs->af_sample(dev));
}

/**
 * Blocking read of an UWB channel.  This is implemented by the HW specific
 * drivers.
 *
 * @param The UWB device to perform the blocking read on
 * @param The channel to read
 * @param The result to put the UWB reading into
 *
 * @return 0 on success, non-zero error code on failure
 */
static inline int
uwb_read_channel(struct uwb_dev *dev, uint8_t ch, int *result)
{
    return (dev->ad_funcs->af_read_channel(dev, ch, result));
}

/**
 * Set a result buffer to store data into.  Optionally, provide a
 * second buffer to continue writing data into as the first buffer
 * fills up.  Both buffers must be the same size.
 *
 * @param dev The UWB device to set the buffer for
 * @param buf1 The first buffer to spool data into
 * @param buf2 The second buffer to spool data into, while the first
 *             buffer is being processed.  If NULL is provided, it's
 *             unused.
 * @param buf_len The length of both buffers, in bytes.
 *
 * @return 0 on success, non-zero on failure.
 */
static inline int
uwb_buf_set(struct uwb_dev *dev, void *buf1, void *buf2,
        int buf_len)
{
    return (dev->ad_funcs->af_set_buffer(dev, buf1, buf2, buf_len));
}

/**
 * Release a result buffer on the UWB device, and allow for it to be
 * re-used for DMA'ing results.
 *
 * @param dev The device to release the buffer for
 * @param buf The buffer to release
 * @param buf_len The length of the buffer being released.
 *
 * @return 0 on success, non-zero error code on failure.
 */
static inline int
uwb_buf_release(struct uwb_dev *dev, void *buf, int buf_len)
{
    return (dev->ad_funcs->af_release_buffer(dev, buf, buf_len));
}

/**
 * Read an entry from an UWB buffer
 *
 * @param dev The device that the entry is being read from
 * @param buf The buffer that we're reading the entry from
 * @param buf_len The length of the buffer we're reading the entry from
 * @param off The entry number to read from the buffer
 * @param result A pointer to the result to store the data in
 *
 * @return 0 on success, non-zero error code on failure
 */
static inline int
uwb_buf_read(struct uwb_dev *dev, void *buf, int buf_len, int entry,
        int *result)
{
    return (dev->ad_funcs->af_read_buffer(dev, buf, buf_len, entry, result));
}

/**
 * Return the size of an UWB buffer
 *
 * @param dev The UWB device to return the buffer size from
 * @param channels The number of channels for these samples
 * @param samples The number of SAMPLES on this UWB device
 *
 * @return The size of the resulting buffer
 */
static inline int
uwb_buf_size(struct uwb_dev *dev, int chans, int samples)
{
    return (dev->ad_funcs->af_size_buffer(dev, chans, samples));
}

/**
 * Take an UWB result and convert it to millivolts.
 *
 * @param dev The UWB dev to convert the result on
 * @param cnum The channel number to convert the result from
 * @param val The UWB value to convert to millivolts
 *
 * @return The convert value in millivolts
 */
static inline int
uwb_result_mv(struct uwb_dev *dev, uint8_t cnum, int val)
{
    int res;
    int refmv;
    int ret;

    refmv = (int) dev->ad_chans[cnum].c_refmv;
    res = (int) dev->ad_chans[cnum].c_res;

    ret = val * refmv;
    ret += (1 << (res - 2));
    ret = ret >> res;

    return (ret);
}

#ifdef __cplusplus
}
#endif

#endif /* __UWB_H__ */
