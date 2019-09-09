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

/**
 * Configure a channel on the UWB device.
 *
 * @param dev The device to configure
 * @param cnum The channel number to configure
 * @param data Driver specific configuration data for this channel.
 *
 * @return 0 on success, non-zero error code on failure.
 */
int
uwb_chan_config(struct uwb_dev *dev, uint8_t cnum, void *data)
{
    assert(dev->ad_funcs->af_configure_channel != NULL);

    if (cnum >= dev->ad_chan_count) {
        return (EINVAL);
    }

    return (dev->ad_funcs->af_configure_channel(dev, cnum, data));
}

/**
 * Blocking read of an UWB channel.
 *
 * @param dev The UWB device to read
 * @param cnum The channel number to read from that device
 * @param result Where to put the result of the read
 *
 * @return 0 on success, non-zero on error
 */
int
uwb_chan_read(struct uwb_dev *dev, uint8_t cnum, int *result)
{
    assert(dev->ad_funcs->af_read_channel != NULL);

    if (cnum >= dev->ad_chan_count) {
        return (EINVAL);
    }

    if (!dev->ad_chans[cnum].c_configured) {
        return (EINVAL);
    }

    return (dev->ad_funcs->af_read_channel(dev, cnum, result));
}

/**
 * Set an event handler.  This handler is called for all UWB events.
 *
 * @param dev The UWB device to set the event handler for
 * @param func The event handler function to call
 * @param arg The argument to pass the event handler function
 *
 * @return 0 on success, non-zero on failure
 */
int
uwb_event_handler_set(struct uwb_dev *dev, uwb_event_handler_func_t func,
        void *arg)
{
    dev->ad_event_handler_func = func;
    dev->ad_event_handler_arg = arg;

    return (0);
}

