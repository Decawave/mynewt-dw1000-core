<!--
# Copyright (C) 2017-2018, Decawave Limited, All Rights Reserved
#
# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
#  KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.
#
-->

# mynewt-dw1000-core

## Overview

The distribution <https://github.com/decawave/mynewt-dw1000-core> contains the device driver model for the Decawave DW1000 Impulse Radio-Ultra Wideband (IR-UWB) transceiver within the Mynewt-OS. The driver includes hardware abstraction layers (HAL), media access control (MAC) layer, Ranging Services (RNG) and light weight IP (lwip) stacks. The DW1000 driver and Mynewt-OS combine to create a hardware and architecture agnostic platform for IoT Location Based Services (LBS). This augmented with the newtmgt management tools creates a compelling environment for large-scale deployment of LBS. The newt management tool uses a repo methodology for package management. The driver contained herein becomes a dependent repo for any DW1000 related project and is automatically included in projects as required––the <https://github.com/decawave/mynewt-dw1000-apps> showcases this relationship.

This repo also contains the board support package (BSP) for the Decawave dwm10001 module and dwm1001-dev kit. The dwm1001 includes a nrf52832 and the dw1000 transceiver. dwm1001-dev is a breakout board that supports a Seggar OB-JLink interface with RTT support. The mynewt build environment provides a clean interface for maintaining these BSPs, and the contained DWM1001 BSP can serve as a template for other DW1000 enabled platforms. The dwm1001-dev and the driver contained herein provide a clean out-of-the-box experience for UWB LBS products.

A single dw1000 transceiver can be used to form a 6LowPAN mesh network while concurrently measuring distance between nodes. Dual DW1000 devices can be used to measure Range and Azimuth also known as Angle-of-Arrival (AoA). With this intent, this driver is thread-safe and multi-instance. The driver has a hierarchical architecture and is partitioned into functional groups. 

## Under-the-hood

The mynewt-dw1000-core driver implements the MAC layers and exports a MAC extension interface for additional services, this MAC interface is defined in the struct _dw1000_mac_interface_t found in (../master/hw/driver/dw1000/include/dw1000_dev.h)


### Ranging Services (RNG)

Ranging services binds to the MAC interface; this interfaces expose callbacks to various events within the ranging process. The driver currently support the following ranging profiles;

### Default Config:

| Config  | Description          |  Value  |
| ------------- |:-------------:| -----:|
| PRF  | Pulse Repetition Frequency   |  64MHz  |
| PLEN      | Preamble length         | 128  |
| NPHR      | Number of symbols       | 16  |
| SDF     | start of frame deliminator length  |  8 |
| DataRate     |Data Rate       | 6.8Mbps |

### RNG profile:
| profile       | Description          | Benchmark  |
| ------------- |:-------------:| -----:|
| twr_ss        | Single Sided Two Way Ranging | 1110us|
| twr_ds      | Double Sided Two Way Ranging      |  2420us |
| twr_ds_ext | DS-TWR /w extended data payload      |   2775us |

### NRNG profile:

| profile       | Description  | Benchmark  |
| ------------- |:-------------:| -----:|
| nrng_ds | n twr_ds ranges with 2*n+2 messages  | 6200us for n=4|


### Clock Calibration Packet (CCP) Service

### Time Division Multiple Access (TDMA) Service

### Light Weight IP (lwIP) Service

## Project Status

The mynewt-dw1000-core repo is still a work in progress with the following extension in development:

| New Fetures   | Description          | schedule |
| ------------- |:-------------:| -----:|
| TDOA  | Time Difference of Arrival profiles| 18Q4|
| PDOA  | Phase Difference of Arrival profiles| 18Q4|
| OpenThread Shim  | MAC extension supporting OpenThread stack| 18Q4|

## Current BSPs and supported hardware
* DWM1001   from <https://www.decawave.com/products/dwm1001-module>
* DWM1002   from <https://decawave.com>
* DWM1003   from <https://decawave.com>
* lps2mini  from <https://loligoelectronics.com>
* lps2nano  from <https://loligoelectronics.com>

## File Description
```
├── dw1000
│   ├── pkg.yml                 // Project file
│   ├── src
│   │   ├── dw1000_ccp.c        // Clock calibration packets
│   │   ├── dw1000_dev.c        // Driver instance
│   │   ├── dw1000_gpio.c       // GPIO interface
│   │   ├── dw1000_hal.c        // Hardware abstraction
│   │   ├── dw1000_mac.c        // MAC lower layer
│   │   ├── dw1000_otp.c        // One Time Programming API
│   │   ├── dw1000_phy.c        // Physical layer controller
│   │   ├── dw1000_pkg.c        // Mynewt pkg API
│   └── syscfg.yml              // Project config
├── bsp                         // Board Support Packages
│   ├── dwm1001                 // BSP for DWM1001 TWR/TDOA Module
│   ├── dwm1002                 // BSP for DWM1002 Dual DW1000 PDOA Node
│   ├── dwm1003                 // BSP for DWM1002 TWR/PDOA/IMU TAG
│   ├── lps2mini                // BSP for LPS2MINI board from <https://loligoelectronics.com>
│   └── lps2nano                // BSP for LPS2NANO board from <https://loligoelectronics.com>
├── ccp                         // Clock Calibration Packet synchronization 
├── dsp                         // Signal Proceesing library
├── lwip                        // Light weight IP extension
├── nrng                        // N ranges in 2*N+2 messages
├── rng                         // TWR toplevel API
├── tdma                        // Time Devision Multiplex API
├── twr_ds                      // Double Sided TWR
├── twr_ds_ext                  // Double Sided TWR with extended payload
└── twr_ss                      // Single Sided TWR

```

## Building

See the companion repo https://github.com/decawave/mynewt-dw1000-apps for building instructions. Recall that the above driver will be cloned as a dependent repo and will be built from within that parent project. 


