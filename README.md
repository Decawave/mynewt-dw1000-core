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

The distribution (https://github.com/decawave/mynewt-dw1000-core) contains the device driver model for the DW1000 IR-UWB transceiver within the mynewt-OS. The driver includes hardware abstraction layers, low-level MAC, Ranging Services and raw-lwip stacks. The DW1000 driver and mynewt-OS combine to create a hardware and architecture agnostic platform for IoT Location Based Services (LBS). This augmented with the newtmgt management tools creates a compelling environment for large-scale deployment of LBS. The newt management tool uses a repo methodology for package management. The driver contained herein becomes a dependent repo for any DW1000 related project and is automatically included in projects as required––the https://github.com/decawave/mynewt-dw1000-apps showcases this relationship.

This repo also contains the board support package (BSP) for the Decawave DWM1001 module and DWM1001-DEV kit. The DWM1001 includes a nrf52832 and the DW1000 transceiver. The DWM1001-DEV is a breakout board that supports a Seggar OB-JLink interface with RTT support. The mynewt build environment provides a clean interface for maintaining these BSPs, and the contained DWM1001 BSP can serve as a template for other DW1000 enabled platforms. The DWM1001-DEV and the driver contained herein provide a clean out-of-the-box experience for UWB LBS products.

A single DW1000 UWB transceiver can be used to form a 6LowPAN mesh network while concurrently measuring distance between nodes (Range). Dual DW1000 devices can be used to measure Range and Azimuth also known as Angle-of-Arrival (AoA). With this intent, this driver is thread-safe and multi-instance. The driver has a hierarchical architecture and is partitioned into functional groups. 

## Project Status

* See companion repo mynewt-dw1000-apps

## Current BSPs and supported hardware
* DWM1001   from https://www.decawave.com/products/dwm1001-module
* DWM1002   from https://decawave.com (coming soon)
* DWM1003   from https://decawave.com (coming soon)
* lps2mini      from https://loligoelectronics.com
* lps2nano      from https://loligoelectronics.com

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
│   ├── lps2mini                // BSP for LPS2MINI board from https://loligoelectronics.com
│   └── lps2nano                // BSP for LPS2NANO board from https://loligoelectronics.com
├── ccp                         // Clock Calibration Packet synchronization 
├── dsp                         // Signal Proceesing library
├── nrng                        // N ranges in 2*N+2 messages
├── rng                         // TWR toplevel API
├── tdma                        // Time Devision Multiplex API
├── twr_ds                      // Double Sided TWR
├── twr_ds_ext                  // Double Sided TWR with extended payload
└── twr_ss                      // Single Sided TWR

```

## Building

See the companion repo https://github.com/decawave/mynewt-dw1000-apps for building instructions. Recall that the above driver will be cloned as a dependent repo and will be built from within that parent project. 


