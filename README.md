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

This distribution contains the device driver model for the dw1000 IR-UWB transceiver within the mynewt-OS. The driver includes hardware abstraction layers, low-level MAC, Ranging Services and raw-lwip stacks. The dw1000 driver and mynewt-OS combine to create a hardware and architecture agnostic platform for IoT Location Based Services. Augmented this with the newtmgt management tools to create a compelling environment and deploying large-scale distributions within IoT and beyond. The newt management tool uses a repo methodology for package management. The driver contained herein becomes a dependent repo for any DW1000 related project and is automatically included in projects as required––the https://github.com/devawave/mynewt-dw1000-apps showcase this relationship.

This repo also contains the board support package (BSP) for the Decawave dwm1001 module and dwm1001-dev kit. The dwm1001 includes a nrf52832 and the dw1000 transceiver. The dwm1001-dev is a breakout board that supports a Seggar OB-JLink interface with RTT support. The mynewt build environment provides a clean interface for maintaining BSP, and the contained dwm1001 BSP can serve as a template for other dw1000 enabled platforms. The dwm1001-dev and the driver contain herein provide a clean out-of-the-box experience for UWB Location Based Services.    

    Also supported are the DWM1002 and DWM1003 which are AoA varients of the DWM1001 with 10DoF IMU.  


A single DW1000 UWB transceiver can be used to form a 6LowPAN mesh network while concurrent measuring distance between nodes (Range). Dual dw1000 devices can be used to measure Range and Azimuth also known as Angle-of-Arrival (AoA). With this intent, this driver is thread-safe and multi-instance. The driver has a hierarchical architecture and is partitioned into functional groups. 

## Project Status

This project is destined to be up-streamed into the mynewt repo Q1 2018:

* DW1000 Device Driver
* DW1000 TWR Services
* DW1000 LWIP Driver
* DW1000 CCP Driver
* Example (see companion repo mynewt-dw1000-apps)

## Current BSPs and supported hardware
* DWM1001-DEV
* DWM1002-DEV
* DWM1003-DEV
* lps2mini
* lps2nano

## File Description
```
dw1000/
├── include
│   └── dw1000
│       ├── dw1000_ccp.h
│       ├── dw1000_dev.h
│       ├── dw1000_ftypes.h
│       ├── dw1000_gpio.h
│       ├── dw1000_hal.h
│       ├── dw1000_lwip.h
│       ├── dw1000_mac.h
│       ├── dw1000_otp.h
│       ├── dw1000_phy.h
│       ├── dw1000_regs.h
│       ├── dw1000_rng.h
│       └── triad.h
├── pkg.yml
├── src
│   ├── dw1000_ccp.c     // Clock calibration packets
│   ├── dw1000_dev.c     // Driver instance
│   ├── dw1000_gpio.c    // GPIO interface
│   ├── dw1000_hal.c     // Hardware abstraction
│   ├── dw1000_lwip.c    // lwip stack lower layer
│   ├── dw1000_mac.c     // MAC lower layer
│   ├── dw1000_otp.c     // One Time Programming
│   ├── dw1000_phy.c     // Physical layer controller
│   ├── dw1000_pkg.c
│   └── dw1000_rng.c    // Ranging services
└── syscfg.yml
```

## Building

See the companion repo mynewt-dw1000-apps for building instructions, recall that this driver will is pulled into projects as a dependency and will be build from within that project. As such build instructions are light here. 


