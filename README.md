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

# DW1000 mynewt OS driver 

## Overview

This distribution contains the device driver model for the dw1000 IR-UWB transceiver within the mynewt-OS. The driver includes hardware abstraction layers, low-level MAC, Ranging Services and raw-lwip stacks. The dw1000 driver and mynewt-OS combination to create a hardware and architecture agnostic platform for IoT Location Based Services. Augmented this with the newtmgt management tools to create a powerful environment and deploying large-scale distributions within IoT and beyond. The newt management tool uses a repo methodology for package management. The driver contained herein becomes a dependent repo for any DW1000 related project and is automatically included in projects as required––the https://github.com/devawave/mynewt-dw1000-apps showcase this relationship.

This reps also contains the board support package (BSP) for the Decawave dwm1001 module and dwm1001-dev kit. The dwm1001 includes a nrf52832 and the dw1000 transceiver. The dwm1001-dev is a breakout board that supports a Seggar OB-JLink interface with RTT support. The mynewt build environment provides a clean interface for maintaining BSP, and the contained dwm1001 BSP can serve as a template for other dw1000 enabled platforms. The dwm1001-dev and the driver contain herein provide a clean out-of-the-box experience for UWB Location Based Services. 

Warning: The dwm1001 out-of-the-box is flashed with a UWB Location Based Services stack. This distribution repurposes the hardware and is not intended to replace the functionality of the shipped stack. This distribution is intended to be a starting point for evaluating and developing ones own such stacks. 

A signle DW1000 UWB transceiver can be used to form a 6LowPAN mesh network while concurrent measuring distance between nodes (Range). Dual DW1000 device can be used to measure Range and Azimuth also known as Angle-of-Arrival (AoA). With this intent, this dirver is thread-safe and multi-instance. The driver has a hierarchical architecture and is partitioned into functional groups. 

## Project Status
This project is destined to be up-streamed into the mynewt repo Q1 2018:

* DW1000 Device Driver (complete)
* DWM1001 Board Support Package (complete)
* DW1000 Ranging Services (complete)
* CLI node_cfg (Pending)
* CLI autosurvey (Pending)
* CLI anchor (Pending)
* CLI mlat (Pending)
* CLI network tools, ping, scan etc. (Pending)
* Open-Thread support (Pending) 
* Example (see companion repo mynewt-dw1000-apps)


## File Description
```
└── drivers
│       └── dw1000
│           ├── include
│           │   └── dw1000
│           │       ├── dw1000_dev.h
│           │       ├── dw1000_ftypes.h         // Frame types
│           │       ├── dw1000_gpio.h
│           │       ├── dw1000_hal.h
│           │       ├── dw1000_lwip.h
│           │       ├── dw1000_mac.h
│           │       ├── dw1000_otp.h
│           │       ├── dw1000_phy.h
│           │       ├── dw1000_regs.h
│           │       ├── dw1000_regulatory.h
│           │       └── dw1000_rng.h
│           ├── pkg.yml
│           └── src
│               ├── dw1000_dev.c                // Driver instance
│               ├── dw1000_gpio.c               // DW1000 gpio interface
│               ├── dw1000_hal.c                // Hardware abstraction
│               ├── dw1000_lwip.c               // raw-lwip stack
│               ├── dw1000_mac.c                // MAC lowerlevel
│               ├── dw1000_otp.c                // One Time Programming
│               ├── dw1000_phy.c                // Physical layer controller
│               ├── dw1000_regulatory.c         // Regulatory profiles 
│               └── dw1000_rng.c                // Ranging services
```

## Supported Hardware
* Decawave dwm1001 and companion dwm1001 (Complete)
* Decawave AoA-Node (Pending)
* Decawave TAG-Node (Pending)

## Building

See the companion repo mynewt-dw1000-apps for building instructions, recall that this driver will is pulled into projects as a dependency and will be build form within that project. As such build instructions are light here. 

The dw1000 driver make use of c99 anonymous union extensions, this need to be enabled within the mynewt build enviorement. A patch is provided for the apache-mynewt-core distribution, this can be found at ./mynewt-dw1000-apps/repos/.patches/apache-mynewt-core.patch. This patch simply adds -fms-extensions to the cflag.

