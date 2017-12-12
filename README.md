<!--
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

# DW1000 mynewt driver 

## Overview

This distribution contains the device driver model for the DW1000 UWB transceiver within the mynewt-OS. The driver includes hardware abstraction layers, low-level MAC, Ranging Services and raw-lwip stacks. The DW1000 driver and mynewt-OS combination to create a hardware and architecture agnostic platform for IoT location-aware applications. Augmented with the newt package management tools provides a powerful environment and deploying large-scale distributions within IoT and beyond. The newt management tools uses a repo methodology for package management. The driver contained herein becomes a dependent repo for any DW1000 related project and is automatically included in these project––the https://github.com/devawave/mynewt-dw1000-apps showcase this behavior.  

## Project Status
This project is destined to be up-streamed into the mynewt repo early Q1 2018:

* DW1000 Device Driver (complete)
* DWM1001 Board Support Package (complete)
* DW1000 Ranging Services (complete)
* CLI node_cfg (Pending)
* CLI autosurvey (Pending)
* CLI anchor (Pending)
* CLI mlat (Pending)
* Example (see companion repo mynewt-dw1000-apps)

The driver is layered out hierarchical architecture sorted into functional groups. The dirver is also designed for thread safe and multi-instance. 
```
└── drivers
│       └── dw1000
│           ├── include
│           │   └── dw1000
│           │       ├── dw1000_dev.h
│           │       ├── dw1000_ftypes.h
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

## Building

1. Download and install Apache Newt.

2. Download and install Apache Newt.

You will need to download the Apache Newt tool, as documented in the [Getting Started Guide](http://mynewt.apache.org/os/get_started/introduction/).

3. Download the Apache Mynewt Core package (executed from the blinky directory).

```no-highlight
    $ newt install
```

4. Build the Single Side Two Way Ranging (ss_twr) Applicaitons for the DWM1000 hardware platform using the "dwm1001" target
(executed from the mynewt-dw1000 directory).

```no-highlight
    $ newt build ss_twr_master
```

The Apache Newt tool should indicate the location of the generated blinky
executable.  Since the simulator does not have an LED to blink, this version of
blinky is not terribly exciting - a printed message indicating the current LED
state.  To learn how to build blinky for actual hardware, please see the
[Getting Started Guide](http://mynewt.apache.org/os/get_started/introduction/).
