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

# Decawave DW1000 driver 

## Overview

This repo contains the device driver model for the DW1000 for the mynewt OS. The  
diver support, hardware abstraction layers, low-level MAC, Ranging Services and a 
rawlwip stack. 

## Project Status
This project is destined to be streamed into the mynewt repo early Q1 2018. The example contained herein will
become part of the mynewt distribution and maintained within that distribution.

DW1000 Device Driver (complete)
DWM1001 Board Support Package (complete)
DW1000 Ranging Services ss_twr (complete)
DW1000 Ranging Services ds_twr (under development)
CLI node_cfg (Pending)
CLI autosurvey (Pending)
CLI anchor (Pending)
CLI mlat (pending)

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
