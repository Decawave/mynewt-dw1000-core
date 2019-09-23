/**
 * Copyright 2018, Decawave Limited, All Rights Reserved
 * 
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

/**
 * @file cir.c
 * @author Paul.Kettle@decawave.com
 * @date Nov 29 2019
 * @brief Channel Impulse Response
 *
 * @details 
 *
 */

#include <stdio.h>
#include <math.h>
#include <os/os.h>
#include <cir/cir.h>


/*! 
 * @fn cir_calc_aoa(float pdoa, float wavelength, float antenna_separation)
 *
 * @brief Calculate the phase difference between receivers
 * 
 * @param pdoa       - phase difference of arrival in radians
 * @param wavelength - wavelength of the radio channel used in meters
 * @param antenna_separation - distance between centers of antennas in meters
 * 
 * output parameters
 *
 * returns angle of arrival - float, in radians
 */
float
cir_calc_aoa(float pdoa, float wavelength, float antenna_separation)
{
    float pd_dist = pdoa / (2.0f*M_PI) * wavelength;
    return asinf(pd_dist / antenna_separation);
}
