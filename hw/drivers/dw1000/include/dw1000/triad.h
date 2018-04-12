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
 * @file triad.h
 * @author Paul Kettle
 * @date March 27 2017
 * @brief latitude, longitude, elevation tryad
 *
 */

#ifndef _TRIAD_H_
#define _TRIAD_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief tryad definition
 */

#ifdef TRIAD_INT16
typedef union __triad_t{
    struct _int16_axis{
        int16_t x,y,z;
    };
    struct _int16_spherical{
        int16_t range, azimuth, zenith;
    };
    int16_t array[sizeof(struct _int16_axis)/sizeof(int16_t)];
}int16_triad_t; 
#else
typedef union _triad_t{
    struct _axis{
        float  x,y,z;
    };
    struct _spherical{
       float range, azimuth, zenith;
    };
    float array[sizeof(struct _axis)/sizeof(float)];
}triad_t; 
#endif

#ifdef __cplusplus
}
#endif
#endif