/*
 * Copyright (C) 2017-2018, Decawave Limited, All Rights Reserved
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
 * @file slots.c
 * @author paul.kettle@decawave.com
 * @date 12/2018
 * @brief node-slot utils 
 *
 * @details Help function to calculate the numerical ordering of a bit within a bitmask
 * addresses.
 */

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <dw1000/dw1000_dev.h>
#include <rng/slots.h>

/**
 * Help function to calculate the number of active slots within a bitmask
 *
 * @param n bitfield to count bits within
 * @return number of set bits
 */
uint32_t NumberOfBits(uint32_t n) {
    uint32_t count = 0;
    while (n) {
        n &= (n-1);
        count++;
    }
    return count;
}

/**
 * Help function to calculate the position of active slots within a bitmask
 *
 * @param n bitfield to count bits within
 * @return number of set bits
 */
static uint32_t BitPosition(uint32_t n) {
    
    uint32_t count = 0;
    assert(n && (! (n & (n-1)) )); // single bit set

    while (n){
        n = n >> 1;
        ++count; // position of bit within bitfield
    }
    return count;
}


/**
 * Help function to calculate the numerical ordering of a bit within a bitmask
 *
 * @param n bitfield
 * @param mask
 * @return numerical ordering of a bit witin bitmask.
 */
uint32_t BitIndex(uint32_t nslots_mask, uint32_t n, uint8_t mode) {

    assert(n && (! (n & (n-1)) )); // single bit set
    assert(n & nslots_mask); // bit set is within ROI
    
    uint32_t idx = BitPosition(n);
    uint32_t slot_mask =  (((uint32_t)~0UL >> (sizeof(uint32_t) * 8 - idx)));
    uint32_t remaining_mask = ((uint32_t)~0UL << idx);
        
    if (mode == SLOT_POSITION)
        return NumberOfBits(nslots_mask & slot_mask) - 1; // slot position
    else
        return NumberOfBits(nslots_mask & remaining_mask) - 1; // no. of slots remaining
}



