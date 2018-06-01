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

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <os/os.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_gpio.h>


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_phy_config_leds()
 *
 * @brief This is used to set up Tx/Rx GPIOs which could be used to control LEDs
 * Note: not completely IC dependent, also needs board with LEDS fitted on right I/O lines
 *       this function enables GPIOs 2 and 3 which are connected to LED3 and LED4 on EVB1000
 *
 * input parameters
 * @param mode - this is a bit field interpreted as follows:
 *          - bit 0: 1 to enable LEDs, 0 to disable them
 *          - bit 1: 1 to make LEDs blink once on init. Only valid if bit 0 is set (enable LEDs)
 *          - bit 2 to 7: reserved
 *
 * output parameters none
 *
 * no return value
 */
void dw1000_gpio_config_leds(dw1000_dev_instance_t * inst, dw1000_led_modes_t mode)
{
    uint32_t reg;

    if (mode & DWT_LEDS_ENABLE){
        // Set up MFIO for LED output.
        reg = (uint32_t) dw1000_read_reg(inst, GPIO_CTRL_ID, GPIO_MODE_OFFSET, sizeof(uint32_t));
        reg &= ~(GPIO_MSGP2_MASK | GPIO_MSGP3_MASK);
        reg |= (GPIO_PIN2_RXLED | GPIO_PIN3_TXLED);
        dw1000_write_reg(inst, GPIO_CTRL_ID, GPIO_MODE_OFFSET, reg, sizeof(uint32_t));

        // Enable LP Oscillator to run from counter and turn on de-bounce clock.
        reg = (uint32_t) dw1000_read_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, sizeof(uint32_t));
        reg |= (PMSC_CTRL0_GPDCE | PMSC_CTRL0_KHZCLEN);
        dw1000_write_reg(inst, PMSC_ID, PMSC_CTRL0_OFFSET, reg, sizeof(uint32_t));
        dw1000_write_reg(inst, PMSC_ID, PMSC_LEDC_OFFSET, PMSC_LEDC_BLNKEN | PMSC_LEDC_BLINK_TIME_DEF, sizeof(uint32_t));

        if (mode & DWT_LEDS_INIT_BLINK){
            // Single blink sign-of-life.
            reg = (uint32_t) dw1000_read_reg(inst, PMSC_ID, PMSC_LEDC_OFFSET, sizeof(uint32_t));
            reg |= PMSC_LEDC_BLINK_NOW_ALL;

            dw1000_write_reg(inst, PMSC_ID, PMSC_LEDC_OFFSET, reg, sizeof(uint32_t));
            reg &= ~PMSC_LEDC_BLINK_NOW_ALL;
            os_cputime_delay_usecs(10);
            dw1000_write_reg(inst, PMSC_ID, PMSC_LEDC_OFFSET, reg, sizeof(uint32_t));
        }
    }else{
        // Clear the GPIO bits that are used for LED control.
        reg = dw1000_read_reg(inst,GPIO_CTRL_ID, GPIO_MODE_OFFSET,sizeof(uint32_t));
        reg &= ~(GPIO_MSGP2_MASK | GPIO_MSGP3_MASK);
        dw1000_write_reg(inst, GPIO_CTRL_ID, GPIO_MODE_OFFSET, reg, sizeof(uint32_t));
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_gpio_direction()
 *
 * @brief This is used to set GPIO direction as an input (1) or output (0)
 *
 * input parameters
 * @param gpioNum    -   this is the GPIO to configure - see GxM0... GxM8 in the deca_regs.h file
 * @param direction  -   this sets the GPIO direction - see GxP0... GxP8 in the deca_regs.h file
 *
 * output parameters
 *
 * no return value
 */
void dw1000_gpio_direction(dw1000_dev_instance_t * inst, uint32_t gpioNum, uint32_t direction)
{
    uint8_t buf[GPIO_DIR_LEN];
    uint32_t command = direction | gpioNum;

    buf[0] = command & 0xff;
    buf[1] = (command >> 8) & 0xff;
    buf[2] = (command >> 16) & 0xff;

    dw1000_write(inst, GPIO_CTRL_ID, GPIO_DIR_OFFSET, buf, GPIO_DIR_LEN);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_gpio_set()
 *
 * @brief This is used to set GPIO value as (1) or (0) only applies if the GPIO is configured as output
 *
 * input parameters
 * @param gpioNum    -   this is the GPIO to configure - see GxM0... GxM8 in the deca_regs.h file
 * @param value  -   this sets the GPIO value - see GDP0... GDP8 in the deca_regs.h file
 *
 * output parameters
 *
 * no return value
 */
void dw1000_gpio_set(dw1000_dev_instance_t * inst, uint32_t gpioNum, uint32_t value)
{
    uint8_t buf[GPIO_DOUT_LEN];
    uint32_t command = value | gpioNum;

    buf[0] = command & 0xff;
    buf[1] = (command >> 8) & 0xff;
    buf[2] = (command >> 16) & 0xff;

    dw1000_write(inst, GPIO_CTRL_ID, GPIO_DOUT_OFFSET, buf, GPIO_DOUT_LEN);
}

