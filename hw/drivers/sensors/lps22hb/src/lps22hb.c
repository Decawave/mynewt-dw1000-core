/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * resarding copyright ownership.  The ASF licenses this file
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

#include <string.h>
#include <errno.h>
#include <assert.h>
#include <stdio.h>

#include "defs/error.h"
#include "os/os.h"
#include "sysinit/sysinit.h"
#include "hal/hal_i2c.h"
#include "sensor/sensor.h"
#include "sensor/pressure.h"
#include "sensor/temperature.h"
#include "lps22hb/lps22hb.h"
#include "lps22hb_priv.h"
#include "log/log.h"
#include "stats/stats.h"

/* Define the stats section and records */
STATS_SECT_START(lps22hb_stat_section)
    STATS_SECT_ENTRY(read_errors)
    STATS_SECT_ENTRY(write_errors)
STATS_SECT_END

/* Define stat names for querying */
STATS_NAME_START(lps22hb_stat_section)
    STATS_NAME(lps22hb_stat_section, read_errors)
    STATS_NAME(lps22hb_stat_section, write_errors)
STATS_NAME_END(lps22hb_stat_section)

/* Global variable used to hold stats data */
STATS_SECT_DECL(lps22hb_stat_section) g_lps22hbstats;

#define LOG_MODULE_LPS22HB    (2200)
#define LPS22HB_INFO(...)     LOG_INFO(&_log, LOG_MODULE_LPS22HB, __VA_ARGS__)
#define LPS22HB_ERR(...)      LOG_ERROR(&_log, LOG_MODULE_LPS22HB, __VA_ARGS__)
static struct log _log;

/* Exports for the sensor API */
static int lps22hb_sensor_read(struct sensor *, sensor_type_t,
        sensor_data_func_t, void *, uint32_t);
static int lps22hb_sensor_get_config(struct sensor *, sensor_type_t,
        struct sensor_cfg *);

static const struct sensor_driver g_lps22hb_sensor_driver = {
    lps22hb_sensor_read,
    lps22hb_sensor_get_config
};

/**
 * Writes a single byte to the specified register
 *
 * @param The sensor interface
 * @param The register address to write to
 * @param The value to write
 *
 * @return 0 on success, non-zero error on failure.
 */
int
lps22hb_write8(struct sensor_itf *itf, uint8_t reg, uint32_t value)
{
    int rc;
    uint8_t payload[2] = { reg, value & 0xFF };

    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr,
        .len = 2,
        .buffer = payload
    };

    rc = hal_i2c_master_write(itf->si_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 1);

    if (rc) {
        LPS22HB_ERR("Failed to write to 0x%02X:0x%02X with value 0x%02X\n",
                       itf->si_addr, reg, value);
        STATS_INC(g_lps22hbstats, read_errors);
    }

    return rc;
}

/**
 * Reads a single byte from the specified register
 *
 * @param The sensor interface
 * @param The register address to read from
 * @param Pointer to where the register value should be written
 *
 * @return 0 on success, non-zero error on failure.
 */
int
lps22hb_read8(struct sensor_itf *itf, uint8_t reg, uint8_t *value)
{
    int rc;

    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr,
        .len = 1,
        .buffer = &reg
    };

    /* Register write */
    rc = hal_i2c_master_write(itf->si_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 0);
    if (rc) {
        LPS22HB_ERR("I2C access failed at address 0x%02X\n", itf->si_addr);
        STATS_INC(g_lps22hbstats, write_errors);
        return rc;
    }

    /* Read one byte back */
    data_struct.buffer = value;
    rc = hal_i2c_master_read(itf->si_num, &data_struct,
                             OS_TICKS_PER_SEC / 10, 1);

    if (rc) {
         LPS22HB_ERR("Failed to read from 0x%02X:0x%02X\n", itf->si_addr, reg);
         STATS_INC(g_lps22hbstats, read_errors);
    }
    return rc;
}

/**
 * Reads n bytes from the specified register
 *
 * @param The sensor interface
 * @param The register address to read from
 * @param Pointer to where the register value should be written
 * @param number of bytes to read
 *
 * @return 0 on success, non-zero error on failure.
 */
int
lps22hb_read_bytes(struct sensor_itf *itf, uint8_t reg, uint8_t *buffer, uint32_t length)
{
    int rc;

    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr,
        .len = 1,
        .buffer = &reg
    };

    /* Register write */
    rc = hal_i2c_master_write(itf->si_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 0);
    if (rc) {
        LPS22HB_ERR("I2C access failed at address 0x%02X\n", itf->si_addr);
        STATS_INC(g_lps22hbstats, write_errors);
        return rc;
    }

    /* Read n bytes back */
    data_struct.len = length;
    data_struct.buffer = buffer;
    rc = hal_i2c_master_read(itf->si_num, &data_struct,
                             OS_TICKS_PER_SEC / 10, 1);

    if (rc) {
         LPS22HB_ERR("Failed to read from 0x%02X:0x%02X\n", itf->si_addr, reg);
         STATS_INC(g_lps22hbstats, read_errors);
    }
    return rc;
}

int
lps22hb_reset(struct sensor_itf *itf)
{
    int rc;
    uint8_t reg;
    rc = lps22hb_read8(itf, LPS22HB_CTRL_REG2, &reg);
    if (rc) {
        return rc;
    }

    // Reset
    return lps22hb_write8(itf, LPS22HB_CTRL_REG2, reg | LPS22HB_REG2_RESET);
}

int
lps22hb_set_output_rate(struct sensor_itf *itf, enum lps22hb_output_rate rate)
{
    int rc;
    uint8_t reg;
    // 0x80 - temperature compensation, continuous mode (bits 0:1 == 00)
    rc = lps22hb_read8(itf, LPS22HB_CTRL_REG1, &reg);
    if (rc) {
        return rc;
    }

    reg = (reg & 0x0F) | ((uint8_t)rate);
    return lps22hb_write8(itf, LPS22HB_CTRL_REG1, reg);
}

int
lps22hb_get_output_rate(struct sensor_itf *itf, enum lps22hb_output_rate *rate)
{
    int rc;
    uint8_t reg;

    rc = lps22hb_read8(itf, LPS22HB_CTRL_REG1, &reg);
    if (rc) {
        return rc;
    }

    *rate  = (enum lps22hb_output_rate)(reg & 0x70);

    return 0;
}


int
lps22hb_enable_interrupt(struct sensor_itf *itf, uint8_t enable)
{
    int rc;
    uint8_t reg;

    rc = lps22hb_read8(itf, LPS22HB_CTRL_REG3, &reg);
    if (rc) {
        return rc;
    }

    reg = (reg & (~LPS22HB_REG3_DRDY)) | ((enable) ? LPS22HB_REG3_DRDY : 0x00);
    return lps22hb_write8(itf, LPS22HB_CTRL_REG3, reg);
}


int
lps22hb_set_lpf(struct sensor_itf *itf, enum lps22hb_lpf_config cfg)
{
    int rc;
    uint8_t reg;

    rc = lps22hb_read8(itf, LPS22HB_CTRL_REG1, &reg);
    if (rc) {
        return rc;
    }

    reg = (reg & 0x0C) | ((uint8_t)cfg);
    return lps22hb_write8(itf, LPS22HB_CTRL_REG1, reg);
}

int
lps22hb_get_lpf(struct sensor_itf *itf, uint8_t *cfg)
{
    int rc;
    uint8_t reg;

    rc = lps22hb_read8(itf, LPS22HB_CTRL_REG1, &reg);
    if (rc) {
        return rc;
    }

    *cfg  = (enum lps22hb_output_rate)(reg & 0x0C);

    return 0;
}

int
lps22hb_oneshot(struct sensor_itf *itf)
{
    int rc;
    uint8_t reg;
    rc = lps22hb_read8(itf, LPS22HB_CTRL_REG2, &reg);
    if (rc) {
        return rc;
    }
    // Order a single measurement
    return lps22hb_write8(itf, LPS22HB_CTRL_REG2, reg | LPS22HB_REG2_ONESHOT);
}


/**
 * Expects to be called back through os_dev_create().
 *
 * @param The device object associated with this accellerometer
 * @param Argument passed to OS device init, unused
 *
 * @return 0 on success, non-zero error on failure.
 */
int
lps22hb_init(struct os_dev *dev, void *arg)
{
    struct lps22hb *lhb;
    struct sensor *sensor;
    int rc;

    if (!arg || !dev) {
        return SYS_ENODEV;
    }
    
    lhb = (struct lps22hb *) dev;

    lhb->cfg.mask = SENSOR_TYPE_ALL;

    log_register(dev->od_name, &_log, &log_console_handler, NULL, LOG_SYSLEVEL);

    sensor = &lhb->sensor;

    /* Initialise the stats entry */
    rc = stats_init(
        STATS_HDR(g_lps22hbstats),
        STATS_SIZE_INIT_PARMS(g_lps22hbstats, STATS_SIZE_32),
        STATS_NAME_INIT_PARMS(lps22hb_stat_section));
    SYSINIT_PANIC_ASSERT(rc == 0);
    /* Register the entry with the stats registry */
    rc = stats_register(dev->od_name, STATS_HDR(g_lps22hbstats));
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = sensor_init(sensor, dev);
    if (rc) {
        return rc;
    }

    /* Add the accelerometer/gyroscope driver */
    rc = sensor_set_driver(sensor, (SENSOR_TYPE_PRESSURE | SENSOR_TYPE_TEMPERATURE),
         (struct sensor_driver *) &g_lps22hb_sensor_driver);
    if (rc) {
        return rc;
    }

    rc = sensor_set_interface(sensor, arg);
    if (rc) {
        return rc;
    }

    return sensor_mgr_register(sensor);
}

int
lps22hb_config(struct lps22hb *lhb, struct lps22hb_cfg *cfg)
{
    int rc;
    struct sensor_itf *itf;

    itf = SENSOR_GET_ITF(&(lhb->sensor));

    uint8_t val;
    rc = lps22hb_read8(itf, LPS22HB_WHO_AM_I, &val);
    if (rc) {
        return rc;
    }
    if (val != LPS22HB_WHO_AM_I_VAL) {
        return SYS_EINVAL;
    }

    rc = lps22hb_set_output_rate(itf, cfg->output_rate);
    if (rc) {
        return rc;
    }
    lhb->cfg.output_rate = lhb->cfg.output_rate;

    rc = lps22hb_set_lpf(itf, cfg->lpf_cfg);
    if (rc) {
        return rc;
    }
    lhb->cfg.lpf_cfg = lhb->cfg.lpf_cfg;
	
	// enable block data read (bit 1 == 1)
    rc = lps22hb_write8(itf, LPS22HB_CTRL_REG1, 0x02);
    if (rc) {
        return rc;
    }
    
    rc = lps22hb_enable_interrupt(itf, cfg->int_enable);
    if (rc) {
        return rc;
    }
    lhb->cfg.int_enable = lhb->cfg.int_enable;
        
    rc = sensor_set_type_mask(&(lhb->sensor), cfg->mask);
    if (rc) {
        return rc;
    }

    lhb->cfg.mask = cfg->mask;

    return 0;
}

static int
lps22hb_sensor_read(struct sensor *sensor, sensor_type_t type,
        sensor_data_func_t data_func, void *data_arg, uint32_t timeout)
{
    (void)timeout;
    int rc;
    int32_t temperature;
    uint32_t pressure;
    uint8_t payload[3];
    struct sensor_itf *itf;
    struct lps22hb *lhb;
    union {
        struct sensor_press_data spd;
        struct sensor_temp_data std;
    } databuf;
    uint8_t reg;

    /* If the read isn't looking for accel or gyro, don't do anything. */
    if (!(type & (SENSOR_TYPE_PRESSURE | SENSOR_TYPE_TEMPERATURE))) {
        return SYS_EINVAL;
    }

    itf = SENSOR_GET_ITF(sensor);
    lhb = (struct lps22hb *) SENSOR_GET_DEVICE(sensor);

    /* If sensor is in ONE_SHOT mode, activate it now */
    if (lhb->cfg.output_rate == LPS22HB_OUTPUT_RATE_ONESHOT)
    {
        /* Get a new sample */
        lps22hb_oneshot(itf);
        reg = LPS22HB_REG2_ONESHOT;
        while (reg & LPS22HB_REG2_ONESHOT)
        {
            os_cputime_delay_usecs(1000);
            lps22hb_read8(itf, LPS22HB_CTRL_REG2, &reg);
        }
    }

    if (type & SENSOR_TYPE_PRESSURE) {
        if (lhb->cfg.output_rate == LPS22HB_OUTPUT_RATE_ONESHOT)
        {
            rc = lps22hb_read8(itf, LPS22HB_PRESS_OUT_XL, payload);
            rc = lps22hb_read8(itf, LPS22HB_PRESS_OUT_L, payload+1);
            rc = lps22hb_read8(itf, LPS22HB_PRESS_OUT_H, payload+2);
        }
        else
        {
            // bit 7 must be one to read multiple bytes
            rc = lps22hb_read_bytes(itf, (LPS22HB_PRESS_OUT_XL | 0x80), payload, 3);
        }
            
        if (rc) {
            return rc;
        }
        pressure = ((uint32_t)payload[2]<<16) | ((uint32_t)payload[1]<<8) | payload[0];

        /* Data in Pa */ 
        databuf.spd.spd_press = pressure *100/4096.0F;
        databuf.spd.spd_press_is_valid = 1;

        rc = data_func(sensor, data_arg, &databuf.spd,
             SENSOR_TYPE_PRESSURE);
        if (rc) {
            return rc;
        }
    }

    if (type & SENSOR_TYPE_TEMPERATURE) {
        if (lhb->cfg.output_rate == LPS22HB_OUTPUT_RATE_ONESHOT)
        {
            rc = lps22hb_read8(itf, LPS22HB_TEMP_OUT_L, payload);
            rc = lps22hb_read8(itf, LPS22HB_TEMP_OUT_H, payload+1);
        }
        else
        {
            // bit 7 must be one to read multiple bytes
            rc = lps22hb_read_bytes(itf, (LPS22HB_TEMP_OUT_L | 0x80), payload, 2);
        }

        if (rc) {
            return rc;
        }
        temperature = ((int32_t)payload[1]<<8) | (int32_t)payload[0];

        /* Data in C */ 
        databuf.std.std_temp = temperature/100.0F;
        databuf.std.std_temp_is_valid = 1;

        rc = data_func(sensor, data_arg, &databuf.std,
             SENSOR_TYPE_TEMPERATURE);
        if (rc) {
            return rc;
        }
    }
    
    return 0;
}

static int
lps22hb_sensor_get_config(struct sensor *sensor, sensor_type_t type,
        struct sensor_cfg *cfg)
{
    /* If the read isn't looking for accel or gyro, don't do anything. */
    if (!(type & (SENSOR_TYPE_PRESSURE|SENSOR_TYPE_TEMPERATURE))) {
        return SYS_EINVAL;
    }

    cfg->sc_valtype = SENSOR_VALUE_TYPE_FLOAT;

    return 0;
}
