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

#include "defs/error.h"
#include "os/os.h"
#include "sysinit/sysinit.h"
#include "hal/hal_i2c.h"
#include "sensor/sensor.h"
#include "sensor/mag.h"
#include "lis2mdl/lis2mdl.h"
#include "lis2mdl_priv.h"
#include "log/log.h"
#include "stats/stats.h"

/* Define the stats section and records */
STATS_SECT_START(lis2mdl_stat_section)
    STATS_SECT_ENTRY(read_errors)
    STATS_SECT_ENTRY(write_errors)
STATS_SECT_END

/* Define stat names for querying */
STATS_NAME_START(lis2mdl_stat_section)
    STATS_NAME(lis2mdl_stat_section, read_errors)
    STATS_NAME(lis2mdl_stat_section, write_errors)
STATS_NAME_END(lis2mdl_stat_section)

/* Global variable used to hold stats data */
STATS_SECT_DECL(lis2mdl_stat_section) g_lis2mdlstats;

#define LOG_MODULE_LIS2MDL    (2000)
#define LIS2MDL_INFO(...)     LOG_INFO(&_log, LOG_MODULE_LIS2MDL, __VA_ARGS__)
#define LIS2MDL_ERR(...)      LOG_ERROR(&_log, LOG_MODULE_LIS2MDL, __VA_ARGS__)
static struct log _log;

/* Exports for the sensor API */
static int lis2mdl_sensor_read(struct sensor *, sensor_type_t,
        sensor_data_func_t, void *, uint32_t);
static int lis2mdl_sensor_get_config(struct sensor *, sensor_type_t,
        struct sensor_cfg *);

static const struct sensor_driver g_lis2mdl_sensor_driver = {
    lis2mdl_sensor_read,
    lis2mdl_sensor_get_config
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
lis2mdl_write8(struct sensor_itf *itf, uint8_t reg, uint32_t value)
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
        LIS2MDL_ERR("Failed to write to 0x%02X:0x%02X with value 0x%02X\n",
                       itf->si_addr, reg, value);
        STATS_INC(g_lis2mdlstats, read_errors);
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
lis2mdl_read8(struct sensor_itf *itf, uint8_t reg, uint8_t *value)
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
        LIS2MDL_ERR("I2C access failed at address 0x%02X\n", itf->si_addr);
        STATS_INC(g_lis2mdlstats, write_errors);
        return rc;
    }

    /* Read one byte back */
    data_struct.buffer = value;
    rc = hal_i2c_master_read(itf->si_num, &data_struct,
                             OS_TICKS_PER_SEC / 10, 1);

    if (rc) {
         LIS2MDL_ERR("Failed to read from 0x%02X:0x%02X\n", itf->si_addr, reg);
         STATS_INC(g_lis2mdlstats, read_errors);
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
lis2mdl_read_bytes(struct sensor_itf *itf, uint8_t reg, uint8_t *buffer, uint32_t length)
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
        LIS2MDL_ERR("I2C access failed at address 0x%02X\n", itf->si_addr);
        STATS_INC(g_lis2mdlstats, write_errors);
        return rc;
    }

    /* Read n bytes back */
    data_struct.len = length;
    data_struct.buffer = buffer;
    rc = hal_i2c_master_read(itf->si_num, &data_struct,
                             OS_TICKS_PER_SEC / 10, 1);

    if (rc) {
         LIS2MDL_ERR("Failed to read from 0x%02X:0x%02X\n", itf->si_addr, reg);
         STATS_INC(g_lis2mdlstats, read_errors);
    }
    return rc;
}

int
lis2mdl_reset(struct sensor_itf *itf)
{
    int rc;
    uint8_t reg;
    lis2mdl_read8(itf, LIS2MDL_CFG_REG_A, &reg);
    // Reset
    rc = lis2mdl_write8(itf, LIS2MDL_CFG_REG_A, reg | 0x20);
    if (rc) {
        return rc;
    }
    os_cputime_delay_usecs(1000);
    // Boot
    rc = lis2mdl_write8(itf, LIS2MDL_CFG_REG_A, reg | 0x40);
    return rc;
}

int
lis2mdl_sleep(struct sensor_itf *itf)
{
    uint8_t reg;
    lis2mdl_read8(itf, LIS2MDL_CFG_REG_A, &reg);
    // Reset
    return lis2mdl_write8(itf, LIS2MDL_CFG_REG_A, reg | 0x20);
}

int
lis2mdl_set_output_rate(struct sensor_itf *itf, enum lis2mdl_output_rate rate)
{
    int rc;
    uint8_t reg;
    // 0x80 - temperature compensation, continuous mode (bits 0:1 == 00)
    rc = lis2mdl_read8(itf, LIS2MDL_CFG_REG_A, &reg);
    if (rc) {
        return rc;
    }

    reg = (reg & (~0xC)) | ((uint8_t)rate << 2);
    return lis2mdl_write8(itf, LIS2MDL_CFG_REG_A, reg);
}

int
lis2mdl_get_output_rate(struct sensor_itf *itf, enum lis2mdl_output_rate *rate)
{
    int rc;
    uint8_t reg;

    rc = lis2mdl_read8(itf, LIS2MDL_CFG_REG_A, &reg);
    if (rc) {
        return rc;
    }

    *rate  = (enum lis2mdl_output_rate)((reg>>2)  & 0x03);

    return 0;
}


int
lis2mdl_enable_interrupt(struct sensor_itf *itf, uint8_t enable)
{
    int rc;
    uint8_t reg;
    // 0x80 - temperature compensation, continuous mode (bits 0:1 == 00)
    rc = lis2mdl_read8(itf, LIS2MDL_CFG_REG_C, &reg);
    if (rc) {
        return rc;
    }

    if (enable)
        reg |= 0x01;
    else
        reg &= ~(0x01);
    
    return lis2mdl_write8(itf, LIS2MDL_CFG_REG_A, reg);
}


int
lis2mdl_set_lpf(struct sensor_itf *itf, uint8_t enable)
{
    int rc;
    uint8_t reg;

    rc = lis2mdl_read8(itf, LIS2MDL_CFG_REG_B, &reg);
    if (rc) {
        return rc;
    }

    if (enable)
        reg |= 0x01;
    else
        reg &= ~(0x01);
    
    return lis2mdl_write8(itf, LIS2MDL_CFG_REG_B, reg);
}

int
lis2mdl_get_lpf(struct sensor_itf *itf, uint8_t *cfg)
{
    return lis2mdl_read8(itf, LIS2MDL_CFG_REG_B, cfg);
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
lis2mdl_init(struct os_dev *dev, void *arg)
{
    struct lis2mdl *lis;
    struct sensor *sensor;
    int rc;

    if (!arg || !dev) {
        return SYS_ENODEV;
    }
    
    lis = (struct lis2mdl *) dev;

    lis->cfg.mask = SENSOR_TYPE_ALL;

    log_register(dev->od_name, &_log, &log_console_handler, NULL, LOG_SYSLEVEL);

    sensor = &lis->sensor;

    /* Initialise the stats entry */
    rc = stats_init(
        STATS_HDR(g_lis2mdlstats),
        STATS_SIZE_INIT_PARMS(g_lis2mdlstats, STATS_SIZE_32),
        STATS_NAME_INIT_PARMS(lis2mdl_stat_section));
    SYSINIT_PANIC_ASSERT(rc == 0);
    /* Register the entry with the stats registry */
    rc = stats_register(dev->od_name, STATS_HDR(g_lis2mdlstats));
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = sensor_init(sensor, dev);
    if (rc) {
        return rc;
    }

    /* Add the accelerometer/gyroscope driver */
    rc = sensor_set_driver(sensor, SENSOR_TYPE_MAGNETIC_FIELD,
         (struct sensor_driver *) &g_lis2mdl_sensor_driver);
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
lis2mdl_config(struct lis2mdl *lis, struct lis2mdl_cfg *cfg)
{
    int rc;
    struct sensor_itf *itf;

    itf = SENSOR_GET_ITF(&(lis->sensor));

    uint8_t val;
    rc = lis2mdl_read8(itf, LIS2MDL_WHO_AM_I, &val);
    if (rc) {
        return rc;
    }
    if (val != LIS2MDL_WHO_AM_I_VAL) {
        return SYS_EINVAL;
    }

    rc = lis2mdl_set_output_rate(itf, cfg->output_rate);
    if (rc) {
        return rc;
    }
    lis->cfg.output_rate = lis->cfg.output_rate;

    rc = lis2mdl_set_lpf(itf, cfg->lpf_enable);
    if (rc) {
        return rc;
    }
    lis->cfg.lpf_enable = lis->cfg.lpf_enable;
	
	// enable block data read (bit 4 == 1)
    rc = lis2mdl_write8(itf, LIS2MDL_CFG_REG_C, 0x10);
    if (rc) {
        return rc;
    }
    
    rc = lis2mdl_enable_interrupt(itf, cfg->int_enable);
    if (rc) {
        return rc;
    }
    lis->cfg.int_enable = lis->cfg.int_enable;
        
    rc = sensor_set_type_mask(&(lis->sensor), cfg->mask);
    if (rc) {
        return rc;
    }

    lis->cfg.mask = cfg->mask;

    return 0;
}

static int
lis2mdl_sensor_read(struct sensor *sensor, sensor_type_t type,
        sensor_data_func_t data_func, void *data_arg, uint32_t timeout)
{
    (void)timeout;
    int rc;
    int16_t x, y, z;
    uint8_t payload[8];
    struct sensor_itf *itf;
    union {
        struct sensor_mag_data smd;
    } databuf;

    /* If the read isn't looking for accel or gyro, don't do anything. */
    if (!(type & SENSOR_TYPE_MAGNETIC_FIELD)) {
        return SYS_EINVAL;
    }

    itf = SENSOR_GET_ITF(sensor);

    /* Get a new accelerometer sample */
    if (type & SENSOR_TYPE_MAGNETIC_FIELD) {
        rc = lis2mdl_read_bytes(itf, LIS2MDL_OUTX_L_REG, payload, 8);
        if (rc) {
            return rc;
        }
        x = (((int16_t)payload[1] << 8) | payload[0]);
        y = (((int16_t)payload[3] << 8) | payload[2]);
        z = (((int16_t)payload[5] << 8) | payload[4]);

        /* Data is already in mG (same as uT) */ 
        databuf.smd.smd_x = x;
        databuf.smd.smd_x_is_valid = 1;
        databuf.smd.smd_y = y;
        databuf.smd.smd_y_is_valid = 1;
        databuf.smd.smd_z = z;
        databuf.smd.smd_z_is_valid = 1;

        rc = data_func(sensor, data_arg, &databuf.smd,
             SENSOR_TYPE_MAGNETIC_FIELD);
        if (rc) {
            return rc;
        }
    }

    return 0;
}

static int
lis2mdl_sensor_get_config(struct sensor *sensor, sensor_type_t type,
        struct sensor_cfg *cfg)
{
    /* If the read isn't looking for accel or gyro, don't do anything. */
    if (!(type & SENSOR_TYPE_MAGNETIC_FIELD)) {
        return SYS_EINVAL;
    }

    cfg->sc_valtype = SENSOR_VALUE_TYPE_FLOAT_TRIPLET;

    return 0;
}
