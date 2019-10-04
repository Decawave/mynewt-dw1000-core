/*
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
#include <assert.h>

#include "os/mynewt.h"
#include <bsp/bsp.h>
#include "os/os_dev.h"
#include <mcu/cmsis_nvic.h>
#include <flash_map/flash_map.h>
#include <stm32f429xx.h>
#include <stm32f4xx_hal_gpio_ex.h>

#if MYNEWT_VAL(UART_0)
#include <uart/uart.h>
#include <uart_hal/uart_hal.h>
#endif

#include <hal/hal_bsp.h>
#include <hal/hal_gpio.h>
#include <hal/hal_flash.h>
#include <hal/hal_flash_int.h>
#include <hal/hal_timer.h>
#include <hal/hal_system.h>
#include <hal/hal_spi.h>

#include <mcu/stm32f4_bsp.h>

#if MYNEWT_VAL(DW1000_DEVICE_0)
#include "dw1000/dw1000_dev.h"
#include "dw1000/dw1000_hal.h"
#endif
#if MYNEWT_VAL(ETH_0)
#include "stm32_eth/stm32_eth.h"
#include "stm32_eth/stm32_eth_cfg.h"
#endif

const uint32_t stm32_flash_sectors[] = {
    /* Bank 1 */
    0x08000000,     /* 16kB */
    0x08004000,     /* 16kB */
    0x08008000,     /* 16kB */
    0x0800c000,     /* 16kB */
    0x08010000,     /* 64kB */
    0x08020000,     /* 128kB */
    0x08040000,     /* 128kB */
    0x08060000,     /* 128kB */
    0x08080000,     /* 128kB */
    0x080a0000,     /* 128kB */
    0x080c0000,     /* 128kB */
    0x080e0000,     /* 128kB */
    /* Bank 2 */
    0x08100000,     /* 16kB */
    0x08104000,     /* 16kB */
    0x08108000,     /* 16kB */
    0x0810c000,     /* 16kB */
    0x08110000,     /* 64kB */
    0x08120000,     /* 128kB */
    0x08140000,     /* 128kB */
    0x08160000,     /* 128kB */
    0x08180000,     /* 128kB */
    0x081a0000,     /* 128kB */
    0x081c0000,     /* 128kB */
    0x081e0000,     /* 128kB */
    0x08200000,     /* End of flash */
};

#define SZ (sizeof(stm32_flash_sectors) / sizeof(stm32_flash_sectors[0]))
_Static_assert(MYNEWT_VAL(STM32_FLASH_NUM_AREAS) == SZ,
        "STM32_FLASH_NUM_AREAS does not match flash sectors");

#if MYNEWT_VAL(UART_0)
static struct uart_dev hal_uart0;

/* UART connected to J8 */
static const struct stm32_uart_cfg os_bsp_uart_cfg[UART_CNT] = {
    [0] = {
        .suc_uart = USART3,
        .suc_rcc_reg = &RCC->APB1ENR,
        .suc_rcc_dev = RCC_APB1ENR_USART3EN,
        .suc_pin_tx = MCU_GPIO_PORTB(10),
        .suc_pin_rx = MCU_GPIO_PORTB(11),
        .suc_pin_rts = -1,
        .suc_pin_cts = -1,
        .suc_pin_af = GPIO_AF7_USART3,
        .suc_irqn = USART3_IRQn
    }
};
#endif

struct stm32_hal_spi_cfg {
    int ss_pin;                     /* for slave mode */
    int sck_pin;
    int miso_pin;
    int mosi_pin;
    int irq_prio;
};

#if MYNEWT_VAL(SPI_0_MASTER)
struct stm32_hal_spi_cfg os_bsp_spi0m_cfg = {
    .ss_pin   = MCU_GPIO_PORTD(14),
    .sck_pin  = MCU_GPIO_PORTA(5),
    .miso_pin = MCU_GPIO_PORTA(6),
    .mosi_pin = MCU_GPIO_PORTB(5),
    .irq_prio = 2,
};
struct dpl_sem g_spi0_sem;
#endif

#if MYNEWT_VAL(ETH_0)
static const struct stm32_eth_cfg eth_cfg = {
    /*
     * PORTA
     *   PA1  - ETH_RMII_REF_CLK
     *   PA2  - ETH_RMII_MDIO
     *   PE15 - ETH_RMII_MDINT  (GPIO irq?)
     *   PA7  - ETH_RMII_CRS_DV
     */
    .sec_port_mask[0] = (1 << 1) | (1 << 2) | (1 << 7),

    /*
     * PORTC
     *   PC1 - ETH_RMII_MDC
     *   PC4 - ETH_RMII_RXD0
     *   PC5 - ETH_RMII_RXD1
     */
    .sec_port_mask[2] = (1 << 1) | (1 << 4) | (1 << 5),

    /*
     * PORTG
     *   PG11 - ETH_RMII_TXEN
     *   PG13 - ETH_RMII_TXD0
     */
    .sec_port_mask[6] = (1 << 11) | (1 << 13) ,
    /*
     *  PORT B
     *   PB13 - ETH_RMII_TXD1
     */
    .sec_port_mask[1] = (1 << 13),
    .sec_phy_type = LAN_8742_RMII,
    //.sec_phy_irq = MCU_GPIO_PORTE(15)
    .sec_phy_irq = -1
};
#endif


static const struct hal_bsp_mem_dump dump_cfg[] = {
    [0] = {
        .hbmd_start = &_ram_start,
        .hbmd_size = RAM_SIZE
    },
    [1] = {
        .hbmd_start = &_ccram_start,
        .hbmd_size = CCRAM_SIZE
    }
};

extern const struct hal_flash stm32_flash_dev;
const struct hal_flash *
hal_bsp_flash_dev(uint8_t id)
{
    /*
     * Internal flash mapped to id 0.
     */
    if (id != 0) {
        return NULL;
    }
    return &stm32_flash_dev;
}

const struct hal_bsp_mem_dump *
hal_bsp_core_dump(int *area_cnt)
{
    *area_cnt = sizeof(dump_cfg) / sizeof(dump_cfg[0]);
    return dump_cfg;
}

#if MYNEWT_VAL(DW1000_DEVICE_0)
/*
 * dw1000 device structure defined in dw1000_hal.c
 */
static struct _dw1000_dev_instance_t * dw1000_0 = 0;
static const struct dw1000_dev_cfg dw1000_0_cfg = {
    .spi_sem = &g_spi0_sem,
    .spi_num = 0,
};
#endif


void SystemClock_Config(void);
void SystemClockHSI_Config(void);
void
hal_bsp_init(void)
{
    int rc;

    (void)rc;

#if MYNEWT_VAL(TIMER_0)
    hal_timer_init(0, TIM9);
#endif

#if MYNEWT_VAL(TIMER_1)
    hal_timer_init(1, TIM10);
#endif

#if MYNEWT_VAL(TIMER_2)
    hal_timer_init(2, TIM11);
#endif

#if (MYNEWT_VAL(OS_CPUTIME_TIMER_NUM) >= 0)
    rc = os_cputime_init(MYNEWT_VAL(OS_CPUTIME_FREQ));
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_0_MASTER)
    rc = hal_spi_init(0, (void *)&os_bsp_spi0m_cfg, HAL_SPI_TYPE_MASTER);
    assert(rc == 0);
    dpl_error_t err = dpl_sem_init(&g_spi0_sem, 0x1);
    assert(err == DPL_OK);
#endif

#if MYNEWT_VAL(DW1000_DEVICE_0)
    dw1000_0 = hal_dw1000_inst(0);
    rc = os_dev_create((struct os_dev *) dw1000_0, "dw1000_0",
            OS_DEV_INIT_PRIMARY, 0, dw1000_dev_init, (void *)&dw1000_0_cfg);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(UART_0)
    rc = os_dev_create((struct os_dev *) &hal_uart0, "uart0",
      OS_DEV_INIT_PRIMARY, 0, uart_hal_init, (void *)&os_bsp_uart_cfg[0]);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(ETH_0)
    stm32_eth_init(&eth_cfg);
#endif

}

/**
 * Returns the configured priority for the given interrupt. If no priority
 * configured, return the priority passed in
 *
 * @param irq_num
 * @param pri
 *
 * @return uint32_t
 */
uint32_t
hal_bsp_get_nvic_priority(int irq_num, uint32_t pri)
{
    /* Add any interrupt priorities configured by the bsp here */
    return pri;
}
