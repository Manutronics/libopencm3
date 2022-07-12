/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2012 Silvio Gissi <silvio.gissi@outlook.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LPC17XX_MEMORYMAP_H
#define LPC17XX_MEMORYMAP_H

#include <libopencm3/cm3/memorymap.h>

/* --- LPC17XX specific peripheral definitions ----------------------------- */

/* Memory map for all busses */
#define PERIPH_BASE_GPIO		(0x2009C000U)
#define PERIPH_BASE_APB0		(0x40000000U)
#define PERIPH_BASE_APB1		(0x40080000U)
#define PERIPH_BASE_AHB			(0x50000000U)

/* Register boundary addresses */

/* GPIO */
#define GPIO_PIO0_BASE			(PERIPH_BASE_GPIO + 0x00)
#define GPIO_PIO1_BASE			(PERIPH_BASE_GPIO + 0x20)
#define GPIO_PIO2_BASE			(PERIPH_BASE_GPIO + 0x40)
#define GPIO_PIO3_BASE			(PERIPH_BASE_GPIO + 0x60)
#define GPIO_PIO4_BASE			(PERIPH_BASE_GPIO + 0x80)

/* APB0 */
#define WDT_BASE			(PERIPH_BASE_APB0 + 0x00000)
#define TIMER0_BASE			(PERIPH_BASE_APB0 + 0x04000)
#define TIMER1_BASE			(PERIPH_BASE_APB0 + 0x08000)
#define UART0_BASE			(PERIPH_BASE_APB0 + 0x0c000)
#define UART1_BASE			(PERIPH_BASE_APB0 + 0x10000)
/* PERIPH_BASE_APB0 + 0X14000 (0x4001 4000 - 0x4001 7FFF): Reserved */
#define PWM1_BASE			(PERIPH_BASE_APB0 + 0x18000)
#define I2C0_BASE			(PERIPH_BASE_APB0 + 0x1c000)
#define SPI_BASE			(PERIPH_BASE_APB0 + 0x20000)
#define RTC_BASE			(PERIPH_BASE_APB0 + 0x24000)
#define GPIOINTERRUPT_BASE		(PERIPH_BASE_APB0 + 0x28000)
#define PINCONNECT_BASE			(PERIPH_BASE_APB0 + 0x2c000)
#define SSP1_BASE			(PERIPH_BASE_APB0 + 0x30000)
#define ADC_BASE			(PERIPH_BASE_APB0 + 0x34000)
#define CANAFRAM_BASE			(PERIPH_BASE_APB0 + 0x38000)
#define CANAFREG_BASE			(PERIPH_BASE_APB0 + 0x3C000)
#define CANCOMMONREG_BASE		(PERIPH_BASE_APB0 + 0x40000)
#define CAN1_BASE			(PERIPH_BASE_APB0 + 0x44000)
#define CAN2_BASE			(PERIPH_BASE_APB0 + 0x48000)
/* PERIPH_BASE_APB0 + 0X4C000 (0x4004 C000 - 0x4005 BFFF): Reserved */
#define I2C1_BASE			(PERIPH_BASE_APB0 + 0x5C000)
/* PERIPH_BASE_APB0 + 0X60000 (0x4006 0000 - 0x4007 BFFF): Reserved */

/* APB1 */
/* PERIPH_BASE_APB1 + 0X00000 (0x4008 0000 - 0x4008 7FFF): Reserved */
#define SSP0_BASE			(PERIPH_BASE_APB1 + 0x08000)
#define DAC_BASE			(PERIPH_BASE_APB1 + 0x0c000)
#define TIMER2_BASE			(PERIPH_BASE_APB1 + 0x10000)
#define TIMER3_BASE			(PERIPH_BASE_APB1 + 0x14000)
#define UART2_BASE			(PERIPH_BASE_APB1 + 0x18000)
#define UART3_BASE			(PERIPH_BASE_APB1 + 0x1c000)
#define I2C2_BASE			(PERIPH_BASE_APB1 + 0x20000)
/* PERIPH_BASE_APB1 + 0X24000 (0x400A 4000 - 0x400A 7FFF): Reserved */
#define I2S_BASE			(PERIPH_BASE_APB1 + 0x28000)
/* PERIPH_BASE_APB1 + 0X2C000 (0x400A C000 - 0x400A FFFF): Reserved */
#define RIT_BASE			(PERIPH_BASE_APB1 + 0x30000)
/* PERIPH_BASE_APB1 + 0X34000 (0x400B 4000 - 0x400B 7FFF): Reserved */
#define MCPWM_BASE			(PERIPH_BASE_APB1 + 0x38000)
#define QEI_BASE			(PERIPH_BASE_APB1 + 0x3c000)
/* PERIPH_BASE_APB1 + 0X40000 (0x400C 0000 - 0x400F BFFF): Reserved */
#define SYSCON_BASE			(PERIPH_BASE_APB1 + 0x7c000)

/* AHB */
#define ETHERNET_BASE			(PERIPH_BASE_AHB + 0x00000)
#define GPDMA_BASE			(PERIPH_BASE_AHB + 0x04000)
/* PERIPH_BASE_AHB + 0X08000 (0x5000 8000 - 0x5000 BFFF): Reserved */
#define USB_BASE			(PERIPH_BASE_AHB + 0x0c000)
/* PERIPH_BASE_AHB + 0X10000 (0x5001 0000 - 0x501F FFFF): Reserved */

/* --- Convenience macros -------------------------------------------------- */
#define SYS_PCONP						MMIO32(SYSCON_BASE + 0x0C4)
#define PINCON_PINSEL0					MMIO32(PERIPH_BASE_APB0 + 0x2C000)
#define PINCON_PINSEL1					MMIO32(PERIPH_BASE_APB0 + 0x2C004)
#define PINCON_PINSEL2					MMIO32(PERIPH_BASE_APB0 + 0x2C008)
#define PINCON_PINSEL3					MMIO32(PERIPH_BASE_APB0 + 0x2C00C)
#define PINCON_PINSEL4					MMIO32(PERIPH_BASE_APB0 + 0x2C010)
#define PINCON_PINSEL7					MMIO32(PERIPH_BASE_APB0 + 0x2C01C)
#define PINCON_PINSEL9					MMIO32(PERIPH_BASE_APB0 + 0x2C024)
#define PINCON_PINSEL10					MMIO32(PERIPH_BASE_APB0 + 0x2C028)

#define PINCON_PINMODE0					MMIO32(PERIPH_BASE_APB0 + 0x2C040)
#define PINCON_PINMODE1					MMIO32(PERIPH_BASE_APB0 + 0x2C044)
#define PINCON_PINMODE2					MMIO32(PERIPH_BASE_APB0 + 0x2C048)
#define PINCON_PINMODE3					MMIO32(PERIPH_BASE_APB0 + 0x2C04C)
#define PINCON_PINMODE4					MMIO32(PERIPH_BASE_APB0 + 0x2C050)
#define PINCON_PINMODE7					MMIO32(PERIPH_BASE_APB0 + 0x2C05C)
#define PINCON_PINMODE9					MMIO32(PERIPH_BASE_APB0 + 0x2C064)

#define PINCON_PINMODE_OD0              MMIO32(PERIPH_BASE_APB0 + 0x2C068)
#define PINCON_PINMODE_OD1              MMIO32(PERIPH_BASE_APB0 + 0x2C06C)
#define PINCON_PINMODE_OD2              MMIO32(PERIPH_BASE_APB0 + 0x2C070)
#define PINCON_PINMODE_OD3              MMIO32(PERIPH_BASE_APB0 + 0x2C074)
#define PINCON_PINMODE_OD4              MMIO32(PERIPH_BASE_APB0 + 0x2C078)

#endif
