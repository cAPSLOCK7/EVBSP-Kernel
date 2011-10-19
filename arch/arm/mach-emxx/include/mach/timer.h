/*
 *  File Name       : arch/arm/mach-emxx/include/mach/timer.h
 *  Function	    : timer
 *  Release Version : Ver 1.00
 *  Release Date    : 2010/02/05
 *
 * Copyright (C) 2010 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA.
 *
 */

#ifndef __ASM_ARCH_TIMER_H
#define __ASM_ARCH_TIMER_H

#define	TIMER_TG0		0
#define	TIMER_TG1		1
#define	TIMER_TG2		2
#define	TIMER_TG3		3
#define	TIMER_TG4		4
#define	TIMER_TG5		5
#define	TIMER_TI0		6
#define	TIMER_TI1		7
#define	TIMER_TI2		8
#define	TIMER_TI3		9
#define	TIMER_TW0		10
#define	TIMER_TW1		11
#define	TIMER_TW2		12
#define	TIMER_TW3		13
#define	TIMER_TW4		14

#define TIMER_MAX_NUM		4
#define TIMER_TG_MAX_NUM	6


#define TIMER_MIN_USECS		1
#define TIMER_MAX_USECS		748982856
#define TIMER_MAX_COUNT         0xffffffff
#define TIMER_DEFAULT_TIME	TIMER_MAX_USECS

#ifdef CONFIG_EMXX_PLL3_238MHZ
#define TIMER_CLOCK_TICK_RATE_PLL3	5939200
#else
#define TIMER_CLOCK_TICK_RATE_PLL3	5734400
#endif

#define TIMER_CLOCK_TICK_RATE_32K	32000

#define TW_MIN_USECS		157
#define TW_MAX_USECS		4294967282U
#define TW_DEFAULT_TIME		32000000	/* 32sec */
#define TW_CLOCK_TICK_RATE	32000


#include <mach/timex.h>

#include <linux/interrupt.h>

extern irqreturn_t(*timer_cb_t(int, void *));

extern int emxx_timer_start(unsigned int timer);
extern int emxx_timer_stop(unsigned int timer);
extern int emxx_timer_set_period(unsigned int timer, unsigned int usecs);
extern int emxx_timer_get_period(unsigned int timer, unsigned int *usecs);
extern int emxx_timer_get_usecs(unsigned int timer, unsigned int *usecs);
extern int emxx_timer_get_counts(unsigned int timer, unsigned int *counts);
extern int emxx_timer_register_cb(unsigned int timer, char *devname,
				irqreturn_t(*cb) (int, void *), void *dev_id);
extern int emxx_timer_unregister_cb(unsigned int timer, void *dev_id);
extern int emxx_wdt_set_timeout(unsigned int usecs);
extern void emxx_wdt_ping(void);
extern void emxx_wdt_enable(void);
extern void emxx_wdt_disable(void);
extern void emxx_wdt_setup(void);
#endif	/* __ASM_ARCH_TIMER_H */
