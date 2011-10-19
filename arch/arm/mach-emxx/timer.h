/*
 *  File Name	    : arch/arm/mach-emxx/time.h
 *  Function	    : time
 *  Release Version : Ver 1.02
 *  Release Date    : 2010/09/24
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
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA.
 */

#ifndef __ARCH_ARM_MACH_EMXX_TIME_H
#define __ARCH_ARM_MACH_EMXX_TIME_H

/*
 * General Timer
 */
#define TM_EN			0x00000001
#define TSTART			0x00000002
#define TO_EN			0x00000004
#define TCR_CLR			0x00000002
#define TSTOP			0x00000000

/*
 * Timer clock condition
 */
#define TIMER_SUSPEND		0
#define TIMER_RESUME		1
#define TIMER_FULLSPEED		2
#define TIMER_ECONOMY		3
#define TIMER_INIT		4
#define TIMER_SUSPEND_PDMA	5
#define TIMER_RESUME_PDMA	6


#define TIMER_WDT		TIMER_TW0

#define TIMER_MIN		TIMER_TG0
#define TIMER_MAX		TIMER_TG5

#define TIMER_SYSTEM		TIMER_TI0
#define TIMER_SYSTEM2		TIMER_TI1
#define TIMER_PMU		TIMER_TI2
#define TIMER_DSP		TIMER_TI3

#ifdef CONFIG_EMXX_PLL3_238MHZ
#define TIMER_INTERVAL_PLL3     (59392 - 1)
#else
#define TIMER_INTERVAL_PLL3     (57344 - 1)
#endif

#define TIMER_INTERVAL_32K      (320 - 1)
#define TIMER_INTERVAL_DSP      (32 - 1)

#define COUNTER_MAXVAL          TIMER_INTERVAL_PLL3

/* x:clock source value	*/
#define TIMER_DELAY(x)		((40000000 / x)/10 + 1)

#define reg_volatile	volatile

struct tm_param_t {
	unsigned int usecs;
	unsigned int tm_op;
};

struct timer_reg_t {
	reg_volatile unsigned int tm_op;
	reg_volatile unsigned int tm_clr;
	reg_volatile unsigned int tm_set;
	reg_volatile unsigned int tm_rcr;
	reg_volatile unsigned int reserved1;
	reg_volatile unsigned int tm_sclr;
	reg_volatile unsigned int tm_one;
	reg_volatile unsigned int tm_int;
};

struct tm_reg_t {
	unsigned int irq;
	struct timer_reg_t *reg;
	unsigned int clkdev;
	unsigned int rstdev;
	unsigned int tin_sel;
};

struct sti_reg_t {
	reg_volatile unsigned int control;	/* 0x00 */
	reg_volatile unsigned int rfu1[3];	/* 0x04-0x0c */
	reg_volatile unsigned int compa_h;	/* 0x10 */
	reg_volatile unsigned int compa_l;	/* 0x14 */
	reg_volatile unsigned int compb_h;	/* 0x18 */
	reg_volatile unsigned int compb_l;	/* 0x1c */
	reg_volatile unsigned int count_h;	/* 0x20 */
	reg_volatile unsigned int count_l;	/* 0x24 */
	reg_volatile unsigned int count_raw_h;	/* 0x28 */
	reg_volatile unsigned int count_raw_l;	/* 0x2c */
	reg_volatile unsigned int set_h;	/* 0x30 */
	reg_volatile unsigned int set_l;	/* 0x34 */
	reg_volatile unsigned int rfu2[2];	/* 0x38-0x3c */
	reg_volatile unsigned int intstatus;	/* 0x40 */
	reg_volatile unsigned int intrawstatus;	/* 0x44 */
	reg_volatile unsigned int intenset;	/* 0x48 */
	reg_volatile unsigned int intenclr;	/* 0x4c */
	reg_volatile unsigned int intffclr;	/* 0x50 */
};

extern struct sys_timer emxx_timer;

extern void timer_set_clock_parm(unsigned int mode);
extern void timer_set_clock(unsigned int mode);

#endif
