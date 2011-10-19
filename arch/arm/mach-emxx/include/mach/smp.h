/*
 *  File Name       : arch/arm/mach-emxx/include/mach/smp.h
 *  Function        : smp
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

#ifndef __ASM_ARCH_SMP_H
#define __ASM_ARCH_SMP_H


#include <asm/hardware/gic.h>

#define hard_smp_processor_id()			\
	({						\
		unsigned int cpunum;			\
		__asm__("mrc p15, 0, %0, c0, c0, 5"	\
			: "=r" (cpunum));		\
		cpunum &= 0x0F;				\
	})

/*
 * We use IRQ1 as the IPI
 */
static inline void smp_cross_call(cpumask_t callmap)
{
	gic_raise_softirq(callmap, 1);
}

static inline void smp_cross_call_done(cpumask_t callmap)
{
}

#endif	/* __ASM_ARCH_SMP_H */
