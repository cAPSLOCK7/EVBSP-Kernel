/*
 *  File Name       : arch/arm/mach-emxx/include/mach/pm.h
 *  Function	    : Power Management
 *  Release Version : Ver 1.01
 *  Release Date    : 2010/10/27
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
#ifndef __ASM_ARCH_EMXX_PM_H
#define __ASM_ARCH_EMXX_PM_H

#ifdef CONFIG_MACH_EMEV
#define NORMAL_B_VALUE	0x05757133
#elif defined(CONFIG_MACH_EMGR)
#define NORMAL_B_VALUE	0x05757333
#endif

extern int emxx_sleep_while_idle;
#ifdef CONFIG_PM
extern void emxx_pm_pdma_suspend_enable(void);
extern void emxx_pm_pdma_suspend_disable(void);
extern unsigned int emxx_pm_pdma_suspend_status(void);
#else
#define emxx_pm_pdma_suspend_enable()
#define emxx_pm_pdma_suspend_disable()
#define emxx_pm_pdma_suspend_status() 	0
#endif

#endif /* __ASM_ARCH_EMXX_PM_H */
