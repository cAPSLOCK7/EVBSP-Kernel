/*
 * File Name       : arch/arm/mach-emxx/include/mach/emxx_imc.h
 * Function        : IMC Driver I/F definitions
 * Release Version : Ver 1.01
 * Release Date    : 2010.05.14
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
 */

#ifndef _EMXX_IMC__H_
#define _EMXX_IMC__H_


extern void imc_hw_reset(int channel);
extern void imc_hw_unreset(int channel);
extern void imc_hw_wait_stop(int channel);
#if defined(CONFIG_PM) || defined(CONFIG_DPM)
extern int emxx_imc_suspend(struct platform_device *dev, pm_message_t state);
extern int emxx_imc_resume(struct platform_device *dev);
#endif /* CONFIG_PM || CONFIG_DPM */


#endif /* _EMXX_IMC__H_ */
