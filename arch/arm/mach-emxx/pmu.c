/*
 *  File Name	    : arch/arm/mach-emxx/pmu.c
 *  Function	    : pmu
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
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/io.h>

#include <mach/smu.h>
#include <mach/pmu.h>

#define SMU_REG_MASK		0xffff0000
#define SMU_REG_BIT_MASK	0x0000ffff
#define SMU_REG_MAX_BIT_LEN	32

static DEFINE_SPINLOCK(lock_clkctrl);

static int clk_rst_func(unsigned int clk_rst, int on, int clk, int status)
{
	unsigned int mask, offset, regval;
	unsigned int reg_addr = IO_ADDRESS(EMXX_SMU_BASE);

	mask = clk_rst & SMU_REG_BIT_MASK;
	offset = (clk_rst & SMU_REG_MASK) >> 16;

	reg_addr += offset;

	if (clk) {
		if ((reg_addr > SMU_PDMAGCLKCTRL) ||
				(reg_addr < SMU_DSPGCLKCTRL))
			return -EINVAL;
	} else {
		if ((reg_addr > SMU_P2M_SAFE_RESET) ||
				(reg_addr < SMU_DSP_RSTCTRL))
			return -EINVAL;
	}

	regval = readl(reg_addr);

	if (status) {
		return (regval & mask) ? 1 : 0;
	} else {
		if (on)
			regval |= mask;
		else
			regval &= ~mask;
		writel(regval, reg_addr);
		return 0;
	}
}

static int clkctrl_func(unsigned int dev, int on, int status)
{
	unsigned long cpu_flags;
	unsigned int regval, reg_addr;
	unsigned int mask;
	int bit;

	bit = dev & SMU_REG_BIT_MASK;
	if (bit >= SMU_REG_MAX_BIT_LEN)
		return -EINVAL;
	mask = 1 << bit;

	switch (dev & SMU_REG_MASK) {
	case SMU_AHBCLKCTRL0_GROUP:
		reg_addr = SMU_AHBCLKCTRL0;
		break;
	case SMU_AHBCLKCTRL1_GROUP:
		reg_addr = SMU_AHBCLKCTRL1;
		break;
	case SMU_AHBCLKCTRL2_GROUP:
		reg_addr = SMU_AHBCLKCTRL2;
		break;
	case SMU_AHBCLKCTRL3_GROUP:
		reg_addr = SMU_AHBCLKCTRL3;
		break;
	case SMU_APBCLKCTRL0_GROUP:
		reg_addr = SMU_APBCLKCTRL0;
		break;
	case SMU_APBCLKCTRL1_GROUP:
		reg_addr = SMU_APBCLKCTRL1;
		break;
	case SMU_APBCLKCTRL2_GROUP:
		reg_addr = SMU_APBCLKCTRL2;
		break;
	case SMU_CLKCTRL_GROUP:
		reg_addr = SMU_CLKCTRL;
		break;
	case SMU_AVECLKCTRL_GROUP:
		reg_addr = SMU_AVECLKCTRL;
		break;
	default:
		return -EINVAL;
	}

	if (status) {
		regval = readl(reg_addr);
		return (regval & mask) ? 1 : 0;
	} else {
		spin_lock_irqsave(&lock_clkctrl, cpu_flags);

		regval = readl(reg_addr);
		if (on)
			regval |= mask;
		else
			regval &= ~mask;
		writel(regval, reg_addr);

		spin_unlock_irqrestore(&lock_clkctrl, cpu_flags);
		return 0;
	}
}

int emxx_open_clockgate(unsigned int clk)
{
	return clk_rst_func(clk, 1, 1, 0);
}
EXPORT_SYMBOL(emxx_open_clockgate);

int emxx_close_clockgate(unsigned int clk)
{
	return clk_rst_func(clk, 0, 1, 0);
}
EXPORT_SYMBOL(emxx_close_clockgate);

int emxx_get_clockgate(unsigned int clk)
{
	return clk_rst_func(clk, 0, 1, 1);
}
EXPORT_SYMBOL(emxx_get_clockgate);


int emxx_reset_device(unsigned int rst)
{
	return clk_rst_func(rst, 0, 0, 0);
}
EXPORT_SYMBOL(emxx_reset_device);

int emxx_unreset_device(unsigned int rst)
{
	return clk_rst_func(rst, 1, 0, 0);
}
EXPORT_SYMBOL(emxx_unreset_device);

int emxx_get_reset(unsigned int rst)
{
	return clk_rst_func(rst, 0, 0, 1);
}
EXPORT_SYMBOL(emxx_get_reset);


int emxx_clkctrl_on(unsigned int dev)
{
	return clkctrl_func(dev, 1, 0);
}
EXPORT_SYMBOL(emxx_clkctrl_on);

int emxx_clkctrl_off(unsigned int dev)
{
	return clkctrl_func(dev, 0, 0);
}
EXPORT_SYMBOL(emxx_clkctrl_off);

int emxx_get_clkctrl(unsigned int dev)
{
	return clkctrl_func(dev, 0, 1);
}
EXPORT_SYMBOL(emxx_get_clkctrl);

