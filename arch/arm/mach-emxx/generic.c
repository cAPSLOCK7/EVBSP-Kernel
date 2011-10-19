/*
 *  File Name       : arch/arm/mach-emxx/generic.c
 *  Function        : generic
 *  Release Version : Ver 1.01
 *  Release Date    : 2010/08/26
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

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/serial_8250.h>
#include <linux/serial_reg.h>

#include <asm/mach/map.h>

#include <mach/hardware.h>
#include <mach/pmu.h>
#include <mach/smu.h>

#include "generic.h"

/*
 * Common EMXX I/O Mapping
 *
 * Logical  Physical
 * e0000000 e0000000-e2ffffff
 * e3000000 1e000000-1e0fffff
 */

static struct map_desc standard_io_desc[] __initdata = {
	{
		.virtual	= IO_ADDRESS(EMXX_INTERNAL_IO_BASE1),
		.pfn		= __phys_to_pfn(EMXX_INTERNAL_IO_BASE1),
		.length		= EMXX_INTERNAL_IO_SIZE1,
		.type		= MT_DEVICE,
	},
	{
		.virtual	= IO_ADDRESS(EMXX_INTERNAL_IO_BASE2),
		.pfn		= __phys_to_pfn(EMXX_INTERNAL_IO_BASE2),
		.length		= EMXX_INTERNAL_IO_SIZE2,
		.type		= MT_DEVICE_SO,
	},
	{
		.virtual	= IO_ADDRESS(EMXX_INTERNAL_IO_BASE3),
		.pfn		= __phys_to_pfn(EMXX_INTERNAL_IO_BASE3),
		.length		= EMXX_INTERNAL_IO_SIZE3,
		.type		= MT_DEVICE,
	},
	{
		.virtual	= EMXX_CPU_LOCAL_VIRT,
		.pfn		= __phys_to_pfn(EMXX_CPU_LOCAL_BASE),
		.length		= EMXX_CPU_LOCAL_SIZE,
		.type		= MT_DEVICE,
	},
	{
		.virtual	= EMXX_SRAM_VIRT,
		.pfn		= __phys_to_pfn(EMXX_SRAM_BASE),
		.length		= EMXX_SRAM_SIZE,
		.type		= MT_DEVICE,
	},
};

static struct plat_serial8250_port serial_platform_data[] = {
	{
	 .membase = (char *)IO_ADDRESS(EMXX_UART0_BASE),
	 .mapbase = (unsigned long)EMXX_UART0_BASE,
	 .irq = INT_UART0,
	 .flags = UPF_BOOT_AUTOCONF | UPF_NO_TXEN_TEST,
	 .iotype = UPIO_MEM32,
	 .regshift = 2,
	 .uartclk = EMXX_BASE_BAUD,
	 },
	{
	 .membase = (char *)IO_ADDRESS(EMXX_UART1_BASE),
	 .mapbase = (unsigned long)EMXX_UART1_BASE,
	 .irq = INT_UART1,
	 .flags = UPF_BOOT_AUTOCONF | UPF_NO_TXEN_TEST,
	 .iotype = UPIO_MEM32,
	 .regshift = 2,
	 .uartclk = EMXX_BASE_BAUD,
	 },
	{
	 .membase = (char *)IO_ADDRESS(EMXX_UART2_BASE),
	 .mapbase = (unsigned long)EMXX_UART2_BASE,
	 .irq = INT_UART2,
	 .flags = UPF_BOOT_AUTOCONF | UPF_NO_TXEN_TEST,
	 .iotype = UPIO_MEM32,
	 .regshift = 2,
	 .uartclk = EMXX_BASE_BAUD,
	 },
	{
	 .membase = (char *)IO_ADDRESS(EMXX_UART3_BASE),
	 .mapbase = (unsigned long)EMXX_UART3_BASE,
	 .irq = INT_UART3,
	 .flags = UPF_BOOT_AUTOCONF | UPF_NO_TXEN_TEST,
	 .iotype = UPIO_MEM32,
	 .regshift = 2,
	 .uartclk = EMXX_BASE_BAUD,
	 },
	{
	 /* terminate */
	 },
};

static struct platform_device dma_device = {
	.name = "dma",
	.id = -1,
};

static struct platform_device spi0_device = {
	.name = "spi0",
	.id = 0,
};

static struct platform_device spi1_device = {
	.name = "spi",
	.id = 1,
};

static struct platform_device i2c0_device = {
	.name = "i2c",
	.id = 0,
};

#ifdef CONFIG_I2C_EMXX_ENABLE_CH2
static struct platform_device i2c1_device = {
	.name = "i2c",
	.id = 1,
};
#endif

static struct platform_device serial8250_device = {
	.name = "serial8250",
	.id = 0,
	.dev = {
		.platform_data = serial_platform_data,
		},
};

static struct platform_device pcm_device = {
	.name = "pcm",
	.id = -1,
};

static struct platform_device pcm1_device = {
	.name = "pcm1",
	.id = -1,
};

static struct platform_device emxx_fb_device = {
	.name = "emxx_fb",
	.id = -1,
};

static struct platform_device emxx_image_device = {
	.name = "emxx_image",
	.id = -1,
};

static struct platform_device emxx_dsp_device = {
	.name = "emxx_dsp",
	.id = -1,
};

static struct platform_device emxx_udc_device = {
	.name = "emxx_udc",
	.id = -1,
};

static struct platform_device emxx_ehci_device = {
	.name = "emxx-ehci-driver",
	.id = -1,
	.dev			= {
		.dma_mask	= (void *)0xffffffff,
		.coherent_dma_mask = 0xffffffff,
	},
};
static struct platform_device emxx_ohci_device = {
	.name = "emxx-ohci-driver",
	.id = -1,
	.dev			= {
		.dma_mask	= (void *)0xffffffff,
		.coherent_dma_mask = 0xffffffff,
	},
};

static struct platform_device emxx_sdio0_device = {
	.name = "emxx_sdio",
	.id = 0,
};
static struct platform_device emxx_sdio1_device = {
	.name = "emxx_sdio1",
	.id = 1,
};

static struct platform_device emxx_sdc_device = {
	.name = "emxx_sdc",
	.id = -1,
};

static struct resource emxx_cfi_resources[] = {
	[0] = {
		.start	= EMXX_CFI_BASE,
		.end	= EMXX_CFI_BASE + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_CFI,
		.end	= INT_CFI,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device emxx_cfi_device = {
	.name = "emxx_cfi_ide",
	.id = -1,
	.num_resources	= ARRAY_SIZE(emxx_cfi_resources),
	.resource	= emxx_cfi_resources,

};

static struct platform_device *platform_devs[] __initdata = {
	&dma_device,
	&spi0_device,
	&spi1_device,
	&i2c0_device,
#ifdef CONFIG_I2C_EMXX_ENABLE_CH2
	&i2c1_device,
#endif
	&serial8250_device,
	&pcm_device,
	&pcm1_device,
	&emxx_fb_device,
	&emxx_image_device,
	&emxx_dsp_device,
	&emxx_udc_device,
	&emxx_ehci_device,
	&emxx_ohci_device,
	&emxx_sdio0_device,
	&emxx_sdio1_device,
	&emxx_sdc_device,
	&emxx_cfi_device,
};

void __init emxx_map_io(void)
{
	iotable_init(standard_io_desc, ARRAY_SIZE(standard_io_desc));
}

void __init emxx_serial_init(int *ports)
{
	int i;
	unsigned int clk, rst, val;

#ifdef CONFIG_MACH_EMEV
	for (i = 0; i < 4; i++) {
#else
	for (i = 0; i < 6; i++) {
#endif
		if (ports[i] == 0) {
			serial_platform_data[i].flags = 0;
			continue;
		}
		switch (i) {
		case 0:
			clk = EMXX_CLK_USIA_U0_P | EMXX_CLK_USIA_U0_S;
			rst = EMXX_RST_USIA_U0_A;
			writel(SMU_PLLSEL_PLL3 | SMU_DIV(2),
				SMU_USIAU0SCLKDIV);
			break;
		case 1:
			clk = EMXX_CLK_USIB_U1_P | EMXX_CLK_USIB_U1_S;
			rst = EMXX_RST_USIB_U1_A;
			val = readl(SMU_USIB2SCLKDIV) & ~0xffff;
			writel(val | SMU_PLLSEL_PLL3 | SMU_DIV(2),
				SMU_USIB2SCLKDIV);
			break;
		case 2:
			clk = EMXX_CLK_USIB_U2_P | EMXX_CLK_USIB_U2_S;
			rst = EMXX_RST_USIB_U2_A;
			val = readl(SMU_USIB2SCLKDIV) & ~0xffff0000;
			writel(val | ((SMU_PLLSEL_PLL3 | SMU_DIV(2)) << 16),
				SMU_USIB2SCLKDIV);
			break;
#ifdef CONFIG_MACH_EMEV
		case 3:
			clk = EMXX_CLK_USIB_U3_P | EMXX_CLK_USIB_U3_S;
			rst = EMXX_RST_USIB_U3_A;
			writel(SMU_PLLSEL_PLL3 | SMU_DIV(2), SMU_USIB3SCLKDIV);
			break;
#else
		case 3:
			clk = EMXX_CLK_USIB_U3_P | EMXX_CLK_USIB_U3_S;
			rst = EMXX_RST_USIB_U3_A;
			val = readl(SMU_USIB3SCLKDIV) & ~0xffff;
			writel(val | SMU_PLLSEL_PLL3 | SMU_DIV(2),
				SMU_USIB3SCLKDIV);
			break;
		case 4:
			clk = EMXX_CLK_USIB_U4_P | EMXX_CLK_USIB_U4_S;
			rst = EMXX_RST_USIB_U4_A;
			val = readl(SMU_USIB3SCLKDIV) & ~0xffff0000;
			writel(val | ((SMU_PLLSEL_PLL3 | SMU_DIV(2)) << 16),
				SMU_USIB3SCLKDIV);
			break;
		case 5:
			clk = EMXX_CLK_USIB_U5_P | EMXX_CLK_USIB_U5_S;
			rst = EMXX_RST_USIB_U5_A;
			writel(SMU_PLLSEL_PLL3 | SMU_DIV(2), SMU_USIB4SCLKDIV);
			break;
#endif
		default:
			return;
		}
		/* Unreset U7x */
		emxx_open_clockgate(clk);
		emxx_unreset_device(rst);
	}
}

static int __init emxx_init(void)
{
	return platform_add_devices(platform_devs, ARRAY_SIZE(platform_devs));
}

arch_initcall(emxx_init);
