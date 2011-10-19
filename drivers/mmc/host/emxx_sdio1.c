/*
 *  File Name		: linux/drivers/mmc/host/emxx_sdio1.c
 *  Function		: MMC
 *  Release Version	: Ver 1.00
 *  Release Date	: 2010/07/23
 *
 *  Copyright (C) 2010 Renesas Electronics Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include "emxx_sdio.h"

static int __init emxx_sdio1_init(void)
{
	return platform_driver_register(&emxx_mmc_driver2);
}

static void __exit emxx_sdio1_cleanup(void)
{
	platform_driver_unregister(&emxx_mmc_driver2);
}

module_init(emxx_sdio1_init);
module_exit(emxx_sdio1_cleanup);

MODULE_DESCRIPTION("EMXX SDIO High Speed Multimedia Card driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Renesas Electronics Corporation");
