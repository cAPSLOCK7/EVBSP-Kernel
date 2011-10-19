/*
 *  File Name       : linux/drivers/input/keyboard/emev_max7318.h
 *  Function        : button Interface
 *  Release Version : Ver 1.01
 *  Release Date    : 2010/08/09
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

#ifndef __EMXX_MAX7324_H__
#define __EMXX_MAX7324_H__

#include <mach/irqs.h>
#include <mach/gpio.h>

/* key scan default parameter */
#define BTN_CHAT_GUARDTIME	20	/* 20ms */
#define BTN_SCAN_INTERVAL	30	/* 30ms */

#define EMXX_BTN_NAME		"max7318_key"

struct emxx_btn_table {
	unsigned int scancode;	/* keycode */
	unsigned long btn_id_bit;	/* button id bit */
};

#endif /* __EMXX_MAX7324_H__ */
