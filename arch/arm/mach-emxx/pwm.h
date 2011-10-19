/*
 *  File Name	    : linux/drivers/char/pwm.h
 *  Function	    : Pulse Width Moduration(pwm).
 *  Release Version : Ver 1.00
 *  Release Date    : 2010/03/18
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

#ifndef __ARCH_MACH_EMXX_PWM_H
#define __ARCH_MACH_EMXX_PWM_H

#include <mach/pmu.h>

#define PWM_MAX_CHANNEL_NUM	2
#define PWM_MAX_COUNTER_NUM	3

#define PWM_DIVPWMPWCLK     SMU_DIV(8)
#define PWM_INIT_CLOCK      (PWM_DIVPWMPWCLK | (PWM_DIVPWMPWCLK << 16))

/* device name */
#define PWM_NAME		"pwm"

#define PWM_CODE_COUNTER0	0x01
#define PWM_CODE_COUNTER1	0x02
#define PWM_CODE_COUNTER2	0x04
#define PWM_MASK_CODE_COUNTER	0x07
#define PWM_MASK_CODE_INVERSE	0x07
#define PWM_MASK_CODE_INTCOUNT	0x07
#define PWM_MASK_CODE_INTLOOP	0x07

#define PWM_STATE_STOP		1
#define PWM_STATE_START		2

#define PWM_PINSEL_MASK		0x50000000	/* set 0x5 is PWM */

#define	PWM_START_BIT_ON	0x00000001	/* count start */
#define	PWM_START_BIT_OFF	0x00000000	/* count stop */

#define PWM_MASK_MODE		0x00030000
#define PWM_MODE_SFT		16

#define PWM_CMP_ATST2_BIT	0x00000200
#define PWM_CMP_ATST1_BIT	0x00000020
#define PWM_CMP_ATST0_BIT	0x00000002
#define PWM_CMP_ATST2_SFT	9
#define PWM_CMP_ATST1_SFT	5
#define PWM_CMP_ATST0_SFT	1

#define PWM_CMP_EN2_BIT		0x00000100
#define PWM_CMP_EN1_BIT		0x00000010
#define PWM_CMP_EN0_BIT		0x00000001
#define PWM_CMP_EN2_SFT		8
#define PWM_CMP_EN1_SFT		4
#define PWM_CMP_EN0_SFT		0
#define PWM_CMP_EN2_POS		(PWM_CMP_EN2_SFT-2)
#define PWM_CMP_EN1_POS		(PWM_CMP_EN1_SFT-1)
#define PWM_CMP_EN0_POS		(PWM_CMP_EN0_SFT-0)

#define PWM_CMP_INV2_BIT	0x00000400
#define PWM_CMP_INV1_BIT	0x00000040
#define PWM_CMP_INV0_BIT	0x00000004
#define PWM_CMP_INV2_SFT	10
#define PWM_CMP_INV1_SFT	6
#define PWM_CMP_INV0_SFT	2
#define PWM_CMP_INV2_POS	(PWM_CMP_INV2_SFT-2)
#define PWM_CMP_INV1_POS	(PWM_CMP_INV1_SFT-1)
#define PWM_CMP_INV0_POS	(PWM_CMP_INV0_SFT-0)

#define PWM_MASK_MODE_SETBIT \
	(PWM_MASK_MODE | PWM_CMP_EN2_BIT | PWM_CMP_EN1_BIT | \
	 PWM_CMP_EN0_BIT | PWM_CMP_INV2_BIT | PWM_CMP_INV1_BIT | \
	 PWM_CMP_INV0_BIT)


#define PWM_CMP2_CENDSET_BIT	0x00000010
#define PWM_CMP1_CENDSET_BIT	0x00000004
#define PWM_CMP0_CENDSET_BIT	0x00000001
#define PWM_CMP2_CENDSET_SFT	4
#define PWM_CMP1_CENDSET_SFT	2
#define PWM_CMP0_CENDSET_SFT	0
#define PWM_CMP2_CENDSET_POS	(PWM_CMP2_CENDSET_SFT-2)
#define PWM_CMP1_CENDSET_POS	(PWM_CMP1_CENDSET_SFT-1)
#define PWM_CMP0_CENDSET_POS	(PWM_CMP0_CENDSET_SFT-0)

#define PWM_CMP2_LENDSET_BIT	0x00000020
#define PWM_CMP1_LENDSET_BIT	0x00000008
#define PWM_CMP0_LENDSET_BIT	0x00000002
#define PWM_CMP2_LENDSET_SFT	5
#define PWM_CMP1_LENDSET_SFT	3
#define PWM_CMP0_LENDSET_SFT	1
#define PWM_CMP2_LENDSET_POS	(PWM_CMP2_LENDSET_SFT-2)
#define PWM_CMP1_LENDSET_POS	(PWM_CMP1_LENDSET_SFT-1)
#define PWM_CMP0_LENDSET_POS	(PWM_CMP0_LENDSET_SFT-0)


#define PWM_MASK_ENDSET_SETBIT \
	(PWM_CMP2_CENDSET_BIT | PWM_CMP1_CENDSET_BIT | \
	 PWM_CMP0_CENDSET_BIT | PWM_CMP2_LENDSET_BIT | \
	 PWM_CMP1_LENDSET_BIT | PWM_CMP0_LENDSET_BIT)


/* -------- */

#define PWM_CH0_CTRL			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0000)
#define PWM_CH0_MODE			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0004)
#define PWM_CH0_DELAY0			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0010)
#define PWM_CH0_LEDGE0			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0014)
#define PWM_CH0_TEDGE0			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0018)
#define PWM_CH0_TOTAL0			(IO_ADDRESS(EMXX_PWM_BASE) + 0x001C)
#define PWM_CH0_LOOP0			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0020)
#define PWM_CH0_DELAY1			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0040)
#define PWM_CH0_LEDGE1			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0044)
#define PWM_CH0_TEDGE1			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0048)
#define PWM_CH0_TOTAL1			(IO_ADDRESS(EMXX_PWM_BASE) + 0x004C)
#define PWM_CH0_LOOP1			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0050)
#define PWM_CH0_DELAY2			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0080)
#define PWM_CH0_LEDGE2			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0084)
#define PWM_CH0_TEDGE2			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0088)
#define PWM_CH0_TOTAL2			(IO_ADDRESS(EMXX_PWM_BASE) + 0x008C)
#define PWM_CH0_LOOP2			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0090)

#define PWM_CH1_CTRL			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0100)
#define PWM_CH1_MODE			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0104)
#define PWM_CH1_DELAY0			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0110)
#define PWM_CH1_LEDGE0			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0114)
#define PWM_CH1_TEDGE0			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0118)
#define PWM_CH1_TOTAL0			(IO_ADDRESS(EMXX_PWM_BASE) + 0x011C)
#define PWM_CH1_LOOP0			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0120)
#define PWM_CH1_DELAY1			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0140)
#define PWM_CH1_LEDGE1			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0144)
#define PWM_CH1_TEDGE1			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0148)
#define PWM_CH1_TOTAL1			(IO_ADDRESS(EMXX_PWM_BASE) + 0x014C)
#define PWM_CH1_LOOP1			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0150)
#define PWM_CH1_DELAY2			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0180)
#define PWM_CH1_LEDGE2			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0184)
#define PWM_CH1_TEDGE2			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0188)
#define PWM_CH1_TOTAL2			(IO_ADDRESS(EMXX_PWM_BASE) + 0x018C)
#define PWM_CH1_LOOP2			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0190)

/* ---------------
   PWM Interrupt
   --------------- */
#define PWM_INTSTATUS			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0400)
#define PWM_INTRAWSTATUS		(IO_ADDRESS(EMXX_PWM_BASE) + 0x0404)
#define PWM_INTENSET			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0408)
#define PWM_INTENCLR			(IO_ADDRESS(EMXX_PWM_BASE) + 0x040C)
#define PWM_INTFFCLR			(IO_ADDRESS(EMXX_PWM_BASE) + 0x0410)
/* -------- */

struct pwm_info_t {
	unsigned int state;
	unsigned int channel_control;
	unsigned int channel_mode;
};

#endif	/* __ARCH_MACH_EMXX_PWM_H */
