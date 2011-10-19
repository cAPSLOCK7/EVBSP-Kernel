/*
 *  File Name	    : arch/arm/mach-emxx/include/mach/pwm.h
 *  Function        : pwm
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
 * Inc., 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA.
 *
 */
#ifndef __ASM_ARCH_EMXX_PWM_H
#define __ASM_ARCH_EMXX_PWM_H

#define EMXX_PWM_CH0		0
#define EMXX_PWM_CH1		1

#define EMXX_PWM_COMPARE0	0
#define EMXX_PWM_COMPARE1	1
#define EMXX_PWM_COMPARE2	2

#define PWM_BIT_CMP2		0x04
#define PWM_BIT_CMP1		0x02
#define PWM_BIT_CMP0		0x01

#define PWM_CODE_MODE_OR	0		/* CMP0-CMP 2 OR output */
#define PWM_CODE_MODE_AND	1		/* CMP0-CMP 2 AND output */
#define PWM_CODE_MODE_XOR	2		/* CMP0-CMP 2 XOR output */

typedef void (*pwm_callback_t) (void *data, int intsts, int intrawsts);

struct emxx_pwm_cb_info_t {
	pwm_callback_t cb_count;
	void *cb_count_data;
	pwm_callback_t cb_loop;
	void *cb_loop_data;
};

struct emxx_pwm_cmpcnt_t {
	unsigned char cmp;
	unsigned int delay;
	unsigned int lead_edge;
	unsigned int trail_edge;
	unsigned int total_cycle;
	unsigned int loop_count;
	struct emxx_pwm_cb_info_t *cb_info;
};

struct emxx_pwm_ch_config_t {
	unsigned char use_cmp;		/* cmp enable status */
	unsigned char mode;
	unsigned char inverse;
	unsigned char interrupt_count;
	unsigned char interrupt_loop;
};

extern int emxx_pwm_start(unsigned char channel);
extern int emxx_pwm_stop(unsigned char channel);
extern int emxx_pwm_set_channel_config(unsigned char channel,
				struct emxx_pwm_ch_config_t *ch_config);
extern int emxx_pwm_get_channel_config(unsigned char channel,
				struct emxx_pwm_ch_config_t *ch_config);
extern int emxx_pwm_set_compare_counter(unsigned char channel,
				struct emxx_pwm_cmpcnt_t *pwm_cmpcnt);
extern int emxx_pwm_get_compare_counter(unsigned char channel,
				struct emxx_pwm_cmpcnt_t *pwm_cmpcnt);

#endif	/* end of __ASM_ARCH_EMXX_PWM_H */

