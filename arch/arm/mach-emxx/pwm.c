/*
 *  File Name       : arch/arm/mach-emxx/pwm.c
 *  Function        : Pulse Width Moduration(pwm).
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

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#include <mach/pmu.h>
#include <mach/smu.h>
#include <mach/irqs.h>
#include <mach/pwm.h>

/* #define EMXX_PWM_DEBUG 1 */

#include "pwm.h"

#if defined(EMXX_PWM_DEBUG)
#define FNC_ENTRY \
	if (debug == 1 || debug >= 9) {	\
		printk(KERN_DEBUG "entry:%s\n", __func__); \
	}
#define FNC_EXIT \
	if (debug == 1 || debug >= 9) {	\
		printk(KERN_DEBUG "exit:%s:%d\n", __func__ , __LINE__); \
	}
#define d0b(fmt, args...)	\
		printk(KERN_DEBUG "%s:%d: " fmt , __func__ , __LINE__ , \
		       ## args);
static int debug;
#else
#define FNC_ENTRY
#define FNC_EXIT
#define d0b(fmt, args...)
#endif

#if	defined(EMXX_PWM_DEBUG)
#define DEBUG_PRINT(FMT, ARGS...) \
		printk(KERN_INFO "%s(): " FMT, __func__ , ##ARGS)
#else
#define DEBUG_PRINT(FMT, ARGS...)
#endif

static DEFINE_SPINLOCK(spin_lock);

#if	defined(EMXX_PWM_DEBUG)
int pwm_init_flag = 1;
EXPORT_SYMBOL(pwm_init_flag);
#else
static int pwm_init_flag = 1;
#endif

/* ------------------------------------------------------------------
   register address, state, callback default define
   ------------------------------------------------------------------ */
static struct emxx_pwm_cb_info_t
g_pwm_cb_info[PWM_MAX_CHANNEL_NUM][PWM_MAX_COUNTER_NUM];

static struct pwm_info_t g_pwm_info[PWM_MAX_CHANNEL_NUM] = {
	{
		PWM_STATE_STOP,
		PWM_CH0_CTRL,
		PWM_CH0_MODE,
	},
	{
		PWM_STATE_STOP,
		PWM_CH1_CTRL,
		PWM_CH1_MODE,
	},
};


static struct emxx_pwm_cmpcnt_t
g_pwm_cmpcnt[PWM_MAX_CHANNEL_NUM][PWM_MAX_COUNTER_NUM] = {
	{
		{
			EMXX_PWM_COMPARE0,
			PWM_CH0_DELAY0,
			PWM_CH0_LEDGE0,
			PWM_CH0_TEDGE0,
			PWM_CH0_TOTAL0,
			PWM_CH0_LOOP0,
			NULL,
		},
		{
			EMXX_PWM_COMPARE1,
			PWM_CH0_DELAY1,
			PWM_CH0_LEDGE1,
			PWM_CH0_TEDGE1,
			PWM_CH0_TOTAL1,
			PWM_CH0_LOOP1,
			NULL,
		},
		{
			EMXX_PWM_COMPARE2,
			PWM_CH0_DELAY2,
			PWM_CH0_LEDGE2,
			PWM_CH0_TEDGE2,
			PWM_CH0_TOTAL2,
			PWM_CH0_LOOP2,
			NULL,
		},
	},
	{
		{
			EMXX_PWM_COMPARE0,
			PWM_CH1_DELAY0,
			PWM_CH1_LEDGE0,
			PWM_CH1_TEDGE0,
			PWM_CH1_TOTAL0,
			PWM_CH1_LOOP0,
			NULL,
		},
		{
			EMXX_PWM_COMPARE1,
			PWM_CH1_DELAY1,
			PWM_CH1_LEDGE1,
			PWM_CH1_TEDGE1,
			PWM_CH1_TOTAL1,
			PWM_CH1_LOOP1,
			NULL,
		},
		{
			EMXX_PWM_COMPARE2,
			PWM_CH1_DELAY2,
			PWM_CH1_LEDGE2,
			PWM_CH1_TEDGE2,
			PWM_CH1_TOTAL2,
			PWM_CH1_LOOP2,
			NULL,
		},
	},
};

/* ------------------------------------------------------------------
   smu control.
   ------------------------------------------------------------------ */
static inline void pwm_pwclk_start(int ch)
{
	if (ch == EMXX_PWM_CH0)
		emxx_open_clockgate(EMXX_CLK_PWM0);
	else if (ch == EMXX_PWM_CH1)
		emxx_open_clockgate(EMXX_CLK_PWM1);
}

static inline void pwm_pwclk_stop(int ch)
{
	if (ch == EMXX_PWM_CH0)
		emxx_close_clockgate(EMXX_CLK_PWM0);
	else if (ch == EMXX_PWM_CH1)
		emxx_close_clockgate(EMXX_CLK_PWM1);
}

static inline void pwm_clock_start(void)
{
	emxx_open_clockgate(EMXX_CLK_PWM1);
	emxx_open_clockgate(EMXX_CLK_PWM0);
	emxx_open_clockgate(EMXX_CLK_PWM_P);
}

static inline void pwm_clock_stop(void)
{
	emxx_close_clockgate(EMXX_CLK_PWM1);
	emxx_close_clockgate(EMXX_CLK_PWM0);
	emxx_close_clockgate(EMXX_CLK_PWM_P);
}

static inline void pwm_dev_unreset(void)
{
	emxx_unreset_device(EMXX_RST_PWM);
}

/* ------------------------------------------------------------------
   channel number check.
   ------------------------------------------------------------------ */
static inline int
check_validity_channel(unsigned char channel)
{
	FNC_ENTRY
	if ((channel == EMXX_PWM_CH0) || (channel == EMXX_PWM_CH1)) {
		FNC_EXIT
		return 0;
	}
	DEBUG_PRINT("channnel parameter error.\n");
	FNC_EXIT
	return 1;
}


/* ------------------------------------------------------------------
   compare_counter number check.
   ------------------------------------------------------------------ */
static inline int
check_validity_compare_counter(unsigned char cmp_counter)
{
	FNC_ENTRY
	if (cmp_counter > EMXX_PWM_COMPARE2) {
		FNC_EXIT
		return 1;
	}
	FNC_EXIT
	return 0;
}

/* --------------------------------------------------------------
   function : pwm_irq_handerl
   argument :
   return   : none
   comment  : dev_id ... is compare_registers address.
   -------------------------------------------------------------- */
static irqreturn_t
pwm_irq_handler(int irq, void *dev_id)
{
	struct emxx_pwm_cb_info_t *info;
	unsigned int intsts, rawsts, sts, raw;
	unsigned int shift;
	unsigned int ch, cmp;
	FNC_ENTRY

	/* save interrupt status */
	intsts = readl(PWM_INTSTATUS);
	rawsts = readl(PWM_INTRAWSTATUS);
	/* clear interrupt status */
	writel(intsts, PWM_INTFFCLR);

	for (ch = 0; ch < PWM_MAX_CHANNEL_NUM; ch++) {
		if ((intsts & (PWM_MASK_ENDSET_SETBIT << (ch << 3))) == 0)
			continue;

		for (cmp = 0; cmp < PWM_MAX_COUNTER_NUM; cmp++) {
			if ((intsts & (0x3 << ((cmp << 1) + (ch << 3)))) == 0)
				continue;

			info = g_pwm_cmpcnt[ch][cmp].cb_info;
			if (NULL == info)
				continue;

			shift = ((cmp << 1) + (ch << 3));
			sts = (intsts >> shift) & 0x3;
			raw = (rawsts >> shift) & 0x3;
			/* check interrupt */
			if (sts & 0x1) {
				if (info->cb_count) {
					info->cb_count(
						info->cb_count_data, sts, raw);
				}
			}
			if (sts & 0x2) {
				if (info->cb_loop) {
					info->cb_loop(
						info->cb_loop_data, sts, raw);
				}
			}
		}
	}

	FNC_EXIT
	return IRQ_HANDLED;
}

/* --------------------------------------------------------------
   function : emxx_pwm_start
   argument : channel  . channel number
   -------------------------------------------------------------- */
int emxx_pwm_start(unsigned char channel)
{
	struct pwm_info_t *pwm_info_p;
	int err;
	unsigned int reg_value;
	unsigned long lock_flags;
	FNC_ENTRY

	if (pwm_init_flag) {
		DEBUG_PRINT("pwm initialize error\n");
		FNC_EXIT
		return -EPERM;
	}

	/* check parameter */
	err = check_validity_channel(channel);
	if (err != 0) {
		FNC_EXIT
		return -ENODEV;
	}
	DEBUG_PRINT("channle ok (ch%d)\n", channel);

	pwm_info_p = &g_pwm_info[channel];

	spin_lock_irqsave(&spin_lock, lock_flags);

	/* check state */
	if (PWM_STATE_STOP != pwm_info_p->state) {
		DEBUG_PRINT("device active now. ---> end for unlock.\n");
		spin_unlock_irqrestore(&spin_lock, lock_flags);
		FNC_EXIT return(-EBUSY);
	}
	DEBUG_PRINT("state ok (0x%08X)\n", pwm_info_p->state);

	/* regist irq_handler */
	/* 5,dev_id  field : use for regs */

	/* read use ch(read mode reg) */
	reg_value = readl(pwm_info_p->channel_mode);

	if (!(reg_value &
		(PWM_CMP_EN0_BIT | PWM_CMP_EN1_BIT | PWM_CMP_EN2_BIT))) {
		/* counter all not use */
		DEBUG_PRINT("compare counter no setting.\n");
		spin_unlock_irqrestore(&spin_lock, lock_flags);
		FNC_EXIT
		return -EINVAL;
	}

	pwm_pwclk_start(channel);
	if (PWM_STATE_STOP == g_pwm_info[(channel + 1) & 0x01].state) {
		DEBUG_PRINT("device all stop in now. ---> enable irq.\n");
		enable_irq(INT_PWM);
	}
	/* start countup */
	writel(PWM_START_BIT_ON, pwm_info_p->channel_control);
	DEBUG_PRINT("PWM Start.(CONTROL[0x%08X])\n",
			pwm_info_p->channel_control);

	/* change state */
	pwm_info_p->state = PWM_STATE_START;

	spin_unlock_irqrestore(&spin_lock, lock_flags);

	FNC_EXIT
	return 0;
}
EXPORT_SYMBOL(emxx_pwm_start);

/* --------------------------------------------------------------
   function : emxx_pwm_stop
   explain  : stop pwm
   argument : channel        : channel number
   return   : BIOS_OK        : success
	      BIOS_ERR_PARAM : parameter error
   -------------------------------------------------------------- */
int emxx_pwm_stop(unsigned char channel)
{
	int err;
	unsigned long lock_flags;
	FNC_ENTRY

	if (pwm_init_flag) {
		DEBUG_PRINT("pwm initialize error\n");
		FNC_EXIT
		return -EPERM;
	}

	/* check parameter */
	err = check_validity_channel(channel);
	if (err != 0) {
		FNC_EXIT
		return -ENODEV;
	}

	spin_lock_irqsave(&spin_lock, lock_flags);

	/* check state */
	if (PWM_STATE_STOP == g_pwm_info[channel].state) {
		DEBUG_PRINT("device active now. ---> end for unlock.\n");
		spin_unlock_irqrestore(&spin_lock, lock_flags);
		FNC_EXIT
		return 0;
	}

	/* stop countup */
	writel(PWM_START_BIT_OFF, g_pwm_info[channel].channel_control);
	DEBUG_PRINT("PWM Stop.(CONTROL[0x%08X])\n",
			g_pwm_info[channel].channel_control);

	/* change state */
	g_pwm_info[channel].state = PWM_STATE_STOP;
	if (PWM_STATE_STOP == g_pwm_info[(channel + 1) & 0x01].state) {
		DEBUG_PRINT("device all stop in now. ---> disable irq..\n");
		disable_irq(INT_PWM);
	}
	pwm_pwclk_stop(channel);

	spin_unlock_irqrestore(&spin_lock, lock_flags);

	FNC_EXIT
	return 0;
}
EXPORT_SYMBOL(emxx_pwm_stop);

/* --------------------------------------------------------------
   function : emxx_pwm_set_channel_config
   explain  : set pwm workings
   argument : channel   : channel number
	      ch_config : PWM configuration structure pointer
   return   : BIOS_OK         : success
	      BIOS_ERR_PARAM  : parameter error
	      BIOS_ERR_CONFIG : illegal configuration structure member
	      BIOS_ERR_STATE  : During counting
   -------------------------------------------------------------- */
int emxx_pwm_set_channel_config(unsigned char channel,
		struct emxx_pwm_ch_config_t *ch_config)
{
	int err;
	unsigned int mode_value;
	unsigned int set_value;
	unsigned int value;
	unsigned int reg_int_sft;
	struct pwm_info_t *pwm_info_p;
	unsigned long lock_flags;
	FNC_ENTRY

	if (pwm_init_flag) {
		DEBUG_PRINT("pwm initialize error\n");
		FNC_EXIT
		return -EPERM;
	}

	/* check parameter */
	err = check_validity_channel(channel);
	if (err != 0) {
		FNC_EXIT
		return -ENODEV;
	}
	if (NULL == ch_config) {
		FNC_EXIT
		return -EINVAL;
	}

	pwm_info_p = &g_pwm_info[channel];

	spin_lock_irqsave(&spin_lock, lock_flags);

	/* check state */
	if (PWM_STATE_STOP != pwm_info_p->state) {
		DEBUG_PRINT("device active now. ---> end for unlock.\n");
		spin_unlock_irqrestore(&spin_lock, lock_flags);
		FNC_EXIT return(-EBUSY);
	}

	/* check config argument value  */
	if ((~PWM_MASK_CODE_COUNTER & ch_config->use_cmp)
	      || (PWM_CODE_MODE_XOR < ch_config->mode)	/* max is MODE_XOR */
	      || (~PWM_MASK_CODE_INVERSE & ch_config->inverse)
	      || (~PWM_MASK_CODE_INTCOUNT & ch_config->interrupt_count)
	      || (~PWM_MASK_CODE_INTLOOP & ch_config->interrupt_loop)) {
		DEBUG_PRINT("argument invalid. end for unlock.\n");
		spin_unlock_irqrestore(&spin_lock, lock_flags);
		FNC_EXIT
		return -EINVAL;
	}

	/* set use counters */
	reg_int_sft = channel << 3;


	/* read mode reg */
	mode_value = readl(pwm_info_p->channel_mode) &
			~PWM_MASK_MODE_SETBIT;

	/* set mode */
	mode_value |= ch_config->mode << PWM_MODE_SFT;

	/* set use_cmp */
	value = ch_config->use_cmp;
	mode_value |= ((value & PWM_BIT_CMP2) << PWM_CMP_EN2_POS)
		| ((value & PWM_BIT_CMP1) << PWM_CMP_EN1_POS)
		| ((value & PWM_BIT_CMP0) << PWM_CMP_EN0_POS);

	/* set inverse */
	value = ch_config->inverse;
	mode_value |= ((value & PWM_BIT_CMP2) << PWM_CMP_INV2_POS)
		| ((value & PWM_BIT_CMP1) << PWM_CMP_INV1_POS)
		| ((value & PWM_BIT_CMP0) << PWM_CMP_INV0_POS);
	/* set mode reg */
	writel(mode_value, pwm_info_p->channel_mode);

	/* set interrupt count */
	value = ch_config->interrupt_count;
	set_value = ((value & PWM_BIT_CMP2) << PWM_CMP2_CENDSET_POS)
		| ((value & PWM_BIT_CMP1) << PWM_CMP1_CENDSET_POS)
		| ((value & PWM_BIT_CMP0) << PWM_CMP0_CENDSET_POS);

	/* interrupt_loop */
	value = ch_config->interrupt_loop;
	set_value |= ((value & PWM_BIT_CMP2) << PWM_CMP2_LENDSET_POS)
		| ((value & PWM_BIT_CMP1) << PWM_CMP1_LENDSET_POS)
		| ((value & PWM_BIT_CMP0) << PWM_CMP0_LENDSET_POS);

	/* set int disable reg */
	writel((PWM_MASK_ENDSET_SETBIT << reg_int_sft), PWM_INTENCLR);

	/* set int enable reg */
	writel((set_value << reg_int_sft), PWM_INTENSET);

	spin_unlock_irqrestore(&spin_lock, lock_flags);

	FNC_EXIT
	return 0;
}
EXPORT_SYMBOL(emxx_pwm_set_channel_config);

/* --------------------------------------------------------------
   function : emxx_pwm_get_channel_config;
	    : get channnel status
   argument : channel   ... ch number
	      ch_config ... PWM configuration structure pointer
   return   :
   -------------------------------------------------------------- */
int emxx_pwm_get_channel_config(unsigned char channel,
		struct emxx_pwm_ch_config_t *ch_config)
{
	int err;
	unsigned int reg_value;
	FNC_ENTRY

	if (pwm_init_flag) {
		DEBUG_PRINT("pwm initialize error\n");
		FNC_EXIT
		return -EPERM;
	}

	/* check parameter */
	err = check_validity_channel(channel);
	if (err != 0) {
		FNC_EXIT
		return -ENODEV;
	}
	if ((NULL == ch_config)) {
		FNC_EXIT
		return -EINVAL;
	}

	/* read mode reg */
	reg_value = readl(g_pwm_info[channel].channel_mode);

	/* mode */
	ch_config->mode = (reg_value & PWM_MASK_MODE) >> PWM_MODE_SFT;

	/* use_cmp */
	ch_config->use_cmp = ((reg_value & PWM_CMP_EN0_BIT) >> PWM_CMP_EN0_POS)
		| ((reg_value & PWM_CMP_EN1_BIT) >> PWM_CMP_EN1_POS)
		| ((reg_value & PWM_CMP_EN2_BIT) >> PWM_CMP_EN2_POS);

	/* inverse */
	ch_config->inverse =
		((reg_value & PWM_CMP_INV0_BIT) >> PWM_CMP_INV0_POS)
		| ((reg_value & PWM_CMP_INV1_BIT) >> PWM_CMP_INV1_POS)
		| ((reg_value & PWM_CMP_INV2_BIT) >> PWM_CMP_INV2_POS);


	/* read int enable reg */

	reg_value = readl(PWM_INTENSET) >> (channel << 3);

	/* int_count */
	ch_config->interrupt_count =
		((reg_value & PWM_CMP0_CENDSET_BIT) >> PWM_CMP0_CENDSET_POS)
		| ((reg_value & PWM_CMP1_CENDSET_BIT) >> PWM_CMP1_CENDSET_POS)
		| ((reg_value & PWM_CMP2_CENDSET_BIT) >> PWM_CMP2_CENDSET_POS);

	/* interrupt_loop */
	ch_config->interrupt_loop =
		((reg_value & PWM_CMP0_LENDSET_BIT) >> PWM_CMP0_LENDSET_POS)
		| ((reg_value & PWM_CMP1_LENDSET_BIT) >> PWM_CMP1_LENDSET_POS)
		| ((reg_value & PWM_CMP2_LENDSET_BIT) >> PWM_CMP2_LENDSET_POS);

	FNC_EXIT
	return 0;
}
EXPORT_SYMBOL(emxx_pwm_get_channel_config);

/* --------------------------------------------------------------
   function : pwm_set_compare_config
	      regist irq_handler, and compare counter parameters.
   argument : channel
	      <pwm_cmpcnt_state> : setting parameters.
		.cmp
		.delay
		.lead_edge
		.trail_edge
		.total_cycel
		.loop_count
		.cb_info
   return :
   -------------------------------------------------------------- */
int emxx_pwm_set_compare_counter(unsigned char channel,
		struct emxx_pwm_cmpcnt_t *src_cmpcnt)
{
	unsigned int value;
	int err;
	unsigned char cmp;
	struct emxx_pwm_cmpcnt_t *dst_cmpcnt;
	struct emxx_pwm_cb_info_t *src_cb_info;
	struct emxx_pwm_cb_info_t *dst_cb_info;
	struct pwm_info_t *pwm_info_p;
	unsigned long lock_flags;
	FNC_ENTRY

	if (pwm_init_flag) {
		DEBUG_PRINT("pwm initialize error\n");
		FNC_EXIT
		return -EPERM;
	}

	err = check_validity_channel(channel);
	if (err != 0) {
		FNC_EXIT
		return -ENODEV;
	}
	/* check parameter */
	if (NULL == src_cmpcnt) {
		FNC_EXIT
		return -EINVAL;
	}
	err = check_validity_compare_counter(src_cmpcnt->cmp);
	if (err != 0) {
		DEBUG_PRINT("compare counter illegal(cmp=0x%04X)\n",
				src_cmpcnt->cmp);
		FNC_EXIT
		return -ENODEV;
	}

	cmp = src_cmpcnt->cmp;
	pwm_info_p = &g_pwm_info[channel];

	spin_lock_irqsave(&spin_lock, lock_flags);

	/* check state */
	if (PWM_STATE_STOP != pwm_info_p->state) {
		DEBUG_PRINT("device active now. ---> end for unlock.\n");
		spin_unlock_irqrestore(&spin_lock, lock_flags);
		FNC_EXIT return(-EBUSY);
	}
	dst_cmpcnt = &g_pwm_cmpcnt[channel][cmp];

	/* check config (lead_edge < trail_edge <= total_cycle) */
	if ((src_cmpcnt->lead_edge >= src_cmpcnt->trail_edge)
	    || (src_cmpcnt->trail_edge > src_cmpcnt->total_cycle)) {
		DEBUG_PRINT("(lead_edge < trail_edge <="
				"total_cycle) error. ---> end for unlock.\n");
		spin_unlock_irqrestore(&spin_lock, lock_flags);
		FNC_EXIT
		return -EINVAL;
	}

	/* read mode reg */
	value = readl(pwm_info_p->channel_mode);
	/* set reg */
	writel(src_cmpcnt->delay, dst_cmpcnt->delay);
	writel(src_cmpcnt->lead_edge, dst_cmpcnt->lead_edge);
	writel(src_cmpcnt->trail_edge, dst_cmpcnt->trail_edge);
	writel(src_cmpcnt->total_cycle, dst_cmpcnt->total_cycle);
	if (src_cmpcnt->loop_count) {
		writel(src_cmpcnt->loop_count, dst_cmpcnt->loop_count);
		value |= PWM_CMP_ATST0_BIT << (cmp<<2); /* fixed loop */
	} else {
		value &= ~(PWM_CMP_ATST0_BIT << (cmp<<2)); /* infinity loop */
	}
	/* set mode reg */
	writel(value, pwm_info_p->channel_mode);

	/* save callback function & data */
	if (src_cmpcnt->cb_info == NULL) {
		DEBUG_PRINT("callback not use!!! ---> cb_info NULL set\n");
		dst_cmpcnt->cb_info = NULL;
	} else {
		d0b("\n");
		src_cb_info = src_cmpcnt->cb_info;

		if ((src_cb_info->cb_count != NULL)
		    || (src_cb_info->cb_loop != NULL)) {

			d0b("\n");
			/* set callback */
			dst_cmpcnt->cb_info = &g_pwm_cb_info[channel][cmp];
			dst_cb_info = dst_cmpcnt->cb_info;
			src_cb_info = src_cmpcnt->cb_info;

			if (src_cb_info->cb_count != NULL) {
				d0b("\n");
				dst_cb_info->cb_count = src_cb_info->cb_count;
				dst_cb_info->cb_count_data
				    = src_cb_info->cb_count_data;
			} else {
				d0b("\n");
				dst_cb_info->cb_count = NULL;
				dst_cb_info->cb_count_data = NULL;
			}
			if (src_cb_info->cb_loop != NULL) {
				d0b("\n");
				dst_cb_info->cb_loop = src_cb_info->cb_loop;
				dst_cb_info->cb_loop_data
				    = src_cb_info->cb_loop_data;
			} else {
				d0b("\n");
				dst_cb_info->cb_loop = NULL;
				dst_cb_info->cb_loop_data = NULL;
			}
		} else {
			DEBUG_PRINT("ccounter/loop callback nothing!!!"
					" ---> cb_info NULL set\n");
			dst_cmpcnt->cb_info = NULL;
		}
	}
	spin_unlock_irqrestore(&spin_lock, lock_flags);

	FNC_EXIT
	return 0;
}
EXPORT_SYMBOL(emxx_pwm_set_compare_counter);


/* --------------------------------------------------------------
   function : emxx_pwm_get_compare_counter
   explain  : get pwm compare counter workings stat
   argument : channel   : channel number
	      ch_config : PWM compare counter configuration structure pointer
   return   : BIOS_OK         : success
	      BIOS_ERR_PARAM  : parameter error
   -------------------------------------------------------------- */
int emxx_pwm_get_compare_counter(unsigned char channel,
		struct emxx_pwm_cmpcnt_t *pwm_cmpcnt)
{
	int err;
	struct emxx_pwm_cb_info_t *info;
	unsigned int mode_value;
	struct pwm_info_t *pwm_info_p;
	struct emxx_pwm_cmpcnt_t *cmpcnt_reg_p;
	unsigned char cmp;
	FNC_ENTRY

	if (pwm_init_flag) {
		DEBUG_PRINT("pwm initialize error\n");
		FNC_EXIT
		return -EPERM;
	}

	/* check parameter */
	err = check_validity_channel(channel);
	if (err != 0) {
		FNC_EXIT
		return -ENODEV;
	}
	if (pwm_cmpcnt == NULL) {
		DEBUG_PRINT("pwm_cmcnt is NULL!\n");
		FNC_EXIT
		return -EINVAL;
	}
	cmp = pwm_cmpcnt->cmp;

	err = check_validity_compare_counter(cmp);
	if (err != 0) {
		DEBUG_PRINT("compare counter illegal(cmp=0x%04X)\n", cmp);
		FNC_EXIT
		return -ENODEV;
	}

	pwm_info_p = &g_pwm_info[channel];
	cmpcnt_reg_p = &g_pwm_cmpcnt[channel][cmp];

	/* read reg */
	pwm_cmpcnt->delay       = readl(cmpcnt_reg_p->delay);
	pwm_cmpcnt->total_cycle = readl(cmpcnt_reg_p->total_cycle);
	pwm_cmpcnt->lead_edge   = readl(cmpcnt_reg_p->lead_edge);
	pwm_cmpcnt->trail_edge  = readl(cmpcnt_reg_p->trail_edge);
	/* read mode reg */
	mode_value = readl(pwm_info_p->channel_mode);
	if (mode_value &  (PWM_CMP_ATST0_BIT << (cmp<<2)))
		pwm_cmpcnt->loop_count	= readl(cmpcnt_reg_p->loop_count);
	else
		pwm_cmpcnt->loop_count	= 0;

	/* load callback function & data */
	if (NULL != pwm_cmpcnt->cb_info) {
		DEBUG_PRINT("cb_info is not NULL.\n");

		if (NULL == cmpcnt_reg_p->cb_info) {
			pwm_cmpcnt->cb_info->cb_count = NULL;
			pwm_cmpcnt->cb_info->cb_count_data = NULL;
			pwm_cmpcnt->cb_info->cb_loop = NULL;
			pwm_cmpcnt->cb_info->cb_loop_data = NULL;
		} else {
		/* regist irq_interrupt() */
			info = cmpcnt_reg_p->cb_info;
			if (NULL == info->cb_count) {
				pwm_cmpcnt->cb_info->cb_count = NULL;
				pwm_cmpcnt->cb_info->cb_count_data = NULL;
			} else {
				pwm_cmpcnt->cb_info->cb_count = info->cb_count;
				if (NULL == info->cb_count_data)
					pwm_cmpcnt->cb_info->cb_count_data
						= NULL;
				else
					pwm_cmpcnt->cb_info->cb_count_data
						= info->cb_count_data;
			}
			if (NULL == info->cb_loop) {
				pwm_cmpcnt->cb_info->cb_loop = NULL;
				pwm_cmpcnt->cb_info->cb_loop_data = NULL;
			} else {
				pwm_cmpcnt->cb_info->cb_loop = info->cb_loop;
				if (NULL == info->cb_loop_data)
					pwm_cmpcnt->cb_info->cb_loop_data
						= NULL;
				else
					pwm_cmpcnt->cb_info->cb_loop_data
						= info->cb_loop_data;
			}
		}
	}
#if	defined(EMXX_PWM_DEBUG)
	else
		DEBUG_PRINT("cb_info is NULL!!\n");
#endif

	FNC_EXIT
	return 0;
}
EXPORT_SYMBOL(emxx_pwm_get_compare_counter);


/*
 * init
 */
static int __init pwm_init(void)
{
	int ret = 0;
	int i, j;

	FNC_ENTRY

	/* stop clock */
	pwm_clock_stop();

	for (i = 0; i < PWM_MAX_CHANNEL_NUM; i++) {
		for (j = 0; j < PWM_MAX_COUNTER_NUM; j++) {
			g_pwm_cb_info[i][j].cb_count = NULL;
			g_pwm_cb_info[i][j].cb_count_data = NULL;
			g_pwm_cb_info[i][j].cb_loop = NULL;
			g_pwm_cb_info[i][j].cb_loop_data = NULL;
		}
	}

	writel(PWM_INIT_CLOCK, SMU_PWMPWCLKDIV);

	/* supply clock */
	pwm_clock_start();
	/* unreset PWM */
	pwm_dev_unreset();

	/* stop pwclk[0-1] */
	pwm_pwclk_stop(EMXX_PWM_CH0);
	pwm_pwclk_stop(EMXX_PWM_CH1);

	/* regist irq handler */
	ret = request_irq(INT_PWM, pwm_irq_handler, IRQF_DISABLED, PWM_NAME, 0);
	if (ret != 0) {
		printk(KERN_INFO
			"%s(): unable to request IRQ %d for PWM channel \n",
			__func__, INT_PWM);
		/* stop clock */
		pwm_clock_stop();
		FNC_EXIT return(ret);
	}

	disable_irq(INT_PWM);

	pwm_init_flag = 0;

	FNC_EXIT
	return 0;
}

/*
 * exit
 */
static void __exit pwm_exit(void)
{
	FNC_ENTRY

	pwm_init_flag = 1;

	free_irq(INT_PWM, 0);

	/* stop clock */
	pwm_clock_stop();

	FNC_EXIT return;
}


module_init(pwm_init);
module_exit(pwm_exit);

#ifdef EMXX_DEBUG
module_param(debug,  int, 0644);
#endif
MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("'PWM' pulse width moduration device driver.");
