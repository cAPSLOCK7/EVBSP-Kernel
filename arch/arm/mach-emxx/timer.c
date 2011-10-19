/*
 *  File Name       : arch/arm/mach-emxx/time.c
 *  Function        : time
 *  Release Version : Ver 1.02
 *  Release Date    : 2010/09/24
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
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/version.h>
#include <linux/io.h>

#include <asm/div64.h>
#include <asm/mach/time.h>

#include <mach/smu.h>
#include <mach/pmu.h>
#include <mach/timer.h>

#include "timer.h"

static DEFINE_SPINLOCK(timer_spinlock);
static DEFINE_SPINLOCK(wdt_spinlock);

static struct sti_reg_t *sti_timer =
	(struct sti_reg_t *)(IO_ADDRESS(EMXX_STI_BASE));

static struct tm_param_t tm_param[] = {
	/* TIMER_TGn */
	{.usecs = TIMER_DEFAULT_TIME,},
	{.usecs = TIMER_DEFAULT_TIME,},
	{.usecs = TIMER_DEFAULT_TIME,},
	{.usecs = TIMER_DEFAULT_TIME,},
	{.usecs = TIMER_DEFAULT_TIME,},
	{.usecs = TIMER_DEFAULT_TIME,},
	/* TIMER_TIn */
	{.usecs = TIMER_DEFAULT_TIME,},
	{.usecs = TIMER_DEFAULT_TIME,},
	{.usecs = TIMER_DEFAULT_TIME,},
	{.usecs = TIMER_DEFAULT_TIME,},
	/* TIMER_TWn */
	{.usecs = TW_DEFAULT_TIME,},
	{.usecs = TIMER_DEFAULT_TIME,},
	{.usecs = TIMER_DEFAULT_TIME,},
	{.usecs = TIMER_DEFAULT_TIME,},
};

static const struct tm_reg_t tm_reg[] = {
	/* TIMER_TGn */
	{
	 .irq    = INT_TG0,
	 .reg    = (struct timer_reg_t *)IO_ADDRESS(EMXX_TG0_BASE),
	 .clkdev = EMXX_CLK_TG0,
	 .rstdev = EMXX_RST_TG0,
	},
	{
	 .irq    = INT_TG1,
	 .reg    = (struct timer_reg_t *)IO_ADDRESS(EMXX_TG1_BASE),
	 .clkdev = EMXX_CLK_TG1,
	 .rstdev = EMXX_RST_TG1,
	},
	{
	 .irq    = INT_TG2,
	 .reg    = (struct timer_reg_t *)IO_ADDRESS(EMXX_TG2_BASE),
	 .clkdev = EMXX_CLK_TG2,
	 .rstdev = EMXX_RST_TG2,
	},
	{
	 .irq    = INT_TG3,
	 .reg    = (struct timer_reg_t *)IO_ADDRESS(EMXX_TG3_BASE),
	 .clkdev = EMXX_CLK_TG3,
	 .rstdev = EMXX_RST_TG3,
	},
	{
	 .irq    = INT_TG4,
	 .reg    = (struct timer_reg_t *)IO_ADDRESS(EMXX_TG4_BASE),
	 .clkdev = EMXX_CLK_TG4,
	 .rstdev = EMXX_RST_TG4,
	},
	{
	 .irq    = INT_TG5,
	 .reg    = (struct timer_reg_t *)IO_ADDRESS(EMXX_TG5_BASE),
	 .clkdev = EMXX_CLK_TG5,
	 .rstdev = EMXX_RST_TG5,
	},
	/* TIMER_TIn */
	{
	 .irq    = INT_TIMER0,
	 .reg    = (struct timer_reg_t *)IO_ADDRESS(EMXX_TIMER0_BASE),
	 .clkdev = EMXX_CLK_TI0,
	 .rstdev = EMXX_RST_TI0,
	},
	{
	 .irq    = INT_TIMER1,
	 .reg    = (struct timer_reg_t *)IO_ADDRESS(EMXX_TIMER1_BASE),
	 .clkdev = EMXX_CLK_TI1,
	 .rstdev = EMXX_RST_TI1,
	},
	{
	 .irq    = INT_TIMER2,
	 .reg    = (struct timer_reg_t *)IO_ADDRESS(EMXX_TIMER2_BASE),
	 .clkdev = EMXX_CLK_TI2,
	 .rstdev = EMXX_RST_TI2,
	},
	{
	 .irq    = INT_TIMER3,
	 .reg    = (struct timer_reg_t *)IO_ADDRESS(EMXX_TIMER3_BASE),
	 .clkdev = EMXX_CLK_TI3,
	 .rstdev = EMXX_RST_TI3,
	},
	/* TIMER_TWn */
	{
	 .irq     = INT_WDT0,
	 .reg     = (struct timer_reg_t *)IO_ADDRESS(EMXX_WDT0_BASE),
	 .clkdev  = EMXX_CLK_TW0,
	 .rstdev  = EMXX_RST_TW0,
	 .tin_sel = SMU_TWI0TIN_SEL,
	},
	{
	 .irq     = INT_WDT1,
	 .reg     = (struct timer_reg_t *)IO_ADDRESS(EMXX_WDT1_BASE),
	 .clkdev  = EMXX_CLK_TW1,
	 .rstdev  = EMXX_RST_TW1,
	 .tin_sel = SMU_TWI1TIN_SEL,
	},
	{
	 .irq     = INT_WDT2,
	 .reg     = (struct timer_reg_t *)IO_ADDRESS(EMXX_WDT2_BASE),
	 .clkdev  = EMXX_CLK_TW2,
	 .rstdev  = EMXX_RST_TW2,
	 .tin_sel = SMU_TWI2TIN_SEL,
	},
	{
	 .irq     = INT_WDT3,
	 .reg     = (struct timer_reg_t *)IO_ADDRESS(EMXX_WDT3_BASE),
	 .clkdev  = EMXX_CLK_TW3,
	 .rstdev  = EMXX_RST_TW3,
	 .tin_sel = SMU_TWI3TIN_SEL,
	},
	{
	 .irq     = INT_WDT4,
	 .reg     = (struct timer_reg_t *)IO_ADDRESS(EMXX_WDT4_BASE),
	 .clkdev  = EMXX_CLK_TW4,
	 .rstdev  = EMXX_RST_TW4,
	 .tin_sel = SMU_TW4TIN_SEL,
	},
};

/* inline function */
inline int check_validity_channel(unsigned int timer)
{
	if ((TIMER_MIN <= timer) && (timer <= TIMER_MAX))
		return 0;
	else
		return -ENODEV;
}

inline unsigned int get_count_value(unsigned timer, unsigned int usecs)
{
	uint64_t counts;

	if (usecs == TIMER_MAX_COUNT)
		return TIMER_MAX_COUNT;

	if (!(readl(SMU_TGNTIN_SEL) & (0x3 << (timer * 4))))
		counts = (uint64_t)usecs * TIMER_CLOCK_TICK_RATE_PLL3;
	else
		counts = (uint64_t)usecs * TIMER_CLOCK_TICK_RATE_32K;

	do_div(counts, 1000000);
	counts--;

	return (unsigned int)counts;
}
static inline unsigned int
get_tw_count_value(unsigned timer, unsigned int usecs)
{
	uint64_t counts;

	if (timer != TIMER_TW4) {
		if (!(readl(tm_reg[timer].tin_sel) & MASK_TWNTIN_SEL))
			counts = (uint64_t)usecs * TIMER_CLOCK_TICK_RATE_PLL3;
		else
			counts = (uint64_t)usecs * TIMER_CLOCK_TICK_RATE_32K;
	} else {
		if (!(readl(tm_reg[timer].tin_sel) & 0x03))
			counts = (uint64_t)usecs * TIMER_CLOCK_TICK_RATE_PLL3;
		else
			counts = (uint64_t)usecs * TIMER_CLOCK_TICK_RATE_32K;
	}

	do_div(counts, 1000000);
	counts--;

	return (unsigned int)counts;
}


inline unsigned int get_usec_value(unsigned timer, unsigned int counts)
{
	uint64_t usecs;

	usecs = (uint64_t)counts * 1000000;

	if (!(readl(SMU_TGNTIN_SEL) & (0x3 << (timer * 4))))
		do_div(usecs, TIMER_CLOCK_TICK_RATE_PLL3);
	else
		do_div(usecs, TIMER_CLOCK_TICK_RATE_32K);

	return (unsigned int)usecs;
}

static void timer_set_mode(enum clock_event_mode mode,
			   struct clock_event_device *clk)
{
	struct timer_reg_t *reg = tm_reg[TIMER_SYSTEM].reg;

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		reg->tm_clr = TCR_CLR;
		reg->tm_set = TIMER_INTERVAL_PLL3;
		reg->tm_op = TO_EN | TSTART | TM_EN;
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		reg->tm_clr = TCR_CLR;
		reg->tm_set = 0xffffffff;
		reg->tm_op = TO_EN | TSTART | TM_EN;
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		reg->tm_op = TSTOP;
		break;
	}
}

static int timer_set_next_event(unsigned long evt,
		struct clock_event_device *unused)
{
	struct timer_reg_t *reg = tm_reg[TIMER_SYSTEM].reg;

	reg->tm_clr = TCR_CLR;
	reg->tm_set = evt;

	return 0;
}

static struct clock_event_device timer0_clockevent = {
	.name		= "timer0",
	.shift		= 32,
	.features	= CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode	= timer_set_mode,
	.set_next_event	= timer_set_next_event,
	.rating		= 200,
	.cpumask	= cpu_all_mask,
};

static irqreturn_t emxx_system_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &timer0_clockevent;
	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static void __init emxx_clockevents_init(unsigned int timer_irq)
{
	unsigned int cpu = smp_processor_id();

	timer0_clockevent.cpumask = &cpumask_of_cpu(cpu);
	timer0_clockevent.irq = timer_irq;
	timer0_clockevent.mult = div_sc(TIMER_CLOCK_TICK_RATE_PLL3,
		NSEC_PER_SEC, timer0_clockevent.shift);
	timer0_clockevent.max_delta_ns =
		clockevent_delta2ns(0xffffffff, &timer0_clockevent);
	timer0_clockevent.min_delta_ns =
		clockevent_delta2ns(0x7f, &timer0_clockevent);

	clockevents_register_device(&timer0_clockevent);
}

static cycle_t emxx_get_cycles(void)
{
	cycle_t ticks;

	ticks = (cycle_t)(sti_timer->count_h & 0xffff) << 32ULL;
	ticks |= (cycle_t)sti_timer->count_l;
	return ticks;
}

static struct clocksource clocksource_emxx = {
	.name	= "sti",
	.rating	= 250,
	.read	= emxx_get_cycles,
	.mask	= CLOCKSOURCE_MASK(48),
	.shift	= 10,
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void __init emxx_clocksource_init(void)
{
	/* Setup system timer */
	writel(0, SMU_STI_CLKSEL);	/* 32768 */
	emxx_open_clockgate(EMXX_CLK_STI_P | EMXX_CLK_STI_S);
	emxx_unreset_device(EMXX_RST_STI);

	sti_timer->set_h    = 0x80000000;
	sti_timer->set_l    = 0x00000000;
	sti_timer->intenclr = 0x3;
	sti_timer->intffclr = 0x3;

	clocksource_emxx.mult =
		clocksource_hz2mult(32768, clocksource_emxx.shift);
	clocksource_register(&clocksource_emxx);
}

static struct irqaction emxx_system_timer_irq = {
	.name    = "system_timer",
	.flags   = IRQF_DISABLED | IRQF_TIMER,
	.handler = emxx_system_timer_interrupt
};


/*
 * Set up timer interrupt, and return the current time in seconds.
 */
static void __init emxx_init_timer(void)
{
	timer_set_clock(TIMER_INIT);

#ifdef CONFIG_GENERIC_CLOCKEVENTS_BROADCAST
	local_timer_setup();
#endif

	setup_irq(tm_reg[TIMER_SYSTEM].irq, &emxx_system_timer_irq);

	emxx_clocksource_init();
	emxx_clockevents_init(tm_reg[TIMER_SYSTEM].irq);
}

struct sys_timer emxx_timer = {
	.init = emxx_init_timer,
};

static unsigned int tg_tm_op[] = { 0, 0, 0, 0, 0, 0 };

void timer_set_clock(unsigned int mode)
{
	int i;
	unsigned int count;
	unsigned int tm_delay;
	struct timer_reg_t *reg;
	static unsigned int timer_pmu_flag;

	/* delay value set */
	tm_delay = TIMER_DELAY(TIMER_CLOCK_TICK_RATE_32K);

	switch (mode) {
	case TIMER_SUSPEND:
		/* close clockgate */
		emxx_close_clockgate(tm_reg[TIMER_SYSTEM].clkdev);
		emxx_close_clockgate(tm_reg[TIMER_WDT].clkdev);
		emxx_close_clockgate(tm_reg[TIMER_DSP].clkdev);

		timer_pmu_flag = emxx_get_clockgate(tm_reg[TIMER_PMU].clkdev);
		if (timer_pmu_flag)
			emxx_close_clockgate(tm_reg[TIMER_PMU].clkdev);

		for (i = 0; i < TIMER_TG_MAX_NUM; i++) {
			if (tg_tm_op[i] == 1)
				emxx_close_clockgate(
					tm_reg[TIMER_TG0 + i].clkdev);
		}
		break;
	case TIMER_RESUME:
		emxx_open_clockgate(tm_reg[TIMER_SYSTEM].clkdev);
		emxx_open_clockgate(tm_reg[TIMER_WDT].clkdev);
		emxx_open_clockgate(tm_reg[TIMER_DSP].clkdev);

		if (timer_pmu_flag)
			emxx_open_clockgate(tm_reg[TIMER_PMU].clkdev);

		for (i = 0; i < TIMER_TG_MAX_NUM; i++) {
			if (tg_tm_op[i] == 1)
				emxx_open_clockgate(
					tm_reg[TIMER_TG0 + i].clkdev);
		}
		break;
	case TIMER_INIT:
		emxx_open_clockgate(EMXX_CLK_TIM_P);

		emxx_open_clockgate(tm_reg[TIMER_SYSTEM].clkdev);
		emxx_open_clockgate(tm_reg[TIMER_DSP].clkdev);
		emxx_open_clockgate(tm_reg[TIMER_PMU].clkdev);
		emxx_unreset_device(tm_reg[TIMER_SYSTEM].rstdev);
		emxx_unreset_device(tm_reg[TIMER_DSP].rstdev);
		emxx_unreset_device(tm_reg[TIMER_PMU].rstdev);

		for (i = 0; i < TIMER_TG_MAX_NUM; i++)
			emxx_unreset_device(tm_reg[TIMER_TG0 + i].rstdev);

		reg = tm_reg[TIMER_SYSTEM].reg;
		reg->tm_op = TSTOP;
		reg->tm_clr = TCR_CLR;

		reg = tm_reg[TIMER_WDT].reg;
		reg->tm_op = TSTOP;
		reg->tm_clr = TCR_CLR;

		reg = tm_reg[TIMER_DSP].reg;
		reg->tm_op = TSTOP;
		reg->tm_clr = TCR_CLR;
		reg->tm_set = TIMER_INTERVAL_DSP;

		reg = tm_reg[TIMER_PMU].reg;
		reg->tm_op = TSTOP;
		reg->tm_clr = TCR_CLR;
		emxx_close_clockgate(tm_reg[TIMER_PMU].clkdev);

		/* select TIN clock */
		writel(TINTIN_SEL_PLL3 | TWNTIN_SEL_32K, SMU_TWI0TIN_SEL);
		writel(TINTIN_SEL_PLL3 | TWNTIN_SEL_PLL3, SMU_TWI1TIN_SEL);
		writel(TINTIN_SEL_32K | TWNTIN_SEL_PLL3, SMU_TWI2TIN_SEL);
		writel(TINTIN_SEL_32K | TWNTIN_SEL_PLL3, SMU_TWI3TIN_SEL);
		writel(TINTIN_SEL_32K, SMU_TW4TIN_SEL);
		writel(TGNTIN_SEL_PLL3, SMU_TGNTIN_SEL);

		/* DIVTIMTIN Register set */
		writel(SMU_PLLSEL_PLL3 | SMU_DIV(40), SMU_TIMCLKDIV);

		for (i = 0; i < TIMER_TG_MAX_NUM; i++) {
			count = get_count_value(i,
				tm_param[TIMER_TG0 + i].usecs);
			reg  = tm_reg[TIMER_TG0 + i].reg;
			reg->tm_set = count;
		}
		break;
	default:
		printk(KERN_INFO "%s(): set clock error mode = %d.\n",
			__func__, mode);
		break;
	}
}


int emxx_timer_start(unsigned int timer)
{
	int ret = 0;
	unsigned long flags;
	struct timer_reg_t *reg;

	ret = check_validity_channel(timer);
	if (ret != 0)
		return ret;

	reg = tm_reg[TIMER_TG0 + timer].reg;

	spin_lock_irqsave(&timer_spinlock, flags);

	emxx_open_clockgate(tm_reg[TIMER_TG0 + timer].clkdev);
	tg_tm_op[timer] = 1;

	/* Timer start */
	reg->tm_op = TSTART | TM_EN | TO_EN;

	spin_unlock_irqrestore(&timer_spinlock, flags);

	return 0;
}
EXPORT_SYMBOL(emxx_timer_start);

int emxx_timer_stop(unsigned int timer)
{
	int ret = 0;
	unsigned int tm_delay;
	unsigned long flags;
	struct timer_reg_t *reg;

	ret = check_validity_channel(timer);
	if (ret != 0)
		return ret;

	/* delay value set */
	tm_delay = TIMER_DELAY(TIMER_CLOCK_TICK_RATE_PLL3);

	reg = tm_reg[TIMER_TG0 + timer].reg;

	spin_lock_irqsave(&timer_spinlock, flags);

	/* Timer stop */
	reg->tm_op = TSTOP;

	udelay(tm_delay);

	emxx_close_clockgate(tm_reg[TIMER_TG0 + timer].clkdev);
	tg_tm_op[timer] = 0;

	spin_unlock_irqrestore(&timer_spinlock, flags);

	return 0;
}
EXPORT_SYMBOL(emxx_timer_stop);

int emxx_timer_set_period(unsigned int timer, unsigned int usecs)
{
	int ret = 0, regval = 0;
	unsigned long	flags;
	unsigned int tm_delay;
	struct timer_reg_t *reg;

	ret = check_validity_channel(timer);
	if (ret != 0)
		return ret;

	if ((usecs < TIMER_MIN_USECS) || (TIMER_MAX_USECS < usecs))
		return -EINVAL;

	/* 4clk x2 wait value */
	tm_delay = TIMER_DELAY(TIMER_CLOCK_TICK_RATE_PLL3) * 2;
	reg = tm_reg[TIMER_TG0 + timer].reg;

	spin_lock_irqsave(&timer_spinlock, flags);

	if (reg->tm_op & TSTART) {
		spin_unlock_irqrestore(&timer_spinlock, flags);
		return -EBUSY;
	}

	emxx_open_clockgate(tm_reg[TIMER_TG0 + timer].clkdev);
	udelay(tm_delay);

	tm_param[TIMER_TG0 + timer].usecs = usecs;

	/* set counter */
	regval = get_count_value(timer, usecs);
	reg->tm_set = regval;

	emxx_close_clockgate(tm_reg[TIMER_TG0 + timer].clkdev);
	spin_unlock_irqrestore(&timer_spinlock, flags);

	return 0;
}
EXPORT_SYMBOL(emxx_timer_set_period);

int emxx_timer_get_period(unsigned int timer, unsigned int *usecs)
{
	int ret = 0;

	ret = check_validity_channel(timer);
	if (ret != 0)
		return ret;

	if (usecs == NULL)
		return -EINVAL;

	*usecs = tm_param[TIMER_TG0 + timer].usecs;

	return 0;
}
EXPORT_SYMBOL(emxx_timer_get_period);

int emxx_timer_get_usecs(unsigned int timer, unsigned int *usecs)
{
	unsigned int regval = 0;
	int ret = 0;
	struct timer_reg_t *reg;

	ret = check_validity_channel(timer);
	if (ret != 0)
		return ret;

	if (usecs == NULL)
		return -EINVAL;

	reg = tm_reg[TIMER_TG0 + timer].reg;
	regval = reg->tm_rcr;
	*usecs = get_usec_value(timer, regval);

	return 0;
}
EXPORT_SYMBOL(emxx_timer_get_usecs);

int emxx_timer_register_cb(unsigned int timer, char *devname,
			irqreturn_t(*cb) (int, void *), void *dev_id)
{
	unsigned int irq = 0;
	int ret = 0;
	struct timer_reg_t *reg;

	ret = check_validity_channel(timer);
	if (ret != 0)
		return ret;

	reg = tm_reg[TIMER_TG0 + timer].reg;

	if (reg->tm_op & TSTART)
		return -EBUSY;

	irq = tm_reg[TIMER_TG0 + timer].irq;
	ret = request_irq(irq, cb, IRQF_SHARED, devname, dev_id);
	if (ret != 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL(emxx_timer_register_cb);

int emxx_timer_unregister_cb(unsigned int timer, void *dev_id)
{
	unsigned int irq = 0;
	int ret = 0;
	struct timer_reg_t *reg;

	ret = check_validity_channel(timer);
	if (ret != 0)
		return ret;

	reg = tm_reg[TIMER_TG0 + timer].reg;

	if (reg->tm_op & TSTART)
		return -EBUSY;

	irq = tm_reg[TIMER_TG0 + timer].irq;
	free_irq(irq, dev_id);

	return 0;
}
EXPORT_SYMBOL(emxx_timer_unregister_cb);


/*
 * WatchDog Timer Function
 */
int emxx_wdt_set_timeout(unsigned int usecs)
{
	int ret = 0, val_tm_op, regval;
	unsigned int tm_delay;
	struct timer_reg_t *reg = tm_reg[TIMER_WDT].reg;

	if ((usecs < TW_MIN_USECS) || (TW_MAX_USECS < usecs))
		return -EINVAL;
	tm_delay = TIMER_DELAY(TW_CLOCK_TICK_RATE) * 2;

	spin_lock(&wdt_spinlock);

	/* get tm_op */
	val_tm_op = reg->tm_op;

	/* timer stop */
	reg->tm_op = TSTOP;

	/* set counter */
	regval = get_tw_count_value(TIMER_WDT, usecs);

	udelay(tm_delay);
	reg->tm_set = regval;

	/* timer start (restore tm_op) */
	reg->tm_op = val_tm_op;

	spin_unlock(&wdt_spinlock);

	return ret;
}
EXPORT_SYMBOL(emxx_wdt_set_timeout);

/* Clear watchdog timer. */
void emxx_wdt_ping(void)
{
	unsigned int tm_delay = TIMER_DELAY(TW_CLOCK_TICK_RATE);
	struct timer_reg_t *reg = tm_reg[TIMER_WDT].reg;

	spin_lock(&wdt_spinlock);
	reg->tm_clr = TCR_CLR;
	udelay(tm_delay);
	spin_unlock(&wdt_spinlock);
}
EXPORT_SYMBOL(emxx_wdt_ping);

/* Start watchdog timer counting. */
void emxx_wdt_enable(void)
{
	struct timer_reg_t *reg = tm_reg[TIMER_WDT].reg;

	spin_lock(&wdt_spinlock);
	reg->tm_op = TM_EN | TSTART | TO_EN;
	spin_unlock(&wdt_spinlock);
}
EXPORT_SYMBOL(emxx_wdt_enable);

/* Stop watchdog timer counting. */
void emxx_wdt_disable(void)
{
	unsigned int tm_delay = TIMER_DELAY(TW_CLOCK_TICK_RATE);
	struct timer_reg_t *reg = tm_reg[TIMER_WDT].reg;

	spin_lock(&wdt_spinlock);
	reg->tm_op = TSTOP;
	udelay(tm_delay);
	spin_unlock(&wdt_spinlock);
}
EXPORT_SYMBOL(emxx_wdt_disable);

/* Setup watchdog timer. */
void emxx_wdt_setup(void)
{
	unsigned int regval;
	unsigned int tm_delay;
	struct timer_reg_t *reg = tm_reg[TIMER_WDT].reg;

	tm_delay = TIMER_DELAY(TW_CLOCK_TICK_RATE) * 2;
	spin_lock(&wdt_spinlock);

	writel(readl(SMU_WDT_INT_RESET) | 0x01003003, SMU_WDT_INT_RESET);

	emxx_open_clockgate(EMXX_CLK_TIM_P);
	emxx_open_clockgate(tm_reg[TIMER_WDT].clkdev);
	emxx_unreset_device(tm_reg[TIMER_WDT].rstdev);

	/* stop timer for sure. */
	reg->tm_op = TSTOP;

	/* set counter */
	regval = get_tw_count_value(TIMER_WDT, tm_param[TIMER_WDT].usecs);
	udelay(tm_delay);
	reg->tm_set = regval;

	spin_unlock(&wdt_spinlock);
}
EXPORT_SYMBOL(emxx_wdt_setup);

#ifdef CONFIG_SMP

/* Dummy Function */
void local_timer_interrupt(void)
{
}

#ifdef CONFIG_LOCAL_TIMERS

/* Dummy Function */
int local_timer_ack(void)
{
	return 1;
}

static void local_timer_set_mode(enum clock_event_mode mode,
				 struct clock_event_device *clk)
{
	struct timer_reg_t *reg = tm_reg[TIMER_SYSTEM2].reg;

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		reg->tm_clr = TCR_CLR;
		reg->tm_set = TIMER_INTERVAL_PLL3;
		reg->tm_op = TO_EN | TSTART | TM_EN;
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		reg->tm_clr = TCR_CLR;
		reg->tm_set = 0xffffffff;
		reg->tm_op = TO_EN | TSTART | TM_EN;
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		reg->tm_op = TSTOP;
		break;
	}
}

static int local_timer_set_next_event(unsigned long evt,
				struct clock_event_device *unused)
{
	struct timer_reg_t *reg = tm_reg[TIMER_SYSTEM2].reg;

	reg->tm_clr = TCR_CLR;
	reg->tm_set = evt;

	return 0;
}

static struct clock_event_device timer1_clockevent = {
	.name		= "timer1",
	.shift		= 32,
	.features	= CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode	= local_timer_set_mode,
	.set_next_event	= local_timer_set_next_event,
	.rating		= 250,
};

static irqreturn_t emxx_system_timer1_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *clk = &timer1_clockevent;
	clk->event_handler(clk);

	return IRQ_HANDLED;
}

static struct irqaction emxx_system_timer1_irq = {
	.name    = "system_timer1",
	.flags   = IRQF_DISABLED | IRQF_TIMER,
	.handler = emxx_system_timer1_interrupt
};

/*
 * Setup the local clock events for a CPU.
 */
void __cpuinit local_timer_setup(void)
{
	unsigned int cpu = smp_processor_id();
	struct clock_event_device *clk;
	unsigned long flags;
	void __iomem *int_reg;
	u32 val, int_shift, irq;

	if (cpu == 1) {
		clk = &timer1_clockevent;
		irq = tm_reg[TIMER_SYSTEM2].irq;

		clk->cpumask = &cpumask_of_cpu(cpu);
		clk->irq     = irq;
		clk->mult    = div_sc(TIMER_CLOCK_TICK_RATE_PLL3,
				NSEC_PER_SEC, clk->shift);
		clk->max_delta_ns = clockevent_delta2ns(0xffffffff, clk);
		clk->min_delta_ns = clockevent_delta2ns(0x7f, clk);

		emxx_open_clockgate(tm_reg[TIMER_SYSTEM2].clkdev);
		emxx_unreset_device(tm_reg[TIMER_SYSTEM2].rstdev);

		local_irq_save(flags);
		setup_irq(irq, &emxx_system_timer1_irq);
		irq_desc[irq].cpu = cpu;

		/* Setting interrupt */
		int_reg = __io_address(EMXX_INTA_DIST_BASE) +
				GIC_DIST_TARGET + (INT_TIMER1 & ~3);
		int_shift = (irq % 4) * 8;
		val = readl(int_reg) & ~(0xff << int_shift);
		val |= 1 << (cpu + int_shift);
		writel(val, int_reg);
		local_irq_restore(flags);

		clockevents_register_device(clk);
	}
}

#ifdef CONFIG_HOTPLUG_CPU
/*
 * take a local timer down
 */
void local_timer_stop(void)
{
	struct timer_reg_t *reg = tm_reg[TIMER_SYSTEM2].reg;

	reg->tm_op = TSTOP;
	udelay(TIMER_DELAY(TIMER_CLOCK_TICK_RATE_PLL3));

	emxx_close_clockgate(tm_reg[TIMER_SYSTEM2].clkdev);
}
#endif

#else	/* CONFIG_LOCAL_TIMERS */

static void dummy_timer_set_mode(enum clock_event_mode mode,
				 struct clock_event_device *clk)
{
}

void __cpuinit local_timer_setup(void)
{
	unsigned int cpu = smp_processor_id();
	struct clock_event_device *clk = &per_cpu(local_clockevent, cpu);

	clk->name	= "dummy_timer";
	clk->features	= CLOCK_EVT_FEAT_DUMMY;
	clk->rating	= 200;
	clk->set_mode	= dummy_timer_set_mode;
	clk->broadcast	= smp_timer_broadcast;
	clk->cpumask	= &cpumask_of_cpu(cpu);

	clockevents_register_device(clk);
}

#endif	/* !CONFIG_LOCAL_TIMERS */

#endif	/* CONFIG_SMP */
