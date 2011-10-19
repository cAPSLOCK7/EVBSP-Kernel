
/*
 *  File Name	    : linux/arch/arm/mach-emxx/pm_pmu.c
 *  Function	    : pm_pmu
 *  Release Version : Ver 1.14
 *  Release Date    : 2011/01/18
 *
 * Copyright (C) 2010-2011 Renesas Electronics Corporation
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

char const pmu_version[] =
		"PMU sequencer ver1.00l and Compiled "__DATE__" "__TIME__"";
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <mach/pmu.h>
#include <mach/pwc.h>
#include <mach/smu.h>
#include <mach/pm.h>
#include <mach/timer.h>
#include <asm/cacheflush.h>
#include <asm/hardware/gic.h>
#include <asm/hardware/cache-l2x0.h>

#include "pm_pmu.h"
#include "timer.h"

#include <mach/spi.h>


/* PM flags */
#define PM_CONTROL_MODE	1	/* 1: lowpower, 0: SPI */

/* flag for debug */
/* #define DEBUG_LED */
/* #define IDLE_DEBUG_LOG */
/* #define PM_DEBUG */


#if defined(IDLE_DEBUG_LOG) || defined(PM_DEBUG)
#define DPRINTK(format, args...) printk(KERN_INFO "PMU_DEBUG: " format, ##args)
#else
#define DPRINTK(format, args...)
#endif

#ifdef PM_DEBUG
static unsigned int pm_try_count;
static unsigned int pmu_count;
#endif

static struct register_state_t reg_state;
static unsigned int wdt_op_reg;

/*
 * INT mask and unmask
 * ----
 *  Don't call at idle sleep.
 */
static int pmu_int_mask(int mask)
{
	struct intc_state *state = &reg_state.intc;
	unsigned int max_irq, i;

	max_irq = (INT_LAST + 1);
	switch (mask) {
	case PMU_INT_MASK_SAVE_AND_MASK:
		DPRINTK("mask = PMU_INT_MASK_SAVE_AND_MASK\n");

		/* save all interrupt of INTA_DIST(32-) */
		state->dist_ien1 = readl(GIC_032_IEN);		/* 32-63 */
		state->dist_ien2 = readl(GIC_064_IEN);		/* 64-95 */
		state->dist_ien3 = readl(GIC_096_IEN);		/* 96-127 */
		state->dist_ien4 = readl(GIC_128_IEN);		/* 128-159 */
		state->dist_ien5 = readl(GIC_160_IEN);		/* 160-191 */

		/* disable all interrupts of INTA_DIST(32-) */
		for (i = 32; i < max_irq; i += 32)
			writel(MASK_INT_ALL, GIC_000_IDS + i * 4 / 32);

		writel(RESUME_INT_1, GIC_096_IEN);
		/* set sec int, include MEMC, ABx and AFS */
		writel(SEC_ERR_INT, GIC_160_IEN);

		break;
	case PMU_INT_MASK_RESTORE:
		/* disable all interrupts of INTA_DIST(from 32 to INT_LAST) */
		for (i = 32; i < max_irq; i += 32)
			writel(MASK_INT_ALL, GIC_000_IDS + i * 4 / 32);

		/* restore INTA_DIST(32-) */
		writel(state->dist_ien1, GIC_032_IEN);
		writel(state->dist_ien2, GIC_064_IEN);
		writel(state->dist_ien3, GIC_096_IEN);
		writel(state->dist_ien4, GIC_128_IEN);
		writel(state->dist_ien5, GIC_160_IEN);
		break;
	case PMU_INT_ALLMASK:
		/* Disable all interrupts. */
		for (i = 0; i < max_irq; i += 32)
			writel(MASK_INT_ALL, GIC_000_IDS + i * 4 / 32);

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * GPIO mask and unmask
 *
 */
static int pmu_gpio_mask(int flag)
{
	struct gpio_state *gpio = &reg_state.gpio;
	struct pwc_state *pwc = &reg_state.pwc;

	/* GPIO/PowerIC_GPIO Interrupt Enable/Disable */
	switch (flag) {
	case EMXX_PMU_CLK_FULLSPEED:
		/* mask all gpio interrupt */
		outl(MASK_GPIO_ALL, GIO_000_IDS);
		outl(MASK_GPIO_ALL, GIO_032_IDS);
		outl(MASK_GPIO_ALL, GIO_064_IDS);
		outl(MASK_GPIO_ALL, GIO_096_IDS);
		outl(MASK_GPIO_ALL, GIO_128_IDS);
		/* restore all gpio interrupt */
		outl(gpio->ien0, GIO_000_IEN);
		outl(gpio->ien1, GIO_032_IEN);
		outl(gpio->ien2, GIO_064_IEN);
		outl(gpio->ien3, GIO_096_IEN);
		outl(gpio->ien4, GIO_128_IEN);

		pwc_reg_write(DA9052_IRQMASKA_REG, pwc->mask_a); /* 10 */
		pwc_reg_write(DA9052_IRQMASKB_REG, pwc->mask_b); /* 11 */
		pwc_reg_write(DA9052_IRQMASKC_REG, pwc->mask_c); /* 12 */
		pwc_reg_write(DA9052_IRQMASKD_REG, pwc->mask_d); /* 13 */

		break;
	case EMXX_PMU_CLK_SLEEP:
	case EMXX_PMU_CLK_DEEPSLEEP:
		/* read all gpio interrupt */
		gpio->ien0 = inl(GIO_000_IIM);
		gpio->ien1 = inl(GIO_032_IIM);
		gpio->ien2 = inl(GIO_064_IIM);
		gpio->ien3 = inl(GIO_096_IIM);
		gpio->ien4 = inl(GIO_128_IIM);

		/* mask all gpio interrupt */
		outl(MASK_GPIO_ALL, GIO_000_IDS);
		outl(MASK_GPIO_ALL, GIO_032_IDS);
		outl(MASK_GPIO_ALL, GIO_064_IDS);
		outl(MASK_GPIO_ALL, GIO_096_IDS);
		outl(MASK_GPIO_ALL, GIO_128_IDS);

		/* unmask PMIC to GPIO interrupt(GPIO00) */
		outl(GPIO_INT_PWRIC, GIO_000_IEN);

		/* read pmic interrupt */
		pwc_reg_read(DA9052_IRQMASKA_REG, &pwc->mask_a);
		pwc_reg_read(DA9052_IRQMASKB_REG, &pwc->mask_b);
		pwc_reg_read(DA9052_IRQMASKC_REG, &pwc->mask_c);
		pwc_reg_read(DA9052_IRQMASKD_REG, &pwc->mask_d);
		/* mask all pmic interrupt */
		pwc_reg_write(DA9052_IRQMASKA_REG, 0xff);	/* 10 */
		pwc_reg_write(DA9052_IRQMASKB_REG, 0xff);	/* 11 */
		pwc_reg_write(DA9052_IRQMASKC_REG, 0xff);	/* 12 */
		pwc_reg_write(DA9052_IRQMASKD_REG, 0xff);	/* 13 */

		/* Clear all pmic interrupt flag */
		pwc_reg_write(DA9052_EVENTA_REG, 0xff);		/* 5 */
		pwc_reg_write(DA9052_EVENTB_REG, 0xff);		/* 6 */
		pwc_reg_write(DA9052_EVENTC_REG, 0xff);		/* 7 */
		pwc_reg_write(DA9052_EVENTD_REG, 0xff);

		break;
	case EMXX_PMU_CLK_POWEROFF:
		/* mask all gpio interrupt */
		outl(MASK_GPIO_ALL, GIO_000_IDS);
		outl(MASK_GPIO_ALL, GIO_032_IDS);
		outl(MASK_GPIO_ALL, GIO_064_IDS);
		outl(MASK_GPIO_ALL, GIO_096_IDS);
		outl(MASK_GPIO_ALL, GIO_128_IDS);

		/* mask all pmic interrupt */
		pwc_write(DA9052_IRQMASKA_REG, 0xff, 0xff);
		pwc_write(DA9052_IRQMASKB_REG, 0xff, 0xff);
		pwc_write(DA9052_IRQMASKC_REG, 0xff, 0xff);
		pwc_write(DA9052_IRQMASKD_REG, 0xff, 0xff);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*
 * Set PMU Command Sequence
 *  - lowpwer_flag
 *    1: lowpoer, 0: SPI
 */
static void pmu_set_command_sequence(unsigned int sleep_flag)
{
	unsigned tmp_pc;
	unsigned int *pmu_cmd_adr;
#ifdef CONFIG_MACH_EMEV
	int l2_off_flag = 1;	/* 1:exec OFF/ON, 0: not exec */
#endif
	unsigned int spi0_back_addr;
	unsigned int vdd_back_addr;
	unsigned int value;

	/* RAM */
	if ((readl(PMU_SLEEPFLAG) == sleep_flag)
		 && (readl(PMU_IDLEFLAG) == emxx_sleep_while_idle)) {
		/* no need code change. */
		return;
	}
	writel(sleep_flag, PMU_SLEEPFLAG);
	writel(emxx_sleep_while_idle, PMU_IDLEFLAG);


#ifdef CONFIG_MACH_EMEV
	if (!((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_DEEPSLEEP))
		l2_off_flag = 0;
#endif

	/*********************************************************************/
	/* PMU_PC_MAIN(Normal to Economy)                                    */
	/*********************************************************************/
	pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_MAIN);

	/* 1000 */
	PCMD_INT_MASK(PMU_INT_MASK);
#ifdef CONFIG_MACH_EMEV
	if (l2_off_flag) {
		/* macro use {} */
		PCMD_SUBROUTINE_START(PMU_PC_SUB_L2OFF); /* L2RAM OFF */
	} else {
		/* macro use {} */
		PCMD_NOP();
	}
#endif
	/* 0x1008: here is return position */
	PCMD_REG_READ(SMU_MACRO, SMU_CMD_POWER_STATUS, PMU_REGB);
	PCMD_REG_WRITE2(PMU_MACRO, PMU_RAM_SMU_POWERSTATUS, PMU_REGB);
	PCMD_SUBROUTINE_START(PMU_PC_SUB_SWOFF0); /* PowerSW OFF #0 */
#ifdef CONFIG_MACH_EMEV
	PCMD_SUBROUTINE_START(PMU_PC_SUB_SWOFF1); /* PowerSw OFF #1 */
#endif
	/* Economy Mode */
	PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_CLK_MODE_SEL, 0x0005);
	PCMD_NOP();
	PCMD_SMU_READY_WAIT(PMU_HIGH_LEVEL);

	/* jump to Economy/Sleep/DeepSleep */
	switch (sleep_flag & EMXX_PMU_CLK_MASK) {
	case EMXX_PMU_CLK_ECONOMY:
		PCMD_AJUMP(PMU_PC_ECONOMY);
		break;
	case EMXX_PMU_CLK_SLEEP:
		PCMD_AJUMP(PMU_PC_SLEEP);
		break;
	case EMXX_PMU_CLK_DEEPSLEEP:
		PCMD_AJUMP(PMU_PC_DEEP);
		break;
	default:
		PCMD_AJUMP(PMU_PC_ON1);
		break;
	}

	/****************************************************************/
	/* PMU_PC_ECONOMY						*/
	/* Use mode: Economy.						*/
	/****************************************************************/
	if ((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_ECONOMY) {
		pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_ECONOMY);
#ifdef CONFIG_MACH_EMEV
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PLL1CTRL1, 0x00FF);
#endif
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PLL2CTRL1, 0x00FF);
		PCMD_WDT_STOP();
		PCMD_TRIG_WAIT(1, PMU_TRIG_INT | PMU_TRIG_P1WAKEUP_HIGH);
#ifdef CONFIG_MACH_EMEV
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PLL1CTRL1, 0x0000);
#endif
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PLL2CTRL1, 0x0000);
		PCMD_WDT_RESTART();
		PCMD_AJUMP(PMU_PC_ON1);
	}

	/****************************************************************/
	/* PMU_PC_SLEEP							*/
	/* Use mode: Sleep						*/
	/****************************************************************/
	if ((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_SLEEP) {
		pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_SLEEP);

		PCMD_TRIG_WAIT(1, PMU_TRIG_INT
		 | PMU_TRIG_P1WAKEUP_LOW | PMU_TRIG_P1WAKEUP_HIGH);
		PCMD_AND(0x00000038, PMU_REGA);
		PCMD_CMP2(0x00000008, PMU_REGA);
		PCMD_BRANCH(PMU_PC_ON1, PMU_BNE);		/* 0x1200 */
#ifdef CONFIG_MACH_EMEV
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PLL1CTRL1, 0x00FF);
#endif
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PLL2CTRL1, 0x00FF);
		PCMD_SUBROUTINE_START(PMU_PC_SUB_SWOFF2);	/* 0x1400 */
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_CLK_MODE_SEL, 0x00000007);
								/* SleepMode */
		PCMD_NOP();
		PCMD_SMU_READY_WAIT(PMU_HIGH_LEVEL);
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PLL3CTRL1, 0x000000FF);
		PCMD_WDT_STOP();
		PCMD_TRIG_WAIT(1, PMU_TRIG_INT | PMU_TRIG_P1WAKEUP_HIGH);
		PCMD_WDT_RESTART();
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PLL3CTRL1, 0x00000000);
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_CLK_MODE_SEL, 0x00000005);
								/* EcoMode */
#ifdef CONFIG_MACH_EMEV
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PLL1CTRL1, 0x0000);
#endif
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PLL2CTRL1, 0x0000);
		PCMD_SMU_READY_WAIT(PMU_HIGH_LEVEL);
		PCMD_AJUMP(PMU_PC_ON1);
	}

	/****************************************************************/
	/* PMU_PC_DEEP							*/
	/* Use mode: DeepSleep						*/
	/****************************************************************/
	if ((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_DEEPSLEEP) {
		pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_DEEP);

		/* 1100 */
		PCMD_TRIG_WAIT(1, PMU_TRIG_INT
		 | PMU_TRIG_P1WAKEUP_LOW | PMU_TRIG_P1WAKEUP_HIGH);
		PCMD_AND(0x00000038, PMU_REGA);
		PCMD_CMP2(0x00000008, PMU_REGA);
		PCMD_BRANCH(PMU_PC_ON1, PMU_BNE);		/* to 0x1200 */
#ifdef CONFIG_MACH_EMEV
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PLL1CTRL1, 0x00FF);
#endif
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PLL2CTRL1, 0x00FF);
		/* PowerDown by SPI. if setting is LOWPWR then change to "NOP"*/
		vdd_back_addr = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr)
			+ PCMD_AJUMP_SIZE;
#if PM_CONTROL_MODE
		PCMD_AJUMP(PMU_PC_SUB_VDD_LPW);			/* to 0x2400 */
#else
		PCMD_AJUMP(PMU_PC_SUB_VDD_SPI);			/* to 0x1F00 */
#endif
		PCMD_SUBROUTINE_START(PMU_PC_SUB_SWOFF2);	/* to 0x1400 */
		PCMD_SUBROUTINE_START(PMU_PC_SUB_SWOFF3);	/* to 0x14A0 */
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_AUTO_MODE_EN, 0x00000021);
		PCMD_WDT_STOP();
		tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
		PCMD_REG_READ(SMU_MACRO, SMU_CMD_SEQ_BUSY, PMU_REGB);
		PCMD_AND(0x00000FFF, PMU_REGB);
		PCMD_CMP2(0x00000000, PMU_REGB);
		PCMD_BRANCH(tmp_pc, PMU_BNE);	/* to 0x1140 */
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_CLK_MODE_SEL, 0x00000008);
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_OSC0CTRL1, 0x000000FF);
#ifdef CONFIG_MACH_EMEV
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_OSC1CTRL1, 0x000000FF);
#endif
#if PM_CONTROL_MODE
		/* case lowpower */
#ifdef CONFIG_MACH_EMEV
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_INTAGCLKCTRL, 0x00000000);
#elif defined(CONFIG_MACH_EMGR)
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_S0_DUMMY_REG4, 0x00000001);
#endif
#else
		/* case spi */
		PCMD_NOP();
#endif
		PCMD_SMU_READY_WAIT(PMU_HIGH_LEVEL);
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PLL3CTRL1, 0x000000FF);
		PCMD_AJUMP(PMU_PC_PWRCNT);
	}

	/****************************************************************/
	/* PMU_PC_PWRCNT						*/
	/* Use mode: DeepSleep						*/
	/****************************************************************/
	if ((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_DEEPSLEEP) {
		pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_PWRCNT);
#if PM_CONTROL_MODE
		/* case lowpower */
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_LOWPWR, 0x00000001);
		PCMD_CYCLE_WAIT(0x0F, PMU_WAIT_INT_CONTINUE);
#ifdef CONFIG_MACH_EMEV
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_INTAGCLKCTRL, 0x00000007);
#elif defined(CONFIG_MACH_EMGR)
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_S0_DUMMY_REG4, 0x00000000);
#endif
#else
		/* case SPI power down. */
		PCMD_NOP();
		PCMD_NOP();
		PCMD_NOP();
		PCMD_NOP();
		PCMD_NOP();
#endif
		PCMD_TRIG_WAIT(1, PMU_TRIG_INT | PMU_TRIG_P1WAKEUP_HIGH
		 | PMU_INT_MASK);
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_AUTO_MODE_EN, 0x00000001);
#ifdef CONFIG_MACH_EMEV
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_OSC1CTRL1, 0x0000);
#endif
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PLL2CTRL1, 0x0000);
#ifdef CONFIG_MACH_EMEV
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PLL1CTRL1, 0x0000);
#endif
		PCMD_SMU_READY_WAIT(PMU_HIGH_LEVEL);
		PCMD_AJUMP(PMU_PC_PWRCNT2);
	}

	/****************************************************************/
	/* PMU_PC_PWRCNT2						*/
	/* Use mode: DeepSleep						*/
	/****************************************************************/
	if ((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_DEEPSLEEP) {
		pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_PWRCNT2);
		PCMD_WDT_RESTART();
		PCMD_AJUMP(PMU_PC_ON1);
	}


	/****************************************************************/
	/* PMU_PC_ON1							*/
	/* Use mode: All						*/
	/****************************************************************/
	/* 1200 */
	pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_ON1);
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
	PCMD_REG_READ(SMU_MACRO, SMU_CMD_SEQ_BUSY, PMU_REGB);
	PCMD_AND(0x00000FFF, PMU_REGB);
	PCMD_CMP2(0x00000000, PMU_REGB);
	PCMD_BRANCH(tmp_pc, PMU_BNE);		/* to 0x1200 */
	PCMD_SUBROUTINE_START(PMU_PC_SUB_SETPARA);	/* to 0x1A00 */
#ifdef CONFIG_MACH_EMEV
	if (l2_off_flag) {
		PCMD_SUBROUTINE_START(PMU_PC_SUB_L2ON);	/* to 0x1D70 */
	} else {
		/* macro use "{}"*/
		PCMD_NOP();
	}
#endif
	if (((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_DEEPSLEEP)) {
		PCMD_SUBROUTINE_START(PMU_PC_SUB_SWON3);/* to 0x1800 */
	} else {
		/* economy, sleep */
		PCMD_NOP();
	}
	if (((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_ECONOMY)) {
		/* economy */
		PCMD_NOP();
	} else {
		PCMD_SUBROUTINE_START(PMU_PC_SUB_SWON2);	/* to 0x1750 */
	}
#ifdef CONFIG_MACH_EMEV
	PCMD_SUBROUTINE_START(PMU_PC_SUB_SWON1);	/* to 0x1700 */
#endif
	PCMD_SUBROUTINE_START(PMU_PC_SUB_SWON0);	/* to 0x1500 */
	if ((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_DEEPSLEEP) {
		/* to 0x1850 */
		spi0_back_addr = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr)
			+ PCMD_AJUMP_SIZE;
		PCMD_AJUMP(PMU_PC_SUB_SPI0);
	} else {
		PCMD_REG_WRITE(PMU_MACRO, PMU_RAM_RUNCHECK, 0x00000001);
	}
	PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_MEMC_HAND_SHAKE_FAKE, 0x00000000);
	PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_CLK_MODE_SEL, 0x00000001);
	PCMD_NOP();
	PCMD_SMU_READY_WAIT(PMU_HIGH_LEVEL);
	PCMD_WDT_STOP();
	PCMD_TRIG_WAIT(1, PMU_TRIG_INT | PMU_TRIG_P1WAKEUP_LOW);
	PCMD_WDT_RESTART();
	PCMD_AND(0x00000038, PMU_REGA);
	PCMD_CMP2(0x00000008, PMU_REGA);
	PCMD_BRANCH(0x1004, PMU_BEQ);	/* 0x1004@PMU_PC_MAIN*/

#ifdef CONFIG_MACH_EMEV
	/* [BIT0] 0: async, 1: sync */
	if (!(readl(SMU_CPUCLK_SYNCSET) & 0x01)) {
		tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
		PCMD_REG_READ(SMU_MACRO, SMU_CMD_PLL_STATUS, PMU_REGB);
		PCMD_AND(0x00000001, PMU_REGB)
		PCMD_CMP2(0x00000001, PMU_REGB);
		PCMD_BRANCH(tmp_pc, PMU_BNE);
	}
#elif defined(CONFIG_MACH_EMGR)
	if (!(readl(SMU_CPUCLK_SYNCSET) & 0x01)) {
		tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
		PCMD_REG_READ(SMU_MACRO, SMU_CMD_PLL_STATUS, PMU_REGB);
		PCMD_AND(0x00000010, PMU_REGB)
		PCMD_CMP2(0x00000010, PMU_REGB);
		PCMD_BRANCH(tmp_pc, PMU_BNE);
	}
#endif
	PCMD_AJUMP(PMU_PC_ON2);		/* to 0x12C0 */

	/****************************************************************/
	/* PMU_PC_ON2							*/
	/* Use mode: All						*/
	/****************************************************************/
	pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_ON2);

	PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PMU_INTCTRL, 0x00000001);
	PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PMU_INTCTRL, 0x00000000);
	PCMD_PMU_END();

	/****************************************************************/
	/* PMU_PC_SUB_SWOFF0						*/
	/* Use mode: All						*/
	/****************************************************************/
	pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_SUB_SWOFF0);
#if 0
	PCMD_RMW(SMU_MACRO, SMU_CMD_PD_SWON, 0x00000000, 0x00000001);
#else
	PCMD_NOP();
	PCMD_NOP();
	PCMD_NOP();
#endif
	PCMD_RMW(SMU_MACRO, SMU_CMD_PV_SWON, 0x00000000, 0x00000001);
	PCMD_RMW(SMU_MACRO, SMU_CMD_PR_SWON, 0x00000000, 0x00000001);
	PCMD_RMW(SMU_MACRO, SMU_CMD_PG_SWON, 0x00000000, 0x00000001);
#ifdef CONFIG_MACH_EMEV
	PCMD_RMW(SMU_MACRO, SMU_CMD_P2_SWON, 0x00000000, 0x00000001);
#endif
	PCMD_RMW(SMU_MACRO, SMU_CMD_PU_SWON, 0x00000000, 0x00000001);
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
	PCMD_REG_READ(SMU_MACRO, SMU_CMD_SEQ_BUSY, PMU_REGB);
#ifdef CONFIG_MACH_EMEV
	PCMD_AND(0x000007E0, PMU_REGB);
#elif defined(CONFIG_MACH_EMGR)
	PCMD_AND(0x00000760, PMU_REGB);
#endif
	PCMD_CMP2(0x00000000, PMU_REGB);
	PCMD_BRANCH(tmp_pc, PMU_BNE);	/* to 0x1348 */
	PCMD_RFS();


	/****************************************************************/
	/* PMU_PC_SUB_SWOFF1						*/
	/* Use mode: All						*/
	/****************************************************************/
#ifdef CONFIG_MACH_EMEV
	pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_SUB_SWOFF1);

	PCMD_RMW(SMU_MACRO, SMU_CMD_P1_SWON, 0x00000000, 0x00000001);
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
	PCMD_REG_READ(SMU_MACRO, SMU_CMD_SEQ_BUSY, PMU_REGB);
	PCMD_AND(0x00000010, PMU_REGB);
	PCMD_CMP2(0x00000000, PMU_REGB);
	PCMD_BRANCH(tmp_pc, PMU_BNE);	/* to 0x13AC */
	PCMD_RFS();
#endif

	/****************************************************************/
	/* PMU_PC_SUB_SWOFF2						*/
	/* Use mode: Sleep, DeepSleep					*/
	/****************************************************************/
	if (!((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_ECONOMY)) {

		/* 1400 */
		pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_SUB_SWOFF2);
		PCMD_REG_READ(MEMC_MACRO, MEMC_CMD_DDR_STATE8, PMU_REGB);
		PCMD_AND(0x00000003, PMU_REGB);
		PCMD_CMP2(0x00000003, PMU_REGB);
		PCMD_BRANCH(PMU_PC_SUB_SWOFF2, PMU_BNE);	/* to up */
#ifdef CONFIG_MACH_EMEV
		PCMD_REG_READ(SMU_MACRO, SMU_CMD_PM_SWON, PMU_REGB);
		PCMD_AND(0x00000100, PMU_REGB);
		PCMD_CMP2(0x00000100, PMU_REGB);
		tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr)
			+ PCMD_BRANCH_SIZE
			+ PCMD_NOP_SIZE;
		PCMD_BRANCH(tmp_pc, PMU_BNE);	/* to up */
		PCMD_NOP();
		PCMD_RMW(SMU_MACRO, SMU_CMD_PM_SWON, 0x00000000, 0x00000001);
#elif defined(CONFIG_MACH_EMGR)
		PCMD_REG_READ(SMU_MACRO, SMU_CMD_P1_SWON, PMU_REGB);
		PCMD_AND(0x00000100, PMU_REGB);
		PCMD_CMP2(0x00000100, PMU_REGB);
		tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr)
			+ PCMD_BRANCH_SIZE
			+ PCMD_NOP_SIZE;
		PCMD_BRANCH(tmp_pc, PMU_BNE);	/* to up */
		PCMD_NOP();
		PCMD_RMW(SMU_MACRO, SMU_CMD_CPUGCLKCTRL, 0x00000000,
		 0x00000004);
		PCMD_RMW(SMU_MACRO, SMU_CMD_P1_SWON, 0x00000000, 0x00000001);
#endif
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_MEMC_HAND_SHAKE_FAKE,
		 0x00000001);
		tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
		PCMD_REG_READ(SMU_MACRO, SMU_CMD_SEQ_BUSY, PMU_REGB);
#ifdef CONFIG_MACH_EMEV
		PCMD_AND(0x00000004, PMU_REGB);
#elif defined(CONFIG_MACH_EMGR)
		PCMD_AND(0x00000010, PMU_REGB);
#endif
		PCMD_CMP2(0x00000000, PMU_REGB);
		PCMD_BRANCH(tmp_pc, PMU_BNE);	/* to up */
#ifdef CONFIG_MACH_EMEV
		PCMD_RMW(SMU_MACRO, SMU_CMD_PL_SWON, 0x00000000, 0x00000001);
		tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
		PCMD_REG_READ(SMU_MACRO, SMU_CMD_SEQ_BUSY, PMU_REGB);
		PCMD_AND(0x00000008, PMU_REGB);
		PCMD_CMP2(0x00000000, PMU_REGB);
		PCMD_BRANCH(tmp_pc, PMU_BNE);	/* to up */
#endif
		PCMD_RFS();
	}


	/****************************************************************/
	/* PMU_PC_SUB_SWOFF3						*/
	/* Use mode: Sleep, DeepSleep					*/
	/****************************************************************/
	if (!((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_ECONOMY)) {
		pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_SUB_SWOFF3);

		PCMD_RMW(SMU_MACRO, SMU_CMD_P0_SWON, 0x00000000, 0x00000001);
		PCMD_RFS();
	}

	/****************************************************************/
	/* PMU_PC_SUB_SWON0						*/
	/* Use mode: All						*/
	/****************************************************************/
	pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_SUB_SWON0);

	/* 1500 */
	PCMD_REG_READ(PMU_MACRO, PMU_RAM_SMU_POWERSTATUS, PMU_REGB);
	PCMD_AND(0x00003000, PMU_REGB);
	PCMD_CMP2(0x00000000, PMU_REGB);
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr)
		+ PCMD_BRANCH_SIZE
		+ PCMD_RMW_SIZE
		+ PCMD_REG_READ_SIZE
		+ PCMD_AND_SIZE
		+ PCMD_CMP2_SIZE
		+ PCMD_BRANCH_SIZE;
	PCMD_BRANCH(tmp_pc, PMU_BNE);	/* jump over */
	PCMD_RMW(SMU_MACRO, SMU_CMD_PU_SWON, 0x00000001, 0x00000001);
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
	PCMD_REG_READ(SMU_MACRO, SMU_CMD_SEQ_BUSY, PMU_REGB);
	PCMD_AND(0x00000040, PMU_REGB);
	PCMD_CMP2(0x00000000, PMU_REGB);
	PCMD_BRANCH(tmp_pc, PMU_BNE);	/* to up */
	PCMD_REG_READ(PMU_MACRO, PMU_RAM_SMU_POWERSTATUS, PMU_REGB);
	PCMD_AND(0x0000C000, PMU_REGB);
	PCMD_CMP2(0x00000000, PMU_REGB);
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr)
		+ PCMD_BRANCH_SIZE
		+ PCMD_RMW_SIZE
		+ PCMD_REG_READ_SIZE
		+ PCMD_AND_SIZE
		+ PCMD_CMP2_SIZE
		+ PCMD_BRANCH_SIZE;
	PCMD_BRANCH(tmp_pc, PMU_BNE);	/* jump over */
#ifdef CONFIG_MACH_EMEV
	PCMD_RMW(SMU_MACRO, SMU_CMD_P2_SWON, 0x00000001, 0x00000001);
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
	PCMD_REG_READ(SMU_MACRO, SMU_CMD_SEQ_BUSY, PMU_REGB);
	PCMD_AND(0x00000080, PMU_REGB);
	PCMD_CMP2(0x00000000, PMU_REGB);
	PCMD_BRANCH(tmp_pc, PMU_BNE);	/* to up */
	PCMD_REG_READ(PMU_MACRO, PMU_RAM_SMU_POWERSTATUS, PMU_REGB);
	PCMD_AND(0x00030000, PMU_REGB);
	PCMD_CMP2(0x00000000, PMU_REGB);
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr)
		+ PCMD_BRANCH_SIZE
		+ PCMD_RMW_SIZE
		+ PCMD_REG_READ_SIZE
		+ PCMD_AND_SIZE
		+ PCMD_CMP2_SIZE
		+ PCMD_BRANCH_SIZE;
	PCMD_BRANCH(tmp_pc, PMU_BNE);	/* jump over */
#endif
	PCMD_RMW(SMU_MACRO, SMU_CMD_PG_SWON, 0x00000001, 0x00000001);
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
	PCMD_REG_READ(SMU_MACRO, SMU_CMD_SEQ_BUSY, PMU_REGB);
	PCMD_AND(0x00000100, PMU_REGB);
	PCMD_CMP2(0x00000000, PMU_REGB);
	PCMD_BRANCH(tmp_pc, PMU_BNE);	/* to up */
	PCMD_REG_READ(PMU_MACRO, PMU_RAM_SMU_POWERSTATUS, PMU_REGB);
	PCMD_AND(0x00300000, PMU_REGB);
	PCMD_CMP2(0x00000000, PMU_REGB);
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr)
		+ PCMD_BRANCH_SIZE
		+ PCMD_RMW_SIZE
		+ PCMD_REG_READ_SIZE
		+ PCMD_AND_SIZE
		+ PCMD_CMP2_SIZE
		+ PCMD_BRANCH_SIZE;
	PCMD_BRANCH(tmp_pc, PMU_BNE);	/* jump over */
	PCMD_RMW(SMU_MACRO, SMU_CMD_PR_SWON, 0x00000001, 0x00000001);
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
	PCMD_REG_READ(SMU_MACRO, SMU_CMD_SEQ_BUSY, PMU_REGB);
	PCMD_AND(0x00000400, PMU_REGB);
	PCMD_CMP2(0x00000000, PMU_REGB);
	PCMD_BRANCH(tmp_pc, PMU_BNE);	/* to up */
	PCMD_REG_READ(PMU_MACRO, PMU_RAM_SMU_POWERSTATUS, PMU_REGB);
	PCMD_AND(0x000C0000, PMU_REGB);
	PCMD_CMP2(0x00000000, PMU_REGB);
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr)
		+ PCMD_BRANCH_SIZE
		+ PCMD_RMW_SIZE
		+ PCMD_REG_READ_SIZE
		+ PCMD_AND_SIZE
		+ PCMD_CMP2_SIZE
		+ PCMD_BRANCH_SIZE;
	PCMD_BRANCH(tmp_pc, PMU_BNE);	/* to up */
	PCMD_RMW(SMU_MACRO, SMU_CMD_PV_SWON, 0x00000001, 0x00000001);
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
	PCMD_REG_READ(SMU_MACRO, SMU_CMD_SEQ_BUSY, PMU_REGB);
	PCMD_AND(0x00000200, PMU_REGB);
	PCMD_CMP2(0x00000000, PMU_REGB);
	PCMD_BRANCH(tmp_pc, PMU_BNE);	/* to 0x1614 */
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
	PCMD_REG_READ(SMU_MACRO, SMU_CMD_PLL_STATUS, PMU_REGB);
#ifdef CONFIG_MACH_EMEV
	PCMD_AND(0x00000001, PMU_REGB);
	PCMD_CMP2(0x00000001, PMU_REGB);
#elif defined(CONFIG_MACH_EMGR)
	PCMD_AND(0x00000010, PMU_REGB);	/* PLL2 */
	PCMD_CMP2(0x00000010, PMU_REGB);
#endif
	PCMD_BRANCH(tmp_pc, PMU_BNE);	/* to up */

#if 0		/* for DSP */
	PCMD_REG_READ(PMU_MACRO, PMU_RAM_SMU_POWERSTATUS, PMU_REGB);
	PCMD_AND(0x00000C00, PMU_REGB);
	PCMD_CMP2(0x00000000, PMU_REGB);
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr)
		+ PCMD_BRANCH_SIZE
		+ PCMD_RMW_SIZE
		+ PCMD_REG_READ_SIZE
		+ PCMD_AND_SIZE
		+ PCMD_CMP2_SIZE
		+ PCMD_BRANCH_SIZE;
	PCMD_BRANCH(tmp_pc, PMU_BNE);	/* jump over */
	PCMD_RMW(SMU_MACRO, SMU_CMD_PD_SWON, 0x00000001, 0x00000001);
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
	PCMD_REG_READ(SMU_MACRO, SMU_CMD_SEQ_BUSY, PMU_REGB);
	PCMD_AND(0x00000020, PMU_REGB);
	PCMD_CMP2(0x00000000, PMU_REGB);
	PCMD_BRANCH(tmp_pc, PMU_BNE);	/* to 0x165C */
	PCMD_RFS();
#else
	PCMD_RFS();
#endif


	/****************************************************************/
	/* PMU_PC_SUB_SWON1						*/
	/* Use mode: All						*/
	/****************************************************************/
#ifdef CONFIG_MACH_EMEV
	pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_SUB_SWON1);

	PCMD_RMW(SMU_MACRO, SMU_CMD_P1_SWON, 0x00000001, 0x00000001);
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
	PCMD_REG_READ(SMU_MACRO, SMU_CMD_SEQ_BUSY, PMU_REGB);
	PCMD_AND(0x00000010, PMU_REGB);
	PCMD_CMP2(0x00000000, PMU_REGB);
	PCMD_BRANCH(tmp_pc, PMU_BNE);	/* 0x170C@PMU_PC_SUB_SWON1 */
	PCMD_RFS();
#endif

	/****************************************************************/
	/* PMU_PC_SUB_SWON2						*/
	/* Use mode: All						*/
	/****************************************************************/
	pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_SUB_SWON2);

#ifdef CONFIG_MACH_EMEV
	PCMD_RMW(SMU_MACRO, SMU_CMD_PL_SWON, 0x00000001, 0x00000001);
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
	PCMD_REG_READ(SMU_MACRO, SMU_CMD_SEQ_BUSY, PMU_REGB);
	PCMD_AND(0x00000008, PMU_REGB);
	PCMD_CMP2(0x00000000, PMU_REGB);
	PCMD_BRANCH(tmp_pc, PMU_BNE);		/* to 0x175C */
	PCMD_RMW(SMU_MACRO, SMU_CMD_PM_SWON, 0x00000001, 0x00000001);
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
	PCMD_REG_READ(SMU_MACRO, SMU_CMD_SEQ_BUSY, PMU_REGB);
	PCMD_AND(0x00000004, PMU_REGB);
	PCMD_CMP2(0x00000000, PMU_REGB);
	PCMD_BRANCH(tmp_pc, PMU_BNE);		/* to 0x1780 */
	PCMD_RFS();
#elif defined(CONFIG_MACH_EMGR)
	PCMD_RMW(SMU_MACRO, SMU_CMD_P1_SWON, 0x00000001, 0x00000001);
	tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
	PCMD_REG_READ(SMU_MACRO, SMU_CMD_SEQ_BUSY, PMU_REGB);
	PCMD_AND(0x00000010, PMU_REGB);
	PCMD_CMP2(0x00000000, PMU_REGB);
	PCMD_BRANCH(tmp_pc, PMU_BNE);		/* to 0x175C */
	PCMD_RMW(SMU_MACRO, SMU_CMD_CPUGCLKCTRL, 0x00000004, 0x00000004);
	PCMD_RFS();
#endif

	/****************************************************************/
	/* PMU_PC_SUB_SWON3						*/
	/* Use mode : Sleep, DeepSleep					*/
	/****************************************************************/
	if (!((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_ECONOMY)) {
		pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_SUB_SWON3);

		PCMD_RMW(SMU_MACRO, SMU_CMD_P0_SWON, 0x00000001, 0x00000001);
		tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr);
		PCMD_REG_READ(SMU_MACRO, SMU_CMD_SEQ_BUSY, PMU_REGB);
		PCMD_AND(0x00000001, PMU_REGB);
		PCMD_CMP2(0x00000000, PMU_REGB);
		PCMD_BRANCH(tmp_pc, PMU_BNE);		/* to up */
		PCMD_RFS();
	}

	/****************************************************************/
	/* PMU_PC_SUB_SPI0						*/
	/* Use mode : DeepSleep						*/
	/****************************************************************/
	if ((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_DEEPSLEEP) {
		pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_SUB_SPI0);

		/* 1850 */
		PCMD_REG_READ(SMU_MACRO, SMU_CMD_P0_SWON, PMU_REGB);
		PCMD_AND(0x00000100, PMU_REGB);
		PCMD_CMP2(0x00000100, PMU_REGB);
		tmp_pc = PMU_VIRADDR_TO_PC((unsigned int)pmu_cmd_adr)
			+ PCMD_BRANCH_SIZE
			+ PCMD_REG_WRITE_SIZE;
		PCMD_BRANCH(tmp_pc, PMU_BNE);		/* jump over */
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_USAIS0_RSTCTRL, 0x00000003);
		PCMD_SUBROUTINE_START(PMU_PC_SUB_SPI1);		/* to 0x1940 */
#ifdef CONFIG_MACH_EMEV
		PCMD_REG_WRITE(SIO0_MACRO, SIO_CMD_SP0_TX_DATA, 0x000058AA);
		PCMD_SP0_WRITE(SIO_CMD_SP0_CONTROL, 0x00000009);
		PCMD_REG_WRITE(SIO0_MACRO, SIO_CMD_SP0_FFCLR, 0x000000FF);
#endif
		/* R64 : ex)1300mA : 0xF8 */
		PCMD_REG_READ(PMU_MACRO, PMU_RAM_PWIC_ISET, PMU_REGB);
		PCMD_EXOR(0x00008000, PMU_REGB);
		PCMD_REG_WRITE2(SIO0_MACRO, SIO_CMD_SP0_TX_DATA, PMU_REGB);
		PCMD_SP0_WRITE(SIO_CMD_SP0_CONTROL, 0x00000009);
		PCMD_REG_WRITE(SIO0_MACRO, SIO_CMD_SP0_FFCLR, 0x000000FF);
		/* R62 : ex)1300mA : 0xDF */
		PCMD_REG_READ(PMU_MACRO, PMU_RAM_PWIC_CHGBUCK, PMU_REGB);
		PCMD_EXOR(0x00007C00, PMU_REGB);
		PCMD_REG_WRITE2(SIO0_MACRO, SIO_CMD_SP0_TX_DATA, PMU_REGB);
		PCMD_SP0_WRITE(SIO_CMD_SP0_CONTROL, 0x00000009);
		PCMD_REG_WRITE(SIO0_MACRO, SIO_CMD_SP0_FFCLR, 0x000000FF);
#ifdef CONFIG_MACH_EMEV
		/* R46 : ex)1.3V : 0x60 */
		PCMD_REG_READ(PMU_MACRO, PMU_RAM_PWIC_BUCKCORE, PMU_REGB);
		PCMD_EXOR(0x00005C00, PMU_REGB);
		PCMD_REG_WRITE2(SIO0_MACRO, SIO_CMD_SP0_TX_DATA, PMU_REGB);
		PCMD_SP0_WRITE(SIO_CMD_SP0_CONTROL, 0x00000009);
		PCMD_REG_WRITE(SIO0_MACRO, SIO_CMD_SP0_FFCLR, 0x000000FF);
		/* wait for change R44 */
		PCMD_CYCLE_WAIT(0x35F, PMU_WAIT_INT_CONTINUE);
		PCMD_REG_WRITE(SIO0_MACRO, SIO_CMD_SP0_TX_DATA, 0x00007801);
		PCMD_SP0_WRITE(SIO_CMD_SP0_CONTROL, 0x00000009);
		PCMD_REG_WRITE(SIO0_MACRO, SIO_CMD_SP0_FFCLR, 0x000000FF);
#endif
		PCMD_REG_WRITE(PMU_MACRO, PMU_RAM_RUNCHECK, 0x00000001);
		PCMD_SUBROUTINE_START(PMU_PC_SUB_SPI2);		/* 0x1980 */
		PCMD_TIMERWAIT(1, PMU_WAIT_INT_CONTINUE);
		PCMD_AJUMP(spi0_back_addr);	/* return to 0x1234(@F_ON) */
	}

	/****************************************************************/
	/* PMU_PC_SUB_SPI1						*/
	/* Use mode : All(SPI)						*/
	/* Use mode : DeepSleep(lowpwr)					*/
	/****************************************************************/
#if PM_CONTROL_MODE
	if ((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_DEEPSLEEP) {
#else
	{
#endif
		pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_SUB_SPI1);

		/* 1940 */
		PCMD_REG_READ(SMU_MACRO, SMU_CMD_USIAS0GCLKCTRL, PMU_REGB);
		PCMD_REG_WRITE2(PMU_MACRO, PMU_RAM_SMU_USIAS0GCLKCTRL,
		 PMU_REGB);
		PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_USIAS0GCLKCTRL, 0x00000007);
		PCMD_REG_WRITE(SIO0_MACRO, SIO_CMD_SP0_FFCLR, 0x000000FF);
		PCMD_REG_WRITE(SIO0_MACRO, SIO_CMD_SP0_ENSET, 0x000000FF);
		PCMD_REG_WRITE(SIO0_MACRO, SIO_CMD_SP0_POL, 0x00007004);
		PCMD_REG_WRITE(SIO0_MACRO, SIO_CMD_SP0_MODE, 0x00000F04);
		PCMD_RFS();
	}

	/****************************************************************/
	/* PMU_PC_SUB_SPI2						*/
	/* Use mode : All						*/
	/****************************************************************/
	pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_SUB_SPI2);

	/* 1980 */
	PCMD_REG_READ(PMU_MACRO, PMU_RAM_SMU_USIAS0GCLKCTRL, PMU_REGB);
	PCMD_REG_WRITE2(SMU_MACRO, SMU_CMD_USIAS0GCLKCTRL, PMU_REGB);
	PCMD_RFS();

	/*********************************************************************/
	/* PMU_PC_SUB_SETPARA */
	/*********************************************************************/
	pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_SUB_SETPARA);

	value = 0x00000085;

	PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_P0_PWSW_PARA, value);
#ifdef CONFIG_MACH_EMEV
	PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PM_PWSW_PARA, value);
	PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PL_PWSW_PARA, value);
#endif
	PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_P1_PWSW_PARA, value);
	PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PD_PWSW_PARA, value);
	PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PU_PWSW_PARA, value);
#ifdef CONFIG_MACH_EMEV
	PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_P2_PWSW_PARA, value);
#endif
	PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PG_PWSW_PARA, value);
	PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PR_PWSW_PARA, value);
	PCMD_REG_WRITE(SMU_MACRO, SMU_CMD_PV_PWSW_PARA, value);
	PCMD_RFS();

	/*********************************************************************/
	/* PMU_PC_SUB_L2OFF */
	/*********************************************************************/
#ifdef CONFIG_MACH_EMEV
	if (l2_off_flag) {
		pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_SUB_L2OFF);

		PCMD_RMW(SMU_MACRO, SMU_CMD_CPU_PWSW_CTRL, 0x0000, 0x0004);
		PCMD_RMW(SMU_MACRO, SMU_CMD_CPU_PWSW_L2RAM, 0x0080, 0x0080);
		PCMD_RMW(SMU_MACRO, SMU_CMD_CPU_PWSW_L2RAM, 0x0040, 0x0040);
		PCMD_RMW(SMU_MACRO, SMU_CMD_CPU_PWSW_L2RAM, 0x0020, 0x0020);
		PCMD_RMW(SMU_MACRO, SMU_CMD_CPU_PWSW_L2RAM, 0x0010, 0x0010);
		PCMD_RMW(SMU_MACRO, SMU_CMD_CPU_PWSW_L2RAM, 0x0008, 0x0008);
		PCMD_RMW(SMU_MACRO, SMU_CMD_CPU_PWSW_L2RAM, 0x0004, 0x0004);
		PCMD_RMW(SMU_MACRO, SMU_CMD_CPU_PWSW_L2RAM, 0x0002, 0x0002);
		PCMD_RMW(SMU_MACRO, SMU_CMD_CPU_PWSW_L2RAM, 0x0001, 0x0001);
		PCMD_RFS();
	}
#endif

	/*********************************************************************/
	/* PMU_PC_SUB_L2ON */
	/*********************************************************************/
#ifdef CONFIG_MACH_EMEV
	if (l2_off_flag) {
		pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_SUB_L2ON);

	    /* 1D70 */
	    PCMD_RMW(SMU_MACRO, SMU_CMD_CPU_PWSW_L2RAM, 0x00000000, 0x00000001);
	    PCMD_CYCLE_WAIT(0x10, PMU_WAIT_INT_END);
	    PCMD_RMW(SMU_MACRO, SMU_CMD_CPU_PWSW_L2RAM, 0x00000000, 0x00000002);
	    PCMD_CYCLE_WAIT(0x10, PMU_WAIT_INT_END);
	    PCMD_RMW(SMU_MACRO, SMU_CMD_CPU_PWSW_L2RAM, 0x00000000, 0x00000004);
	    PCMD_CYCLE_WAIT(0x10, PMU_WAIT_INT_END);
	    PCMD_RMW(SMU_MACRO, SMU_CMD_CPU_PWSW_L2RAM, 0x00000000, 0x00000008);
	    PCMD_CYCLE_WAIT(0x10, PMU_WAIT_INT_END);
	    PCMD_RMW(SMU_MACRO, SMU_CMD_CPU_PWSW_L2RAM, 0x00000000, 0x00000010);
	    PCMD_CYCLE_WAIT(0x10, PMU_WAIT_INT_END);
	    PCMD_RMW(SMU_MACRO, SMU_CMD_CPU_PWSW_L2RAM, 0x00000000, 0x00000020);
	    PCMD_CYCLE_WAIT(0x10, PMU_WAIT_INT_END);
	    PCMD_RMW(SMU_MACRO, SMU_CMD_CPU_PWSW_L2RAM, 0x00000000, 0x00000040);
	    PCMD_CYCLE_WAIT(0x10, PMU_WAIT_INT_END);
	    PCMD_RMW(SMU_MACRO, SMU_CMD_CPU_PWSW_L2RAM, 0x00000000, 0x00000080);
	    PCMD_RMW(SMU_MACRO, SMU_CMD_CPU_PWSW_CTRL, 0x00000004, 0x00000004);
	    PCMD_RFS();
	}
#endif

	/****************************************************************/
	/* PMU_PC_SUB_VDD_SPI						*/
	/* Use mode: DeepSleep						*/
	/****************************************************************/
#if !PM_CONTROL_MODE
	if ((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_DEEPSLEEP) {
		pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_SUB_VDD_SPI);

		/* 1F00 */
		PCMD_SUBROUTINE_START(PMU_PC_SUB_SPI1);		/* to 0x1940 */
#ifdef CONFIG_MACH_EMEV
		/* R44 <- sleep_mode */
		PCMD_REG_WRITE(SIO0_MACRO, SIO_CMD_SP0_TX_DATA, 0x00005898);
		PCMD_SP0_WRITE(SIO_CMD_SP0_CONTROL, 0x00000009);
		PCMD_REG_WRITE(SIO0_MACRO, SIO_CMD_SP0_FFCLR, 0x000000FF);
#endif
		PCMD_REG_WRITE(SIO0_MACRO, SIO_CMD_SP0_TX_DATA, 0x00005CCA);
		PCMD_SP0_WRITE(SIO_CMD_SP0_CONTROL, 0x00000009);
		PCMD_REG_WRITE(SIO0_MACRO, SIO_CMD_SP0_FFCLR, 0x000000FF);
		PCMD_REG_WRITE(SIO0_MACRO, SIO_CMD_SP0_TX_DATA, 0x00001E6D);
		PCMD_SP0_WRITE(SIO_CMD_SP0_CONTROL, 0x00000009);
		PCMD_REG_WRITE(SIO0_MACRO, SIO_CMD_SP0_FFCLR, 0x000000FF);
		PCMD_SUBROUTINE_START(PMU_PC_SUB_SPI2);		/* to 0x1980 */
		PCMD_AJUMP(vdd_back_addr);			/* to 0x112C */
	}
#endif

	/****************************************************************/
	/* PMU_PC_SUB_VDD_LPW						*/
	/* Use mode: DeepSleep						*/
	/****************************************************************/
	if ((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_DEEPSLEEP) {
		pmu_cmd_adr = PMU_PC_TO_VIRADDR(PMU_PC_SUB_VDD_LPW);

		/* 2400 */
		PCMD_SUBROUTINE_START(PMU_PC_SUB_SPI1);		/* to 0x1940 */
		PCMD_REG_WRITE(SIO0_MACRO, SIO_CMD_SP0_TX_DATA, 0x00005898);
		PCMD_SP0_WRITE(SIO_CMD_SP0_CONTROL, 0x00000009);
		PCMD_REG_WRITE(SIO0_MACRO, SIO_CMD_SP0_FFCLR, 0x000000FF);
		PCMD_SUBROUTINE_START(PMU_PC_SUB_SPI2);		/* to 0x1980 */
		PCMD_AJUMP(vdd_back_addr);			/* to 0x112C */
	}

	return;
}


#if PM_CONTROL_MODE
static void pw_ic_lowpower(void)
{
#ifdef CONFIG_MACH_EMGR
	unsigned char buckperi, ldo3;

	pwc_reg_read(DA9052_BUCKPERI_REG, &buckperi);	/* 49 */
	pwc_reg_read(DA9052_LDO3_REG, &ldo3);		/* 52 */
#endif

	pwc_reg_write(DA9052_GPIO0809_REG, 0x08);	/* 25 */
	pwc_reg_write(DA9052_ID01_REG, 0x90);		/* 29 */
	pwc_reg_write(DA9052_BUCKCORE_REG, 0xCA);	/* 46 */
	pwc_reg_write(DA9052_BUCKMEM_REG, 0xE3);	/* 48 */

#ifdef CONFIG_MACH_EMEV
	pwc_reg_write(DA9052_BUCKPERI_REG, 0xDB);	/* 49 */
#elif defined(CONFIG_MACH_EMGR)
	if (buckperi & 0x40) {
		/* 49 */
		pwc_reg_write(DA9052_BUCKPERI_REG, buckperi | 0x80);
	}
#endif
	pwc_reg_write(DA9052_LDO2_REG, 0xD4);		/* 51 */
#ifdef CONFIG_MACH_EMEV
	pwc_reg_write(DA9052_LDO3_REG, 0xFF);		/* 52 */
#elif defined(CONFIG_MACH_EMGR)
	if (ldo3 & 0x40) {
		/* 52 */
		pwc_reg_write(DA9052_LDO3_REG, ldo3 | 0x80);
	}
#endif
	pwc_reg_write(DA9052_LDO4_REG, 0xED);		/* 53 */
	pwc_reg_write(DA9052_LDO6_REG, 0xC6);		/* 55 */
	pwc_reg_write(DA9052_LFO7_REG, 0xE1);		/* 56 */
	if (reg_state.pwc.ldo8 & 0x40) {
		/* 57 */
		pwc_reg_write(DA9052_LDO8_REG, reg_state.pwc.ldo8 | 0x80);
	}
	if (reg_state.pwc.ldo9 & 0x40) {
		/* 58 */
		pwc_reg_write(DA9052_LDO9_REG, reg_state.pwc.ldo9 | 0x80);
	}
	pwc_reg_write(DA9052_LDO10_REG, 0xCC);		/* 59 */
	pwc_reg_write(DA9052_BUCKB_REG, 0x88);		/* 45 */
	pwc_reg_write(DA9052_PDDIS_REG, 0x00);		/* 18 */
	pwc_reg_write(DA9052_SEQTIMER_REG, 0x11);	/* 43 */

	pwc_reg_write(DA9052_CONTROLC_REG, 0x60);	/* 16 */


	/* SMU VDD Wait Setting */
	writel(0x147, SMU_PLLVDDWAIT);
}
#endif

#if !PM_CONTROL_MODE
static void pw_ic_spi(void)
{
	/* setup for deepsleep */
	pwc_reg_write(DA9052_ID01_REG, 0x90);		/* 29 */
	pwc_reg_write(DA9052_BUCKCORE_REG, 0xCA);	/* 46 */
	pwc_reg_write(DA9052_BUCKB_REG, 0x88);		/* 45 */
	pwc_reg_write(DA9052_PDDIS_REG, 0x40);		/* 18 */
	pwc_reg_write(DA9052_SEQTIMER_REG, 0xC1);	/* 43 */

	if (reg_state.pwc.ldo8 & 0x40) {
		/* 57 */
		pwc_reg_write(DA9052_LDO8_REG, reg_state.pwc.ldo8 | 0x80);
	}

	if (reg_state.pwc.ldo9 & 0x40) {
		/* 58 */
		pwc_reg_write(DA9052_LDO9_REG, reg_state.pwc.ldo9 | 0x80);
	}

#ifdef CONFIG_MACH_EMEV
	/* for DA9052 auto wake. */
	pwc_reg_write(DA9052_GPIO0809_REG, 0x11);	/* 25 */
#endif
}
#endif


static void pw_ic_mask(int sleep_flag)
{
	switch (sleep_flag) {
	case EMXX_PMU_CLK_SLEEP:
	case EMXX_PMU_CLK_DEEPSLEEP:
		/* case: enable-key */
		/* key -> io_exp -> DA9052:GPIO0 */
#if PM_CONTROL_MODE
		/* lowpwr */
		pwc_reg_write(DA9052_GPIO0001_REG, 0x81);	/* 21 */
#else
		/* spi */
		pwc_reg_write(DA9052_GPIO0001_REG, 0x89);	/* 21 */
#endif

		/* DA9052 irq unmask */
		/* IO expander0 lines to GPIO0 of DA9052 */
		pwc_write(DA9052_IRQMASKC_REG, 0x00, MASK_KEY_INT);	/* 12 */
		/* enable pen down(LCD panel) */
		pwc_write(DA9052_IRQMASKB_REG, 0x00, MASK_PEN_DOWN_INT);/* 11 */
		/* enable usb detect(only insert) */
		pwc_write(DA9052_IRQMASKA_REG, 0x00,
		 MASK_CHARGER_INT | MASK_ALARM_INT);	/* 10 */
		pwc_write(DA9052_IRQMASKD_REG, 0x00, MASK_SYS_EN_INT);

		break;
	case EMXX_PMU_CLK_POWEROFF:
		pwc_reg_write(DA9052_GPIO0001_REG, 0x89);	/* 21 */
		pwc_write(DA9052_IRQMASKA_REG, 0x00,
		 MASK_CHARGER_INT | MASK_ALARM_INT);
		pwc_write(DA9052_IRQMASKC_REG, 0x00, MASK_KEY_INT);
		break;
	default:
		break;
	}
}



/*
 * pmu_set_regs(unsigned int sleep_flag)
 *
 */
static void pmu_set_regs(unsigned int sleep_flag)
{
	unsigned int regval;
	struct irq_chip *chip;
	unsigned long last_jiffies, delta_jiffies;


	/* pmu command sequence set */
	pmu_set_command_sequence(sleep_flag);

	/* if P0_ON ? */
	writel(0x00111011, SMU_P0_SWENA);	/* RAM Disable */

	/* backup to pmu ram */
	writel(0x00000000, PMU_RUNCHECK);
	if ((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_DEEPSLEEP) {
		pwc_reg_read(DA9052_ISET_REG, (unsigned char *)&regval);
		writel(regval & 0xff, PMU_PWIC_ISET);	/* RAM */
		pwc_reg_read(DA9052_CHGBUCK_REG, (unsigned char *)&regval);
		writel(regval & 0xff, PMU_PWIC_CHGBUCK);
		pwc_reg_read(DA9052_BUCKCORE_REG,
		 (unsigned char *)&regval);	/* 46 */
		writel(regval & 0xff, PMU_PWIC_BUCKCORE);
	}

	if (emxx_sleep_while_idle) {
		unsigned int tm_value;

		/* set enable TI2 */
		chip = get_irq_chip(INT_TIMER2);
		chip->unmask(INT_TIMER2);

		/* check kernel timer */
		last_jiffies = jiffies;
		delta_jiffies = get_next_timer_interrupt(last_jiffies)
		 - last_jiffies;
		tm_value = 0x8000*delta_jiffies/100;
		if (tm_value == 0)
			writel(1, TI2_SET);
		else
			writel(tm_value, TI2_SET);
	}

	if ((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_DEEPSLEEP) {
		/* deepsleep setting */
#if PM_CONTROL_MODE
		pw_ic_lowpower();
#else
		pw_ic_spi();
#endif
	}

	if (!emxx_sleep_while_idle) {
		/* Set IRQ of PowerIC */
		pw_ic_mask(sleep_flag & EMXX_PMU_CLK_MASK);
	}

	/* INT set(sec) */
	writel(0x00000001, PMU_INTFFCLR_M);
	writel(0x00000003, PMU_INTFFCLR_A);
	writel(0x00000001, PMU_INTENSET_M);
	writel(0x00000003, PMU_INTENSET_A);

	writel(0x00040B04, SMU_P1_RFF_PARA1);

	outl(PMU_PC_MAIN, PMU_PC);		/* 1000 */
	outl(PMU_PC_ON2, PMU_POWER_ON_PC);	/* 12C0 */
	outl(PMU_PC_ON2, PMU_INT_HANDLER_PC);

	outl(PMU_WDT_ENABLE, PMU_WDT_COUNT_EN);
	outl(PMU_WDT_MAX_COUNT, PMU_WDT_COUNT_LMT);

	/* pmu start */
	outl(PMU_START_SET, PMU_START);
	do {
		regval = inl(PMU_START);
	} while ((regval & PMU_START_SET) != PMU_START_SET);

	DPRINTK("PMU_PC           =0x%08x \n", inl(PMU_PC));
	DPRINTK("PMU_POWER_ON_PC  =0x%08x \n", inl(PMU_POWER_ON_PC));
	DPRINTK("PMU_WDT_COUNT_EN =0x%08x \n", inl(PMU_WDT_COUNT_EN));
	DPRINTK("PMU_WDT_COUNT_LMT=0x%08x \n", inl(PMU_WDT_COUNT_LMT));
	DPRINTK("PMU_START        =0x%08x \n", inl(PMU_START));
}


static void pmu_save_state(void)
{
	/* modify register save */
	reg_state.smu.ckrq_mode = readl(SMU_CKRQ_MODE);
	/* save only. not restore */
	reg_state.smu.clk_mode_sel = readl(SMU_CLK_MODE_SEL);

	/* da9052 modify register save */
	pwc_reg_read(DA9052_CONTROLC_REG, &reg_state.pwc.controlc);
	pwc_reg_read(DA9052_GPIO0001_REG, &reg_state.pwc.gpio0001);
	pwc_reg_read(DA9052_LDO8_REG, &reg_state.pwc.ldo8);	/* 57 */
	pwc_reg_read(DA9052_LDO9_REG, &reg_state.pwc.ldo9);	/* 58 */
}

static void pmu_restore_state(void)
{
	/* modified register restore */
	writel(reg_state.smu.ckrq_mode, SMU_CKRQ_MODE);

	pwc_reg_write(DA9052_CONTROLC_REG, reg_state.pwc.controlc);
	pwc_reg_write(DA9052_GPIO0001_REG, reg_state.pwc.gpio0001);
	pwc_reg_write(DA9052_LDO8_REG, reg_state.pwc.ldo8);	/* 57 */
	pwc_reg_write(DA9052_LDO9_REG, reg_state.pwc.ldo9);	/* 58 */
}


/*
 * Wait for Interrupt
 *
 */
void emxx_cpu_do_idle(unsigned int pmu_boot)
{
	/* WFI */
	cpu_do_idle();

	/* Clear PMU boot bit */
	if (inl(PMU_START))
		outl(0x00000000, PMU_START);

	return;
}

static void save_spi_state(void)
{
	struct spi_state *state = &reg_state.spi;

	state->enset = readl(SIO0_SPI_ENSET);
}

static void restore_spi_state(void)
{
	struct spi_state *state = &reg_state.spi;

	writel(0xFF, SIO0_SPI_ENCLR);
	writel(0x07, SIO0_SPI_FFCLR);
	writel(state->enset, SIO0_SPI_ENSET);
}

static void save_swon_state(void)
{
	struct swon_state *state = &reg_state.swon;

	state->pv_swon = readl(SMU_PV_SWON);
	state->pr_swon = readl(SMU_PR_SWON);
	state->pg_swon = readl(SMU_PG_SWON);
#ifdef CONFIG_MACH_EMEV
	state->p2_swon = readl(SMU_P2_SWON);
#endif
	state->pu_swon = readl(SMU_PU_SWON);
	state->pd_swon = readl(SMU_PD_SWON);
	state->p1_swon = readl(SMU_P1_SWON);
#ifdef CONFIG_MACH_EMEV
	state->pl_swon = readl(SMU_PL_SWON);
	state->pm_swon = readl(SMU_PM_SWON);
#endif
	state->ps_swon = readl(SMU_PS_SWON);
	state->p0_swon = readl(SMU_P0_SWON);
}


static void restore_swon_state(void)
{
	struct swon_state *state = &reg_state.swon;

	writel((readl(SMU_PV_SWON) & 0xFFFFFEFF)
	 | (state->pv_swon & 0x00000100), SMU_PV_SWON);
	writel((readl(SMU_PR_SWON) & 0xFFFFFEFF)
	 | (state->pr_swon & 0x00000100), SMU_PR_SWON);
	writel((readl(SMU_PG_SWON) & 0xFFFFFEFF)
	 | (state->pg_swon & 0x00000100), SMU_PG_SWON);
#ifdef CONFIG_MACH_EMEV
	writel((readl(SMU_P2_SWON) & 0xFFFFFEFF)
	 | (state->p2_swon & 0x00000100), SMU_P2_SWON);
#endif
	writel((readl(SMU_PU_SWON) & 0xFFFFFEFF)
	 | (state->pu_swon & 0x00000100), SMU_PU_SWON);
	writel((readl(SMU_PD_SWON) & 0xFFFFFEFF)
	 | (state->pd_swon & 0x00000100), SMU_PD_SWON);
	writel((readl(SMU_P1_SWON) & 0xFFFFFEFF)
	 | (state->p1_swon & 0x00000100), SMU_P1_SWON);
#ifdef CONFIG_MACH_EMEV
	writel((readl(SMU_PL_SWON) & 0xFFFFFEFF)
	 | (state->pl_swon & 0x00000100), SMU_PL_SWON);
	writel((readl(SMU_PM_SWON) & 0xFFFFFEFF)
	 | (state->pm_swon & 0x00000100), SMU_PM_SWON);
#endif
	writel((readl(SMU_PS_SWON) & 0xFFFFFEFF)
	 | (state->ps_swon & 0x00000100), SMU_PS_SWON);
	writel((readl(SMU_P0_SWON) & 0xFFFFFEFF)
	 | (state->p0_swon & 0x00000100), SMU_P0_SWON);
}

/*
 * set retention mode.
 */
static void pmu_set_p0_mode(void)
{
	/* set mode: retention */
	writel(readl(SMU_PV_SWON) & 0xFFFFFEFF, SMU_PV_SWON);
	writel(readl(SMU_PR_SWON) & 0xFFFFFEFF, SMU_PR_SWON);
	writel(readl(SMU_PG_SWON) & 0xFFFFFEFF, SMU_PG_SWON);
#ifdef CONFIG_MACH_EMEV
	writel(readl(SMU_P2_SWON) & 0xFFFFFEFF, SMU_P2_SWON);
#endif
	writel(readl(SMU_PU_SWON) & 0xFFFFFEFF, SMU_PU_SWON);
	writel(readl(SMU_PD_SWON) & 0xFFFFFEFF, SMU_PD_SWON);
	writel(readl(SMU_P1_SWON) & 0xFFFFFEFF, SMU_P1_SWON);
#ifdef CONFIG_MACH_EMEV
	writel(readl(SMU_PL_SWON) & 0xFFFFFEFF, SMU_PL_SWON);
	writel(readl(SMU_PM_SWON) & 0xFFFFFEFF, SMU_PM_SWON);
#endif
	writel(readl(SMU_PS_SWON) & 0xFFFFFEFF, SMU_PS_SWON);
	writel(readl(SMU_P0_SWON) & 0xFFFFFEFF, SMU_P0_SWON);

	return;
}



/*
 * pmu_do_suspend()
 *
 */
static void pmu_do_suspend(unsigned int sleep_flag)
{
	save_spi_state();
	save_swon_state();
	pmu_set_p0_mode();		/* set to retention mode */

	if (!emxx_sleep_while_idle) {
		pmu_int_mask(PMU_INT_MASK_SAVE_AND_MASK);
		timer_set_clock(TIMER_SUSPEND);
		/* Disable GPIO Interrupt & Set PWC Resume Interrupt */
		pmu_gpio_mask(sleep_flag & EMXX_PMU_CLK_MASK);
	}

	/* save registers */
	pmu_save_state();

	/* PMU registers setting and boot */
	pmu_set_regs(sleep_flag);

	/* WDT stop */
	wdt_op_reg = inl(WDT_OP);
	if (wdt_op_reg & 0x1) { /* TM_EN */
		DPRINTK("disable WDT\n");
		emxx_wdt_disable();
	}
	/* Auto Frq Change Disable */

	outl(inl(SMU_CKRQ_MODE) & ~0x00000001, SMU_CKRQ_MODE);

	/* Auto Self Reflesh enable by PM */
	outl((inl(MEMC_DDR_CONFIGR2) & ~0x000000FC) | 0x0000001E,
	 MEMC_DDR_CONFIGR2);

	/* TIMER START , idle sleep */
	if (emxx_sleep_while_idle)
		writel(0x07, TI2_OP);

#if defined(CONFIG_MACH_EMEV) && defined(CONFIG_CACHE_L2X0)
	if ((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_DEEPSLEEP)
		l2x0_suspend();
#endif
	emxx_cpu_do_idle(EMXX_PMU_BOOT);

	if (((reg_state.smu.clk_mode_sel & 0x00000F00) >> 8) == NORMAL_B) {
		/* NORMALB mode */
		pm_change_normalB();
	} else {
		/* other to  Normal A mode */
		pm_change_normalA();
	}

	return;
}


/*
 * pmu_do_resume()
 *
 */
static void pmu_do_resume(int sleep_flag)
{
	unsigned int regval;
	DPRINTK("resume start... \n");

#if defined(CONFIG_MACH_EMEV) && defined(CONFIG_CACHE_L2X0)
	if ((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_DEEPSLEEP)
		l2x0_resume();
#endif

	restore_swon_state();
	restore_spi_state();
	pmu_restore_state();

	if (((sleep_flag & EMXX_PMU_CLK_MASK) == EMXX_PMU_CLK_DEEPSLEEP)
			&& (readl(PMU_RUNCHECK) == 0)) {
		/* not run pmu_code. */
		regval = readl(PMU_PWIC_BUCKCORE);
		/* 46 */
		pwc_reg_write(DA9052_BUCKCORE_REG, (unsigned char)regval);
	}
#ifdef PM_DEBUG
	else
		pmu_count++;

	pm_try_count++;
	printk(KERN_INFO "(pm_try,pmu)=(%d, %d)\n", pm_try_count, pmu_count);
#endif

	if (emxx_sleep_while_idle) {
		struct irq_chip *chip;

		writel(0x00, TI2_OP);
		writel(0x02, TI2_CLR);

		/* mask & clear */
		chip = get_irq_chip(INT_TIMER2);
		chip->mask(INT_TIMER2);

		/* check pending */
		regval = readl(GIC_064_PEN);
		if (regval & TIMER2_INT_BIT) {
			/* TI2 clear */
			writel(TIMER2_INT_BIT, GIC_064_PDS);
		}
	} else {
		/* Restore GPIO/PWC Interrupt */
		pmu_gpio_mask(EMXX_PMU_CLK_FULLSPEED);
		/* INT restore */
		pmu_int_mask(PMU_INT_MASK_RESTORE);
		timer_set_clock(TIMER_RESUME);
	}

	/* WDT start */
	if (wdt_op_reg & 0x1)
		emxx_wdt_enable();

}


/*
 * emxx_pmu_sleep(unsigned int flag)
 */
int emxx_pmu_sleep(unsigned int sleep_flag)
{
	DPRINTK("emxx_pmu_sleep.. sleep_flag=0x%x \n", sleep_flag);

#ifdef	DEBUG_LED
	pwc_write(DA9052_GPIO1415_REG, 0x22, 0xFF);	/* off */
#endif

	/* suspend */
	pmu_do_suspend(sleep_flag);
	pmu_do_resume(sleep_flag);

#ifdef	DEBUG_LED
	pwc_write(DA9052_GPIO1415_REG, 0xaa, 0xFF);	/* on */
#endif
	return 0;
}

int emxx_pm_do_poweroff(void)
{
#ifdef	CONFIG_FB_EMXX
	struct pm_message message = { .event = PM_EVENT_SUSPEND, };
#endif

#ifdef	CONFIG_FB_EMXX
	emxx_lcd_suspend((struct platform_device *)NULL, message);
#endif
	if (!in_interrupt())
		gpio_set_value(GPIO_AUDIO_RST, 0);	/* PDN-pin @ AK4648 */

	outl(0x03, SMU_QR_WFI); /* PMU BOOT */
	/* setup for power down */
	pwc_write(DA9052_BUCKCORE_REG, 0x18, 0x3F);	/* 46 */
	pwc_reg_write(DA9052_RESET_REG, 0x41);		/* 20 */
	pwc_reg_write(DA9052_ID01_REG, 0x95);		/* 29 */
	pwc_reg_write(DA9052_BUCKB_REG, 0x88);		/* 45 */
	pwc_reg_write(DA9052_SEQTIMER_REG, 0xC1);
	/* for DA9052 auto wake. */
	pwc_reg_write(DA9052_GPIO0809_REG, 0x11);		/* 18 */

	/* GPIO Disable */
	pmu_gpio_mask(EMXX_PMU_CLK_POWEROFF);

	/* INT Disable */
	pmu_int_mask(PMU_INT_ALLMASK);
	/* Set IRQ of PowerIC */
	pw_ic_mask(EMXX_PMU_CLK_POWEROFF);

	/* power down */
	pwc_reg_write(DA9052_CONTROLB_REG, 0xAD);

	/* WFI */
	cpu_do_idle();

	return 0;
}

/*
 * init
 *
 */
int __init emxx_pmu_init(void)
{
	wdt_op_reg = 0;

	printk(KERN_INFO "Starting pmu... \n");

#ifdef PM_DEBUG
	pm_try_count = 0;
	pmu_count = 0;
#endif
	/* pmu code area status init */
	writel(0xffffffff, PMU_SLEEPFLAG);
	writel(0xffffffff, PMU_LOWPOWERFLAG);
	writel(0xffffffff, PMU_IDLEFLAG);

	return 0;
}

device_initcall(emxx_pmu_init);
