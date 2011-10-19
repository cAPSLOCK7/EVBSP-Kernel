/*
 *  File Name	    : linux/arch/arm/mach-emxx/pm_pmu.h
 *  Function	    : pmu
 *  Release Version : Ver 1.07
 *  Release Date    : 2011/01/24
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

#ifndef __ARCH_ARM_MACH_EMXX_PM_PMU_H
#define __ARCH_ARM_MACH_EMXX_PM_PMU_H

#include <mach/smu.h>
#include <mach/gpio.h>
#include <asm/hardware/gic.h>

/*
 * Interrupt mask and unmask
 *
 */

#define INT_SDMA_SIO1		70
#define INT_AB_SEC		162
#define INT_MEMC_SEC		163
#define INT_AFS_SEC		165

#define MASK_INT_ALL		0xffffffff
#define PCM0_INT_BIT 		(1 << (INT_SIO1 - 32))
#define TW1_INT_BIT 		(1 << (INT_WDT1 - 64))
#define GIO0_INT_BIT		(1 << (INT_GIO0 - 96))
#define PDMA_INT_BIT 		(1 << (INT_PDMA - 96))
#define AB_SEC_INT_BIT		(1 << (INT_AB_SEC - 160))
#define MEMC_SEC_INT_BIT	(1 << (INT_MEMC_SEC - 160))
#define AFS_SEC_INT_BIT		(1 << (INT_AFS_SEC - 160))
#define TIMER2_INT_BIT		(1 << (INT_TIMER2 - 64))

#define SEC_ERR_INT	(MEMC_SEC_INT_BIT | AB_SEC_INT_BIT | AFS_SEC_INT_BIT)

#define RESUME_INT_1	GIO0_INT_BIT

#define SIO0_SPI_ENSET	(IO_ADDRESS(EMXX_SIO0_BASE) + 0x1020)
#define SIO0_SPI_ENCLR	(IO_ADDRESS(EMXX_SIO0_BASE) + 0x1024)
#define SIO0_SPI_FFCLR	(IO_ADDRESS(EMXX_SIO0_BASE) + 0x1028)

/* Timer registers */
#define TI2_OP		(IO_ADDRESS(EMXX_TIMER2_BASE) + 0x0000)
#define TI2_CLR		(IO_ADDRESS(EMXX_TIMER2_BASE) + 0x0004)
#define TI2_SET		(IO_ADDRESS(EMXX_TIMER2_BASE) + 0x0008)
#define WDT_OP		(IO_ADDRESS(EMXX_WDT0_BASE) + 0x0000)

/* GIC */
#define GIC_000_IEN	(IO_ADDRESS(EMXX_INTA_DIST_BASE) \
			 + GIC_DIST_ENABLE_SET + 0x00)
#define GIC_032_IEN	(IO_ADDRESS(EMXX_INTA_DIST_BASE) \
			 + GIC_DIST_ENABLE_SET + 0x04)
#define GIC_064_IEN	(IO_ADDRESS(EMXX_INTA_DIST_BASE) \
			 + GIC_DIST_ENABLE_SET + 0x08)
#define GIC_096_IEN	(IO_ADDRESS(EMXX_INTA_DIST_BASE) \
			 + GIC_DIST_ENABLE_SET + 0x0C)
#define GIC_128_IEN	(IO_ADDRESS(EMXX_INTA_DIST_BASE) \
			 + GIC_DIST_ENABLE_SET + 0x10)
#define GIC_160_IEN	(IO_ADDRESS(EMXX_INTA_DIST_BASE) \
			 + GIC_DIST_ENABLE_SET + 0x14)

#define GIC_000_IDS	(IO_ADDRESS(EMXX_INTA_DIST_BASE) \
			 + GIC_DIST_ENABLE_CLEAR + 0x00)

#define GIC_064_PEN	(IO_ADDRESS(EMXX_INTA_DIST_BASE) \
			 + GIC_DIST_PENDING_SET + 0x08)
#define GIC_064_PDS	(IO_ADDRESS(EMXX_INTA_DIST_BASE) \
			 + GIC_DIST_PENDING_CLEAR + 0x08)


/* GPIO registers */
/* GPIO 0-31 */
#define GIO_000_IEN	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_000_OFFSET + GIO_IEN)
#define GIO_000_RAW	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_000_OFFSET + GIO_RAW)
#define GIO_000_IDS	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_000_OFFSET + GIO_IDS)
#define GIO_000_IIM	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_000_OFFSET + GIO_IIM)
#define GIO_000_MST	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_000_OFFSET + GIO_MST)
#define GIO_000_IIA	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_000_OFFSET + GIO_IIA)
#define GIO_000_IIR	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_000_OFFSET + GIO_IIR)
#define GIO_000_IDT0	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_000_OFFSET + GIO_IDT0)
#define GIO_000_IDT1	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_000_OFFSET + GIO_IDT1)
#define GIO_000_IDT2	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_000_OFFSET + GIO_IDT2)
#define GIO_000_IDT3	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_000_OFFSET + GIO_IDT3)
#define GIO_000_E0	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_000_OFFSET + GIO_E0)
#define GIO_000_EM	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_000_OFFSET + GIO_EM)
/* GPIO 32-63 */
#define GIO_032_IEN	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_032_OFFSET + GIO_IEN)
#define GIO_032_RAW	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_032_OFFSET + GIO_RAW)
#define GIO_032_IDS	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_032_OFFSET + GIO_IDS)
#define GIO_032_IIM	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_032_OFFSET + GIO_IIM)
#define GIO_032_MST	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_032_OFFSET + GIO_MST)
#define GIO_032_IIA	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_032_OFFSET + GIO_IIA)
#define GIO_032_IIR	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_032_OFFSET + GIO_IIR)
#define GIO_032_IDT0	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_032_OFFSET + GIO_IDT0)
#define GIO_032_IDT1	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_032_OFFSET + GIO_IDT1)
#define GIO_032_IDT2	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_032_OFFSET + GIO_IDT2)
#define GIO_032_IDT3	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_032_OFFSET + GIO_IDT3)
#define GIO_032_E0	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_032_OFFSET + GIO_E0)
#define GIO_032_EM	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_032_OFFSET + GIO_EM)
/* GPIO 64-95 */
#define GIO_064_IEN	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_064_OFFSET + GIO_IEN)
#define GIO_064_RAW	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_064_OFFSET + GIO_RAW)
#define GIO_064_IDS	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_064_OFFSET + GIO_IDS)
#define GIO_064_IIM	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_064_OFFSET + GIO_IIM)
#define GIO_064_MST	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_064_OFFSET + GIO_MST)
#define GIO_064_IIA	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_064_OFFSET + GIO_IIA)
#define GIO_064_IIR	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_064_OFFSET + GIO_IIR)
#define GIO_064_IDT0	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_064_OFFSET + GIO_IDT0)
#define GIO_064_IDT1	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_064_OFFSET + GIO_IDT1)
#define GIO_064_IDT2	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_064_OFFSET + GIO_IDT2)
#define GIO_064_IDT3	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_064_OFFSET + GIO_IDT3)
#define GIO_064_E0	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_064_OFFSET + GIO_E0)
#define GIO_064_EM	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_064_OFFSET + GIO_EM)
/* GPIO 96-127 */
#define GIO_096_IEN	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_096_OFFSET + GIO_IEN)
#define GIO_096_RAW	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_096_OFFSET + GIO_RAW)
#define GIO_096_IDS	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_096_OFFSET + GIO_IDS)
#define GIO_096_IIM	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_096_OFFSET + GIO_IIM)
#define GIO_096_MST	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_096_OFFSET + GIO_MST)
#define GIO_096_IIA	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_096_OFFSET + GIO_IIA)
#define GIO_096_IIR	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_096_OFFSET + GIO_IIR)
#define GIO_096_IDT0	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_096_OFFSET + GIO_IDT0)
#define GIO_096_IDT1	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_096_OFFSET + GIO_IDT1)
#define GIO_096_IDT2	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_096_OFFSET + GIO_IDT2)
#define GIO_096_IDT3	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_096_OFFSET + GIO_IDT3)
#define GIO_096_E0	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_096_OFFSET + GIO_E0)
#define GIO_096_EM	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_096_OFFSET + GIO_EM)

/* GPIO 128-159*/
#define GIO_128_IEN	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_128_OFFSET + GIO_IEN)
#define GIO_128_RAW	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_128_OFFSET + GIO_RAW)
#define GIO_128_IDS	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_128_OFFSET + GIO_IDS)
#define GIO_128_IIM	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_128_OFFSET + GIO_IIM)
#define GIO_128_MST	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_128_OFFSET + GIO_MST)
#define GIO_128_IIA	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_128_OFFSET + GIO_IIA)
#define GIO_128_IIR	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_128_OFFSET + GIO_IIR)
#define GIO_128_IDT0	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_128_OFFSET + GIO_IDT0)
#define GIO_128_IDT1	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_128_OFFSET + GIO_IDT1)
#define GIO_128_IDT2	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_128_OFFSET + GIO_IDT2)
#define GIO_128_IDT3	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_128_OFFSET + GIO_IDT3)
#define GIO_128_E0	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_128_OFFSET + GIO_E0)
#define GIO_128_E1	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_128_OFFSET + GIO_E1)
#define GIO_128_EM	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_128_OFFSET + GIO_EM)
#define GIO_128_OL	(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_128_OFFSET + GIO_OL)


/* GPIO parameters */
#define MASK_GPIO_ALL		   0xFFFFFFFF
#define GPIO_INT_PWRIC		   (1 << 0)  /* P00 */

/* for emev board */
#define MASK_KEY_INT		(1<<0)		/* event C */
#define MASK_PEN_DOWN_INT	(1<<6)		/* event B */
#define MASK_ALARM_INT		(1<<5)		/* event A */
#define MASK_DCIN_DET		(1<<0)		/* event A */
#define MASK_VBUS_DET		(1<<1)		/* event A */
#define MASK_VBUS_REM		(1<<3)		/* event A */
#define MASK_CHARGER_INT	(0x0F<<0)	/* event A */
#define MASK_SYS_EN_INT		(1<<0)		/* event D */

/* MEMC registers*/
#define MEMC_DDR_CONFIGR2	(IO_ADDRESS(EMXX_MEMC_BASE) + 0x2018)
#define MEMC_DDR_CONFIGR3	(IO_ADDRESS(EMXX_MEMC_BASE) + 0x201c)

#define GPIO_OUTPUT_LOW        0

/* gpio mask */
#define MASK_GPIO_VBUS		(1<<25)


/* SMU registers */

#define PMU_PW_NORMAL		(0)
#define PMU_PW_RETENTION	(1)
#define PMU_PW_POWERDOWN	(2)

/* reg:POWER_STATUS */
#define PR_STATE_BIT(a)		((a)<<(20))
#define PV_STATE_BIT(a)		((a)<<(18))
#define PG_STATE_BIT(a)		((a)<<(16))
#define P2_STATE_BIT(a)		((a)<<(14))
#define PU_STATE_BIT(a)		((a)<<(12))
#define PD_STATE_BIT(a)		((a)<<(10))
#define P1_STATE_BIT(a)		((a)<<(8))
#define PL_STATE_BIT(a)		((a)<<(6))
#define PM_STATE_BIT(a)		((a)<<(4))
#define P0_STATE_BIT(a)		((a)<<(0))


/*
 * Registers store and restore
 *
 */
/* #define SMU_ENA_ALL			0xFFFFFFFF */
/* #define SMU_ENA_CLEAR		0x00000000 */

/*
 * PMU boot setting
 *
 */

/* PMU registers */
#define PMU_PC			(IO_ADDRESS(EMXX_PMU_BASE) + 0x0004)
#define PMU_START		(IO_ADDRESS(EMXX_PMU_BASE) + 0x0008)
#define PMU_POWER_ON_PC 	(IO_ADDRESS(EMXX_PMU_BASE) + 0x0030)
#define PMU_WDT_COUNT_EN	(IO_ADDRESS(EMXX_PMU_BASE) + 0x0060)
#define PMU_WDT_COUNT_LMT	(IO_ADDRESS(EMXX_PMU_BASE) + 0x0064)
#define PMU_INTENSET_A		(IO_ADDRESS(EMXX_PMU_BASE) + 0x0088)
#define PMU_INTFFCLR_A		(IO_ADDRESS(EMXX_PMU_BASE) + 0x0090)
#define PMU_INTENSET_M		(IO_ADDRESS(EMXX_PMU_BASE) + 0x009c)
#define PMU_INTFFCLR_M		(IO_ADDRESS(EMXX_PMU_BASE) + 0x00a4)


#define PMU_CMD_BUF_RAM 	(IO_ADDRESS(EMXX_PMU_BASE) + 0x1000)
#define PMU_CMD_BUF_FF		(IO_ADDRESS(EMXX_PMU_BASE) + 0x0100)

#define PMU_WDT_COUNT_LMT	(IO_ADDRESS(EMXX_PMU_BASE) + 0x0064)
#define PMU_INT_HANDLER_PC	(IO_ADDRESS(EMXX_PMU_BASE) + 0x0068)


/* PMU, ASMU parameters */
#define PMU_WDT_ENABLE			0x00000001
#define PMU_WDT_DISABLE 		0x00000000
#define PMU_WDT_MAX_COUNT		0x0003FFFF
#define PMU_START_SET			0x00000001
#define PMU_CMD_BUF_RAM_BOOT		0x504D5520
#define PMU_CMD_BUF_RAM_STOP		0x00000000
/*
 * Flags
 *
 */
#define EMXX_PMU_CLK_MASK		0x001f0000
#define EMXX_PMU_CLK_FULLSPEED		0x00010000
#define EMXX_PMU_CLK_DEEPSLEEP		0x00020000
#define EMXX_PMU_CLK_SLEEP		0x00040000
#define EMXX_PMU_CLK_POWEROFF		0x00080000
#define EMXX_PMU_CLK_ECONOMY		0x00100000



#define EMXX_PMU_BOOT			0
#define EMXX_PMU_NOTBOOT		1

#define PMU_INT_MASK_SAVE_AND_MASK	1
#define PMU_INT_MASK_RESTORE		2
#define PMU_INT_ALLMASK			3

#define PM_SLEEP_MODE_NONE		0

#define PMU_EMMC_POWER_OFF		0
#define PMU_EMMC_POWER_ON		1


#define NORMAL_A	1
#define NORMAL_B	2
#define NORMAL_C	3
#define NORMAL_D	4


/*
 * Structures
 *
 */

/* PWC*/
struct pwc_state {
	unsigned char mask_a;
	unsigned char mask_b;
	unsigned char mask_c;
	unsigned char mask_d;
	unsigned char controlc;
	unsigned char gpio0001;
	unsigned char ldo8;
	unsigned char ldo9;
};
/* SMU */
struct smu_state {
  unsigned int ckrq_mode;	/* E011_0708 */
  unsigned int clk_mode_sel;	/* E011_0300 */
};
/* AINT */
struct intc_state {
  unsigned int dist_ien0;	/* E002_8100H */
  unsigned int dist_ien1;	/* E002_8104H */
  unsigned int dist_ien2;	/* E002_8108H */
  unsigned int dist_ien3;	/* E002_810CH */
  unsigned int dist_ien4;	/* E002_8110H */
  unsigned int dist_ien5;	/* E002_8114H */
};
/* GPIO */
struct gpio_state {
  unsigned int ien0;		/* GIO_000_IDS */
  unsigned int ien1;		/* GIO_032_IDS */
  unsigned int ien2;		/* GIO_064_IDS */
  unsigned int ien3;		/* GIO_096_IDS */
  unsigned int ien4;		/* GIO_128_IDS */
};
struct memc_state{
  unsigned int configr2;
};
/* CHG */
struct chg_state {
  unsigned int pinsel_g000;	/* E014_0200H */
  unsigned int pinsel_g032;	/* E014_0204H */
  unsigned int pinsel_g064;	/* E014_0208H */
  unsigned int pinsel_g096;	/* E014_020CH */
  unsigned int pinsel_g128;	/* E014_0210H */
};

struct swon_state {
  unsigned int pv_swon;
  unsigned int pr_swon;
  unsigned int pg_swon;
  unsigned int p2_swon;
  unsigned int pu_swon;
  unsigned int pd_swon;
  unsigned int p1_swon;
  unsigned int pl_swon;
  unsigned int pm_swon;
  unsigned int ps_swon;
  unsigned int p0_swon;
};

struct spi_state {
 unsigned int enset;
};

struct register_state_t{
  struct pwc_state   pwc;
  struct smu_state   smu;
  struct intc_state  intc;
  struct gpio_state  gpio;
  struct memc_state  memc;
  struct chg_state   chg;
  struct swon_state  swon;	/* smu */
  struct spi_state   spi;
};


/*
 * Functions
 *
 */
extern void emxx_cpu_do_idle(unsigned int pmu_boot);
extern int emxx_pmu_sleep(unsigned int sleep_flag);
extern int emxx_pm_do_poweroff(void);

#if defined(CONFIG_FB_EMXX)
#include <linux/platform_device.h>
#include <linux/pm.h>
extern int emxx_lcd_suspend(struct platform_device *dev, pm_message_t state);
#endif /* CONFIG_FB_EMXX */

extern int emxx_touch_pm_state(void);
extern int emxx_key_pm_state(void);
extern int serial8250_idle_suspend(void);
extern void serial8250_idle_resume(void);
extern void emxx_sdio_idle_suspend(void);
extern void emxx_ohci_idle_suspend(void);
extern void emxx_ohci_idle_resume(void);
extern void emxx_ehci_idle_suspend(void);
extern void emxx_ehci_idle_resume(void);

extern void pm_change_normalA(void);
extern void pm_change_normalB(void);

extern void emxx_add_neigh_timer(void);
extern void emxx_del_neigh_timer(void);
extern void emxx_add_timer_writeback(void);
extern void emxx_del_timer_writeback(void);
extern void emxx_add_workqueue_timer(void);
extern void emxx_del_workqueue_timer(void);

/*
 * PMU Command Sequence
 *
 */
#define PMU_BASE		    IO_ADDRESS(EMXX_PMU_BASE)
#define PMU_VIRADDR_TO_PC(addr)     (addr - PMU_BASE)
#define PMU_PC_TO_VIRADDR(pc)	    (unsigned int *)(pc + PMU_BASE)


/* for Command Sequence SMU Register Adr */
#define SMU_CMD_MEMC_RSTCTRL		0x004C
#define SMU_CMD_USAIS0_RSTCTRL		0x008C

#define SMU_CMD_AVE_RSTCTRL		0x0068
#define SMU_CMD_A3D_RSTCTRL		0x006C
#define SMU_CMD_CCP_RSTCTRL		0x007C
#define SMU_CMD_CAM_RSTCTRL		0x0078
#define SMU_CMD_IRDA_RSTCTRL		0x0080
#define SMU_CMD_USB0_RSTCTRL		0x00E4
#define SMU_CMD_USB1_RSTCTRL		0x00E8

#define SMU_CMD_CHG1_RSTCTRL		0x0018
#define SMU_CMD_P2M_RSTCTRL		0x0030
#define SMU_CMD_M2P_RSTCTRL		0x0034
#define SMU_CMD_M2M_RSTCTRL		0x0038
#define SMU_CMD_IMC_RSTCTRL		0x0054
#define SMU_CMD_IMCW_RSTCTRL		0x0058
#define SMU_CMD_SIZ_RSTCTRL		0x005C
#define SMU_CMD_ROT_RSTCTRL		0x0060
#define SMU_CMD_JPEG_RSTCTRL		0x0064
#define SMU_CMD_DTV_RSTCTRL		0x0070
#define SMU_CMD_NTS_RSTCTRL		0x0074
#define SMU_CMD_PWM_RSTCTRL		0x0088
#define SMU_CMD_USIBS2_RSTCTRL		0x009C
#define SMU_CMD_USIBS3_RSTCTRL		0x00A0
#define SMU_CMD_USIBS4_RSTCTRL		0x00A4
#define SMU_CMD_USIBS5_RSTCTRL		0x00A8
#define SMU_CMD_USIAU0_RSTCTRL		0x0094
#define SMU_CMD_USIBU1_RSTCTRL		0x00AC
#define SMU_CMD_USIBU2_RSTCTRL		0x00B0
#define SMU_CMD_USIBU3_RSTCTRL		0x00B4
#define SMU_CMD_USIBDMA_RSTCTRL		0x00B8
#define SMU_CMD_SDIO0_RSTCTRL		0x00BC
#define SMU_CMD_SDIO1_RSTCTRL		0x00C0
#define SMU_CMD_SDIO2_RSTCTRL		0x00C4
#define SMU_CMD_SDC_RSTCTRL		0x00C8
#define SMU_CMD_CFI_RSTCTRL		0x00D0
#define SMU_CMD_MSP_RSTCTRL		0x00D4
#define SMU_CMD_HSI_RSTCTRL		0x00D8
#define SMU_CMD_IIC0_RSTCTRL		0x00DC
#define SMU_CMD_IIC1_RSTCTRL		0x00E0
#define SMU_CMD_CRP_RSTCTRL		0x0128
#define SMU_CMD_MMM_RSTCTRL		0x0130
#define SMU_CMD_BCIF_RSTCTRL		0x0134
#define SMU_CMD_MLT2_RSTCTRL		0x0138
#define SMU_CMD_CAN_RSTCTRL		0x013C

#define SMU_CMD_LCD_RSTCTRL		0x0050

#define SMU_CMD_USIAS0_RSTCTRL		0x008C
#define SMU_CMD_USIAS1_RSTCTRL		0x0090
#define SMU_CMD_USIADMA_RSTCTRL		0x0098
#define SMU_CMD_LAE_RSTCTRL		0x0144
#define SMU_CMD_PDMA_RSTCTRL		0x0148

#define SMU_CMD_PLL1CTRL1		0x0204
#define SMU_CMD_PLL2CTRL1		0x020C
#define SMU_CMD_PLL3CTRL1		0x0214
#define SMU_CMD_PLL4CTRL1		0x021C
#define SMU_CMD_OSC0CTRL1		0x0220
#define SMU_CMD_OSC1CTRL1		0x0224
#define SMU_CMD_PLL_STATUS		0x0234
#define SMU_CMD_AUTO_MODE_EN		0x02F4
#define SMU_CMD_CLK_MODE_SEL		0x0300
#define SMU_CMD_MEMC_HAND_SHAKE_FAKE	0x0304
#define SMU_CMD_CPUGCLKCTRL		0x0400
#define SMU_CMD_INTAGCLKCTRL		0x040C
#define SMU_CMD_USIAS0GCLKCTRL		0x0498
#define SMU_CMD_USIASCLKDIV		0x0618
#define SMU_CMD_LOWPWR			0x07F0
#define SMU_CMD_POWER_STATUS		0x082C
#define SMU_CMD_SEQ_BUSY		0x0830
#define SMU_CMD_P0_SWON			0x0834
#define SMU_CMD_PU_SWON			0x083C
#define SMU_CMD_PM_SWON			0x0840
#define SMU_CMD_PL_SWON			0x0844
#define SMU_CMD_PD_SWON			0x0848
#define SMU_CMD_P1_SWON			0x084C
#define SMU_CMD_P2_SWON			0x0850
#define SMU_CMD_PG_SWON			0x0854
#define SMU_CMD_PV_SWON			0x0858
#define SMU_CMD_PR_SWON			0x085C
#define SMU_CMD_P0_PWSW_PARA		0x0868
#define SMU_CMD_PM_PWSW_PARA		0x088C
#define SMU_CMD_PL_PWSW_PARA		0x0898
#define SMU_CMD_P1_PWSW_PARA		0x08B0
#define SMU_CMD_PD_PWSW_PARA		0x08A4
#define SMU_CMD_PU_PWSW_PARA		0x0880
#define SMU_CMD_P2_PWSW_PARA		0x08BC
#define SMU_CMD_PG_PWSW_PARA		0x08C8
#define SMU_CMD_PR_PWSW_PARA		0x08E0
#define SMU_CMD_PV_PWSW_PARA		0x08D4
#define SMU_CMD_CPU_PWSW_L2RAM		0x098C
#define SMU_CMD_CPU_PWSW_CTRL		0x099C
#define SMU_CMD_CPU_SEQ_BUSY		0x09A4
#define SMU_CMD_DS1_SWON		0x09D4
#define SMU_CMD_PE1_SWON		0x09E0
#define SMU_CMD_NE0_SWON		0x09E4
#define SMU_CMD_NE1_SWON		0x09E8
#define SMU_CMD_DS0_SWON		0x09D0
#define SMU_CMD_HM_SWON			0x09D8
#define SMU_CMD_PE0_SWON		0x09DC
#define SMU_CMD_QR_BYPS_SYS		0x0A24
#define SMU_CMD_QR_BYPS_PSW		0x0A28
#define SMU_CMD_PMU_INTCTRL		0x0A30
#define SMU_CMD_S0_DUMMY_REG4		0x101C


/* for Command Sequence SPI Register Adr */
#define SIO_CMD_SP0_MODE		0x1000
#define SIO_CMD_SP0_POL			0x1004
#define SIO_CMD_SP0_CONTROL		0x1008
#define SIO_CMD_SP0_TX_DATA		0x1010
#define SIO_CMD_SP0_ENSET		0x1020
#define SIO_CMD_SP0_FFCLR		0x1028


/* for Command Sequence MEM Register Adr */
#define MEMC_CMD_CACHE_MODE		0x0000
#define MEMC_CMD_DEGFUN			0x0008
#define MEMC_CMD_LDP_MODE		0x0070
#define MEMC_CMD_REQSCH			0x1000
#define MEMC_CMD_DDR_STATE8		0x202C


/* PMU PC */
#define PMU_PC_MAIN			0x1000	/* Normal2Economy */
#define PMU_PC_ECONOMY			0x1050
#define PMU_PC_SLEEP			0x1080
#define PMU_PC_DEEP			0x1100
#define PMU_PC_PWRCNT			0x0100
#define PMU_PC_PWRCNT2			0x11C0
#define PMU_PC_ON1			0x1200
#define PMU_PC_ON2			0x12C0
#define PMU_PC_SUB_SWOFF0		0x1300
#ifdef CONFIG_MACH_EMEV
#define PMU_PC_SUB_SWOFF1		0x13A0
#endif
#define PMU_PC_SUB_SWOFF2		0x1400
#define PMU_PC_SUB_SWOFF3		0x14A0
#define PMU_PC_SUB_SWON0		0x1500
#ifdef CONFIG_MACH_EMEV
#define PMU_PC_SUB_SWON1		0x1700
#endif
#define PMU_PC_SUB_SWON2		0x1750
#define PMU_PC_SUB_SWON3		0x1800
#define PMU_PC_SUB_SPI0			0x1850
#define PMU_PC_SUB_SPI1			0x1940
#define PMU_PC_SUB_SPI2			0x1980
#define PMU_PC_SUB_SETPARA		0x1A00
#ifdef CONFIG_MACH_EMEV
#define PMU_PC_SUB_L2OFF		0x1D00
#define PMU_PC_SUB_L2ON			0x1D70
#endif
#define PMU_PC_SUB_VDD_SPI		0x1F00
#define PMU_PC_SUB_VDD_LPW		0x2400


/* APB Macro Parameter */
#define TIM_MACRO			(0x00 << 16)
#define INTA_MACRO			(0x01 << 16)
#define LCD_MACRO			(0x02 << 16)
#define GIO_MACRO			(0x03 << 16)
#define MEMC_MACRO			(0x04 << 16)
#define AFS_MACRO			(0x05 << 16)
#define IRR_MACRO			(0x06 << 16)
#define STI_MACRO			(0x07 << 16)
#define IIC0_MACRO			(0x08 << 16)
#define PMU_MACRO			(0x09 << 16)
#define SMU_MACRO			(0x0A << 16)
#define SIO0_MACRO			(0x0B << 16)	/* USIA */
/* 0x0C:reserved */
#define SDMA_MACRO			(0x0D << 16)
#define CHG_MACRO			(0x0E << 16)
#define SIO1_MACRO			(0x0F << 16)
#define AFS_SEC_MACRO			(0x10 << 16)
#define BUS0_MACRO			(0x11 << 16)
#define INTD_MACRO			(0x12 << 16)	/* INTA */
#define INTT_MACRO			(0x13 << 16)	/* INTA */
/* 0x14-1E:reserved */
#define SMU_SEC_MACRO			(0x1F << 16)

/* Command Parameter */
#define CMD_REG_WRITE			(0x00 << 24)
#define CMD_USI0_WRITE			(0x01 << 24)
#define CMD_REG_READ			(0x02 << 24)
#define CMD_RMW				(0x03 << 24)
#define CMD_MOVE			(0x04 << 24)
#define CMD_AND 			(0x05 << 24)
#define CMD_EXOR			(0x06 << 24)
#define CMD_CMP1			(0x07 << 24)
#define CMD_CMP2			(0x08 << 24)
#define CMD_REG_WRITE2			(0x09 << 24)
#define CMD_BRANCH			(0x10 << 24)
#define CMD_JUMP			(0x11 << 24)
#define CMD_AJUMP			(0x12 << 24)
#define CMD_SUBROUTINE_START		(0x13 << 24)
#define CMD_RFS 			(0x14 << 24)
#define CMD_TIMERWAIT			(0x20 << 24)
#define CMD_INTWAIT			(0x21 << 24)
#define CMD_SMU_READY_WAIT		(0x23 << 24)
#define CMD_TRIG_WAIT			(0x24 << 24)
#define CMD_CYCLE_WAIT			(0x26 << 24)
#define CMD_INT_MASK			(0x30 << 24)
#define CMD_ARMINT_MASK 		(0x31 << 24)
#define CMD_WDT_CLEAR			(0x32 << 24)
#define CMD_WDT_STOP			(0x33 << 24)
#define CMD_WDT_RESTART 		(0x34 << 24)
#define CMD_PMU_END			(0x35 << 24)
#define CMD_NOP 			(0x36 << 24)	/* as Other value */

/* Other Parameters */
#define PMU_REGA			(0 << 30)
#define PMU_REGB			(1 << 30)
#define PMU_BEQ 			(0 << 30)
#define PMU_BNE 			(1 << 30)
#define PMU_WAIT_INT_END		(0 << 30)
#define PMU_WAIT_INT_CONTINUE		(1 << 30)
#define PMU_LOW_LEVEL			(0 << 30)
#define PMU_HIGH_LEVEL			(1 << 30)
#define PMU_TRIG_TIMER			(1 << 16)
#define PMU_TRIG_P1WAKEUP_LOW		(1 << 19)
#define PMU_TRIG_P1WAKEUP_HIGH		(1 << 20)
#define PMU_TRIG_INT			(1 << 21)
#define PMU_INT_UNMASK			(0 << 30)
#define PMU_INT_MASK			(1 << 30)


/* PMU command size */
#define PCMD_BRANCH_SIZE	4
#define PCMD_REG_WRITE_SIZE	8
#define PCMD_AJUMP_SIZE		4
#define PCMD_RMW_SIZE		12
#define PCMD_REG_READ_SIZE	4
#define PCMD_CMP2_SIZE		8
#define PCMD_AND_SIZE		8
#define PCMD_NOP_SIZE		4

/* PMU RAM */
#define PMU_RAM_SMU_USIAS0GCLKCTRL	0x2004
#define PMU_RAM_SMU_POWERSTATUS		0x2700
#define PMU_RAM_PWIC_ISET		0x2704	/* R64 */
#define PMU_RAM_PWIC_CHGBUCK		0x2708	/* R62 */
#define PMU_RAM_PWIC_BUCKCORE		0x270C	/* R46 */
#define PMU_RAM_RUNCHECK		0x2710
#define PMU_RAM_SLEEPFLAG		0x2714
#define PMU_RAM_LOWPOWERFLAG		0x2718
#define PMU_RAM_IDLEFLAG		0x271C

#define PMU_PWIC_ISET		(IO_ADDRESS(EMXX_PMU_BASE) \
				 + PMU_RAM_PWIC_ISET)
#define PMU_PWIC_CHGBUCK	(IO_ADDRESS(EMXX_PMU_BASE) \
				 + PMU_RAM_PWIC_CHGBUCK)
#define PMU_PWIC_BUCKCORE	(IO_ADDRESS(EMXX_PMU_BASE) \
				 + PMU_RAM_PWIC_BUCKCORE)
#define PMU_RUNCHECK		(IO_ADDRESS(EMXX_PMU_BASE) \
				 + PMU_RAM_RUNCHECK)
#define PMU_SLEEPFLAG		(IO_ADDRESS(EMXX_PMU_BASE) \
				 + PMU_RAM_SLEEPFLAG)
#define PMU_LOWPOWERFLAG	(IO_ADDRESS(EMXX_PMU_BASE) \
				 + PMU_RAM_LOWPOWERFLAG)
#define PMU_IDLEFLAG		(IO_ADDRESS(EMXX_PMU_BASE) \
				 + PMU_RAM_IDLEFLAG)


/* Commands */
#define PCMD_REG_WRITE(macro, addr, data) {				\
    writel((CMD_REG_WRITE | macro | addr), pmu_cmd_adr++);		\
    writel(data, pmu_cmd_adr++);					\
}

#define PCMD_SP0_WRITE(addr, data)	PCMD_USI0_WRITE(addr, data)

#define PCMD_USI0_WRITE(addr, data) {					\
    writel((CMD_USI0_WRITE | addr), pmu_cmd_adr++);			\
    writel(data, pmu_cmd_adr++);					\
}
#define PCMD_REG_READ(macro, addr, aorb) {				\
    writel((CMD_REG_READ | aorb | macro | addr), pmu_cmd_adr++);	\
}
#define PCMD_RMW(macro, addr, data, data_en) {				\
    writel((CMD_RMW | macro | addr), pmu_cmd_adr++);			\
    writel(data, pmu_cmd_adr++);					\
    writel(data_en, pmu_cmd_adr++);					\
}
#define PCMD_MOVE(data, aorb) {						\
    writel((CMD_MOVE | aorb), pmu_cmd_adr++);				\
    writel(data, pmu_cmd_adr++);					\
}
#define PCMD_AND(data, aorb) {						\
    writel((CMD_AND | aorb), pmu_cmd_adr++);				\
    writel(data, pmu_cmd_adr++);					\
}
#define PCMD_EXOR(data, aorb) { 					\
    writel((CMD_EXOR | aorb), pmu_cmd_adr++);				\
    writel(data, pmu_cmd_adr++);					\
}
#define PCMD_CMP1() {							\
    writel(CMD_CMP1, pmu_cmd_adr++);					\
}
#define PCMD_CMP2(data, aorb) { 					\
    writel((CMD_CMP2 | aorb), pmu_cmd_adr++);				\
    writel(data, pmu_cmd_adr++);				        \
}
#define PCMD_REG_WRITE2(macro, addr, aorb) {				\
    writel((CMD_REG_WRITE2 | aorb | macro | addr), pmu_cmd_adr++);	\
}
#define PCMD_BRANCH(jump, op) { 					\
    writel((CMD_BRANCH | op | jump), pmu_cmd_adr++);			\
}
#define PCMD_JUMP(jump) {						\
    writel((CMD_JUMP | jump), pmu_cmd_adr++);				\
}
#define PCMD_AJUMP(jump) {						\
    writel((CMD_AJUMP | (jump)), pmu_cmd_adr++);			\
}
#define PCMD_SUBROUTINE_START(jump) {					\
    writel((CMD_SUBROUTINE_START | jump), pmu_cmd_adr++);		\
}
#define PCMD_RFS() {							\
    writel(CMD_RFS, pmu_cmd_adr++);					\
}
#define PCMD_TIMERWAIT(count, int) {					\
    writel((CMD_TIMERWAIT | int | count), pmu_cmd_adr++);		\
}
#define PCMD_INTWAIT() {						\
    writel(CMD_INTWAIT, pmu_cmd_adr++);					\
}
#define PCMD_SMU_READY_WAIT(level) {					\
    writel((CMD_SMU_READY_WAIT | level), pmu_cmd_adr++);		\
}
#define PCMD_TRIG_WAIT(count, trig) {					\
    writel((CMD_TRIG_WAIT | trig | count), pmu_cmd_adr++);		\
}
#define PCMD_CYCLE_WAIT(count, int) {					\
    writel((CMD_CYCLE_WAIT | int | count), pmu_cmd_adr++);      	\
}
#define PCMD_INT_MASK(mask) {						\
    writel((CMD_INT_MASK | mask), pmu_cmd_adr++);			\
}
#define PCMD_ARMINT_MASK(mask) {					\
    writel((CMD_ARMINT_MASK | mask), pmu_cmd_adr++);			\
}
#define PCMD_WDT_CLEAR() {						\
    writel(CMD_WDT_CLEAR, pmu_cmd_adr++); 				\
}
#define PCMD_WDT_STOP() { 						\
    writel(CMD_WDT_STOP, pmu_cmd_adr++);				\
}
#define PCMD_WDT_RESTART() {						\
    writel(CMD_WDT_RESTART, pmu_cmd_adr++);				\
}
#define PCMD_PMU_END() {						\
    writel(CMD_PMU_END, pmu_cmd_adr++);					\
}
#define PCMD_NOP() {							\
    writel(CMD_NOP, pmu_cmd_adr++);					\
}


/* for driver idle check */
#define PM_ERROR_LCD	1
#define PM_ERROR_DSP	2
#define PM_ERROR_IMC	3
#define PM_ERROR_SIZ	4
#define PM_ERROR_KEY	5
#define PM_ERROR_PWM	6
#define PM_ERROR_SDIO0	7
#define PM_ERROR_SDIO1	8
#define PM_ERROR_SDIO2	9
#define PM_ERROR_SDC	10
#define PM_ERROR_USBH	11
#define PM_ERROR_USBF	12
#define PM_ERROR_TOUCH	13
#define PM_ERROR_SPI	14
#define PM_ERROR_AVE	15
#define PM_ERROR_IIC0	16
#define PM_ERROR_IIC1	17
#define PM_ERROR_A2D	18
#define PM_ERROR_A3D	19
#define PM_ERROR_DMA	20
#define PM_ERROR_NTS	21
#define PM_ERROR_CPU1	50


#define GCLKSDIO_CHECK_BIT	0x00000007
#define GCLKUSB_CHECK_BIT	0x04000000
#define GCLKPWM_CHECK_BIT	0x00000007
#define GCLKSPI_CHECK_BIT	0x00000007
#define GCLKAVE_CHECK_BIT	0x00000007
#define GCLKIIC_CHECK_BIT	0x00000003
#define GCLKA2D_CHECK_BIT	0x00000003
#define GCLKA3D_CHECK_BIT	0x00000007
#define GCLKNTS_CHECK_BIT	0x00000003
#define AHBSIZ_CHECK_BIT	0x00000100
#define LCD_BASE		IO_ADDRESS(EMXX_LCD_BASE)
#define LCD_LCDOUT		(LCD_BASE + 0x10)
#define LCD_STATUS		(LCD_BASE + 0x18)

#define IMC_BASE		IO_ADDRESS(EMXX_IMC_BASE)
#define IMC_STATUS		(IMC_BASE + 0x14)
#define IMCW_BASE		IO_ADDRESS(EMXX_IMCW_BASE)
#define IMCW_STATUS		(IMCW_BASE + 0x14)

#define	INTD_IT3_BASE		IO_ADDRESS(EMXX_INTA_D_BASE)
#define	INTD_IT3_IEN0		(INTD_IT3_BASE + 0x0000)	/* enable */
#define	INTD_IT3_IEN1		(INTD_IT3_BASE + 0x0004)	/* enable */
#define	INTD_IT3_IEN2		(INTD_IT3_BASE + 0x0100)	/* enable */
#define	INTD_IT3_IDS0		(INTD_IT3_BASE + 0x0008)	/* disable */
#define	INTD_IT3_IDS1		(INTD_IT3_BASE + 0x000c)	/* disable */
#define	INTD_IT3_IDS2		(INTD_IT3_BASE + 0x0104)	/* disable */
#define	INTD_IT3_RAW0		(INTD_IT3_BASE + 0x0010)	/* status */
#define	INTD_IT3_RAW1		(INTD_IT3_BASE + 0x0014)	/* status */
#define	INTD_IT3_RAW2		(INTD_IT3_BASE + 0x0108)	/* status */
#define INTD_LIIS01		0x0000c000

#define	PWSTATE_PD_NE1		0x00004000	/* CPU1 Power State */
#define	PD_PWRSTATUS		0x00000c00	/* DSP Power Satate */

#define EMXX_SLEEP_THRESHOLD	5

#endif /* __ARCH_ARM_MACH_EMXX_PM_PMU_H */
