/*
 *  File Name       : arch/arm/mach-emxx/pwc.c
 *  Function        : pwc
 *  Release Version : Ver 1.03
 *  Release Date    : 2011/02/08
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
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <linux/gpio.h>
#include <asm/mach/irq.h>
#include <mach/pwc.h>
#include <mach/spi.h>

/* #define PWC_DEBUG */
#ifdef PWC_DEBUG
#define DPRINT(FMT, ARGS...) \
	printk(KERN_INFO "%s(): " FMT, __func__, ##ARGS)
#else
#define DPRINT(FMT, ARGS...)
#endif

static DEFINE_SPINLOCK(pwc_spinlock);
static DEFINE_SPINLOCK(pwc_write_spinlock);

static SPI_CONFIG device0_config_read = {
	.dev	= SPI_DEV_SP0,
#ifdef CONFIG_EMGR_TI_PMIC
	.cs_sel	= SPI_CS_SEL_CS3,
#else
	.cs_sel	= SPI_CS_SEL_CS0,
#endif
	.m_s	= SPI_M_S_MASTER,
	.dma	= SPI_DMA_OFF,
	.pol	= SPI_CSW_8CLK | SPI_CK_DLY_ON |
			SPI_CK_POL_POS | SPI_CS_POL_POS,
	.tiecs	= SPI_TIECS_NORMAL,
	.nbw	= SPI_NB_8BIT,
	.nbr	= SPI_NB_8BIT,
	.sclk	= SPI_SCLK_12MHZ,
};

static SPI_CONFIG device0_config_write = {
	.dev	= SPI_DEV_SP0,
#ifdef CONFIG_EMGR_TI_PMIC
	.cs_sel	= SPI_CS_SEL_CS3,
#else
	.cs_sel	= SPI_CS_SEL_CS0,
#endif
	.m_s	= SPI_M_S_MASTER,
	.dma	= SPI_DMA_OFF,
	.pol	= SPI_CSW_8CLK | SPI_CK_DLY_ON |
			SPI_CK_POL_POS | SPI_CS_POL_POS,
	.tiecs	= SPI_TIECS_NORMAL,
	.nbw	= SPI_NB_16BIT,
	.nbr	= 0,
	.sclk	= SPI_SCLK_12MHZ,
};

#define PWC_REG_RESERVED          0x00
#define PWC_REG_WRITEONLY         0x01
#define PWC_REG_READONLY          0x02
#define PWC_REG_WRITENOMODIFY     0x04

#define PWC_REG_READWRITE         (PWC_REG_WRITEONLY | PWC_REG_READONLY)
#define PWC_REG_READWRITENOMODIFY (PWC_REG_READWRITE | PWC_REG_WRITENOMODIFY)

/* GPIO port */
#define PWC_GPI_INTERNAL   0x00
#define PWC_GPI_INPORT     0x01
#define PWC_GPO_OPEN_DRAIN 0x02
#define PWC_GPO_PUSH_PULL  0x03

#define REG_PAGE1_BASE 0
#define REG_PAGE2_BASE 128
#define REG_MAX        REG_PAGE2_BASE

static const char pwc_rw_table[] = {
	PWC_REG_READONLY,		/* R0 PAGE_CON */
					/* PAGE_CON is r/w, bat disable write */
	PWC_REG_READONLY,		/* R1 STATUS_A */
	PWC_REG_READONLY,		/* R2 STATUS_B */
	PWC_REG_READONLY,		/* R3 STATUS_C */
	PWC_REG_READONLY,		/* R4 STATUS_D */
	PWC_REG_READWRITENOMODIFY,	/* R5 EVENT_A */
	PWC_REG_READWRITENOMODIFY,	/* R6 EVENT_B */
	PWC_REG_READWRITENOMODIFY,	/* R7 EVENT_C */
	PWC_REG_READWRITENOMODIFY,	/* R8 EVENT_D */
	PWC_REG_READWRITENOMODIFY,	/* R9 FAULT_LOG */
	PWC_REG_READWRITE,		/* R10 IRQ_MASK_A */
	PWC_REG_READWRITE,		/* R11 IRQ_MASK_B */
	PWC_REG_READWRITE,		/* R12 IRQ_MASK_C */
	PWC_REG_READWRITE,		/* R13 IRQ_MASK_D */
	PWC_REG_READWRITE,		/* R14 CONTROL_A */
	PWC_REG_READWRITE,		/* R15 CONTROL_B */
	PWC_REG_READWRITE,		/* R16 CONTROL_C */
	PWC_REG_READWRITE,		/* R17 CONTROL_D */
	PWC_REG_READWRITE,		/* R18 PD_DIS */
	PWC_REG_READWRITE,		/* R19 INTERFACE */  /* ? */
	PWC_REG_READWRITE,		/* R20 RESET */
	PWC_REG_READWRITE,		/* R21 GPIO_0-1 */
	PWC_REG_READWRITE,		/* R22 GPIO_2-3 */
	PWC_REG_READWRITE,		/* R23 GPIO_4-5 */
	PWC_REG_READWRITE,		/* R24 GPIO_6-7 */
	PWC_REG_READWRITE,		/* R25 GPIO_8-9 */
	PWC_REG_READWRITE,		/* R26 GPIO_10-11 */
	PWC_REG_READWRITE,		/* R27 GPIO_12-13 */
	PWC_REG_READWRITE,		/* R28 GPIO_14-15 */
				/* GPIO_14-15 bit3 read only */
	PWC_REG_READWRITE,		/* R29 ID_0_1 */
	PWC_REG_READWRITE,		/* R30 ID_2_3 */
	PWC_REG_READWRITE,		/* R31 ID_4_5 */
	PWC_REG_READWRITE,		/* R32 ID_6_7 */
	PWC_REG_READWRITE,		/* R34 ID_10_ */
	PWC_REG_READWRITE,		/* R34 ID_10_11 */
	PWC_REG_READWRITE,		/* R35 ID_12_13 */
	PWC_REG_READWRITE,		/* R36 ID_14_15 */
	PWC_REG_READWRITE,		/* R37 ID_16_17 */
	PWC_REG_READWRITE,		/* R38 ID_18_19 */
	PWC_REG_READWRITE,		/* R39 ID_20_21 */
	PWC_REG_READWRITE,		/* R40 SEQ_STATUS */
	PWC_REG_READWRITE,		/* R41 SEQ_A */
	PWC_REG_READWRITE,		/* R42 SEQ_B */
	PWC_REG_READWRITE,		/* R43 SEQ_TIMER */
	PWC_REG_READWRITE,		/* R44 BUCK_A */
	PWC_REG_READWRITE,		/* R45 BUCK_B */
	PWC_REG_READWRITE,		/* R46 BUCKCORE */
	PWC_REG_READWRITE,		/* R47 BUCKPRO */
	PWC_REG_READWRITE,		/* R48 BUCKMEM */
	PWC_REG_READWRITE,		/* R49 BUCKPERI */
				/* BUCKPERI bit6(BPERI_EN) read only*/
	PWC_REG_READWRITE,		/* R50 LDO1 */
				/* BUCKPERI bit5 read only*/
	PWC_REG_READWRITE,		/* R51 LDO2 */
	PWC_REG_READWRITE,		/* R52 LDO3 */
	PWC_REG_READWRITE,		/* R53 LDO4 */
	PWC_REG_READWRITE,		/* R54 LDO5 */
	PWC_REG_READWRITE,		/* R55 LDO6 */
	PWC_REG_READWRITE,		/* R56 LDO7 */
	PWC_REG_READWRITE,		/* R57 LDO8 */
	PWC_REG_READWRITE,		/* R58 LDO9 */
	PWC_REG_READWRITE,		/* R59 LDO10 */
	PWC_REG_READWRITE,		/* R60 SUPPLY */
	PWC_REG_READWRITE,		/* R61 PULLDOWN */
	PWC_REG_READWRITE,		/* R62 CHG_BUCK */
	PWC_REG_READWRITE,		/* R63 WAIT_CONT */
	PWC_REG_READWRITE,		/* R64 ISET */
	PWC_REG_READWRITE,		/* R65 BAT_CHG */
	PWC_REG_READWRITE,		/* R66 CHG_CONT */
	PWC_REG_READWRITE,		/* R67 INPUT_CONT */
	PWC_REG_READONLY,		/* R68 CHG_TIME */
	PWC_REG_READWRITE,		/* R69 BBAT_CONT */
	PWC_REG_READWRITE,		/* R70 BOOST */
				/* BOOST bit7(E_B_FAULT) read only*/
	PWC_REG_READWRITE,		/* R71 LED_CONT */
				/* LED_CONT bit7 read only*/
	PWC_REG_READWRITE,		/* R72 LEDMIN */
	PWC_REG_READWRITE,		/* R73 LED1_CONF */
	PWC_REG_READWRITE,		/* R74 LED2_CONF */
	PWC_REG_READWRITE,		/* R75 LED3_CONF */
	PWC_REG_READWRITE,		/* R76 LED1_CONT */
	PWC_REG_READWRITE,		/* R77 LED2_CONT */
	PWC_REG_READWRITE,		/* R78 LED3_CONT */
	PWC_REG_READWRITE,		/* R79 LED4_CONT */
	PWC_REG_READWRITE,		/* R80 LED5_CONT */
	PWC_REG_READWRITE,		/* R81 ADC_MAN */
				/* ADC_MAN bit7-5 read only */
	PWC_REG_READWRITE,		/* R82 ADC_CONT */
	PWC_REG_READONLY,		/* R83 ADC_RES_L */
	PWC_REG_READONLY,		/* R84 ADC_RES_H */
	PWC_REG_READONLY,		/* R85 VDD_RES */
	PWC_REG_READWRITE,		/* R86 VDD_MON */
	PWC_REG_READONLY,		/* R87 ICHG_AV */
	PWC_REG_READWRITE,		/* R88 ICHG_THD */
	PWC_REG_READWRITE,		/* R89 ICHG_END */
	PWC_REG_READONLY,		/* R90 TBAT_RES */
	PWC_REG_READWRITE,		/* R91 TBAT_HIGHP */
	PWC_REG_READWRITE,		/* R92 TBAT_HIGHN */
	PWC_REG_READWRITE,		/* R93 TBAT_LOW */
	PWC_REG_READWRITE,		/* R94 T_OFFSET */
	PWC_REG_READONLY,		/* R95 ADCIN4_RES */
	PWC_REG_READWRITE,		/* R96 AUTO4_HIGH */
	PWC_REG_READWRITE,		/* R97 AUTO4_LOW */
	PWC_REG_READONLY,		/* R98 ADCIN5_RES */
	PWC_REG_READWRITE,		/* R99 AUTO5_HIGH */
	PWC_REG_READWRITE,		/* R100 AUTO5_LOW */
	PWC_REG_READONLY,		/* R101 ADCIN6_RES */
	PWC_REG_READWRITE,		/* R102 AUTO6_HIGH */
	PWC_REG_READWRITE,		/* R103 AUTO6_LOW */
	PWC_REG_READONLY,		/* R104 TJUNC_RES */
	PWC_REG_READWRITE,		/* R105 TSI_CONT_A */
	PWC_REG_READWRITE,		/* R106 TSI_CONT_B */
	PWC_REG_READONLY,		/* R107 TSI_X_MSB */
	PWC_REG_READONLY,		/* R108 TSI_Y_MSB */
	PWC_REG_READONLY,		/* R109 TSI_LSB */
	PWC_REG_READONLY,		/* R110 TSI_Z_MSB */
	PWC_REG_READWRITE,		/* R111 COUNT_S */
				/* bit7 read only*/
	PWC_REG_READWRITE,		/* R112 COUNT_MI */
				/* COUNT_MI bit7-6 read only*/
	PWC_REG_READWRITE,		/* R113 COUNT_H */
				/* COUNT_H bit7-5 read only*/
	PWC_REG_READWRITE,		/* R114 COUNT_D */
				/* COUNT_D bit7-5 read only*/
	PWC_REG_READWRITE,		/* R115 COUNT_MO */
				/* COUNT_MO bit7-4 read only*/
	PWC_REG_READWRITE,		/* R116 COUNT_Y */
	PWC_REG_READWRITE,		/* R117 ALARM_MI */
	PWC_REG_READWRITE,		/* R118 ALARM_H */
				/* ALARM_H bit7-5 read only */
	PWC_REG_READWRITE,		/* R119 ALARM_D */
				/* ALARM_D bit7-5 read only */
	PWC_REG_READWRITE,		/* R120 ALARM_MO */
				/* ALARM_MO bit7-4 read only */
	PWC_REG_READWRITE,		/* R121 ALARM_Y */
	PWC_REG_READONLY,		/* R122 SECOND_A */
	PWC_REG_READONLY,		/* R123 SECOND_B */
	PWC_REG_READONLY,		/* R124 SECOND_C */
	PWC_REG_READONLY,		/* R125 SECOND_D */
	PWC_REG_RESERVED,		/*   */
	PWC_REG_RESERVED,		/*   */
	PWC_REG_READWRITE,		/* R128 PAGE_CON */
	PWC_REG_READONLY,		/* R129 CHIP_ID */
	PWC_REG_READONLY,		/* R130 CONFIG_ID */
	PWC_REG_READWRITE,		/* R131 OTP_CONT */
		/* OTP_CONT bit5-4(bit5 OTP_GP_LOCK) read only */
	PWC_REG_READWRITE,		/* R132 OSC_TRIM */
	PWC_REG_READWRITE,		/* R133 GP_ID_0 */
	PWC_REG_READWRITE,		/* R134 GP_ID_1 */
	PWC_REG_READWRITE,		/* R135 GP_ID_2 */
	PWC_REG_READWRITE,		/* R136 GP_ID_3 */
	PWC_REG_READWRITE,		/* R137 GP_ID_4 */
	PWC_REG_READWRITE,		/* R138 GP_ID_5 */
	PWC_REG_READWRITE,		/* R139 GP_ID_6 */
	PWC_REG_READWRITE,		/* R140 GP_ID_7 */
	PWC_REG_READWRITE,		/* R141 GP_ID_8 */
	PWC_REG_READWRITE,		/* R142 GP_ID_9 */
};

static const unsigned short pwc_gpio_reg[] = {
	DA9052_GPIO0001_REG, DA9052_GPIO0203_REG,
	DA9052_GPIO0405_REG, DA9052_GPIO0607_REG,
	DA9052_GPIO0809_REG, DA9052_GPIO1011_REG,
	DA9052_GPIO1213_REG, DA9052_GPIO1415_REG
};

/* PWC access functions */
int pwc_reg_read(unsigned char addr, unsigned char *data)
{
	int ret = 0;

	if (addr >= REG_MAX)
		return -EINVAL;
	if (data == NULL)
		return -EINVAL;

	if (pwc_rw_table[addr] & PWC_REG_READONLY) {
		addr = (addr << 1) | 0x1;
		ret = spi_cmd_read(&device0_config_read, &addr, data, 0);
		if (ret < 0) {
			printk(KERN_INFO
				"pwc_reg_read(): spi_read error(addr=0x%02x)\n",
				addr >> 1);
			return ret;
		}
	} else {
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(pwc_reg_read);

int pwc_reg_write(unsigned char addr, unsigned char data)
{
	int ret = 0;
	char buf[2];

	if (addr >= REG_MAX)
		return -EINVAL;

	if (pwc_rw_table[addr] & PWC_REG_WRITEONLY) {
		addr = (addr << 1) & 0xFE;
		buf[0] = (char)data;
		buf[1] = (char)addr;
		ret = spi_write(&device0_config_write, buf, 0, 2, 0);
		if (ret < 0) {
			printk(KERN_INFO
				"pwc_reg_write(): spi_write error. %d\n", ret);
			return ret;
		}
	} else {
		printk(KERN_INFO
			"pwc_reg_write(): Read Only error(addr = 0x%02x)\n",
			addr);
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(pwc_reg_write);

int pwc_read(unsigned short offset, unsigned int *data)
{
	if (data == NULL)
		return -EINVAL;
	*data = 0;
	return pwc_reg_read(offset, (unsigned char *)data);
}
EXPORT_SYMBOL(pwc_read);

int pwc_write(unsigned short offset, unsigned int data, unsigned int mask)
{
	int ret;
	unsigned long flags;
	unsigned char tmp_data;

	spin_lock_irqsave(&pwc_write_spinlock, flags);
	ret = pwc_reg_read(offset, &tmp_data);
	if (ret != 0) {
		spin_unlock_irqrestore(&pwc_write_spinlock, flags);
		return ret;
	}
	if (pwc_rw_table[offset] & PWC_REG_WRITENOMODIFY)
		tmp_data = data & mask;
	else if (pwc_rw_table[offset] & PWC_REG_WRITEONLY)
		tmp_data = (tmp_data & (~mask)) | (data & mask);
	ret = pwc_reg_write(offset, tmp_data);
	spin_unlock_irqrestore(&pwc_write_spinlock, flags);

	return ret;
}
EXPORT_SYMBOL(pwc_write);


int pwc_set_direction(unsigned gpio, int is_input)
{
	unsigned int mask;
	unsigned char addr;
	unsigned char data;

	/* When set outport, set "Open drain". */
	switch (gpio) {
	case GPIO_PWC_P0:
	case GPIO_PWC_P1:
	case GPIO_PWC_P2:
	case GPIO_PWC_P8:
	case GPIO_PWC_P9:
	case GPIO_PWC_P10:
	case GPIO_PWC_P12:
	case GPIO_PWC_P14:
	case GPIO_PWC_P15:
		data = (is_input ? PWC_GPI_INPORT : PWC_GPO_OPEN_DRAIN);
		break;

	case GPIO_PWC_P3:
	case GPIO_PWC_P4:
	case GPIO_PWC_P5:
	case GPIO_PWC_P6:
	case GPIO_PWC_P7:
	case GPIO_PWC_P11:
	case GPIO_PWC_P13:
		data = (is_input ? PWC_GPI_INTERNAL : PWC_GPO_OPEN_DRAIN);
		break;
	default:
		return -EINVAL;
	}

	mask = 0x03;
	addr = pwc_gpio_reg[(gpio - GPIO_PWC_BASE) / 2];
	if ((gpio - GPIO_PWC_BASE) & 0x01) {
		mask = mask << 4;
		data = data << 4;
	}
	return pwc_write(addr, data, mask);
}
EXPORT_SYMBOL(pwc_set_direction);

int pwc_get_value(unsigned int gpio)
{
	int ret = -EINVAL;
	unsigned char data = 0, mask = 0, addr;

	if ((GPIO_PWC_BASE <= gpio) && (gpio <= GPIO_PWC_LAST)) {
		if (gpio < GPIO_PWC_P8) {
			mask = 1U << (gpio - GPIO_PWC_P0);
			addr = DA9052_EVENTC_REG;
		} else {
			mask = 1U << (gpio - GPIO_PWC_P8);
			addr = DA9052_EVENTD_REG;
		}
		ret = pwc_reg_read(addr, &data);
	}

	return ret ? ret : ((data & mask) ? 1 : 0);
}
EXPORT_SYMBOL(pwc_get_value);

void pwc_set_value(unsigned int gpio, int value)
{
	unsigned int mask;
	unsigned char addr;

	if ((GPIO_PWC_BASE <= gpio) && (gpio <= GPIO_PWC_LAST)) {
		addr = pwc_gpio_reg[(gpio - GPIO_PWC_BASE) / 2];

		if ((gpio - GPIO_PWC_BASE) & 0x01)
			mask = 0x80;
		else
			mask = 0x08;

		pwc_write(addr, value ? mask : 0, mask);
	}
}
EXPORT_SYMBOL(pwc_set_value);


/* PWC interrupt functions */
static void emxx_pwc_mask_irq(unsigned int irq)
{
	unsigned int port;
	unsigned char addr;

	if ((irq < INT_PWC_BASE) || (INT_PWC_LAST < irq)) {
		printk("emxx_pwc_mask_irq return\n");
		return;
	}

	if (irq < INT_PWC_MASK2_BASE) {
		port = irq - INT_PWC_MASK1_BASE;
		addr = DA9052_IRQMASKA_REG;
		DPRINT("EVENT A mask port = %d\n", port);
	} else if (irq < INT_PWC_MASK3_BASE) {
		port = irq - INT_PWC_MASK2_BASE;
		addr = DA9052_IRQMASKB_REG;
		DPRINT("EVENT B mask port = %d\n", port);
	} else if (irq < INT_PWC_MASK4_BASE) {
		port = irq - INT_PWC_MASK3_BASE;
		addr = DA9052_IRQMASKC_REG;
		DPRINT("EVENT C mask port = %d\n", port);
	} else {
		port = irq - INT_PWC_MASK4_BASE;
		addr = DA9052_IRQMASKD_REG;
		DPRINT("EVENT D mask port = %d\n", port);
	}
	pwc_write(addr, (1 << port), (1 << port));
}

static void emxx_pwc_unmask_irq(unsigned int irq)
{
	unsigned int port;
	unsigned char addr;

	if ((irq < INT_PWC_BASE) || (INT_PWC_LAST < irq))
		return;

	if (irq < INT_PWC_MASK2_BASE) {
		port = irq - INT_PWC_MASK1_BASE;
		addr = DA9052_IRQMASKA_REG;
		DPRINT("EVENT A unmask port = %d\n", port);
	} else if (irq < INT_PWC_MASK3_BASE) {
		port = irq - INT_PWC_MASK2_BASE;
		addr = DA9052_IRQMASKB_REG;
		DPRINT("EVENT B unmask port = %d\n", port);
	} else if (irq < INT_PWC_MASK4_BASE) {
		port = irq - INT_PWC_MASK3_BASE;
		addr = DA9052_IRQMASKC_REG;
		DPRINT("EVENT C unmask port = %d\n", port);
	} else {
		port = irq - INT_PWC_MASK4_BASE;
		addr = DA9052_IRQMASKD_REG;
		DPRINT("EVENT D unmask port = %d\n", port);
	}
	pwc_write(addr, 0 , (1 << port));
}

static void emxx_pwc_ack_irq(unsigned int irq)
{
	unsigned int port = 0;

	if (irq < INT_PWC_MASK2_BASE) {
		port = irq - INT_PWC_MASK1_BASE;
		DPRINT("EVENT A clear port = %d\n", port);
		pwc_write(DA9052_EVENTA_REG, (1 << port), (1 << port));
	} else if (irq < INT_PWC_MASK3_BASE) {
		port = irq - INT_PWC_MASK2_BASE;
		DPRINT("EVENT B clear port = %d\n", port);
		pwc_write(DA9052_EVENTB_REG, (1 << port), (1 << port));
	} else if (irq < INT_PWC_MASK4_BASE) {
		port = irq - INT_PWC_MASK3_BASE;
		pwc_write(DA9052_EVENTC_REG, (1 << port), (1 << port));
		DPRINT("EVENT C clear port = %d\n", port);
	} else {
		port = irq - INT_PWC_MASK4_BASE;
		DPRINT("EVENT D clear port = %d\n", port);
		pwc_write(DA9052_EVENTD_REG, (1 << port), (1 << port));
	}
}

static void emxx_pwc_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	int i;
	unsigned int pwc_factor1_stat = 0;
	unsigned int pwc_factor2_stat = 0;
	unsigned int pwc_factor3_stat = 0;
	unsigned int pwc_factor4_stat = 0;
	unsigned int pwc_factor1_mask = 0;
	unsigned int pwc_factor2_mask = 0;
	unsigned int pwc_factor3_mask = 0;
	unsigned int pwc_factor4_mask = 0;
	unsigned int pwc_irq;
	struct irq_desc *d;

	DPRINT("entry\n");

	if (irq != INT_PWC)
		return;

	desc->chip->ack(irq);

	pwc_read(DA9052_EVENTA_REG, &pwc_factor1_stat);
	pwc_read(DA9052_EVENTB_REG, &pwc_factor2_stat);
	pwc_read(DA9052_EVENTC_REG, &pwc_factor3_stat);
	pwc_read(DA9052_EVENTD_REG, &pwc_factor4_stat);
	pwc_read(DA9052_IRQMASKA_REG, &pwc_factor1_mask);
	pwc_read(DA9052_IRQMASKB_REG, &pwc_factor2_mask);
	pwc_read(DA9052_IRQMASKC_REG, &pwc_factor3_mask);
	pwc_read(DA9052_IRQMASKD_REG, &pwc_factor4_mask);

	pwc_factor1_stat &= ~pwc_factor1_mask;
	pwc_factor2_stat &= ~pwc_factor2_mask;
	pwc_factor3_stat &= ~pwc_factor3_mask;
	pwc_factor4_stat &= ~pwc_factor4_mask;

	if (pwc_factor1_stat != 0) {
		DPRINT("DA9052_EVENTA_REG = 0x%02x\n", pwc_factor1_stat);
		/* IO interrupt */
		for (i = 0; i <= 7; i++) {
			if ((pwc_factor1_stat & (1 << i)) != 0) {
				pwc_irq = INT_PWC_MASK1_BASE + i;
				d = irq_desc + pwc_irq;
				d->handle_irq(pwc_irq, d);
			}
		}
	}
	if (pwc_factor2_stat != 0) {
		DPRINT("DA9052_EVENTB_REG = 0x%02x\n", pwc_factor2_stat);
		/* IO interrupt */
		for (i = 0; i <= 7; i++) {
			if ((pwc_factor2_stat & (1 << i)) != 0) {
				pwc_irq = INT_PWC_MASK2_BASE + i;
				d = irq_desc + pwc_irq;
				d->handle_irq(pwc_irq, d);
			}
		}
	}
	if (pwc_factor3_stat != 0) {
		DPRINT("DA9052_EVENTC_REG = 0x%02x\n", pwc_factor3_stat);
		/* IO interrupt */
		for (i = 0; i <= 7; i++) {
			if ((pwc_factor3_stat & (1 << i)) != 0) {
				pwc_irq = INT_PWC_MASK3_BASE + i;
				d = irq_desc + pwc_irq;
				d->handle_irq(pwc_irq, d);
			}
		}
	}
	if (pwc_factor4_stat != 0) {
		DPRINT("DA9052_EVENTD_REG = 0x%02x\n", pwc_factor4_stat);
		/* IO interrupt */
		for (i = 0; i <= 7; i++) {
			if ((pwc_factor4_stat & (1 << i)) != 0) {
				pwc_irq = INT_PWC_MASK4_BASE + i;
				d = irq_desc + pwc_irq;
				d->handle_irq(pwc_irq, d);
			}
		}
	}

	desc->chip->unmask(irq);
	DPRINT("exit\n");
}


static int emxx_pwc_set_irq_type(unsigned int irq, unsigned int type);

static struct irq_chip emxx_pwc_chip = {
	.name    = "PWC",
	.ack = emxx_pwc_ack_irq,
	.mask = emxx_pwc_mask_irq,
	.unmask = emxx_pwc_unmask_irq,
	.disable = emxx_pwc_mask_irq,
	.set_type = emxx_pwc_set_irq_type,
};


/* called from set_irq_type() */
static int emxx_pwc_set_irq_type(unsigned int irq, unsigned int type)
{
	unsigned int port;
	unsigned int mode = 0;
	unsigned long flags;
	unsigned int mode_mask = 0x04;
	unsigned int pwc_mask;
	unsigned short pwc_mode_reg;
	unsigned short pwc_mask_reg;
	unsigned short pwc_clear_reg;

	if ((irq < INT_PWC_BASE) || (INT_PWC_LAST < irq))
		return -EINVAL;
	else if (irq < INT_PWC_MASK3_BASE)
		return 0;

	switch (type) {
	case IRQ_TYPE_LEVEL_HIGH:
		mode = 0x04;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		mode = 0x00;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&pwc_spinlock, flags);

	pwc_mode_reg = pwc_gpio_reg[(irq - INT_PWC_MASK3_BASE) / 2];
	if ((irq - INT_PWC_MASK3_BASE) & 0x01) {
		mode = mode << 4;
		mode_mask = mode_mask << 4;
	}
	if (irq < INT_PWC_MASK4_BASE) {
		port = 1 << (irq - INT_PWC_MASK3_BASE);
		pwc_mask_reg = DA9052_IRQMASKC_REG;
		pwc_clear_reg = DA9052_EVENTC_REG;
	} else {
		port = 1 << (irq - INT_PWC_MASK4_BASE);
		pwc_mask_reg = DA9052_IRQMASKD_REG;
		pwc_clear_reg = DA9052_EVENTD_REG;
	}
	DPRINT("mode reg 0x%02x, mask reg 0x%02x, clear reg 0x%02x, port %d\n",
			pwc_mode_reg, pwc_mask_reg, pwc_clear_reg,
			irq - INT_PWC_MASK1_BASE);
	DPRINT("mode 0x%02x, mode mask reg 0x%02x, \n",
			mode, mode_mask);

	/* IRQMASKA enable -> disable */
	pwc_read(pwc_mask_reg, &pwc_mask);
	if ((pwc_mask & port) == 0)
		pwc_write(pwc_mask_reg, port, port);

	/* set int type */
	pwc_write(pwc_mode_reg, mode, mode_mask);

	udelay(60);
	pwc_write(pwc_clear_reg, port, port);

	/* Restore if changed */
	if ((pwc_mask & port) == 0)
		pwc_write(pwc_mask_reg, 0, port);

	spin_unlock_irqrestore(&pwc_spinlock, flags);

	return 0;
}


static int __init emxx_pwc_init(void)
{
	int i;

	/* setup default PWC Interrupt modes */
	for (i = INT_PWC_BASE; i <= INT_PWC_LAST; i++) {
		set_irq_chip(i, &emxx_pwc_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	/* Initialize PWC registers */

	/* GPIO_00 set GPI(active low) */
	pwc_write(DA9052_GPIO0001_REG, PWC_GPI_INPORT, 0x07);

	pwc_reg_write(DA9052_LDO5_REG, 0x66); /* LCD power ON */

	/* All Mask */
	pwc_reg_write(DA9052_IRQMASKA_REG, 0xff);
	pwc_reg_write(DA9052_IRQMASKB_REG, 0xff);
	pwc_reg_write(DA9052_IRQMASKC_REG, 0xff);
	pwc_reg_write(DA9052_IRQMASKD_REG, 0xff);

	/* All Clear */
	pwc_reg_write(DA9052_EVENTA_REG, 0xff);
	pwc_reg_write(DA9052_EVENTB_REG, 0xff);
	pwc_reg_write(DA9052_EVENTC_REG, 0xff);
	pwc_reg_write(DA9052_EVENTD_REG, 0xff);

	/* Initialize PWC INT */
	set_irq_type(INT_PWC, IRQ_TYPE_EDGE_FALLING);
	set_irq_chained_handler(INT_PWC, emxx_pwc_irq_handler);

	return 0;
}

module_init(emxx_pwc_init);

