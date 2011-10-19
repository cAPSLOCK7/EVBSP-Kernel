/*
 *  File Name       : arch/arm/mach-emxx/pwc_ext.c
 *  Function        : pwc
 *  Release Version : Ver 1.00
 *  Release Date    : 2011/12/24
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
 */

#ifdef CONFIG_EMGR_TI_PMIC
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

#define PWC_EXT_GPIO_NUM (8)

static DEFINE_SPINLOCK(pwc_ext_spinlock);
static DEFINE_SPINLOCK(pwc_ext_write_spinlock);
static unsigned int pwc_ext_irq_type[PWC_EXT_GPIO_NUM];

static SPI_CONFIG device_ext_config_read = {
	.dev	= SPI_DEV_SP0,
	.cs_sel	= SPI_CS_SEL_CS0,
	.m_s	= SPI_M_S_MASTER,
	.dma	= SPI_DMA_OFF,
	.pol	= SPI_CSW_8CLK | SPI_CK_DLY_ON |
			SPI_CK_POL_POS | SPI_CS_POL_POS,
	.tiecs	= SPI_TIECS_NORMAL,
	.nbw	= SPI_NB_8BIT,
	.nbr	= SPI_NB_8BIT,
#if 0
	.sclk	= SPI_SCLK_12MHZ,
#else
	.sclk	= SPI_SCLK_6MHZ,
#endif
};

static SPI_CONFIG device_ext_config_write = {
	.dev	= SPI_DEV_SP0,
	.cs_sel	= SPI_CS_SEL_CS0,
	.m_s	= SPI_M_S_MASTER,
	.dma	= SPI_DMA_OFF,
	.pol	= SPI_CSW_8CLK | SPI_CK_DLY_ON |
			SPI_CK_POL_POS | SPI_CS_POL_POS,
	.tiecs	= SPI_TIECS_NORMAL,
	.nbw	= SPI_NB_16BIT,
	.nbr	= 0,
#if 0
	.sclk	= SPI_SCLK_12MHZ,
#else
	.sclk	= SPI_SCLK_6MHZ,
#endif
};


#define PWC_REG_RESERVED  0x00
#define PWC_REG_WRITEONLY 0x01
#define PWC_REG_READONLY  0x02
#define PWC_REG_NOMODIFY  0x04

#define PWC_REG_READWRITE         (PWC_REG_WRITEONLY | PWC_REG_READONLY)
#define PWC_REG_WRITENOMODIFY     (PWC_REG_WRITEONLY | PWC_REG_NOMODIFY)

static const char pwc_ext_rw_table[] = {
	PWC_REG_RESERVED,	/* 0x00	Reserved */
	PWC_REG_READONLY,	/* 0x01	TI_DIE_REV_REG */
	PWC_REG_WRITEONLY,	/* 0x02	TI_SFT_RST_REG */
	PWC_REG_READONLY,	/* 0x03	Reserved */
	PWC_REG_WRITEONLY,	/* 0x04	TI_LDOCTRL1_REG */
	PWC_REG_READONLY,	/* 0x05	TI_LDOCTRL1_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x06	TI_LDOCTRL2_REG */
	PWC_REG_READONLY,	/* 0x07	TI_LDOCTRL2_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x08	TI_LDOCTRL3_REG */
	PWC_REG_READONLY,	/* 0x09	TI_LDOCTRL3_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x0a	TI_DCDCCTRL_REG */
	PWC_REG_READONLY,	/* 0x0b	TI_DCDCCTRL_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x0c	TI_DCDC1_VSET_REG */
	PWC_REG_READONLY,	/* 0x0d	TI_DCDC1_VSET_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x0e	TI_DCDC2_VSET_REG */
	PWC_REG_READONLY,	/* 0x0f	TI_DCDC2_VSET_RD_REG */
	PWC_REG_WRITENOMODIFY,	/* 0x10	TI_SEQCTRL_REG */
	PWC_REG_RESERVED,	/* 0x11	Reserved */
	PWC_REG_WRITEONLY,	/* 0x12	TI_CHG_SETTING_REG */
	PWC_REG_READONLY,	/* 0x13	TI_CHG_SETTING_RD_REG */
	PWC_REG_RESERVED,	/* 0x14	Reserved */
	PWC_REG_READONLY,	/* 0x15	TI_CHGINT_STATE_REG */
	PWC_REG_RESERVED,	/* 0x16	Reserved */
	PWC_REG_READONLY,	/* 0x17	TI_MISCINT_STATE_REG */
	PWC_REG_WRITENOMODIFY,	/* 0x18	TI_IOINT_ACK_REG */
	PWC_REG_READONLY,	/* 0x19	TI_IOINT_FACTOR_REG */
	PWC_REG_WRITENOMODIFY,	/* 0x1a	TI_RTCINT_ACK_REG */
	PWC_REG_READONLY,	/* 0x1b	TI_RTCINT_FACTOR_REG */
	PWC_REG_WRITENOMODIFY,	/* 0x1c	TI_CHGINT_ACK_REG */
	PWC_REG_READONLY,	/* 0x1d	TI_CHGINT_FACTOR_REG */
	PWC_REG_WRITENOMODIFY,	/* 0x1e	TI_MISCINT_ACK_REG */
	PWC_REG_READONLY,	/* 0x1f	TI_MISCINT_FACTOR_REG */
	PWC_REG_WRITENOMODIFY,	/* 0x20	TI_ADCINT_ACK1_REG */
	PWC_REG_READONLY,	/* 0x21	TI_ADCINT_FACTOR1_REG */
	PWC_REG_WRITENOMODIFY,	/* 0x22	TI_ADCINT_ACK2_REG */
	PWC_REG_READONLY,	/* 0x23	TI_ADCINT_FACTOR2_REG */
	PWC_REG_WRITEONLY,	/* 0x24	TI_IO_MASK_REG */
	PWC_REG_READONLY,	/* 0x25	TI_IO_MASK_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x26	TI_RTC_MASK_REG */
	PWC_REG_READONLY,	/* 0x27	TI_RTC_MASK_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x28	TI_CHG_MASK_REG */
	PWC_REG_READONLY,	/* 0x29	TI_CHG_MASK_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x2a	TI_MISC_MASK_REG */
	PWC_REG_READONLY,	/* 0x2b	TI_MISC_MASK_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x2c	TI_ADC_MASK1_REG */
	PWC_REG_READONLY,	/* 0x2d	TI_ADC_MASK1_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x2e	TI_ADC_MASK2_REG */
	PWC_REG_READONLY,	/* 0x2f	TI_ADC_MASK2_RD_REG */
	PWC_REG_RESERVED,	/* 0x30	Reserved */
	PWC_REG_READONLY,	/* 0x31	TI_GPIOSTATE_REG */
	PWC_REG_WRITEONLY,	/* 0x32	TI_GPIODIR_REG */
	PWC_REG_READONLY,	/* 0x33	TI_GPIODIR_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x34	TI_GPIOOUT_REG */
	PWC_REG_READONLY,	/* 0x35	TI_GPIOOUT_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x36	TI_GPIOCHATTIM1_REG */
	PWC_REG_READONLY,	/* 0x37	TI_GPIOCHATTIM1_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x38	TI_GPIOCHATTIM2_REG */
	PWC_REG_READONLY,	/* 0x39	TI_GPIOCHATTIM2_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x3a	TI_GPIOPUPD0TO3_REG */
	PWC_REG_READONLY,	/* 0x3b	TI_GPIOPUPD0TO3_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x3c	TI_GPIOPUPD4TO7_REG */
	PWC_REG_READONLY,	/* 0x3d	TI_GPIOPUPD4TO7_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x3e	TI_SCRATCH_REG */
	PWC_REG_READONLY,	/* 0x3f	TI_SCRATCH_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x40	TI_RTC_CTRL_REG */
	PWC_REG_READONLY,	/* 0x41	TI_RTC_CTRL_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x42	TI_RTC_UPDATE_REG */
	PWC_REG_READONLY,	/* 0x43	TI_RTC_UPDATE_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x44	TI_SEC_REG */
	PWC_REG_READONLY,	/* 0x45	TI_SEC_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x46	TI_MIN_REG */
	PWC_REG_READONLY,	/* 0x47	TI_MIN_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x48	TI_HR_REG */
	PWC_REG_READONLY,	/* 0x49	TI_HR_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x4a	TI_DAY_REG */
	PWC_REG_READONLY,	/* 0x4b	TI_DAY_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x4c	TI_MONTH_REG */
	PWC_REG_READONLY,	/* 0x4d	TI_MONTH_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x4e	TI_YR_REG */
	PWC_REG_READONLY,	/* 0x4f	TI_YR_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x50	TI_WKDAY_REG */
	PWC_REG_READONLY,	/* 0x51	TI_WKDAY_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x52	TI_ALM_SEC_REG */
	PWC_REG_READONLY,	/* 0x53	TI_ALM_SEC_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x54	TI_ALM_MIN_REG */
	PWC_REG_READONLY,	/* 0x55	TI_ALM_MIN_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x56	TI_ALM_HR_REG */
	PWC_REG_READONLY,	/* 0x57	TI_ALM_HR_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x58	TI_ALM_DAY_REG */
	PWC_REG_READONLY,	/* 0x59	TI_ALM_DAY_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x5a	TI_ALM_MONTH_REG */
	PWC_REG_READONLY,	/* 0x5b	TI_ALM_MONTH_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x5c	TI_ALM_YR_REG */
	PWC_REG_READONLY,	/* 0x5d	TI_ALM_YR_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x5e	TI_COMP_MSB_REG */
	PWC_REG_READONLY,	/* 0x5f	TI_COMP_MSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x60	TI_COMP_LSB_REG */
	PWC_REG_READONLY,	/* 0x61	TI_COMP_LSB_RD_REG */
	PWC_REG_RESERVED,	/* 0x62	Reserved */
	PWC_REG_RESERVED,	/* 0x63	Reserved */
	PWC_REG_RESERVED,	/* 0x64	Reserved */
	PWC_REG_RESERVED,	/* 0x65	Reserved */
	PWC_REG_WRITEONLY,	/* 0x66	TI_ADC_CTRL0_REG */
	PWC_REG_READONLY,	/* 0x67	TI_ADC_CTRL0_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x68	TI_ADC_CTRL1_REG */
	PWC_REG_READONLY,	/* 0x69	TI_ADC_CTRL1_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x6a	TI_ADC_AUTO0_REG */
	PWC_REG_READONLY,	/* 0x6b	TI_ADC_AUTO0_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x6c	TI_ADC_AUTO1_REG */
	PWC_REG_READONLY,	/* 0x6d	TI_ADC_AUTO1_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x6e	TI_ADC_AUTO2_REG */
	PWC_REG_READONLY,	/* 0x6f	TI_ADC_AUTO2_RD_REG */
	PWC_REG_RESERVED,	/* 0x70	Reserved */
	PWC_REG_READONLY,	/* 0x71	TI_CH0_DATA_LSB_RD_REG */
	PWC_REG_RESERVED,	/* 0x72	Reserved */
	PWC_REG_READONLY,	/* 0x73	TI_CH0_DATA_MSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x74	TI_CH0_AL_LSB_REG */
	PWC_REG_READONLY,	/* 0x75	TI_CH0_AL_LSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x76	TI_CH0_AL_MSB_REG */
	PWC_REG_READONLY,	/* 0x77	TI_CH0_AL_MSB_RD_REG */
	PWC_REG_RESERVED,	/* 0x78	Reserved */
	PWC_REG_READONLY,	/* 0x79	TI_CH1_DATA_LSB_RD_REG */
	PWC_REG_RESERVED,	/* 0x7a	Reserved */
	PWC_REG_READONLY,	/* 0x7b	TI_CH1_DATA_MSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x7c	TI_CH1_AL_LSB_REG */
	PWC_REG_READONLY,	/* 0x7d	TI_CH1_AL_LSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x7e	TI_CH1_AL_MSB_REG */
	PWC_REG_READONLY,	/* 0x7f	TI_CH1_AL_MSB_RD_REG */
	PWC_REG_RESERVED,	/* 0x80	Reserved */
	PWC_REG_READONLY,	/* 0x81	TI_CH2_DATA_LSB_RD_REG */
	PWC_REG_RESERVED,	/* 0x82	Reserved */
	PWC_REG_READONLY,	/* 0x83	TI_CH2_DATA_MSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x84	TI_CH2_AL_LSB_REG */
	PWC_REG_READONLY,	/* 0x85	TI_CH2_AL_LSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x86	TI_CH2_AL_MSB_REG */
	PWC_REG_READONLY,	/* 0x87	TI_CH2_AL_MSB_RD_REG */
	PWC_REG_RESERVED,	/* 0x88	Reserved */
	PWC_REG_READONLY,	/* 0x89	TI_CH3_DATA_LSB_RD_REG */
	PWC_REG_RESERVED,	/* 0x8a	Reserved */
	PWC_REG_READONLY,	/* 0x8b	TI_CH3_DATA_MSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x8c	TI_CH3_AL_LSB_REG */
	PWC_REG_READONLY,	/* 0x8d	TI_CH3_AL_LSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x8e	TI_CH3_AL_MSB_REG */
	PWC_REG_READONLY,	/* 0x8f	TI_CH3_AL_MSB_RD_REG */
	PWC_REG_RESERVED,	/* 0x90	Reserved */
	PWC_REG_READONLY,	/* 0x91	TI_CH4_DATA_LSB_RD_REG */
	PWC_REG_RESERVED,	/* 0x92	Reserved */
	PWC_REG_READONLY,	/* 0x93	TI_CH4_DATA_MSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x94	TI_CH4_AL_LSB_REG */
	PWC_REG_READONLY,	/* 0x95	TI_CH4_AL_LSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x96	TI_CH4_AL_MSB_REG */
	PWC_REG_READONLY,	/* 0x97	TI_CH4_AL_MSB_RD_REG */
	PWC_REG_RESERVED,	/* 0x98	Reserved */
	PWC_REG_READONLY,	/* 0x99	TI_CH5_DATA_LSB_RD_REG */
	PWC_REG_RESERVED,	/* 0x9a	Reserved */
	PWC_REG_READONLY,	/* 0x9b	TI_CH5_DATA_MSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x9c	TI_CH5_AL_LSB_REG */
	PWC_REG_READONLY,	/* 0x9d	TI_CH5_AL_LSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0x9e	TI_CH5_AL_MSB_REG */
	PWC_REG_READONLY,	/* 0x9f	TI_CH5_AL_MSB_RD_REG */
	PWC_REG_RESERVED,	/* 0xa0	Reserved */
	PWC_REG_READONLY,	/* 0xa1	TI_CH6_DATA_LSB_RD_REG */
	PWC_REG_RESERVED,	/* 0xa2	Reserved */
	PWC_REG_READONLY,	/* 0xa3	TI_CH6_DATA_MSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xa4	TI_CH6_AL_LSB_REG */
	PWC_REG_READONLY,	/* 0xa5	TI_CH6_AL_LSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xa6	TI_CH6_AL_MSB_REG */
	PWC_REG_READONLY,	/* 0xa7	TI_CH6_AL_MSB_RD_REG */
	PWC_REG_RESERVED,	/* 0xa8	Reserved */
	PWC_REG_READONLY,	/* 0xa9	TI_CH7_DATA_LSB_RD_REG */
	PWC_REG_RESERVED,	/* 0xaa	Reserved */
	PWC_REG_READONLY,	/* 0xab	TI_CH7_DATA_MSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xac	TI_CH7_AL_LSB_REG */
	PWC_REG_READONLY,	/* 0xad	TI_CH7_AL_LSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xae	TI_CH7_AL_MSB_REG */
	PWC_REG_READONLY,	/* 0xaf	TI_CH7_AL_MSB_RD_REG */
	PWC_REG_RESERVED,	/* 0xb0	Reserved */
	PWC_REG_READONLY,	/* 0xb1	TI_CH8_DATA_LSB_RD_REG */
	PWC_REG_RESERVED,	/* 0xb2	Reserved */
	PWC_REG_READONLY,	/* 0xb3	TI_CH8_DATA_MSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xb4	TI_CH8_ALL_LSB_REG */
	PWC_REG_READONLY,	/* 0xb5	TI_CH8_ALL_LSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xb6	TI_CH8_ALL_MSB_REG */
	PWC_REG_READONLY,	/* 0xb7	TI_CH8_ALL_MSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xb8	TI_CH8_ALL_LSB_REG */
	PWC_REG_READONLY,	/* 0xb9	TI_CH8_ALL_LSB_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xba	TI_CH8_ALL_MSB_REG */
	PWC_REG_READONLY,	/* 0xbb	TI_CH8_ALL_MSB_RD_REG */
	PWC_REG_RESERVED,	/* 0xbc	Reserved */
	PWC_REG_RESERVED,	/* 0xbd	Reserved */
	PWC_REG_RESERVED,	/* 0xbe	Reserved */
	PWC_REG_RESERVED,	/* 0xbf	Reserved */
	PWC_REG_RESERVED,	/* 0xc0	Reserved */
	PWC_REG_RESERVED,	/* 0xc1	Reserved */
	PWC_REG_RESERVED,	/* 0xc2	Reserved */
	PWC_REG_RESERVED,	/* 0xc3	Reserved */
	PWC_REG_RESERVED,	/* 0xc4	Reserved */
	PWC_REG_RESERVED,	/* 0xc5	Reserved */
	PWC_REG_RESERVED,	/* 0xc6	Reserved */
	PWC_REG_RESERVED,	/* 0xc7	Reserved */
	PWC_REG_RESERVED,	/* 0xc8	Reserved */
	PWC_REG_RESERVED,	/* 0xc9	Reserved */
	PWC_REG_RESERVED,	/* 0xca	Reserved */
	PWC_REG_RESERVED,	/* 0xcb	Reserved */
	PWC_REG_RESERVED,	/* 0xcc	Reserved */
	PWC_REG_RESERVED,	/* 0xcd	Reserved */
	PWC_REG_RESERVED,	/* 0xce	Reserved */
	PWC_REG_RESERVED,	/* 0xcf	Reserved */
	PWC_REG_WRITEONLY,	/* 0xd0	TI_DIEID1_REG */
	PWC_REG_READONLY,	/* 0xd1	TI_DIEID1_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xd2	TI_DIEID2_REG */
	PWC_REG_READONLY,	/* 0xd3	TI_DIEID2_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xd4	TI_DIEID3_REG */
	PWC_REG_READONLY,	/* 0xd5	TI_DIEID3_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xd6	TI_DIEID4_REG */
	PWC_REG_READONLY,	/* 0xd7	TI_DIEID4_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xd8	TI_TST9_REG */
	PWC_REG_READONLY,	/* 0xd9	TI_TST9_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xda	TI_TST10_REG */
	PWC_REG_READONLY,	/* 0xdb	TI_TST10_RD_REG */
	PWC_REG_RESERVED,	/* 0xdc	Reserved */
	PWC_REG_RESERVED,	/* 0xdd	Reserved */
	PWC_REG_RESERVED,	/* 0xde	Reserved */
	PWC_REG_RESERVED,	/* 0xdf	Reserved */
	PWC_REG_WRITEONLY,	/* 0xe0	TI_TRM1_REG */
	PWC_REG_READONLY,	/* 0xe1	TI_TRM1_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xe2	TI_TRM2_REG */
	PWC_REG_READONLY,	/* 0xe3	TI_TRM2_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xe4	TI_TRM3_REG */
	PWC_REG_READONLY,	/* 0xe5	TI_TRM3_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xe6	TI_TRM4_REG */
	PWC_REG_READONLY,	/* 0xe7	TI_TRM4_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xe8	TI_TRM5_REG */
	PWC_REG_READONLY,	/* 0xe9	TI_TRM5_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xea	TI_TRM6_REG */
	PWC_REG_READONLY,	/* 0xeb	TI_TRM6_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xec	TI_TRM7_REG */
	PWC_REG_READONLY,	/* 0xed	TI_TRM7_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xee	TI_TRM8_REG */
	PWC_REG_READONLY,	/* 0xef	TI_TRM8_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xf0	TI_TST1_REG */
	PWC_REG_READONLY,	/* 0xf1	TI_TST1_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xf2	TI_TST2_REG */
	PWC_REG_READONLY,	/* 0xf3	TI_TST2_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xf4	TI_TST3_REG */
	PWC_REG_READONLY,	/* 0xf5	TI_TST3_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xf6	TI_TST4_REG */
	PWC_REG_READONLY,	/* 0xf7	TI_TST4_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xf8	TI_TST5_REG */
	PWC_REG_READONLY,	/* 0xf9	TI_TST5_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xfa	TI_TST6_REG */
	PWC_REG_READONLY,	/* 0xfb	TI_TST6_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xfc	TI_TST7_REG */
	PWC_REG_READONLY,	/* 0xfd	TI_TST7_RD_REG */
	PWC_REG_WRITEONLY,	/* 0xfe	TI_TST8_REG */
	PWC_REG_READONLY,	/* 0xff	TI_TST8_RD_REG */
};

static struct workqueue_struct *emxx_pwc_ext_workqueue;
static struct delayed_work      emxx_pwc_ext_delayed_work;


/* PWC(ext) access functions */
int pwc_reg_read_ext(unsigned char addr, unsigned char *data)
{
	int ret = 0;

	if (data == NULL)
		return -EINVAL;

	if (pwc_ext_rw_table[addr] & PWC_REG_READONLY) {
		ret = spi_cmd_read(&device_ext_config_read, &addr, data, 0);
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
EXPORT_SYMBOL(pwc_reg_read_ext);

int pwc_reg_write_ext(unsigned char addr, unsigned char data)
{
	int ret = 0;
	char buf[2];

	if (pwc_ext_rw_table[addr] & PWC_REG_WRITEONLY) {
		buf[0] = (char)data;
		buf[1] = (char)addr;
		ret = spi_write(&device_ext_config_write, buf, 0, 2, 0);
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
EXPORT_SYMBOL(pwc_reg_write_ext);

int pwc_read_ext(unsigned short offset, unsigned int *data)
{
	if (data == NULL)
		return -EINVAL;
	*data = 0;
	return pwc_reg_read_ext(offset, (unsigned char *)data);
}
EXPORT_SYMBOL(pwc_read_ext);

int pwc_write_ext(unsigned short offset, unsigned int data, unsigned int mask)
{
	int ret;
	unsigned long flags;
	unsigned char tmp_data;

	if (!(pwc_ext_rw_table[offset] & PWC_REG_WRITEONLY))
		return -EINVAL;

	spin_lock_irqsave(&pwc_ext_write_spinlock, flags);

	if (pwc_ext_rw_table[offset] & PWC_REG_NOMODIFY)
		tmp_data = data & mask;
	else {
		ret = pwc_reg_read_ext(offset + 1, &tmp_data);
		if (ret != 0) {
			spin_unlock_irqrestore(&pwc_ext_write_spinlock, flags);
			return ret;
		}
		tmp_data = (tmp_data & (~mask)) | (data & mask);
	}
	ret = pwc_reg_write_ext(offset, tmp_data);

	spin_unlock_irqrestore(&pwc_ext_write_spinlock, flags);

	return ret;
}
EXPORT_SYMBOL(pwc_write_ext);


int pwc_ext_set_direction(unsigned gpio, int is_input)
{
	unsigned int mask;
	unsigned char data;

	if ((gpio < GPIO_PWC_EXT_BASE) || (GPIO_PWC_EXT_LAST < gpio))
		return  -EINVAL;

	data = (is_input ? 1 : 0) << (gpio - GPIO_PWC_EXT_BASE);
	mask = 1 << (gpio - GPIO_PWC_EXT_BASE);

	return pwc_write_ext(TI_GPIODIR_REG, data, mask);
}
EXPORT_SYMBOL(pwc_ext_set_direction);

int pwc_ext_get_value(unsigned int gpio)
{
	int ret;
	unsigned char data = 0, mask = 0;

	if ((gpio < GPIO_PWC_EXT_BASE) || (GPIO_PWC_EXT_LAST < gpio))
		return -EINVAL;

	mask = 1 << (gpio - GPIO_PWC_EXT_BASE);

	ret = pwc_reg_read_ext(TI_GPIOSTATE_REG, &data);

	return ret ? ret : ((data & mask) ? 1 : 0);
}
EXPORT_SYMBOL(pwc_ext_get_value);

void pwc_ext_set_value(unsigned int gpio, int value)
{
	unsigned char data = 0, mask = 0;

	if ((gpio < GPIO_PWC_EXT_BASE) || (GPIO_PWC_EXT_LAST < gpio))
		return;

	data = (value ? 1 : 0) << (gpio - GPIO_PWC_EXT_BASE);
	mask = 1 << (gpio - GPIO_PWC_EXT_BASE);

	pwc_write_ext(TI_GPIOOUT_REG, data, mask);
}
EXPORT_SYMBOL(pwc_ext_set_value);


/* PWC interrupt functions */
static void emxx_pwc_ext_mask_irq(unsigned int irq)
{
	unsigned int data;
	unsigned char addr;

	if (INT_PWC_EXT_LAST < irq) {
		return;
	} else if (INT_PWC_EXT_ADC2_BASE <= irq) {
		addr = TI_ADC_MASK2_REG;
		data = 1 << (irq - INT_PWC_EXT_ADC2_BASE);
	} else if (INT_PWC_EXT_ADC1_BASE <= irq) {
		addr = TI_ADC_MASK1_REG;
		data = 1 << (irq - INT_PWC_EXT_ADC1_BASE);
	} else if (INT_PWC_EXT_MISC_BASE <= irq) {
		addr = TI_MISC_MASK_REG;
		data = 1 << (irq - INT_PWC_EXT_MISC_BASE);
	} else if (INT_PWC_EXT_CHG_BASE <= irq) {
		addr = TI_RTC_MASK_REG;
		data = 1 << (irq - INT_PWC_EXT_CHG_BASE);
	} else if (INT_PWC_EXT_RTC_BASE <= irq) {
		addr = TI_CHG_MASK_REG;
		data = 1 << (irq - INT_PWC_EXT_RTC_BASE);
	} else if (INT_PWC_EXT_IO_BASE <= irq) {
		addr = TI_IO_MASK_REG;
		data = 1 << (irq - INT_PWC_EXT_IO_BASE);
	} else {
		return;
	}

	pwc_write_ext(addr, data, data);
}

static void emxx_pwc_ext_unmask_irq(unsigned int irq)
{
	unsigned int data;
	unsigned char addr;

	if (INT_PWC_EXT_LAST < irq) {
		return;
	} else if (INT_PWC_EXT_ADC2_BASE <= irq) {
		addr = TI_ADC_MASK2_REG;
		data = 1 << (irq - INT_PWC_EXT_ADC2_BASE);
	} else if (INT_PWC_EXT_ADC1_BASE <= irq) {
		addr = TI_ADC_MASK1_REG;
		data = 1 << (irq - INT_PWC_EXT_ADC1_BASE);
	} else if (INT_PWC_EXT_MISC_BASE <= irq) {
		addr = TI_MISC_MASK_REG;
		data = 1 << (irq - INT_PWC_EXT_MISC_BASE);
	} else if (INT_PWC_EXT_CHG_BASE <= irq) {
		addr = TI_RTC_MASK_REG;
		data = 1 << (irq - INT_PWC_EXT_CHG_BASE);
	} else if (INT_PWC_EXT_RTC_BASE <= irq) {
		addr = TI_CHG_MASK_REG;
		data = 1 << (irq - INT_PWC_EXT_RTC_BASE);
	} else if (INT_PWC_EXT_IO_BASE <= irq) {
		addr = TI_IO_MASK_REG;
		data = 1 << (irq - INT_PWC_EXT_IO_BASE);
	} else {
		return;
	}

	pwc_write_ext(addr, 0, data);
}

static void emxx_pwc_ext_ack_irq(unsigned int irq)
{
	unsigned int data;
	unsigned char addr;

	if (INT_PWC_EXT_LAST < irq) {
		return;
	} else if (INT_PWC_EXT_ADC2_BASE <= irq) {
		addr = TI_ADCINT_ACK2_REG;
		data = 1 << (irq - INT_PWC_EXT_ADC2_BASE);
	} else if (INT_PWC_EXT_ADC1_BASE <= irq) {
		addr = TI_ADCINT_ACK1_REG;
		data = 1 << (irq - INT_PWC_EXT_ADC1_BASE);
	} else if (INT_PWC_EXT_MISC_BASE <= irq) {
		addr = TI_MISCINT_ACK_REG;
		data = 1 << (irq - INT_PWC_EXT_MISC_BASE);
	} else if (INT_PWC_EXT_CHG_BASE <= irq) {
		addr = TI_CHGINT_ACK_REG;
		data = 1 << (irq - INT_PWC_EXT_CHG_BASE);
	} else if (INT_PWC_EXT_RTC_BASE <= irq) {
		addr = TI_RTCINT_ACK_REG;
		data = 1 << (irq - INT_PWC_EXT_RTC_BASE);
	} else if (INT_PWC_EXT_IO_BASE <= irq) {
		addr = TI_IOINT_ACK_REG;
		data = 1 << (irq - INT_PWC_EXT_IO_BASE);
	} else {
		return;
	}

	pwc_write_ext(addr, data, data);
}

static void emxx_pwc_ext_dummy_irq(unsigned int irq)
{
	return;
}

static inline void pwc_handler_call(unsigned int factor, unsigned long irq_base,
					unsigned long irq_last)
{
	struct irq_desc *d;
	unsigned int pwc_irq;
	int i;
	int port_num = irq_last - irq_base + 1;

	if (factor == 0)
		return;

	for (i = 0; i < port_num; i++) {
		if ((factor & (1 << i)) != 0) {
			pwc_irq = irq_base + i;
			d = irq_desc + pwc_irq;
			d->handle_irq(pwc_irq, d);
		}
	}
}

static int pwc_ext_gpio_level(void)
{
	int gpio_stat;

	pwc_read_ext(TI_GPIOSTATE_REG, &gpio_stat);

	if ((system_rev & EMXX_REV_MASK) <= 0x12) {
		unsigned int pwc_factor1_stat = 0;
		unsigned int pwc_factor2_stat = 0;
		unsigned int pwc_factor3_stat = 0;
		unsigned int pwc_factor4_stat = 0;
		unsigned int pwc_factor1_mask = 0;
		unsigned int pwc_factor2_mask = 0;
		unsigned int pwc_factor3_mask = 0;
		unsigned int pwc_factor4_mask = 0;

		gpio_stat &= ~1;

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

		switch (pwc_ext_irq_type[0]) {
		case IRQ_TYPE_EDGE_RISING:
			gpio_stat |= (pwc_factor1_stat || pwc_factor2_stat ||
				pwc_factor3_stat || pwc_factor4_stat) ? 1 : 0;
			break;

		case IRQ_TYPE_EDGE_FALLING:
			gpio_stat |= (pwc_factor1_stat || pwc_factor2_stat ||
				pwc_factor3_stat || pwc_factor4_stat) ? 0 : 1;
			break;
		}
	}
	return gpio_stat;
}

static unsigned int emxx_pwc_get_ioint(void)
{
	int i;
	unsigned int ioint_stat = 0;
	unsigned int gpio_stat = 0;

	pwc_read_ext(TI_IOINT_FACTOR_REG, &ioint_stat);
	gpio_stat = pwc_ext_gpio_level();
	for (i = 0; i < PWC_EXT_GPIO_NUM; i++) {
		if ((ioint_stat & (1 << i)) == 0)
			continue;

		switch (pwc_ext_irq_type[i]) {
		case IRQ_TYPE_EDGE_RISING:
			if ((gpio_stat & (1 << i)) == 0) {
				ioint_stat &= ~(1 << i);
				emxx_pwc_ext_ack_irq(INT_PWC_EXT_IO_BASE + i);
			}
			break;

		case IRQ_TYPE_EDGE_FALLING:
			if ((gpio_stat & (1 << i)) != 0) {
				ioint_stat &= ~(1 << i);
				emxx_pwc_ext_ack_irq(INT_PWC_EXT_IO_BASE + i);
			}
			break;
		}
	}
	return ioint_stat;
}

static void emxx_pwc_ext_gpio_work(struct work_struct *work)
{
	struct irq_desc *d;
	int i;
	unsigned int ioint_stat = 0;
	unsigned int gpio_stat = 0;

	ioint_stat = emxx_pwc_get_ioint();

	if (!ioint_stat)
		goto end;

	pwc_handler_call(ioint_stat, INT_PWC_EXT_IO_BASE, INT_PWC_EXT_IO_LAST);

	msleep(10);

	gpio_stat = pwc_ext_gpio_level();
	for (i = 0; i < PWC_EXT_GPIO_NUM; i++) {
		if ((ioint_stat & (1 << i)) != 0)
			continue;

		switch (pwc_ext_irq_type[i]) {
		case IRQ_TYPE_EDGE_RISING:
			if ((gpio_stat & (1 << i)) == 0)
				emxx_pwc_ext_ack_irq(INT_PWC_EXT_IO_BASE + i);
			break;

		case IRQ_TYPE_EDGE_FALLING:
			if ((gpio_stat & (1 << i)) != 0)
				emxx_pwc_ext_ack_irq(INT_PWC_EXT_IO_BASE + i);
			break;
		}
	}
end:
	d = irq_desc + INT_PWC_EXT;
	d->chip->unmask(INT_PWC_EXT);
}

static void emxx_pwc_ext_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	unsigned int rtcint_stat = 0;
	unsigned int chgint_stat = 0;
	unsigned int miscint_stat = 0;
	unsigned int adcint1_stat = 0;
	unsigned int adcint2_stat = 0;
	unsigned int rtcint_mask = 0;
	unsigned int chgint_mask = 0;
	unsigned int miscint_mask = 0;
	unsigned int adcint1_mask = 0;
	unsigned int adcint2_mask = 0;
	unsigned int ioint_stat = 0;

	if (irq != INT_PWC_EXT)
		return;

	desc->chip->mask(irq);
	pwc_read_ext(TI_RTCINT_FACTOR_REG, &rtcint_stat);
	pwc_read_ext(TI_RTC_MASK_RD_REG, &rtcint_mask);
	pwc_read_ext(TI_CHGINT_FACTOR_REG, &chgint_stat);
	pwc_read_ext(TI_CHG_MASK_RD_REG, &chgint_mask);
	pwc_read_ext(TI_MISCINT_FACTOR_REG, &miscint_stat);
	pwc_read_ext(TI_MISC_MASK_RD_REG, &miscint_mask);
	pwc_read_ext(TI_ADCINT_FACTOR1_REG, &adcint1_stat);
	pwc_read_ext(TI_ADC_MASK1_RD_REG, &adcint1_mask);
	pwc_read_ext(TI_ADCINT_FACTOR2_REG, &adcint2_stat);
	pwc_read_ext(TI_ADC_MASK2_RD_REG, &adcint2_mask);

	if ((system_rev & EMXX_REV_MASK) > 0x12) {
		ioint_stat = emxx_pwc_get_ioint();
		pwc_handler_call(ioint_stat,
			INT_PWC_EXT_IO_BASE, INT_PWC_EXT_IO_LAST);
	} else {
		queue_delayed_work(emxx_pwc_ext_workqueue,
			&emxx_pwc_ext_delayed_work, msecs_to_jiffies(10));
	}

	rtcint_stat &= ~rtcint_mask;
	chgint_stat &= ~chgint_mask;
	miscint_stat &= ~miscint_mask;
	adcint1_stat &= ~adcint1_mask;
	adcint2_stat &= ~adcint2_mask;

	pwc_handler_call(rtcint_stat,
		INT_PWC_EXT_RTC_BASE, INT_PWC_EXT_RTC_LAST);
	pwc_handler_call(chgint_stat,
		INT_PWC_EXT_CHG_BASE, INT_PWC_EXT_CHG_LAST);
	pwc_handler_call(miscint_stat,
		INT_PWC_EXT_MISC_BASE, INT_PWC_EXT_MISC_LAST);
	pwc_handler_call(adcint1_stat,
		INT_PWC_EXT_ADC1_BASE, INT_PWC_EXT_ADC1_LAST);
	pwc_handler_call(adcint2_stat,
		INT_PWC_EXT_ADC2_BASE, INT_PWC_EXT_ADC2_LAST);

	if ((system_rev & EMXX_REV_MASK) > 0x12)
		desc->chip->unmask(irq);
}


/* called from set_irq_type() */
static int emxx_pwc_ext_set_irq_type(unsigned int irq, unsigned int type)
{
	unsigned int port;
	unsigned long flags;
	unsigned int pwc_mask;

	if ((irq < INT_PWC_EXT_BASE) || (INT_PWC_EXT_LAST < irq))
		return -EINVAL;
	else if ((irq < INT_PWC_EXT_IO_BASE) || (INT_PWC_EXT_IO_LAST < irq))
		return 0;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_EDGE_FALLING:
		break;

	default:
		return -EINVAL;
	}

	port = 1 << (irq - INT_PWC_EXT_IO_BASE);

	spin_lock_irqsave(&pwc_ext_spinlock, flags);

	/* IRQMASKA enable -> disable */
	pwc_read_ext(TI_IO_MASK_RD_REG, &pwc_mask);
	if ((pwc_mask & port) == 0)
		pwc_reg_write_ext(TI_IO_MASK_REG, pwc_mask | port);

	pwc_ext_irq_type[irq - INT_PWC_EXT_IO_BASE] = type;
	pwc_reg_write_ext(TI_IOINT_ACK_REG, port);

	/* Restore if changed */
	if ((pwc_mask & port) == 0)
		pwc_reg_write_ext(TI_IO_MASK_REG, pwc_mask & ~port);

	spin_unlock_irqrestore(&pwc_ext_spinlock, flags);

	return 0;
}

static struct irq_chip emxx_pwc_ext_edge = {
	.name     = "PWC EXT edge",
	.ack = emxx_pwc_ext_ack_irq,
	.mask = emxx_pwc_ext_mask_irq,
	.unmask = emxx_pwc_ext_unmask_irq,
	.disable = emxx_pwc_ext_mask_irq,
	.set_type = emxx_pwc_ext_set_irq_type,
};

static struct irq_chip emxx_pwc_ext_level = {
	.name     = "PWC EXT level",
	.ack = emxx_pwc_ext_mask_irq,
	.mask = emxx_pwc_ext_mask_irq,
	.unmask = emxx_pwc_ext_unmask_irq,
	.disable = emxx_pwc_ext_mask_irq,
};

static int __init emxx_pwc_ext_init(void)
{
	int i;

	if ((system_rev & EMXX_REV_MASK) <= 0x12) {
		emxx_pwc_ext_workqueue =
			create_singlethread_workqueue("emgr-pwc-ti");
		INIT_DELAYED_WORK(&emxx_pwc_ext_delayed_work,
			emxx_pwc_ext_gpio_work);
	}

	for (i = 0; i < PWC_EXT_GPIO_NUM; i++)
		pwc_ext_irq_type[i] = IRQ_TYPE_EDGE_FALLING;

	/* setup default PWC Interrupt modes */
	if ((system_rev & EMXX_REV_MASK) <= 0x12)
		emxx_pwc_ext_edge.ack = emxx_pwc_ext_dummy_irq;

	for (i = INT_PWC_EXT_IO_BASE; i <= INT_PWC_EXT_IO_LAST; i++) {
		set_irq_chip(i, &emxx_pwc_ext_edge);
		set_irq_handler(i, handle_edge_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	for (i = INT_PWC_EXT_RTC_BASE; i <= INT_PWC_EXT_LAST; i++) {
		set_irq_chip(i, &emxx_pwc_ext_level);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	/* TI register init */
	pwc_write_ext(TI_GPIODIR_REG, 0x00, 0x01);

	/* All Mask */
	pwc_reg_write_ext(TI_IO_MASK_REG, 0xff);
	pwc_reg_write_ext(TI_RTC_MASK_REG, 0x07);
	pwc_reg_write_ext(TI_CHG_MASK_REG, 0x3f);
	pwc_reg_write_ext(TI_MISC_MASK_REG, 0x3f);
	pwc_reg_write_ext(TI_ADC_MASK1_REG, 0xff);
	pwc_reg_write_ext(TI_ADC_MASK2_REG, 0x07);

	/* All Clear */
	pwc_reg_write_ext(TI_IOINT_ACK_REG, 0xff);
	pwc_reg_write_ext(TI_RTCINT_ACK_REG, 0x07);
	pwc_reg_write_ext(TI_CHGINT_ACK_REG, 0x3f);
	pwc_reg_write_ext(TI_MISCINT_ACK_REG, 0x3f);
	pwc_reg_write_ext(TI_ADCINT_ACK1_REG, 0xff);
	pwc_reg_write_ext(TI_ADCINT_ACK2_REG, 0x07);

	/* Initialize PWC INT */
	set_irq_type(INT_PWC_EXT, IRQ_TYPE_LEVEL_HIGH);
	set_irq_chained_handler(INT_PWC_EXT, emxx_pwc_ext_irq_handler);

	return 0;
}

module_init(emxx_pwc_ext_init);

#endif
