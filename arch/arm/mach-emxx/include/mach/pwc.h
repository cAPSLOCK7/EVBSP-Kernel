/*
 *  File Name	    : arch/arm/mach-emxx/include/mach/pwc.h
 *  Function        : pwc
 *  Release Version : Ver 1.01
 *  Release Date    : 2010/12/24
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
#ifndef _ASM_ARCH_PWC_H_
#define _ASM_ARCH_PWC_H_

/* Power register */

/* System control and event registers */
#define DA9052_PAGECON0_REG		(0) /* read/write */
#define DA9052_STATUSA_REG		(1) /* read */
#define DA9052_STATUSB_REG		(2) /* read */
#define DA9052_STATUSC_REG		(3) /* read */
#define DA9052_STATUSD_REG		(4) /* read */
#define DA9052_EVENTA_REG		(5) /* read */
#define DA9052_EVENTB_REG		(6) /* read */
#define DA9052_EVENTC_REG		(7) /* read */
#define DA9052_EVENTD_REG		(8) /* read */
#define DA9052_FAULTLOG_REG		(9) /* read */
#define DA9052_IRQMASKA_REG		(10) /* read/write */
#define DA9052_IRQMASKB_REG		(11) /* read/write */
#define DA9052_IRQMASKC_REG		(12) /* read/write */
#define DA9052_IRQMASKD_REG		(13) /* read/write */
#define DA9052_CONTROLA_REG		(14) /* read/write */
#define DA9052_CONTROLB_REG		(15) /* read/write */
#define DA9052_CONTROLC_REG		(16) /* read/write */
#define DA9052_CONTROLD_REG		(17) /* read/write */
#define DA9052_PDDIS_REG		(18) /* read/write */
#define DA9052_INTERFACE_REG	(19) /* read */
#define DA9052_RESET_REG		(20) /* read/write */

/* GPIO control registers */
#define DA9052_GPIO0001_REG		(21) /* read/write */
#define DA9052_GPIO0203_REG		(22) /* read/write */
#define DA9052_GPIO0405_REG		(23) /* read/write */
#define DA9052_GPIO0607_REG		(24) /* read/write */
#define DA9052_GPIO0809_REG		(25) /* read/write */
#define DA9052_GPIO1011_REG		(26) /* read/write */
#define DA9052_GPIO1213_REG		(27) /* read/write */
#define DA9052_GPIO1415_REG		(28) /* read/write bit3 read only */

/* Power sequencer control registers */
#define DA9052_ID01_REG			(29) /* read/write */
#define DA9052_ID23_REG			(30) /* read/write */
#define DA9052_ID45_REG			(31) /* read/write */
#define DA9052_ID67_REG			(32) /* read/write */
#define DA9052_ID89_REG			(33) /* read/write */
#define DA9052_ID1011_REG		(34) /* read/write */
#define DA9052_ID1213_REG		(35) /* read/write */
#define DA9052_ID1415_REG		(36) /* read/write */
#define DA9052_ID1617_REG		(37) /* read/write */
#define DA9052_ID1819_REG		(38) /* read/write */
#define DA9052_ID2021_REG		(39) /* read/write */
#define DA9052_SEQSTATUS_REG	(40) /* read/write */
#define DA9052_SEQA_REG			(41) /* read/write */
#define DA9052_SEQB_REG			(42) /* read/write */
#define DA9052_SEQTIMER_REG		(43) /* read/write */

/* Power supply control registers */
#define DA9052_BUCKA_REG		(44) /* read/write */
#define DA9052_BUCKB_REG		(45) /* read/write */
#define DA9052_BUCKCORE_REG		(46) /* read/write */
#define DA9052_BUCKPRO_REG		(47) /* read/write */
#define DA9052_BUCKMEM_REG		(48) /* read/write */
#define DA9052_BUCKPERI_REG		(49) /* read/write bit6 read only*/
#define DA9052_LDO1_REG			(50) /* read/write bit5 read only*/
#define DA9052_LDO2_REG			(51) /* read/write */
#define DA9052_LDO3_REG			(52) /* read/write */
#define DA9052_LDO4_REG			(53) /* read/write */
#define DA9052_LDO5_REG			(54) /* read/write */
#define DA9052_LDO6_REG			(55) /* read/write */
#define DA9052_LFO7_REG			(56) /* read/write */
#define DA9052_LDO8_REG			(57) /* read/write */
#define DA9052_LDO9_REG			(58) /* read/write */
#define DA9052_LDO10_REG		(59) /* read/write */
#define DA9052_SUPPLY_REG		(60) /* read/write */
#define DA9052_PULLDOWN_REG		(61) /* read/write */

/* Charging control registers */
#define DA9052_CHGBUCK_REG		(62) /* read/write */
#define DA9052_WAITCONT_REG		(63) /* read/write */
#define DA9052_ISET_REG			(64) /* read/write */
#define DA9052_BATCHG_REG		(65) /* read/write */
#define DA9052_CHGCONT_REG		(66) /* read/write */
#define DA9052_INPUTCONT_REG	(67) /* read */
#define DA9052_CHGTIME_REG		(68) /* read/write */

/* Backup battery charging control registers */
#define DA9052_BBATCONT_REG		(69) /* read/write */

/* Boost and LED driver control registers */
#define DA9052_BOOST_REG		(70) /* read/write bit7 read only*/
#define DA9052_LEDCONT_REG		(71) /* read/write bit7 read only*/
#define DA9052_LEDMIN123_REG	(72) /* read/write */
#define DA9052_LED1CONF_REG		(73) /* read/write */
#define DA9052_LED2CONF_REG		(74) /* read/write */
#define DA9052_LED3CONF_REG		(75) /* read/write */
#define DA9052_LED1CONT_REG		(76) /* read/write */
#define DA9052_LED2CONT_REG		(77) /* read/write */
#define DA9052_LED3CONT_REG		(78) /* read/write */
#define DA9052_LED4CONT_REG		(79) /* read/write */
#define DA9052_LED5CONT_REG		(80) /* read/write */

/* GP-ADC control registers */
#define DA9052_ADCMAN_REG		(81) /* read/write bit7-5 read only*/
#define DA9052_ADCCONT_REG		(82) /* read/write */
#define DA9052_ADCRESL_REG		(83) /* read */
#define DA9052_ADCRESH_REG		(84) /* read */
#define DA9052_VDDRES_REG		(85) /* read */
#define DA9052_VDDMON_REG		(86) /* read/write */
#define DA9052_ICHGAV_REG		(87) /* read */
#define DA9052_ICHGTHD_REG		(88) /* read/write */
#define DA9052_ICHGEND_REG		(89) /* read/write */
#define DA9052_TBATRES_REG		(90) /* read */
#define DA9052_TBATHIGHP_REG		(91) /* read/write */
#define DA9052_TBATHIGHIN_REG		(92) /* read/write */
#define DA9052_TBATLOW_REG		(93) /* read/write */
#define DA9052_TOFFSET_REG		(94) /* read/write */
#define DA9052_ADCIN4RES_REG		(95) /* read */
#define DA9052_AUTO4HIGH_REG		(96) /* read/write */
#define DA9052_AUTO4LOW_REG		(97) /* read/write */
#define DA9052_ADCIN5RES_REG		(98) /* read */
#define DA9052_AUTO5HIGH_REG		(99) /* read/write */
#define DA9052_AUTO5LOW_REG		(100) /* read/write */
#define DA9052_ADCIN6RES_REG		(101) /* read */
#define DA9052_AUTO6HIGH_REG		(102) /* read/write */
#define DA9052_AUTO6LOW_REG		(103) /* read/write */
#define DA9052_TJUNCRES_REG		(104) /* read */

/* TSI control registers */
#define DA9052_TSICONTA_REG		(105) /* read/write */
#define DA9052_TSICONTB_REG		(106) /* read/write */
#define DA9052_TSIXMSB_REG		(107) /* read */
#define DA9052_TSIYMSB_REG		(108) /* read */
#define DA9052_TSILSB_REG		(109) /* read */
#define DA9052_TSIZMSB_REG		(110) /* read */

/* RTC calendar and alarm */
#define DA9052_COUNTS_REG		(111) /* read/write bit7 read only*/
#define DA9052_COUNTMI_REG		(112) /* read/write bit7-6 read only*/
#define DA9052_COUNTH_REG		(113) /* read/write bit7-5 read only*/
#define DA9052_COUNTD_REG		(114) /* read/write bit7-5 read only*/
#define DA9052_COUNTMO_REG		(115) /* read/write bit7-4 read only*/
#define DA9052_COUNTY_REG		(116) /* read/write */
#define DA9052_ALARMMI_REG		(117) /* read/write */
#define DA9052_ALARMH_REG		(118) /* read/write bit7-5 read only */
#define DA9052_ALARMD_REG		(119) /* read/write bit7-5 read only */
#define DA9052_ALARMMO_REG		(120) /* read/write bit7-4 read only */
#define DA9052_ALARMY_REG		(121) /* read/write */
#define DA9052_SECONDA_REG		(122) /* read */
#define DA9052_SECONDB_REG		(123) /* read */
#define DA9052_SECONDC_REG		(124) /* read */
#define DA9052_SECONDD_REG		(125) /* read */

#if 0
/* Customer OTP */
#define DA9052_PAGECON128_REG		(128) /* read/write */
#define DA9052_CHIPID_REG		(129) /* read */
#define DA9052_CONFIGID_REG		(130) /* read */
#define DA9052_OTPCONT_REG		(131) /* read/write bit5-4 read only */
#define DA9052_OSCTRIM_REG		(132) /* read/write */
#define DA9052_GPID0_REG		(133) /* read/write */
#define DA9052_GPID1_REG		(134) /* read/write */
#define DA9052_GPID2_REG		(135) /* read/write */
#define DA9052_GPID3_REG		(136) /* read/write */
#define DA9052_GPID4_REG		(137) /* read/write */
#define DA9052_GPID5_REG		(138) /* read/write */
#define DA9052_GPID6_REG		(139) /* read/write */
#define DA9052_GPID7_REG		(140) /* read/write */
#define DA9052_GPID8_REG		(141) /* read/write */
#define DA9052_GPID9_REG		(142) /* read/write */

#define DA9052_PAGE1_REG_START	(DA9052_CHIPID_REG)
#define DA9052_PAGE1_REG_END	(DA9052_GPID9_REG)
#endif

#define DA9052_PAGE0_REG_START	(DA9052_STATUSA_REG)
#define DA9052_PAGE0_REG_END	(DA9052_SECONDD_REG)

#define PWC_MAX			125



#ifdef CONFIG_EMGR_TI_PMIC
/* REGULATOR Block */
/* Reserved	0x00 */
#define	TI_DIE_REV_REG	0x01
#define	TI_SFT_RST_REG	0x02
/* Reserved	0x03 */
#define	TI_LDOCTRL1_REG	0x04
#define	TI_LDOCTRL1_RD_REG	0x05
#define	TI_LDOCTRL2_REG	0x06
#define	TI_LDOCTRL2_RD_REG	0x07
#define	TI_LDOCTRL3_REG	0x08
#define	TI_LDOCTRL3_RD_REG	0x09
#define	TI_DCDCCTRL_REG	0x0a
#define	TI_DCDCCTRL_RD_REG	0x0b
#define	TI_DCDC1_VSET_REG	0x0c
#define	TI_DCDC1_VSET_RD_REG	0x0d
#define	TI_DCDC2_VSET_REG	0x0e
#define	TI_DCDC2_VSET_RD_REG	0x0f
#define	TI_SEQCTRL_REG	0x10
/* Reserved	0x11 */

/* CHG Block */
#define	TI_CHG_SETTING_REG	0x12
#define	TI_CHG_SETTING_RD_REG	0x13
/* Reserved	0x14 */
#define	TI_CHGINT_STATE_REG	0x15
/* Reserved	0x16 */
#define	TI_MISCINT_STATE_REG	0x17

/* INT Block */
#define	TI_IOINT_ACK_REG	0x18
#define	TI_IOINT_FACTOR_REG	0x19
#define	TI_RTCINT_ACK_REG	0x1a
#define	TI_RTCINT_FACTOR_REG	0x1b
#define	TI_CHGINT_ACK_REG	0x1c
#define	TI_CHGINT_FACTOR_REG	0x1d
#define	TI_MISCINT_ACK_REG	0x1e
#define	TI_MISCINT_FACTOR_REG	0x1f
#define	TI_ADCINT_ACK1_REG	0x20
#define	TI_ADCINT_FACTOR1_REG	0x21
#define	TI_ADCINT_ACK2_REG	0x22
#define	TI_ADCINT_FACTOR2_REG	0x23
#define	TI_IO_MASK_REG	0x24
#define	TI_IO_MASK_RD_REG	0x25
#define	TI_RTC_MASK_REG	0x26
#define	TI_RTC_MASK_RD_REG	0x27
#define	TI_CHG_MASK_REG	0x28
#define	TI_CHG_MASK_RD_REG	0x29
#define	TI_MISC_MASK_REG	0x2a
#define	TI_MISC_MASK_RD_REG	0x2b
#define	TI_ADC_MASK1_REG	0x2c
#define	TI_ADC_MASK1_RD_REG	0x2d
#define	TI_ADC_MASK2_REG	0x2e
#define	TI_ADC_MASK2_RD_REG	0x2f
/* Reserved	0x30 */

/* GPIO Block */
#define	TI_GPIOSTATE_REG	0x31
#define	TI_GPIODIR_REG	0x32
#define	TI_GPIODIR_RD_REG	0x33
#define	TI_GPIOOUT_REG	0x34
#define	TI_GPIOOUT_RD_REG	0x35
#define	TI_GPIOCHATTIM1_REG	0x36
#define	TI_GPIOCHATTIM1_RD_REG	0x37
#define	TI_GPIOCHATTIM2_REG	0x38
#define	TI_GPIOCHATTIM2_RD_REG	0x39
#define	TI_GPIOPUPD0TO3_REG	0x3a
#define	TI_GPIOPUPD0TO3_RD_REG	0x3b
#define	TI_GPIOPUPD4TO7_REG	0x3c
#define	TI_GPIOPUPD4TO7_RD_REG	0x3d
#define	TI_SCRATCH_REG	0x3e
#define	TI_SCRATCH_RD_REG	0x3f

/* RTC Block */
#define	TI_RTC_CTRL_REG	0x40
#define	TI_RTC_CTRL_RD_REG	0x41
#define	TI_RTC_UPDATE_REG	0x42
#define	TI_RTC_UPDATE_RD_REG	0x43

#define	TI_COMP_MSB_REG	0x5e
#define	TI_COMP_MSB_RD_REG	0x5f
#define	TI_COMP_LSB_REG	0x60
#define	TI_COMP_LSB_RD_REG	0x61

/* RTC_BCD Block */
#define	TI_SEC_REG	0x44
#define	TI_SEC_RD_REG	0x45
#define	TI_MIN_REG	0x46
#define	TI_MIN_RD_REG	0x47
#define	TI_HR_REG	0x48
#define	TI_HR_RD_REG	0x49
#define	TI_DAY_REG	0x4a
#define	TI_DAY_RD_REG	0x4b
#define	TI_MONTH_REG	0x4c
#define	TI_MONTH_RD_REG	0x4d
#define	TI_YR_REG	0x4e
#define	TI_YR_RD_REG	0x4f
#define	TI_WKDAY_REG	0x50
#define	TI_WKDAY_RD_REG	0x51
#define	TI_ALM_SEC_REG	0x52
#define	TI_ALM_SEC_RD_REG	0x53
#define	TI_ALM_MIN_REG	0x54
#define	TI_ALM_MIN_RD_REG	0x55
#define	TI_ALM_HR_REG	0x56
#define	TI_ALM_HR_RD_REG	0x57
#define	TI_ALM_DAY_REG	0x58
#define	TI_ALM_DAY_RD_REG	0x59
#define	TI_ALM_MONTH_REG	0x5a
#define	TI_ALM_MONTH_RD_REG	0x5b
#define	TI_ALM_YR_REG	0x5c
#define	TI_ALM_YR_RD_REG	0x5d
/* Reserved	0x62 */
/* Reserved	0x63 */
/* Reserved	0x64 */
/* Reserved	0x65 */

/* BPADC1 Block */
#define	TI_ADC_CTRL0_REG	0x66
#define	TI_ADC_CTRL0_RD_REG	0x67
#define	TI_ADC_CTRL1_REG	0x68
#define	TI_ADC_CTRL1_RD_REG	0x69
#define	TI_ADC_AUTO0_REG	0x6a
#define	TI_ADC_AUTO0_RD_REG	0x6b
#define	TI_ADC_AUTO1_REG	0x6c
#define	TI_ADC_AUTO1_RD_REG	0x6d
#define	TI_ADC_AUTO2_REG	0x6e
#define	TI_ADC_AUTO2_RD_REG	0x6f
/* Reserved	0x70 */
#define	TI_CH0_DATA_LSB_RD_REG	0x71
/* Reserved	0x72 */
#define	TI_CH0_DATA_MSB_RD_REG	0x73
#define	TI_CH0_AL_LSB_REG	0x74
#define	TI_CH0_AL_LSB_RD_REG	0x75
#define	TI_CH0_AL_MSB_REG	0x76
#define	TI_CH0_AL_MSB_RD_REG	0x77
/* Reserved	0x78 */
#define	TI_CH1_DATA_LSB_RD_REG	0x79
/* Reserved	0x7a */
#define	TI_CH1_DATA_MSB_RD_REG	0x7b
#define	TI_CH1_AL_LSB_REG	0x7c
#define	TI_CH1_AL_LSB_RD_REG	0x7d
#define	TI_CH1_AL_MSB_REG	0x7e
#define	TI_CH1_AL_MSB_RD_REG	0x7f
/* Reserved	0x80 */
#define	TI_CH2_DATA_LSB_RD_REG	0x81
/* Reserved	0x82 */
#define	TI_CH2_DATA_MSB_RD_REG	0x83
#define	TI_CH2_AL_LSB_REG	0x84
#define	TI_CH2_AL_LSB_RD_REG	0x85
#define	TI_CH2_AL_MSB_REG	0x86
#define	TI_CH2_AL_MSB_RD_REG	0x87
/* Reserved	0x88 */
#define	TI_CH3_DATA_LSB_RD_REG	0x89
/* Reserved	0x8a */
#define	TI_CH3_DATA_MSB_RD_REG	0x8b
#define	TI_CH3_AL_LSB_REG	0x8c
#define	TI_CH3_AL_LSB_RD_REG	0x8d
#define	TI_CH3_AL_MSB_REG	0x8e
#define	TI_CH3_AL_MSB_RD_REG	0x8f
/* Reserved	0x90 */
#define	TI_CH4_DATA_LSB_RD_REG	0x91
/* Reserved	0x92 */
#define	TI_CH4_DATA_MSB_RD_REG	0x93
#define	TI_CH4_AL_LSB_REG	0x94
#define	TI_CH4_AL_LSB_RD_REG	0x95
#define	TI_CH4_AL_MSB_REG	0x96
#define	TI_CH4_AL_MSB_RD_REG	0x97
/* Reserved	0x98 */
#define	TI_CH5_DATA_LSB_RD_REG	0x99
/* Reserved	0x9a */
#define	TI_CH5_DATA_MSB_RD_REG	0x9b
#define	TI_CH5_AL_LSB_REG	0x9c
#define	TI_CH5_AL_LSB_RD_REG	0x9d
#define	TI_CH5_AL_MSB_REG	0x9e
#define	TI_CH5_AL_MSB_RD_REG	0x9f
/* Reserved	0xa0 */

/* BPADC2 Block */
#define	TI_CH6_DATA_LSB_RD_REG	0xa1
/* Reserved	0xa2 */
#define	TI_CH6_DATA_MSB_RD_REG	0xa3
#define	TI_CH6_AL_LSB_REG	0xa4
#define	TI_CH6_AL_LSB_RD_REG	0xa5
#define	TI_CH6_AL_MSB_REG	0xa6
#define	TI_CH6_AL_MSB_RD_REG	0xa7
/* Reserved	0xa8 */
#define	TI_CH7_DATA_LSB_RD_REG	0xa9
/* Reserved	0xaa */
#define	TI_CH7_DATA_MSB_RD_REG	0xab
#define	TI_CH7_AL_LSB_REG	0xac
#define	TI_CH7_AL_LSB_RD_REG	0xad
#define	TI_CH7_AL_MSB_REG	0xae
#define	TI_CH7_AL_MSB_RD_REG	0xaf
/* Reserved	0xb0 */
#define	TI_CH8_DATA_LSB_RD_REG	0xb1
/* Reserved	0xb2 */
#define	TI_CH8_DATA_MSB_RD_REG	0xb3
#define	TI_CH8_ALL_LSB_REG	0xb4
#define	TI_CH8_ALL_LSB_RD_REG	0xb5
#define	TI_CH8_ALL_MSB_REG	0xb6
#define	TI_CH8_ALL_MSB_RD_REG	0xb7
#define	TI_CH8_ALH_LSB_REG	0xb8
#define	TI_CH8_ALH_LSB_RD_REG	0xb9
#define	TI_CH8_ALH_MSB_REG	0xba
#define	TI_CH8_ALH_MSB_RD_REG	0xbb
/* Reserved	0xbc */
/* Reserved	0xbd */
/* Reserved	0xbe */
/* Reserved	0xbf */
/* Reserved	0xc0 */
/* Reserved	0xc1 */
/* Reserved	0xc2 */
/* Reserved	0xc3 */
/* Reserved	0xc4 */
/* Reserved	0xc5 */
/* Reserved	0xc6 */
/* Reserved	0xc7 */
/* Reserved	0xc8 */
/* Reserved	0xc9 */
/* Reserved	0xca */
/* Reserved	0xcb */
/* Reserved	0xcc */
/* Reserved	0xcd */
/* Reserved	0xce */
/* Reserved	0xcf */

/* TRIM Block */
#define	TI_DIEID1_REG	0xd0
#define	TI_DIEID1_RD_REG	0xd1
#define	TI_DIEID2_REG	0xd2
#define	TI_DIEID2_RD_REG	0xd3
#define	TI_DIEID3_REG	0xd4
#define	TI_DIEID3_RD_REG	0xd5
#define	TI_DIEID4_REG	0xd6
#define	TI_DIEID4_RD_REG	0xd7
/* Reserved	0xdc */
/* Reserved	0xdd */
/* Reserved	0xde */
/* Reserved	0xdf */
#define	TI_TRM1_REG	0xe0
#define	TI_TRM1_RD_REG	0xe1
#define	TI_TRM2_REG	0xe2
#define	TI_TRM2_RD_REG	0xe3
#define	TI_TRM3_REG	0xe4
#define	TI_TRM3_RD_REG	0xe5
#define	TI_TRM4_REG	0xe6
#define	TI_TRM4_RD_REG	0xe7
#define	TI_TRM5_REG	0xe8
#define	TI_TRM5_RD_REG	0xe9
#define	TI_TRM6_REG	0xea
#define	TI_TRM6_RD_REG	0xeb
#define	TI_TRM7_REG	0xec
#define	TI_TRM7_RD_REG	0xed
#define	TI_TRM8_REG	0xee
#define	TI_TRM8_RD_REG	0xef


/* TEST Block */
#define	TI_TST9_REG	0xd8
#define	TI_TST9_RD_REG	0xd9
#define	TI_TST10_REG	0xda
#define	TI_TST10_RD_REG	0xdb

#define	TI_TST1_REG	0xf0
#define	TI_TST1_RD_REG	0xf1
#define	TI_TST2_REG	0xf2
#define	TI_TST2_RD_REG	0xf3
#define	TI_TST3_REG	0xf4
#define	TI_TST3_RD_REG	0xf5
#define	TI_TST4_REG	0xf6
#define	TI_TST4_RD_REG	0xf7
#define	TI_TST5_REG	0xf8
#define	TI_TST5_RD_REG	0xf9
#define	TI_TST6_REG	0xfa
#define	TI_TST6_RD_REG	0xfb
#define	TI_TST7_REG	0xfc
#define	TI_TST7_RD_REG	0xfd
#define	TI_TST8_REG	0xfe
#define	TI_TST8_RD_REG	0xff

#endif


/* GPIO Output/Input select */
#define PWC_GPIO_OUTPUT	0
#define PWC_GPIO_INPUT	1

#ifdef CONFIG_EMXX_SPI0
extern int  pwc_reg_read(unsigned char addr, unsigned char *data);
extern int  pwc_reg_write(unsigned char addr, unsigned char data);
extern int  pwc_read(unsigned short offset, unsigned int *data);
extern int  pwc_write(unsigned short offset,
			unsigned int data, unsigned int mask);
extern int  pwc_set_direction(unsigned gpio, int is_input);
extern int  pwc_get_value(unsigned int gpio);
extern void pwc_set_value(unsigned int gpio, int value);
extern int  pwc_get_output_value(unsigned int gpio);
extern int pwc_ext_set_direction(unsigned gpio, int is_input);
extern int pwc_ext_get_value(unsigned int gpio);
extern void pwc_ext_set_value(unsigned int gpio, int value);
#ifdef CONFIG_EMGR_TI_PMIC
extern int  pwc_reg_read_ext(unsigned char addr, unsigned char *data);
extern int  pwc_reg_write_ext(unsigned char addr, unsigned char data);
extern int  pwc_read_ext(unsigned short offset, unsigned int *data);
extern int  pwc_write_ext(unsigned short offset,
			unsigned int data, unsigned int mask);
#endif
#else
static inline int  pwc_reg_read(unsigned char addr, unsigned char *data)
	{ return 0; }
static inline  int  pwc_reg_write(unsigned char addr, unsigned char data)
	{ return 0; }
static inline  int  pwc_read(unsigned short offset, unsigned int *data)
	{ return 0; }
static inline  int  pwc_write(unsigned short offset,
			unsigned int data, unsigned int mask)
	{ return 0; }
static inline  int  pwc_set_direction(unsigned gpio, int is_input)
	{ return 0; }
static inline  int  pwc_get_value(unsigned int gpio)
	{ return 0; }
static inline  void pwc_set_value(unsigned int gpio, int value) { }
static inline  int  pwc_get_output_value(unsigned int gpio)
	{ return 0; }
#ifdef CONFIG_EMGR_TI_PMIC
static inline int pwc_reg_read_ext(unsigned char addr, unsigned char *data)
	{ return 0; }
static inline int pwc_reg_write_ext(unsigned char addr, unsigned char data)
	{ return 0; }
static inline int pwc_read_ext(unsigned short offset, unsigned int *data)
	{ return 0; }
static inline int pwc_write_ext(unsigned short offset,
			unsigned int data, unsigned int mask)
	{ return 0; }
a
static int pwc_ext_set_direction(unsigned gpio, int is_input)
	{ return 0; }
static int pwc_ext_get_value(unsigned int gpio)
	{ return 0; }
static void pwc_ext_set_value(unsigned int gpio, int value)
	{ return 0; }

#endif
#endif

#endif /* _ASM_ARCH_PWC_H_ */
