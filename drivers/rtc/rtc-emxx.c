/*
 *  File Name       : drivers/rtc/rtc-emxx.c
 *  Function        : EMXX Real Time Clock interface
 *  Release Version : Ver 1.02
 *  Release Date    : 2010/12/28
 *
 *  Copyright (C) 2010 Renesas Electronics Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/log2.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>

#include <mach/hardware.h>
#include <mach/pwc.h>

#ifndef BCD2BIN
#define BCD2BIN(val)	bcd2bin(val)
#endif
#ifndef BIN2BCD
#define BIN2BCD(val)	bin2bcd(val)
#endif

static DEFINE_SPINLOCK(emxx_rtc_lock);

#ifdef CONFIG_EMGR_TI_PMIC
/* for TI (SN96036) */
/* RTC Interrupt */
#define RTC_INT_ACK		TI_RTCINT_ACK_REG	/* 0x1A */
#define	RTC_INT_STATUS_RD	TI_RTCINT_FACTOR_REG	/* 0x1B */
#define	RTC_INT_MASK		TI_RTC_MASK_REG		/* 0x26 */
#define	RTC_INT_MASK_RD		TI_RTC_MASK_RD_REG	/* 0x27 */

#define RTC_INT_TMR		(1 << 0)
#define RTC_INT_ALM		(1 << 1)
#define RTC_INT_ERR		(1 << 2)

/* RTC Block */
#define	RTC_CTRL		TI_RTC_CTRL_REG		/* 0x40 */
#define	RTC_CTRL_RD		TI_RTC_CTRL_RD_REG	/* 0x41 */
#define	RTC_UPDATE		TI_RTC_UPDATE_REG	/* 0x42 */
#define	RTC_UPDATE_RD		TI_RTC_UPDATE_RD_REG	/* 0x43 */

#define	RTC_COMP_MSB		TI_COMP_MSB_REG		/* 0x5e */
#define	RTC_COMP_MSB_RD		TI_COMP_MSB_RD_REG	/* 0x5f */
#define	RTC_COMP_LSB		TI_COMP_LSB_REG		/* 0x60 */
#define	RTC_COMP_LSB_RD		TI_COMP_LSB_RD_REG	/* 0x61 */

/* RTC_CTRL Register bit info. */
#define RTC_CTRL_EVERY		(3 << 3)    /* EVERY (D(4:3)) */
#define RTC_CTRL_EVERY_SEC	(0 << 3)    /* event occurs every second */
#define RTC_CTRL_EVERY_MIN	(1 << 3)    /* event occurs every minute */
#define RTC_CTRL_EVERY_HOUR	(2 << 3)    /* event occurs every hours */
#define RTC_CTRL_EVERY_DAY	(3 << 3)    /* event occurs every day */
#define RTC_CTRL_MODE_12_24	(1 << 2)    /* MODE_12_24 (D2) */
#define RTC_CTRL_RTC_AL_EN_ON	(1 << 1)    /* RTC_AL_EN (D1) */
#define RTC_CTRL_RTC_EN_ON	(1 << 0)    /* RTC_EN (D0) */

/* RTC_UPDATE Register data info. */
#define RTC_UPDATE_NO		0	/* No udate */
#define RTC_UPDATE_SEC		1	/* Update seconds */
#define RTC_UPDATE_MIN		2	/* Update minutes */
#define RTC_UPDATE_HOUR		3	/* Update hours */
#define RTC_UPDATE_DAY		4	/* Update days */
#define RTC_UPDATE_MONTH	5	/* Update months */
#define RTC_UPDATE_YEAR		6	/* Update years */
#define RTC_UPDATE_WEEK		7	/* Update week day */
#define RTC_UPDATE_ALL		8	/* Update everything */

/* RTC_BCD Block */
#define	RTC_SEC			TI_SEC_REG		/* 0x44 */
#define	RTC_SEC_RD		TI_SEC_RD_REG		/* 0x45 */
#define	RTC_MIN			TI_MIN_REG		/* 0x46 */
#define	RTC_MIN_RD		TI_MIN_RD_REG		/* 0x47 */
#define	RTC_HOUR		TI_HR_REG		/* 0x48 */
#define	RTC_HOUR_RD		TI_HR_RD_REG		/* 0x49 */
#define	RTC_DAY			TI_DAY_REG		/* 0x4a */
#define	RTC_DAY_RD		TI_DAY_RD_REG		/* 0x4b */
#define	RTC_MONTH		TI_MONTH_REG		/* 0x4c */
#define	RTC_MONTH_RD		TI_MONTH_RD_REG		/* 0x4d */
#define	RTC_YEAR		TI_YR_REG		/* 0x4e */
#define	RTC_YEAR_RD		TI_YR_RD_REG		/* 0x4f */
#define	RTC_WEEK		TI_WKDAY_REG		/* 0x50 */
#define	RTC_WEEK_RD		TI_WKDAY_RD_REG		/* 0x51 */
#define	RTC_ALM_SEC		TI_ALM_SEC_REG		/* 0x52 */
#define	RTC_ALM_SEC_RD		TI_ALM_SEC_RD_REG	/* 0x53 */
#define	RTC_ALM_MIN		TI_ALM_MIN_REG		/* 0x54 */
#define	RTC_ALM_MIN_RD		TI_ALM_MIN_RD_REG	/* 0x55 */
#define	RTC_ALM_HOUR		TI_ALM_HR_REG		/* 0x56 */
#define	RTC_ALM_HOUR_RD		TI_ALM_HR_RD_REG	/* 0x57 */
#define	RTC_ALM_DAY		TI_ALM_DAY_REG		/* 0x58 */
#define	RTC_ALM_DAY_RD		TI_ALM_DAY_RD_REG	/* 0x59 */
#define	RTC_ALM_MONTH		TI_ALM_MONTH_REG	/* 0x5a */
#define	RTC_ALM_MONTH_RD	TI_ALM_MONTH_RD_REG	/* 0x5b */
#define	RTC_ALM_YEAR		TI_ALM_YR_REG		/* 0x5c */
#define	RTC_ALM_YEAR_RD		TI_ALM_YR_RD_REG	/* 0x5d */

/* RTC register access macros: */
#define RTC_READ(addr, val) pwc_read_ext((addr), &(val));
#define RTC_WRITE(val, addr) pwc_reg_write_ext((addr), (val)&0xff);

#else	/* CONFIG_EMGR_TI_PMIC */
/* for DA9052 */
#define RTC_SEC			DA9052_COUNTS_REG
#define RTC_SEC_RD		DA9052_COUNTS_REG
#define RTC_MIN			DA9052_COUNTMI_REG
#define RTC_MIN_RD		DA9052_COUNTMI_REG
#define RTC_HOUR		DA9052_COUNTH_REG
#define RTC_HOUR_RD		DA9052_COUNTH_REG
#define RTC_WEEK		0
#define RTC_WEEK_RD		0
#define RTC_DAY			DA9052_COUNTD_REG
#define RTC_DAY_RD		DA9052_COUNTD_REG
#define RTC_MONTH		DA9052_COUNTMO_REG
#define RTC_MONTH_RD		DA9052_COUNTMO_REG
#define RTC_YEAR		DA9052_COUNTY_REG	/* 2000-2063 */
#define RTC_YEAR_RD		DA9052_COUNTY_REG
#define RTC_ALM_MIN		DA9052_ALARMMI_REG
#define RTC_ALM_MIN_RD		DA9052_ALARMMI_REG
#define RTC_ALM_HOUR		DA9052_ALARMH_REG
#define RTC_ALM_HOUR_RD		DA9052_ALARMH_REG
#define RTC_ALM_WEEK		0
#define RTC_ALM_WEEK_RD		0
#define RTC_ALM_DAY		DA9052_ALARMD_REG
#define RTC_ALM_DAY_RD		DA9052_ALARMD_REG
#define RTC_ALM_MONTH		DA9052_ALARMMO_REG
#define RTC_ALM_MONTH_RD	DA9052_ALARMMO_REG
#define RTC_ALM_YEAR		DA9052_ALARMY_REG
#define RTC_ALM_YEAR_RD		DA9052_ALARMY_REG

#define RTC_ALARM_TYPE_ALM	0x40
#define RTC_TICK_TYPE_MIN	0x80
#define RTC_ALARMY_ALARM_ON	0x40
#define RTC_ALARMY_TICK_ON	0x80

/* RTC register access macros: */
#define RTC_READ(addr, val) pwc_read((addr), &(val));
#define RTC_WRITE(val, addr) pwc_reg_write((addr), (val)&0xff);

static unsigned long epoch = 1900;	/* year corresponding to 0x00 */
#endif	/* CONFIG_EMGR_TI_PMIC */

static const unsigned char days_in_mo[] =
    { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

#ifdef CONFIG_EMGR_TI_PMIC
static int rtc_irq_tmr;
static int rtc_irq_alm;
static int p_freq = 60;
static struct rtc_device *rtc_dev;
static struct i2c_client *rtc_client;
#else	/* CONFIG_EMGR_TI_PMIC */
static int rtc_irq;
static int p_freq = 60;
static int write_jiff;
static struct rtc_device *rtc_dev;
static struct i2c_client *rtc_client;
#endif	/* CONFIG_EMGR_TI_PMIC */

static int emxx_backup_rtc_read(struct rtc_time *rtc_tm)
{
	unsigned char val[7];
	int ret;

	ret = i2c_smbus_read_i2c_block_data(rtc_client, 2, 7, val);
	if (ret < 0)
		return ret;

	if (val[0] & 0x80)
		return -EIO;

	val[0] &= 0x7f;
	val[1] &= 0x7f;
	val[2] &= 0x3f;
	val[3] &= 0x3f;
	val[5] &= 0x1f;
	rtc_tm->tm_sec  = bcd2bin(val[0]);
	rtc_tm->tm_min  = bcd2bin(val[1]);
	rtc_tm->tm_hour = bcd2bin(val[2]);
	rtc_tm->tm_mday = bcd2bin(val[3]);
	rtc_tm->tm_mon  = bcd2bin(val[5]);
	rtc_tm->tm_year = bcd2bin(val[6]);

	rtc_tm->tm_year += 100;
	rtc_tm->tm_mon--;

	return 0;
}

static int emxx_backup_rtc_set(struct rtc_time *rtc_tm)
{
	unsigned char val[7];
	int ret;

	ret = i2c_smbus_write_byte_data(rtc_client, 0, 0x20);
	if (ret < 0)
		return ret;

	val[0] = bin2bcd(rtc_tm->tm_sec);
	val[1] = bin2bcd(rtc_tm->tm_min);
	val[2] = bin2bcd(rtc_tm->tm_hour);
	val[3] = bin2bcd(rtc_tm->tm_mday);
	val[4] = 0;
	val[5] = bin2bcd(rtc_tm->tm_mon + 1);
	val[6] = bin2bcd(rtc_tm->tm_year - 100);

	ret = i2c_smbus_write_i2c_block_data(rtc_client, 2, 7, val);
	if (ret < 0)
		return ret;

	return i2c_smbus_write_byte_data(rtc_client, 0, 0);
}

static int emxx_backup_rtc_init(void)
{
	unsigned char val[2] = {0, 0};

	return i2c_smbus_write_i2c_block_data(rtc_client, 0, 2, val);
}

static int emxx_read_time(struct device *dev, struct rtc_time *rtc_tm)
#ifdef CONFIG_EMGR_TI_PMIC
/* for TI (SN96036) */
{
	unsigned long flags;
	unsigned int val[7];
	spin_lock_irqsave(&emxx_rtc_lock, flags);
	RTC_READ(RTC_SEC_RD,	val[0]);
	RTC_READ(RTC_MIN_RD,	val[1]);
	RTC_READ(RTC_HOUR_RD,	val[2]);
	RTC_READ(RTC_DAY_RD,	val[3]);
	RTC_READ(RTC_MONTH_RD,	val[4]);
	RTC_READ(RTC_YEAR_RD,	val[5]);
	spin_unlock_irqrestore(&emxx_rtc_lock, flags);

	rtc_tm->tm_sec  = bcd2bin((unsigned char)val[0]);
	rtc_tm->tm_min  = bcd2bin((unsigned char)val[1]);
	rtc_tm->tm_hour = bcd2bin((unsigned char)val[2]);
	rtc_tm->tm_mday = bcd2bin((unsigned char)val[3]);
	rtc_tm->tm_mon  = bcd2bin((unsigned char)val[4]);
	rtc_tm->tm_year = bcd2bin((unsigned char)val[5]);

	rtc_tm->tm_year += 100;
	rtc_tm->tm_mon--;

	return 0;
}
#else	/* CONFIG_EMGR_TI_PMIC */
/* for DA9052 */
{
	unsigned long flags;

	if (write_jiff) {
		if ((jiffies - write_jiff) < HZ/2) {
			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_timeout(HZ - (jiffies - write_jiff));
		}
		write_jiff = 0;
	}

	spin_lock_irqsave(&emxx_rtc_lock, flags);
	RTC_READ(RTC_SEC_RD, rtc_tm->tm_sec);
	RTC_READ(RTC_MIN_RD, rtc_tm->tm_min);
	RTC_READ(RTC_HOUR_RD, rtc_tm->tm_hour);
	RTC_READ(RTC_DAY_RD, rtc_tm->tm_mday);
	RTC_READ(RTC_MONTH_RD, rtc_tm->tm_mon);
	RTC_READ(RTC_YEAR_RD, rtc_tm->tm_year);
	spin_unlock_irqrestore(&emxx_rtc_lock, flags);

	rtc_tm->tm_sec &= 0x3f;	/* Mask of bit0-bit5 */

	if ((rtc_tm->tm_year + (epoch - 1900)) <= 69)
		rtc_tm->tm_year += 100;
	rtc_tm->tm_mon--;

	return 0;
}
#endif	/* CONFIG_EMGR_TI_PMIC */

static int emxx_set_time(struct device *dev, struct rtc_time *rtc_tm)
#ifdef CONFIG_EMGR_TI_PMIC
/* for TI (SN96036) */
{
	unsigned char mon, day, hrs, min, sec, leap_yr;
	unsigned int yrs;
	unsigned int val;
	unsigned long flags;

	yrs = rtc_tm->tm_year + 1900;
	mon = rtc_tm->tm_mon + 1;	/* tm_mon starts at zero */
	day = rtc_tm->tm_mday;
	hrs = rtc_tm->tm_hour;
	min = rtc_tm->tm_min;
	sec = rtc_tm->tm_sec;

	if ((yrs < 2000) || (yrs > 2099))
		return -EINVAL;

	if ((mon > 12) || (day == 0))
		return -EINVAL;

	leap_yr = ((!(yrs % 4) && (yrs % 100)) || !(yrs % 400));
	if (day > (days_in_mo[mon] + ((mon == 2) && leap_yr)))
		return -EINVAL;
	if ((hrs >= 24) || (min >= 60) || (sec >= 60))
		return -EINVAL;
	yrs -= 2000;

	spin_lock_irqsave(&emxx_rtc_lock, flags);
	RTC_WRITE(bin2bcd(sec),  RTC_SEC);
	RTC_WRITE(bin2bcd(min),  RTC_MIN);
	RTC_WRITE(bin2bcd(hrs),  RTC_HOUR);
	RTC_WRITE(bin2bcd(day),  RTC_DAY);
	RTC_WRITE(bin2bcd(mon),  RTC_MONTH);
	RTC_WRITE(bin2bcd(yrs),  RTC_YEAR);

	RTC_WRITE(RTC_UPDATE_ALL, RTC_UPDATE);
	/* wait update comp. */
	do {
		RTC_READ(RTC_UPDATE, val);
	} while (val);

	spin_unlock_irqrestore(&emxx_rtc_lock, flags);

	emxx_backup_rtc_set(rtc_tm);

	return 0;
}
#else	/* CONFIG_EMGR_TI_PMIC */
/* for DA9052 */
{
	unsigned char mon, day, hrs, min, sec, leap_yr;
	unsigned int yrs;
	unsigned long flags;

	yrs = rtc_tm->tm_year + 1900;
	mon = rtc_tm->tm_mon + 1;	/* tm_mon starts at zero */
	day = rtc_tm->tm_mday;
	hrs = rtc_tm->tm_hour;
	min = rtc_tm->tm_min;
	sec = rtc_tm->tm_sec;

	if ((yrs < 2000) || (yrs > 2063))
		return -EINVAL;

	if ((mon > 12) || (day == 0))
		return -EINVAL;

	leap_yr = ((!(yrs % 4) && (yrs % 100)) || !(yrs % 400));
	if (day > (days_in_mo[mon] + ((mon == 2) && leap_yr)))
		return -EINVAL;
	if ((hrs >= 24) || (min >= 60) || (sec >= 60))
		return -EINVAL;
	yrs -= 2000;

	spin_lock_irqsave(&emxx_rtc_lock, flags);
	RTC_WRITE(mon,  RTC_MONTH);
	RTC_WRITE(day,  RTC_DAY);
	RTC_WRITE(hrs,  RTC_HOUR);
	RTC_WRITE(min,  RTC_MIN);
	RTC_WRITE(sec,  RTC_SEC);
	RTC_WRITE(yrs,  RTC_YEAR);
	spin_unlock_irqrestore(&emxx_rtc_lock, flags);

	write_jiff = jiffies;

	emxx_backup_rtc_set(rtc_tm);

	return 0;
}
#endif	/* CONFIG_EMGR_TI_PMIC */


static inline void emxx_rtc_setaie(unsigned int enable)
#ifdef CONFIG_EMGR_TI_PMIC
/* for TI (SN96036) */
{
	unsigned int rtc_val;
	unsigned int rtc_val_2;

	RTC_READ(RTC_CTRL_RD, rtc_val);
	RTC_READ(RTC_INT_MASK_RD,  rtc_val_2);
	if (enable) {
		rtc_val |= RTC_CTRL_RTC_AL_EN_ON;
		rtc_val_2 &= ~RTC_INT_ALM;
	} else {
		rtc_val &= ~RTC_CTRL_RTC_AL_EN_ON;
		rtc_val_2 |= RTC_INT_ALM;
	}
	RTC_WRITE(rtc_val, RTC_CTRL);
	RTC_WRITE(rtc_val_2, RTC_INT_MASK);
}
#else	/* CONFIG_EMGR_TI_PMIC */
/*for DA9052 */
{
	unsigned int rtc_val;

	RTC_READ(RTC_ALM_YEAR_RD, rtc_val);
	if (enable)
		rtc_val |= RTC_ALARMY_ALARM_ON;
	else
		rtc_val &= ~RTC_ALARMY_ALARM_ON;
	RTC_WRITE(rtc_val, RTC_ALM_YEAR);
}
#endif	/* CONFIG_EMGR_TI_PMIC */

static inline void emxx_rtc_setpie(unsigned int enable)
#ifdef CONFIG_EMGR_TI_PMIC
/* for TI (SN96036) */
{
	unsigned int rtc_val;
	unsigned int rtc_val_2;

	disable_irq(rtc_irq_tmr);

	RTC_READ(RTC_CTRL_RD,  rtc_val);
	RTC_READ(RTC_INT_MASK_RD,  rtc_val_2);
	if (enable) {
		rtc_val &= ~RTC_CTRL_EVERY;
		if (p_freq == 60) {
			/* one minute */
			rtc_val |= RTC_CTRL_EVERY_MIN;
		} else {
			/* one second */
			rtc_val |= RTC_CTRL_EVERY_SEC;
		}
		rtc_val_2 &= ~RTC_INT_TMR;
	} else {
		rtc_val_2 |= RTC_INT_TMR;
	}
	RTC_WRITE(rtc_val, RTC_CTRL);
	RTC_WRITE(rtc_val_2, RTC_INT_MASK);

	udelay(61);
	enable_irq(rtc_irq_tmr);
}
#else	/* CONFIG_EMGR_TI_PMIC */
/*for DA9052 */
{
	unsigned int rtc1_val, rtc2_val;

	disable_irq(rtc_irq);

	RTC_READ(RTC_ALM_MIN_RD,  rtc1_val);
	RTC_READ(RTC_ALM_YEAR_RD, rtc2_val);
	if (enable) {
		rtc2_val |= RTC_ALARMY_TICK_ON;
		if (p_freq == 60) {
			/* one minute */
			rtc1_val |= RTC_TICK_TYPE_MIN;
		} else {
			/* one second */
			rtc1_val &= ~RTC_TICK_TYPE_MIN;
		}
	} else {
		rtc2_val &= ~RTC_ALARMY_TICK_ON;
	}
	RTC_WRITE(rtc1_val, RTC_ALM_MIN);
	RTC_WRITE(rtc2_val, RTC_ALM_YEAR);

	udelay(61);
	enable_irq(rtc_irq);
}
#endif	/* CONFIG_EMGR_TI_PMIC */

static inline int emxx_rtc_setfreq(int freq)
{
	switch (freq) {
	case 1:
	case 60:
		break;
	default:
		return -EINVAL;
	}

	p_freq = freq;

	return 0;
}

static int emxx_rtc_getalarm(struct device *dev, struct rtc_wkalrm *alrm)
#ifdef CONFIG_EMGR_TI_PMIC
/* for TI (SN96036) */
{
	struct rtc_time *alm_tm = &alrm->time;
	unsigned long flags;
	unsigned int val[7];

	spin_lock_irqsave(&emxx_rtc_lock, flags);
	RTC_READ(RTC_ALM_SEC_RD,	val[0]);
	RTC_READ(RTC_ALM_MIN_RD,	val[1]);
	RTC_READ(RTC_ALM_HOUR_RD,	val[2]);
	RTC_READ(RTC_ALM_DAY_RD,	val[3]);
	RTC_READ(RTC_ALM_MONTH_RD,	val[4]);
	RTC_READ(RTC_ALM_YEAR_RD,	val[5]);

	spin_unlock_irqrestore(&emxx_rtc_lock, flags);

	alm_tm->tm_sec  = bcd2bin((unsigned char)val[0]);
	alm_tm->tm_min  = bcd2bin((unsigned char)val[1]);
	alm_tm->tm_hour = bcd2bin((unsigned char)val[2]);
	alm_tm->tm_mday = bcd2bin((unsigned char)val[3]);
	alm_tm->tm_mon  = bcd2bin((unsigned char)val[4]);
	alm_tm->tm_year = bcd2bin((unsigned char)val[5]);

	alm_tm->tm_year += 100;
	alm_tm->tm_mon--;

	return 0;
}
#else	/* CONFIG_EMGR_TI_PMIC */
/*for DA9052 */
{
	struct rtc_time *alm_tm = &alrm->time;
	unsigned long flags;

	spin_lock_irqsave(&emxx_rtc_lock, flags);
	RTC_READ(RTC_ALM_MIN_RD,  alm_tm->tm_min);
	RTC_READ(RTC_ALM_HOUR_RD, alm_tm->tm_hour);
	RTC_READ(RTC_ALM_DAY_RD,   alm_tm->tm_mday);
	RTC_READ(RTC_ALM_MONTH_RD, alm_tm->tm_mon);
	RTC_READ(RTC_ALM_YEAR_RD,  alm_tm->tm_year);
	spin_unlock_irqrestore(&emxx_rtc_lock, flags);

	alm_tm->tm_min  &= 0x3f;		/* Mask of bit0-bit5 */
	alm_tm->tm_year &= 0x3f;		/* Mask of bit0-bit5 */
	alm_tm->tm_mon--;
	alm_tm->tm_year += 100;

	return 0;
}
#endif	/* CONFIG_EMGR_TI_PMIC */

static int emxx_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
#ifdef CONFIG_EMGR_TI_PMIC
/* for TI (SN96036) */
{
	struct rtc_time *alm_tm = &alrm->time;
	unsigned char mon, day, hrs, min, sec, leap_yr;
	unsigned int yrs;
	unsigned long flags;

	yrs = alm_tm->tm_year + 1900;
	mon = alm_tm->tm_mon + 1;	/* tm_mon starts at zero */
	day = alm_tm->tm_mday;
	hrs = alm_tm->tm_hour;
	min = alm_tm->tm_min;
	sec = alm_tm->tm_sec;

	if ((yrs < 2000) || (yrs > 2099))
		return -EINVAL;

	if ((mon > 12) || (day == 0))
		return -EINVAL;

	leap_yr = ((!(yrs % 4) && (yrs % 100)) || !(yrs % 400));
	if (day > (days_in_mo[mon] + ((mon == 2) && leap_yr)))
		return -EINVAL;
	if ((hrs >= 24) || (min >= 60) || (sec >= 60))
		return -EINVAL;

	yrs -= 2000;

	spin_lock_irqsave(&emxx_rtc_lock, flags);
	RTC_WRITE(bin2bcd(sec),  RTC_ALM_SEC);
	RTC_WRITE(bin2bcd(min),  RTC_ALM_MIN);
	RTC_WRITE(bin2bcd(hrs),  RTC_ALM_HOUR);
	RTC_WRITE(bin2bcd(day),  RTC_ALM_DAY);
	RTC_WRITE(bin2bcd(mon),  RTC_ALM_MONTH);
	RTC_WRITE(bin2bcd(yrs),  RTC_ALM_YEAR);
	spin_unlock_irqrestore(&emxx_rtc_lock, flags);

	if (alrm->enabled)
		emxx_rtc_setaie(1);
	else
		emxx_rtc_setaie(0);

	return 0;
}
#else	/* CONFIG_EMGR_TI_PMIC */
/*for DA9052 */
{
	struct rtc_time *alm_tm = &alrm->time;
	unsigned int hrs, min, day, mon, yrs, leap_yr;
	unsigned long flags;

	hrs = alm_tm->tm_hour;
	min = alm_tm->tm_min;
	yrs = alm_tm->tm_year + 1900;
	mon  = alm_tm->tm_mon + 1;
	day  = alm_tm->tm_mday;

	if ((yrs < 2000) || (yrs > 2063))
		return -EINVAL;
	if (hrs >= 24)
		return -EINVAL;
	if (min >= 60)
		return -EINVAL;

	leap_yr = ((!(yrs % 4) && (yrs % 100)) || !(yrs % 400));
	if (day > (days_in_mo[mon] + ((mon == 2) && leap_yr)))
		return -EINVAL;
	if (mon >= 60)
		return -EINVAL;
	yrs -= 2000;

	spin_lock_irqsave(&emxx_rtc_lock, flags);
	RTC_WRITE(hrs,  RTC_ALM_HOUR);
	pwc_write(RTC_ALM_MIN, min, 0x3f);
	RTC_WRITE(day,  RTC_ALM_DAY);
	RTC_WRITE(mon,  RTC_ALM_MONTH);
	pwc_write(RTC_ALM_YEAR, yrs, 0x3f);
	spin_unlock_irqrestore(&emxx_rtc_lock, flags);

	if (alrm->enabled)
		emxx_rtc_setaie(1);
	else
		emxx_rtc_setaie(0);

	return 0;
}
#endif	/* CONFIG_EMGR_TI_PMIC */

int emxx_set_reboot_alarm(void)
{
	p_freq = 1;	/* second */
	emxx_rtc_setpie(1);	/* enable */

	return 0;
}

static irqreturn_t emxx_rtc_interrupt(int irq, void *dev_id)
#ifdef CONFIG_EMGR_TI_PMIC
/* for TI (SN96036) */
{
	struct rtc_device *rtc = dev_id;
	unsigned long events = RTC_IRQF;
	unsigned int int_val;

	RTC_READ(RTC_INT_STATUS_RD,  int_val);
	RTC_WRITE(int_val, RTC_INT_ACK);

	if (int_val & RTC_INT_ALM)
		events |= RTC_AF;

	if (int_val & RTC_INT_TMR)
		events |= RTC_PF;

	rtc_update_irq(rtc, 1, events);

	return IRQ_HANDLED;
}
#else	/* CONFIG_EMGR_TI_PMIC */
/* for DA9052 */
{
	struct rtc_device *rtc = dev_id;
	unsigned long events = RTC_IRQF;
	unsigned int int_val;

	RTC_READ(RTC_ALM_MIN_RD,  int_val);

	if (int_val & RTC_ALARM_TYPE_ALM) {
		/* caused by timer alarm */
		events |= RTC_AF;
		RTC_READ(RTC_ALM_YEAR_RD,  int_val);
		if (int_val & RTC_ALARMY_TICK_ON)
			events |= RTC_PF;
	} else {
		/* caused by TICK */
		events |= RTC_PF;
	}

	rtc_update_irq(rtc, 1, events);

	return IRQ_HANDLED;
}
#endif	/* CONFIG_EMGR_TI_PMIC */


static int emxx_rtc_ioctl(struct device *dev, unsigned int cmd,
 unsigned long arg)
{
	switch (cmd) {
	case RTC_AIE_ON:
	case RTC_AIE_OFF:
		emxx_rtc_setaie((cmd == RTC_AIE_ON) ? 1 : 0);
		break;

	case RTC_PIE_ON:
	case RTC_PIE_OFF:
		emxx_rtc_setpie((cmd == RTC_PIE_ON) ? 1 : 0);
		break;

	case RTC_IRQP_SET:
		return emxx_rtc_setfreq(arg);

	case RTC_IRQP_READ:
		return  put_user(p_freq, (unsigned long __user *)arg);

	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}


static int emxx_rtc_open(struct device *dev)
{
	return 0;
}

static void emxx_rtc_release(struct device *dev)
{
	emxx_rtc_setpie(0);
}

static struct rtc_class_ops emxx_rtcops = {
	.open       = emxx_rtc_open,
	.release    = emxx_rtc_release,
	.ioctl      = emxx_rtc_ioctl,
	.read_time  = emxx_read_time,
	.set_time   = emxx_set_time,
	.read_alarm = emxx_rtc_getalarm,
	.set_alarm  = emxx_rtc_setalarm,
};

static int
emxx_rtc_probe(struct i2c_client *client, const struct i2c_device_id *id)
#ifdef CONFIG_EMGR_TI_PMIC
/* for TI (SN96036) */
{
	struct rtc_time rtc_tm;
	unsigned int rtc2_val;
	int ret;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_I2C_BLOCK)) {
		return -EIO;
	}

	RTC_READ(RTC_CTRL_RD, rtc2_val);
	rtc2_val &= ~RTC_CTRL_RTC_AL_EN_ON;
	rtc2_val |= RTC_CTRL_RTC_EN_ON;
	RTC_WRITE(rtc2_val, RTC_CTRL);

	rtc_client = client;

	emxx_backup_rtc_init();
	if (emxx_backup_rtc_read(&rtc_tm) == 0)
		emxx_set_time(NULL, &rtc_tm);


	rtc_irq_tmr = INT_PWC_EXT_RTCTMR;
	rtc_irq_alm = INT_PWC_EXT_RTCALM;

	rtc_dev = rtc_device_register(client->name, &client->dev,
			&emxx_rtcops, THIS_MODULE);
	if (IS_ERR(rtc_dev)) {
		pr_debug("%s: can't register RTC device, err %ld\n",
			client->name, PTR_ERR(rtc_dev));
		return -EIO;
	}

	ret = request_irq(rtc_irq_tmr, emxx_rtc_interrupt,
			IRQF_DISABLED, "rtc_tmr", rtc_dev);
	if (ret < 0) {
		printk(KERN_ERR "rtc: request_irq error! (%d)\n", rtc_irq_tmr);
		goto err_irq;
	}

	ret = request_irq(rtc_irq_alm, emxx_rtc_interrupt,
			IRQF_DISABLED, "rtc_alm", rtc_dev);
	if (ret < 0) {
		printk(KERN_ERR "rtc: request_irq error! (%d)\n", rtc_irq_alm);
		goto err_irq;
	}

	return 0;

err_irq:
	rtc_device_unregister(rtc_dev);
	return ret;
}
#else	/* CONFIG_EMGR_TI_PMIC */
/* for DA9052 */
{
	struct rtc_time rtc_tm;
	unsigned int rtc2_val;
	int ret;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_I2C_BLOCK)) {
		return -EIO;
	}

	RTC_READ(RTC_ALM_YEAR_RD, rtc2_val);
	rtc2_val &= ~RTC_ALARMY_TICK_ON;
	RTC_WRITE(rtc2_val, RTC_ALM_YEAR);

	rtc_client = client;

	emxx_backup_rtc_init();
	if (emxx_backup_rtc_read(&rtc_tm) == 0)
		emxx_set_time(NULL, &rtc_tm);

	rtc_irq = INT_RTCINT;
	rtc_dev = rtc_device_register(client->name, &client->dev,
			&emxx_rtcops, THIS_MODULE);
	if (IS_ERR(rtc_dev)) {
		pr_debug("%s: can't register RTC device, err %ld\n",
			client->name, PTR_ERR(rtc_dev));
		return -EIO;
	}

	ret = request_irq(rtc_irq, emxx_rtc_interrupt,
			IRQF_DISABLED, "rtc", rtc_dev);
	if (ret < 0) {
		printk(KERN_ERR "rtc: request_irq error! (%d)\n", rtc_irq);
		goto err_irq;
	}

	return 0;

err_irq:
	rtc_device_unregister(rtc_dev);
	return ret;
}
#endif	/* CONFIG_EMGR_TI_PMIC */

static int emxx_rtc_remove(struct i2c_client *client)
#ifdef CONFIG_EMGR_TI_PMIC
/* for TI (SN96036) */
{
	rtc_device_unregister(rtc_dev);
	free_irq(rtc_irq_tmr, rtc_dev);
	free_irq(rtc_irq_alm, rtc_dev);

	return 0;
}
#else	/* CONFIG_EMGR_TI_PMIC */
/* for DA9052 */
{
	rtc_device_unregister(rtc_dev);
	free_irq(rtc_irq, rtc_dev);

	return 0;
}
#endif	/* CONFIG_EMGR_TI_PMIC */

static struct i2c_device_id emxx_rtc_idtable[] = {
	{I2C_SLAVE_RTC_NAME, 0},
	{ }
};

static struct i2c_driver emxx_rtcdrv = {
	.driver = {
		.name	= "emxx-rtc",
		.owner	= THIS_MODULE,
	},
	.id_table	= emxx_rtc_idtable,
	.probe		= emxx_rtc_probe,
	.remove		= __devexit_p(emxx_rtc_remove),
};

static int __init rtc_init(void)
{
	return i2c_add_driver(&emxx_rtcdrv);

}
module_init(rtc_init);

static void __exit rtc_exit(void)
{
	i2c_del_driver(&emxx_rtcdrv);
}
module_exit(rtc_exit);

MODULE_DESCRIPTION("EMXX series RTC Driver");
MODULE_LICENSE("GPL");
