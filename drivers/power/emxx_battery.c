/*
 *  File Name       : drivers/power/emxx_battery.c
 *  Function        : EMMA Mobile series dummy Battery
 *  Release Version : Ver 1.10
 *  Release Date    : 2010/09/10
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

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/delay.h>
#ifdef CONFIG_EMXX_ANDROID
#include <linux/wakelock.h>
#endif
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <mach/charger.h>

#include <mach/pwc.h>
#include <asm/irq.h>
#include "emxx_battery.h"

/* #define EMXX_BATTERY_DEBUG 1 */

/* Set ADC sample rate (30sec)*/
#define ADC_SAMPLE_RATE	30

#ifndef TRUE
#define TRUE    1
#define FALSE   0
#endif

#ifdef EMXX_BATTERY_DEBUG
#define DEBUG_PRINT(FMT, ARGS...) \
		printk(KERN_INFO "%s(): " FMT, __func__ , ##ARGS)
#else
#define DEBUG_PRINT(FMT, ARGS...)
#endif

enum charger_type_t {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
};
#define charger_type_t enum charger_type_t


struct emxx_battery_data {
	struct power_supply battery;
	struct power_supply usb;
	struct power_supply ac;

	charger_type_t charger;

	unsigned int battery_present;
	unsigned int voltage_level;
	unsigned int battery_voltage;
	unsigned int flag_adc_battery;
	int usb_state;
	spinlock_t lock;

#ifdef CONFIG_EMXX_ANDROID
	int lock_status;
	struct wake_lock vbus_suspend_lock;
#else
	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
#endif
};

static struct emxx_battery_data *battery_data;

static enum power_supply_property emxx_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property emxx_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static unsigned int emxx_charger_get_status(void)
{
	unsigned int status = 0;
	/* Charger status */
	pwc_read(DA9052_STATUSA_REG, &status);
	if (status & 0x20)
		battery_data->charger = CHARGER_AC;
	else if (status & 0x40)
		battery_data->charger = CHARGER_USB;
	else
		battery_data->charger = CHARGER_BATTERY;

	battery_data->battery_present = 1;
	return status;
}

static int emxx_battery_get_status(void)
{
	int ret;

	switch (battery_data->charger) {
	case CHARGER_BATTERY:
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case CHARGER_USB:
	case CHARGER_AC:
		if (battery_data->voltage_level == 100)
			ret = POWER_SUPPLY_STATUS_FULL;
		else
			ret = POWER_SUPPLY_STATUS_CHARGING;
		break;
	default:
		ret = POWER_SUPPLY_STATUS_UNKNOWN;
	}
	return ret;
}

static int emxx_battery_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:		/* 0 */
		val->intval = emxx_battery_get_status();
		break;
	case POWER_SUPPLY_PROP_HEALTH:		/* 1 */
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:		/* 2 */
		val->intval = battery_data->battery_present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:	/* 4 */
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:	/* 26 */
		val->intval = battery_data->voltage_level;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

#ifndef CONFIG_EMXX_ANDROID
static unsigned int emxx_charger_select_temperature(
	unsigned int bat_temperature)
{
	unsigned int temp_temperature = 0;
	unsigned int index = 0;

	for (index = 0; index < (NUM_OF_LOOKUP_TABLE - 1); index++) {
		temp_temperature = (temperature_lookup_ref[index] +
					temperature_lookup_ref[index+1]) / 2;
		if (bat_temperature >= temp_temperature)
			continue;
		else
			break;
	}
	return index;
}

static unsigned int emxx_charger_check_voltage(unsigned int access_index,
						unsigned int bat_voltage)
{
	unsigned int voltage_level = 0;
	unsigned int index = 0;

	for (index = 0; index < (LOOKUP_TABLE_SIZE - 1); index++) {
		if ((bat_voltage <=
			vbat_capacity_look_up[access_index][index][0]) &&
			(bat_voltage >
			vbat_capacity_look_up[access_index][index+1][0])) {
			voltage_level =
				vbat_capacity_look_up[access_index][index][1];
			break;
		}
	}
	if ((bat_voltage <=
		vbat_capacity_look_up[access_index][LOOKUP_TABLE_SIZE-1][0]))
		voltage_level =
		vbat_capacity_look_up[access_index][LOOKUP_TABLE_SIZE-1][1];

	if (bat_voltage > vbat_capacity_look_up[access_index][0][0])
		voltage_level = vbat_capacity_look_up[access_index][0][1];

	return voltage_level;
}

static unsigned int emxx_charger_update_status(void)
{
	charger_type_t old_charger = battery_data->charger;
	unsigned int status;

	status = emxx_charger_get_status();
	if (old_charger != battery_data->charger) {
		power_supply_changed(&battery_data->battery);
		power_supply_changed(&battery_data->usb);
		power_supply_changed(&battery_data->ac);
	}
	return status;
}

static irqreturn_t emxx_charger_irq(int irq, void *dev_id)
{
	unsigned int status;

	status = emxx_charger_update_status();
	return IRQ_HANDLED;
}

static void emxx_battery_work(struct work_struct *work)
{
	const int interval = HZ * ADC_SAMPLE_RATE;
	unsigned int adc_tsi = 0;	/* check tsi using ADC */
	unsigned int temp_value = 0;
	unsigned int old_voltage_level;
	unsigned int reg_voltage;
	unsigned int access_index = 0;
	unsigned int value = 0;
	/* retry 8 times for timeout */
	unsigned int man_timeout_cnt = 8;

	old_voltage_level = battery_data->voltage_level;

	/* Clear ADC_EOM bit */
	pwc_read(DA9052_EVENTB_REG, &value);
	pwc_write(DA9052_EVENTB_REG, value & (1 << 5), (1 << 5));

	pwc_read(DA9052_TSICONTA_REG, &adc_tsi);

	/* tsi doesn't use ADC */
	if ((adc_tsi & 0x01) == 0) {
		if (battery_data->flag_adc_battery == FALSE) {
			/* start battery ADC */
			pwc_write(DA9052_ADCMAN_REG, 0x13, 0x1F);
			battery_data->flag_adc_battery = TRUE;

			/* wait 1000ms for ADC complete*/
			do {
				msleep(1000);
				pwc_read(DA9052_ADCMAN_REG, &value);
				man_timeout_cnt--;
				if (man_timeout_cnt == 1) {
					if (!(value & DA9052_ADCMAN_MANCONV))
						break;
					else
						goto err;
				}
			} while (value & DA9052_ADCMAN_MANCONV);
		}
	} else {
		battery_data->flag_adc_battery = FALSE;
	}

	/* get battery voltage */
	if (battery_data->flag_adc_battery == TRUE) {
		battery_data->flag_adc_battery = FALSE;
		pwc_read(DA9052_ADCMAN_REG, &temp_value);
		/* VBAT AD data is valid */
		if ((temp_value & 0xF) == 0x03) {
			/* battery voltage */
			pwc_read(DA9052_ADCRESL_REG, &temp_value);
			pwc_read(DA9052_ADCRESH_REG, &reg_voltage);
			reg_voltage = ((u16)reg_voltage << 2) |
					((u16)temp_value & 0x03);
			battery_data->battery_voltage =
				volt_reg_to_value(reg_voltage);
			/* dummy temperature */
			access_index = emxx_charger_select_temperature(25);
			battery_data->voltage_level =
				emxx_charger_check_voltage(access_index,
						battery_data->battery_voltage);
			DEBUG_PRINT("voltage reg value = %d\n", reg_voltage);
			DEBUG_PRINT("battery_data->battery_voltage = %d\n",
					battery_data->battery_voltage);
			DEBUG_PRINT("access_index = %d\n", access_index);
			DEBUG_PRINT("battery_data->voltage_level= %d\n",
				battery_data->voltage_level);
		}
	}

err:
	/* If status have changed, update the status */
	if (old_voltage_level != battery_data->voltage_level) {
		DEBUG_PRINT("Update charger status...\n\n");
		power_supply_changed(&battery_data->battery);
	}
	queue_delayed_work(battery_data->monitor_wqueue,
			&(battery_data->monitor_work), interval);
}
#endif

static int emxx_power_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	charger_type_t charger;

	charger = battery_data->charger;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:	/* 3 */
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = (charger ==  CHARGER_AC ? 1 : 0);
		else if (psy->type == POWER_SUPPLY_TYPE_USB) {
			if (battery_data->usb_state == EMXX_USB_OFF)
				val->intval = 0;
			else
				val->intval = 1;
		} else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int emxx_battery_usb_state(int state)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&battery_data->lock, flags);

	if (battery_data->usb_state != state) {
		switch (state) {
		case EMXX_USB_DEVICE:
#ifdef CONFIG_EMXX_ANDROID
			if (battery_data->lock_status == 0) {
				battery_data->lock_status = 1;
				wake_lock(&battery_data->vbus_suspend_lock);
			}
#endif
			break;
		case EMXX_USB_DEVICE_100:
#ifdef CONFIG_EMXX_ANDROID
			if (battery_data->lock_status == 0) {
				battery_data->lock_status = 1;
				wake_lock(&battery_data->vbus_suspend_lock);
			}
#endif
			break;
		case EMXX_USB_CHARGER:
#ifdef CONFIG_EMXX_ANDROID
			if (battery_data->lock_status == 0) {
				battery_data->lock_status = 1;
				wake_lock(&battery_data->vbus_suspend_lock);
			}
#endif
			break;
		case EMXX_USB_OFF:
#ifdef CONFIG_EMXX_ANDROID
			if (battery_data->lock_status == 1) {
				battery_data->lock_status = 0;
				wake_unlock(&battery_data->vbus_suspend_lock);
			}
#endif
			break;
		default:
			printk(KERN_ERR "%s: error state. (%d)\n",
				__func__, state);
			ret = -EINVAL;
			goto err;
		}
		battery_data->usb_state = state;
	}
err:
	spin_unlock_irqrestore(&battery_data->lock, flags);
	return ret;
}
EXPORT_SYMBOL(emxx_battery_usb_state);

static int emxx_battery_probe(struct platform_device *pdev)
{
	int ret;
	struct emxx_battery_data *data;
	unsigned int status;

	printk(KERN_INFO "Battery probe...\n");
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	spin_lock_init(&data->lock);

	/* Battey */
	data->battery.name = "battery";
	data->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	data->battery.properties = emxx_battery_props;
	data->battery.num_properties = ARRAY_SIZE(emxx_battery_props);
	data->battery.get_property = emxx_battery_get_property;

	/* USB */
	data->usb.name = "usb";
	data->usb.type = POWER_SUPPLY_TYPE_USB;
	data->usb.properties = emxx_power_props;
	data->usb.num_properties = ARRAY_SIZE(emxx_power_props);
	data->usb.get_property = emxx_power_get_property;

	/* AC */
	data->ac.name = "ac";
	data->ac.type = POWER_SUPPLY_TYPE_MAINS;
	data->ac.properties = emxx_power_props;
	data->ac.num_properties = ARRAY_SIZE(emxx_power_props);
	data->ac.get_property = emxx_power_get_property;

	battery_data = data;

#ifdef CONFIG_EMXX_ANDROID
	/* Dummy battery setting (always 100%) */
	battery_data->voltage_level = 100;
	battery_data->usb_state = EMXX_USB_OFF;
	battery_data->lock_status = 0;

	wake_lock_init(&data->vbus_suspend_lock,
			WAKE_LOCK_SUSPEND, "vbus_suspend");

	status = emxx_charger_get_status();
#else
	/* Dummy battery setting (100%) */
	battery_data->voltage_level = 100;
	battery_data->usb_state = EMXX_USB_OFF;
	battery_data->flag_adc_battery = FALSE;

	status = emxx_charger_get_status();

	ret = request_irq(INT_PWC_E_DCIN_DET, emxx_charger_irq,
		IRQF_SHARED, "charger", data);
	if (ret)
		goto err_irq_ac1;

	ret = request_irq(INT_PWC_E_DCIN_REM, emxx_charger_irq,
		IRQF_SHARED, "charger", data);
	if (ret)
		goto err_irq_ac2;

	ret = request_irq(INT_PWC_E_VBUS_DET, emxx_charger_irq,
		IRQF_SHARED, "charger", data);
	if (ret)
		goto err_irq_usb1;

	ret = request_irq(INT_PWC_E_VBUS_REM, emxx_charger_irq,
		IRQF_SHARED, "charger", data);
	if (ret)
		goto err_irq_usb2;
#endif

	ret = power_supply_register(&pdev->dev, &data->battery);
	if (ret)
		goto err_battery_failed;

	ret = power_supply_register(&pdev->dev, &data->usb);
	if (ret)
		goto err_usb_failed;

	ret = power_supply_register(&pdev->dev, &data->ac);
	if (ret)
		goto err_ac_failed;

#ifndef CONFIG_EMXX_ANDROID
	/* Init DA9052 for Charger */
	pwc_write(DA9052_CHGBUCK_REG, 0x0F, 0x0F);
	pwc_write(DA9052_ISET_REG, 0xF8, 0xFF);
	pwc_write(DA9052_BATCHG_REG, 0x37, 0x3F);
	pwc_write(DA9052_CHGCONT_REG, 0xD1, 0xFF);
	pwc_write(DA9052_BBATCONT_REG, 0x1E, 0xFF);

	INIT_DELAYED_WORK(&data->monitor_work, emxx_battery_work);
	data->monitor_wqueue =
		create_singlethread_workqueue(pdev->dev.bus_id);
	if (!data->monitor_wqueue) {
		ret = -ESRCH;
		goto err_workqueue_failed;
	}
	queue_delayed_work(data->monitor_wqueue, &data->monitor_work, HZ);
#endif

	platform_set_drvdata(pdev, data);
	return 0;

#ifdef CONFIG_EMXX_ANDROID
err_ac_failed:
	power_supply_unregister(&data->usb);
err_usb_failed:
	power_supply_unregister(&data->battery);
err_battery_failed:
	wake_lock_destroy(&data->vbus_suspend_lock);
	kfree(data);
#else
err_workqueue_failed:
	power_supply_unregister(&data->ac);
err_ac_failed:
	power_supply_unregister(&data->usb);
err_usb_failed:
	power_supply_unregister(&data->battery);
err_battery_failed:
	free_irq(INT_PWC_E_VBUS_REM, data);
err_irq_usb2:
	free_irq(INT_PWC_E_VBUS_DET, data);
err_irq_usb1:
	free_irq(INT_PWC_E_DCIN_REM, data);
err_irq_ac2:
	free_irq(INT_PWC_E_DCIN_DET, data);
err_irq_ac1:
	kfree(data);
#endif
err_data_alloc_failed:
	return ret;
}

static int emxx_battery_remove(struct platform_device *pdev)
{
	struct emxx_battery_data *data = platform_get_drvdata(pdev);

	printk(KERN_INFO "Battery driver remove...\n");

#ifdef CONFIG_EMXX_ANDROID
	wake_lock_destroy(&data->vbus_suspend_lock);
#else
	free_irq(INT_PWC_E_DCIN_DET, data);
	free_irq(INT_PWC_E_DCIN_REM, data);
	free_irq(INT_PWC_E_VBUS_DET, data);
	free_irq(INT_PWC_E_VBUS_REM, data);
#endif
	power_supply_unregister(&data->battery);
	power_supply_unregister(&data->usb);
	power_supply_unregister(&data->ac);

	kfree(data);
	battery_data = NULL;
	return 0;
}

#ifdef CONFIG_PM
static int emxx_battery_suspend(struct platform_device *pdev,
				  pm_message_t state)
{
	return 0;
}

static int emxx_battery_resume(struct platform_device *pdev)
{
#ifndef CONFIG_EMXX_ANDROID
	struct emxx_battery_data *data = platform_get_drvdata(pdev);

	power_supply_changed(&data->battery);

	cancel_delayed_work(&data->monitor_work);
	queue_delayed_work(data->monitor_wqueue, &data->monitor_work, HZ);

	battery_data->flag_adc_battery = FALSE;
#endif
	return 0;
}
#else
#define emxx_battery_suspend NULL
#define emxx_battery_resume NULL
#endif /* CONFIG_PM */

static struct platform_driver emxx_battery_driver = {
	.probe		= emxx_battery_probe,
	.remove		= emxx_battery_remove,
	.suspend	= emxx_battery_suspend,
	.resume 	= emxx_battery_resume,
	.driver = {
		.name = "emxx-battery"
	}
};

static int __init emxx_battery_init(void)
{
	return platform_driver_register(&emxx_battery_driver);
}

static void __exit emxx_battery_exit(void)
{
	platform_driver_unregister(&emxx_battery_driver);
}

module_init(emxx_battery_init);
module_exit(emxx_battery_exit);

MODULE_AUTHOR("Renesas");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Battery driver for the EMMA Mobile series");
