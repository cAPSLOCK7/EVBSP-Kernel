/*
 *  File Name       : arch/arm/mach-emxx/light.c
 *  Function        : LED
 *  Release Version : Ver 1.00
 *  Release Date    : 2010/02/19
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>

#include <linux/gpio.h>
#include <mach/pwc.h>

#define DEFAULT_BACKLIGHT_BRIGHTNESS 255
#define HW_MAX_BRIGHTNESS 95
#define HW_MIN_BRIGHTNESS 16

/*******************************/
/* LCD BackLight */
static void
emxx_brightness_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	int hw_val = (value * HW_MAX_BRIGHTNESS) / 255;

	if (hw_val < HW_MIN_BRIGHTNESS)
		hw_val = HW_MIN_BRIGHTNESS;
	if (hw_val > HW_MAX_BRIGHTNESS)
		hw_val = HW_MAX_BRIGHTNESS;
	pwc_reg_write(DA9052_LED1CONT_REG, hw_val);
	pwc_reg_write(DA9052_LED2CONT_REG, hw_val);
}

static struct led_classdev emxx_backlight_led = {
	.name = "lcd-backlight",
	.brightness = DEFAULT_BACKLIGHT_BRIGHTNESS,
	.brightness_set = emxx_brightness_set,
};


/*******************************/
/* LED */
static struct gpio_led emxx_led_list[] = {
	{
		.name = "led1",
		.gpio = GPIO_PWC_LED1,
	},
	{
		.name = "led2",
		.gpio = GPIO_PWC_LED2,
	},
};
static struct gpio_led_platform_data emxx_leds_data = {
	.num_leds	= ARRAY_SIZE(emxx_led_list),
	.leds		= emxx_led_list,
};
static struct platform_device emxx_leds = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &emxx_leds_data,
	},
};

/*******************************/
static int emxx_light_probe(struct platform_device *pdev)
{
	/* Init LCD Backlight */
	pwc_reg_write(DA9052_LED1CONF_REG, 0xEA);  /* LCD panel max electric
						      current 15mA, set
						      14.986mA */
	pwc_reg_write(DA9052_LED1CONT_REG, 0xDF);
	pwc_reg_write(DA9052_LED2CONF_REG, 0xEA);  /* LCD panel max electric
						      current 15mA, set
						      14.986mA */
	pwc_reg_write(DA9052_LED2CONT_REG, 0xDF);
	led_classdev_register(&pdev->dev, &emxx_backlight_led);

	/* Init LED */
	platform_device_register(&emxx_leds);

	return 0;
}

static int emxx_light_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&emxx_backlight_led);

	platform_device_unregister(&emxx_leds);

	return 0;
}

static struct platform_driver emxx_light_driver = {
	.probe		= emxx_light_probe,
	.remove		= emxx_light_remove,
	.driver		= {
		.name	= "emxx-light",
		.owner	= THIS_MODULE,
	},
};

static int __init emxx_light_init(void)
{
	return platform_driver_register(&emxx_light_driver);
}

late_initcall(emxx_light_init);

