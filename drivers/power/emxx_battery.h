/*
 *  File Name       : drivers/power/emxx_battery.c
 *  Function        : EMMA Mobile series dummy Battery
 *  Release Version : Ver 1.00
 *  Release Date    : 2010/08/30
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

#ifndef _EMXX_BATTERY_H_
#define _EMXX_BATTERY_H_


#define DA9052_ADCMAN_MANCONV		(1<<4)

/*
 * Number of lookup table for battery temperature.
 * This define the number of different voltage at which temperature
 * lookup table need to be defined
 */
#define TEMP_TABLE_SIZE			7

/* I = 50uA, Vref = VDDCORE = 2.5V
 * V = Vref * reg_value / 256 = reg_value * 2.5 / 256
 * V = I * Rt = V = reg_value *2.5/256, So,
 *    Rt = reg_value *2.5/(256*50uA)=50*reg_value/256K=0.2reg_value K
*/
/* This Li-TBAT, R->temp, 6.8K->25C */
static const unsigned int temper_lookup[TEMP_TABLE_SIZE][2] =
{
	{60, 10}, {50, 15},
	{41, 20}, {34, 25},
	{28, 30}, {23, 35},
	{20, 40},
};

/*
 * Number of lookup table for BAT Capacity at different battery temperature.
 * This define the number of different temperature at which battery capacity
 * lookup table need to be defined
 */
#define NUM_OF_LOOKUP_TABLE		3
#define LOOKUP_TABLE_SIZE		18
/* BYD Li-ion 3.7V 1200mA */
static const unsigned int
	temperature_lookup_ref[NUM_OF_LOOKUP_TABLE] = {10, 25, 40};
static unsigned int const
	vbat_capacity_look_up[NUM_OF_LOOKUP_TABLE][LOOKUP_TABLE_SIZE][2] =
{
	/* For temperature 10 degree celisus*/
	{
		{4130, 100}, {4080, 96},
		{4030, 92}, {4020, 88},
		{3960, 83}, {3930, 79},
		{3900, 75}, {3880, 71},
		{3840, 67}, {3810, 63},
		{3790, 58}, {3750, 50},
		{3730, 42}, {3700, 33},
		{3680, 25}, {3620, 17},
		{3560, 8}, {3000, 0},
	},

	/* For temperature 25 degree celisus */
	{
		{4150, 100}, {4100, 96},
		{4050, 92}, {4020, 88},
		{3980, 83}, {3950, 79},
		{3920, 75}, {3900, 71},
		{3860, 67}, {3830, 63},
		{3810, 58}, {3770, 50},
		{3750, 42}, {3720, 33},
		{3700, 25}, {3640, 17},
		{3580, 8}, {3000, 0},
	},

	/* For temperature 40 degree celisus*/
	{
		{4170, 100}, {4120, 96},
		{4070, 92}, {4040, 88},
		{4000, 83}, {3970, 79},
		{3940, 75}, {3920, 71},
		{3880, 67}, {3850, 63},
		{3830, 58}, {3790, 50},
		{3770, 42}, {3740, 33},
		{3720, 25}, {3660, 17},
		{3600, 8}, {3000, 0},
	}
};

/*
 * bat_temp_reg_to_C:	Convert battery temperature to degree Celicus
 * this function need be updated in according to the specied Li battery
 */
static inline  unsigned char temp_reg_to_value(unsigned int value)
{
	unsigned int temp = 0;
	unsigned int index = 0;
	for (index = 0; index < (TEMP_TABLE_SIZE - 2); index++) {
		if ((value <= (temper_lookup[index][0] +
				temper_lookup[index+1][0]) / 2) &&
			(value > (temper_lookup[index+1][0] +
				temper_lookup[index+2][0]) / 2)) {
			temp = temper_lookup[index + 1][1];
			break;
		}
	}
	if (value <= (temper_lookup[TEMP_TABLE_SIZE - 1][0] +
		temper_lookup[TEMP_TABLE_SIZE-2][0])/2)
		temp = temper_lookup[TEMP_TABLE_SIZE - 1][1];

	if (value >= (temper_lookup[0][0] + temper_lookup[1][0]) / 2)
		temp = temper_lookup[0][1];

	return temp;
}

/*
 * volt_reg_to_mV: Convert measured voltage to mV.
 *                 For details,refer to table ADC in P33 of PMIC datesheet
 */
static inline  unsigned int volt_reg_to_value(unsigned int value)
{
	return ((value * 1000)/512)+2500;
}

#endif /*_EMXX_BATTERY_H_ */

