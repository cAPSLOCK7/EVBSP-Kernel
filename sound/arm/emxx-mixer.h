/*
 *  File Name		: sound/arm/emxx-mixer.c
 *  Function		: AK4648 CODEC
 *  Release Version 	: Ver 1.00
 *  Release Date	: 2010/04/01
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

#ifndef __EMXX_MIXER_H
#define __EMXX_MIXER_H

#define AK4648REG_POWER_MANAGEMENT_1            0x00
#define AK4648REG_POWER_MANAGEMENT_2            0x01
#define AK4648REG_SIGNAL_SELECT_1               0x02
#define AK4648REG_SIGNAL_SELECT_2               0x03
#define AK4648REG_MODE_CONTROL_1                0x04
#define AK4648REG_MODE_CONTROL_2                0x05
#define AK4648REG_TIMER_SELECT                  0x06
#define AK4648REG_ALC_MODE_CONTROL_1            0x07
#define AK4648REG_ALC_MODE_CONTROL_2            0x08
#define AK4648REG_LCH_INPUT_VOLUME_CONTROL      0x09
#define AK4648REG_LCH_DIGITAL_VOLUME_CONTROL    0x0a
#define AK4648REG_ALC_MODE_CONTROL_3            0x0b
#define AK4648REG_RCH_INPUT_VOLUME_CONTROL      0x0c
#define AK4648REG_RCH_DIGITAL_VOLUME_CONTROL    0x0d
#define AK4648REG_MODE_CONTROL_3                0x0e
#define AK4648REG_MODE_CONTROL_4                0x0f
#define AK4648REG_POWER_MANAGEMENT_3            0x10
#define AK4648REG_DIGITAL_FILTER_SELECT         0x11
#define AK4648REG_FIL3_CO_EFFICIENT_0           0x12
#define AK4648REG_FIL3_CO_EFFICIENT_1           0x13
#define AK4648REG_FIL3_CO_EFFICIENT_2           0x14
#define AK4648REG_FIL3_CO_EFFICIENT_3           0x15
#define AK4648REG_EQ_CO_EFFICIENT_0             0x16
#define AK4648REG_EQ_CO_EFFICIENT_1             0x17
#define AK4648REG_EQ_CO_EFFICIENT_2             0x18
#define AK4648REG_EQ_CO_EFFICIENT_3             0x19
#define AK4648REG_EQ_CO_EFFICIENT_4             0x1a
#define AK4648REG_EQ_CO_EFFICIENT_5             0x1b
#define AK4648REG_FIL1_CO_EFFICIENT_0           0x1c
#define AK4648REG_FIL1_CO_EFFICIENT_1           0x1d
#define AK4648REG_FIL1_CO_EFFICIENT_2           0x1e
#define AK4648REG_FIL1_CO_EFFICIENT_3           0x1f
#define AK4648REG_POWER_MANAGEMENT_4            0x20
#define AK4648REG_MODE_CONTROL_5                0x21
#define AK4648REG_LINEOUT_MIXING_SELECT         0x22
#define AK4648REG_HP_MIXING_SELECT              0x23
#define AK4648REG_SPK_MIXING_SELECT             0x24
#define AK4648REG_EQ_CONTROL_250HZ_100HZ        0x25
#define AK4648REG_EQ_CONTROL_3_5KHZ_1KHZ        0x26
#define AK4648REG_EQ_CONTROL_10KHZ              0x27
#define AK4648REG_MAX                           0x28

#define AK4648BIT_PMSPR         0x80
#define AK4648BIT_PMVCM         0x40
#define AK4648BIT_PMMIN         0x20
#define AK4648BIT_PMSPL         0x10
#define AK4648BIT_PMLO          0x08
#define AK4648BIT_PMDAC         0x04
#define AK4648BIT_PMADL         0x01

#define AK4648BIT_HPZ           0x80
#define AK4648BIT_HPMTN         0x40
#define AK4648BIT_PMHPL         0x20
#define AK4648BIT_PMHPR         0x10
#define AK4648BIT_M_S           0x08
#define AK4648BIT_PMHPC         0x04
#define AK4648BIT_MCKO          0x02
#define AK4648BIT_PMPLL         0x01

#define AK4648BIT_SPPSN         0x80
#define AK4648BIT_MINS          0x40
#define AK4648BIT_DACS          0x20
#define AK4648BIT_DACL          0x10
#define AK4648BIT_PMMP          0x04
#define AK4648BIT_MGAIN0        0x01

#define AK4648BIT_LOVL          0x80
#define AK4648BIT_LOPS          0x40
#define AK4648BIT_MGAIN1        0x20
#define AK4648BIT_SPKG1         0x10
#define AK4648BIT_SPKG0         0x08
#define AK4648BIT_MINL          0x04
#define AK4648BIT_SPKG2         0x02
#define AK4648BIT_SPKG_MASK     0x1a

#define AK4648BIT_PLL3          0x80
#define AK4648BIT_PLL2          0x40
#define AK4648BIT_PLL1          0x20
#define AK4648BIT_PLL0          0x10
#define AK4648BIT_PLL_MASK      0xF0
#define AK4648BIT_BCKO          0x08
#define AK4648BIT_DIF1          0x02
#define AK4648BIT_DIF0          0x01
#define AK4648BIT_DIF_MASK      0x03

#define AK4648BIT_PS1           0x80
#define AK4648BIT_PS0           0x40
#define AK4648BIT_PS_MASK       0xC0
#define AK4648BIT_MSBS          0x10
#define AK4648BIT_BCKP          0x08

#define AK4648BIT_DVTM          0x80
#define AK4648BIT_WTM2          0x40
#define AK4648BIT_ZTM1          0x20
#define AK4648BIT_ZTM0          0x10
#define AK4648BIT_ZTM_MASK      0x30
#define AK4648BIT_WTM1          0x08
#define AK4648BIT_WTM0          0x04
#define AK4648BIT_WTM_MASK      0x4C
#define AK4648BIT_RFST1         0x02
#define AK4648BIT_RFST0         0x01
#define AK4648BIT_RFST_MASK     0x03

#define AK4648BIT_ALC           0x20
#define AK4648BIT_ZELMN         0x10
#define AK4648BIT_LMAT1         0x08
#define AK4648BIT_LMAT0         0x04
#define AK4648BIT_LMAT_MASK     0x0C
#define AK4648BIT_RGAIN0        0x02
#define AK4648BIT_LMTH0         0x01

#define AK4648BIT_RGAIN1        0x80
#define AK4648BIT_LMTH1         0x40
#define AK4648BIT_VBAT          0x02

#define AK4648BIT_LOOP          0x40
#define AK4648BIT_SMUTE         0x20
#define AK4648BIT_DVOLC         0x10
#define AK4648BIT_FBEQ          0x04
#define AK4648BIT_DEM1          0x02
#define AK4648BIT_DEM0          0x01
#define AK4648BIT_DEM_MASK      0x03

#define AK4648BIT_IVOLC         0x08
#define AK4648BIT_HPM           0x04
#define AK4648BIT_MINH          0x02
#define AK4648BIT_DACH          0x01

#define AK4648BIT_INR1          0x80
#define AK4648BIT_INL1          0x40
#define AK4648BIT_MDIF2         0x10
#define AK4648BIT_MDIF1         0x08
#define AK4648BIT_MDIF_MASK     0x18
#define AK4648BIT_INR0          0x04
#define AK4648BIT_INL0          0x02
#define AK4648BIT_IN_MASK       0xc6
#define AK4648BIT_PMADR         0x01

#define AK4648BIT_GN1           0x80
#define AK4648BIT_GN0           0x40
#define AK4648BIT_GN_MASK       0xC0
#define AK4648BIT_FIL1          0x10
#define AK4648BIT_EQ            0x08
#define AK4648BIT_FIL3          0x04

#define AK4648BIT_SPKMN         0x40
#define AK4648BIT_MICR3         0x20
#define AK4648BIT_MICL3         0x10
#define AK4648BIT_L4DIF         0x08
#define AK4648BIT_MIX           0x04
#define AK4648BIT_AIN3          0x02
#define AK4648BIT_LODIF         0x01

/* PLL2 bit = "1" : MCKI input */
#define AK4648BIT_FS_8KHZ       0x00
#define AK4648BIT_FS_12KHZ      0x01
#define AK4648BIT_FS_16KHZ      0x02
#define AK4648BIT_FS_24KHZ      0x03
#define AK4648BIT_FS_7_35KHZ    0x04
#define AK4648BIT_FS_11_025KHZ  0x05
#define AK4648BIT_FS_14_7KHZ    0x06
#define AK4648BIT_FS_22_05KHZ   0x07
#define AK4648BIT_FS_32KHZ      0x22
#define AK4648BIT_FS_48KHZ      0x23
#define AK4648BIT_FS_29_4KHZ    0x26
#define AK4648BIT_FS_44_1KHZ    0x27
#define AK4648BIT_FS_MASK       0x27

#define AK4648_INPUT_VOL_MAX    241
#define AK4648_OUTPUT_VOL_MAX   255

#define AK4648_WAIT_PLL_LOCK            (4 * HZ / 100)  /* 40ms */
#define AK4648_WAIT_LO_LOCK             (30 * HZ / 100) /* 300ms */
#define AK4648_WAIT_ALC_LOCK            (7 * HZ / 100)  /* 70ms */
#define AK4648_WAIT_HPMTN_EN_LOCK       (21 * HZ / 100) /* 210ms */
#define AK4648_WAIT_HPMTN_DIS_LOCK      (26 * HZ / 100) /* 260ms */

#endif	/* __EMXX_MIXER_H */
