/*
 *  File Name		: drivers/i2c/busses/i2c-emxx.h
 *  Function		: i2c
 *  Release Version	: Ver 1.02
 *  Release Date	: 2010/05/24
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

#ifndef __DRIVERS_I2C_BUSSES_I2C_EMXX_H
#define __DRIVERS_I2C_BUSSES_I2C_EMXX_H

/* INT Reg */
#define EMXX_I2C_INT_RAW	(IO_ADDRESS(EMXX_INTA_D_BASE) + 0x014)
#define EMXX_I2C_INT_IIR	(IO_ADDRESS(EMXX_INTA_D_BASE) + 0x024)

/* INT Reg bit */
#define EMXX_I2C_INTC_MST	(1 << (32 - 32))
#define EMXX_I2C2_INTC_MST	(1 << (33 - 32))

#define EMXX_I2C_INTC_IIR	(1 << 16)
#define EMXX_I2C2_INTC_IIR	(1 << 17)

/* I2C Registers */
#define I2C_ADDR		IO_ADDRESS(EMXX_IIC0_BASE)
#define I2C2_ADDR		IO_ADDRESS(EMXX_IIC11_BASE)

#define I2C_OFS_IICACT0		0x00	/* start */
#define I2C_OFS_IIC0		0x04	/* shift */
#define I2C_OFS_IICC0		0x08	/* control */
#define I2C_OFS_SVA0		0x0c	/* slave address */
#define I2C_OFS_IICCL0		0x10	/* clock select */
#define I2C_OFS_IICX0		0x14	/* extention */
#define I2C_OFS_IICS0		0x18	/* status */
#define I2C_OFS_IICSE0		0x1c	/* status For emulation */
#define I2C_OFS_IICF0		0x20	/* IIC flag */


/* I2C IICACT0 Masks */
#define I2C_BIT_IICE0		0x0001

/* I2C IICC0 Masks */
#define I2C_BIT_LREL0		0x0040
#define I2C_BIT_WREL0		0x0020
#define I2C_BIT_SPIE0		0x0010
#define I2C_BIT_WTIM0		0x0008
#define I2C_BIT_ACKE0		0x0004
#define I2C_BIT_STT0		0x0002
#define I2C_BIT_SPT0		0x0001

/* I2C IICCL0 Masks */
#define I2C_BIT_CLD0		0x0020
#define I2C_BIT_DAD0		0x0010
#define I2C_BIT_SMC0		0x0008
#define I2C_BIT_DFC0		0x0004
#define I2C_BIT_CLO1		0x0002
#define I2C_BIT_CLO0		0x0001

/* I2C IICSE0 Masks */
#define I2C_BIT_MSTS0		0x0080
#define I2C_BIT_ALD0		0x0040
#define I2C_BIT_EXC0		0x0020
#define I2C_BIT_COI0		0x0010
#define I2C_BIT_TRC0		0x0008
#define I2C_BIT_ACKD0		0x0004
#define I2C_BIT_STD0		0x0002
#define I2C_BIT_SPD0		0x0001

/* I2C IICF0 Masks */
#define I2C_BIT_STCF		0x0080
#define I2C_BIT_IICBSY		0x0040
#define I2C_BIT_STCEN		0x0002
#define I2C_BIT_IICRSV		0x0001

/* For setting of sending and receiving */
#define I2C_DIR_SHIFT		1
#define I2C_DIR_R		0x01
#define I2C_DIR_W		0x00

struct i2c_ctrl {
	unsigned char smc;	/* mode */
	unsigned char dfc;	/* digital filter */
};
#define i2c_ctrl_t	struct i2c_ctrl

/* mode */
#define I2C_SMC_NORMAL_SPEED	0	/* nomal mode(maximum transfer rate
					   70kbit/s)(default) */
#define I2C_SMC_HIGH_SPEED	1	/* high-speed mode(maximum transfer
					   rate 250kbit/s) */

/* digital filter */
#define I2C_DFC_OFF		0	/* digital filter OFF (default) */
#define I2C_DFC_ON		1	/* digital filter ON */

/* emxx wait */
#define EMXX_I2C_WAIT		1	/* wait 1us after STT0 */

/* IIC_SCLK ( HIGHSPEED : 4.19MHz-8.38MHz */
#define I2C_DIVIIC_VAL1 (SMU_DIV(32))
	/* (229.376MHz or 237.568MHz)/32/24 */
#define I2C_DIVIIC_VAL2 (SMU_DIV(32) << 16)
	/* (229.376MHz or 237.568MHz)/32/24 */


#define EMXX_I2C_CH1	0
#define EMXX_I2C_CH2	1


#ifdef CONFIG_I2C_EMXX_ENABLE_CH2
#define I2C_NR	2
#else
#define I2C_NR	1
#endif


#endif	/* __DRIVERS_I2C_BUSSES_I2C_EMXX_H */
