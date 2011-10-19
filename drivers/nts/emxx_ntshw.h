/*
 * File Name       : drivers/nts/emxx_ntshw.h
 * Function        : NTSC Driver (H/W Control)
 * Release Version : Ver 1.00
 * Release Date    : 2010.09.24
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


#ifndef _EMXX_NTSHW_H_
#define _EMXX_NTSHW_H_



/********************************************************
 *  Variables                                           *
 *******************************************************/


/********************************************************
 *  Prototype declarations of function                  *
 *******************************************************/
/* ------------------ NTSC H/W control function ---------------------------- */
extern void          ntshw_start(int iStartFlg);
#define NTSHW_START 0
#define NTSHW_STOP  1

extern void          ntshw_reset(int iResetFlg);
#define NTSHW_RESET   0
#define NTSHW_UNRESET 1

extern void          ntshw_module_reset(int iResetFlg);


/* ------------------ NTSC rgister set function ---------------------------- */
extern void          ntshw_set_interruput(int iIntFlg);
#define NTSHW_INTENSET 0
#define NTSHW_INTENCLR 1
#define NTSHW_INTFFCLR 2
extern int           ntshw_set_framesel(int iSetFrameNo);
extern int           ntshw_set_ntsout(int iSetNtsoutFlg);
/* ------------------ NTSC rgister check function -------------------------- */
extern unsigned long ntshw_chk_control(void);
extern unsigned long ntshw_chk_status(void);
extern unsigned long ntshw_chk_framesel(void);
extern unsigned long ntshw_chk_intstatus(void);
/* ------------------ NTSC rgister backup function ------------------------- */
extern void          ntshw_save_reg(void);
extern void          ntshw_restore_reg(void);
/* ------------------ NTSC initialize function ----------------------------- */
extern int           ntshw_reserve(int iSetOutmode);
extern void          ntshw_release(void);
extern void          ntshw_clr_framebuf(int iClrFrameNo);
extern int           ntshw_init(struct emxx_nts_dev *ntsc);
extern void          ntshw_exit(void);



#endif /* _EMXX_NTSHW_H_ */

