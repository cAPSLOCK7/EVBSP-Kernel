/*
 * InterDSP Driver
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

#ifndef INTER_DSP_H
#define INTER_DSP_H

/* communication channel num */
#define INTERDSP_CH_NUM		3	/* Channel num */

/* Channel invalid mark.
   more than INTERDSP_CH_NUM, and not minus. */
#define INTERDSP_CH_INVALID	(INTERDSP_CH_NUM+1)

/* SPA/SPX command format */
union interdsp_spa_header {
	uint32_t header[2];	/* Accessor */

	/* Control Command Format */
	struct {
		unsigned issue:8;
		unsigned taskid:4;
		unsigned reserve1:2;
		unsigned buffer:2; /* 00 */
		unsigned reserve2:13;
		unsigned buffer_no:3;	/* 000 */
		unsigned length:16;
		unsigned block_num:16;
	} spa_ccf;

	/* Transfer Command Format */
	struct {
		unsigned issue:8;
		unsigned taskid:4;
		unsigned reserve1:2;
		unsigned buffer:2; /* 1x */
		unsigned reserve2:13;
		unsigned buffer_no:3;
		unsigned length:16;
		unsigned block_num:16;
	} spa_tcf;

	/* Response Format */
	struct {
		unsigned issue:8;
		unsigned taskid:4;
		unsigned reserve1:2;
		unsigned eor:1;
		unsigned rrf:1;	/* 0 */
		unsigned code:15;
		unsigned error:1;
		unsigned length:16;
		unsigned block_num:16;
	} spa_res;

	/* Request Format */
	struct {
		unsigned reserve:8;
		unsigned taskid:4;
		unsigned reserve1:3;
		unsigned rrf:1;	/* 1 */
		unsigned code:16;
		unsigned length:16;
		unsigned block_num:16;
	} spa_req;

	/* SPA Command Format */
	struct {
		unsigned issue:8;
		unsigned taskid:4; /* 0 */
		unsigned reserve1:4;
		unsigned code:8;
		unsigned reserve2:8;
		unsigned length:16;
		unsigned block_num:16;
	} spa_cmd;
};

#endif /* INTER_DSP_H */
