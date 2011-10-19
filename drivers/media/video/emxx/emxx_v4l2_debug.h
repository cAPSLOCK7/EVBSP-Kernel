/*
 * File Name       : drivers/media/video/emxx/emxx_v4l2_debug.h
 * Function        : V4L2 driver for EM/EV
 * Release Version : Ver 1.00
 * Release Date    : 2010.03.05
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


#ifndef EMXX_V4L2_DEBUG__H
#define EMXX_V4L2_DEBUG__H


#if _TRACE_DBG
/*===============================================================*/
/* trace v4l2 proccess                                           */
/*===============================================================*/
/* --- backtrace log buffer ------------------------------------ */
struct CallFunc{
	struct timeval time;
	unsigned long call_seq;
	unsigned long call_stat;
	unsigned long call_qued;
	unsigned long call_prcs;
		 long call_frm;
	unsigned long call_mixi;
	char call_func[80];
};
#define CALLFUNC_INIT	NULL
struct CallFunc *callfunc = CALLFUNC_INIT;

#define TRACE_LOG_MAX	500 /* max CallFunc */
#define CNT_INIT	0

/* --- Flags --------------------------------------------------- */
static int		call_func_max = TRACE_LOG_MAX;
static int		call_func_cnt = CNT_INIT;

static char		cp_name[100];
static int		func_cnt = CNT_INIT, cnt = CNT_INIT;
static unsigned long	flags;


/* --- Macros -------------------------------------------------- */
/* --- init backtrace log buffer ------------------------------- */
#define trace_clr() \
	do { \
		if (callfunc == NULL) { \
			callfunc = kmalloc(sizeof(struct CallFunc) * \
				   call_func_max, GFP_KERNEL); \
		} \
		memset(callfunc, 0, sizeof(struct CallFunc) * call_func_max); \
		call_func_cnt = 0; \
	} while (0)

/* --- add backtrace log --------------------------------------- */
#define trace_add(vb, name) \
	do { \
		if (dev->streaming) { \
			do_gettimeofday(&callfunc[call_func_cnt].time); \
			if ((int)vb == 0) { \
				callfunc[call_func_cnt].call_seq  = 0; \
				callfunc[call_func_cnt].call_stat = 0; \
				callfunc[call_func_cnt].call_qued = 0; \
				callfunc[call_func_cnt].call_prcs = 0; \
				callfunc[call_func_cnt].call_frm  = 0; \
			} else { \
				callfunc[call_func_cnt].call_seq  = \
				((struct videobuf_buffer *)vb)->sequence; \
				callfunc[call_func_cnt].call_stat = \
				((struct videobuf_buffer *)vb)->state; \
				callfunc[call_func_cnt].call_qued = \
				((struct videobuf_buffer *)vb)->state_queued; \
				callfunc[call_func_cnt].call_prcs = \
				((struct videobuf_buffer *)vb)->state_proccess;\
				callfunc[call_func_cnt].call_frm  = \
				((struct videobuf_buffer *)vb)->state_frame; \
			} \
			callfunc[call_func_cnt].call_mixi = dev->mixing; \
			strcpy(callfunc[call_func_cnt].call_func, name); \
			call_func_cnt++; \
			if (call_func_cnt == call_func_max) \
				call_func_cnt = 0; \
		} \
	} while (0)

#define trace_add_th(vb, name1, name2) \
	do { \
		memset(cp_name, 0, sizeof(cp_name)); \
		if (th_obj->th_idle_fixed == STATE_QUEUED_ROT_PREPARED) \
			sprintf(cp_name, "%sROT: %s", name1, name2); \
		else if (th_obj->th_idle_fixed == STATE_QUEUED_TMR_PREPARED) \
			sprintf(cp_name, "%sTMR: %s", name1, name2); \
		else if (th_obj->th_idle_fixed == STATE_QUEUED_LCD_PREPARED) \
			sprintf(cp_name, "%sLCD: %s", name1, name2); \
		trace_add(vb, cp_name); \
	} while (0)

#define trace_add_func(vb, name, func) \
	do { \
		memset(cp_name, 0, sizeof(cp_name)); \
		switch (func) { \
		case VBQ_QUEUE: \
			sprintf(cp_name, "%s: <<< VBQ_QUEUE", name); \
			break; \
		case RESCHEDULE: \
			sprintf(cp_name, "%s: <<< RESCHEDULE", name); \
			break; \
		case ROT_REQUEST: \
			sprintf(cp_name, "%s: <<< ROT_REQUEST", name); \
			break; \
		case ROT_CALLBACK: \
			sprintf(cp_name, "%s: <<< ROT_CALLBACK", name); \
			break; \
		case TMR_REQUEST: \
			sprintf(cp_name, "%s: <<< TMR_REQUEST", name); \
			break; \
		case TMR_CALLBACK: \
			sprintf(cp_name, "%s: <<< TMR_CALLBACK", name); \
			break; \
		case LCD_REQUEST: \
			sprintf(cp_name, "%s: <<< LCD_REQUEST", name); \
			break; \
		case LCD_CALLBACK: \
			sprintf(cp_name, "%s: <<< LCD_CALLBACK", name); \
			break; \
		case VBQ_COMPLETE: \
			sprintf(cp_name, "%s: <<< VBQ_COMPLETE", name); \
			break; \
		case VBQ_DQUEUE: \
			sprintf(cp_name, "%s: <<< VBQ_DQUEUE", name); \
			break; \
		} \
		trace_add(vb, cp_name); \
	} while (0)

/* --- output backtrace log ------------------------------------ */
#define trace_out() \
	do { \
		long usec = 0; \
		for (cnt = 1; cnt < call_func_max; cnt++) { \
			if (call_func_cnt - cnt >= 0) \
				func_cnt = call_func_cnt - cnt; \
			else \
				func_cnt = call_func_max + \
					   (call_func_cnt - cnt); \
			usec = callfunc[func_cnt+1].time.tv_usec - \
			       callfunc[func_cnt].time.tv_usec; \
			if (usec < 0) \
				usec += 1000000; \
			printk(KERN_INFO "v4l2: backtrace, msec(%3ld.%03ld), "\
			 "seq(%ld), st(%ld), st_qued(%ld), st_prcs(%02lx), "\
			 "st_frm(%2ld), mix(%04lx), %s\n", \
			 usec/1000, usec%1000, callfunc[func_cnt].call_seq, \
			 callfunc[func_cnt].call_stat, \
			 callfunc[func_cnt].call_qued, \
			 callfunc[func_cnt].call_prcs, \
			 callfunc[func_cnt].call_frm, \
			 callfunc[func_cnt].call_mixi, \
			 callfunc[func_cnt].call_func); \
		} \
		printk(KERN_INFO "\n"); \
	} while (0)


#define dbg_backtrace(dev) \
	do { \
		spin_lock_irqsave( \
		 &((struct emxx_v4l2_device *)dev)->vbq_lock, flags); \
		printk(KERN_INFO "\n"); \
		trace_out(); \
		trace_clr(); \
		printk(KERN_INFO "\n"); \
		spin_unlock_irqrestore( \
		 &((struct emxx_v4l2_device *)dev)->vbq_lock, flags); \
	} while (0)


/*===============================================================*/
/* trace video_buffer                                            */
/*===============================================================*/
#define dbg_vbqtrace(dev, q) \
	do { \
		spin_lock_irqsave( \
		 &((struct emxx_v4l2_device *)dev)->vbq_lock, flags); \
		printk(KERN_INFO "\n*** video_buffer *************************"\
		 "*****************\n"); \
		for (cnt = 0; cnt < ((struct videobuf_queue *)q)->index_max; \
		     cnt++) { \
			printk(KERN_INFO "v4l2: video_buffer[%2d](%p), "\
			 "prv(%p), nxt(%p), st(%d), st_qued(%d), "\
			 "st_prcs(%02lx), st_frm(%2ld) seq(%d)\n", cnt, \
			 &((struct videobuf_queue *)q)->bufs[cnt]->stream, \
			 ((struct videobuf_queue *)q)->bufs[cnt]->stream.prev, \
			 ((struct videobuf_queue *)q)->bufs[cnt]->stream.next, \
			 ((struct videobuf_queue *)q)->bufs[cnt]->state, \
			 ((struct videobuf_queue *)q)->bufs[cnt]->state_queued,\
			 ((struct videobuf_queue *)q)->bufs[cnt]->\
			 state_proccess, \
			 ((struct videobuf_queue *)q)->bufs[cnt]->state_frame, \
			 ((struct videobuf_queue *)q)->bufs[cnt]->sequence); \
		} \
		printk(KERN_INFO "********************************************"\
		 "***************\n"); \
		spin_unlock_irqrestore( \
		 &((struct emxx_v4l2_device *)dev)->vbq_lock, flags); \
	} while (0)


#else /* _TRACE_DBG */
#define trace_clr()
#define trace_add(vb, name)
#define trace_add_th(vb, name1, name2)
#define trace_add_func(vb, name, func)
#define trace_out()
#define dbg_backtrace(dev)
#define dbg_vbqtrace(dev, q)
#endif /* _TRACE_DBG */



#endif /* EMXX_V4L2_DEBUG__H */
