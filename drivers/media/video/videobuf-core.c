/*
 * drivers/media/video/videobuf-core.c
 *  generic helper functions for handling video4linux capture buffers
 *
 * Copyright (C) 2010 Renesas Electronics Corporation
 *
 * (c) 2007 Mauro Carvalho Chehab, <mchehab@infradead.org>
 *
 * Highly based on video-buf written originally by:
 * (c) 2001,02 Gerd Knorr <kraxel@bytesex.org>
 * (c) 2006 Mauro Carvalho Chehab, <mchehab@infradead.org>
 * (c) 2006 Ted Walther and John Sokol
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include <media/videobuf-core.h>

#define MAGIC_BUFFER 0x20070728
#define MAGIC_CHECK(is, should) do { \
	if (unlikely((is) != (should))) { \
	printk(KERN_ERR "magic mismatch: %x (expected %x)\n", is, should); \
	BUG(); } } while (0)

static int debug;
module_param(debug, int, 0644);

MODULE_DESCRIPTION("helper module to manage video4linux buffers");
MODULE_AUTHOR("Mauro Carvalho Chehab <mchehab@infradead.org>");
MODULE_LICENSE("GPL");

#define dprintk(level, fmt, arg...) do { \
	if (debug >= level) \
	printk(KERN_DEBUG "vbuf: " fmt , ## arg); } while (0)

#ifdef CONFIG_VIDEO_EMXX
#define _POLL_DBG 0
#define _QBUF_DBG 0

#define dbg_printk(level, fmt, arg...) \
	do { \
		if (level > 0) \
			printk(KERN_DEBUG " @vbq: " fmt, ## arg); \
	} while (0)
#endif

/* --------------------------------------------------------------------- */

#define CALL(q, f, arg...)						\
	((q->int_ops->f) ? q->int_ops->f(arg) : 0)

void *videobuf_alloc(struct videobuf_queue *q)
{
	struct videobuf_buffer *vb;

	BUG_ON(q->msize < sizeof(*vb));
#ifndef CONFIG_VIDEO_EMXX
	if (!q->int_ops || !q->int_ops->alloc) {
		printk(KERN_ERR "No specific ops defined!\n");
		BUG();
	}

	vb = q->int_ops->alloc(q->msize);
#else
	vb = kzalloc(q->msize, GFP_KERNEL);
#endif

	if (NULL != vb) {
#ifdef CONFIG_VIDEO_EMXX
		init_waitqueue_head(&vb->clear_done);
#endif /* CONFIG_VIDEO_EMXX */
		init_waitqueue_head(&vb->done);
		vb->magic     = MAGIC_BUFFER;
	}

	return vb;
}
EXPORT_SYMBOL_GPL(videobuf_alloc);

#ifdef CONFIG_VIDEO_EMXX
#define WAITON_CONDITION ((vb->state != VIDEOBUF_ACTIVE &&          \
			   vb->state != VIDEOBUF_QUEUED) &&         \
			  (vb->state_queued == STATE_QUEUED_IDLE || \
			   vb->state_queued == STATE_QUEUED_DONE))

#define __emxx_wait_event_timeout(wq, condition, ret)			\
do {									\
	DEFINE_WAIT(__wait);						\
									\
	for (;;) {							\
		prepare_to_wait(&wq, &__wait, TASK_UNINTERRUPTIBLE);	\
		if (condition)						\
			break;						\
		schedule_timeout(ret);					\
	}								\
	finish_wait(&wq, &__wait);					\
} while (0)

#define emxx_wait_event_timeout(wq, condition, timeout)		\
({									\
	long __ret = timeout;						\
	if (!(condition))						\
		__emxx_wait_event_timeout(wq, condition, __ret);	\
	__ret;								\
})
#else
#define WAITON_CONDITION (vb->state != VIDEOBUF_ACTIVE &&\
				vb->state != VIDEOBUF_QUEUED)
#endif

int videobuf_waiton(struct videobuf_buffer *vb, int non_blocking, int intr)
{
	MAGIC_CHECK(vb->magic, MAGIC_BUFFER);

	if (non_blocking) {
		if (WAITON_CONDITION)
			return 0;
		else
			return -EAGAIN;
	}

	if (intr)
#ifdef CONFIG_VIDEO_EMXX
		return wait_event_interruptible(vb->clear_done,
						WAITON_CONDITION);
#else
		return wait_event_interruptible(vb->done, WAITON_CONDITION);
#endif
	else
#ifdef CONFIG_VIDEO_EMXX
		emxx_wait_event_timeout(vb->clear_done, WAITON_CONDITION,
					 msecs_to_jiffies(10));
#else
		wait_event(vb->done, WAITON_CONDITION);
#endif

	return 0;
}
EXPORT_SYMBOL_GPL(videobuf_waiton);


#ifdef CONFIG_VIDEO_EMXX
int videobuf_waiton_dqbuf(struct videobuf_buffer *vb, int non_blocking,
 int intr)
{
	int tout_ms = 10;
	int tout_cnt = 0;
	int retval = 0;
	DECLARE_WAITQUEUE(wait, current);

	MAGIC_CHECK(vb->magic, MAGIC_BUFFER);
	add_wait_queue(&vb->done, &wait);
	while (vb->state != VIDEOBUF_DQBUF_PERMIT) {
		if (non_blocking || tout_cnt == 5) {
			retval = -EAGAIN;
			break;
		}
		set_current_state(intr  ? TASK_INTERRUPTIBLE
					: TASK_UNINTERRUPTIBLE);
		if (vb->state != VIDEOBUF_DQBUF_PERMIT)
			schedule_timeout(msecs_to_jiffies(tout_ms));
		set_current_state(TASK_RUNNING);
		if (intr && signal_pending(current)) {
			dprintk(1, "buffer waiton: -EINTR\n");
			retval = -EINTR;
			break;
		}
		tout_cnt++;
	}
	remove_wait_queue(&vb->done, &wait);
	return retval;
}
#endif

int videobuf_iolock(struct videobuf_queue *q, struct videobuf_buffer *vb,
		    struct v4l2_framebuffer *fbuf)
{
	MAGIC_CHECK(vb->magic, MAGIC_BUFFER);
	MAGIC_CHECK(q->int_ops->magic, MAGIC_QTYPE_OPS);

	return CALL(q, iolock, q, vb, fbuf);
}
EXPORT_SYMBOL_GPL(videobuf_iolock);

void *videobuf_queue_to_vmalloc(struct videobuf_queue *q,
			   struct videobuf_buffer *buf)
{
	if (q->int_ops->vmalloc)
		return q->int_ops->vmalloc(buf);
	else
		return NULL;
}
EXPORT_SYMBOL_GPL(videobuf_queue_to_vmalloc);

/* --------------------------------------------------------------------- */


void videobuf_queue_core_init(struct videobuf_queue *q,
			 struct videobuf_queue_ops *ops,
			 struct device *dev,
			 spinlock_t *irqlock,
			 enum v4l2_buf_type type,
			 enum v4l2_field field,
			 unsigned int msize,
			 void *priv,
			 struct videobuf_qtype_ops *int_ops)
{
	memset(q, 0, sizeof(*q));
	q->irqlock   = irqlock;
	q->dev       = dev;
	q->type      = type;
	q->field     = field;
	q->msize     = msize;
	q->ops       = ops;
	q->priv_data = priv;
	q->int_ops   = int_ops;

	/* All buffer operations are mandatory */
	BUG_ON(!q->ops->buf_setup);
	BUG_ON(!q->ops->buf_prepare);
	BUG_ON(!q->ops->buf_queue);
	BUG_ON(!q->ops->buf_release);

	/* Lock is mandatory for queue_cancel to work */
	BUG_ON(!irqlock);

	/* Having implementations for abstract methods are mandatory */
	BUG_ON(!q->int_ops);

	mutex_init(&q->vb_lock);
	init_waitqueue_head(&q->wait);
	INIT_LIST_HEAD(&q->stream);
}
EXPORT_SYMBOL_GPL(videobuf_queue_core_init);

/* Locking: Only usage in bttv unsafe find way to remove */
int videobuf_queue_is_busy(struct videobuf_queue *q)
{
	int i;

	MAGIC_CHECK(q->int_ops->magic, MAGIC_QTYPE_OPS);

	if (q->streaming) {
		dprintk(1, "busy: streaming active\n");
		return 1;
	}
	if (q->reading) {
		dprintk(1, "busy: pending read #1\n");
		return 1;
	}
	if (q->read_buf) {
		dprintk(1, "busy: pending read #2\n");
		return 1;
	}
	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		if (NULL == q->bufs[i])
			continue;
		if (q->bufs[i]->map) {
			dprintk(1, "busy: buffer #%d mapped\n", i);
			return 1;
		}
		if (q->bufs[i]->state == VIDEOBUF_QUEUED) {
			dprintk(1, "busy: buffer #%d queued\n", i);
			return 1;
		}
		if (q->bufs[i]->state == VIDEOBUF_ACTIVE) {
			dprintk(1, "busy: buffer #%d avtive\n", i);
			return 1;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(videobuf_queue_is_busy);

/* Locking: Caller holds q->vb_lock */
void videobuf_queue_cancel(struct videobuf_queue *q)
{
	unsigned long flags = 0;
	int i;

	q->streaming = 0;
	q->reading  = 0;
	wake_up_interruptible_sync(&q->wait);

	/* remove queued buffers from list */
	spin_lock_irqsave(q->irqlock, flags);
	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		if (NULL == q->bufs[i])
			continue;
#ifdef CONFIG_VIDEO_EMXX
		if (q->bufs[i]->state == VIDEOBUF_PREPARED ||
		    q->bufs[i]->state == VIDEOBUF_QUEUED) {
			if (q->ops->buf_cancel(q, q->bufs[i]))
				q->bufs[i]->state = VIDEOBUF_CANCELED;
			else
				q->bufs[i]->state = VIDEOBUF_DQBUF_PERMIT;
		}
#else	/* CONFIG_VIDEO_EMXX */
		if (q->bufs[i]->state == VIDEOBUF_QUEUED) {
			list_del(&q->bufs[i]->queue);
			q->bufs[i]->state = VIDEOBUF_ERROR;
			wake_up_all(&q->bufs[i]->done);
		}
#endif	/* CONFIG_VIDEO_EMXX */
	}
	spin_unlock_irqrestore(q->irqlock, flags);

	/* free all buffers + clear queue */
	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		if (NULL == q->bufs[i])
			continue;
		q->ops->buf_release(q, q->bufs[i]);
	}
	INIT_LIST_HEAD(&q->stream);
}
EXPORT_SYMBOL_GPL(videobuf_queue_cancel);

#ifdef CONFIG_VIDEO_EMXX
void
videobuf_queue_clear(struct videobuf_queue *q)
{
	unsigned long flags;
	struct list_head *list;
	struct videobuf_buffer *vb;

	/* remove queued buffers from list */
	spin_lock_irqsave(q->irqlock, flags);

	list_for_each(list, &q->stream) {
		vb = list_entry(list, struct videobuf_buffer, stream);
		if (vb->ts.tv_sec != -1 && vb->ts.tv_usec != -1) {
			if (vb->state == VIDEOBUF_PREPARED ||
			    vb->state == VIDEOBUF_QUEUED) {
				if (q->ops->buf_cancel(q, vb))
					vb->state = VIDEOBUF_CANCELED;
				else
					vb->state = VIDEOBUF_DQBUF_PERMIT;
			}
		}
		dbg_printk(_QBUF_DBG,
				"clear:  (%p) ->prev(%p) ->next(%p) "
				"->state(%d) ->sequence(%d)\n",
				&vb->stream, vb->stream.prev, vb->stream.next,
				vb->state, vb->sequence);
	}

	spin_unlock_irqrestore(q->irqlock, flags);
}
#endif

/* --------------------------------------------------------------------- */

/* Locking: Caller holds q->vb_lock */
enum v4l2_field videobuf_next_field(struct videobuf_queue *q)
{
	enum v4l2_field field = q->field;

	BUG_ON(V4L2_FIELD_ANY == field);

	if (V4L2_FIELD_ALTERNATE == field) {
		if (V4L2_FIELD_TOP == q->last) {
			field   = V4L2_FIELD_BOTTOM;
			q->last = V4L2_FIELD_BOTTOM;
		} else {
			field   = V4L2_FIELD_TOP;
			q->last = V4L2_FIELD_TOP;
		}
	}
	return field;
}
EXPORT_SYMBOL_GPL(videobuf_next_field);

/* Locking: Caller holds q->vb_lock */
static void videobuf_status(struct videobuf_queue *q, struct v4l2_buffer *b,
			    struct videobuf_buffer *vb, enum v4l2_buf_type type)
{
	MAGIC_CHECK(vb->magic, MAGIC_BUFFER);
	MAGIC_CHECK(q->int_ops->magic, MAGIC_QTYPE_OPS);

#ifndef CONFIG_VIDEO_EMXX
	b->index    = vb->i;
#endif
	b->type     = type;

	b->memory   = vb->memory;
	switch (b->memory) {
	case V4L2_MEMORY_MMAP:
		b->m.offset  = vb->boff;
		b->length    = vb->bsize;
		break;
	case V4L2_MEMORY_USERPTR:
#ifndef CONFIG_VIDEO_EMXX
		b->m.userptr = vb->baddr;
		b->length    = vb->bsize;
#endif
		break;
	case V4L2_MEMORY_OVERLAY:
		b->m.offset  = vb->boff;
		break;
#ifdef CONFIG_VIDEO_EMXX
	case V4L2_MEMORY_PHYSADDR:
		break;
#endif
	}

	b->flags    = 0;
#ifdef CONFIG_VIDEO_EMXX
	switch (vb->state_frame) {
	case STATE_FRAME_DONE:
	case STATE_FRAME_IMMEDIATE:
		b->flags |= V4L2_BUF_FLAG_DONE;
		break;
	case STATE_FRAME_CANCELED:
		b->flags |= V4L2_BUF_FLAG_CANCELED;
		break;
	case STATE_FRAME_SKIPPED:
		b->flags |= V4L2_BUF_FLAG_SKIPPED;
		break;
	default:
		break;
	}
#else
	if (vb->map)
		b->flags |= V4L2_BUF_FLAG_MAPPED;

	switch (vb->state) {
	case VIDEOBUF_PREPARED:
	case VIDEOBUF_QUEUED:
	case VIDEOBUF_ACTIVE:
		b->flags |= V4L2_BUF_FLAG_QUEUED;
		break;
	case VIDEOBUF_DONE:
	case VIDEOBUF_ERROR:
		b->flags |= V4L2_BUF_FLAG_DONE;
		break;
	case VIDEOBUF_NEEDS_INIT:
	case VIDEOBUF_IDLE:
		/* nothing */
		break;
	}
#endif /* CONFIG_VIDEO_EMXX */

	if (vb->input != UNSET) {
		b->flags |= V4L2_BUF_FLAG_INPUT;
		b->input  = vb->input;
	}

	b->field     = vb->field;
	b->timestamp = vb->ts;
	b->bytesused = vb->size;
#ifdef CONFIG_VIDEO_EMXX
	b->sequence  = vb->sequence;
#else
	b->sequence  = vb->field_count >> 1;
#endif
}

/* Locking: Caller holds q->vb_lock */
static int __videobuf_mmap_free(struct videobuf_queue *q)
{
	int i;
#ifndef CONFIG_VIDEO_EMXX
	int rc;
#else
	int rc = 0;
#endif
	if (!q)
		return 0;

	MAGIC_CHECK(q->int_ops->magic, MAGIC_QTYPE_OPS);
#ifndef CONFIG_VIDEO_EMXX
	rc  = CALL(q, mmap_free, q);
	q->is_mmapped = 0;

	if (rc < 0)
		return rc;
#else
	for (i = 0; i < VIDEO_MAX_FRAME; i++)
		if (q->bufs[i] && q->bufs[i]->map)
			return -EBUSY;
#endif
	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		if (NULL == q->bufs[i])
			continue;
		q->ops->buf_release(q, q->bufs[i]);
		kfree(q->bufs[i]);
		q->bufs[i] = NULL;
	}

	return rc;
}

int videobuf_mmap_free(struct videobuf_queue *q)
{
	int ret;
	mutex_lock(&q->vb_lock);
	ret = __videobuf_mmap_free(q);
	mutex_unlock(&q->vb_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(videobuf_mmap_free);

/* Locking: Caller holds q->vb_lock */
int __videobuf_mmap_setup(struct videobuf_queue *q,
			unsigned int bcount, unsigned int bsize,
			enum v4l2_memory memory)
{
	unsigned int i;
	int err;

	MAGIC_CHECK(q->int_ops->magic, MAGIC_QTYPE_OPS);

	err = __videobuf_mmap_free(q);
	if (0 != err)
		return err;

	/* Allocate and initialize buffers */
	for (i = 0; i < bcount; i++) {
		q->bufs[i] = videobuf_alloc(q);

		if (q->bufs[i] == NULL)
			break;

		q->bufs[i]->i      = i;
		q->bufs[i]->input  = UNSET;
		q->bufs[i]->memory = memory;
		q->bufs[i]->bsize  = bsize;
		switch (memory) {
		case V4L2_MEMORY_MMAP:
			q->bufs[i]->boff  = bsize * i;
			break;
#ifndef CONFIG_VIDEO_EMXX
		case V4L2_MEMORY_USERPTR:
#endif
		case V4L2_MEMORY_OVERLAY:
			/* nothing */
			break;
#ifdef CONFIG_VIDEO_EMXX
		case V4L2_MEMORY_USERPTR:
		case V4L2_MEMORY_PHYSADDR:
			if (q->bufs[i] == NULL)
				return -EINVAL;
			break;
#endif
		}
	}

	if (!i)
		return -ENOMEM;

	dprintk(1, "mmap setup: %d buffers, %d bytes each\n",
		i, bsize);
#ifndef CONFIG_VIDEO_EMXX
	return i;
#else	/* CONFIG_VIDEO_EMXX */
	return 0;
#endif	/* CONFIG_VIDEO_EMXX */
}
EXPORT_SYMBOL_GPL(__videobuf_mmap_setup);

int videobuf_mmap_setup(struct videobuf_queue *q,
			unsigned int bcount, unsigned int bsize,
			enum v4l2_memory memory)
{
	int ret;
	mutex_lock(&q->vb_lock);
	ret = __videobuf_mmap_setup(q, bcount, bsize, memory);
	mutex_unlock(&q->vb_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(videobuf_mmap_setup);

int videobuf_reqbufs(struct videobuf_queue *q,
		 struct v4l2_requestbuffers *req)
{
	unsigned int size, count;
	int retval;

	if (req->count < 1) {
		dprintk(1, "reqbufs: count invalid (%d)\n", req->count);
		return -EINVAL;
	}

	if (req->memory != V4L2_MEMORY_MMAP     &&
	    req->memory != V4L2_MEMORY_USERPTR  &&
#ifdef CONFIG_VIDEO_EMXX
	    req->memory != V4L2_MEMORY_PHYSADDR &&
#endif
	    req->memory != V4L2_MEMORY_OVERLAY) {
		dprintk(1, "reqbufs: memory type invalid\n");
		return -EINVAL;
	}

	mutex_lock(&q->vb_lock);
	if (req->type != q->type) {
		dprintk(1, "reqbufs: queue type invalid\n");
		retval = -EINVAL;
		goto done;
	}

	if (q->streaming) {
		dprintk(1, "reqbufs: streaming already exists\n");
		retval = -EBUSY;
		goto done;
	}
	if (!list_empty(&q->stream)) {
		dprintk(1, "reqbufs: stream running\n");
		retval = -EBUSY;
		goto done;
	}

	count = req->count;
	if (count > VIDEO_MAX_FRAME)
		count = VIDEO_MAX_FRAME;
	size = 0;
#ifdef CONFIG_VIDEO_EMXX
	retval = q->ops->buf_setup(q, &count, &size);
	if (retval < 0)
		goto done;
#else
	q->ops->buf_setup(q, &count, &size);
#endif
	size = PAGE_ALIGN(size);
	dprintk(1, "reqbufs: bufs=%d, size=0x%x [%d pages total]\n",
		count, size, (count*size)>>PAGE_SHIFT);

	retval = __videobuf_mmap_setup(q, count, size, req->memory);
	if (retval < 0) {
		dprintk(1, "reqbufs: mmap setup returned %d\n", retval);
		goto done;
	}

	req->count = retval;
#ifdef CONFIG_VIDEO_EMXX
	q->index_max = count;
#endif

 done:
	mutex_unlock(&q->vb_lock);
	return retval;
}
EXPORT_SYMBOL_GPL(videobuf_reqbufs);

int videobuf_querybuf(struct videobuf_queue *q, struct v4l2_buffer *b)
{
	int ret = -EINVAL;

	mutex_lock(&q->vb_lock);
	if (unlikely(b->type != q->type)) {
		dprintk(1, "querybuf: Wrong type.\n");
		goto done;
	}
	if (unlikely(b->index < 0 || b->index >= VIDEO_MAX_FRAME)) {
		dprintk(1, "querybuf: index out of range.\n");
		goto done;
	}
	if (unlikely(NULL == q->bufs[b->index])) {
		dprintk(1, "querybuf: buffer is null.\n");
		goto done;
	}

	videobuf_status(q, b, q->bufs[b->index], q->type);

	ret = 0;
done:
	mutex_unlock(&q->vb_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(videobuf_querybuf);

int videobuf_qbuf(struct videobuf_queue *q,
	      struct v4l2_buffer *b)
{
	struct videobuf_buffer *buf;
	enum v4l2_field field;
	unsigned long flags = 0;
	int retval;
#ifdef CONFIG_VIDEO_EMXX
	int index;
#endif

	MAGIC_CHECK(q->int_ops->magic, MAGIC_QTYPE_OPS);

	if (b->memory == V4L2_MEMORY_MMAP)
		down_read(&current->mm->mmap_sem);

	mutex_lock(&q->vb_lock);
	retval = -EBUSY;

#ifdef CONFIG_VIDEO_EMXX
	for (index = 0; index < q->index_max; index++) {
		buf = q->bufs[index];
		if (NULL == buf)
			continue;

		if (buf->state == VIDEOBUF_NEEDS_INIT ||
		    buf->state == VIDEOBUF_ERROR ||
		    buf->state == VIDEOBUF_IDLE)
			goto index_done;
	}
	goto done;
 index_done:
#endif
	if (q->reading) {
		dprintk(1, "qbuf: Reading running...\n");
		goto done;
	}
	retval = -EINVAL;
	if (b->type != q->type) {
		dprintk(1, "qbuf: Wrong type.\n");
		goto done;
	}

#ifndef CONFIG_VIDEO_EMXX
	if (b->index < 0 || b->index >= VIDEO_MAX_FRAME) {
		dprintk(1, "qbuf: index out of range.\n");
		goto done;
	}
	buf = q->bufs[b->index];
	if (NULL == buf) {
		dprintk(1, "qbuf: buffer is null.\n");
		goto done;
	}
#endif
	MAGIC_CHECK(buf->magic, MAGIC_BUFFER);
	if (buf->memory != b->memory) {
		dprintk(1, "qbuf: memory type is wrong.\n");
		goto done;
	}
#ifndef CONFIG_VIDEO_EMXX
	if (buf->state != VIDEOBUF_NEEDS_INIT && buf->state != VIDEOBUF_IDLE) {
		dprintk(1, "qbuf: buffer is already queued or active.\n");
		goto done;
	}
#endif
	if (b->flags & V4L2_BUF_FLAG_INPUT) {
		if (b->input >= q->inputs) {
			dprintk(1, "qbuf: wrong input.\n");
			goto done;
		}
		buf->input = b->input;
	} else {
		buf->input = UNSET;
	}

	switch (b->memory) {
	case V4L2_MEMORY_MMAP:
		if (0 == buf->baddr) {
			dprintk(1, "qbuf: mmap requested "
				   "but buffer addr is zero!\n");
			goto done;
		}
		break;
	case V4L2_MEMORY_USERPTR:
#ifndef CONFIG_VIDEO_EMXX
		if (b->length < buf->bsize) {
			dprintk(1, "qbuf: buffer length is not enough\n");
			goto done;
		}
		if (VIDEOBUF_NEEDS_INIT != buf->state &&
		    buf->baddr != b->m.userptr)
			q->ops->buf_release(q, buf);
		buf->baddr = b->m.userptr;
#endif
		break;
	case V4L2_MEMORY_OVERLAY:
		buf->boff = b->m.offset;
		break;
#ifdef CONFIG_VIDEO_EMXX
	case V4L2_MEMORY_PHYSADDR:
		break;
#endif
	default:
		dprintk(1, "qbuf: wrong memory type\n");
		goto done;
	}

#ifdef CONFIG_VIDEO_EMXX
	field             = b->field;
	memcpy(&buf->base_addr, &b->m.phys_add, sizeof(struct v4l2_phys_add));
	buf->ts           = b->timestamp;
	buf->width        = b->width;
	buf->height       = b->height;
	buf->bytesperline = b->d_width;
#ifdef CONFIG_VIDEO_EMXX_FILTER
	memcpy(&buf->filter, &b->filter, sizeof(struct v4l2_filter_option));
#endif

	retval = q->ops->buf_prepare(q, buf, field);
	if (0 > retval) {
		dprintk(1, "qbuf: buffer_prepare returned %d\n", retval);
		goto done;
	}

	b->sequence   = q->sequence;
	b->index      = index;

	if (buf->state_frame == STATE_FRAME_IMMEDIATE)
		videobuf_queue_clear(q);
#else
	dprintk(1, "qbuf: requesting next field\n");
	field = videobuf_next_field(q);
	retval = q->ops->buf_prepare(q, buf, field);
	if (0 != retval) {
		dprintk(1, "qbuf: buffer_prepare returned %d\n", retval);
		goto done;
	}
#endif

#ifdef CONFIG_VIDEO_EMXX
	spin_lock_irqsave(q->irqlock, flags);
#endif
	list_add_tail(&buf->stream, &q->stream);
#ifdef CONFIG_VIDEO_EMXX
	spin_unlock_irqrestore(q->irqlock, flags);
#endif

#ifdef CONFIG_VIDEO_EMXX
	dbg_printk(_QBUF_DBG,
			"qbuf:  (%p) ->prev(%p) ->next(%p) "
			"->state(%d) ->sequence(%d)\n",
			&buf->stream, buf->stream.prev, buf->stream.next,
			buf->state, buf->sequence);
#endif

	if (q->streaming) {
		spin_lock_irqsave(q->irqlock, flags);
		q->ops->buf_queue(q, buf);
		spin_unlock_irqrestore(q->irqlock, flags);
	}
	dprintk(1, "qbuf: succeded\n");
	retval = 0;
	wake_up_interruptible_sync(&q->wait);

done:

#if _QBUF_DBG
	if (retval) {
		int i;
		for (i = 0; i < VIDEO_MAX_FRAME; i++) {
			if (q->bufs[i]->state != STATE_NEEDS_INIT &&
			    q->bufs[i]->state != STATE_IDLE) {
				printk(KERN_INFO
				 " @vbq: qbuf: %2d(%p) ->prev(%p) ->next(%p) "
				 "->state(%d) ->sequence(%d)\n",
				 i, &q->bufs[i]->stream,
				 q->bufs[i]->stream.prev,
				 q->bufs[i]->stream.next, q->bufs[i]->state,
				 q->bufs[i]->sequence);
			}
		}
		printk(KERN_INFO "\n");
	}
#endif
	mutex_unlock(&q->vb_lock);

	if (b->memory == V4L2_MEMORY_MMAP)
		up_read(&current->mm->mmap_sem);

	return retval;
}
EXPORT_SYMBOL_GPL(videobuf_qbuf);


/* Locking: Caller holds q->vb_lock */
static int stream_next_buffer_check_queue(struct videobuf_queue *q, int noblock)
{
	int retval;

checks:
	if (!q->streaming) {
		dprintk(1, "next_buffer: Not streaming\n");
		retval = -EINVAL;
		goto done;
	}

	if (list_empty(&q->stream)) {
		if (noblock) {
			retval = -EAGAIN;
			dprintk(2, "next_buffer: no buffers to dequeue\n");
			goto done;
		} else {
			dprintk(2, "next_buffer: waiting on buffer\n");

			/* Drop lock to avoid deadlock with qbuf */
			mutex_unlock(&q->vb_lock);

			/* Checking list_empty and streaming is safe without
			 * locks because we goto checks to validate while
			 * holding locks before proceeding */
			retval = wait_event_interruptible(q->wait,
				!list_empty(&q->stream) || !q->streaming);
			mutex_lock(&q->vb_lock);

			if (retval)
				goto done;

			goto checks;
		}
	}

	retval = 0;

done:
	return retval;
}


/* Locking: Caller holds q->vb_lock */
static int stream_next_buffer(struct videobuf_queue *q,
			struct videobuf_buffer **vb, int nonblocking)
{
	int retval;
	struct videobuf_buffer *buf = NULL;

	retval = stream_next_buffer_check_queue(q, nonblocking);
	if (retval)
		goto done;

	buf = list_entry(q->stream.next, struct videobuf_buffer, stream);
#ifdef CONFIG_VIDEO_EMXX
	retval = videobuf_waiton_dqbuf(buf, nonblocking, 1);
#else
	retval = videobuf_waiton(buf, nonblocking, 1);
#endif
	if (retval < 0)
		goto done;

	*vb = buf;
done:
	return retval;
}

int videobuf_dqbuf(struct videobuf_queue *q,
	       struct v4l2_buffer *b, int nonblocking)
{
	struct videobuf_buffer *buf = NULL;
	int retval;
#ifdef CONFIG_VIDEO_EMXX
	unsigned long flags = 0;
#endif

	MAGIC_CHECK(q->int_ops->magic, MAGIC_QTYPE_OPS);

	mutex_lock(&q->vb_lock);

	retval = stream_next_buffer(q, &buf, nonblocking);
	if (retval < 0) {
		dprintk(1, "dqbuf: next_buffer error: %i\n", retval);
		goto done;
	}

	switch (buf->state) {
	case VIDEOBUF_ERROR:
		dprintk(1, "dqbuf: state is error\n");
		retval = -EIO;
#ifdef CONFIG_VIDEO_EMXX
		q->ops->buf_dqueue(q, buf);
		buf->state = VIDEOBUF_IDLE;
		break;
	case VIDEOBUF_DQBUF_PERMIT:
		dprintk(1, "dqbuf: state is dqbuf permit\n");
		q->ops->buf_dqueue(q, buf);
#else
		CALL(q, sync, q, buf);
		buf->state = VIDEOBUF_IDLE;
		break;
	case VIDEOBUF_DONE:
		dprintk(1, "dqbuf: state is done\n");
		CALL(q, sync, q, buf);
#endif
		buf->state = VIDEOBUF_IDLE;
		break;
	default:
		dprintk(1, "dqbuf: state invalid\n");
		retval = -EINVAL;
		goto done;
	}

#ifdef CONFIG_VIDEO_EMXX
	dbg_printk(_QBUF_DBG,
			"dqbuf: (%p) ->prev(%p) ->next(%p) "
			"->state(%d) ->sequence(%d)\n",
			&buf->stream, buf->stream.prev, buf->stream.next,
			buf->state, buf->sequence);
#endif

#ifdef CONFIG_VIDEO_EMXX
	spin_lock_irqsave(q->irqlock, flags);
#endif
	list_del(&buf->stream);
#ifdef CONFIG_VIDEO_EMXX
	spin_unlock_irqrestore(q->irqlock, flags);
#endif
	memset(b, 0, sizeof(*b));
	videobuf_status(q, b, buf, q->type);

done:
#if _QBUF_DBG
	if (retval) {
		int i;
		for (i = 0; i < VIDEO_MAX_FRAME; i++) {
			if (q->bufs[i]->state != STATE_NEEDS_INIT &&
			    q->bufs[i]->state != STATE_IDLE) {
				printk(KERN_INFO
				 " @vbq: dqbuf: %2d(%p) ->prev(%p) ->next(%p) "
				 "->state(%d) ->sequence(%d)\n",
				 i, &q->bufs[i]->stream,
				 q->bufs[i]->stream.prev,
				 q->bufs[i]->stream.next, q->bufs[i]->state,
				 q->bufs[i]->sequence);
			}
		}
		printk(KERN_INFO "\n");
	}
#endif
	mutex_unlock(&q->vb_lock);
	return retval;
}
EXPORT_SYMBOL_GPL(videobuf_dqbuf);

int videobuf_streamon(struct videobuf_queue *q)
{
	struct videobuf_buffer *buf;
	unsigned long flags = 0;
	int retval;

	mutex_lock(&q->vb_lock);
	retval = -EBUSY;
	if (q->reading)
		goto done;
	retval = 0;
	if (q->streaming)
		goto done;
	q->streaming = 1;
	spin_lock_irqsave(q->irqlock, flags);
	list_for_each_entry(buf, &q->stream, stream)
		if (buf->state == VIDEOBUF_PREPARED)
#ifdef CONFIG_VIDEO_EMXX
			{
				q->ops->buf_queue(q, buf);
				break;
			}
#else
			q->ops->buf_queue(q, buf);
#endif
	spin_unlock_irqrestore(q->irqlock, flags);

	wake_up_interruptible_sync(&q->wait);
 done:
	mutex_unlock(&q->vb_lock);
	return retval;
}
EXPORT_SYMBOL_GPL(videobuf_streamon);

/* Locking: Caller holds q->vb_lock */
static int __videobuf_streamoff(struct videobuf_queue *q)
{
	if (!q->streaming)
		return -EINVAL;

	videobuf_queue_cancel(q);

	return 0;
}

int videobuf_streamoff(struct videobuf_queue *q)
{
	int retval;

	mutex_lock(&q->vb_lock);
	retval = __videobuf_streamoff(q);
	mutex_unlock(&q->vb_lock);

	return retval;
}
EXPORT_SYMBOL_GPL(videobuf_streamoff);

/* Locking: Caller holds q->vb_lock */
static ssize_t videobuf_read_zerocopy(struct videobuf_queue *q,
				      char __user *data,
				      size_t count, loff_t *ppos)
{
	enum v4l2_field field;
	unsigned long flags = 0;
	int retval;

	MAGIC_CHECK(q->int_ops->magic, MAGIC_QTYPE_OPS);

	/* setup stuff */
	q->read_buf = videobuf_alloc(q);
	if (NULL == q->read_buf)
		return -ENOMEM;

	q->read_buf->memory = V4L2_MEMORY_USERPTR;
	q->read_buf->baddr  = (unsigned long)data;
	q->read_buf->bsize  = count;

	field = videobuf_next_field(q);
	retval = q->ops->buf_prepare(q, q->read_buf, field);
	if (0 != retval)
		goto done;

	/* start capture & wait */
	spin_lock_irqsave(q->irqlock, flags);
	q->ops->buf_queue(q, q->read_buf);
	spin_unlock_irqrestore(q->irqlock, flags);
	retval = videobuf_waiton(q->read_buf, 0, 0);
	if (0 == retval) {
		CALL(q, sync, q, q->read_buf);
		if (VIDEOBUF_ERROR == q->read_buf->state)
			retval = -EIO;
		else
			retval = q->read_buf->size;
	}

 done:
	/* cleanup */
	q->ops->buf_release(q, q->read_buf);
	kfree(q->read_buf);
	q->read_buf = NULL;
	return retval;
}

ssize_t videobuf_read_one(struct videobuf_queue *q,
			  char __user *data, size_t count, loff_t *ppos,
			  int nonblocking)
{
	enum v4l2_field field;
	unsigned long flags = 0;
	unsigned size = 0, nbufs = 1;
	int retval;

	MAGIC_CHECK(q->int_ops->magic, MAGIC_QTYPE_OPS);

	mutex_lock(&q->vb_lock);

	q->ops->buf_setup(q, &nbufs, &size);

	if (NULL == q->read_buf  &&
	    count >= size        &&
	    !nonblocking) {
		retval = videobuf_read_zerocopy(q, data, count, ppos);
		if (retval >= 0  ||  retval == -EIO)
			/* ok, all done */
			goto done;
		/* fallback to kernel bounce buffer on failures */
	}

	if (NULL == q->read_buf) {
		/* need to capture a new frame */
		retval = -ENOMEM;
		q->read_buf = videobuf_alloc(q);

		dprintk(1, "video alloc=0x%p\n", q->read_buf);
		if (NULL == q->read_buf)
			goto done;
		q->read_buf->memory = V4L2_MEMORY_USERPTR;
		q->read_buf->bsize = count; /* preferred size */
		field = videobuf_next_field(q);
		retval = q->ops->buf_prepare(q, q->read_buf, field);

		if (0 != retval) {
			kfree(q->read_buf);
			q->read_buf = NULL;
			goto done;
		}

		spin_lock_irqsave(q->irqlock, flags);
		q->ops->buf_queue(q, q->read_buf);
		spin_unlock_irqrestore(q->irqlock, flags);

		q->read_off = 0;
	}

	/* wait until capture is done */
	retval = videobuf_waiton(q->read_buf, nonblocking, 1);
	if (0 != retval)
		goto done;

	CALL(q, sync, q, q->read_buf);

	if (VIDEOBUF_ERROR == q->read_buf->state) {
		/* catch I/O errors */
		q->ops->buf_release(q, q->read_buf);
		kfree(q->read_buf);
		q->read_buf = NULL;
		retval = -EIO;
		goto done;
	}

	/* Copy to userspace */
	retval = CALL(q, video_copy_to_user, q, data, count, nonblocking);
	if (retval < 0)
		goto done;

	q->read_off += retval;
	if (q->read_off == q->read_buf->size) {
		/* all data copied, cleanup */
		q->ops->buf_release(q, q->read_buf);
		kfree(q->read_buf);
		q->read_buf = NULL;
	}

 done:
	mutex_unlock(&q->vb_lock);
	return retval;
}
EXPORT_SYMBOL_GPL(videobuf_read_one);

/* Locking: Caller holds q->vb_lock */
static int __videobuf_read_start(struct videobuf_queue *q)
{
	enum v4l2_field field;
	unsigned long flags = 0;
	unsigned int count = 0, size = 0;
	int err, i;

	q->ops->buf_setup(q, &count, &size);
	if (count < 2)
		count = 2;
	if (count > VIDEO_MAX_FRAME)
		count = VIDEO_MAX_FRAME;
	size = PAGE_ALIGN(size);

	err = __videobuf_mmap_setup(q, count, size, V4L2_MEMORY_USERPTR);
	if (err < 0)
		return err;

	count = err;

	for (i = 0; i < count; i++) {
		field = videobuf_next_field(q);
		err = q->ops->buf_prepare(q, q->bufs[i], field);
		if (err)
			return err;
		list_add_tail(&q->bufs[i]->stream, &q->stream);
	}
	spin_lock_irqsave(q->irqlock, flags);
	for (i = 0; i < count; i++)
		q->ops->buf_queue(q, q->bufs[i]);
	spin_unlock_irqrestore(q->irqlock, flags);
	q->reading = 1;
	return 0;
}

static void __videobuf_read_stop(struct videobuf_queue *q)
{
	int i;

	videobuf_queue_cancel(q);
	__videobuf_mmap_free(q);
	INIT_LIST_HEAD(&q->stream);
	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		if (NULL == q->bufs[i])
			continue;
		kfree(q->bufs[i]);
		q->bufs[i] = NULL;
	}
	q->read_buf = NULL;

}

int videobuf_read_start(struct videobuf_queue *q)
{
	int rc;

	mutex_lock(&q->vb_lock);
	rc = __videobuf_read_start(q);
	mutex_unlock(&q->vb_lock);

	return rc;
}
EXPORT_SYMBOL_GPL(videobuf_read_start);

void videobuf_read_stop(struct videobuf_queue *q)
{
	mutex_lock(&q->vb_lock);
	__videobuf_read_stop(q);
	mutex_unlock(&q->vb_lock);
}
EXPORT_SYMBOL_GPL(videobuf_read_stop);

void videobuf_stop(struct videobuf_queue *q)
{
	mutex_lock(&q->vb_lock);

	if (q->streaming)
		__videobuf_streamoff(q);

	if (q->reading)
		__videobuf_read_stop(q);

	mutex_unlock(&q->vb_lock);
}
EXPORT_SYMBOL_GPL(videobuf_stop);


ssize_t videobuf_read_stream(struct videobuf_queue *q,
			     char __user *data, size_t count, loff_t *ppos,
			     int vbihack, int nonblocking)
{
	int rc, retval;
	unsigned long flags = 0;

	MAGIC_CHECK(q->int_ops->magic, MAGIC_QTYPE_OPS);

	dprintk(2, "%s\n", __func__);
	mutex_lock(&q->vb_lock);
	retval = -EBUSY;
	if (q->streaming)
		goto done;
	if (!q->reading) {
		retval = __videobuf_read_start(q);
		if (retval < 0)
			goto done;
	}

	retval = 0;
	while (count > 0) {
		/* get / wait for data */
		if (NULL == q->read_buf) {
			q->read_buf = list_entry(q->stream.next,
						 struct videobuf_buffer,
						 stream);
			list_del(&q->read_buf->stream);
			q->read_off = 0;
		}
		rc = videobuf_waiton(q->read_buf, nonblocking, 1);
		if (rc < 0) {
			if (0 == retval)
				retval = rc;
			break;
		}

		if (q->read_buf->state == VIDEOBUF_DONE) {
			rc = CALL(q, copy_stream, q, data + retval, count,
					retval, vbihack, nonblocking);
			if (rc < 0) {
				retval = rc;
				break;
			}
			retval      += rc;
			count       -= rc;
			q->read_off += rc;
		} else {
			/* some error */
			q->read_off = q->read_buf->size;
			if (0 == retval)
				retval = -EIO;
		}

		/* requeue buffer when done with copying */
		if (q->read_off == q->read_buf->size) {
			list_add_tail(&q->read_buf->stream,
				      &q->stream);
			spin_lock_irqsave(q->irqlock, flags);
			q->ops->buf_queue(q, q->read_buf);
			spin_unlock_irqrestore(q->irqlock, flags);
			q->read_buf = NULL;
		}
		if (retval < 0)
			break;
	}

 done:
	mutex_unlock(&q->vb_lock);
	return retval;
}
EXPORT_SYMBOL_GPL(videobuf_read_stream);

unsigned int videobuf_poll_stream(struct file *file,
				  struct videobuf_queue *q,
				  poll_table *wait)
{
	struct videobuf_buffer *buf = NULL;
	unsigned int rc = 0;

	mutex_lock(&q->vb_lock);
	if (q->streaming) {
		if (!list_empty(&q->stream))
			buf = list_entry(q->stream.next,
					 struct videobuf_buffer, stream);
	} else {
#ifndef CONFIG_VIDEO_EMXX
		if (!q->reading)
			__videobuf_read_start(q);
		if (!q->reading) {
			rc = POLLERR;
		} else if (NULL == q->read_buf) {
			q->read_buf = list_entry(q->stream.next,
						 struct videobuf_buffer,
						 stream);
			list_del(&q->read_buf->stream);
			q->read_off = 0;
		}
		buf = q->read_buf;
#endif
	}
#ifdef CONFIG_VIDEO_EMXX
	if (!buf) {
		mutex_unlock(&q->vb_lock);
		return rc;
	}

	if (0 == rc) {
		if (buf->state == VIDEOBUF_DQBUF_PERMIT ||
		    buf->state == VIDEOBUF_ERROR) {
			rc = POLLIN|POLLRDNORM;
			goto done;
		}
		dbg_printk(_POLL_DBG, "poll:  (%p) ->done(%p) ->state(%d)\n",
			&buf->stream, &buf->done, buf->state);
		poll_wait(file, &buf->done, wait);
		if (buf->state == VIDEOBUF_DQBUF_PERMIT ||
		    buf->state == VIDEOBUF_ERROR)
			rc = POLLIN|POLLRDNORM;
	}
 done:
#else
	if (!buf)
		rc = POLLERR;

	if (0 == rc) {
		poll_wait(file, &buf->done, wait);
		if (buf->state == VIDEOBUF_DONE ||
		    buf->state == VIDEOBUF_ERROR)
			rc = POLLIN|POLLRDNORM;
	}
#endif
	mutex_unlock(&q->vb_lock);
	return rc;
}
EXPORT_SYMBOL_GPL(videobuf_poll_stream);

int videobuf_mmap_mapper(struct videobuf_queue *q,
			 struct vm_area_struct *vma)
{
	int retval;

	MAGIC_CHECK(q->int_ops->magic, MAGIC_QTYPE_OPS);

	mutex_lock(&q->vb_lock);
	retval = CALL(q, mmap_mapper, q, vma);
	q->is_mmapped = 1;
	mutex_unlock(&q->vb_lock);

	return retval;
}
EXPORT_SYMBOL_GPL(videobuf_mmap_mapper);

#ifdef CONFIG_VIDEO_V4L1_COMPAT
int videobuf_cgmbuf(struct videobuf_queue *q,
		    struct video_mbuf *mbuf, int count)
{
	struct v4l2_requestbuffers req;
	int rc, i;

	MAGIC_CHECK(q->int_ops->magic, MAGIC_QTYPE_OPS);

	memset(&req, 0, sizeof(req));
	req.type   = q->type;
	req.count  = count;
	req.memory = V4L2_MEMORY_MMAP;
	rc = videobuf_reqbufs(q, &req);
	if (rc < 0)
		return rc;

	mbuf->frames = req.count;
	mbuf->size   = 0;
	for (i = 0; i < mbuf->frames; i++) {
		mbuf->offsets[i]  = q->bufs[i]->boff;
		mbuf->size       += q->bufs[i]->bsize;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(videobuf_cgmbuf);
#endif

