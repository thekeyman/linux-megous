/*
 * Copyright (c) 2017 Magewell Electronics Co., Ltd. (Nanjing).
 * All rights reserved.
 * Author: Yong Deng <yong.deng@magewell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define DEBUG

#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/slab.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#include "sun6i_csi.h"

// {{{ vb2

struct sun6i_csi_buffer {
	struct vb2_v4l2_buffer		vb;
	struct list_head		list;

	dma_addr_t			dma_addr;
};

static int sun6i_video_queue_setup(struct vb2_queue *vq,
				 unsigned int *nbuffers, unsigned int *nplanes,
				 unsigned int sizes[],
				 struct device *alloc_devs[])
{
	struct sun6i_csi *csi = vb2_get_drv_priv(vq);
	unsigned int size = csi->fmt.fmt.pix.sizeimage;

	if (*nplanes)
		return sizes[0] < size ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = size;

	return 0;
}

static int sun6i_video_buffer_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct sun6i_csi_buffer *buf =
			container_of(vbuf, struct sun6i_csi_buffer, vb);
	struct sun6i_csi *csi = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = csi->fmt.fmt.pix.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		v4l2_err(csi->vdev.v4l2_dev, "buffer too small (%lu < %lu)\n",
			 vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);

	buf->dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);

	vbuf->field = csi->fmt.fmt.pix.field;

	return 0;
}

static int sun6i_pipeline_set_stream(struct sun6i_csi *csi, bool enable)
{
	struct media_entity *entity;
	struct media_pad *pad;
	struct v4l2_subdev *subdev;
	int ret;

	entity = &csi->vdev.entity;
	while (1) {
		pad = &entity->pads[0];
		if (!(pad->flags & MEDIA_PAD_FL_SINK))
			break;

		pad = media_entity_remote_pad(pad);
		if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
			break;

		entity = pad->entity;
		subdev = media_entity_to_v4l2_subdev(entity);

		ret = v4l2_subdev_call(subdev, video, s_stream, enable);
		if (enable && ret < 0 && ret != -ENOIOCTLCMD)
			return ret;
	}

	return 0;
}

static int sun6i_video_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct sun6i_csi *csi = vb2_get_drv_priv(vq);
	struct sun6i_csi_buffer *buf;
	unsigned long flags;
	int ret;

	csi->sequence = 0;

	ret = media_pipeline_start(&csi->vdev.entity, &csi->vdev.pipe);
	if (ret < 0)
		goto err_start_pipeline;

	ret = sun6i_pipeline_set_stream(csi, true);
	if (ret < 0)
		goto err_start_stream;

	ret = sun6i_csi_apply_config(csi);
	if (ret < 0)
		goto err_apply_config;

	spin_lock_irqsave(&csi->dma_queue_lock, flags);
	csi->cur_frm = list_first_entry(&csi->dma_queue,
					  struct sun6i_csi_buffer, list);
	list_del(&csi->cur_frm->list);
	spin_unlock_irqrestore(&csi->dma_queue_lock, flags);

	ret = sun6i_csi_update_buf_addr(csi, csi->cur_frm->dma_addr);
	if (ret < 0)
		goto err_update_addr;

	ret = sun6i_csi_set_stream(csi, true);
	if (ret < 0)
		goto err_csi_stream;

	return 0;

err_csi_stream:
err_update_addr:
err_apply_config:
	sun6i_pipeline_set_stream(csi, false);
err_start_stream:
	media_pipeline_stop(&csi->vdev.entity);
err_start_pipeline:
	spin_lock_irqsave(&csi->dma_queue_lock, flags);
	list_for_each_entry(buf, &csi->dma_queue, list)
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
	INIT_LIST_HEAD(&csi->dma_queue);
	spin_unlock_irqrestore(&csi->dma_queue_lock, flags);

	return ret;
}

static void sun6i_video_stop_streaming(struct vb2_queue *vq)
{
	struct sun6i_csi *csi = vb2_get_drv_priv(vq);
	unsigned long flags;
	struct sun6i_csi_buffer *buf;

	sun6i_pipeline_set_stream(csi, false);

	sun6i_csi_set_stream(csi, false);

	media_pipeline_stop(&csi->vdev.entity);

	/* Release all active buffers */
	spin_lock_irqsave(&csi->dma_queue_lock, flags);
	if (unlikely(csi->cur_frm)) {
		vb2_buffer_done(&csi->cur_frm->vb.vb2_buf,
				VB2_BUF_STATE_ERROR);
		csi->cur_frm = NULL;
	}
	list_for_each_entry(buf, &csi->dma_queue, list)
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	INIT_LIST_HEAD(&csi->dma_queue);
	spin_unlock_irqrestore(&csi->dma_queue_lock, flags);
}

static void sun6i_video_buffer_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct sun6i_csi_buffer *buf =
			container_of(vbuf, struct sun6i_csi_buffer, vb);
	struct sun6i_csi *csi = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long flags;

	spin_lock_irqsave(&csi->dma_queue_lock, flags);
	if (!csi->cur_frm && list_empty(&csi->dma_queue) &&
		vb2_is_streaming(vb->vb2_queue)) {
		csi->cur_frm = buf;
		sun6i_csi_update_buf_addr(csi, csi->cur_frm->dma_addr);
		sun6i_csi_set_stream(csi, 1);
	} else
		list_add_tail(&buf->list, &csi->dma_queue);
	spin_unlock_irqrestore(&csi->dma_queue_lock, flags);
}

void sun6i_video_frame_done(struct sun6i_csi *csi)
{
	spin_lock(&csi->dma_queue_lock);

	if (csi->cur_frm) {
		struct vb2_v4l2_buffer *vbuf = &csi->cur_frm->vb;
		struct vb2_buffer *vb = &vbuf->vb2_buf;

		vb->timestamp = ktime_get_ns();
		vbuf->sequence = csi->sequence++;
		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
		csi->cur_frm = NULL;
	}

	if (!list_empty(&csi->dma_queue)
	    && vb2_is_streaming(&csi->vb2_vidq)) {
		csi->cur_frm = list_first_entry(&csi->dma_queue,
				struct sun6i_csi_buffer, list);
		list_del(&csi->cur_frm->list);
		sun6i_csi_update_buf_addr(csi, csi->cur_frm->dma_addr);
	} else
		sun6i_csi_set_stream(csi, 0);

	spin_unlock(&csi->dma_queue_lock);
}

static struct vb2_ops sun6i_csi_vb2_ops = {
	.queue_setup		= sun6i_video_queue_setup,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.buf_prepare		= sun6i_video_buffer_prepare,
	.start_streaming	= sun6i_video_start_streaming,
	.stop_streaming		= sun6i_video_stop_streaming,
	.buf_queue		= sun6i_video_buffer_queue,
};

// }}}
// {{{ videodev ioctl

static struct v4l2_subdev *
sun6i_video_remote_subdev(struct sun6i_csi *csi, u32 *pad)
{
	struct media_pad *remote;

	remote = media_entity_remote_pad(&csi->pad);

	if (!remote || !is_media_entity_v4l2_subdev(remote->entity))
		return NULL;

	if (pad)
		*pad = remote->index;

	return media_entity_to_v4l2_subdev(remote->entity);
}

static struct sun6i_csi_format *
find_format_by_fourcc(struct sun6i_csi *csi, unsigned int fourcc)
{
	unsigned int num_formats = csi->num_formats;
	struct sun6i_csi_format *fmt;
	unsigned int i;

	for (i = 0; i < num_formats; i++) {
		fmt = &csi->formats[i];
		if (fmt->fourcc == fourcc)
			return fmt;
	}

	return NULL;
}

static int sun6i_video_try_fmt(struct sun6i_csi *csi, struct v4l2_format *f,
			       struct sun6i_csi_format **current_fmt)
{
	struct sun6i_csi_format *csi_fmt;
	struct v4l2_pix_format *pixfmt = &f->fmt.pix;
	struct v4l2_subdev_format format;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	subdev = sun6i_video_remote_subdev(csi, &pad);
	if (subdev == NULL)
		return -ENXIO;

	csi_fmt = find_format_by_fourcc(csi, pixfmt->pixelformat);
	if (csi_fmt == NULL)
		return -EINVAL;

	format.pad = pad;
	format.which = V4L2_SUBDEV_FORMAT_TRY;
	v4l2_fill_mbus_format(&format.format, pixfmt, csi_fmt->mbus_code);
	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &format);
	if (ret)
		return ret;

	v4l2_fill_pix_format(pixfmt, &format.format);

	pixfmt->bytesperline = (pixfmt->width * csi_fmt->bpp) >> 3;
	pixfmt->sizeimage = (pixfmt->width * csi_fmt->bpp * pixfmt->height) / 8;

	if (current_fmt)
		*current_fmt = csi_fmt;

	return 0;
}

static int sun6i_video_set_fmt(struct sun6i_csi *csi, struct v4l2_format *f)
{
	struct v4l2_subdev_format format;
	struct sun6i_csi_format *current_fmt;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	subdev = sun6i_video_remote_subdev(csi, &pad);
	if (subdev == NULL)
		return -ENXIO;

	ret = sun6i_video_try_fmt(csi, f, &current_fmt);
	if (ret)
		return ret;

	format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	v4l2_fill_mbus_format(&format.format, &f->fmt.pix,
			      current_fmt->mbus_code);
	ret = v4l2_subdev_call(subdev, pad, set_fmt, NULL, &format);
	if (ret < 0)
		return ret;

	csi->fmt = *f;
	csi->current_fmt = current_fmt;

	return 0;
}

static int sun6i_querycap(struct file *file, void *priv,
				struct v4l2_capability *cap)
{
	struct sun6i_csi *csi = video_drvdata(file);

	strlcpy(cap->driver, "sun6i-video", sizeof(cap->driver));
	strlcpy(cap->card, csi->vdev.name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 csi->dev->of_node->name);

	return 0;
}

static int sun6i_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct sun6i_csi *csi = video_drvdata(file);

	return sun6i_video_try_fmt(csi, f, NULL);
}

static int sun6i_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct sun6i_csi *csi = video_drvdata(file);

	*fmt = csi->fmt;

	return 0;
}

static int sun6i_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct sun6i_csi *csi = video_drvdata(file);

	if (vb2_is_streaming(&csi->vb2_vidq))
		return -EBUSY;

	return sun6i_video_set_fmt(csi, f);
}

static int sun6i_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	struct sun6i_csi *csi = video_drvdata(file);
	u32 index = f->index;

	if (index >= csi->num_formats)
		return -EINVAL;

	f->pixelformat = csi->formats[index].fourcc;

	return 0;
}

static int sun6i_enum_input(struct file *file, void *priv,
			   struct v4l2_input *i)
{
	struct sun6i_csi *csi = video_drvdata(file);
	int ret;

	if (i->index != 0 || !csi->sensor_subdev)
		return -EINVAL;

	ret = v4l2_subdev_call(csi->sensor_subdev, video, g_input_status, &i->status);
	if (ret < 0 && ret != -ENOIOCTLCMD && ret != -ENODEV)
		return ret;

	i->type = V4L2_INPUT_TYPE_CAMERA;

	strlcpy(i->name, "Camera", sizeof(i->name));

	return 0;
}

static int sun6i_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;

	return 0;
}

static int sun6i_s_input(struct file *file, void *priv, unsigned int i)
{
	if (i > 0)
		return -EINVAL;

	return 0;
}

static const struct v4l2_ioctl_ops sun6i_video_ioctl_ops = {
	.vidioc_querycap		= sun6i_querycap,
	.vidioc_try_fmt_vid_cap		= sun6i_try_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap		= sun6i_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap		= sun6i_s_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap	= sun6i_enum_fmt_vid_cap,

	.vidioc_enum_input		= sun6i_enum_input,
	.vidioc_g_input			= sun6i_g_input,
	.vidioc_s_input			= sun6i_s_input,

	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,

	.vidioc_log_status		= v4l2_ctrl_log_status,
};

// }}}
// {{{ videodev fops

/* -----------------------------------------------------------------------------
 * V4L2 file operations
 */
static int sun6i_video_open(struct file *file)
{
	struct sun6i_csi *csi = video_drvdata(file);
	struct v4l2_format format;
	int ret;

	if (mutex_lock_interruptible(&csi->lock))
		return -ERESTARTSYS;

	ret = v4l2_fh_open(file);
	if (ret < 0)
		goto unlock;

	ret = v4l2_pipeline_pm_use(&csi->vdev.entity, 1);
	if (ret < 0)
		goto fh_release;

	if (!v4l2_fh_is_singular_file(file))
		goto fh_release;

	ret = sun6i_csi_set_power(csi, true);
	if (ret < 0)
		goto fh_release;

	/* setup default format */
	if (csi->num_formats > 0) {
		format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		format.fmt.pix.width = 1280;
		format.fmt.pix.height = 720;
		format.fmt.pix.pixelformat = csi->formats[0].fourcc;
		sun6i_video_set_fmt(csi, &format);
	}

	mutex_unlock(&csi->lock);
	return 0;

fh_release:
	v4l2_fh_release(file);
unlock:
	mutex_unlock(&csi->lock);
	return ret;
}

static int sun6i_video_close(struct file *file)
{
	struct sun6i_csi *csi = video_drvdata(file);
	bool last_fh;

	mutex_lock(&csi->lock);

	last_fh = v4l2_fh_is_singular_file(file);

	_vb2_fop_release(file, NULL);

	v4l2_pipeline_pm_use(&csi->vdev.entity, 0);

	if (last_fh)
		sun6i_csi_set_power(csi, false);

	mutex_unlock(&csi->lock);

	return 0;
}

static const struct v4l2_file_operations sun6i_video_fops = {
	.owner		= THIS_MODULE,
	.open		= sun6i_video_open,
	.release	= sun6i_video_close,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vb2_fop_mmap,
	.poll		= vb2_fop_poll
};

// }}}
// {{{ media ops

/* -----------------------------------------------------------------------------
 * Media Operations
 */
static int sun6i_video_formats_init(struct sun6i_csi *csi)
{
	struct v4l2_subdev_mbus_code_enum mbus_code = { 0 };
	struct v4l2_subdev *subdev;
	u32 pad;
	const u32 *pixformats;
	int pixformat_count = 0;
	u32 subdev_codes[32]; /*XXX: subdev format codes, 32 should be enough (overflow) */
	int codes_count = 0;
	int num_fmts = 0;
	int i, j;

	subdev = sun6i_video_remote_subdev(csi, &pad);
	if (subdev == NULL)
		return -ENXIO;

	/* Get supported pixformats of CSI */
	pixformat_count = sun6i_csi_get_supported_pixformats(csi, &pixformats);
	if (pixformat_count <= 0)
		return -ENXIO;

	/* Get subdev formats codes */
	mbus_code.pad = pad;
	mbus_code.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	while (!v4l2_subdev_call(subdev, pad, enum_mbus_code, NULL,
			&mbus_code)) {
		subdev_codes[codes_count] = mbus_code.code;
		codes_count++;
		mbus_code.index++;
	}

	if (!codes_count)
		return -ENXIO;

	/* Get supported formats count */
	for (i = 0; i < codes_count; i++) {
		for (j = 0; j < pixformat_count; j++) {
			if (!sun6i_csi_is_format_support(csi, pixformats[j],
					mbus_code.code)) {
				continue;
			}
			num_fmts++;
		}
	}

	if (!num_fmts)
		return -ENXIO;

	//XXX: memory leak
	csi->num_formats = num_fmts;
	csi->formats = devm_kcalloc(csi->dev, num_fmts,
			sizeof(struct sun6i_csi_format), GFP_KERNEL);
	if (!csi->formats) {
		dev_err(csi->dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	/* Get supported formats */
	num_fmts = 0;
	for (i = 0; i < codes_count; i++) {
		for (j = 0; j < pixformat_count; j++) {
			if (!sun6i_csi_is_format_support(csi, pixformats[j],
					mbus_code.code)) {
				continue;
			}

			csi->formats[num_fmts].fourcc = pixformats[j];
			csi->formats[num_fmts].mbus_code =
					mbus_code.code;
			csi->formats[num_fmts].bpp =
					v4l2_pixformat_get_bpp(pixformats[j]);
			num_fmts++;
		}
	}

	return 0;
}

static int sun6i_video_link_setup(struct media_entity *entity,
				  const struct media_pad *local,
				  const struct media_pad *remote, u32 flags)
{
	struct video_device *vdev = media_entity_to_video_device(entity);
	struct sun6i_csi *csi = video_get_drvdata(vdev);

	if (WARN_ON(csi == NULL))
		return 0;

	return sun6i_video_formats_init(csi);
}

static const struct media_entity_operations sun6i_video_media_ops = {
	.link_setup = sun6i_video_link_setup,
};

// }}}
// {{{ sun6i_csi video init/cleanup

static void sun6i_video_cleanup(struct sun6i_csi *csi)
{
	if (video_is_registered(&csi->vdev))
		video_unregister_device(&csi->vdev);

	media_entity_cleanup(&csi->vdev.entity);
}

static int sun6i_video_init(struct sun6i_csi *csi, const char *name)
{
	struct video_device *vdev = &csi->vdev;
	struct vb2_queue *vidq = &csi->vb2_vidq;
	int ret;

	/* Initialize the media entity... */
	csi->pad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	vdev->entity.ops = &sun6i_video_media_ops;
	ret = media_entity_pads_init(&vdev->entity, 1, &csi->pad);
	if (ret < 0)
		return ret;

	mutex_init(&csi->lock);

	INIT_LIST_HEAD(&csi->dma_queue);
	spin_lock_init(&csi->dma_queue_lock);

	csi->cur_frm = NULL;
	csi->sequence = 0;
	csi->num_formats = 0;

	/* Initialize videobuf2 queue */
	vidq->type			= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vidq->io_modes			= VB2_MMAP | VB2_DMABUF;
	vidq->drv_priv			= csi;
	vidq->buf_struct_size		= sizeof(struct sun6i_csi_buffer);
	vidq->ops			= &sun6i_csi_vb2_ops;
	vidq->mem_ops			= &vb2_dma_contig_memops;
	vidq->timestamp_flags		= V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	vidq->lock			= &csi->lock;
	vidq->min_buffers_needed	= 1;
	vidq->dev			= csi->dev;

	ret = vb2_queue_init(vidq);
	if (ret) {
		v4l2_err(&csi->v4l2_dev, "vb2_queue_init failed: %d\n", ret);
		goto error;
	}

	/* Register video device */
	strlcpy(vdev->name, name, sizeof(vdev->name));
	vdev->release		= video_device_release_empty;
	vdev->fops		= &sun6i_video_fops;
	vdev->ioctl_ops		= &sun6i_video_ioctl_ops;
	vdev->vfl_type		= VFL_TYPE_GRABBER;
	vdev->vfl_dir		= VFL_DIR_RX;
	vdev->v4l2_dev		= &csi->v4l2_dev;
	vdev->queue		= vidq;
	vdev->lock		= &csi->lock;
	vdev->device_caps	= V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE;
	video_set_drvdata(vdev, csi);

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret < 0) {
		v4l2_err(&csi->v4l2_dev,
			 "video_register_device failed: %d\n", ret);
		goto error;
	}

	return 0;

error:
	sun6i_video_cleanup(csi);
	return ret;
}

// }}}
// {{{ sun6i_csi init/cleanup - DT parsing/subdev setup

struct sun6i_csi_async_subdev {
	struct v4l2_async_subdev asd; /* must be first */
};

#define notifier_to_csi(n) container_of(n, struct sun6i_csi, notifier)
#define asd_to_csi_asd(a) container_of(a, struct sun6i_csi_async_subdev, asd)

static int sun6i_csi_notify_bound(struct v4l2_async_notifier *notifier,
				     struct v4l2_subdev *subdev,
				     struct v4l2_async_subdev *asd)
{
	struct sun6i_csi *csi = notifier_to_csi(notifier);
	struct sun6i_csi_async_subdev *csi_asd = asd_to_csi_asd(asd);
	//int ret;

	dev_dbg(csi->dev, "bound subdev %s\n", subdev->name);

	if (subdev->entity.function != MEDIA_ENT_F_CAM_SENSOR) {
		dev_err(csi->dev, "bound subdev %s - not a camera sensor\n", subdev->name);
		return -EINVAL;
	}

	csi->sensor_subdev = subdev;
	v4l2_set_subdev_hostdata(subdev, csi);

	return 0;
}

static void sun6i_csi_notify_unbind(struct v4l2_async_notifier *notifier,
				       struct v4l2_subdev *subdev,
				       struct v4l2_async_subdev *asd)
{
	struct sun6i_csi *csi = notifier_to_csi(notifier);
	struct sun6i_csi_async_subdev *csi_asd = asd_to_csi_asd(asd);

	dev_err(csi->dev, "unbind subdev %s\n", subdev->name);

	if (csi->sensor_subdev == subdev) {
		csi->sensor_subdev = NULL;
	}
}

static int sun6i_csi_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct sun6i_csi *csi = notifier_to_csi(notifier);
	struct v4l2_subdev *subdev;
	struct media_entity *sink = &csi->vdev.entity;
	unsigned int pad;
	int ret;

	dev_dbg(csi->dev, "notify complete, all subdevs bound\n");

        subdev = csi->sensor_subdev;
	if (subdev) {
		for (pad = 0; pad < subdev->entity.num_pads; pad++) {
			if (subdev->entity.pads[pad].flags & MEDIA_PAD_FL_SOURCE) {
				ret = media_create_pad_link(&subdev->entity, pad, sink, 0, MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
				if (ret)
					return ret;

				dev_dbg(csi->dev, "created pad link %s:%u -> %s:0\n", subdev->name, pad, csi->vdev.name);

				ret = media_entity_call(sink, link_setup, &sink->pads[0], &subdev->entity.pads[pad], 0);
				if (ret)
					return ret;

				goto register_subdevs;
			}
		}

		dev_err(csi->dev, "bound subdev %s - no source pad found\n", subdev->name);
		return -EINVAL;
	}

register_subdevs:
	ret = v4l2_device_register_subdev_nodes(&csi->v4l2_dev);
	if (ret < 0) {
		dev_err(csi->dev, "failed to register subdev nodes\n");
		return ret;
	}

	dev_dbg(csi->dev, "registering media device\n");

	return media_device_register(&csi->media_dev);
}

// this is called for each controller endpoint
static int sun6i_csi_parse_subdev_endpoint(struct device *dev,
				   struct v4l2_fwnode_endpoint *vep,
				   struct v4l2_async_subdev *asd)
{
	struct sun6i_csi *csi = dev_get_drvdata(dev); // this is really a struct sun6i_csi_dev, struct sun6i_csi is its first member
	//struct sun6i_csi_async_subdev *csi_asd = asd_to_csi_asd(asd);

	// ony one endpoint is supported
	//XXX: we will need to support multiple switchable endpoints (or can
	//this be done by some multiplexer code?)
	if (vep->base.port || vep->base.id)
		return -ENOTCONN;

	switch (vep->bus_type) {
	case V4L2_MBUS_PARALLEL:
		dev_dbg(csi->dev, "Found PARALLEL media bus endpoint\n");

		csi->bus_type = vep->bus_type;
		csi->bus_width = vep->bus.parallel.bus_width;
		csi->bus_flags = vep->bus.parallel.flags;
		return 0;
	default:
		dev_err(csi->dev, "Unsupported media bus type\n");
		return -EINVAL;
	}
}

int sun6i_csi_init(struct sun6i_csi *csi)
{
	int ret;

	csi->media_dev.dev = csi->dev;
	strlcpy(csi->media_dev.model, "Allwinner Video Capture Device",
		sizeof(csi->media_dev.model));
	media_device_init(&csi->media_dev);

	dev_dbg(csi->dev, "step 0\n");

	csi->v4l2_dev.mdev = &csi->media_dev;
	ret = v4l2_device_register(csi->dev, &csi->v4l2_dev);
	if (ret < 0) {
		dev_err(csi->dev, "V4L2 device registration failed (%d)\n",
			ret);
		goto media_clean;
	}

	dev_dbg(csi->dev, "step 1\n");

	ret = sun6i_video_init(csi, "sun6i-csi");
	if (ret < 0)
		goto v4l2_clean;

	dev_dbg(csi->dev, "step 2\n");

	// Parse DT and build notifier.subdevs (struct v4l2_async_subdev) list
	// that will be used to match and bind/unbind subdevices (sensors) to
	// the csi->v4l2_dev when they are probed and registered by their own
	// drivers. sun6i_csi_parse_subdev_endpoint callback can be used to:
	// - exclude certain subdev endpoints from being watched
	// - parse subdev DT endpoint properties and pass them later to bound
	//   callback via internediate struct sun6i_csi_async_subdev
	ret = v4l2_async_notifier_parse_fwnode_endpoints(
		csi->dev, &csi->notifier,
		sizeof(struct sun6i_csi_async_subdev), sun6i_csi_parse_subdev_endpoint);
	if (ret)
		goto video_clean;

	dev_dbg(csi->dev, "step 3\n");

	csi->notifier.bound = sun6i_csi_notify_bound;
	csi->notifier.unbind = sun6i_csi_notify_unbind;
	csi->notifier.complete = sun6i_csi_notify_complete;
	ret = v4l2_async_notifier_register(&csi->v4l2_dev, &csi->notifier);
	if (ret < 0) {
		dev_err(csi->dev, "Notifier registration failed\n");
		goto notifier_clean;
	}

	dev_dbg(csi->dev, "step 4\n");

	dev_dbg(csi->dev, "driver registered perfectly 0!\n");

	return 0;

notifier_clean:
	v4l2_async_notifier_cleanup(&csi->notifier);
video_clean:
	sun6i_video_cleanup(csi);
v4l2_clean:
	v4l2_device_unregister(&csi->v4l2_dev);
	media_device_unregister(&csi->media_dev);
media_clean:
	media_device_cleanup(&csi->media_dev);
	return ret;
}

int sun6i_csi_cleanup(struct sun6i_csi *csi)
{
	v4l2_async_notifier_unregister(&csi->notifier);
	v4l2_async_notifier_cleanup(&csi->notifier);
	sun6i_video_cleanup(csi);
	v4l2_device_unregister(&csi->v4l2_dev);
	media_device_unregister(&csi->media_dev);
	media_device_cleanup(&csi->media_dev);

	return 0;
}

// }}}
