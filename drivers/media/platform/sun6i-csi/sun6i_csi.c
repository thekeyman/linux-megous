/*
 * Copyright (c) 2017 Magewell Electronics Co., Ltd. (Nanjing),
 * All rights reserved.
 * Author: Yong Deng <yong.deng@magewell.com>
 *
 * Based on drivers/media/platform/xilinx/xilinx-vipp.c
 * Copyright (C) 2013-2015 Ideas on Board
 * Copyright (C) 2013-2015 Xilinx, Inc.
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
#include <linux/slab.h>
#include <linux/of_graph.h>

#include "sun6i_video.h"

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
	unsigned int pad;
	int ret;

	dev_dbg(csi->dev, "bound subdev %s\n", subdev->name);

	if (subdev->entity.function != MEDIA_ENT_F_CAM_SENSOR)
		return -EINVAL;

	for (pad = 0; pad < subdev->entity.num_pads; pad++) {
		if (subdev->entity.pads[pad].flags & MEDIA_PAD_FL_SOURCE) {
			ret = media_create_pad_link(&subdev->entity, pad, &csi->video.vdev.entity, 0, MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
			if (ret)
				return -EINVAL;

			csi->subdev = subdev;
			return 0;
		}
	}

	//v4l2_set_subdev_hostdata(subdev, csi);

	return 0;
}

static void sun6i_csi_notify_unbind(struct v4l2_async_notifier *notifier,
				       struct v4l2_subdev *subdev,
				       struct v4l2_async_subdev *asd)
{
	struct sun6i_csi *csi = notifier_to_csi(notifier);
	struct sun6i_csi_async_subdev *csi_asd = asd_to_csi_asd(asd);

	if (csi->subdev == subdev) {
		csi->subdev = NULL;
	}

	dev_err(csi->dev, "unbind subdev %s\n", subdev->name);
}

static int sun6i_csi_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct sun6i_csi *csi = notifier_to_csi(notifier);
	int ret;

	dev_dbg(csi->dev, "notify complete, all subdevs bound\n");

	ret = v4l2_device_register_subdev_nodes(&csi->v4l2_dev);
	if (ret < 0) {
		dev_err(csi->dev, "failed to register subdev nodes\n");
		return ret;
	}

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
		//XXX: not entirely cosher (see link_frequencies, dynamically
		//allocated, but we are not using this)
		csi->v4l2_ep = *vep;
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
	csi->media_dev.hw_revision = 0;
	media_device_init(&csi->media_dev);

	csi->v4l2_dev.mdev = &csi->media_dev;
	ret = v4l2_device_register(csi->dev, &csi->v4l2_dev);
	if (ret < 0) {
		dev_err(csi->dev, "V4L2 device registration failed (%d)\n",
			ret);
		goto media_clean;
	}

	ret = sun6i_video_init(&csi->video, csi, "sun6i-csi");
	if (ret < 0)
		goto v4l2_clean;

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

	csi->notifier.bound = sun6i_csi_notify_bound;
	csi->notifier.unbind = sun6i_csi_notify_unbind;
	csi->notifier.complete = sun6i_csi_notify_complete;
	ret = v4l2_async_notifier_register(&csi->v4l2_dev, &csi->notifier);
	if (ret < 0) {
		dev_err(csi->dev, "Notifier registration failed\n");
		goto notifier_clean;
	}

	return 0;

notifier_clean:
	v4l2_async_notifier_cleanup(&csi->notifier);
video_clean:
	sun6i_video_cleanup(&csi->video);
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
	sun6i_video_cleanup(&csi->video);
	v4l2_device_unregister(&csi->v4l2_dev);
	media_device_unregister(&csi->media_dev);
	media_device_cleanup(&csi->media_dev);

	return 0;
}
