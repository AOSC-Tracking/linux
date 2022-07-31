// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig <alyssa@rosenzweig.io> */

#ifndef __APPLE_DCP_H__
#define __APPLE_DCP_H__

#include <drm/drm_atomic.h>
#include "parser.h"

struct apple_crtc {
	struct drm_crtc base;
	struct drm_pending_vblank_event *event;
	bool vsync_disabled;

	/* Reference to the DCP device owning this CRTC */
	struct platform_device *dcp;
};

#define to_apple_crtc(x) container_of(x, struct apple_crtc, base)

void dcp_hotplug(struct work_struct *work);

struct apple_connector {
	struct drm_connector base;
	bool connected;

	struct platform_device *dcp;

	/* Workqueue for sending hotplug events to the associated device */
	struct work_struct hotplug_wq;
};

#define to_apple_connector(x) container_of(x, struct apple_connector, base)

void dcp_poweroff(struct platform_device *pdev);
void dcp_poweron(struct platform_device *pdev);
void dcp_link(struct platform_device *pdev, struct apple_crtc *apple,
	      struct apple_connector *connector);
void dcp_flush(struct drm_crtc *crtc, struct drm_atomic_state *state);
bool dcp_is_initialized(struct platform_device *pdev);
void apple_crtc_vblank(struct apple_crtc *apple);
int dcp_get_modes(struct drm_connector *connector);
int dcp_mode_valid(struct drm_connector *connector,
		   struct drm_display_mode *mode);

#endif
