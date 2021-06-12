// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 Rockchip Electronics Co. Ltd.
 * Copyright 2019 Toradex AG
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>

#include <drm/drmP.h>
#include <drm/drm_of.h>
#include <drm/drm_atomic.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_panel.h>
#include <drm/drm_mipi_dsi.h>

#include <sound/core.h>
#include <sound/hdmi-codec.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <linux/of_graph.h>

struct lt8912 {
	struct drm_bridge bridge;
	struct drm_connector connector;
	struct drm_display_mode mode;
	struct device *dev;
	struct mipi_dsi_device *dsi;
	struct device_node *host_node;
	u8 num_dsi_lanes;
	u8 channel_id;
	unsigned int irq;
	u8 sink_is_hdmi;
	struct regmap *regmap[3];
	struct gpio_desc *hpd_gpio;
	struct gpio_desc *reset_n;
	struct i2c_adapter *ddc;        /* optional regular DDC I2C bus */
	struct backlight_device *backlight;
	struct platform_device *audio_pdev;
};

static int lt8912_attach_dsi(struct lt8912 *lt);

static inline struct lt8912 *bridge_to_lt8912(struct drm_bridge *b)
{
	return container_of(b, struct lt8912, bridge);
}

static inline struct lt8912 *connector_to_lt8912(struct drm_connector *c)
{
	return container_of(c, struct lt8912, connector);
}

/* LT8912 MIPI to HDMI & LVDS REG setting - 20180115.txt */
static void lt8912_init(struct lt8912 *lt)
{
	const struct drm_display_mode *mode = &lt->mode;
	u32 hactive, hfp, hsync, hbp, vfp, vsync, vbp, htotal, vtotal;
	unsigned int hsync_activehigh, vsync_activehigh;
	unsigned int version[2];

	dev_info(lt->dev, DRM_MODE_FMT "\n", DRM_MODE_ARG(mode));
	/* TODO: lvds output init */

	hactive = mode->hdisplay;
	hfp = mode->hsync_start - mode->hdisplay;
	hsync = mode->hsync_end - mode->hsync_start;
	hsync_activehigh = !!(mode->flags & DRM_MODE_FLAG_NHSYNC);
	hbp = mode->htotal - mode->hsync_end;
	vfp = mode->vsync_start - mode->vdisplay;
	vsync = mode->vsync_end - mode->vsync_start;
	vsync_activehigh = !!(mode->flags & DRM_MODE_FLAG_NVSYNC);
	vbp = mode->vtotal - mode->vsync_end;
	htotal = mode->htotal;
	vtotal = mode->vtotal;

	regmap_read(lt->regmap[0], 0x00, &version[0]);
	regmap_read(lt->regmap[0], 0x01, &version[1]);

	dev_info(lt->dev, "LT8912 ID: %02x, %02x\n",
		 version[0], version[1]);

	/*1280x800*/

        regmap_write(lt->regmap[1],0x10,0x01);
        regmap_write(lt->regmap[1],0x11,0x04);
        regmap_write(lt->regmap[1],0x18,0x14);
        regmap_write(lt->regmap[1],0x19,0x07);
        regmap_write(lt->regmap[1],0x1c,0x00);
        regmap_write(lt->regmap[1],0x1d,0x05);
        regmap_write(lt->regmap[1],0x2f,0x0c);
        regmap_write(lt->regmap[1],0x34,0xa0);
        regmap_write(lt->regmap[1],0x35,0x05);
        regmap_write(lt->regmap[1],0x36,0x37);
        regmap_write(lt->regmap[1],0x37,0x03);
        regmap_write(lt->regmap[1],0x38,0x8);
        regmap_write(lt->regmap[1],0x39,0x0);
        regmap_write(lt->regmap[1],0x3a,0x8);
        regmap_write(lt->regmap[1],0x3b,0x00);
        regmap_write(lt->regmap[1],0x3c,0x46);
        regmap_write(lt->regmap[1],0x3d,0x00);
        regmap_write(lt->regmap[1],0x3e,0x46);
        regmap_write(lt->regmap[1],0x3f,0x00);

	regmap_write(lt->regmap[1],0x4e,0xff);
	regmap_write(lt->regmap[1],0x4f,0x56);
	regmap_write(lt->regmap[1],0x50,0x69);
	regmap_write(lt->regmap[1],0x51,0x80);

	regmap_write(lt->regmap[1],0x1f,0x5e);
	regmap_write(lt->regmap[1],0x20,0x01);
	regmap_write(lt->regmap[1],0x21,0x2c);
	regmap_write(lt->regmap[1],0x22,0x01);
	regmap_write(lt->regmap[1],0x23,0xfa);
	regmap_write(lt->regmap[1],0x24,0x00);
	regmap_write(lt->regmap[1],0x25,0xc8);
	regmap_write(lt->regmap[1],0x26,0x00);
	regmap_write(lt->regmap[1],0x27,0x5e);
	regmap_write(lt->regmap[1],0x28,0x01);
	regmap_write(lt->regmap[1],0x29,0x2c);
	regmap_write(lt->regmap[1],0x2a,0x01);
	regmap_write(lt->regmap[1],0x2b,0xfa);
	regmap_write(lt->regmap[1],0x2c,0x00);
	regmap_write(lt->regmap[1],0x2d,0xc8);
	regmap_write(lt->regmap[1],0x2e,0x00);	
	regmap_write(lt->regmap[1],0x42,0x64);
	regmap_write(lt->regmap[1],0x43,0x00);
	regmap_write(lt->regmap[1],0x44,0x04);
	regmap_write(lt->regmap[1],0x45,0x00);
	regmap_write(lt->regmap[1],0x46,0x59);
	regmap_write(lt->regmap[1],0x47,0x00);
	regmap_write(lt->regmap[1],0x48,0xf2);
	regmap_write(lt->regmap[1],0x49,0x06);
	regmap_write(lt->regmap[1],0x4a,0x00);
	regmap_write(lt->regmap[1],0x4b,0x72);
	regmap_write(lt->regmap[1],0x4c,0x45);
	regmap_write(lt->regmap[1],0x4d,0x00);
	regmap_write(lt->regmap[1],0x52,0x08);
	regmap_write(lt->regmap[1],0x53,0x00);
	regmap_write(lt->regmap[1],0x54,0xb2);
	regmap_write(lt->regmap[1],0x55,0x00);
	regmap_write(lt->regmap[1],0x56,0xe4);
	regmap_write(lt->regmap[1],0x57,0x0d);
	regmap_write(lt->regmap[1],0x58,0x00);
	regmap_write(lt->regmap[1],0x59,0xe4);
	regmap_write(lt->regmap[1],0x5a,0x8a);
	regmap_write(lt->regmap[1],0x5b,0x00);
	regmap_write(lt->regmap[1],0x5c,0x34);
	regmap_write(lt->regmap[1],0x1e,0x4f);
	regmap_write(lt->regmap[1],0x51,0x00);


	regmap_write(lt->regmap[0],0x03,0x7f);
	regmap_write(lt->regmap[0],0x03,0xff);
	regmap_write(lt->regmap[0],0x05,0xfb);
	regmap_write(lt->regmap[0],0x05,0xff);

	//core pll bypass
	regmap_write(lt->regmap[0],0x50, 0x24);//cp=50uA
	regmap_write(lt->regmap[0],0x52, 0x04);//loopdiv=0;use second-order PLL
	regmap_write(lt->regmap[0],0x51, 0x2d);//Pix_clk as reference,second order passive LPF PLL
	regmap_write(lt->regmap[0],0x69, 0x0e);//CP_PRESET_DIV_RATIO
	regmap_write(lt->regmap[0],0x69, 0x8e);
	regmap_write(lt->regmap[0],0x6a, 0x00);
	regmap_write(lt->regmap[0],0x6c, 0x94);//RGD_CP_SOFT_K_EN,RGD_CP_SOFT_K[13:8]
	regmap_write(lt->regmap[0],0x6b, 0x17);

	regmap_write(lt->regmap[0],0x04, 0xfb);//core pll reset
	regmap_write(lt->regmap[0],0x04, 0xff);

	//scaler bypass
	regmap_write(lt->regmap[0],0x7f, 0x00);//disable scaler
	//regmap_write(lt->regmap[0],0xa8, 0x33);//0x13 : JEIDA, 0x33:VESA
	regmap_write(lt->regmap[0],0xa8, 0x13);//0x13 : JEIDA, 0x33:VESA

	regmap_write(lt->regmap[0],0x03, 0x7f);
	regmap_write(lt->regmap[0],0x03, 0xff); //mipi rx reset

	regmap_write(lt->regmap[0],0x05, 0xfb);
	regmap_write(lt->regmap[0],0x05, 0xff); //dds reset 


	regmap_write(lt->regmap[0], 0x02, 0xf7);//lvds pll reset
	regmap_write(lt->regmap[0], 0x02, 0xff);
	regmap_write(lt->regmap[0], 0x03, 0xcb);//scaler module reset
	regmap_write(lt->regmap[0], 0x03, 0xfb);//lvds tx module reset
	regmap_write(lt->regmap[0], 0x03, 0xff);

	regmap_write(lt->regmap[0], 0x44, 0x30); //enbale lvds output
}

static void lt8912_init_hdmi(struct lt8912 *lt)
{

	u8 lanes = lt->dsi->lanes;
	const struct drm_display_mode *mode = &lt->mode;
	u32 hactive, hfp, hsync, hbp, vfp, vsync, vbp, htotal, vtotal;
	unsigned int hsync_activehigh, vsync_activehigh, reg;
	unsigned int version[2];

	dev_info(lt->dev, DRM_MODE_FMT "\n", DRM_MODE_ARG(mode));

	/* TODO: lvds output init */
	hactive = mode->hdisplay;
	hfp = mode->hsync_start - mode->hdisplay;
	hsync = mode->hsync_end - mode->hsync_start;
	hsync_activehigh = !!(mode->flags & DRM_MODE_FLAG_PHSYNC);
	hbp = mode->htotal - mode->hsync_end;
	vfp = mode->vsync_start - mode->vdisplay;
	vsync = mode->vsync_end - mode->vsync_start;
	vsync_activehigh = !!(mode->flags & DRM_MODE_FLAG_PVSYNC);
	vbp = mode->vtotal - mode->vsync_end;
	htotal = mode->htotal;
	vtotal = mode->vtotal;

	regmap_read(lt->regmap[0], 0x00, &version[0]);
	regmap_read(lt->regmap[0], 0x01, &version[1]);

	dev_info(lt->dev, "LT8912 ID: %02x, %02x\n",
		 version[0], version[1]);

	/* DigitalClockEn */
	regmap_write(lt->regmap[0], 0x08, 0xff);
	regmap_write(lt->regmap[0], 0x09, 0xff);
	regmap_write(lt->regmap[0], 0x0a, 0xff);
	regmap_write(lt->regmap[0], 0x0b, 0x7c);
	regmap_write(lt->regmap[0], 0x0c, 0xff);

	/* TxAnalog */
	regmap_write(lt->regmap[0], 0x31, 0xa1);
	regmap_write(lt->regmap[0], 0x32, 0xa1);
	regmap_write(lt->regmap[0], 0x33, 0x03);
	regmap_write(lt->regmap[0], 0x37, 0x00);
	regmap_write(lt->regmap[0], 0x38, 0x22);
	regmap_write(lt->regmap[0], 0x60, 0x82);

	/* CbusAnalog */
	regmap_write(lt->regmap[0], 0x39, 0x45);
	regmap_write(lt->regmap[0], 0x3a, 0x00);
	regmap_write(lt->regmap[0], 0x3b, 0x00);

	/* HDMIPllAnalog */
	regmap_write(lt->regmap[0], 0x44, 0x31);
	regmap_write(lt->regmap[0], 0x55, 0x44);
	regmap_write(lt->regmap[0], 0x57, 0x01);
	regmap_write(lt->regmap[0], 0x5a, 0x02);

	/* MIPIAnalog */
	regmap_write(lt->regmap[0], 0x3e, 0xce);
	regmap_write(lt->regmap[0], 0x3f, 0xd4);
	regmap_write(lt->regmap[0], 0x41, 0x3c);

	/* MipiBasicSet */
	regmap_write(lt->regmap[1], 0x12, 0x04);
	regmap_write(lt->regmap[1], 0x13, lanes % 4);
	regmap_write(lt->regmap[1], 0x14, 0x00);
	regmap_write(lt->regmap[1], 0x15, 0x00);
	regmap_write(lt->regmap[1], 0x1a, 0x03);
	regmap_write(lt->regmap[1], 0x1b, 0x03);

	/* MIPIDig */
	regmap_write(lt->regmap[1], 0x10, 0x01);
	regmap_write(lt->regmap[1], 0x11, 0x0a);
	regmap_write(lt->regmap[1], 0x18, hsync);
	regmap_write(lt->regmap[1], 0x19, vsync);
	regmap_write(lt->regmap[1], 0x1c, hactive % 0x100);
	regmap_write(lt->regmap[1], 0x1d, hactive >> 8);
	regmap_write(lt->regmap[1], 0x2f, 0x0c);
	regmap_write(lt->regmap[1], 0x34, htotal % 0x100);
	regmap_write(lt->regmap[1], 0x35, htotal >> 8);
	regmap_write(lt->regmap[1], 0x36, vtotal % 0x100);
	regmap_write(lt->regmap[1], 0x37, vtotal >> 8);
	regmap_write(lt->regmap[1], 0x38, vbp % 0x100);
	regmap_write(lt->regmap[1], 0x39, vbp >> 8);
	regmap_write(lt->regmap[1], 0x3a, vfp % 0x100);
	regmap_write(lt->regmap[1], 0x3b, vfp >> 8);
	regmap_write(lt->regmap[1], 0x3c, hbp % 0x100);
	regmap_write(lt->regmap[1], 0x3d, hbp >> 8);
	regmap_write(lt->regmap[1], 0x3e, hfp % 0x100);
	regmap_write(lt->regmap[1], 0x3f, hfp >> 8);
	regmap_read(lt->regmap[0], 0xab, &reg);
	reg &= 0xfc;
	reg |= (hsync_activehigh < 1) | vsync_activehigh;
	regmap_write(lt->regmap[0], 0xab, reg);

	/* DDSConfig */
	regmap_write(lt->regmap[1], 0x4e, 0x6a);
	regmap_write(lt->regmap[1], 0x4f, 0xad);
	regmap_write(lt->regmap[1], 0x50, 0xf3);
	regmap_write(lt->regmap[1], 0x51, 0x80);
	regmap_write(lt->regmap[1], 0x1f, 0x5e);
	regmap_write(lt->regmap[1], 0x20, 0x01);
	regmap_write(lt->regmap[1], 0x21, 0x2c);
	regmap_write(lt->regmap[1], 0x22, 0x01);
	regmap_write(lt->regmap[1], 0x23, 0xfa);
	regmap_write(lt->regmap[1], 0x24, 0x00);
	regmap_write(lt->regmap[1], 0x25, 0xc8);
	regmap_write(lt->regmap[1], 0x26, 0x00);
	regmap_write(lt->regmap[1], 0x27, 0x5e);
	regmap_write(lt->regmap[1], 0x28, 0x01);
	regmap_write(lt->regmap[1], 0x29, 0x2c);
	regmap_write(lt->regmap[1], 0x2a, 0x01);
	regmap_write(lt->regmap[1], 0x2b, 0xfa);
	regmap_write(lt->regmap[1], 0x2c, 0x00);
	regmap_write(lt->regmap[1], 0x2d, 0xc8);
	regmap_write(lt->regmap[1], 0x2e, 0x00);
	regmap_write(lt->regmap[1], 0x42, 0x64);
	regmap_write(lt->regmap[1], 0x43, 0x00);
	regmap_write(lt->regmap[1], 0x44, 0x04);
	regmap_write(lt->regmap[1], 0x45, 0x00);
	regmap_write(lt->regmap[1], 0x46, 0x59);
	regmap_write(lt->regmap[1], 0x47, 0x00);
	regmap_write(lt->regmap[1], 0x48, 0xf2);
	regmap_write(lt->regmap[1], 0x49, 0x06);
	regmap_write(lt->regmap[1], 0x4a, 0x00);
	regmap_write(lt->regmap[1], 0x4b, 0x72);
	regmap_write(lt->regmap[1], 0x4c, 0x45);
	regmap_write(lt->regmap[1], 0x4d, 0x00);
	regmap_write(lt->regmap[1], 0x52, 0x08);
	regmap_write(lt->regmap[1], 0x53, 0x00);
	regmap_write(lt->regmap[1], 0x54, 0xb2);
	regmap_write(lt->regmap[1], 0x55, 0x00);
	regmap_write(lt->regmap[1], 0x56, 0xe4);
	regmap_write(lt->regmap[1], 0x57, 0x0d);
	regmap_write(lt->regmap[1], 0x58, 0x00);
	regmap_write(lt->regmap[1], 0x59, 0xe4);
	regmap_write(lt->regmap[1], 0x5a, 0x8a);
	regmap_write(lt->regmap[1], 0x5b, 0x00);
	regmap_write(lt->regmap[1], 0x5c, 0x34);
	regmap_write(lt->regmap[1], 0x1e, 0x4f);
	regmap_write(lt->regmap[1], 0x51, 0x00);
	regmap_write(lt->regmap[0], 0xb2, lt->sink_is_hdmi);

	/* Audio Disable */
	regmap_write(lt->regmap[2], 0x06, 0x08);
	regmap_write(lt->regmap[2], 0x07, 0xf0);
	regmap_write(lt->regmap[2], 0x0f, 0x38);
	regmap_write(lt->regmap[2], 0x34, 0xe2);
	regmap_write(lt->regmap[2], 0x3c, 0x41);

	/* MIPIRxLogicRes */
	regmap_write(lt->regmap[0], 0x03, 0x7f);
	usleep_range(10000, 20000);
	regmap_write(lt->regmap[0], 0x03, 0xff);
	regmap_write(lt->regmap[1], 0x51, 0x80);
	usleep_range(10000, 20000);
	regmap_write(lt->regmap[1], 0x51, 0x00);
}

static void lt8912_wakeup(struct lt8912 *lt)
{
	msleep(30);
	gpiod_direction_output(lt->reset_n, 1);
	msleep(120);
	gpiod_direction_output(lt->reset_n, 0);

	regmap_write(lt->regmap[0],0x02, 0xf7); //lvds pll reset.
	regmap_write(lt->regmap[0],0x08, 0xff);
	/* HPD override */
	regmap_write(lt->regmap[0],0x09, 0xff);
	/* color space */
	regmap_write(lt->regmap[0],0x0a, 0xff);
	/* Fixed */
	regmap_write(lt->regmap[0],0x0b, 0x7c);
	/* HDCP */
	regmap_write(lt->regmap[0],0x0c, 0xff);
/*Tx Analog*/
	/* Fixed */
	regmap_write(lt->regmap[0],0x31, 0xb1);
	/* V1P2 */
	regmap_write(lt->regmap[0],0x32, 0xb1);
	/* Fixed */
	regmap_write(lt->regmap[0],0x33, 0x0e);
	/* Fixed */
	regmap_write(lt->regmap[0],0x37, 0x00);
	/* Fixed */
	regmap_write(lt->regmap[0],0x38, 0x22);
	/* Fixed */
	regmap_write(lt->regmap[0],0x60, 0x82);
/*Cbus Analog*/
	/* Fixed */
	regmap_write(lt->regmap[0],0x39, 0x45);
	regmap_write(lt->regmap[0],0x3a, 0x00); //20180718
	regmap_write(lt->regmap[0],0x3b, 0x00);
/*HDMI Pll Analog*/	
	regmap_write(lt->regmap[0],0x44, 0x31);
	/* Fixed */
	regmap_write(lt->regmap[0],0x55, 0x44);
	/* Fixed */
	regmap_write(lt->regmap[0],0x57, 0x01);
	regmap_write(lt->regmap[0],0x5a, 0x02);
/*MIPI Analog*/
	/* Fixed */
	regmap_write(lt->regmap[0],0x3e, 0xd6);  //0xde.  //0xf6 = pn swap
	regmap_write(lt->regmap[0],0x3f, 0xd4);
	regmap_write(lt->regmap[0],0x41, 0x3c);

	regmap_write(lt->regmap[1],0x12,0x04);
	regmap_write(lt->regmap[1],0x13,0x00);  //0x02 = mipi 2 lane
	regmap_write(lt->regmap[1],0x14,0x00);
	regmap_write(lt->regmap[1],0x15,0x00);
	regmap_write(lt->regmap[1],0x1a,0x03);
	regmap_write(lt->regmap[1],0x1b,0x03);
}

static void lt8912_sleep(struct lt8912 *lt)
{
	regmap_write(lt->regmap[0], 0x41,0x3d); /* MIPI Rx Power Down. */
	regmap_write(lt->regmap[0], 0x08,0x00); /* diable DDS clk. */

	msleep(5);
	gpiod_direction_output(lt->reset_n, 1);
}

static bool lt8912_cable_is_connected(struct lt8912 *lt)
{
	int ret;
	unsigned int reg_val;

	ret = regmap_read(lt->regmap[0], 0xC1, &reg_val);
	if (ret)
		return 0;

	return (reg_val & BIT(7));
}

static enum drm_connector_status
lt8912_connector_detect(struct drm_connector *connector, bool force)
{
	struct lt8912 *lt = connector_to_lt8912(connector);

//	if (lt8912_cable_is_connected(lt))
		return connector_status_connected;
//	else
//		return connector_status_disconnected;
}

static const struct drm_connector_funcs lt8912_connector_funcs = {
	.detect = lt8912_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static irqreturn_t lt8912_hpd_irq_thread(int irq, void *arg)
{
	struct lt8912 *lt = arg;
	struct drm_connector *connector = &lt->connector;

	drm_helper_hpd_irq_event(connector->dev);

	return IRQ_HANDLED;
}

static struct drm_encoder *
lt8912_connector_best_encoder(struct drm_connector *connector)
{
	struct lt8912 *lt = connector_to_lt8912(connector);

	return lt->bridge.encoder;
}

static int lt8912_connector_get_modes(struct drm_connector *connector)
{
	struct lt8912 *lt = connector_to_lt8912(connector);
	struct edid *edid;
	struct display_timings *timings;
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;

	int i, ret, num_modes = 0;
	
	/* Check if optional DDC I2C bus should be used. */
	if (lt->ddc) {
		edid = drm_get_edid(connector, lt->ddc);
		if (edid) {
			drm_connector_update_edid_property(connector,
								edid);
			num_modes = drm_add_edid_modes(connector, edid);
			lt->sink_is_hdmi = !!drm_detect_hdmi_monitor(edid);
			kfree(edid);
		}
		if (num_modes == 0) {
			dev_warn(lt->dev, "failed to get display timings from EDID\n");
			return 0;
		}
	} else { /* if not EDID, use dtb timings */
		timings = of_get_display_timings(lt->dev->of_node);

		if (!timings || timings->num_timings == 0) {
			dev_err(lt->dev, "failed to get display timings from dtb\n");
			return 0;
		}

		for (i = 0; i < timings->num_timings; i++) {
			struct drm_display_mode *mode;
			struct videomode vm;

			if (videomode_from_timings(timings, &vm, i)) {
				continue;
			}

			mode = drm_mode_create(connector->dev);
			drm_display_mode_from_videomode(&vm, mode);
			mode->type = DRM_MODE_TYPE_DRIVER;

			if (timings->native_mode == i)
				mode->type |= DRM_MODE_TYPE_PREFERRED;

			drm_mode_set_name(mode);
			drm_mode_probed_add(connector, mode);
			num_modes++;
		}
		if (num_modes == 0) {
			dev_err(lt->dev, "failed to get display modes from dtb\n");
			return 0;
		}
	}

	connector->display_info.bus_flags = DRM_BUS_FLAG_DE_HIGH |
					    DRM_BUS_FLAG_PIXDATA_NEGEDGE;
	ret = drm_display_info_set_bus_formats(&connector->display_info,
					       &bus_format, 1);

	if (ret)
		return ret;

	return num_modes;
}

static enum drm_mode_status lt8912_connector_mode_valid(struct drm_connector *connector,
			     struct drm_display_mode *mode)
{
	if (mode->clock > 150000)
		return MODE_CLOCK_HIGH;

	if (mode->hdisplay > 1920)
		return MODE_BAD_HVALUE;

	if (mode->vdisplay > 1080)
		return MODE_BAD_VVALUE;

	return MODE_OK;
}

static const struct drm_connector_helper_funcs lt8912_connector_helper_funcs = {
	.get_modes = lt8912_connector_get_modes,
	.best_encoder = lt8912_connector_best_encoder,
	.mode_valid = lt8912_connector_mode_valid,
};

static void lt8912_bridge_post_disable(struct drm_bridge *bridge)
{
}

static void lt8912_bridge_enable(struct drm_bridge *bridge)
{
	struct lt8912 *lt = bridge_to_lt8912(bridge);
	lt8912_init_hdmi(lt);
	lt8912_init(lt);
}

static void lt8912_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct lt8912 *lt = bridge_to_lt8912(bridge);
	lt8912_wakeup(lt);
}

static void lt8912_bridge_mode_set(struct drm_bridge *bridge,
				   const struct drm_display_mode *mode,
				   const struct drm_display_mode *adj)
{
	struct lt8912 *lt = bridge_to_lt8912(bridge);
	drm_mode_copy(&lt->mode, adj);
}

static int lt8912_bridge_attach(struct drm_bridge *bridge)
{
	struct lt8912 *lt = bridge_to_lt8912(bridge);
	struct drm_connector *connector = &lt->connector;
	int ret;

	connector->polled = DRM_CONNECTOR_POLL_CONNECT;
//	connector->polled = DRM_CONNECTOR_POLL_CONNECT | DRM_CONNECTOR_POLL_DISCONNECT;
//	connector->polled = DRM_CONNECTOR_POLL_HPD | DRM_CONNECTOR_POLL_CONNECT;

	ret = drm_connector_init(bridge->dev, connector,
				 &lt8912_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);  //DRM_MODE_CONNECTOR_LVDS
	if (ret) {
		dev_err(lt->dev, "failed to initialize connector\n");
		return ret;
	}

	drm_connector_helper_add(connector, &lt8912_connector_helper_funcs);
	drm_connector_attach_encoder(connector, bridge->encoder);

	ret = lt8912_attach_dsi(lt);

	enable_irq(lt->irq);

	return ret;
}

static const struct drm_bridge_funcs lt8912_bridge_funcs = {
	.attach = lt8912_bridge_attach,
	.mode_set = lt8912_bridge_mode_set,
	.pre_enable = lt8912_bridge_pre_enable,
	.enable = lt8912_bridge_enable,
	.post_disable = lt8912_bridge_post_disable,
};

static const struct regmap_config lt8912_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
};

static int lt8912_i2c_init(struct lt8912 *lt,
			   struct i2c_client *client)
{
	struct i2c_board_info info[] = {
		{ I2C_BOARD_INFO("lt8912p0", 0x48), },
		{ I2C_BOARD_INFO("lt8912p1", 0x49), },
		{ I2C_BOARD_INFO("lt8912p2", 0x4a), }
	};
	struct regmap *regmap;
	unsigned int i;
	int ret;

	if (!lt || !client)
		return -ENODEV;

	for (i = 0; i < ARRAY_SIZE(info); i++) {
		if (i > 0 ) {
			client = i2c_new_dummy(client->adapter, info[i].addr);
			if (!client)
				return -ENODEV;
		}
		regmap = devm_regmap_init_i2c(client, &lt8912_regmap_config);
		if (IS_ERR(regmap)) {
			ret = PTR_ERR(regmap);
			dev_err(lt->dev,
				"Failed to initialize regmap: %d\n", ret);
			return ret;
		}

		lt->regmap[i] = regmap;
	}

	return 0;
}

int lt8912_attach_dsi(struct lt8912 *lt)
{
	struct device *dev = lt->dev;
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	int ret = 0;
	const struct mipi_dsi_device_info info = { .type = "lt8912",
						   .channel = lt->channel_id,
						   .node = NULL,
						 };
	host = of_find_mipi_dsi_host_by_node(lt->host_node);
	if (!host) {
		dev_err(dev, "failed to find dsi host\n");
		return -EPROBE_DEFER;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		dev_err(dev, "failed to create dsi device\n");
		ret = PTR_ERR(dsi);
		goto err_dsi_device;
	}

	lt->dsi = dsi;

	dsi->lanes = lt->num_dsi_lanes;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO;
	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "failed to attach dsi to host\n");
		goto err_dsi_attach;
	}

	return 0;

err_dsi_attach:
	mipi_dsi_device_unregister(dsi);
err_dsi_device:
	return ret;
}

void lt8912_detach_dsi(struct lt8912 *lt)
{
	mipi_dsi_detach(lt->dsi);
	mipi_dsi_device_unregister(lt->dsi);
}

/*-----------------------------------------------------------------------------------------
		HDMI AUDIO IMPLEMENTATION
-------------------------------------------------------------------------------------------*/
int lt8912_hdmi_hw_params(struct device *dev, void *data,
                           struct hdmi_codec_daifmt *fmt,
                           struct hdmi_codec_params *hparms)
{
        struct lt8912 *lt8912 = data;

        switch (hparms->sample_rate) {
        case 32000:
                regmap_write(lt8912->regmap[2], 0x0f, 0x38);
                break;
        case 44100:
                regmap_write(lt8912->regmap[2], 0x0f, 0x00);
                break;
        case 48000:
                regmap_write(lt8912->regmap[2], 0x0f, 0x28);
                break;
        case 88200:
                regmap_write(lt8912->regmap[2], 0x0f, 0x88);
                break;
        case 96000:
                regmap_write(lt8912->regmap[2], 0x0f, 0xa8);
                break;
        case 176400:
                regmap_write(lt8912->regmap[2], 0x0f, 0xc8);
                break;
        case 192000:
                regmap_write(lt8912->regmap[2], 0x0f, 0xe8);
                break;
        default:
                return -EINVAL;
        }

        return 0;
}

static int audio_startup(struct device *dev, void *data)
{	
	return 0;
}


static void audio_shutdown(struct device *dev, void *data)
{
}

static int lt8912_hdmi_i2s_get_dai_id(struct snd_soc_component *component,
                                        struct device_node *endpoint)
{
        struct of_endpoint of_ep;
        int ret;

        ret = of_graph_parse_endpoint(endpoint, &of_ep);
        if (ret < 0)
                return ret;

        /*
         * HDMI sound should be located as reg = <2>
         * Then, it is sound port 0
         */
        if (of_ep.port == 2)
                return 0;

        return -EINVAL;
}

static const struct hdmi_codec_ops lt8912_codec_ops = {
        .hw_params      = lt8912_hdmi_hw_params,
        .audio_shutdown = audio_shutdown,
        .audio_startup  = audio_startup,
        .get_dai_id     = lt8912_hdmi_i2s_get_dai_id,
};

int lt8912_audio_init(struct device *dev, struct lt8912 *lt8912)
{
	static struct hdmi_codec_pdata codec_data = {
        .ops = &lt8912_codec_ops,
        .max_i2s_channels = 2,
        .i2s = 1,
	};

	regmap_write(lt8912->regmap[0], 0xb2, 0x01);
	regmap_write(lt8912->regmap[2], 0x06, 0x08);
        regmap_write(lt8912->regmap[2], 0x07, 0xf0);
        regmap_write(lt8912->regmap[2], 0x0f, 0x38);
        regmap_write(lt8912->regmap[2], 0x34, 0xe2);
        regmap_write(lt8912->regmap[2], 0x3c, 0x41);

	codec_data.data = lt8912;
        lt8912->audio_pdev = platform_device_register_data(dev,
                                        HDMI_CODEC_DRV_NAME,
                                        1,
                                        &codec_data,
                                        sizeof(codec_data));
	return PTR_ERR_OR_ZERO(lt8912->audio_pdev);
}

void lt8912_audio_exit(struct lt8912 *lt8912)
{
        if (lt8912->audio_pdev) {
                platform_device_unregister(lt8912->audio_pdev);
                lt8912->audio_pdev = NULL;
        }
}

/*----------------------------------------------------------------------------------------------- 
		END OF HDMI AUDIO IMPLEMENTATION
-------------------------------------------------------------------------------------------------*/

static int lt8912_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct device *dev = &i2c->dev;
	struct lt8912 *lt;
	struct device_node *ddc_phandle;
	struct device_node *endpoint;
	struct device_node *backlight;
	unsigned int irq_flags;
	int ret = 0;

	lt = devm_kzalloc(dev, sizeof(*lt), GFP_KERNEL);
	if (!lt)
		return -ENOMEM;

	lt->dev = dev;

        backlight = of_parse_phandle(dev->of_node, "backlight", 0);
        if (backlight) {
                lt->backlight = of_find_backlight_by_node(backlight);

                if (!lt->backlight)
                        return -EPROBE_DEFER;
        }

	if (IS_ERR_OR_NULL(lt->backlight)) {
		lt->backlight = NULL;
	}

	backlight_enable(lt->backlight);
	
	/* get optional regular DDC I2C bus */
	ddc_phandle = of_parse_phandle(dev->of_node, "ddc-i2c-bus", 0);
	if (ddc_phandle) {
		lt->ddc = of_get_i2c_adapter_by_node(ddc_phandle);
		of_node_put(ddc_phandle);
		if (!(lt->ddc))
			return -EPROBE_DEFER;
	}

	lt->hpd_gpio = devm_gpiod_get(dev, "hpd", GPIOD_IN);
	if (IS_ERR(lt->hpd_gpio)) {
		ret = PTR_ERR(lt->hpd_gpio);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to get hpd gpio: %d\n", ret);
		goto put_i2c_ddc;
	}

	lt->irq = gpiod_to_irq(lt->hpd_gpio);
	if (lt->irq == -ENXIO) {
		printk("Failed to get hpd irq\n");
		dev_err(dev, "failed to get hpd irq\n");
		ret = -ENODEV;
		goto put_i2c_ddc;
	}
	if (lt->irq < 0) {
		printk("_Failed to get hpd irq\n");
		dev_err(dev, "failed to get hpd irq, %i\n", lt->irq);
		ret = lt->irq;
		goto put_i2c_ddc;
	}
	irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	ret = devm_request_threaded_irq(dev, lt->irq,
					NULL,
					lt8912_hpd_irq_thread,
					irq_flags, "lt8912_hpd", lt);
	if (ret) {
		dev_err(dev, "failed to request irq: %d\n", ret);
		ret = -ENODEV;
		goto put_i2c_ddc;
	}

	disable_irq(lt->irq);

	lt->reset_n = devm_gpiod_get_optional(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(lt->reset_n)) {
		ret = PTR_ERR(lt->reset_n);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to request reset GPIO: %d\n", ret);
		goto put_i2c_ddc;
	}

	ret = lt8912_i2c_init(lt, i2c);
	if (ret)
		goto put_i2c_ddc;

	/* TODO: interrupt handing */

	lt->num_dsi_lanes = 4;
	lt->channel_id = 1;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		ret = -ENODEV;
		goto put_i2c_ddc;
	}

	lt->host_node = of_graph_get_remote_port_parent(endpoint);
	of_node_put(endpoint);
	if (!lt->host_node) {
		ret = -ENODEV;
		goto put_i2c_ddc;
	}

	lt->bridge.funcs = &lt8912_bridge_funcs;
	lt->bridge.of_node = dev->of_node;
	drm_bridge_add(&lt->bridge);

	return lt8912_audio_init(dev,lt);

put_i2c_ddc:
	i2c_put_adapter(lt->ddc);
	return ret;
}

static int lt8912_remove(struct i2c_client *i2c)
{
	struct lt8912 *lt = i2c_get_clientdata(i2c);
	lt8912_sleep(lt);
	mipi_dsi_detach(lt->dsi);
	drm_bridge_remove(&lt->bridge);
	of_node_put(lt->host_node);

	return 0;
}

static const struct i2c_device_id lt8912_i2c_ids[] = {
	{ "lt8912", 0 },
	{ }
};

static const struct of_device_id lt8912_of_match[] = {
	{ .compatible = "lontium,lt8912" },
	{}
};
MODULE_DEVICE_TABLE(of, lt8912_of_match);

static struct mipi_dsi_driver lt8912_driver = {
	.driver.name = "lt8912",
};

static struct i2c_driver lt8912_i2c_driver = {
	.driver = {
		.name = "lt8912",
		.of_match_table = lt8912_of_match,
	},
	.id_table = lt8912_i2c_ids,
	.probe = lt8912_probe,
	.remove = lt8912_remove,
};

static int __init lt8912_i2c_drv_init(void)
{
	mipi_dsi_driver_register(&lt8912_driver);

	return i2c_add_driver(&lt8912_i2c_driver);
}
module_init(lt8912_i2c_drv_init);

static void __exit lt8912_i2c_exit(void)
{
	i2c_del_driver(&lt8912_i2c_driver);

	mipi_dsi_driver_unregister(&lt8912_driver);
}
module_exit(lt8912_i2c_exit);

MODULE_AUTHOR("Wyon Bi <bivvy.bi@rock-chips.com>");
MODULE_DESCRIPTION("Lontium LT8912 MIPI-DSI to LVDS and HDMI/MHL bridge");
MODULE_LICENSE("GPL v2");
