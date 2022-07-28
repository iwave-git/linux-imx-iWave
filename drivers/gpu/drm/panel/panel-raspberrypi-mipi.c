// SPDX-License-Identifier: GPL-2.0
/*
 * Raspberry Pi 7 Inch panel driver
 *
 * Copyright 2021-22 iWave Systems Pvt Ltd
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

/* Panel specific color-format bits */
#define COL_FMT_16BPP 0x55
#define COL_FMT_18BPP 0x66
#define COL_FMT_24BPP 0x77

int i2c_rpi_write(u8, u8);
int i2c_rpi_read(u8);

/* I2C registers of the Atmel microcontroller. */
enum REG_ADDR {
       REG_ID = 0x80,
       REG_PORTA, // BIT(2) for horizontal flip, BIT(3) for vertical flip
       REG_PORTB,
       REG_PORTC,
       REG_PORTD,
       REG_POWERON,
       REG_PWM,
       REG_DDRA,
       REG_DDRB,
       REG_DDRC,
       REG_DDRD,
       REG_TEST,
       REG_WR_ADDRL,
       REG_WR_ADDRH,
       REG_READH,
       REG_READL,
       REG_WRITEH,
       REG_WRITEL,
       REG_ID2,
};

/* DSI D-PHY Layer Registers */
#define D0W_DPHYCONTTX 0x0004
#define CLW_DPHYCONTRX 0x0020
#define D0W_DPHYCONTRX 0x0024
#define D1W_DPHYCONTRX 0x0028
#define COM_DPHYCONTRX 0x0038
#define CLW_CNTRL 0x0040
#define D0W_CNTRL 0x0044
#define D1W_CNTRL 0x0048
#define DFTMODE_CNTRL 0x0054

/* DSI PPI Layer Registers */
#define PPI_STARTPPI 0x0104
#define PPI_BUSYPPI 0x0108
#define PPI_LINEINITCNT 0x0110
#define PPI_LPTXTIMECNT 0x0114
#define PPI_CLS_ATMR 0x0140
#define PPI_D0S_ATMR 0x0144
#define PPI_D1S_ATMR 0x0148
#define PPI_D0S_CLRSIPOCOUNT 0x0164
#define PPI_D1S_CLRSIPOCOUNT 0x0168
#define CLS_PRE 0x0180
#define D0S_PRE 0x0184
#define D1S_PRE 0x0188
#define CLS_PREP 0x01A0
#define D0S_PREP 0x01A4
#define D1S_PREP 0x01A8
#define CLS_ZERO 0x01C0
#define D0S_ZERO 0x01C4
#define D1S_ZERO 0x01C8
#define PPI_CLRFLG 0x01E0
#define PPI_CLRSIPO 0x01E4
#define HSTIMEOUT 0x01F0
#define HSTIMEOUTENABLE 0x01F4

/* DSI Protocol Layer Registers */
#define DSI_STARTDSI 0x0204
#define DSI_BUSYDSI 0x0208
#define DSI_LANEENABLE 0x0210
#define DSI_LANEENABLE_CLOCK BIT(0)
#define DSI_LANEENABLE_D0 BIT(1)
#define DSI_LANEENABLE_D1 BIT(2)

#define DSI_LANESTATUS0 0x0214
#define DSI_LANESTATUS1 0x0218
#define DSI_INTSTATUS 0x0220
#define DSI_INTMASK 0x0224
#define DSI_INTCLR 0x0228
#define DSI_LPTXTO 0x0230
#define DSI_MODE 0x0260
#define DSI_PAYLOAD0 0x0268
#define DSI_PAYLOAD1 0x026C
#define DSI_SHORTPKTDAT 0x0270
#define DSI_SHORTPKTREQ 0x0274
#define DSI_BTASTA 0x0278
#define DSI_BTACLR 0x027C

/* DSI General Registers */
#define DSIERRCNT 0x0300
#define DSISIGMOD 0x0304

/* DSI Application Layer Registers */
#define APLCTRL 0x0400
#define APLSTAT 0x0404
#define APLERR 0x0408
#define PWRMOD 0x040C
#define RDPKTLN 0x0410
#define PXLFMT 0x0414
#define MEMWRCMD 0x0418

/* LCDC/DPI Host Registers */
#define LCDCTRL 0x0420
#define HSR 0x0424
#define HDISPR 0x0428
#define VSR 0x042C
#define VDISPR 0x0430
#define VFUEN 0x0434

/* DBI-B Host Registers */
#define DBIBCTRL 0x0440

/* SPI Master Registers */
#define SPICMR 0x0450
#define SPITCR 0x0454

/* System Controller Registers */
#define SYSSTAT 0x0460
#define SYSCTRL 0x0464
#define SYSPLL1 0x0468
#define SYSPLL2 0x046C
#define SYSPLL3 0x0470
#define SYSPMCTRL 0x047C

/* GPIO Registers */
#define GPIOC 0x0480
#define GPIOO 0x0484
#define GPIOI 0x0488

/* I2C Registers */
#define I2CCLKCTRL 0x0490

static const u32 rad_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

static const u32 rad_bus_flags =
	DRM_BUS_FLAG_DE_LOW | DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE;

struct rad_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;

	struct gpio_desc *reset;
	struct backlight_device *backlight;

	struct regulator_bulk_data *supplies;
	unsigned int num_supplies;

	bool prepared;
	bool enabled;

	const struct rad_platform_data *pdata;
	struct i2c_client *bridge_i2c;
};

struct rad_platform_data {
        int (*enable)(struct rad_panel *panel);
};

static const struct drm_display_mode default_mode = {
	/* Modeline comes from the Raspberry Pi firmware, with HFP=1
      ┆* plugged in and clock re-computed from that.
      ┆*/

#define PIXEL_CLOCK ((2000000000 / 3) / 24)
#define VREFRESH    60
#define VTOTAL      (480 + 7 + 2 + 21)
#define HACT        800
#define HSW         5
#define HBP         46
#define HFP         ((PIXEL_CLOCK / (VTOTAL * VREFRESH)) - (HACT + HSW + HBP))

                .clock = 33300000 / 1000,
                .hdisplay = HACT,
                .hsync_start = HACT + HFP,
                .hsync_end = HACT + HFP + HSW,
                .htotal = HACT + HFP + HSW + HBP,
                .vdisplay = 480,
                .vsync_start = 480 + 7,
                .vsync_end = 480 + 7 + 10,
                .vtotal = VTOTAL,
};

static inline struct rad_panel *to_rad_panel(struct drm_panel *panel)
{
	return container_of(panel, struct rad_panel, panel);
}

static int rad_panel_prepare(struct drm_panel *panel)
{
	struct rad_panel *rad = to_rad_panel(panel);
	int ret;

	if (rad->prepared)
		return 0;

	ret = regulator_bulk_enable(rad->num_supplies, rad->supplies);
	if (ret)
		return ret;

	/* At lest 10ms needed between power-on and reset-out as RM specifies */
	usleep_range(10000, 12000);

	if (rad->reset) {
		gpiod_set_value_cansleep(rad->reset, 0);
		/*
		 * 50ms delay after reset-out, as per manufacturer initalization
		 * sequence.
		 */
		msleep(50);
	}

	rad->prepared = true;

	return 0;
}

static int rad_panel_unprepare(struct drm_panel *panel)
{
	struct rad_panel *rad = to_rad_panel(panel);
	int ret;

	if (!rad->prepared)
		return 0;

	/*
	 * Right after asserting the reset, we need to release it, so that the
	 * touch driver can have an active connection with the touch controller
	 * even after the display is turned off.
	 */
	if (rad->reset) {
		gpiod_set_value_cansleep(rad->reset, 1);
		usleep_range(15000, 17000);
		gpiod_set_value_cansleep(rad->reset, 0);
	}

	ret = regulator_bulk_disable(rad->num_supplies, rad->supplies);
	if (ret)
		return ret;

	rad->prepared = false;

	return 0;
}

static int rpi_touchscreen_write_eic(struct rad_panel *ts, u16 reg,
				     u32 val)
{
	u8 msg[] = {
		reg, reg >> 8, val, val >> 8, val >> 16, val >> 24,
	};

	mipi_dsi_generic_write(ts->dsi, msg, sizeof(msg));
	
	return 0;
}

static int rpi_enable(struct rad_panel *panel)
{
	struct mipi_dsi_device *dsi = panel->dsi;
	int i;

	if (panel->enabled)
		return 0;

        /* Wait for nPWRDWN to go low to indicate poweron is done. */
        for (i = 0; i < 100; i++) {
                if (i2c_rpi_read(REG_PORTB) & 1)
                        break;
        }

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	
	rpi_touchscreen_write_eic(panel, DSI_LANEENABLE,
				  DSI_LANEENABLE_CLOCK | DSI_LANEENABLE_D0);
	rpi_touchscreen_write_eic(panel, PPI_D0S_CLRSIPOCOUNT, 0x05);
	rpi_touchscreen_write_eic(panel, PPI_D1S_CLRSIPOCOUNT, 0x05);
	rpi_touchscreen_write_eic(panel, PPI_D0S_ATMR, 0x00);
	rpi_touchscreen_write_eic(panel, PPI_D1S_ATMR, 0x00);
	rpi_touchscreen_write_eic(panel, PPI_LPTXTIMECNT, 0x03);

	rpi_touchscreen_write_eic(panel, SPICMR, 0x00);
	rpi_touchscreen_write_eic(panel, LCDCTRL, 0x00100150);
	rpi_touchscreen_write_eic(panel, SYSCTRL, 0x040f);
	msleep(100);

	rpi_touchscreen_write_eic(panel, PPI_STARTPPI, 0x01);
	rpi_touchscreen_write_eic(panel, DSI_STARTDSI, 0x01);
	msleep(100);
	i2c_rpi_write(REG_PWM, 255);

	panel->enabled = true;

	return 0;
}

static int rad_panel_enable(struct drm_panel *panel)
{
	struct rad_panel *rad = to_rad_panel(panel);

	return rad->pdata->enable(rad);
}

static int rad_panel_disable(struct drm_panel *panel)
{
	struct rad_panel *rad = to_rad_panel(panel);
	struct mipi_dsi_device *dsi = rad->dsi;

	if (!rad->enabled)
		return 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	i2c_rpi_write(REG_PWM, 0);
	rad->enabled = false;
	udelay(1);

	return 0;
}

static int rad_panel_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	connector->display_info.bpc = 8;
	connector->display_info.bus_flags = rad_bus_flags;

	drm_display_info_set_bus_formats(&connector->display_info,
					 rad_bus_formats,
					 ARRAY_SIZE(rad_bus_formats));
	return 1;
}

static int rad_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct rad_panel *rad = mipi_dsi_get_drvdata(dsi);
	int ret = 0;

	if (!rad->prepared)
		return 0;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	
	i2c_rpi_write(REG_PWM, bl->props.brightness);

	ret = mipi_dsi_dcs_set_display_brightness(dsi, bl->props.brightness);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct backlight_ops rad_bl_ops = {
	.update_status = rad_bl_update_status,
};

static const struct drm_panel_funcs rad_panel_funcs = {
	.prepare = rad_panel_prepare,
	.unprepare = rad_panel_unprepare,
	.enable = rad_panel_enable,
	.disable = rad_panel_disable,
	.get_modes = rad_panel_get_modes,
};

static const char *const rad_supply_names[] = {
	"v3p3",
	"v1p8",
};

static int rad_init_regulators(struct rad_panel *rad)
{
	struct device *dev = &rad->dsi->dev;
	int i;

	rad->num_supplies = ARRAY_SIZE(rad_supply_names);
	rad->supplies = devm_kcalloc(dev, rad->num_supplies,
				     sizeof(*rad->supplies), GFP_KERNEL);
	if (!rad->supplies)
		return -ENOMEM;

	for (i = 0; i < rad->num_supplies; i++)
		rad->supplies[i].supply = rad_supply_names[i];

	return devm_regulator_bulk_get(dev, rad->num_supplies, rad->supplies);
};

static const struct rad_platform_data rpi_panel = {
	.enable = &rpi_enable,
};

static const struct of_device_id rad_of_match[] = {
	{ .compatible = "rpi,panelv1", .data = &rpi_panel },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rad_of_match);

static int rad_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct of_device_id *of_id = of_match_device(rad_of_match, dev);
	struct device_node *np = dev->of_node;
	struct rad_panel *panel;
	struct backlight_properties bl_props;
	int ret;
	u32 video_mode;

	if (!of_id || !of_id->data)
		return -ENODEV;

	panel = devm_kzalloc(&dsi->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, panel);

	panel->dsi = dsi;
	panel->pdata = of_id->data;

	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags =  MIPI_DSI_MODE_VSYNC_FLUSH | MIPI_DSI_MODE_VIDEO |
			    MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			     MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_NO_EOT_PACKET;

	ret = of_property_read_u32(np, "video-mode", &video_mode);
	if (!ret) {
		switch (video_mode) {
		case 0:
			/* burst mode */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
			break;
		case 1:
			/* non-burst mode with sync event */
			break;
		case 2:
			/* non-burst mode with sync pulse */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
			break;
		default:
			dev_warn(dev, "invalid video mode %d\n", video_mode);
			break;
		}
	}

	ret = of_property_read_u32(np, "dsi-lanes", &dsi->lanes);
	if (ret) {
		dev_err(dev, "Failed to get dsi-lanes property (%d)\n", ret);
		return ret;
	}
	
	i2c_rpi_write(REG_POWERON, 1);

	panel->reset = devm_gpiod_get_optional(
		dev, "reset", GPIOD_OUT_LOW | GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if (IS_ERR(panel->reset)) {
		ret = PTR_ERR(panel->reset);
		dev_err(dev, "Failed to get reset gpio (%d)\n", ret);
		return ret;
	}
	gpiod_set_value_cansleep(panel->reset, 1);

	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = 255;
	bl_props.max_brightness = 255;

	panel->backlight = devm_backlight_device_register(
		dev, dev_name(dev), dev, dsi, &rad_bl_ops, &bl_props);
	if (IS_ERR(panel->backlight)) {
		ret = PTR_ERR(panel->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}

	ret = rad_init_regulators(panel);
	if (ret)
		return ret;

	drm_panel_init(&panel->panel, dev, &rad_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	dev_set_drvdata(dev, panel);
	drm_panel_add(&panel->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret)
		drm_panel_remove(&panel->panel);

	return ret;
}

static int rad_panel_remove(struct mipi_dsi_device *dsi)
{
	struct rad_panel *rad = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	i2c_rpi_write(REG_POWERON, 0);

	ret = mipi_dsi_detach(dsi);
	if (ret)
		dev_err(dev, "Failed to detach from host (%d)\n", ret);

	drm_panel_remove(&rad->panel);

	return 0;
}

static void rad_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct rad_panel *rad = mipi_dsi_get_drvdata(dsi);

	rad_panel_disable(&rad->panel);
	rad_panel_unprepare(&rad->panel);
}

static struct mipi_dsi_driver rad_panel_driver = {
	.driver = {
		.name = "panel-raspberrypi-mipi",
		.of_match_table = rad_of_match,
	},
	.probe = rad_panel_probe,
	.remove = rad_panel_remove,
	.shutdown = rad_panel_shutdown,
};
module_mipi_dsi_driver(rad_panel_driver);

MODULE_AUTHOR("iWave Systems Pvt Ltd");
MODULE_DESCRIPTION("DRM Driver for Raspberry Pi 7 inch panel");
MODULE_LICENSE("GPL v2");
