// SPDX-License-Identifier: GPL-2.0
/*
 * Raspberry Pi 7 Inch I2C driver
 *
 * Copyright 2021-22 iWave Systems Pvt Ltd
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

struct rpi {
	struct i2c_client *i2c;
	struct regmap *regmap;
};

#define REG_PWM 0x86

static const struct regmap_config attiny_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_PWM,
	.cache_type = REGCACHE_NONE,
};

struct rpi *lt;

static int rpi_remove(struct i2c_client *i2c)
{
        return 0;
}

static int rpi_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{ 
	struct device *dev = &i2c->dev;
	int ret;

        lt = devm_kzalloc(dev, sizeof(*lt), GFP_KERNEL);
        if (!lt)
                return -ENOMEM;

	lt->i2c = i2c;


	lt->regmap = devm_regmap_init_i2c(i2c, &attiny_regmap_config);
	if (IS_ERR(lt->regmap)) {
		ret = PTR_ERR(lt->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	return 0;
};

int i2c_rpi_write(u8 reg, u8 val)
{
	regmap_write(lt->regmap, reg, val);
	return 0;
}

int i2c_rpi_read(u8 reg)
{
	int data;
	regmap_read(lt->regmap, reg, &data);
	return data;
}

static const struct i2c_device_id rpi_i2c_ids[] = {
        { "rpi", 0 },
        { }
};

static const struct of_device_id rpi_of_match[] = {
        { .compatible = "touch-raspberrypi-i2c3" },
        {}
};
MODULE_DEVICE_TABLE(of, rpi_of_match);

static struct i2c_driver rpi_i2c_driver = {
        .driver = {
                .name = "rpi_i2c3",
                .of_match_table = rpi_of_match,
        },
        .id_table = rpi_i2c_ids,
        .probe = rpi_probe,
        .remove = rpi_remove,
};

static int __init rpi_i2c_drv_init(void)
{
        return i2c_add_driver(&rpi_i2c_driver);
}
module_init(rpi_i2c_drv_init);

static void __exit rpi_i2c_exit(void)
{
        i2c_del_driver(&rpi_i2c_driver);

}
module_exit(rpi_i2c_exit);

MODULE_AUTHOR("iWave Systems Pvt Ltd");
MODULE_DESCRIPTION("I2C Driver for Raspberry Pi 7 inch panel");
MODULE_LICENSE("GPL v2");
