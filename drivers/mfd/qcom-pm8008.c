// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 */

#include <linux/bitops.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define I2C_INTR_STATUS_BASE		0x0550
#define INT_RT_STS_OFFSET		0x10
#define INT_SET_TYPE_OFFSET		0x11
#define INT_POL_HIGH_OFFSET		0x12
#define INT_POL_LOW_OFFSET		0x13
#define INT_LATCHED_CLR_OFFSET		0x14
#define INT_EN_SET_OFFSET		0x15
#define INT_EN_CLR_OFFSET		0x16
#define INT_LATCHED_STS_OFFSET		0x18

enum {
	PM8008_MISC,
	PM8008_TEMP_ALARM,
	PM8008_GPIO1,
	PM8008_GPIO2,
	PM8008_NUM_PERIPHS,
};

#define PM8008_PERIPH_0_BASE	0x900
#define PM8008_PERIPH_1_BASE	0x2400
#define PM8008_PERIPH_2_BASE	0xc000
#define PM8008_PERIPH_3_BASE	0xc100

#define PM8008_TEMP_ALARM_ADDR	PM8008_PERIPH_1_BASE
#define PM8008_GPIO1_ADDR	PM8008_PERIPH_2_BASE
#define PM8008_GPIO2_ADDR	PM8008_PERIPH_3_BASE

/* PM8008 IRQ numbers */
#define PM8008_IRQ_MISC_UVLO	0
#define PM8008_IRQ_MISC_OVLO	1
#define PM8008_IRQ_MISC_OTST2	2
#define PM8008_IRQ_MISC_OTST3	3
#define PM8008_IRQ_MISC_LDO_OCP	4
#define PM8008_IRQ_TEMP_ALARM	5
#define PM8008_IRQ_GPIO1	6
#define PM8008_IRQ_GPIO2	7

enum {
	SET_TYPE_INDEX,
	POLARITY_HI_INDEX,
	POLARITY_LO_INDEX,
};

static const unsigned int pm8008_config_regs[] = {
	INT_SET_TYPE_OFFSET,
	INT_POL_HIGH_OFFSET,
	INT_POL_LOW_OFFSET,
};

#define _IRQ(_irq, _off, _mask, _types)			\
	[_irq] = {					\
		.reg_offset = (_off),			\
		.mask = (_mask),			\
		.type = {				\
			.type_reg_offset = (_off),	\
			.types_supported = (_types),	\
		},					\
	}

static const struct regmap_irq pm8008_irqs[] = {
	_IRQ(PM8008_IRQ_MISC_UVLO,    PM8008_MISC,	BIT(0), IRQ_TYPE_EDGE_RISING),
	_IRQ(PM8008_IRQ_MISC_OVLO,    PM8008_MISC,	BIT(1), IRQ_TYPE_EDGE_RISING),
	_IRQ(PM8008_IRQ_MISC_OTST2,   PM8008_MISC,	BIT(2), IRQ_TYPE_EDGE_RISING),
	_IRQ(PM8008_IRQ_MISC_OTST3,   PM8008_MISC,	BIT(3), IRQ_TYPE_EDGE_RISING),
	_IRQ(PM8008_IRQ_MISC_LDO_OCP, PM8008_MISC,	BIT(4), IRQ_TYPE_EDGE_RISING),
	_IRQ(PM8008_IRQ_TEMP_ALARM,   PM8008_TEMP_ALARM,BIT(0), IRQ_TYPE_SENSE_MASK),
	_IRQ(PM8008_IRQ_GPIO1,	      PM8008_GPIO1,	BIT(0), IRQ_TYPE_SENSE_MASK),
	_IRQ(PM8008_IRQ_GPIO2,	      PM8008_GPIO2,	BIT(0), IRQ_TYPE_SENSE_MASK),
};

static const unsigned int pm8008_periph_base[] = {
	PM8008_PERIPH_0_BASE,
	PM8008_PERIPH_1_BASE,
	PM8008_PERIPH_2_BASE,
	PM8008_PERIPH_3_BASE,
};

static unsigned int pm8008_get_irq_reg(struct regmap_irq_chip_data *data,
				       unsigned int base, int index)
{
	/* Simple linear addressing for the main status register */
	if (base == I2C_INTR_STATUS_BASE)
		return base + index;

	return pm8008_periph_base[index] + base;
}

static int pm8008_set_type_config(unsigned int **buf, unsigned int type,
				  const struct regmap_irq *irq_data, int idx,
				  void *irq_drv_data)
{
	switch (type) {
	case IRQ_TYPE_EDGE_FALLING:
	case IRQ_TYPE_LEVEL_LOW:
		buf[POLARITY_HI_INDEX][idx] &= ~irq_data->mask;
		buf[POLARITY_LO_INDEX][idx] |= irq_data->mask;
		break;

	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_LEVEL_HIGH:
		buf[POLARITY_HI_INDEX][idx] |= irq_data->mask;
		buf[POLARITY_LO_INDEX][idx] &= ~irq_data->mask;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		buf[POLARITY_HI_INDEX][idx] |= irq_data->mask;
		buf[POLARITY_LO_INDEX][idx] |= irq_data->mask;
		break;

	default:
		return -EINVAL;
	}

	if (type & IRQ_TYPE_EDGE_BOTH)
		buf[SET_TYPE_INDEX][idx] |= irq_data->mask;
	else
		buf[SET_TYPE_INDEX][idx] &= ~irq_data->mask;

	return 0;
}

static const struct regmap_irq_chip pm8008_irq_chip = {
	.name			= "pm8008",
	.main_status		= I2C_INTR_STATUS_BASE,
	.num_main_regs		= 1,
	.irqs			= pm8008_irqs,
	.num_irqs		= ARRAY_SIZE(pm8008_irqs),
	.num_regs		= PM8008_NUM_PERIPHS,
	.status_base		= INT_LATCHED_STS_OFFSET,
	.mask_base		= INT_EN_CLR_OFFSET,
	.unmask_base		= INT_EN_SET_OFFSET,
	.mask_unmask_non_inverted = true,
	.ack_base		= INT_LATCHED_CLR_OFFSET,
	.config_base		= pm8008_config_regs,
	.num_config_bases	= ARRAY_SIZE(pm8008_config_regs),
	.num_config_regs	= PM8008_NUM_PERIPHS,
	.set_type_config	= pm8008_set_type_config,
	.get_irq_reg		= pm8008_get_irq_reg,
};

static const struct regmap_config qcom_mfd_regmap_cfg = {
	.name		= "primary",
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0xffff,
};

static const struct regmap_config pm8008_regmap_cfg_2 = {
	.name		= "secondary",
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0xffff,
};

static const struct resource pm8008_temp_res[] = {
	DEFINE_RES_MEM(PM8008_TEMP_ALARM_ADDR, 0x100),
	DEFINE_RES_IRQ(PM8008_IRQ_TEMP_ALARM),
};

static const struct mfd_cell pm8008_cells[] = {
	MFD_CELL_NAME("pm8008-regulator"),
	MFD_CELL_RES("qpnp-temp-alarm", pm8008_temp_res),
	MFD_CELL_NAME("pm8008-gpio"),
};

static const struct mfd_cell pm8008_regulator_cells[] = {
	MFD_CELL_NAME("pm8008-regulator"),
};

static void devm_irq_domain_fwnode_release(void *data)
{
	struct fwnode_handle *fwnode = data;

	irq_domain_free_fwnode(fwnode);
}

#define PM8008_FACTORY_ADDRESS	0x08
#define PM8008_ASSIGNED_ADDRESS	0x04
#define PM8008_SID_CONFIG		0x0644

/* Serialize reset/address setup across PM8008 devices on a shared bus. */
static DEFINE_MUTEX(pm8008_address_lock);

static int pm8008_read_identity(struct i2c_client *client, u16 address)
{
	u8 reg[] = { 0x01, 0x00 }, data[6];
	struct i2c_msg messages[] = {
		{ .addr = address, .len = sizeof(reg), .buf = reg },
		{ .addr = address, .flags = I2C_M_RD, .len = sizeof(data), .buf = data },
	};
	int ret;

	ret = i2c_transfer(client->adapter, messages, ARRAY_SIZE(messages));
	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(messages))
		return -EIO;
	if (data[4] != 0x51 || data[5] != 0x2c)
		return -ENODEV;
	return 0;
}

static int pm8008_initialize_address(struct i2c_client *client,
				     struct gpio_desc *reset)
{
	struct device *dev = &client->dev;
	u8 write_sid[] = { PM8008_SID_CONFIG >> 8, PM8008_SID_CONFIG & 0xff, 1 };
	struct i2c_msg message = {
		.addr = PM8008_FACTORY_ADDRESS,
		.len = sizeof(write_sid),
		.buf = write_sid,
	};
	bool release_attempted = false;
	u32 initial;
	int ret, write_ret, reset_ret;

	if (!device_property_present(dev, "qcom,initial-address")) {
		/* Keep the pre-existing reset behaviour for ordinary PM8008 nodes. */
		if (reset) {
			ret = gpiod_direction_output(reset, 0);
			if (ret)
				return ret;
		}
		usleep_range(1000, 2000);
		return 0;
	}

	ret = device_property_read_u32(dev, "qcom,initial-address", &initial);
	if (ret)
		return ret;
	if (initial != PM8008_FACTORY_ADDRESS ||
	    client->addr != PM8008_ASSIGNED_ADDRESS || !reset ||
	    (client->flags & I2C_CLIENT_TEN) ||
	    !i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EINVAL;

	ret = pm8008_read_identity(client, client->addr);
	if (!ret) {
		/* Firmware already assigned the address; preserve its LDO state. */
		ret = gpiod_direction_output(reset, 0);
		goto out;
	}
	if (ret != -ENXIO && ret != -EREMOTEIO)
		goto out;

	ret = gpiod_direction_output(reset, 1);
	if (ret)
		goto out;
	usleep_range(10000, 11000);
	ret = pm8008_read_identity(client, initial);
	if (!ret || ret == -ENODEV) {
		dev_err(dev, "initial I2C address is occupied; sibling PMIC must remain reset\n");
		ret = -EBUSY;
		goto out;
	}
	if (ret != -ENXIO && ret != -EREMOTEIO)
		goto out;

	release_attempted = true;
	ret = gpiod_direction_output(reset, 0);
	if (ret)
		goto out;
	usleep_range(10000, 11000);
	ret = pm8008_read_identity(client, initial);
	if (ret)
		goto out;
	write_ret = i2c_transfer(client->adapter, &message, 1);
	usleep_range(1000, 2000);
	ret = pm8008_read_identity(client, client->addr);
	if (!ret)
		dev_info(dev, "assigned PM8008 primary address 0x%02x\n", client->addr);
	else if (write_ret < 0)
		ret = write_ret;
	else if (write_ret != 1)
		ret = -EIO;
out:
	if (ret && release_attempted) {
		/* Do not leave an unassigned PMIC awake at the shared address. */
		reset_ret = gpiod_direction_output(reset, 1);
		if (reset_ret)
			dev_err(dev, "failed to reassert reset: %d\n", reset_ret);
	}
	return ret;
}

static int pm8008_reset_and_address(struct i2c_client *client)
{
	struct gpio_desc *reset;
	int ret;

	reset = devm_gpiod_get_optional(&client->dev, "reset", GPIOD_ASIS);
	if (IS_ERR(reset))
		return PTR_ERR(reset);

	mutex_lock(&pm8008_address_lock);
	ret = pm8008_initialize_address(client, reset);
	mutex_unlock(&pm8008_address_lock);
	return ret;
}

static int pm8008_probe(struct i2c_client *client)
{
	struct regmap_irq_chip_data *irq_data;
	struct device *dev = &client->dev;
	struct regmap *regmap, *regmap2;
	const struct mfd_cell *cells;
	struct fwnode_handle *fwnode;
	struct irq_domain *domain;
	struct i2c_client *dummy;
	int num_cells;
	char *name;
	int ret;

	ret = pm8008_reset_and_address(client);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize PMIC address\n");

	dummy = devm_i2c_new_dummy_device(dev, client->adapter, client->addr + 1);
	if (IS_ERR(dummy)) {
		ret = PTR_ERR(dummy);
		dev_err(dev, "failed to claim second address: %d\n", ret);
		return ret;
	}

	regmap2 = devm_regmap_init_i2c(dummy, &qcom_mfd_regmap_cfg);
	if (IS_ERR(regmap2))
		return PTR_ERR(regmap2);

	ret = regmap_attach_dev(dev, regmap2, &pm8008_regmap_cfg_2);
	if (ret)
		return ret;

	/* Default regmap must be attached last. */
	regmap = devm_regmap_init_i2c(client, &qcom_mfd_regmap_cfg);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);


	if (client->irq > 0) {
		name = devm_kasprintf(dev, GFP_KERNEL, "%pOF-internal", dev->of_node);
		if (!name)
			return -ENOMEM;

		name = strreplace(name, '/', ':');

		fwnode = irq_domain_alloc_named_fwnode(name);
		if (!fwnode)
			return -ENOMEM;

		ret = devm_add_action_or_reset(dev, devm_irq_domain_fwnode_release, fwnode);
		if (ret)
			return ret;

		ret = devm_regmap_add_irq_chip_fwnode(dev, fwnode, regmap,
						      client->irq, IRQF_SHARED, 0,
						      &pm8008_irq_chip, &irq_data);
		if (ret) {
			dev_err(dev, "failed to add IRQ chip: %d\n", ret);
			return ret;
		}

		domain = regmap_irq_get_domain(irq_data);
		cells = pm8008_cells;
		num_cells = ARRAY_SIZE(pm8008_cells);
	} else {
		domain = NULL;
		cells = pm8008_regulator_cells;
		num_cells = ARRAY_SIZE(pm8008_regulator_cells);
	}

	/* Needed by GPIO driver. */
	dev_set_drvdata(dev, domain);

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, cells,
				num_cells, NULL, 0, domain);
}

static const struct of_device_id pm8008_match[] = {
	{ .compatible = "qcom,pm8008", },
	{ },
};
MODULE_DEVICE_TABLE(of, pm8008_match);

static struct i2c_driver pm8008_mfd_driver = {
	.driver = {
		.name = "pm8008",
		.of_match_table = pm8008_match,
	},
	.probe = pm8008_probe,
};
module_i2c_driver(pm8008_mfd_driver);

MODULE_DESCRIPTION("QCOM PM8008 Power Management IC driver");
MODULE_LICENSE("GPL v2");
