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

static struct i2c_driver pm8008_mfd_driver;

struct pm8008_reset {
	struct device *dev;
	struct gpio_desc *gpio;
	struct gpio_desc *blocker;
};

static bool pm8008_has_reset(struct pm8008_reset *reset)
{
	return reset->gpio;
}

static int pm8008_set_reset(struct pm8008_reset *reset, bool asserted)
{
	if (reset->gpio)
		return gpiod_direction_output(reset->gpio, asserted);
	return 0;
}

/* Keep persistent managed links, including across a supplier rebind. */
static int pm8008_link_bound_supplier(struct device *dev, struct device *supplier,
				      const struct device_driver *expected)
{
	struct device_link *link;
	int ret = 0;

	if (supplier == dev)
		return -EINVAL;
	device_lock(supplier);
	if (!device_is_bound(supplier) ||
	    READ_ONCE(supplier->links.status) != DL_DEV_DRIVER_BOUND)
		ret = -EPROBE_DEFER;
	else if (expected && supplier->driver != expected)
		ret = -EINVAL;
	else {
		link = device_link_add(dev, supplier, DL_FLAG_AUTOPROBE_CONSUMER);
		if (!link)
			ret = -EINVAL;
	}
	device_unlock(supplier);
	return ret;
}

static int pm8008_get_reset(struct i2c_client *client, struct pm8008_reset *reset)
{
	reset->dev = &client->dev;
	reset->gpio = devm_gpiod_get_optional(&client->dev, "reset", GPIOD_ASIS);
	return PTR_ERR_OR_ZERO(reset->gpio);
}

/* Hold a dependent device in reset until address assignment is complete. */
static int pm8008_hold_address_conflict(struct i2c_client *client,
				      struct pm8008_reset *reset)
{
	struct device_node *node, *supplier;
	u32 address, initial;

	if (device_property_read_u32(&client->dev, "qcom,initial-address", &initial))
		return 0;
	for_each_available_child_of_node(client->dev.of_node->parent, node) {
		if (node == client->dev.of_node ||
		    !of_device_is_compatible(node, "qcom,pm8008") ||
		    of_property_read_u32(node, "reg", &address) || address != initial)
			continue;
		supplier = of_parse_phandle(node, "qcom,address-setup-supplier", 0);
		if (supplier != client->dev.of_node) {
			of_node_put(supplier);
			of_node_put(node);
			return -EINVAL;
		}
		of_node_put(supplier);
		reset->blocker = devm_fwnode_gpiod_get_index(&client->dev,
				of_fwnode_handle(node), "reset", 0,
				GPIOD_OUT_HIGH, "pm8008-address-conflict");
		of_node_put(node);
		return PTR_ERR_OR_ZERO(reset->blocker);
	}
	return 0;
}

static struct device_node *pm8008_address_supplier_node(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *node, *p_parent, *q_parent;
	u32 address, initial;
	bool same_parent;

	if (!device_property_present(dev, "qcom,address-setup-supplier"))
		return NULL;
	if (!dev->of_node || client->addr != PM8008_FACTORY_ADDRESS ||
	    (client->flags & I2C_CLIENT_TEN) ||
	    device_property_present(dev, "qcom,initial-address") ||
	    !device_property_present(dev, "reset-gpios") ||
	    of_property_count_u32_elems(dev->of_node, "qcom,address-setup-supplier") != 1)
		return ERR_PTR(-EINVAL);
	node = of_parse_phandle(dev->of_node, "qcom,address-setup-supplier", 0);
	if (!node)
		return ERR_PTR(-EINVAL);
	p_parent = of_get_parent(dev->of_node);
	q_parent = of_get_parent(node);
	same_parent = p_parent && p_parent == q_parent;
	of_node_put(p_parent);
	of_node_put(q_parent);
	if (node == dev->of_node || !same_parent || !of_device_is_available(node) ||
	    !of_device_is_compatible(node, "qcom,pm8008") ||
	    of_property_present(node, "qcom,address-setup-supplier") ||
	    of_property_read_u32(node, "reg", &address) ||
	    of_property_read_u32(node, "qcom,initial-address", &initial) ||
	    address != PM8008_ASSIGNED_ADDRESS || initial != PM8008_FACTORY_ADDRESS) {
		of_node_put(node);
		return ERR_PTR(-EINVAL);
	}
	return node;
}

static int pm8008_link_address_supplier(struct i2c_client *client,
				       struct device_node *node)
{
	struct i2c_client *supplier;
	int ret;

	supplier = i2c_find_device_by_fwnode(of_fwnode_handle(node));
	if (!supplier)
		return -EPROBE_DEFER;
	if (supplier->adapter != client->adapter || (supplier->flags & I2C_CLIENT_TEN))
		ret = -EINVAL;
	else
		ret = pm8008_link_bound_supplier(&client->dev, &supplier->dev,
						&pm8008_mfd_driver.driver);
	put_device(&supplier->dev);
	return ret;
}

static void pm8008_hold_reset(void *data)
{
	struct pm8008_reset *reset = data;
	int ret;

	ret = pm8008_set_reset(reset, true);
	if (ret)
		dev_err(reset->dev, "failed to hold consumer reset: %d\n", ret);
}

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
				     struct pm8008_reset *reset)
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
		if (pm8008_has_reset(reset)) {
			ret = pm8008_set_reset(reset, 0);
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
	    client->addr != PM8008_ASSIGNED_ADDRESS || !pm8008_has_reset(reset) ||
	    (client->flags & I2C_CLIENT_TEN) ||
	    !i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EINVAL;

	ret = pm8008_read_identity(client, client->addr);
	if (!ret) {
		/* Firmware already assigned the address; preserve its LDO state. */
		ret = pm8008_set_reset(reset, 0);
		goto out;
	}
	if (ret != -ENXIO && ret != -EREMOTEIO)
		goto out;

	ret = pm8008_set_reset(reset, 1);
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
	ret = pm8008_set_reset(reset, 0);
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
		reset_ret = pm8008_set_reset(reset, 1);
		if (reset_ret)
			dev_err(dev, "failed to reassert reset: %d\n", reset_ret);
	}
	return ret;
}

static int pm8008_reset_and_address(struct i2c_client *client,
				    struct pm8008_reset **result)
{
	struct device *dev = &client->dev;
	struct device_node *supplier;
	struct pm8008_reset *reset;
	int ret;

	supplier = pm8008_address_supplier_node(client);
	if (IS_ERR(supplier))
		return PTR_ERR(supplier);
	if (supplier) {
		ret = pm8008_link_address_supplier(client, supplier);
		if (ret)
			goto put_node;
	}
	reset = devm_kzalloc(dev, sizeof(*reset), GFP_KERNEL);
	if (!reset) {
		ret = -ENOMEM;
		goto put_node;
	}
	ret = pm8008_hold_address_conflict(client, reset);
	if (ret)
		goto put_node;
	ret = pm8008_get_reset(client, reset);
	if (ret)
		goto put_node;
	/* Children disappear before reset is asserted and its GPIO released. */
	if (pm8008_has_reset(reset)) {
		ret = devm_add_action_or_reset(dev, pm8008_hold_reset, reset);
		if (ret)
			goto put_node;
	}
	mutex_lock(&pm8008_address_lock);
	ret = pm8008_initialize_address(client, reset);
	if (!ret)
		ret = pm8008_read_identity(client, client->addr);
	mutex_unlock(&pm8008_address_lock);
	if (!ret)
		*result = reset;
put_node:
	of_node_put(supplier);
	return ret;
}

struct pm8008_global_enable {
	struct regmap *regmap;
	unsigned int original;
};

static void pm8008_restore_global(void *data)
{
	struct pm8008_global_enable *enable = data;
	int ret;

	ret = regmap_update_bits(enable->regmap, 0x0950, BIT(0), enable->original);
	if (ret)
		dev_err(regmap_get_device(enable->regmap),
			"failed to restore global enable: %d\n", ret);
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
	struct pm8008_reset *reset;
	struct pm8008_global_enable *enable;
	int num_cells;
	char *name;
	int ret;

	ret = pm8008_reset_and_address(client, &reset);
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


	enable = devm_kzalloc(dev, sizeof(*enable), GFP_KERNEL);
	if (!enable)
		return -ENOMEM;
	enable->regmap = regmap;
	ret = regmap_read(regmap, 0x0950, &enable->original);
	if (ret)
		return ret;
	enable->original &= BIT(0);
	ret = devm_add_action_or_reset(dev, pm8008_restore_global, enable);
	if (ret)
		return ret;
	ret = regmap_update_bits(regmap, 0x0950, BIT(0), BIT(0));
	if (ret)
		return ret;

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

	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, cells,
				   num_cells, NULL, 0, domain);
	if (ret)
		return ret;
	/* The persistent consumer link permits the dependent PMIC to probe next. */
	if (reset->blocker) {
		devm_gpiod_put(dev, reset->blocker);
		reset->blocker = NULL;
	}
	return 0;
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
