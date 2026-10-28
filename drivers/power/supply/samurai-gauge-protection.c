// SPDX-License-Identifier: GPL-2.0-only
/* Fresh, read-only factory-family safety/PF/FET-state provider. */
#include <linux/i2c.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include "gauge-i2c.h"

static bool register_board;
module_param(register_board, bool, 0444);
MODULE_PARM_DESC(register_board, "Create the RMX1931 observation device without changing DT");

struct sm_protection {
	struct i2c_client *gauge;
	struct mutex lock;
	unsigned int id, safety, permanent, operation, manufacturing;
	int last_error;
};

static struct platform_device *sm_board;

static int sm_protection_read(struct sm_protection *p)
{
	static const unsigned int commands[] = { 1, 0x51, 0x53, 0x54, 0x57 };
	unsigned int *values[] = { &p->id, &p->safety, &p->permanent,
		&p->operation, &p->manufacturing };
	unsigned char packet[36];
	int ret = 0;

	i2c_lock_bus(p->gauge->adapter, I2C_LOCK_SEGMENT);
	for (int i = 0; i < ARRAY_SIZE(commands); i++) {
		ret = sm_gauge_mac_locked(p->gauge, commands[i], packet);
		if (ret) break;
		*values[i] = sm_mac_data_size(commands[i]) == 2 ?
			sm_le16(packet + 2) : sm_le32(packet + 2);
	}
	i2c_unlock_bus(p->gauge->adapter, I2C_LOCK_SEGMENT);
	if (!ret)
		ret = sm_gauge_protection_state(p->id, p->safety, p->permanent,
			p->operation, p->manufacturing);
	p->last_error = ret;
	return ret;
}

static int sm_protection_get(struct power_supply *psy,
			     enum power_supply_property property,
			     union power_supply_propval *value)
{
	struct sm_protection *p = power_supply_get_drvdata(psy);
	int ret;

	if (property == POWER_SUPPLY_PROP_MODEL_NAME) {
		value->strval = "samurai-factory-2719-protection-status";
		return 0;
	}
	mutex_lock(&p->lock);
	ret = sm_protection_read(p); /* No cached Good on a failed transfer. */
	if (!ret) {
		if (property == POWER_SUPPLY_PROP_PRESENT) value->intval = 1;
		else if (property == POWER_SUPPLY_PROP_HEALTH) value->intval = POWER_SUPPLY_HEALTH_GOOD;
		else ret = -EINVAL;
	}
	mutex_unlock(&p->lock);
	return ret;
}

static ssize_t protection_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sm_protection *p = dev_get_drvdata(dev);
	ssize_t length;

	mutex_lock(&p->lock);
	sm_protection_read(p);
	length = sysfs_emit(buf, "id=0x%x safety=0x%x pf=0x%x operation=0x%x "
		"manufacturing=0x%x pf_enabled=%d error=%d\n", p->id, p->safety,
		p->permanent, p->operation, p->manufacturing,
		!!(p->manufacturing & 0x40), p->last_error);
	mutex_unlock(&p->lock);
	return length;
}
static DEVICE_ATTR_RO(protection_state);
static struct attribute *sm_protection_attrs[] = { &dev_attr_protection_state.attr, NULL };
static const struct attribute_group sm_protection_group = { .attrs = sm_protection_attrs };
static const enum power_supply_property sm_protection_properties[] = {
	POWER_SUPPLY_PROP_PRESENT, POWER_SUPPLY_PROP_HEALTH, POWER_SUPPLY_PROP_MODEL_NAME,
};
static const struct power_supply_desc sm_protection_desc = {
	.name = "samurai-gauge-protection", .type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = sm_protection_properties,
	.num_properties = ARRAY_SIZE(sm_protection_properties), .get_property = sm_protection_get,
	.no_thermal = true,
};

static void sm_protection_put(void *data) { put_device(data); }

static int sm_protection_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node;
	struct power_supply_config config = { 0 };
	struct power_supply *psy;
	struct sm_protection *p;
	int ret;

	if (!of_machine_is_compatible("realme,samurai") ||
	    IS_ENABLED(CONFIG_BATTERY_BQ27XXX_DT_UPDATES_NVM)) return -ENODEV;
	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p) return -ENOMEM;
	node = dev->of_node ? of_parse_phandle(dev->of_node, "battery-gauge", 0) : NULL;
	if (!node && register_board)
		node = of_find_node_by_path("/soc@0/geniqup@cc0000/i2c@c94000/fuel-gauge@55");
	if (!node) return -EINVAL;
	p->gauge = of_find_i2c_device_by_node(node);
	of_node_put(node);
	if (!p->gauge) return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(dev, sm_protection_put, &p->gauge->dev);
	if (ret) return ret;
	if (p->gauge->addr != 0x55 || !i2c_check_functionality(p->gauge->adapter,
		I2C_FUNC_I2C | I2C_FUNC_SMBUS_WORD_DATA)) return -EOPNOTSUPP;
	/* A device reference protects lifetime, not system-sleep ordering. Fresh
	 * MAC reads must wait for the gauge and its I2C parent to resume first.
	 */
	if (!device_link_add(dev, &p->gauge->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -ENOMEM;
	mutex_init(&p->lock);
	platform_set_drvdata(pdev, p);
	ret = sm_protection_read(p);
	if (ret) return dev_err_probe(dev, ret, "factory gauge protection protocol/state\n");
	config.drv_data = p;
	config.fwnode = dev_fwnode(dev);
	psy = devm_power_supply_register(dev, &sm_protection_desc, &config);
	if (IS_ERR(psy)) return PTR_ERR(psy);
	return devm_device_add_group(dev, &sm_protection_group);
}

static const struct of_device_id sm_protection_match[] = {
	{ .compatible = "realme,samurai-gauge-protection" }, { }
};
MODULE_DEVICE_TABLE(of, sm_protection_match);
static struct platform_driver sm_protection_driver = {
	.probe = sm_protection_probe,
	.driver = { .name = "samurai-gauge-protection", .of_match_table = sm_protection_match },
};
static int __init sm_protection_init(void)
{
	int ret = platform_driver_register(&sm_protection_driver);
	if (ret || !register_board) return ret;
	sm_board = platform_device_register_simple("samurai-gauge-protection", -1, NULL, 0);
	if (IS_ERR(sm_board)) {
		ret = PTR_ERR(sm_board);
		platform_driver_unregister(&sm_protection_driver);
	}
	return ret;
}
static void __exit sm_protection_exit(void)
{
	if (register_board && !IS_ERR_OR_NULL(sm_board)) platform_device_unregister(sm_board);
	platform_driver_unregister(&sm_protection_driver);
}
module_init(sm_protection_init);
module_exit(sm_protection_exit);
MODULE_DESCRIPTION("RMX1931 factory-family gauge protection status, no protection writes");
MODULE_LICENSE("GPL");
