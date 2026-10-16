// SPDX-License-Identifier: GPL-2.0-only
/* One bounded IMX471 identification sequence, with unconditional power unwind. */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/machine.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <dt-bindings/clock/qcom,sm8150-camcc.h>

#define FRONT_DEVICE "samurai-front-identify"
#define TLMM_DEVICE "3100000.pinctrl"

static int sequence_result = -EINPROGRESS;
static int restore_result;
static unsigned int sensor_id;
static unsigned int ready_samples;
module_param(sequence_result, int, 0444);
module_param(restore_result, int, 0444);
module_param(sensor_id, uint, 0444);
module_param(ready_samples, uint, 0444);

static struct gpiod_lookup_table front_gpios = {
	.dev_id = FRONT_DEVICE,
	.table = {
		GPIO_LOOKUP(TLMM_DEVICE, 12, "reset", GPIO_ACTIVE_LOW),
		GPIO_LOOKUP(TLMM_DEVICE, 29, "select", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP(TLMM_DEVICE, 15, "clock-pin", GPIO_ACTIVE_HIGH),
		{ }
	},
};

static unsigned long active_configs[2], sleep_configs[2];
static struct pinctrl_map front_maps[] = {
	PIN_MAP_MUX_GROUP(FRONT_DEVICE, "active", TLMM_DEVICE, "gpio15", "cam_mclk"),
	PIN_MAP_CONFIGS_GROUP(FRONT_DEVICE, "active", TLMM_DEVICE, "gpio15", active_configs),
	PIN_MAP_MUX_GROUP(FRONT_DEVICE, "sleep", TLMM_DEVICE, "gpio15", "gpio"),
	PIN_MAP_CONFIGS_GROUP(FRONT_DEVICE, "sleep", TLMM_DEVICE, "gpio15", sleep_configs),
};

static const struct regmap_config front_pmic_regmap = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xffff,
};

struct front_resources {
	struct platform_device *pdev;
	struct i2c_adapter *cci, *qup;
	struct i2c_client *primary, *secondary;
	struct regmap *pmic, *ldos;
	struct regulator *vio, *s8;
	struct gpio_desc *reset, *select;
	struct clk *clock;
	struct pinctrl *pins;
	struct pinctrl_state *active, *sleep;
	unsigned long old_rate;
	void __iomem *clock_ctl;
	u32 old_clock_ctl;
	u8 old_vset[2][2], old_enable[7];
	bool snapshot, s8_on, vio_on, clock_on, pinmux_on;
	bool reset_output, select_output, maps, cci_active;
};

static void remember_restore_error(int ret)
{
	if (ret < 0 && !restore_result)
		restore_result = ret;
}

static struct i2c_adapter *adapter_by_path(const char *path)
{
	struct device_node *node;
	struct i2c_adapter *adapter;

	node = of_find_node_by_path(path);
	if (!node)
		return NULL;
	adapter = of_get_i2c_adapter_by_node(node);
	of_node_put(node);
	return adapter;
}

static int hold_clock_domain(struct front_resources *f)
{
	int ret = pm_runtime_resume_and_get(f->cci->dev.parent);

	if (ret < 0)
		return ret;
	f->cci_active = true;
	return 0;
}

static void release_clock_domain(struct front_resources *f)
{
	if (f->cci_active) {
		remember_restore_error(pm_runtime_put_sync(f->cci->dev.parent));
		f->cci_active = false;
	}
}

static int check_input_low(struct gpio_desc *gpio)
{
	int ret = gpiod_get_direction(gpio);

	if (ret != 1)
		return ret < 0 ? ret : -EBUSY;
	ret = gpiod_get_raw_value_cansleep(gpio);
	return ret < 0 ? ret : ret ? -EBUSY : 0;
}

static int save_clock_pin(struct front_resources *f)
{
	static const enum pin_config_param biases[] = {
		PIN_CONFIG_BIAS_DISABLE, PIN_CONFIG_BIAS_PULL_DOWN,
		PIN_CONFIG_BIAS_BUS_HOLD, PIN_CONFIG_BIAS_PULL_UP,
	};
	struct gpio_desc *gpio;
	int ret;

	gpio = gpiod_get(&f->pdev->dev, "clock-pin", GPIOD_ASIS);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);
	ret = check_input_low(gpio);
	if (ret)
		goto put;
	/* SM8150 NORTH GPIO15_CTL: read only, all writes use pinctrl group APIs. */
	f->clock_ctl = ioremap(0x0310f000, sizeof(u32));
	if (!f->clock_ctl) {
		ret = -ENOMEM;
		goto put;
	}
	f->old_clock_ctl = readl(f->clock_ctl);
	if (f->old_clock_ctl & GENMASK(9, 2)) {
		ret = -EBUSY;
		goto put;
	}
	sleep_configs[0] = pinconf_to_config_packed(biases[f->old_clock_ctl & 3], 1);
	sleep_configs[1] = pinconf_to_config_packed(PIN_CONFIG_DRIVE_STRENGTH,
						    ((f->old_clock_ctl >> 6 & 7) + 1) * 2);
	active_configs[0] = pinconf_to_config_packed(PIN_CONFIG_BIAS_DISABLE, 1);
	active_configs[1] = pinconf_to_config_packed(PIN_CONFIG_DRIVE_STRENGTH, 2);
put:
	gpiod_put(gpio);
	return ret;
}

static int snapshot_pmic(struct front_resources *f)
{
	static const u8 expected[] = { 0, 0, 0, 1, 0x51, 0x2c };
	u8 identity[6];
	unsigned int value;
	int ret, i;

	ret = regmap_bulk_read(f->pmic, 0x0100, identity, sizeof(identity));
	if (ret)
		return ret;
	if (memcmp(identity, expected, sizeof(identity)))
		return -ENODEV;
	ret = regmap_read(f->pmic, 0x0644, &value);
	if (ret || value != 1)
		return ret ? ret : -EINVAL;
	ret = regmap_read(f->pmic, 0x0950, &value);
	if (ret || !(value & 1))
		return ret ? ret : -EIO;
	for (i = 0; i < 7; i++) {
		ret = regmap_read(f->ldos, 0x4046 + i * 0x100, &value);
		if (ret || value)
			return ret ? ret : -EBUSY;
		f->old_enable[i] = value;
	}
	ret = regmap_bulk_read(f->ldos, 0x4440, f->old_vset[0], 2);
	if (ret)
		return ret;
	ret = regmap_bulk_read(f->ldos, 0x4140, f->old_vset[1], 2);
	if (!ret)
		f->snapshot = true;
	return ret;
}

static int enable_ldo(struct front_resources *f, unsigned int base, unsigned int mv)
{
	u8 voltage[] = { mv & 0xff, mv >> 8 };
	unsigned int value;
	int ret;

	ret = regmap_bulk_write(f->ldos, base + 0x40, voltage, sizeof(voltage));
	if (ret)
		return ret;
	ret = regmap_write(f->ldos, base + 0x46, 0x80);
	if (ret)
		return ret;
	ret = regmap_read_poll_timeout(f->ldos, base + 8, value, value & 0x80,
				       100, 20000);
	if (!ret)
		ready_samples++;
	return ret;
}

static int identify_once(struct front_resources *f)
{
	u8 address[] = { 0x00, 0x16 }, data[2] = {};
	struct i2c_msg messages[] = {
		{ .addr = 0x10, .len = sizeof(address), .buf = address },
		{ .addr = 0x10, .flags = I2C_M_RD, .len = sizeof(data), .buf = data },
	};
	int ret;

	f->reset_output = true;
	ret = gpiod_direction_output(f->reset, 1);
	if (ret)
		return ret;
	f->select_output = true;
	ret = gpiod_direction_output(f->select, 1);
	if (ret)
		return ret;
	usleep_range(1000, 2000);
	ret = regulator_enable(f->s8);
	if (ret)
		return ret;
	f->s8_on = true;
	ret = regulator_enable(f->vio);
	if (ret)
		return ret;
	f->vio_on = true;
	usleep_range(1000, 2000);
	ret = enable_ldo(f, 0x4400, 2800);
	if (ret)
		return ret;
	usleep_range(1000, 2000);
	ret = enable_ldo(f, 0x4100, 1056);
	if (ret)
		return ret;
	usleep_range(1000, 2000);
	ret = clk_prepare_enable(f->clock);
	if (ret)
		return ret;
	f->clock_on = true;
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(f->reset, 0);
	usleep_range(10000, 12000);
	ret = i2c_transfer(f->cci, messages, ARRAY_SIZE(messages));
	if (ret != ARRAY_SIZE(messages))
		return ret < 0 ? ret : -EIO;
	sensor_id = data[0] << 8 | data[1];
	return sensor_id == 0x0471 ? 0 : -ENODEV;
}

static void power_off(struct front_resources *f)
{
	if (f->select_output)
		gpiod_set_value_cansleep(f->select, 0);
	usleep_range(1000, 2000);
	if (f->reset_output)
		gpiod_set_value_cansleep(f->reset, 1);
	usleep_range(1000, 2000);
	if (f->vio_on)
		remember_restore_error(regulator_disable(f->vio));
	usleep_range(1000, 2000);
	if (f->snapshot) {
		remember_restore_error(regmap_write(f->ldos, 0x4446, f->old_enable[4]));
		usleep_range(1000, 2000);
		remember_restore_error(regmap_write(f->ldos, 0x4146, f->old_enable[1]));
		usleep_range(1000, 2000);
	}
	if (f->clock_on)
		clk_disable_unprepare(f->clock);
	if (f->snapshot) {
		remember_restore_error(regmap_bulk_write(f->ldos, 0x4440, f->old_vset[0], 2));
		remember_restore_error(regmap_bulk_write(f->ldos, 0x4140, f->old_vset[1], 2));
	}
	if (f->s8_on)
		remember_restore_error(regulator_disable(f->s8));
}

static void release_resources(struct front_resources *f)
{
	struct gpio_desc *gpio;

	if (f->pinmux_on)
		remember_restore_error(pinctrl_select_state(f->pins, f->sleep));
	if (f->pins)
		pinctrl_put(f->pins);
	if (f->maps)
		pinctrl_unregister_mappings(front_maps);
	if (f->pdev && f->pinmux_on) {
		gpio = gpiod_get(&f->pdev->dev, "clock-pin", GPIOD_ASIS);
		if (IS_ERR(gpio)) {
			remember_restore_error(PTR_ERR(gpio));
		} else {
			remember_restore_error(gpiod_direction_input(gpio));
			gpiod_put(gpio);
		}
	}
	if (f->clock) {
		if (f->old_rate)
			remember_restore_error(clk_set_rate(f->clock, f->old_rate));
		clk_put(f->clock);
	}
	release_clock_domain(f);
	if (f->clock_ctl) {
		if (readl(f->clock_ctl) != f->old_clock_ctl)
			remember_restore_error(-EIO);
		iounmap(f->clock_ctl);
	}
	if (f->reset) {
		if (f->reset_output)
			remember_restore_error(gpiod_direction_input(f->reset));
		gpiod_put(f->reset);
	}
	if (f->select) {
		if (f->select_output)
			remember_restore_error(gpiod_direction_input(f->select));
		gpiod_put(f->select);
	}
	if (f->ldos)
		regmap_exit(f->ldos);
	if (f->pmic)
		regmap_exit(f->pmic);
	if (f->secondary)
		i2c_unregister_device(f->secondary);
	if (f->primary)
		i2c_unregister_device(f->primary);
	if (f->cci)
		i2c_put_adapter(f->cci);
	if (f->qup)
		i2c_put_adapter(f->qup);
	if (f->vio)
		regulator_put(f->vio);
	if (f->s8)
		regulator_put(f->s8);
	if (f->pdev)
		platform_device_unregister(f->pdev);
	gpiod_remove_lookup_table(&front_gpios);
}

static int get_resources(struct front_resources *f)
{
	struct of_phandle_args clock_spec = { .args_count = 1, .args = { CAM_CC_MCLK2_CLK } };
	int ret;

	f->pdev = platform_device_register_simple(FRONT_DEVICE, -1, NULL, 0);
	if (IS_ERR(f->pdev)) {
		ret = PTR_ERR(f->pdev);
		f->pdev = NULL;
		return ret;
	}
	/* Development-only name lookup; require the unique 1.8V ldo1, never the 0.752V ldo1. */
	f->vio = regulator_get_optional(&f->pdev->dev, "ldo1");
	if (IS_ERR(f->vio)) {
		ret = PTR_ERR(f->vio);
		f->vio = NULL;
		return ret;
	}
	if (regulator_get_voltage(f->vio) != 1800000)
		return -ENODEV;
	f->s8 = regulator_get_optional(&f->pdev->dev, "smps8");
	if (IS_ERR(f->s8)) {
		ret = PTR_ERR(f->s8);
		f->s8 = NULL;
		return ret;
	}
	if (regulator_get_voltage(f->s8) != 1352000)
		return -ENODEV;
	f->reset = gpiod_get(&f->pdev->dev, "reset", GPIOD_ASIS);
	if (IS_ERR(f->reset)) {
		ret = PTR_ERR(f->reset);
		f->reset = NULL;
		return ret;
	}
	f->select = gpiod_get(&f->pdev->dev, "select", GPIOD_ASIS);
	if (IS_ERR(f->select)) {
		ret = PTR_ERR(f->select);
		f->select = NULL;
		return ret;
	}
	ret = check_input_low(f->reset);
	if (!ret)
		ret = check_input_low(f->select);
	if (!ret)
		ret = save_clock_pin(f);
	if (ret)
		return ret;
	ret = pinctrl_register_mappings(front_maps, ARRAY_SIZE(front_maps));
	if (ret)
		return ret;
	f->maps = true;
	f->pins = pinctrl_get(&f->pdev->dev);
	if (IS_ERR(f->pins)) {
		ret = PTR_ERR(f->pins);
		f->pins = NULL;
		return ret;
	}
	f->active = pinctrl_lookup_state(f->pins, "active");
	if (IS_ERR(f->active))
		return PTR_ERR(f->active);
	f->sleep = pinctrl_lookup_state(f->pins, "sleep");
	if (IS_ERR(f->sleep))
		return PTR_ERR(f->sleep);
	f->pinmux_on = true;
	ret = pinctrl_select_state(f->pins, f->active);
	if (ret)
		return ret;
	f->cci = adapter_by_path("/soc@0/cci@ac4b000/i2c-bus@1");
	f->qup = adapter_by_path("/soc@0/geniqup@8c0000/i2c@890000");
	if (!f->cci || !f->qup)
		return -ENODEV;
	/* MCLK belongs to TITAN_TOP_GDSC; keep CCI's domain awake across all clock I/O. */
	ret = hold_clock_domain(f);
	if (ret)
		return ret;
	clock_spec.np = of_find_node_by_path("/soc@0/clock-controller@ad00000");
	if (!clock_spec.np)
		return -ENODEV;
	f->clock = of_clk_get_from_provider(&clock_spec);
	of_node_put(clock_spec.np);
	if (IS_ERR(f->clock)) {
		ret = PTR_ERR(f->clock);
		f->clock = NULL;
		return ret;
	}
	f->old_rate = clk_get_rate(f->clock);
	ret = clk_set_rate(f->clock, 19200000);
	if (ret)
		return ret;
	f->primary = i2c_new_dummy_device(f->qup, 4);
	if (IS_ERR(f->primary)) {
		ret = PTR_ERR(f->primary);
		f->primary = NULL;
		return ret;
	}
	f->secondary = i2c_new_dummy_device(f->qup, 5);
	if (IS_ERR(f->secondary)) {
		ret = PTR_ERR(f->secondary);
		f->secondary = NULL;
		return ret;
	}
	f->pmic = regmap_init_i2c(f->primary, &front_pmic_regmap);
	if (IS_ERR(f->pmic)) {
		ret = PTR_ERR(f->pmic);
		f->pmic = NULL;
		return ret;
	}
	f->ldos = regmap_init_i2c(f->secondary, &front_pmic_regmap);
	if (IS_ERR(f->ldos)) {
		ret = PTR_ERR(f->ldos);
		f->ldos = NULL;
		return ret;
	}
	return snapshot_pmic(f);
}

static int __init samurai_front_identify_init(void)
{
	struct front_resources front = {};

	if (!of_machine_is_compatible("realme,samurai"))
		return -ENODEV;
	gpiod_add_lookup_table(&front_gpios);
	sequence_result = get_resources(&front);
	if (!sequence_result)
		sequence_result = identify_once(&front);
	power_off(&front);
	release_resources(&front);
	pr_info("samurai front identify: result=%d id=%04x ready=%u restore=%d\n",
		sequence_result, sensor_id, ready_samples, restore_result);
	/* Keep the read-only result parameters available after complete power-off. */
	return 0;
}

static void __exit samurai_front_identify_exit(void)
{
}

module_init(samurai_front_identify_init);
module_exit(samurai_front_identify_exit);
MODULE_DESCRIPTION("Bounded realme X2 Pro front camera identification");
MODULE_AUTHOR("Samurai Linux port contributors");
MODULE_LICENSE("GPL");
