#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/platform_data/leds-pca9956.h>

/* PCA9956 Register Table */
/* LED mode registers (read/write) */
#define PCA9956_MODE_REGISTER_1		0x00
#define PCA9956_MODE_REGISTER_2		0x01

/* If auto-increment is enabled, the MSb(bit) needs to be set to 1 in register
 * address according to datasheet.
 * In this driver, auto-increment is enabled in pca9956_led_init function.
 */
#define AUTO_INCREMENT_ADDRESS_MASK	0x80

#define PCA9955_0x43_DEFAULT_VALUE	0xE0
#define NUM_OF_RETRIES			5

struct pca9956_led {
	struct i2c_client *client;
	const struct pca9956_platform_data *pdata;
	struct attribute **all_attrs;
	struct device_attribute *iref_pwm_attrs;
	struct attribute_group iref_pwm_attr_group;
	const struct pca9956_registers *reg;
};

enum chip_model_t {
	PCA9956B,
	PCA9955B,
};

static const struct pca9956_registers {
	const char* model;
	const u8 reg_global_dimming;
	const u8 reg_pwm_base;
	const u8 reg_iref_base;
	const u8 reg_irefall;
	const int ledout_count;
	const int led_channels;
} kModelRegisterMap[] = {
	{"pca_9956b", 0x08, 0x0A, 0x22, 0x40, 6, 24},
	{"pca_9955b", 0x06, 0x08, 0x18, 0x45, 4, 16},
};

static const struct pca9956_registers* pca9956_dev_to_register (struct device *dev) {
	struct i2c_client *client = to_i2c_client(dev);
	struct pca9956_led *data = i2c_get_clientdata(client);
	return data->reg;
}

static int pca9956_led_read_byte(struct i2c_client *client, u8 reg, u8 *buf)
{
	s32 ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		return ret;

	*buf = ret;
	return 0;
}

static int pca9956_led_write_byte(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to write byte data, ret=%d\n",
			__func__, ret);
	}
	return ret;
}

static int pca9956_led_read_block_data(struct i2c_client *client, u8 reg,
					u8 len, void *val)
{
	int ret = i2c_smbus_read_i2c_block_data(client,
					reg | AUTO_INCREMENT_ADDRESS_MASK,
					len, val);

	if (ret == len)
		return 0;

	dev_err(&client->dev, "%s: failed on block read.\n", __func__);
	return -EIO;
}

static int pca9956_led_write_block_data(struct i2c_client *client, u8 reg,
					u8 len, const void *val)
{
	int ret;
	int retry = 0;
	do {
		ret = i2c_smbus_write_i2c_block_data(client,
			reg | AUTO_INCREMENT_ADDRESS_MASK,
			len, val);
		if (ret < 0) {
			dev_err(&client->dev, "%s: failed on block write. "
				"ret=%d, retry=%d\n",
			__func__, ret, retry);
		}
		++retry;

	} while (ret == -ERESTARTSYS && retry < NUM_OF_RETRIES);

	return ret;
}

static int pca9956_led_init(struct pca9956_led *data)
{
	struct i2c_client *client = data->client;
	int ret;

	/* Find out which model it is */
	u8 read_buf;
	ret = pca9956_led_read_byte(client, 0x43, &read_buf);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to read from register 0x43\n",
			__func__);
	        return ret;
	}
        if (read_buf == PCA9955_0x43_DEFAULT_VALUE) {
	        data->reg = &kModelRegisterMap[PCA9955B];
	} else {
		data->reg = &kModelRegisterMap[PCA9956B];
	}
	dev_info(&client->dev, "%s: driver registered as %s\n",
		 __func__, data->reg->model);

	/* Initialize NXP LED driver */
	/* Set all IREF values */
	ret = pca9956_led_write_byte(client, data->reg->reg_irefall, 0xFF);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to set all IREF values.\n",
			__func__);
		return ret;
	}

	/* Set Mode1 register to enable I2C auto-increment for 00h to 3Eh.
	 */
	ret = pca9956_led_write_byte(client, PCA9956_MODE_REGISTER_1, 0x40);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to set Mode 1 registers.\n",
			__func__);
		return ret;
	}

	int ledout_count = data->reg->ledout_count;
	/* Set registers MODE2, LEDOUT0, LEDOUT1, ..., GRPPWM */
	unsigned char val[ledout_count + 2];
	int i;
	val[0] = 0x05;
	for (i = 1; i < ledout_count + 1; ++i) {
		val[i] = 0xFF;
	}
	val[ledout_count + 1] = 0x08;

	ret = pca9956_led_write_block_data(client, PCA9956_MODE_REGISTER_2,
						sizeof(val), val);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed on initialization LED.",
			__func__);
		return ret;
	}
	return 0;
}

/* Helper function to show pwm/iref control values. */
static ssize_t pca9956_led_show_helper(struct device *dev,
				       unsigned int led_index, char *buf,
				       u8 addr)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca9956_led *data = i2c_get_clientdata(client);
	int ret;
	u8 rgb[3];

	if (led_index >= data->pdata->num_leds) {
		dev_err(dev, "%s: invalid led number: %d\n", __func__,
			led_index);
		return -EINVAL;
	}

	addr += data->pdata->start_num[led_index];
	ret = pca9956_led_read_block_data(client, addr, sizeof(rgb), rgb);
	if (ret) {
		dev_err(dev, "%s: failed to read RGB values.\n", __func__);
		return -EIO;
	}

	return scnprintf(buf, PAGE_SIZE, "%hhu %hhu %hhu\n",
			rgb[0], rgb[1], rgb[2]);
}

/* Helper function to store pwm/iref control value to certain LED.
 * Input is a char array.
 * Format: "255 255 255"
 * The input represent R, G, B values from left to right, respectively.
 * R/G/B value can be either PWM or IREF control value. (0x00 - 0xFF)
 */
static ssize_t pca9956_led_store_helper(struct device *dev,
					unsigned int led_index,
					const char *buf, size_t count, u8 addr)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca9956_led *data = i2c_get_clientdata(client);
	int ret;
	u8 rgb[3];

	if (led_index >= data->pdata->num_leds) {
		dev_err(dev, "%s: invalid led number: %d\n", __func__,
			led_index);
		return -EINVAL;
	}

	ret = sscanf(buf, "%hhu %hhu %hhu", &rgb[0], &rgb[1], &rgb[2]);
	if (ret != 3) {
		dev_err(dev, "%s: failed to parse rgb values, ret=%d.\n",
			__func__, ret);
		return -EINVAL;
	}

	addr += data->pdata->start_num[led_index];
	ret = pca9956_led_write_block_data(client, addr, sizeof(rgb), rgb);
	if (ret < 0) {
		dev_err(dev, "%s: failed to write rgb values to led%d.\n",
			__func__, led_index);
		return ret;
	}
	dev_dbg(dev, "%s %s: set led%u(0x%02x): R=%hhu, G=%hhu, B=%hhu\n",
		__func__, client->name, led_index, addr,
		rgb[0], rgb[1], rgb[2]);
	return count;
}

static ssize_t pca9956_led_pwm_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned int led_index;
	int ret = sscanf(attr->attr.name, "pwm%u", &led_index);

	if (ret != 1) {
		dev_err(dev, "%s: failed to get led index.\n", __func__);
		return -EINVAL;
	}

	return pca9956_led_show_helper(dev, led_index, buf,
					pca9956_dev_to_register(dev)->reg_pwm_base);
}

static ssize_t pca9956_led_pwm_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned int led_index;
	int ret = sscanf(attr->attr.name, "pwm%u", &led_index);

	if (ret != 1) {
		dev_err(dev, "%s: failed to get led index.\n", __func__);
		return -EINVAL;
	}

	return pca9956_led_store_helper(dev, led_index, buf, count,
					pca9956_dev_to_register(dev)->reg_pwm_base);
}

/* Function to store pwm control values to specified LEDs.
 * Input Format" "LED_Index R G B LED_Index R G B ..."
 * Note: All LED indices not specified will have their
 *    (R,G,B) set to (0, 0 ,0)
 * LED indices can be specified in any order
 * For example: "0 255 255 255 1 100 150 200" is equivalent
 *    to "1 100 150 200 0 255 255 255"
 */
static ssize_t pca9956_led_pwm_all_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca9956_led *data = i2c_get_clientdata(client);
	int num_leds = 0;
	int chars_read = 0;
	int led_idx;
	int ret;
	int rgb_start_idx;
	u8 rgb[pca9956_dev_to_register(dev)->led_channels];
	memset(rgb, 0, sizeof(rgb));

	while (sscanf(buf, "%d%n", &led_idx, &chars_read) == 1) {
		if (led_idx > data->pdata->num_leds - 1 || led_idx < 0) {
			dev_err(dev, "%s: incorrect led index %d supplied\n", __func__, led_idx);
			return -EINVAL;
		}

		++num_leds;
		if (num_leds > data->pdata->num_leds) {
			dev_err(dev, "%s: incorrect number of leds supplied\n", __func__);
			return -EINVAL;
		}

		buf += chars_read;
		rgb_start_idx = data->pdata->start_num[led_idx];
		if (sscanf(buf, "%hhu %hhu %hhu%n", &rgb[rgb_start_idx],
					&rgb[rgb_start_idx + 1],
					&rgb[rgb_start_idx + 2],
					&chars_read) != 3) {
			dev_err(dev, "%s: unable to parse led values to write\n", __func__);
			return -EINVAL;
		}
		buf += chars_read;
	}

	ret = pca9956_led_write_block_data(client,
						pca9956_dev_to_register(dev)->reg_pwm_base,
						sizeof(rgb), rgb);
	if (ret < 0) {
		dev_err(dev, "%s: failed to write rgb values to all leds, err: %d\n",
				__func__, ret);
		return ret;
	}

	return count;
}

static ssize_t pca9956_led_iref_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned int led_index;
	int ret = sscanf(attr->attr.name, "iref%u", &led_index);

	if (ret != 1) {
		dev_err(dev, "%s: failed to get led index.\n", __func__);
		return -EINVAL;
	}

	return pca9956_led_show_helper(dev, led_index, buf,
					pca9956_dev_to_register(dev)->reg_iref_base);
}

static ssize_t pca9956_led_iref_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned int led_index;
	int ret = sscanf(attr->attr.name, "iref%u", &led_index);

	if (ret != 1) {
		dev_err(dev, "%s: failed to get led index.\n", __func__);
		return -EINVAL;
	}
	return pca9956_led_store_helper(dev, led_index, buf, count,
					pca9956_dev_to_register(dev)->reg_iref_base);
}

static ssize_t pca9956_led_show_byte_helper(struct device *dev,
					char *buf, u8 addr)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 read_buf;
	int ret = pca9956_led_read_byte(client, addr, &read_buf);

	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%u\n", read_buf);
}

static ssize_t pca9956_led_mode1_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return pca9956_led_show_byte_helper(dev, buf, PCA9956_MODE_REGISTER_1);
}

static ssize_t pca9956_led_global_dimming_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return pca9956_led_show_byte_helper(dev, buf,
			pca9956_dev_to_register(dev)->reg_global_dimming);
}

static ssize_t pca9956_led_global_dimming_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int global_dimming;
	int ret;

	if (kstrtouint(buf, 10, &global_dimming))
		return -EINVAL;

	/* Do the mask to handle minor input errors without reporting error. */
	global_dimming = global_dimming & 0xFF;
	ret = pca9956_led_write_byte(client,
			pca9956_dev_to_register(dev)->reg_global_dimming,
			global_dimming);
	if (ret < 0) {
		dev_err(dev, "%s: failed to write global_dimming.\n", __func__);
		return ret;
	}

	return count;
}

static ssize_t pca9956_led_init_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca9956_led *data = i2c_get_clientdata(client);
	int ret;

	dev_info(dev, "%s: initializing LED...\n", __func__);
	ret = pca9956_led_init(data);
	if (ret) {
		dev_err(dev, "%s: failed to initialize LED. ret=%d\n",
			__func__, ret);
		return ret;
	}

	return count;
}

static DEVICE_ATTR(mode1, S_IRUGO, pca9956_led_mode1_show, NULL);
static DEVICE_ATTR(global_dimming, S_IWUSR | S_IWGRP | S_IRUGO,
			pca9956_led_global_dimming_show,
			pca9956_led_global_dimming_store);
static DEVICE_ATTR(init_led, S_IWUSR | S_IWGRP, NULL, pca9956_led_init_store);
static DEVICE_ATTR(pwm_all, S_IWUSR | S_IWGRP | S_IRUGO,
			NULL, pca9956_led_pwm_all_store);

static struct attribute *other_led_attrs[] = {
	&dev_attr_global_dimming.attr,
	&dev_attr_init_led.attr,
	&dev_attr_mode1.attr,
	&dev_attr_pwm_all.attr,
	NULL
};


static const struct attribute_group pca9956_led_other_attr_group = {
	.attrs = other_led_attrs,
};

#ifdef CONFIG_OF
static struct pca9956_platform_data *pca9956_led_parse_devtree(
					struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *node;
	struct pca9956_platform_data *pdata;
	int reset_gpio = -1;

	node = dev->of_node;
	if (!node) {
		dev_err(dev, "%s: of_node is NULL.\n", __func__);
		return ERR_PTR(-ENODEV);
	}

	pdata = devm_kzalloc(dev, sizeof(struct device_node), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "%s: not enough memory left.\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	if (of_property_read_u32(node, "num_leds", &pdata->num_leds)) {
		dev_err(dev, "%s: failed to get number of leds.\n", __func__);
		devm_kfree(dev, pdata);
		return ERR_PTR(-EINVAL);
	}

	if (of_property_read_u32_array(node, "start_num", &pdata->start_num,
					pdata->num_leds)) {
		dev_err(dev, "%s: failed to get pwm register addresses.\n",
			__func__);
		devm_kfree(dev, pdata);
		return ERR_PTR(-EINVAL);
	}

	reset_gpio = of_get_named_gpio(node, "reset_gpio", 0);
	if (!gpio_is_valid(reset_gpio)) {
		dev_err(dev, "%s: failed to get reset_gpio.\n", __func__);
		return ERR_PTR(-ENODEV);
	}
	pdata->reset_gpio = reset_gpio;

	return pdata;
}
#endif

static int pca9956_led_reset(struct pca9956_led *data)
{
	struct i2c_client *client = data->client;
	int reset = data->pdata->reset_gpio;
	int ret = gpio_request(reset, "LedRST");
	if (ret) {
		dev_err(&client->dev, "%s: failed to request GPIO%d.\n",
			__func__, reset);
		return ret;
	}
	ret = gpio_direction_output(reset, 1);
	if (ret) {
		dev_err(&client->dev,
			"%s: failed to set gpio to 1.\n", __func__);
		gpio_free(reset);
		return ret;
	}
	gpio_set_value(reset, 0);
	msleep(1);
	gpio_set_value(reset, 1);

	gpio_free(reset);
	return ret;
}

static void delete_iref_pwm_attr_array(struct pca9956_led *data, int num_leds)
{
	if (data->iref_pwm_attrs) {
		int i;
		for (i = 0; i < 2 * num_leds ; i++) {
			kfree(data->iref_pwm_attrs[i].attr.name);
		}
		kfree(data->iref_pwm_attrs);
	}
	kfree(data->all_attrs);
	return;
}

static void construct_iref_pwm_device_attributes(struct pca9956_led *data,
                                                 int num_leds,
						 struct kobject *kobj) {
	int i, error;
	// need to NULL terminate the list of attributes -> add 1
	data->all_attrs = kzalloc(sizeof(struct attribute *) * (num_leds * 2 + 1),
				GFP_KERNEL);
	if (data->all_attrs == NULL)
		return;
	data->iref_pwm_attrs =
		kzalloc(sizeof(struct device_attribute) * num_leds * 2,
			GFP_KERNEL);
	if (data->iref_pwm_attrs == NULL)
		goto fail;
	for (i = 0; i < 2 * num_leds; ++i) {
		char buf[8];
		char* name;
		if (i < num_leds)
			snprintf(buf, sizeof(buf), "iref%u", i);
		else
			snprintf(buf, sizeof(buf), "pwm%u", i - num_leds);
		name = kzalloc(strlen(buf) + 1, GFP_KERNEL);
		if (name == NULL)
			goto fail;
		strncpy(name, buf, strlen(buf) + 1);

		data->iref_pwm_attrs[i].attr.name = name;
		data->iref_pwm_attrs[i].attr.mode = S_IWUSR | S_IWGRP | S_IRUGO;
		if (i < num_leds) {
			data->iref_pwm_attrs[i].show = pca9956_led_iref_show;
			data->iref_pwm_attrs[i].store = pca9956_led_iref_store;
		} else {
			data->iref_pwm_attrs[i].show = pca9956_led_pwm_show;
			data->iref_pwm_attrs[i].store = pca9956_led_pwm_store;
		}
		data->all_attrs[i] = &data->iref_pwm_attrs[i].attr;
	}
	data->iref_pwm_attr_group.attrs = data->all_attrs;
	error = sysfs_create_group(kobj, &data->iref_pwm_attr_group);
	if (error)
		printk("%s: failed to create sysfs, err:%d\n", __func__, error);
	return;

fail:
	delete_iref_pwm_attr_array(data, num_leds);
}

static int pca9956_led_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct pca9956_led *data;
	struct device *dev = &client->dev;
	struct pca9956_platform_data *pdata = dev_get_platdata(dev);
	int error = 0;

	if (!pdata) {
		pdata = pca9956_led_parse_devtree(client);
		if (IS_ERR(pdata)) {
			dev_err(dev,
			"%s: failed to get device data from device tree.\n",
			__func__);
			error = -EINVAL;
			goto err_get_pdata;
		}
	}
	dev_info(dev, "%s: %s number of leds supported: %d\n", __func__,
		client->name, pdata->num_leds);

	data = kzalloc(sizeof(struct pca9956_led), GFP_KERNEL);
	if (!data) {
		dev_err(dev, "%s: failed to allocate memory.\n",
			__func__);
		error = -ENOMEM;
		goto err_free_mem;
	}

	data->client = client;
	data->pdata = pdata;

	i2c_set_clientdata(client, data);

	error = pca9956_led_init(data);
	if (error) {
		dev_err(&client->dev, "%s: failed to initialize leds, err:%d\n",
			__func__, error);
		goto err_free_mem;
	}

	construct_iref_pwm_device_attributes(data, pdata->num_leds,
						&client->dev.kobj);
	error = sysfs_create_group(&client->dev.kobj,
					&pca9956_led_other_attr_group);
	if (error) {
		dev_err(&client->dev, "%s: failed to create sysfs, err:%d\n",
			__func__, error);
		goto err_free_sysfs;
	}

	dev_info(dev, "%s: %s probe successfully!\n", __func__, client->name);
	return 0;

err_free_sysfs:
	delete_iref_pwm_attr_array(data, pdata->num_leds);
err_free_mem:
	kfree(data);
err_get_pdata:
	return error;
}

static int pca9956_led_remove(struct i2c_client *client)
{
	/* There is no power-off for the product so probably i2c driver/device
	 * unbinding is not called. Leave this function here for driver
	 * integrity but return 0 directly.
	 * */
	return 0;
}

static void pca9956_led_shutdown(struct i2c_client *client)
{
	struct pca9956_led *data = i2c_get_clientdata(client);
	int num_leds = data->pdata->num_leds;
	int ret;

	dev_info(&client->dev, "%s: prepare to shutdown device.\n", __func__);
	ret = pca9956_led_reset(data);
	if (ret)
		dev_err(&client->dev, "%s: failed to reset LED.\n", __func__);

	sysfs_remove_group(&client->dev.kobj, &data->iref_pwm_attr_group);
	sysfs_remove_group(&client->dev.kobj, &pca9956_led_other_attr_group);
	delete_iref_pwm_attr_array(data, num_leds);
	kfree(data);
}

static const struct i2c_device_id pca9956_led_id[] = {
	{ "led-pca9955b", 0 },
	{ "led-pca9956b", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pca9956_led_id);

#ifdef CONFIG_OF
static const struct of_device_id pca9956_led_of_match[] = {
	{ .compatible = "nxp,led-pca9955b" },
	{ .compatible = "nxp,led-pca9956b" },
	{ }
};
#endif

static struct i2c_driver pca9956_led_driver = {
	.driver = {
		.name    = "pca9956-leds",
		.owner   = THIS_MODULE,
		.pm      = NULL,
		.of_match_table = of_match_ptr(pca9956_led_of_match),
	},
	.probe           = pca9956_led_probe,
	.remove          = pca9956_led_remove,
	.shutdown        = pca9956_led_shutdown,
	.id_table        = pca9956_led_id,
};

module_i2c_driver(pca9956_led_driver);

MODULE_AUTHOR("Mengyu Yuan <mengyu@google.com>");
MODULE_DESCRIPTION("NXP PCA9956B LED driver");
MODULE_LICENSE("GPL");
