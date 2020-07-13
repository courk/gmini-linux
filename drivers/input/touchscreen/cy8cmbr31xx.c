/*
 * Cypress CY8CMBR3xxx CapSense Express Controller Driver
 * All registers mentioned in the document are little endian.
 */
#include <linux/bsearch.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#include <linux/input/cy8cmbr3108.h>

#define CY8CMBR31XX_DRIVER_NAME "cy8cmbr3108"

/*
 * Cypress controller is entering a low power state that it wakes from on the
 * first address match, but will NACK it. Then, it will ACK the following
 * addresses until it re-enters low power mode.
 * We retry if the first few reads/writes fail.
 */
#define MAX_RETRIES			5

/*
 * Capacitive sensor enable/disable configuration. (R/W)
 * bit x corresponds to CSx
 * 0: Disabled, 1: Enabled
 */
#define SENSOR_EN			0x00

/*
 * Sensitivities (units: count/pF) for button sensors 0-3, 4-7
 * Each sensor accounts for two bits
 * Most sensitive <-----------------------------------------> Least sensitive
 * 0: 50counts/0.1pF, 1: 50counts/0.2pF, 2: 50counts/0.3pF, 3: 50counts/0.4pF
 *
 * ADDR: SENSITIVITY_BASE
 * +-------+-----+-----+-----+-----+-----+-----+-----+-----+
 * |  Bit  |  7  |  6  |  5  |	4  |  3  |  2  |  1  |	0  |
 * |-------+-----+-----+-----+-----+-----+-----+-----+-----|
 * | sensor|	CS3    |    CS2    |	CS1    |    CS0    |
 * +-------+-----------+-----------+-----------+-----------+
 * ADDR: SENSITIVITY_BASE+1
 * +-------+-----+-----+-----+-----+-----+-----+-----+-----+
 * |  Bit  |  7  |  6  |  5  |	4  |  3  |  2  |  1  |	0  |
 * |-------+-----+-----+-----+-----+-----+-----+-----+-----|
 * | sensor|	CS7    |    CS6    |	CS5    |    CS4    |
 * +-------+-----+-----+-----+-----+-----+-----+-----+-----+
 *
 */
#define SENSITIVITY_BASE		0x08

/*
 * Finger threshold (units: counts) for sensor 0. (R/W)
 * The valid range of this bit field: 31~200
 * The lower the more sensitive.
 * Dafault value: 128
 * sensor threshold address for CSx is 0x0C+x
 */
#define SENSOR_THRESHOLD_BASE		0x0C

/*
 * Sensor ON debounce configuration.
 * Number of consecutive scans for which a sensor's signal must be
 * above the finger threshold plus hysteresis in order for the device to report
 * an ON status.
 * The valid range of this bit field: 1~15
 */
#define SENSOR_DEBOUNCE			0x1C

/*
 * Button hysteresis override configuration.
 * Bit 7 Hysteresis override. 0: Disabled, 1: Enabled
 * Bit [4:0] Hysteresis value (unit: counts) to apply for button hysteresis
 * override.
 */
#define BUTTON_HYS			0x1D

/*
 * Low baseline reset parameter configuration for button sensor.
 * Bit 7 Low_baseline reset threshold override. 0: Disabled, 1: Enabled
 * Bit [6:0] Low baseline reset threshold
 */
#define BUTTON_LBR			0x1F

/*
 * Button negative noise threshold configuration.
 * Bit 7 Button negative noise threshold override. 0: Disabled, 1: Enabled
 * Bit [6:0] Button negative noise threshold
 */
#define BUTTON_NNT			0x20

/*
 * Button noise threshold configuration.
 * Bit 7 Button noise threshold override. 0: Disabled, 1: Enabled
 * Bit [6:0] Button noise threshold
 */
#define BUTTON_NT			0x21

/*
 * Global sensing and processing configuration. (R/W)
 * EMC solution enable (improves noise mitigation). (Bit 2)
 * 0: EMC solution disabled, 1: enabled
 *
 * Automatic threshold enable/disable configuration. (Bit 3)
 * 0: Disabled, 1: Enabled
 * To use customized thresholds, this bit must be unset.
 * Note that automatic thresholds can only be enabled if EMC solution is
 * disabled.
 *
 * Button and slider auto-reset configuration. (Bit 4 and 5)
 * This feature prevents a sensor from getting stuck ON in situations such as
 * a metal object placed too close to it.
 * 0: Disabled, 1: Enabled; timeout = 5s, 2: Enabled; timeout = 20s
 */
#define DEVICE_CFG2			0x4F

/*
 * Look for Touch/Look for Prox scan refresh time selection. (R/W)
 * Refresh interval in units of 20 ms. Default to 1.
 * The valid range of this bit field: 1~25
 * Bit 6 and 7 are reserved.
 */
#define REFRESH_CTRL			0x52

/*
 * Timeout (in seconds) of no touch activity to trigger (R/W)
 *   Active mode -> Look for Touch mode
 *   Look for Touch mode -> Look for Prox mode
 * Default value is 63.
 * The valid range of this bit field: 0~63
 * Bit 6 and 7 are reserved.
 */
#define STATE_TIMEOUT			0x55

/*
 * Configuration data CRC. (R/W)
 * CCITT CRC16 checksum for all data from offset 0 to 125.
 * The valid value of this bit field ranges from 0 to 65535.
 */
#define CONFIG_CRC			0x7E // 16 bits wide, use word R/W

/*
 * Command to execute. (R/W)
 *
 * Write x to CTRL_CMD...
 * x=2: The device calculates a CRC checksum over the configuration data in
 * this register map and compares the result with the content of CONFIG_CRC.
 * If the two values match, the device saves the configuration and the CRC
 * checksum to nonvolatile memory.
 *
 * x=3: The device calculates a CRC checksum over the configuration data in
 * this register map and places the result in the CALC_CRC register.
 *
 * x=255: The device resets itself
 */
#define CTRL_CMD			0x86

/*
 * Status returned by the most recently executed command. (R only)
 * bit 0 indicates status returned by the most recently executed command
 * 0: no error, 1: an error occurred
 */
#define CTRL_CMD_STATUS			0x88

/*
 * Error code returned from most recently executed command. (R only)
 * 0	: Command was successful
 * 253: Write to flash failed
 * 254: Stored configuration CRC checksum (in CONFIG_CRC register) did not
 *			match calculated configuration CRC checksum
 * 255: Invalid command
 */
#define CTRL_CMD_ERR			0x89

/*
 * Configuration data CRC calculated by host command. (R only)
 */
#define CALC_CRC			0x94 // 16 bits wide, use word R

/*
 * Button status indicators. (R only)
 * bit x corresponds to CSx.
 * 0: Not activated, 1: Activated
 * e.g. a value 6 (0b110)  means CS2 and CS1 are activated.
 */
#define BUTTON_STAT			0xAA

/*
 * Capacitive sensor 0 difference count signal. (R only)
 * 16 bits wide but the MSB is reserved, so only LSB is used
 * The valid range of this bit field: 0~255
 * difference count address for sensor x is (0xBA + 2*x)
 */
#define DIFFERENCE_COUNT_SENSOR_BASE	0xBA

static const int kDefaultConfig = 0x4CF3;

struct cy8cmbr31xx {
	struct i2c_client *client;
	struct input_dev *input;
	const struct cy8cmbr31xx_pdata *pdata;
	char phys[32];
	int irq;
	int reset;
};

// return value read (non-negative) else negative errno on error
static int cy8cmbr31xx_read_regs_byte(struct i2c_client *client, u8 reg){
	int ret, retry = 0;
	do {
		ret = i2c_smbus_read_byte_data(client, reg);
		retry++;
	} while( (ret < 0) && (retry < MAX_RETRIES) );

	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to read reg 0x%x.\n",
			__func__, reg);
		return ret;
	}

	dev_dbg(&client->dev, "%s: read 0x%x from register 0x%x\n", __func__,
		ret, reg);
	dev_dbg(&client->dev, "%s: retry = %d\n", __func__, retry);
	return ret;
}

// return negative errno else zero on success
static int cy8cmbr31xx_write_regs_byte(struct i2c_client *client, u8 reg,
					u8 val)
{
	int ret, retry = 0;
	do {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		retry++;
	} while( (ret < 0) && (retry < MAX_RETRIES) );

	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to write reg 0x%x.\n",
			__func__, reg);
		return ret;
	}

	dev_dbg(&client->dev, "%s: wrote 0x%x to register 0x%x\n", __func__,
		val, reg);
	dev_dbg(&client->dev, "%s: retry = %d\n", __func__, retry );

	return ret;
}

// return value read (non-negative) else negative errno on error
static int cy8cmbr31xx_read_regs_word(struct i2c_client *client, u8 reg){
	int ret, retry = 0;
	do {
		ret = i2c_smbus_read_word_data(client, reg);
		retry++;
	} while( (ret < 0) && (retry < MAX_RETRIES) );

	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to read reg 0x%x.\n",
			__func__, reg);
		return ret;
	}

	dev_dbg(&client->dev, "%s: read 0x%x from register 0x%x\n", __func__,
		ret, reg);
	dev_dbg(&client->dev, "%s: retry = %d\n", __func__, retry );
	return ret;
}

// return negative errno else zero on success
static int cy8cmbr31xx_write_regs_word(struct i2c_client *client, u8 reg,
					 u16 val)
{
	int ret, retry = 0;
	do {
		ret = i2c_smbus_write_word_data(client, reg, val);
		retry++;
	} while( (ret < 0) && (retry < MAX_RETRIES) );

	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to write reg 0x%x, "
			"ret=%d\n", __func__, reg, ret);
		return ret;
	}

	dev_dbg(&client->dev, "%s: wrote 0x%x to register 0x%x\n", __func__,
		val, reg);
	dev_dbg(&client->dev, "%s: retry = %d\n", __func__, retry);

	return ret;
}

// must be called each time after writing to CTRL_CMD
// return zero on success
//	  positive value on control command error
//	  negative value on reading error
static int cy8cmbr31xx_check_control_cmd_status(struct i2c_client *client) {
	int ret;
	ret = cy8cmbr31xx_read_regs_byte(client, CTRL_CMD_STATUS);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to get control command "
			"status\n", __func__);
		return ret;
	} else if (ret	& 0x1){
		ret = cy8cmbr31xx_read_regs_byte(client, CTRL_CMD_ERR);
		if (ret > 0)
			dev_err(&client->dev,"%s: command error. err: %d\n",
				__func__, ret);
		return ret;
	}
	return 0;
}

// must be called each time configuration is changed (i.e. write is performed)
// return zero on success
//	  positive value on control command error
//	  negative value on r/w error
static int cy8cmbr31xx_save_configuration(struct i2c_client *client) {
	int ret;
	/* calculate CRC */
	ret = cy8cmbr31xx_write_regs_byte(client, CTRL_CMD, 3);
	if (ret < 0){
		dev_err(&client->dev, "%s: failed to write calculate CRC "
			"command.\n", __func__);
		return ret;
	}

	ret = cy8cmbr31xx_check_control_cmd_status(client);
	if ( ret != 0)
		return ret;

	msleep(1); //to allow calculating CRC

	/* read out CRC just calculated */
	ret = cy8cmbr31xx_read_regs_word(client, CALC_CRC);
	if (ret < 0){
		dev_err(&client->dev,"%s: failed to get calculated CRC\n",
			__func__);
		return ret;
	}

	/* copy the value over and store in CONFIG_CRC */
	ret = cy8cmbr31xx_write_regs_word(client, CONFIG_CRC, ret);
	if (ret < 0){
		dev_err(&client->dev,"%s: failed to set CONFIG_CRC\n",
			__func__);
		return ret;
	}

	/* compare CRCs and save configuration */
	ret = cy8cmbr31xx_write_regs_byte(client, CTRL_CMD, 2);
	if (ret < 0){
		dev_err(&client->dev, "%s: failed to save CRC\n", __func__);
		return ret;
	}
	dev_info(&client->dev, "%s: saving CRC\n", __func__);
	msleep(1000);
	ret = cy8cmbr31xx_check_control_cmd_status(client);
	if ( ret != 0) {
		dev_err(&client->dev,"%s: failed to save configuration\n",
			__func__);
		return ret;
	}
	ret = cy8cmbr31xx_write_regs_byte(client, CTRL_CMD, 255);
	if (ret < 0){
		dev_err(&client->dev, "%s: failed to do software reset\n",
			__func__);
		return ret;
	}

	dev_info(&client->dev, "%s: successfully saved configuration\n",
		 __func__);
	return 0;
}

static ssize_t cy8cmbr31xx_show_helper(struct i2c_client *client,
	char *buf, u8 reg)
{
	int ret;
	ret = cy8cmbr31xx_read_regs_byte(client, reg);
	if (ret < 0) {
		return ret;
	}
	return scnprintf(buf, PAGE_SIZE, "%hhu\n", ret);
}

// use when you only care about few bits (must be consecutive) in that register
static ssize_t cy8cmbr31xx_show_helper_masked(struct i2c_client *client,
	char *buf, u8 reg, u8 mask, u8 effective_lsb)
{
	int ret;
	ret = cy8cmbr31xx_read_regs_byte(client, reg);
	if (ret < 0) {
		return ret;
	}
	return scnprintf(buf, PAGE_SIZE, "%hhu\n",
			 (ret & mask) >> effective_lsb);
}

static ssize_t cy8cmbr31xx_store_helper(struct i2c_client *client,
	const char *buf, size_t count, u8 reg)
{
	int ret;
	u8 val;

	ret = sscanf(buf, "%hhu", &val);
	if (ret != 1) {
		dev_err(&client->dev, "%s: failed to parse input value (%s), "
			"ret=%d.\n", __func__, buf, ret);
		return -EINVAL;
	}

	if (val > 255) {
	    dev_err(&client->dev,
		    "%s: invalid input value (%s), must be in the range of "
		    "[0:255]\n", __func__, buf);
	    return -EINVAL;
	}

	ret = cy8cmbr31xx_write_regs_byte(client, reg, val);
	if (ret < 0)
		return ret;

	ret = cy8cmbr31xx_save_configuration(client);
	if (ret)
		return ret;
	return count;
}

// use when you only modify few bits (must be consecutive) in that register
static ssize_t cy8cmbr31xx_store_helper_masked(struct i2c_client *client,
	const char *buf, size_t count, u8 reg, u8 mask, u8 effective_lsb)
{
	int ret;
	u8 val;

	ret = sscanf(buf, "%hhu", &val);
	if (ret != 1) {
		dev_err(&client->dev, "%s: failed to parse input value (%s), "
			"ret=%d.\n", __func__, buf, ret);
		return -EINVAL;
	}

	if (val > (mask >> effective_lsb)) {
	    dev_err(&client->dev,
		    "%s: invalid input value (%s), must be in the range of "
		    "[0:%d]\n", __func__, buf, mask >> effective_lsb);
	    return -EINVAL;
	}

	ret = cy8cmbr31xx_read_regs_byte(client, reg);
	if (ret < 0)
		return ret;

	val = (ret & ~mask) | (val << effective_lsb);
	// config unchanged
	if (val == ret)
		return count;

	ret = cy8cmbr31xx_write_regs_byte(client, reg, val);
	if (ret < 0)
		return ret;

	ret = cy8cmbr31xx_save_configuration(client);
	if (ret)
		return ret;
	return count;
}

static ssize_t cy8cmbr31xx_sensor_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf){
	struct i2c_client *client = to_i2c_client(dev);
	struct cy8cmbr31xx *ts = i2c_get_clientdata(client);
	unsigned int sensor_index;
	int ret = sscanf(attr->attr.name, "thold%u", &sensor_index);
	if (ret != 1) {
		dev_err(dev, "%s: failed to get sensor index.\n", __func__);
		return -EINVAL;
	}

	if (sensor_index >= ts->pdata->num_sensors) {
		dev_err(dev, "%s: invalid sensor index: %d\n", __func__,
			sensor_index);
		return -EINVAL;
	}

	return cy8cmbr31xx_show_helper(client, buf,
					SENSOR_THRESHOLD_BASE+sensor_index);
}

static ssize_t cy8cmbr31xx_sensor_threshold_store(struct device *dev,
		struct device_attribute *attr, char *buf, ssize_t count){
	struct i2c_client *client = to_i2c_client(dev);
	struct cy8cmbr31xx *ts = i2c_get_clientdata(client);
	unsigned int sensor_index;
	int ret = sscanf(attr->attr.name, "thold%u", &sensor_index);
	if (ret != 1) {
		dev_err(dev, "%s: failed to get sensor index.\n", __func__);
		return -EINVAL;
	}

	if (sensor_index >= ts->pdata->num_sensors) {
		dev_err(dev, "%s: invalid sensor index: %d\n", __func__,
			sensor_index);
		return -EINVAL;
	}

	return cy8cmbr31xx_store_helper(client, buf, count,
					SENSOR_THRESHOLD_BASE+sensor_index);
}

static ssize_t cy8cmbr31xx_sensitivity_show(struct device *dev,
		struct device_attribute *attr, char *buf){
	struct i2c_client *client = to_i2c_client(dev);
	struct cy8cmbr31xx *ts = i2c_get_clientdata(client);
	unsigned int sensor_index;
	int ret = sscanf(attr->attr.name, "sens%u", &sensor_index);
	if (ret != 1) {
		dev_err(dev, "%s: failed to get sensor index.\n", __func__);
		return -EINVAL;
	}

	if (sensor_index >= ts->pdata->num_sensors) {
		dev_err(dev, "%s: invalid sensor index: %d\n", __func__,
			sensor_index);
		return -EINVAL;
	}

	return cy8cmbr31xx_show_helper_masked(client, buf,
		SENSITIVITY_BASE + sensor_index / 4,
		0x03 << (2 * (sensor_index % 4)), 2 * (sensor_index % 4));
}

static ssize_t cy8cmbr31xx_sensitivity_store(struct device *dev,
		struct device_attribute *attr, char *buf, ssize_t count){
	struct i2c_client *client = to_i2c_client(dev);
	struct cy8cmbr31xx *ts = i2c_get_clientdata(client);
	unsigned int sensor_index;
	int ret = sscanf(attr->attr.name, "sens%u", &sensor_index);
	if (ret != 1) {
		dev_err(dev, "%s: failed to get sensor index.\n", __func__);
		return -EINVAL;
	}

	if (sensor_index >= ts->pdata->num_sensors) {
		dev_err(dev, "%s: invalid sensor index: %d\n", __func__,
			sensor_index);
		return -EINVAL;
	}

	return cy8cmbr31xx_store_helper_masked(client, buf, count,
		SENSITIVITY_BASE + sensor_index / 4,
		0x03 << (2 * (sensor_index % 4)), 2 * (sensor_index % 4));
}

static ssize_t cy8cmbr31xx_sensor_count_show(struct device *dev,
		struct device_attribute *attr, char *buf){
	struct i2c_client *client = to_i2c_client(dev);
	struct cy8cmbr31xx *ts = i2c_get_clientdata(client);
	unsigned int sensor_index;
	int ret = sscanf(attr->attr.name, "count%u", &sensor_index);
	if (ret != 1) {
		dev_err(dev, "%s: failed to get sensor index.\n", __func__);
		return -EINVAL;
	}

	if (sensor_index >= ts->pdata->num_sensors) {
		dev_err(dev, "%s: invalid sensor index: %d\n", __func__,
			sensor_index);
		return -EINVAL;
	}

	return cy8cmbr31xx_show_helper(client, buf,
				 DIFFERENCE_COUNT_SENSOR_BASE+2*sensor_index);
}

static struct attribute **all_attrs;
static struct attribute_group cy8cmbr31xx_sensors_attr_group;
static struct device_attribute *sensors_attrs;

static void cy8cmbr31xx_delete_sensors_attr_array(int num_sensors) {
	if (sensors_attrs) {
		int i;
		for (i = 0; i < 3 * num_sensors ; ++i) {
			if (sensors_attrs[i].attr.name)
				kfree(sensors_attrs[i].attr.name);
		}
		kfree(sensors_attrs);
	}
	kfree (all_attrs);
	return;
}

static void cy8cmbr31xx_construct_sensors_attributes(int num_sensors,
						struct kobject *kobj)
{
	int i, error;
	// need to NULL terminate the list of attributes -> add 1
	all_attrs = kzalloc(sizeof(struct attribute *) * (num_sensors * 3 + 1),
				GFP_KERNEL);
	if (all_attrs == NULL)
		return;
	sensors_attrs = kzalloc(
		sizeof(struct device_attribute) * num_sensors * 3, GFP_KERNEL);
	if (sensors_attrs == NULL)
		goto fail;
	for (i = 0; i < 3 * num_sensors; ++i) {
		char buf[8];
		char* name;
		if (i < num_sensors)
			snprintf(buf, sizeof(buf), "count%u", i);
		else if (i < 2 * num_sensors)
			snprintf(buf, sizeof(buf), "thold%u", i - num_sensors);
		else
			snprintf(buf, sizeof(buf), "sens%u", i - 2 * num_sensors);

		name = kzalloc(strlen(buf) + 1, GFP_KERNEL);
		if (name == NULL)
			goto fail;
		strncpy(name, buf, strlen(buf) + 1);

		sensors_attrs[i].attr.name = name;
		if (i < num_sensors) { //count (R only)
			sensors_attrs[i].attr.mode = S_IRUGO;
			sensors_attrs[i].show = cy8cmbr31xx_sensor_count_show;
		} else if (i < 2 * num_sensors){ //thold
			sensors_attrs[i].attr.mode = S_IWUSR | S_IWGRP | S_IRUGO;
			sensors_attrs[i].show = cy8cmbr31xx_sensor_threshold_show;
			sensors_attrs[i].store = cy8cmbr31xx_sensor_threshold_store;
		} else { //sens
			sensors_attrs[i].attr.mode = S_IWUSR | S_IWGRP | S_IRUGO;
			sensors_attrs[i].show = cy8cmbr31xx_sensitivity_show;
			sensors_attrs[i].store = cy8cmbr31xx_sensitivity_store;
		}
		all_attrs[i] = &sensors_attrs[i].attr;
	}
	cy8cmbr31xx_sensors_attr_group.attrs = all_attrs;
	error = sysfs_create_group(kobj, &cy8cmbr31xx_sensors_attr_group);
	if (error)
		printk("%s: failed to create sysfs, err:%d\n", __func__, error);
	return;

fail:
	cy8cmbr31xx_delete_sensors_attr_array(num_sensors);
}

static ssize_t cy8cmbr31xx_reset_store(struct device *dev,
		struct device_attribute *attr, char *buf, ssize_t count){

	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct cy8cmbr31xx *ts = i2c_get_clientdata(client);

	error = gpio_request(ts->reset, "TouchRST");
	if (error) {
		dev_err(dev, "%s: failed to request GPIO%d.\n",
			__func__, ts->reset);
		return error;
	}

	error = gpio_direction_output(ts->reset, 1);
	if (error) {
		dev_err(dev, "%s: failed to set gpio to output high.\n",
			__func__);
		gpio_free(ts->reset);
		return error;
	}

	dev_info(dev, "%s: resetting touch controller...\n", __func__);
	gpio_set_value(ts->reset, 0);
	msleep(1);
	gpio_set_value(ts->reset, 1);
	gpio_free(ts->reset);
	return count;
}

static const struct map_entry {
	const char* name;
	const u8 reg;
	const u8 mask;
	const u8 lsb;
} kNameRegisterMaskLsbMap[] = {
	// Must keep in alphabetical order to use bsearch
	/*  attr.name	      register	     mask    lsb */
	{"auto_threshold",  DEVICE_CFG2,     0x08,    3},
	{"debounce",	    SENSOR_DEBOUNCE, 0x0F,    0},
	{"emc",		    DEVICE_CFG2,     0x04,    2},
	{"hys_override",    BUTTON_HYS,	     0x80,    7},
	{"hys_threshold",   BUTTON_HYS,	     0x1F,    0},
	{"lbr_override",    BUTTON_LBR,	     0x80,    7},
	{"lbr_threshold",   BUTTON_LBR,	     0x7F,    0},
	{"nnt_override",    BUTTON_NNT,	     0x80,    7},
	{"nnt_threshold",   BUTTON_NNT,	     0x7F,    0},
	{"nt_override",	    BUTTON_NT,	     0x80,    7},
	{"nt_threshold",    BUTTON_NT,	     0x7F,    0},
	{"refresh_ctrl",    REFRESH_CTRL,    0x3F,    0},
	{"state_timeout",   STATE_TIMEOUT,   0x3F,    0},
};

const size_t kMapLength = ARRAY_SIZE(kNameRegisterMaskLsbMap);

static int compare_map_entry(const void *e1, const void *e2) {
	struct map_entry *entry1 = (struct map_entry *) e1;
	struct map_entry *entry2 = (struct map_entry *) e2;
	return strcmp(entry1->name, entry2->name);
}

static ssize_t cy8cmbr31xx_common_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct map_entry key = {
		.name = attr->attr.name,
	};
	struct map_entry *res;
	res = bsearch(&key, kNameRegisterMaskLsbMap, kMapLength,
			sizeof(kNameRegisterMaskLsbMap[0]), compare_map_entry);
	if (res == NULL) {
		printk("%s: attribute name (%s) cannot be found!\n", __func__,
			key.name);
		return -ENOENT;
	}
	return cy8cmbr31xx_show_helper_masked(client, buf, res->reg, res->mask,
						res->lsb);
}

static ssize_t cy8cmbr31xx_common_store(struct device *dev,
		struct device_attribute *attr, char *buf, size_t count) {
	struct i2c_client *client = to_i2c_client(dev);
	struct map_entry key, *res;
	key.name = attr->attr.name;
	res = bsearch(&key, kNameRegisterMaskLsbMap, kMapLength,
			sizeof(struct map_entry), compare_map_entry);
	if (res == NULL) {
		printk("%s: attribute name (%s) cannot be found!\n", __func__,
			key.name);
		return -ENOENT;
	}
	return cy8cmbr31xx_store_helper_masked(client, buf, count, res->reg,
						res->mask, res->lsb);
}

static struct attribute **all_common_attrs;
static struct attribute_group cy8cmbr31xx_common_attr_group;
static struct device_attribute *common_attrs;

static void cy8cmbr31xx_delete_common_attr_array(void) {
	kfree(common_attrs);
	kfree(all_common_attrs);
}

static void cy8cmbr31xx_construct_common_attributes(struct kobject *kobj)
{
	int i, error;
	// add 1 for attribute reset
	size_t kCommonAttrLength = kMapLength + 1;

	// need to NULL terminate the list of attributes -> add 1
	all_common_attrs =
		kzalloc(sizeof(struct attribute *) * kCommonAttrLength + 1,
			GFP_KERNEL);
	if (all_common_attrs == NULL)
		return;
	common_attrs =
		kzalloc(sizeof(struct device_attribute) * kCommonAttrLength,
			GFP_KERNEL);
	if (common_attrs == NULL)
		goto fail;
	for (i = 0; i < kMapLength; ++i) {
		common_attrs[i].attr.name = kNameRegisterMaskLsbMap[i].name;
		common_attrs[i].attr.mode = S_IWUSR | S_IWGRP | S_IRUGO;
		common_attrs[i].show = cy8cmbr31xx_common_show;
		common_attrs[i].store = cy8cmbr31xx_common_store;
		all_common_attrs[i] = &common_attrs[i].attr;
	}
	// The last one is for reset
	common_attrs[i].attr.name = "reset";
	common_attrs[i].attr.mode = S_IWUSR | S_IWGRP | S_IRUGO;
	common_attrs[i].show = NULL;
	common_attrs[i].store = cy8cmbr31xx_reset_store;
	all_common_attrs[i] = &common_attrs[i].attr;

	cy8cmbr31xx_common_attr_group.attrs = all_common_attrs;
	error = sysfs_create_group(kobj, &cy8cmbr31xx_common_attr_group);
	if (error)
		printk("%s: failed to create sysfs, err:%d\n", __func__, error);
	return;

fail:
	cy8cmbr31xx_delete_common_attr_array();
}

static void cy8cmbr31xx_report_touch(struct cy8cmbr31xx *ts)
{
	struct i2c_client *client = ts->client;
	struct input_dev *input = ts->input;
	int ret;
	ret = cy8cmbr31xx_read_regs_byte(client, BUTTON_STAT);

	if (ret >= 0) {
		input_event(input, EV_MSC, MSC_PULSELED, ret);
		input_sync(input);
                // TODO(yichunko): remove this
                printk("%s: button status: %d\n", __func__, ret);
	}
}

static irqreturn_t cy8cmbr31xx_interrupt(int irq, void *handle)
{
	struct cy8cmbr31xx *ts = handle;
	cy8cmbr31xx_report_touch(ts);
	return IRQ_HANDLED;
}


#ifdef CONFIG_OF
static struct cy8cmbr31xx_pdata *cy8cmbr31xx_parse_devtree(
					struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *node;
	struct cy8cmbr31xx_pdata *pdata;
	int touch_gpio = -1;
	int reset_gpio = -1;
	int error = 0;

	node = dev->of_node;
	if (!node) {
		dev_err(dev, "%s: of_node is NULL.\n", __func__);
		return -ENODEV;
	}

	pdata = devm_kzalloc(dev, sizeof(struct device_node), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "%s: not enough memory left.\n", __func__);
		return -ENOMEM;
	}

	if (of_property_read_u32(node, "num_sensors", &pdata->num_sensors)) {
		dev_err(dev, "%s: failed to get number of sensors.\n",
			__func__);
		devm_kfree(dev, pdata);
		return ERR_PTR(-EINVAL);
	}

	/* interrupt gpio. */
	touch_gpio = of_get_named_gpio(node, "touch_gpio", 0);
	if (!gpio_is_valid(touch_gpio))
		goto exit;

	error = gpio_request(touch_gpio, "TouchIRQ");

	if (error)
		goto exit;

	error = gpio_direction_input(touch_gpio);
	if (error) {
		gpio_free(touch_gpio);
		goto exit;
	}
	pdata->irq = gpio_to_irq(touch_gpio);
	if (pdata->irq < 0) {
		dev_err(dev, "%s: failed to get irq number for GPIO%d. "
			"err:%d.\n", __func__, touch_gpio, pdata->irq);
		goto exit;
	}

	/* reset gpio */
	reset_gpio = of_get_named_gpio(node, "reset_gpio", 0);
	if (!gpio_is_valid(reset_gpio))
		goto exit;

	pdata->reset = reset_gpio;

	return pdata;
exit:
	if (touch_gpio != -1)
		gpio_free(touch_gpio);
	dev_err(dev, "%s: Failed to request gpio\n", __func__);
	return -ENODEV;
}
#endif

// return zero on success
//	  positive value on control command error
//	  negative value on r/w error
static int cy8cmbr31xx_init(struct cy8cmbr31xx *ts)
{
	struct i2c_client *client = ts->client;
	int ret, i;
	ret = cy8cmbr31xx_read_regs_word(client, CONFIG_CRC);
	// skip the rest if configuration is default.
	if (ret == kDefaultConfig) {
		dev_info(&client->dev, "%s: default setting, no further "
			 "action needed.\n", __func__);
		return 0;
	}
	if (ret > 0) {
		dev_info(&client->dev, "%s: configuration data CRC (0x%x), "
			 "does not match, need to re-configure.\n", __func__,
			 ret);
	}
	/* Enable CS4 and CS5 */
	ret = cy8cmbr31xx_write_regs_byte(client, SENSOR_EN, 0x30);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to configure CS4, CS5, "
			"ret=%d.\n", __func__, ret);
		return ret;
	}
	ret = cy8cmbr31xx_write_regs_byte(client, DEVICE_CFG2, 0x14);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to enable auto-reset and "
			"EMC, ret=%d.\n", __func__, ret);
		return ret;
	}
	ret = cy8cmbr31xx_write_regs_word(client, SENSITIVITY_BASE, 0x00);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to reset sensitivity0, "
			"ret=%d.\n", __func__, ret);
		return ret;
	}
	ret = cy8cmbr31xx_write_regs_word(client, SENSITIVITY_BASE+1, 0x0F);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to reset sensitivity1, "
			"ret=%d.\n", __func__, ret);
		return ret;
	}
	for (i = 0; i < 6; ++i) {
		u8 value;
		if ( i <= 3 ) {
			value = 0x80;
		} else {
			value = 0x5A;
		}
		ret = cy8cmbr31xx_write_regs_byte(client,
						  SENSOR_THRESHOLD_BASE + i,
						  value);
		if (ret < 0) {
			dev_err(&client->dev,  "%s: failed to set "
				"threshold %d to default value(%d). "
				"ret=%d.\n", __func__, i, value, ret);
			return ret;
		}
	}
	ret = cy8cmbr31xx_write_regs_word(client, REFRESH_CTRL, 0x01);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to set refresh interval, "
			"ret=%d.\n", __func__, ret);
		return ret;
	}
	ret = cy8cmbr31xx_write_regs_word(client, STATE_TIMEOUT, 0x3F);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to set timeout of no touch "
			"activity, ret=%d.\n", __func__, ret);
		return ret;
	}

	ret = cy8cmbr31xx_write_regs_word(client, SENSOR_DEBOUNCE, 0x03);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to reset sensor debounce "
			"configuration, ret=%d.\n", __func__, ret);
		return ret;
	}

	ret = cy8cmbr31xx_write_regs_word(client, BUTTON_HYS, 0x0C);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to reset hysteresis "
			"override configuration, ret=%d.\n", __func__, ret);
		return ret;
	}

	ret = cy8cmbr31xx_write_regs_word(client, BUTTON_LBR, 0x32);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to reset low baseline "
			"parameter configuration, ret=%d.\n", __func__, ret);
		return ret;
	}

	ret = cy8cmbr31xx_write_regs_word(client, BUTTON_NNT, 0x33);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to reset negative noise "
			"threshold configuration, ret=%d.\n", __func__, ret);
		return ret;
	}

	ret = cy8cmbr31xx_write_regs_word(client, BUTTON_NT, 0x33);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to reset noise threshold "
			"configuration, ret=%d.\n", __func__, ret);
		return ret;
	}

	ret = cy8cmbr31xx_save_configuration(client);
	return ret;
}

// return zero on success
static int cy8cmbr31xx_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct cy8cmbr31xx *ts;
	struct device *dev = &client->dev;
	const struct cy8cmbr31xx_pdata *pdata = dev_get_platdata(dev);
	struct input_dev *input_dev;
	int error = 0;

	if (!pdata) {
		pdata = cy8cmbr31xx_parse_devtree(client);
		if (IS_ERR(pdata)) {
			dev_err(dev,
			"%s: failed to get device data from device tree.\n",
			__func__);
			error = -EINVAL;
			goto err_get_pdata;
		}
	}

	ts = kzalloc(sizeof(struct cy8cmbr31xx), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		dev_err(&client->dev, "%s: No enough memory.\n", __func__);
		error = -ENOMEM;
		goto err_free_mem;
	}

	ts->client = client;
	ts->input = input_dev;
	ts->pdata = pdata;
	ts->irq = pdata->irq;
	ts->reset = pdata->reset;

	snprintf(ts->phys, sizeof(ts->phys), "%s/input0",
		 dev_name(&client->dev));

	input_dev->name = CY8CMBR31XX_DRIVER_NAME;
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;

	input_dev->evbit[0] = BIT_MASK(EV_MSC);
	// use MSC_PULSELED for joplin to report button status
	// since MSC_RAW has been used for rotation speed in chirp
	input_set_capability(input_dev, EV_MSC, MSC_PULSELED);

	input_set_drvdata(input_dev, ts);
	i2c_set_clientdata(client, ts);

	error = cy8cmbr31xx_init(ts);
	if (error) {
		dev_err(&client->dev, "%s: init failed...\n", __func__ );
		goto err_free_mem;
	}

	cy8cmbr31xx_construct_sensors_attributes(pdata->num_sensors,
						 &client->dev.kobj);
	cy8cmbr31xx_construct_common_attributes(&client->dev.kobj);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(&client->dev,
			"%s: failed to register input_dev, err:%d\n",
			__func__, error);
		goto err_remove_sysfs;
	}

	error = request_threaded_irq(pdata->irq, NULL, cy8cmbr31xx_interrupt,
					IRQF_ONESHOT | IRQF_TRIGGER_RISING,
					client->name, ts);

	if (error) {
		dev_err(&client->dev, "%s: failed to request IRQ %d, err:%d\n",
			__func__, ts->irq, error);
		goto err_unregister_device;
	}

	dev_info(&client->dev, "%s: probe successfully!\n", __func__);
	return 0;

err_unregister_device:
	input_unregister_device(input_dev);
	input_dev = NULL;
err_remove_sysfs:
	sysfs_remove_group(&client->dev.kobj, &cy8cmbr31xx_common_attr_group);
	cy8cmbr31xx_delete_common_attr_array();
	sysfs_remove_group(&client->dev.kobj, &cy8cmbr31xx_sensors_attr_group);
	cy8cmbr31xx_delete_sensors_attr_array(pdata->num_sensors);
err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
err_get_pdata:
	return error;
}

static int cy8cmbr31xx_remove(struct i2c_client *client)
{
	return 0;
}

static void cy8cmbr31xx_shutdown(struct i2c_client *client)
{
	struct cy8cmbr31xx *ts = i2c_get_clientdata(client);
	int num_sensors = ts->pdata->num_sensors;
	dev_info(&client->dev, "%s: prepare to shutdown device.\n", __func__);

	sysfs_remove_group(&client->dev.kobj, &cy8cmbr31xx_common_attr_group);
	cy8cmbr31xx_delete_common_attr_array();
	sysfs_remove_group(&client->dev.kobj, &cy8cmbr31xx_sensors_attr_group);
	cy8cmbr31xx_delete_sensors_attr_array(num_sensors);
	kfree(ts);
}

static const struct i2c_device_id cy8cmbr31xx_idtable[] = {
	{ CY8CMBR31XX_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cy8cmbr31xx_idtable);

#ifdef CONFIG_OF
static const struct of_device_id cy8cmbr31xx_of_match[] = {
	{ .compatible = CY8CMBR31XX_DRIVER_NAME },
	{ }
};
#endif

static struct i2c_driver cy8cmbr31xx_driver = {
	.driver = {
		.name		= CY8CMBR31XX_DRIVER_NAME,
		.owner		= THIS_MODULE,
		.pm		= NULL,
		.of_match_table = of_match_ptr(cy8cmbr31xx_of_match),
	},
	.probe			= cy8cmbr31xx_probe,
	.remove			= cy8cmbr31xx_remove,
	.shutdown		= cy8cmbr31xx_shutdown,
	.id_table		= cy8cmbr31xx_idtable,
};

module_i2c_driver(cy8cmbr31xx_driver);

MODULE_AUTHOR("Yi-Chun Gina Ko <yichunko@google.com>");
MODULE_DESCRIPTION("CYPRESS CY8CMBR31XX CAPSENSE TOUCH driver");
MODULE_LICENSE("GPL");
