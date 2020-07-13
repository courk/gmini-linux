/*
 * Atmel SAM D21X Series Touchscreen Driver
 *
 * Touch event in application firmware is included inside memory_map in firmware
 * image, here is the struct of memory map as a reference:
 *      typedef struct tag_memory_map_body_t
 *      {
 *              uint8_t firmware_info;  // 0x00
 *              uint8_t host_ctrl;  // 0x01
 *              uint16_t X_position;  // 0x02
 *              uint16_t Y_position;  // 0x04
 *              uint8_t gesture_type;  // 0x06
 *              uint8_t rotation_ticks;  // 0x07
 *              uint16_t als_value;  // 0x08
 *              uint8_t future_1;  // 0x0A
 *              uint8_t reg_status;  // 0x0B
 *              uint8_t reg_boot;  // 0x0C
 *              uint8_t future_2;  // 0x0D
 *              uint8_t future_3;  // 0x0E
 *              uint8_t future_4;  // 0x0F
 *      } memory_map_body_t;
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c/atsam_d21x.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

/* MCU modes */
#define ATSAM_D21X_BOOTLOADER_MODE	0
#define ATSAM_D21X_APPLICATION_MODE	1
#define ATSAM_D21X_FACTORY_MODE		2

#define ATSAM_D21X_TOUCH_EVENT_SIZE	10
#define ATSAM_D21X_FW_FRAME_SIZE	256

/* This is not the real minimum size of firmware, the size of firmware is
 * guaranteed to be a multiple of 256 bytes. When retrieving the firmware
 * version from the binary, we need to go the 8th byte(starts with 1) starting
 * from the end of raw data. This macro is used to make sure the pointer doesn't
 * go out of scope.
 * XX XX XX XX XX XX XX 01 XX XX XX GH EF CD AB <- end of firmware binary
 * firmware version: 0x01
 * crc32: 0xABCDEFGH(little endian)
 */
#define ATSAM_D21X_MINIMUM_FW_SIZE	8

/* 100 points for touch raw data. */
#define ATSAM_D21X_RAW_DATA_NUM		100
/* Each raw data point is of 16 bits. */
#define ATSAM_D21X_RAW_DATA_SIZE	(ATSAM_D21X_RAW_DATA_NUM * 2)

#define ATSAM_D21X_FW_VERSION_ADDR	0x00
#define ATSAM_D21X_HOST_CTRL_ADDR	0x01
#define ATSAM_D21X_MCU_STATUS_ADDR	0x0B
#define ATSAM_D21X_APPLICATION_CMD_ADDR	0x0C
#define ATSAM_D21X_RAW_DATA_ADDR	0x0E
#define ATSAM_D21X_FLASH_ADDR		0x80

#define ATSAM_D21X_BL_BUF_MASK		0x01
#define ATSAM_D21X_MCU_CRC_MASK		0x02
#define ATSAM_D21X_MCU_MODE_MASK	0x80

#define ATSAM_D21X_APPLICATION_FIRMWARE	"atmel/atmel_sam_d21g.bin"
#define ATSAM_D21X_FACTORY_FIRMWARE	"atmel/atmel_sam_d21g_factory.bin"

#define ATSAM_D21X_WORK_INTERVAL_MS	2000  /* ms */

#define NUM_OF_RETRIES			5

struct atsam_d21x {
	struct i2c_client *client;
	struct input_dev *input_dev;
	char phys[64];
	const struct atsam_d21x_platform_data *pdata;
	/* It is used as a flag to prevent request_firmware,
	 * request_firmware_nowait from accessing the same firmware
	 * simultaneously.
	 */
	atomic_t fw_updating;
	int irq;
	int reset;
	u8 fw_version;
	/* Buffer to hold all raw touch data in factory tests.
	 * There are 100 touch points. Each one is 2 bytes.
	 */
	u8 raw_data[ATSAM_D21X_RAW_DATA_SIZE];

	/* Workqueue for touch sanity check. */
	struct delayed_work touch_work;
	struct workqueue_struct *touch_workqueue;
};

static int atsam_d21x_i2c_read_data(struct i2c_client *client, u8 reg,
					  u16 len, void *val)
{
	struct i2c_msg xfer[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = val,
		},
	};
	int bytes_transferred;
	int ret;

	bytes_transferred = i2c_transfer(client->adapter, xfer,
						ARRAY_SIZE(xfer));
	if (bytes_transferred == ARRAY_SIZE(xfer)) {
		ret = 0;
	} else {
		/* read invalid number of bytes */
		if (bytes_transferred >= 0)
			ret = -EIO;
		dev_err(&client->dev, "%s: i2c transfer failed (%d)\n",
			__func__, ret);
	}

	return ret;
}

static int atsam_d21x_i2c_write_data(struct i2c_client *client, u8 reg,
					u16 len, const void *val)
{
	int bytes_transferred;
	int ret;
	size_t count = len + 1;
	u8 buf[count];

	buf[0] = reg;
	memcpy(&buf[1], val, len);

	bytes_transferred = i2c_master_send(client, buf, count);
	if (bytes_transferred == count) {
		ret = 0;
	} else {
		if (bytes_transferred >= 0)
			ret = -EIO;
		dev_err(&client->dev, "%s: i2c send failed, err=%d",
			__func__, ret);
	}

	return ret;
}

static void atsam_d21x_report_raw_data(struct atsam_d21x *data)
{
	struct i2c_client *client = data->client;
	/* Clear the read raw data command before reading raw data, after INT line
	 * is pulled to low, otherwise, the MCU will keep reading raw data.
	 */
	const u8 host_ctrl = 0x00;
	int ret = atsam_d21x_i2c_write_data(client, ATSAM_D21X_HOST_CTRL_ADDR,
					1, &host_ctrl);
	if (ret) {
		dev_err(&client->dev, "%s: failed to write to host_ctrl,\n", __func__);
		return;
	}

	ret = atsam_d21x_i2c_read_data(client, ATSAM_D21X_RAW_DATA_ADDR,
					sizeof(data->raw_data),
					data->raw_data);
	if (ret) {
		dev_err(&client->dev, "%s: failed to read touch raw data.\n",
			__func__);
	}
}

static void atsam_d21x_report_touch(struct atsam_d21x *data)
{
	struct i2c_client *client = data->client;
	u8 val[ATSAM_D21X_TOUCH_EVENT_SIZE];
	u16 x, y;
	int ret = atsam_d21x_i2c_read_data(client, 0, sizeof(val), val);

	/* If it failed to read once, just return without error handling.
	 * Sometime i2c may busy due to i2c-adapter controller times out.
	 * But it may recover very quickly so this event is just dropped.
	 */
	if (ret)
		return;

	/* little endian */
	x = val[2] | (val[3] << 8);
	y = val[4] | (val[5] << 8);

	input_report_abs(data->input_dev, ABS_X, x);
	input_report_abs(data->input_dev, ABS_Y, y);
	input_event(data->input_dev, EV_MSC, MSC_GESTURE, val[6]);
	input_event(data->input_dev, EV_MSC, MSC_RAW, (s8)val[7]);
	input_sync(data->input_dev);
}

/* MCU status is a one byte register used by host to check what status MCU
 * is currently at.
 * bit0: if buffer on bootloader if free(0: buffer not free)
 * bit1: if CRC check is OK, every time at bootup, bootloader will calculate
 *   CRC of current image, if OK, it switch to application mode. Otherwise,
 *   it stays at bootloader.
 * bit2: reset bit. This is writable by host. After host transfers all fw
 *   firmware data, it writes to this bit to 1 to reset bootloader.
 * bit3: run_app. It's added for new firmware update flow. Host driver needs
 *   to explicitly set this bit to ask bootloader to run application. Otherwise,
 *   MCU stays at bootloader forever.
 * bit7: MCU mode bit, if in bootloader, it's always 1.
 * Both bootloader and application allocate same register address for status
 * register so host can check the same register and determine current mode
 * on MCU.
 */
static int atsam_d21x_check_mcu_status(struct atsam_d21x *data, u8 *status)
{
	struct i2c_client *client = data->client;

	return atsam_d21x_i2c_read_data(client, ATSAM_D21X_MCU_STATUS_ADDR, 1,
					status);
}

/* CRC bit. Bit 1 of status register. 1: CRC ok, 0: CRC is wrong. */
static bool is_mcu_crc_ok(u8 status) {
	return status & ATSAM_D21X_MCU_CRC_MASK;
}

/* Status is read from MCU. If bit7 of status is 1, MCU is in booloader.
 * Otherwise, it's in application mode.
 */
static bool is_mcu_in_bootloader(u8 status)
{
	return status & ATSAM_D21X_MCU_MODE_MASK;
}

/* Status is read from MCU. If bit0 of status is 1, Buffer in bootloader is
 * free. This buffer is used in MCU bootloader for loading the firmware image.
 * The size of buffer is 256 bytes, so it have to take data of 256 bytes every
 * time from host via i2c. The application image is guaranteed to be a multiple
 * of 256 bytes. Every time the host will write to this same buffer, it's the
 * bootloader's responsibility to manage where the data should go in flash.
 */
static bool is_bl_buffer_free(u8 status)
{
	return status & ATSAM_D21X_BL_BUF_MASK;
}

static int atsam_d21x_write_firmware(struct device *dev,
				const struct firmware *fw)
{
	int bytes_remaining = fw->size;
	int pos = 0;
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct atsam_d21x *data = i2c_get_clientdata(client);

	while (bytes_remaining) {
		u8* buf = fw->data + pos;
		u8 status = 0;
		int xfer_size = min(ATSAM_D21X_FW_FRAME_SIZE, bytes_remaining);
		int retry;

		/* Wait for the buffer to become available before next write. */
		for (retry = 0; retry < NUM_OF_RETRIES; ++retry) {
			ret = atsam_d21x_check_mcu_status(data, &status);
			if (ret)
				continue;
			/* bootloader buffer is free */
			if (is_bl_buffer_free(status))
				break;
			msleep(10);
		}
		if (retry == NUM_OF_RETRIES) {
			dev_err(dev, "%s: buffer on MCU not free.\n", __func__);
			ret = -1;
			goto exit;
		}

		for (retry = 0; retry < NUM_OF_RETRIES; ++retry) {
			ret = atsam_d21x_i2c_write_data(client,
							ATSAM_D21X_FLASH_ADDR,
							xfer_size, buf);
			if (!ret) {
				break;
			} else {
				dev_err(dev, "%s: write fw failed, retry=%d\n",
					__func__, retry);
			}
		}

		if (retry == NUM_OF_RETRIES) {
			dev_err(dev, "%s: failed to write fw, aborting.\n",
				__func__);
			goto exit;
		}

		pos += xfer_size;
		bytes_remaining -= xfer_size;
	}
exit:
	return ret;
}

static int atsam_d21x_wait_for_bootloader(struct atsam_d21x *data)
{
	/* cmd used to ask application to reboot to bootloader */
	const u8 cmd = 0x01;
	u8 status = 0;
	struct i2c_client *client = data->client;
	int ret;
	int retry;

	for (retry = 0; retry < NUM_OF_RETRIES; ++retry) {
		ret = atsam_d21x_check_mcu_status(data, &status);
		if (ret) {
			dev_err(&client->dev, "%s: failed to get mcu mode. "
				"retry=%d\n", __func__, retry);
			continue;
		}

		if (is_mcu_in_bootloader(status))
			break;

		/* If MCU not is bootloader, it's in application mode.
		 * Need to send command to application asking it to boot into
		 * bootloader. Set bit0 of ATSAM_D21X_APPLICATION_CMD_ADDR to 1
		 * will reboot the MCU to bootloader.
		 */
		ret = atsam_d21x_i2c_write_data(client,
					ATSAM_D21X_APPLICATION_CMD_ADDR,
					1, &cmd);
		if (ret) {
			dev_err(&client->dev,
				"%s: failed to write cmd to application. "
				"retry=%d\n", __func__, retry);
			continue;
		}
		/* Wait for MCU to boot into bootloader. */
		msleep(20);
	}

	if (retry == NUM_OF_RETRIES)
		ret = -1;

	return ret;
}

static int atsam_d21x_wait_for_application(struct atsam_d21x *data) {
	/* Command used to ask bootloader to run application. */
	const u8 cmd = 0x08;
	u8 status = 0;
	struct i2c_client *client = data->client;
	int ret;
	int retry;

	for (retry = 0; retry < NUM_OF_RETRIES; ++retry) {
		/* Usually this function is called after reset or firmware
		 * update. Wait for some time before MCU is stabilized after
		 * state change.
		 */
		msleep(80);
		ret = atsam_d21x_check_mcu_status(data, &status);
		if (ret) {
			dev_err(&client->dev, "%s: failed to get mcu mode. "
				"retry=%d\n", __func__, retry);
			continue;
		}

		/* MCU is already in application mode.
		 * This applies to firmware update flow 1, where bootloader
		 * automatically runs application after boot up.
		 */
		if (!is_mcu_in_bootloader(status)) {
			dev_info(&client->dev,
				"%s: MCU already in application mode.\n",
				__func__);
			break;
		}

		/* TODO(mengyu): decide if anything needs to be done instead of
		 * return directly.
		 */
		if (!is_mcu_crc_ok(status)) {
			dev_err(&client->dev,
				"%s: CRC check failed. status=0x%02X\n",
				__func__, status);
			ret = -1;
			break;
		}

		/* Old bootloader should not reach this line, it should break in
		 * if(!is_mcu_in_bootloader()) at first try.
		 */
		dev_info(&client->dev, "%s: This device has new bootloader."
			" Writing to MCU to run application code.\n",
			__func__);
		/* Write command to status register to run application. */
		ret = atsam_d21x_i2c_write_data(client,
					ATSAM_D21X_MCU_STATUS_ADDR,
					1, &cmd);
		if (ret) {
			dev_err(&client->dev,
				"%s: failed to write cmd to run_app bit.\n",
				__func__);
			continue;
		}
	}

	if (retry == NUM_OF_RETRIES)
		ret = -1;

	return ret;
}

static int atsam_d21x_firmware_update(struct device *dev,
				const struct firmware *fw)
{
	/* Command sent to bootloader status register to reset MCU.
	 * bit2 is used for reset, writing to other bits of register has no
	 * effects. It's safe to write 0x04 to status register.
	 */
	const u8 cmd = 0x04;
	struct i2c_client *client = to_i2c_client(dev);
	struct atsam_d21x *data = i2c_get_clientdata(client);
	int ret = gpio_request(data->reset, "TouchRST");

	/* Reset gpio is held by other devices, abort firmware upgrade to
	 * prevent updating from being interrupted by a reset.
	 */
	if (ret) {
		dev_err(dev, "%s: failed to request GPIO%d.\n",
			__func__, data->reset);
		return ret;
	}

	ret = atsam_d21x_wait_for_bootloader(data);

	if (ret) {
		dev_err(dev, "%s: failed to get into bootloader, aborting.\n",
			__func__);
		goto exit_release_gpio;
	}

	if (!fw->data || fw->size < ATSAM_D21X_MINIMUM_FW_SIZE) {
		dev_err(dev, "%s: fw is not valid.\n", __func__);
		goto exit_release_gpio;
	}

	ret = atsam_d21x_write_firmware(dev, fw);
	if (ret) {
		dev_err(dev, "%s: failed to write firmware to MCU.\n",
			__func__);
		goto exit_release_gpio;
	}

	/* Send command to reset bootloader. */
	ret = atsam_d21x_i2c_write_data(client, ATSAM_D21X_MCU_STATUS_ADDR, 1,
					&cmd);
	if (ret) {
		dev_err(dev, "%s: failed to reset bootloader.\n ", __func__);
		goto exit_release_gpio;
	}

	/* Wait for bootloader to reset. */
	msleep(1);

	dev_info(dev, "%s: firmware_upgrade successfully\n", __func__);

exit_release_gpio:
	gpio_free(data->reset);
	return ret;
}

static irqreturn_t atsam_d21x_interrupt(int irq, void *handle)
{
	struct atsam_d21x *data = handle;

	/* Factory image uses odd firmware version starting from 0x01 and
	 * application image uses even numbers starting from 0x02.
	 * 0 is not a valid firmware version.
	 */
	/* Factory image. */
	if (data->fw_version % 2) {
		atsam_d21x_report_raw_data(data);
	} else if (data->fw_version != 0) {  /* Application image. */
		atsam_d21x_report_touch(data);
	} else {
		dev_err(&data->client->dev, "%s: invalid fw version=0x%02X.\n",
			__func__, data->fw_version);
	}

	return IRQ_HANDLED;
}

static int atsam_d21x_get_mcu_fw_version(struct i2c_client *client, u8 *version)
{
	return atsam_d21x_i2c_read_data(client, ATSAM_D21X_FW_VERSION_ADDR,
					1, version);
}

static void atsam_d21x_fw_request_handler(const struct firmware *fw,
						void *context) {
	struct device *dev = context;
	struct i2c_client *client = to_i2c_client(dev);
	struct atsam_d21x *data = i2c_get_clientdata(client);
	u8 fw_version_mcu;
	u8 fw_version_host;
	int ret;

	if (!fw) {
		dev_err(dev, "%s: fw not found.\n", __func__);
		goto exit;
	}

	if (!fw->data || fw->size < ATSAM_D21X_MINIMUM_FW_SIZE) {
		dev_err(dev, "%s: fw is not valid.\n", __func__);
		goto exit_release_fw;
	}

	ret = atsam_d21x_get_mcu_fw_version(client, &fw_version_mcu);
	if (ret) {
		dev_err(dev, "%s: failed to get current fw version.\n",
			__func__);
		goto exit_release_fw;
	}
	fw_version_host = *(fw->data + fw->size - ATSAM_D21X_MINIMUM_FW_SIZE);

	dev_info(dev, "%s: fw_version_host=0x%02X, fw_version_mcu=0x%02X\n",
		__func__, fw_version_host, fw_version_mcu);
	if (fw_version_host == fw_version_mcu) {
		dev_info(dev, "%s: host fw is the same as current one, "
			"no firmware update is needed.\n", __func__);
		goto exit_wait_for_application;
	}

	dev_info(dev, "%s: will update firmware soon...\n", __func__);

	ret = atsam_d21x_firmware_update(dev, fw);
	if (ret) {
		dev_err(dev, "%s: failed to update firmware.\n", __func__);
		goto exit_wait_for_application;
	}

	data->fw_version = fw_version_host;

exit_wait_for_application:
	ret = atsam_d21x_wait_for_application(data);
	if (ret)
		dev_err(dev, "%s: failed to run application.\n", __func__);

exit_release_fw:
	release_firmware(fw);
exit:
	atomic_set(&data->fw_updating, 0);
}

/* Called by atsam_d21x_probe.
 * This function doesn't have return value because probe shouldn't fail if
 * 1. I2C bus is busy at bootup (unlikely though).
 * 2. Firmware file doesn't exist.
 */
static void update_fw_if_needed(struct atsam_d21x *data)
{
	struct i2c_client *client = data->client;
	int ret;

	if (atomic_xchg(&data->fw_updating, 1)) {
		dev_err(&client->dev, "%s: fw is busy.\n", __func__);
		return;
	}

	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
					ATSAM_D21X_APPLICATION_FIRMWARE,
					&client->dev, GFP_KERNEL, &client->dev,
					atsam_d21x_fw_request_handler);
	if (ret) {
		dev_err(&client->dev, "%s: failed to request firmware.\n",
			__func__);
	}
}

static int atsam_d21x_reset_mcu(struct atsam_d21x *data)
{
	struct i2c_client *client = data->client;
	int ret = gpio_request(data->reset, "TouchRST");
	if (ret) {
		dev_err(&client->dev,
			"%s: failed to request GPIO%d.\n",
			__func__, data->reset);
		return ret;
	}

	ret = gpio_direction_output(data->reset, 1);
	if (ret) {
		dev_err(&client->dev,
			"%s: failed to set set gpio to 1.\n", __func__);
		gpio_free(data->reset);
		return ret;
	}
	gpio_set_value(data->reset, 0);
	msleep(1);
	gpio_set_value(data->reset, 1);

	gpio_free(data->reset);

	ret = atsam_d21x_wait_for_application(data);
	if (ret) {
		dev_err(&client->dev,
			"%s: failed to run application after reset.\n",
			__func__);
	}

	return ret;
}

static ssize_t atsam_d21x_reset_mcu_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct atsam_d21x *data = i2c_get_clientdata(client);
	int ret;

	dev_info(dev, "%s: resetting MCU...\n", __func__);
	ret = atsam_d21x_reset_mcu(data);
	if (ret) {
		dev_err(dev, "%s: failed to reset MCU. ret=%d\n",
			__func__, ret);
		return ret;
	}

	return count;
}

static ssize_t atsam_d21x_do_fw_update_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
			"Usage:\n"
			"  write 1 to load the application firmware.\n"
			"  write 2 to load the factory firmware.\n");
}

/* This function will do a firmware update without version checking.
 * It can be called when we need to force update in recovery state.
 * Write 1 to it will load the application firmware and write 2 to it will
 * load factory firmware.
 */
static ssize_t atsam_d21x_do_fw_update_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct atsam_d21x *data = i2c_get_clientdata(client);
	const struct firmware *fw;
	char *fw_name;
	int input;
	int ret;

	/* Someone else has the lock, cannot do fw updating now. */
	if (atomic_xchg(&data->fw_updating, 1))
		return -EBUSY;

	disable_irq(data->irq);
	ret = kstrtouint(buf, 10, &input);
	if (ret)
		goto exit;

	if (input == 1) {
		dev_info(dev, "%s: loading application firmware.\n", __func__);
		fw_name = ATSAM_D21X_APPLICATION_FIRMWARE;
	} else if (input == 2) {
		dev_info(dev, "%s: loading factory firmware.\n", __func__);
		fw_name = ATSAM_D21X_FACTORY_FIRMWARE;
	} else {
		ret = -EINVAL;
		dev_err(dev, "%s: Invalid input.\n", __func__);
		goto exit;
	}

	dev_info(dev, "%s: will do firmware update.\n", __func__);

	ret = request_firmware(&fw, fw_name, dev);
	if (ret) {
		dev_err(dev, "%s: unable to open firmware %s\n", __func__,
			fw_name);
		goto exit;
	}

	if (fw->size < ATSAM_D21X_MINIMUM_FW_SIZE) {
		dev_err(dev, "%s: invalid firmware size.\n", __func__);
		goto exit_release_fw;
	}

	ret = atsam_d21x_firmware_update(dev, fw);
	if (ret) {
		/* Didn't goto exit_release_fw here but instead waiting for
		 * application to run so that previously application still can
		 * work if firmware update failed at some point which doesn't
		 * mess up with the application image.
		 */
		dev_err(dev, "%s: failed to update firmware\n", __func__);
	} else {
		data->fw_version = *(fw->data + fw->size
					- ATSAM_D21X_MINIMUM_FW_SIZE);
	}

	ret = atsam_d21x_wait_for_application(data);
	if (ret) {
		dev_err(dev, "%s: failed to run application.\n", __func__);
		goto exit_release_fw;
	}

	ret = count;

exit_release_fw:
	release_firmware(fw);
exit:
	atomic_set(&data->fw_updating, 0);
	enable_irq(data->irq);
	return ret;
}

static ssize_t atsam_d21x_fw_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct atsam_d21x *data = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "firmware version: 0x%02X\n",
			data->fw_version);
}

static ssize_t atsam_d21x_raw_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct atsam_d21x *data = dev_get_drvdata(dev);
	int i;
	char formatted_data[1024];
	size_t offset = 0;
	size_t remain = sizeof(formatted_data);

	/* little endian */
	for (i = 0; i < ATSAM_D21X_RAW_DATA_NUM; ++i) {
		int printed = scnprintf(formatted_data + offset, remain, "0x%02X%02X ",
						data->raw_data[i * 2 + 1],
						data->raw_data[i * 2]);
		offset += printed;
		remain -= printed;
	}

	return scnprintf(buf, PAGE_SIZE, "%s\n", formatted_data);
}

static ssize_t atsam_d21x_raw_data_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct atsam_d21x *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	/* When need to grab touch raw data, 0x02 should be write to host_ctrl
	 * register.
	 */
	const u8 host_ctrl = 0x02;
	int input;
	int ret = kstrtouint(buf, 10, &input);

	if (ret) {
		dev_err(dev, "%s: Invalid input.\n", __func__);
		return -EINVAL;
	}

	if (input != 1) {
		dev_err(dev, "%s: Invalid input.\n", __func__);
		return -EINVAL;
	}

	ret = atsam_d21x_i2c_write_data(client, ATSAM_D21X_HOST_CTRL_ADDR,
					1, &host_ctrl);
	if (ret) {
		dev_err(dev, "%s: failed to write to host_ctrl,\n", __func__);
		return -EIO;
	}

	return count;
}

/* Check if I2C is functional or not.
 * Return 1 if I2C is working, 0 otherwise.
 */
static ssize_t atsam_d21x_i2c_ok_show(struct device *dev,
		struct device_attribute *attr, const char *buf)
{
	struct atsam_d21x *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	u8 val;
	int ret = atsam_d21x_i2c_read_data(client, ATSAM_D21X_FW_VERSION_ADDR,
						1, &val);
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret == 0 ? 1 : 0);
}

static DEVICE_ATTR(do_fw_update, S_IWUSR | S_IWGRP | S_IRUGO,
		atsam_d21x_do_fw_update_show,
		atsam_d21x_do_fw_update_store);
static DEVICE_ATTR(fw_version, S_IRUGO, atsam_d21x_fw_version_show, NULL);
static DEVICE_ATTR(i2c_ok, S_IRUGO, atsam_d21x_i2c_ok_show, NULL);
static DEVICE_ATTR(raw_data, S_IWUSR | S_IWGRP | S_IRUGO,
		atsam_d21x_raw_data_show,
		atsam_d21x_raw_data_store);
static DEVICE_ATTR(reset_mcu, S_IWUSR | S_IWGRP | S_IRUGO,
		NULL, atsam_d21x_reset_mcu_store);

static struct attribute *atsam_d21x_attrs[] = {
	&dev_attr_do_fw_update.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_i2c_ok.attr,
	&dev_attr_raw_data.attr,
	&dev_attr_reset_mcu.attr,
	NULL
};

static const struct attribute_group atsam_d21x_attr_group = {
	.attrs = atsam_d21x_attrs,
};

#ifdef CONFIG_OF
static struct atsam_d21x_platform_data *atsam_d21x_parse_devtree(
					struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *node;
	struct atsam_d21x_platform_data *pdata;
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

	/* Allocate INT gpio. */
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

	/* Allocate reset gpio. */
	reset_gpio = of_get_named_gpio(node, "reset_gpio", 0);
	if (!gpio_is_valid(reset_gpio))
		goto exit;

	pdata->reset_gpio = reset_gpio;

	if (of_property_read_u32(node, "max_x", &pdata->max_x)) {
		dev_err(dev, "Failed to get the touch screen x size.\n");
		return ERR_PTR(-EINVAL);
	}

	if (of_property_read_u32(node, "max_y", &pdata->max_y)) {
		dev_err(dev, "Failed to get the touch screen y size.\n");
		return ERR_PTR(-EINVAL);
	}

	return pdata;

exit:
	if (touch_gpio != -1)
		gpio_free(touch_gpio);
	dev_err(dev, "%s: Failed to request gpio\n", __func__);
	return -ENODEV;
}
#endif

static void atsam_d21x_touch_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct atsam_d21x *data = container_of(dwork, struct atsam_d21x,
						touch_work);
	struct i2c_client *client = data->client;
	u8 status = 0;
	int ret;

	if (atomic_read(&data->fw_updating)) /* In middle of fw updating. */
		goto exit_rearm_work;

	ret = atsam_d21x_check_mcu_status(data, &status);
	if (ret) {
		dev_err(&client->dev, "%s: failed to get mcu mode. ret=%d\n",
			__func__, ret);
		goto exit_rearm_work;
	}

	/* In new firmware update flow, MCU always stays at bootloader after
	 * reset. In case any external devices like LED driver toggle the reset
	 * pin, MCU is forced to switch to bootloader again, we need to
	 * periodically check whether MCU is in bootloader or not.
	 */
	if (is_mcu_in_bootloader(status)) {
		ret = atsam_d21x_wait_for_application(data);
		if (ret) {
			dev_err(&client->dev,
				"%s: failed to exit from bootloader. ret=%d\n",
				__func__, ret);
			goto exit_rearm_work;
		}
	}

	/* Explicitly get an updated firmware version in case
	 * data->fw_version got wrong value during firmware update or reset.
	 */
	ret = atsam_d21x_get_mcu_fw_version(client, &data->fw_version);
	if (ret) {
		dev_err(&client->dev,
			"%s: failed to get fw version.\n", __func__);
	}

exit_rearm_work:
	/* Rearm workqueue itself. */
	queue_delayed_work(data->touch_workqueue, &data->touch_work,
			msecs_to_jiffies(ATSAM_D21X_WORK_INTERVAL_MS));
}

static int atsam_d21x_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct atsam_d21x *data;
	struct device *dev = &client->dev;
	struct atsam_d21x_platform_data *pdata = dev_get_platdata(dev);
	struct input_dev *input_dev;
	int error = 0;

	if (!pdata) {
		pdata = atsam_d21x_parse_devtree(client);
		if (IS_ERR(pdata)) {
			dev_err(dev,
			"%s: failed to get device data from device tree.\n",
			__func__);
			error = -EINVAL;
			goto err_get_pdata;
		}
	}

	data = kzalloc(sizeof(struct atsam_d21x), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(&client->dev, "%s: failed to allocate memory.\n",
			__func__);
		error = -ENOMEM;
		goto err_free_mem;
	}

	input_dev->name = "Atmel SAM D21X";
	snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
		client->adapter->nr, client->addr);
	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	data->client = client;
	data->input_dev = input_dev;
	data->pdata = pdata;
	data->irq = client->irq;
	data->reset = pdata->reset_gpio;
	atomic_set(&data->fw_updating, 0);

	memset(data->raw_data, 0, sizeof(data->raw_data));

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_MSC, input_dev->evbit);

	input_set_capability(input_dev, EV_MSC, MSC_GESTURE);
	/* MSC_RAW is used for rotation_speed, represented by ticks. */
	input_set_capability(input_dev, EV_MSC, MSC_RAW);

	input_set_abs_params(input_dev, ABS_X,
			0, pdata->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			0, pdata->max_y, 0, 0);

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	error = atsam_d21x_get_mcu_fw_version(client, &data->fw_version);
	if (error) {
		dev_err(&client->dev, "%s: failed to get fw version.\n",
			__func__);
		data->fw_version = 0;
	}

	update_fw_if_needed(data);

	error = request_threaded_irq(pdata->irq, NULL, atsam_d21x_interrupt,
					IRQF_ONESHOT | IRQF_TRIGGER_LOW,
					client->name, data);

	if (error) {
		dev_err(&client->dev, "%s: failed to request IRQ %d, err: %d\n",
			__func__, data->irq, error);
		goto err_free_mem;
	}

	error = input_register_device(input_dev);
	if (error) {
		dev_err(&client->dev,
			"%s: failed to register input_dev, err:%d\n",
			__func__, error);
		goto err_free_irq;
	}

	error = sysfs_create_group(&client->dev.kobj, &atsam_d21x_attr_group);
	if (error) {
		dev_err(&client->dev, "%s: failed to create sysfs, err:%d\n",
			__func__, error);
		goto err_unregister_device;
	}

	data->touch_workqueue =
			create_singlethread_workqueue("touch_workqueue");
	if (data->touch_workqueue == NULL)
		return -ENOMEM;
	INIT_DELAYED_WORK(&data->touch_work, atsam_d21x_touch_work);
	queue_delayed_work(data->touch_workqueue, &data->touch_work,
			msecs_to_jiffies(ATSAM_D21X_WORK_INTERVAL_MS));

	dev_info(&client->dev, "%s: probe successfully!\n", __func__);

	return 0;

err_unregister_device:
	input_unregister_device(input_dev);
	input_dev = NULL;
err_free_irq:
	free_irq(client->irq, data);
err_free_mem:
	input_free_device(input_dev);
	kfree(data);
err_get_pdata:
	return error;
}

static int atsam_d21x_remove(struct i2c_client *client)
{
	struct atsam_d21x *data = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&data->touch_work);
	flush_workqueue(data->touch_workqueue);
	destroy_workqueue(data->touch_workqueue);
	sysfs_remove_group(&client->dev.kobj, &atsam_d21x_attr_group);
	free_irq(data->irq, data);
	input_unregister_device(data->input_dev);
	kfree(data);

	return 0;
}

static const struct i2c_device_id atsam_d21x_id[] = {
	{ "atmel-sam-d21x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, atsam_d21x_id);

#ifdef CONFIG_OF
static const struct of_device_id atsam_d21x_of_match[] = {
	{ .compatible = "atmel-sam-d21x" },
	{ }
};
#endif

static struct i2c_driver atsam_d21x_driver = {
	.driver = {
		.name    = "atmel-sam-d21x",
		.owner   = THIS_MODULE,
		.pm      = NULL,
		.of_match_table = of_match_ptr(atsam_d21x_of_match),
	},
	.probe           = atsam_d21x_probe,
	.remove          = atsam_d21x_remove,
	.id_table        = atsam_d21x_id,
};

module_i2c_driver(atsam_d21x_driver);

MODULE_AUTHOR("Mengyu Yuan <mengyu@google.com>");
MODULE_DESCRIPTION("Atmel SAM D21X Touchscreen driver");
MODULE_LICENSE("GPL");
