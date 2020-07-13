/*************************************************************************************
*       Copyright (C) 2007-2011
*       Copyright ? 2007 Marvell International Ltd.
*
*       This program is free software; you can redistribute it and/or
*       modify it under the terms of the GNU General Public License
*       as published by the Free Software Foundation; either version 2
*       of the License, or (at your option) any later version.
*
*       This program is distributed in the hope that it will be useful,
*       but WITHOUT ANY WARRANTY; without even the implied warranty of
*       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*       GNU General Public License for more details.
*
*       You should have received a copy of the GNU General Public License
*       along with this program; if not, write to the Free Software
*       Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*
***************************************************************************************/
#include "pcm.h"

#include <asm/cacheflush.h>
#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/file.h>
#if defined(CONFIG_SND_SOC_BERLIN_HW_VOL_CTRL)
#include <linux/i2c.h>
#endif
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/socket.h>
#include <linux/time.h>
#include <linux/version.h>
#include <linux/wait.h>

#include <sound/core.h>
#include <sound/control.h>
#include <sound/hwdep.h>
#include <sound/info.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/pcm-indirect.h>
#include <sound/rawmidi.h>
#if defined(CONFIG_SND_SOC_BERLIN_HW_VOL_CTRL)
#include <sound/tlv.h>
#endif

#include "berlin_memmap.h"
#include "api_avio_dhub.h"
#include "api_dhub.h"
#include "api_avpll.h"
#include "api_playback.h"
#include "api_capture.h"

#define IRQ_GIC_START          (32)
#define IRQ_dHubIntrAvio1      (0x22)
#define IRQ_DHUBINTRAVIO1      (IRQ_GIC_START + IRQ_dHubIntrAvio1)

#if defined(CONFIG_SND_SOC_BERLIN_HW_VOL_CTRL)
#define TAS5720_SADDR_VOLUME 0x04

#define TAS5720_SADDR_POWER 0x01
#define TAS5720_POWER_SDZ 0x01
#define TAS5720_POWER_SHUTDOWN 0xFC
#define TAS5720_POWER_RESET 0xFD

#define TAS5720_SADDR_FAULT 0x08
#define TAS5720_FAULT_OTE (1 << 0)
#define TAS5720_FAULT_DCE (1 << 1)
#define TAS5720_FAULT_OCE (1 << 2)
#define TAS5720_FAULT_CLKE (1 << 3)

const char kTas5720SysfsShutdown[] = "SHUTDOWN";

#endif

enum {
        SNDRV_BERLIN_GET_OUTPUT_MODE = 0x1001,
        SNDRV_BERLIN_SET_OUTPUT_MODE,
        SNDRV_BERLIN_GET_CLOCK_PPM,
        SNDRV_BERLIN_SET_CLOCK_PPM,
};

#if defined(CONFIG_SND_SOC_BERLIN_HW_VOL_CTRL)
static struct i2c_client *tas5720_client;
static struct i2c_board_info tas5720_hwmon_info = {
	I2C_BOARD_INFO("tas5720", 0x6c),
};
#endif

static struct snd_card *snd_berlin_card;

extern atomic_t g_output_mode;

static irqreturn_t berlin_devices_aout_isr(int irq, void *dev_id)
{
	struct snd_pcm *pcm = (struct snd_pcm*)dev_id;
	HDL_semaphore *pSemHandle = dhub_semaphore(&AG_dhubHandle.dhub);
	unsigned int instat = semaphore_chk_full(pSemHandle, -1);
	struct snd_pcm_substream *substream;
	unsigned int chanId;

	substream = pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
	chanId = CAP_ID;
	if (instat & BIT(chanId)) {
		semaphore_pop(pSemHandle, chanId, 1);
		semaphore_clr_full(pSemHandle, chanId);
		if (substream && substream->runtime)
			snd_berlin_capture_isr(substream);
	}

	substream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	chanId = avioDhubChMap_ag_SA0_R_A0;
	if (instat & BIT(chanId)) {
		semaphore_pop(pSemHandle, chanId, 1);
		semaphore_clr_full(pSemHandle, chanId);
		if (substream && substream->runtime)
			snd_berlin_playback_isr(substream);
	}

	return IRQ_HANDLED;
}

static struct snd_pcm_ops snd_berlin_playback_ops = {
	.open    = snd_berlin_playback_open,
	.close   = snd_berlin_playback_close,
	.ioctl   = snd_pcm_lib_ioctl,
	.hw_params = snd_berlin_playback_hw_params,
	.hw_free   = snd_berlin_playback_hw_free,
	.prepare = snd_berlin_playback_prepare,
	.trigger = snd_berlin_playback_trigger,
	.pointer = snd_berlin_playback_pointer,
	.ack     = snd_berlin_playback_ack,
};

static struct snd_pcm_ops snd_berlin_capture_ops = {
	.open      = snd_berlin_capture_open,
	.close     = snd_berlin_capture_close,
	.ioctl     = snd_pcm_lib_ioctl,
	.hw_params = snd_berlin_capture_hw_params,
	.hw_free   = snd_berlin_capture_hw_free,
	.prepare   = snd_berlin_capture_prepare,
	.trigger   = snd_berlin_capture_trigger,
	.pointer   = snd_berlin_capture_pointer,
};

static int snd_berlin_rate_offset_control_info(
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = -500;
	uinfo->value.integer.max = 500;
	uinfo->value.integer.step = 1;
	return 0;
}

static int double_to_int(double ppm)
{
	return ppm >= 0 ? ppm + 0.5: ppm - 0.5;
}

static int snd_berlin_rate_offset_control_get(
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	double ppm_base, ppm_now;
	AVPLL_GetPPM(&ppm_base, &ppm_now);
	ucontrol->value.integer.value[0] = double_to_int(ppm_now - ppm_base);
	return 0;
}

static int snd_berlin_rate_offset_control_put(
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	double ppm_base, ppm_now;
	int ppm, current_ppm;
	ppm = ucontrol->value.integer.value[0];
	if ((ppm < -500) || (ppm > 500))
		return -1;
	AVPLL_GetPPM(&ppm_base, &ppm_now);
	current_ppm = double_to_int(ppm_base - ppm_now);
	if (ppm != current_ppm) {
		AVPLL_AdjustPPM(ppm_base - ppm_now + ppm);
		return 1;
	}
	return 0;
}

static struct snd_kcontrol_new snd_berlin_rate_offset_control = {
	.iface = SNDRV_CTL_ELEM_IFACE_PCM,
	.name = "PCM Playback Rate Offset",
	.index = 0,
	.device = 0,
	.subdevice = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.private_value = 0xffff,
	.info = snd_berlin_rate_offset_control_info,
	.get = snd_berlin_rate_offset_control_get,
	.put = snd_berlin_rate_offset_control_put
};

#if defined(CONFIG_SND_SOC_BERLIN_HW_VOL_CTRL)
static int snd_berlin_volume_control_info(
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0x00;
	uinfo->value.integer.max = 0xff;
	uinfo->value.integer.step = 1;
	return 0;
}

static int snd_berlin_volume_control_get(
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int volume_or_error = i2c_smbus_read_byte_data(
		tas5720_client, TAS5720_SADDR_VOLUME);
	if (volume_or_error < 0) {
		snd_printk("i2c read error (%d): %s\n",
                           volume_or_error, __func__);
		return volume_or_error;
	}
	ucontrol->value.integer.value[0] = volume_or_error;
	kcontrol->private_value = volume_or_error;
	return 0;
}

static int snd_berlin_volume_control_put(
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int new_volume = ucontrol->value.integer.value[0];
	if (kcontrol->private_value != new_volume &&
		0 <= new_volume && new_volume <= 0xff) {
		int ret = i2c_smbus_write_byte_data(
			tas5720_client, TAS5720_SADDR_VOLUME, new_volume);
		if (ret < 0) {
			snd_printk("i2c write error (%d): %s\n",
                                   ret, __func__);
			return ret;
		}
		kcontrol->private_value = new_volume;
		return 1;
	}
	return 0;
}

static DECLARE_TLV_DB_SCALE(db_scale_volume_control, -10350, 50, 1);
static struct snd_kcontrol_new snd_berlin_volume_control = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "PCM Playback Volume",
	.index = 0,
	.device = 0,
	.subdevice = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
		SNDRV_CTL_ELEM_ACCESS_TLV_READ,
	.private_value = 0xffff,
	.info = snd_berlin_volume_control_info,
	.get = snd_berlin_volume_control_get,
	.put = snd_berlin_volume_control_put,
	.tlv.p = db_scale_volume_control
};

static int tas5720_fault_get() {
	int data_or_error = i2c_smbus_read_byte_data(tas5720_client,
			TAS5720_SADDR_FAULT);
	if (data_or_error < 0) {
		snd_printk("Error reading FAULT: %d", data_or_error);
	}
	return data_or_error;
}

static int tas5720_fault_set(u8 data) {
	int ret;
	ret = i2c_smbus_write_byte_data(tas5720_client, TAS5720_SADDR_FAULT,
			data);
	if (ret < 0) {
		snd_printk("Error writing 0x%X to FAULT: %d", data, ret);
	}
	return ret;
}

static int tas5720_power_get() {
	int data_or_error = i2c_smbus_read_byte_data(tas5720_client, TAS5720_SADDR_POWER);
	if (data_or_error < 0) {
		snd_printk("Error reading POWER: %d", data_or_error);
		return data_or_error;
	}
	return data_or_error & TAS5720_POWER_SDZ;
}

static int tas5720_power_set(bool amplifier_on) {
	int ret;
	u8 data = amplifier_on ? TAS5720_POWER_RESET : TAS5720_POWER_SHUTDOWN;
	if (!amplifier_on) {
		// Always clear the fault register.
		// Experimentally, OCE won't clear without writing 0 to OCE_THRESH.
		ret = tas5720_fault_set(0x00);
		if (ret < 0) {
			return ret;
		}
	}
	ret = i2c_smbus_write_byte_data(tas5720_client, TAS5720_SADDR_POWER,
	                                data);
	if (ret < 0) {
		snd_printk("Error writing 0x%X to POWER: %d", data, ret);
	}
	return ret;
}

static int snd_berlin_mute_control_info(
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int snd_berlin_mute_control_get(
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int mute_or_error = tas5720_power_get();
	if (mute_or_error < 0) {
		return mute_or_error;
	}
	ucontrol->value.integer.value[0] = mute_or_error;
	kcontrol->private_value = mute_or_error;
	return 0;
}

static int snd_berlin_mute_control_put(
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	bool new_mute = !!ucontrol->value.integer.value[0];
	if (kcontrol->private_value != new_mute) {
		int ret = tas5720_power_set(new_mute);
		if (ret < 0) {
			return ret;
		}
		kcontrol->private_value = new_mute;
		return 1;
	}
	return 0;
}

static struct snd_kcontrol_new snd_berlin_mute_control = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "PCM Playback Switch",
	.index = 0,
	.device = 0,
	.subdevice = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.private_value = 1,
	.info = snd_berlin_mute_control_info,
	.get = snd_berlin_mute_control_get,
	.put = snd_berlin_mute_control_put,
};

static int snd_berlin_fault_control_info(
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0xFF;
	return 0;
}

static int snd_berlin_fault_control_get(
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int fault_or_error = tas5720_fault_get();
	if (fault_or_error < 0) {
		return fault_or_error;
	}
	ucontrol->value.integer.value[0] = fault_or_error;
	kcontrol->private_value = fault_or_error;
	return 0;
}

static int snd_berlin_fault_control_put(
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.integer.value[0];
	/* datasheet stipulates that 2 most significant bits must be 0 when writing. */
	if (kcontrol->private_value != value && value == (value & 0x3F)) {
		int ret = tas5720_power_set(value);
		if (ret < 0) {
			return ret;
		}
		kcontrol->private_value = value;
		return 1;
	}
	return 0;
}

static struct snd_kcontrol_new snd_berlin_fault_control = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "TAS5720 Fault",
	.index = 0,
	.device = 0,
	.subdevice = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
			SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	.private_value = 0x00,
	.info = snd_berlin_fault_control_info,
	.get = snd_berlin_fault_control_get,
	.put = snd_berlin_fault_control_put,
};

#endif

static int snd_berlin_card_new_pcm(struct snd_berlin_chip *chip)
{
	struct snd_pcm *pcm;
	int err;

	if ((err =
	snd_pcm_new(chip->card, "Berlin ALSA PCM", 0, 1, 1, &pcm)) < 0) {
                snd_printk("error creating PCM instance (%d): %s\n",
                           err, __func__);
		return err;
	}
	err = snd_ctl_add(chip->card,
			snd_ctl_new1(&snd_berlin_rate_offset_control, chip));
	if (err < 0) {
		snd_printk("error adding rate offset control (%d): %s\n",
                           err, __func__);
		return err;
	}
#if defined(CONFIG_SND_SOC_BERLIN_HW_VOL_CTRL)
	err = snd_ctl_add(chip->card,
			snd_ctl_new1(&snd_berlin_volume_control, chip));
	if (err < 0) {
		snd_printk("error adding volume control (%d): %s\n",
                           err, __func__);
		return err;
	}
	err = snd_ctl_add(chip->card,
			snd_ctl_new1(&snd_berlin_mute_control, chip));
	if (err < 0) {
		snd_printk("error adding mute control (%d): %s\n",
                           err, __func__);
		return err;
	}
	err = snd_ctl_add(chip->card,
			snd_ctl_new1(&snd_berlin_fault_control, chip));
	if (err < 0) {
		snd_printk("error adding fault control (%d): %s\n",
                           err, __func__);
		return err;
	}
#endif
	chip->pcm = pcm;
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
			&snd_berlin_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
			&snd_berlin_capture_ops);

	pcm->private_data = chip;
	pcm->info_flags = 0;
	strcpy(pcm->name, "Berlin ALSA PCM");
	/* Make sure size >= MAX_BUFFER_SIZE in both playback.c and
	   PCM_MAX_BUFFER_SIZE in capture.c. */
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
					snd_dma_continuous_data(GFP_KERNEL), 32*1024,
					64*1024);
	return 0;
}

static int snd_berlin_hwdep_dummy_op(struct snd_hwdep *hw, struct file *file)
{
	return 0;
}

static int snd_berlin_hwdep_ioctl(struct snd_hwdep *hw, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	switch(cmd) {
		case SNDRV_BERLIN_GET_OUTPUT_MODE: {
			unsigned int mode = atomic_read(&g_output_mode);
			int bytes_not_copied =
				copy_to_user((void*)arg, &mode, sizeof(mode));
			return bytes_not_copied == 0 ? 0 : -EFAULT;
		}
		case SNDRV_BERLIN_SET_OUTPUT_MODE: {
			unsigned int mode;
			int bytes_not_copied =
				copy_from_user(&mode, (void*)arg, sizeof(mode));
			if (bytes_not_copied)
				return -EFAULT;
			atomic_set(&g_output_mode, mode);
			return 0;
		}
		default:
			return -EINVAL;
	}
}

static int snd_berlin_card_new_hwdep(struct snd_berlin_chip *chip)
{
	struct snd_hwdep *hw;
	unsigned int err;

	err = snd_hwdep_new(chip->card, "Berlin hwdep", 0, &hw);
	if (err < 0)
		return err;

	chip->hwdep = hw;
	hw->private_data = chip;
	strcpy(hw->name, "Berlin hwdep interface");

	hw->ops.open = snd_berlin_hwdep_dummy_op;
	hw->ops.ioctl = snd_berlin_hwdep_ioctl;
	hw->ops.ioctl_compat = snd_berlin_hwdep_ioctl;
	hw->ops.release = snd_berlin_hwdep_dummy_op;

	return 0;
}

static void snd_berlin_private_free(struct snd_card *card)
{
	struct snd_berlin_chip *chip = card->private_data;
	kfree(chip);
}

void berlin_report_xrun(enum berlin_xrun_t xrun_type) {
	struct snd_berlin_chip *chip = snd_berlin_card->private_data;

	atomic_long_inc(chip->xruns + xrun_type);
}

static struct snd_berlin_chip* dev_to_berlin_chip(struct device *dev) {
	struct snd_card *card = NULL;
	card = dev_get_drvdata(dev);
	if (card == NULL) {
		return NULL;
	}
	return (struct snd_berlin_chip*) card->private_data;
}

static enum berlin_xrun_t berlin_xrun_string_to_type(const char *string) {
	if (0 == strcmp("pcm_overrun", string))
		return PCM_OVERRUN;
	if (0 == strcmp("fifo_overrun", string))
		return FIFO_OVERRUN;
	if (0 == strcmp("pcm_underrun", string))
		return PCM_UNDERRUN;
	if (0 == strcmp("fifo_underrun", string))
		return FIFO_UNDERRUN;
	if (0 == strcmp("irq_disable_us", string))
		return IRQ_DISABLE;
	snd_printk("%s: unrecognized xrun type: %s\n", __func__, string);
	return XRUN_T_MAX;
}

static ssize_t xrun_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	unsigned long overruns;

	struct snd_berlin_chip *chip = NULL;
	enum berlin_xrun_t xrun_type = berlin_xrun_string_to_type(attr->attr.name);
	if (xrun_type == XRUN_T_MAX) {
		return -EINVAL;
	}

	chip = dev_to_berlin_chip(dev);
	if (chip == NULL) {
		return -ENODEV;
	}
	overruns = atomic_long_read(chip->xruns + xrun_type);
	return snprintf(buf, PAGE_SIZE, "%lu\n", overruns);
}

static ssize_t xrun_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	unsigned long overruns;
	int err;

	struct snd_berlin_chip *chip = NULL;
	enum berlin_xrun_t xrun_type = berlin_xrun_string_to_type(attr->attr.name);
	if (xrun_type == XRUN_T_MAX) {
		return -EINVAL;
	}

	chip = dev_to_berlin_chip(dev);
	if (chip == NULL) {
		return -ENODEV;
	}

	if (0 > (err = kstrtoul(buf, 10, &overruns))) {
		return err;
	}

	// TODO(yichunko): Remove the following after underrun issue is
	// resolved
	// in IRQ_DISABLE mode, overruns = # of us to block
	if (xrun_type == IRQ_DISABLE) {
		unsigned long flags;
		local_irq_save(flags);
		snd_printk("block for %lu us.\n", overruns);
		udelay(overruns);
		local_irq_restore(flags);
	}
	// TODO(yichunko)
	atomic_long_set(chip->xruns + xrun_type, overruns);
	return count;
}

static DEVICE_ATTR(pcm_overrun, S_IWUSR | S_IRUGO, xrun_show, xrun_store);
static DEVICE_ATTR(fifo_overrun, S_IWUSR | S_IRUGO, xrun_show, xrun_store);
static DEVICE_ATTR(pcm_underrun, S_IWUSR | S_IRUGO, xrun_show, xrun_store);
static DEVICE_ATTR(fifo_underrun, S_IWUSR | S_IRUGO, xrun_show, xrun_store);
static DEVICE_ATTR(irq_disable_us, S_IWUSR | S_IRUGO, xrun_show, xrun_store);

static struct attribute *berlin_xrun_sysfs_entries[] = {
	&dev_attr_pcm_overrun.attr,
	&dev_attr_fifo_overrun.attr,
	&dev_attr_pcm_underrun.attr,
	&dev_attr_fifo_underrun.attr,
	&dev_attr_irq_disable_us.attr,
	NULL,
};

static const struct attribute_group berlin_sysfs_group = {
	.name = "xrun",
	.attrs = berlin_xrun_sysfs_entries,
};

static ssize_t mic_mute_state_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct snd_berlin_chip *chip = dev_to_berlin_chip(dev);
	if (chip == NULL) {
		return -ENODEV;
	}
	return snprintf(buf, PAGE_SIZE, "%u\n", chip->is_mic_mute);
}

static DEVICE_ATTR(mic_mute_state, S_IRUGO, mic_mute_state_show, NULL);

// Fault control
#if defined(CONFIG_SND_SOC_BERLIN_HW_VOL_CTRL)

static ssize_t tas5720_fault_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	struct snd_berlin_chip *chip = NULL;
	size_t len;
	int data_or_error;

	chip = dev_to_berlin_chip(dev);
	if (chip == NULL) {
		return -ENODEV;
	}
	data_or_error = tas5720_fault_get();
	if (data_or_error < 0)
		return data_or_error;

	len = 0;
	if (data_or_error & TAS5720_FAULT_OTE) {
		snd_printk("tas5720 fault: OTE\n");
		len = scnprintf(buf, PAGE_SIZE, "OTE ");
	}
	if (data_or_error & TAS5720_FAULT_DCE) {
		snd_printk("tas5720 fault: DCE\n");
		len += scnprintf(buf + len, PAGE_SIZE - len, "DCE ");
	}
	if (data_or_error & TAS5720_FAULT_OCE) {
		snd_printk("tas5720 fault: OCE\n");
		len += scnprintf(buf + len, PAGE_SIZE - len, "OCE ");
	}
	if (data_or_error & TAS5720_FAULT_CLKE) {
		snd_printk("tas5720 fault: CLKE\n");
		len += scnprintf(buf + len, PAGE_SIZE - len, "CLKE ");
	}

	if (len == 0)
		return scnprintf(buf, PAGE_SIZE, "OK\n");

	// Replace trailing space with newline
	buf[len - 1] = '\n';
	return len;
}

static ssize_t tas5720_fault_store(struct device *dev,
                                   struct device_attribute *attr, char *buf,
                                   size_t count)
{
	return count;
}

static ssize_t tas5720_shutdown_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	struct snd_berlin_chip *chip = NULL;
	int data_or_error;

	chip = dev_to_berlin_chip(dev);
	if (chip == NULL) {
		return -ENODEV;
	}
	data_or_error = tas5720_power_get();
	if (data_or_error < 0) {
		return data_or_error;
	}
	if (data_or_error)
		return scnprintf(buf, PAGE_SIZE, "RESET\n");

	return scnprintf(buf, PAGE_SIZE, "SHUTDOWN\n");
}

// Writing "SHUTDOWN" will shut down the amp.
// Writing anything else will start up the amp.
// TODO(jyw): Use mixer controls to read/write faults and shutdown status for
// health_check.
static ssize_t tas5720_shutdown_store(struct device *dev,
                                      struct device_attribute *attr, char *buf,
                                      size_t count)
{
	struct snd_berlin_chip *chip = NULL;
	int ret;
	bool amplifier_on;
	chip = dev_to_berlin_chip(dev);
	if (chip == NULL) {
		return -ENODEV;
	}

	if (strncmp(buf, kTas5720SysfsShutdown,
	            strlen(kTas5720SysfsShutdown)) == 0) {
		amplifier_on = false;
		snd_printk("Shutting down TAS5720\n");
	} else {
		amplifier_on = true;
		snd_printk("Restarting TAS5720\n");
	}

	// Always clear the fault register.
	// Experimentally, OCE won't clear without writing 0 to OCE_THRESH.
	ret = tas5720_fault_set(0x00);
	if (ret < 0) {
		return ret;
	}

	ret = tas5720_power_set(amplifier_on);
	if (ret < 0) {
		return ret;
	}

	return count;
}

static DEVICE_ATTR(fault, S_IRUGO,
                   tas5720_fault_show, tas5720_fault_store);
static DEVICE_ATTR(shutdown, S_IWUSR | S_IRUGO,
                   tas5720_shutdown_show, tas5720_shutdown_store);

static struct attribute *tas5720_sysfs_entries[] = {
	&dev_attr_fault.attr,
	&dev_attr_shutdown.attr,
	NULL,
};

static const struct attribute_group tas5720_sysfs_group = {
	.name = "tas5720",
	.attrs = tas5720_sysfs_entries,
};

#endif

static int snd_berlin_card_init(int dev)
{
	struct snd_berlin_chip *chip;
	unsigned int vec_num;
	char id[16];
	int err;
	enum berlin_xrun_t xrun_type;

	snd_printd("berlin snd card is probed\n");
	sprintf(id, "MRVLBERLIN");
	err = snd_card_create(-1, id, THIS_MODULE, 0, &snd_berlin_card);
	if (snd_berlin_card == NULL)
		return -ENOMEM;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (chip == NULL) {
		err = -ENOMEM;
		goto __nodev;
	}
	chip->mic_mute_state_input = input_allocate_device();
	if (chip->mic_mute_state_input == NULL) {
		err = -ENOMEM;
		goto __nodev;
	}
	chip->mic_mute_state_input->name = "mic_state";
	chip->mic_mute_state_input->evbit[BIT_WORD(EV_KEY)] = BIT_MASK(EV_KEY);
	chip->mic_mute_state_input->keybit[BIT_WORD(KEY_MICMUTE)] =
							BIT_MASK(KEY_MICMUTE);

	err = input_register_device(chip->mic_mute_state_input);
	if (err) {
		snd_printd("Failed to register mic_mute_state as an input"
				"device\n");
		goto __nodev;
	}
	for (xrun_type = 0; xrun_type < XRUN_T_MAX; ++xrun_type) {
		atomic_long_set(chip->xruns + xrun_type, 0);
	}

	snd_berlin_card->private_data = chip;
	snd_berlin_card->private_free = snd_berlin_private_free;

	chip->card = snd_berlin_card;

	if ((err = snd_berlin_card_new_pcm(chip)) < 0)
		goto __nodev;

	/* Dhub configuration */
	DhubInitialization(0, AG_DHUB_BASE, AG_HBO_SRAM_BASE,
			&AG_dhubHandle, AG_config, AG_NUM_OF_CHANNELS_A0);

	/* register and enable audio out ISR */
	vec_num = IRQ_DHUBINTRAVIO1;
	err = request_irq(vec_num, berlin_devices_aout_isr, IRQF_DISABLED,
		"berlin_module_aout", chip->pcm);
	if (unlikely(err < 0)) {
		snd_printk("irq register error: vec_num:%5d, err:%8x\n", vec_num, err);
		goto __nodev;
	}

	strcpy(snd_berlin_card->driver, "Berlin SoC Alsa");
	strcpy(snd_berlin_card->shortname, "Berlin Alsa");
	sprintf(snd_berlin_card->longname, "Berlin Alsa %i", dev + 1);

	if ((err = snd_berlin_card_new_hwdep(chip)) < 0)
		goto __nodev;

	if ((err = snd_card_register(snd_berlin_card)) != 0)
		goto __nodev;
	else {
		snd_printk("berlin snd card device driver registered\n");
		/* add sysfs files */
		err = sysfs_create_group(&snd_berlin_card->card_dev->kobj, &berlin_sysfs_group);
		err = sysfs_create_file(&snd_berlin_card->card_dev->kobj, &dev_attr_mic_mute_state.attr);
#if defined(CONFIG_SND_SOC_BERLIN_HW_VOL_CTRL)
                err = sysfs_create_group(&snd_berlin_card->card_dev->kobj, &tas5720_sysfs_group);
#endif

		return 0;
	}
__nodev:
	if (chip->mic_mute_state_input)
		input_free_device(chip->mic_mute_state_input);

	if (chip)
		kfree(chip);

	snd_card_free(snd_berlin_card);

	return err;
}

static void snd_berlin_card_exit(void)
{
	struct snd_berlin_chip *chip;
	unsigned int vec_num = IRQ_DHUBINTRAVIO1;

	/* unregister audio out ISR */
	chip = (struct snd_berlin_chip *)snd_berlin_card->private_data;
	free_irq(vec_num, chip->pcm);

	sysfs_remove_group(&snd_berlin_card->card_dev->kobj, &berlin_sysfs_group);
	sysfs_remove_file(&snd_berlin_card->card_dev->kobj, &dev_attr_mic_mute_state.attr);
#if defined(CONFIG_SND_SOC_BERLIN_HW_VOL_CTRL)
	sysfs_remove_group(&snd_berlin_card->card_dev->kobj, &tas5720_sysfs_group);
#endif
	snd_card_free(snd_berlin_card);
}

static int __init snd_berlin_init(void)
{
	snd_printd("snd_berlin_init\n");
#if defined(CONFIG_SND_SOC_BERLIN_HW_VOL_CTRL)
	tas5720_client =
		i2c_new_device(i2c_get_adapter(0), &tas5720_hwmon_info);
	if (!tas5720_client) {
		snd_printk("error instantiating tas5720 client");
		return -ENOMEM;
	}
#endif
	return snd_berlin_card_init(0);
}

static void __exit snd_berlin_exit(void)
{
	snd_printd("snd_berlin_exit\n");
	snd_berlin_card_exit();
#if defined(CONFIG_SND_SOC_BERLIN_HW_VOL_CTRL)
	if (tas5720_client)
		i2c_unregister_device(tas5720_client);
#endif
}

MODULE_LICENSE("GPL");
module_init(snd_berlin_init);
module_exit(snd_berlin_exit);
