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
#include <linux/module.h>
#include <linux/atomic.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/socket.h>
#include <linux/spinlock.h>
#include <linux/file.h>
#include <linux/completion.h>
#include <asm/cacheflush.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/control.h>
#include <sound/hwdep.h>
#include <sound/info.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/pcm-indirect.h>
#include <sound/rawmidi.h>

#include "berlin_memmap.h"
#include "api_avio_dhub.h"
#include "api_dhub.h"
#include "api_aio.h"
#include "api_spdif.h"
#include "spdif_enc.h"
#include "api_avpll.h"
#include "api_playback.h"
#include "pcm.h"

enum {
        SND_BERLIN_OUTPUT_ANALOG = 0x200,
        SND_BERLIN_OUTPUT_SPDIF,
};

#define MIN_PERIOD_SIZE        (1 * 1024)
#define MAX_PERIOD_SIZE        (2 * 1024)
#define MAX_BUFFER_SIZE        (32 * 1024)

struct snd_berlin_card_pcm {
        /*
         * Driver-specific debug proc entry
         */
        struct snd_info_entry *entry;

        /*
         * Tracks the base address of the last submitted DMA block.
         * Moved to next period in ISR.
         * Read in snd_berlin_playback_pointer.
         * Offset is given in the format received from userspace ("virtual
         * bytes").
         * Access under lock.
         */
        unsigned int current_virt_dma_offset;

        /*
         * Is there a submitted DMA request?
         * Set when a DMA request is submitted to DHUB.
         * Cleared on 'stop' or ISR.
         * Access under lock.
         */
        bool dma_pending;
        /*
         * unmute audio when first DMA is back
         */
        bool need_unmute_audio;
        /*
         * Indicates if page memory is allocated
         */
        bool pages_allocated;

        /*
         * Instance lock between ISR and higher contexts.
         */
        spinlock_t   lock;

        /* spdif DMA buffer */
        unsigned char *spdif_dma_area; /* dma buffer for spdif output */
        dma_addr_t spdif_dma_addr;   /* physical address of spdif buffer */
        unsigned int spdif_dma_bytes;  /* size of spdif dma area */

        /* PCM DMA buffer */
        unsigned char *pcm_dma_area;
        dma_addr_t pcm_dma_addr;
        /* Total size of ALSA buffer in the format that would be DMAed */
        unsigned int pcm_dma_bytes;
        /* Total size of ALSA buffer in the format received from userspace.
         * Note that 16 bit input is padded out to 32. */
        unsigned int pcm_virt_bytes;
        /* Size of data sent in one DMA transfer, in the format received from
         * userspace.
         */
        unsigned int pcm_virt_period_bytes;


        /* hw parameter */
        unsigned int sample_rate;
        unsigned int sample_format;
        unsigned int channel_num;
        unsigned int pcm_ratio;
        unsigned int spdif_ratio;

        /* for spdif encoding */
        unsigned int spdif_frames;
        unsigned char channel_status[24];

        /* playback status */
        bool playing;
        unsigned int output_mode;

        struct snd_pcm_substream *substream;
        struct snd_pcm_indirect pcm_indirect;
};

atomic_t g_output_mode = ATOMIC_INIT(SND_BERLIN_OUTPUT_ANALOG);

static unsigned int berlin_playback_rates[] = {
        8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
        64000, 88200, 96000
};

static struct snd_pcm_hw_constraint_list berlin_constraints_rates = {
        .count = ARRAY_SIZE(berlin_playback_rates),
        .list = berlin_playback_rates,
        .mask = 0,
};

/*
 * Applies output configuration of |berlin_pcm| to i2s.
 * Must be called with instance spinlock held.
 */
static void berlin_set_aio(const struct snd_berlin_card_pcm *berlin_pcm)
{
        unsigned int analog_div, spdif_div;

        assert_spin_locked(&berlin_pcm->lock);

        AIO_SetAudChMute(AIO_SEC, AIO_TSD0, AUDCH_CTRL_MUTE_MUTE_ON);
        AIO_SetAudChEn(AIO_SEC, AIO_TSD0, AUDCH_CTRL_ENABLE_DISABLE);

        // update berlin_playback_rates if any rates are added or removed.
        switch (berlin_pcm->sample_rate) {
        case 8000  :
        case 11025 :
        case 12000 :
                analog_div = AIO_DIV32;
                spdif_div  = AIO_DIV16;
                break;
        case 16000 :
        case 22050 :
        case 24000 :
                analog_div = AIO_DIV16;
                spdif_div  = AIO_DIV8;
                break;
        case 32000 :
        case 44100 :
        case 48000 :
                analog_div = AIO_DIV8;
                spdif_div  = AIO_DIV4;
                break;
        case 64000 :
        case 88200 :
        case 96000 :
                analog_div = AIO_DIV4;
                spdif_div  = AIO_DIV2;
                break;
        default:
                break;
        }

        if (berlin_pcm->output_mode == SND_BERLIN_OUTPUT_ANALOG) {
                AIO_SetClkDiv(AIO_SEC, analog_div);
                AIO_SetCtl(AIO_SEC, AIO_I2S_MODE, AIO_32CFM, AIO_24DFM);
        } else if (berlin_pcm->output_mode == SND_BERLIN_OUTPUT_SPDIF) {
                AIO_SetClkDiv(AIO_SEC, spdif_div);
                AIO_SetCtl(AIO_SEC, AIO_I2S_MODE, AIO_32CFM, AIO_32DFM);
        }

        AIO_SetAudChEn(AIO_SEC, AIO_TSD0, AUDCH_CTRL_ENABLE_ENABLE);

        // Only unmute if we were playing.
        if (berlin_pcm->playing)
                AIO_SetAudChMute(AIO_SEC, AIO_TSD0, AUDCH_CTRL_MUTE_MUTE_OFF);
}

static void berlin_set_pll(unsigned int sample_rate)
{
        int apll;

        switch (sample_rate) {
        case 11025 :
        case 22050 :
        case 44100 :
        case 88200 :
                apll = 22579200;
                break;
        case 8000  :
        case 16000 :
        case 32000 :
        case 64000 :
                apll = 16384000;
                break;
        case 12000 :
        case 24000 :
        case 48000 :
        case 96000 :
                apll = 24576000;
                break;
        default :
                apll = 24576000;
                break;
        }
        // WARNING: AVPLL channel 4 is reserved for PDM capture. DO NOT
        // set parameters for it here
        AVPLL_Set(0, 3, apll);
}

/*
 * Kicks off a DMA transfer to audio IO interface for the |berlin_pcm|.
 * Must be called with instance spinlock held.
 * Must be called only when instance is in playing state.
 */
static void start_dma_if_needed(struct snd_berlin_card_pcm *berlin_pcm)
{
        const unsigned int chanId = avioDhubChMap_ag_SA0_R_A0;
        unsigned int mode;
        dma_addr_t dma_source_address;
        int dma_size;

        assert_spin_locked(&berlin_pcm->lock);

        if (!berlin_pcm->playing) {
                snd_printd("%s: playing: %u\n", __func__, berlin_pcm->playing);
                return;
        }

        if (berlin_pcm->pcm_indirect.hw_ready < berlin_pcm->pcm_virt_period_bytes) {
                berlin_report_xrun(PCM_UNDERRUN);
                snd_printk("%s: underrun! hw_ready: %d\n", __func__,
                        berlin_pcm->pcm_indirect.hw_ready);
                return;
        }

        if (berlin_pcm->dma_pending)
                return;

        mode = atomic_read(&g_output_mode);
        if (berlin_pcm->output_mode != mode) {
                berlin_pcm->output_mode = mode;
                berlin_set_aio(berlin_pcm);
        }

        if (mode == SND_BERLIN_OUTPUT_ANALOG) {
                dma_source_address = berlin_pcm->pcm_dma_addr +
                        berlin_pcm->current_virt_dma_offset * berlin_pcm->pcm_ratio;
                dma_size = berlin_pcm->pcm_virt_period_bytes * berlin_pcm->pcm_ratio;
        } else {
                dma_source_address = berlin_pcm->spdif_dma_addr +
                        berlin_pcm->current_virt_dma_offset * berlin_pcm->spdif_ratio;
                dma_size = berlin_pcm->pcm_virt_period_bytes * berlin_pcm->spdif_ratio;
        }

        berlin_pcm->dma_pending = true;

        dhub_channel_write_cmd(&AG_dhubHandle.dhub, chanId,
                dma_source_address, dma_size, 0, 0, 0, 1, 0, 0);
}

static void snd_berlin_playback_trigger_start(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_pcm *berlin_pcm = runtime->private_data;
        unsigned long flags;

        spin_lock_irqsave(&berlin_pcm->lock, flags);
        if (berlin_pcm->playing) {
                spin_unlock_irqrestore(&berlin_pcm->lock, flags);
                return;
        }

        berlin_pcm->playing = true;
        AIO_SetAudChFlush(AIO_SEC, AIO_TSD0, AUDCH_CTRL_FLUSH_ON);
        AIO_SetAudChFlush(AIO_SEC, AIO_TSD0, AUDCH_CTRL_FLUSH_OFF);
        //AIO_SetAudChMute(AIO_SEC, AIO_TSD0, AUDCH_CTRL_MUTE_MUTE_OFF);
        berlin_pcm->need_unmute_audio = true;
	start_dma_if_needed(berlin_pcm);
        spin_unlock_irqrestore(&berlin_pcm->lock, flags);
        snd_printd("%s: finished.\n", __func__);
}

static void snd_berlin_playback_trigger_stop(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_pcm *berlin_pcm = runtime->private_data;
        unsigned long flags;

        spin_lock_irqsave(&berlin_pcm->lock, flags);
        AIO_SetAudChMute(AIO_SEC, AIO_TSD0, AUDCH_CTRL_MUTE_MUTE_ON);
        berlin_pcm->playing = false;
        berlin_pcm->dma_pending = false;
        spin_unlock_irqrestore(&berlin_pcm->lock, flags);
        snd_printd("%s: finished.\n", __func__);
}

static const struct snd_pcm_hardware snd_berlin_playback_hw = {
        .info = (SNDRV_PCM_INFO_MMAP
                | SNDRV_PCM_INFO_INTERLEAVED
                | SNDRV_PCM_INFO_MMAP_VALID
                | SNDRV_PCM_INFO_PAUSE
                | SNDRV_PCM_INFO_RESUME),
        .formats = (SNDRV_PCM_FMTBIT_S32_LE
                | SNDRV_PCM_FMTBIT_S16_LE),
        .rates = (SNDRV_PCM_RATE_8000_96000
                | SNDRV_PCM_RATE_KNOT),
        .rate_min = 8000,
        .rate_max = 96000,
        .channels_min = 1,
        .channels_max = 2,
        .buffer_bytes_max = MAX_BUFFER_SIZE,
        .period_bytes_min = MIN_PERIOD_SIZE,
        .period_bytes_max = MAX_PERIOD_SIZE,
        .periods_min = 2,
        .periods_max = MAX_BUFFER_SIZE / MIN_PERIOD_SIZE,
        .fifo_size = 0
};

static void snd_berlin_runtime_free(struct snd_pcm_runtime *runtime)
{
        struct snd_berlin_card_pcm *berlin_pcm = runtime->private_data;
        if (berlin_pcm) {
                if (berlin_pcm->spdif_dma_area) {
                        dma_free_coherent(NULL, berlin_pcm->spdif_dma_bytes,
                                        berlin_pcm->spdif_dma_area,
                                        berlin_pcm->spdif_dma_addr);
                        berlin_pcm->spdif_dma_area = NULL;
                        berlin_pcm->spdif_dma_addr = 0;
                }

                if (berlin_pcm->pcm_dma_area) {
                        dma_free_coherent(NULL, berlin_pcm->pcm_dma_bytes,
                                        berlin_pcm->pcm_dma_area,
                                        berlin_pcm->pcm_dma_addr);
                        berlin_pcm->pcm_dma_area = NULL;
                        berlin_pcm->pcm_dma_addr = 0;
                }

                if (berlin_pcm->entry)
                        snd_info_free_entry(berlin_pcm->entry);

                kfree(berlin_pcm);
        }
}

static void debug_entry(struct snd_info_entry *entry,
                        struct snd_info_buffer *buffer)
{
        struct snd_berlin_card_pcm *berlin_pcm =
                (struct snd_berlin_card_pcm *)entry->private_data;
        unsigned long flags;
        spin_lock_irqsave(&berlin_pcm->lock, flags);
        snd_iprintf(buffer, "playing:\t\t%d\n", berlin_pcm->playing);
        snd_iprintf(buffer, "dma_pending:\t\t%d\n", berlin_pcm->dma_pending);
        snd_iprintf(buffer, "output_mode:\t\t%d\n", berlin_pcm->output_mode);
        snd_iprintf(buffer, "current_virt_dma_offset:\t%u\n",
                berlin_pcm->current_virt_dma_offset);
        snd_iprintf(buffer, "\n");
        snd_iprintf(buffer, "indirect.hw_data:\t%u\n",
                berlin_pcm->pcm_indirect.hw_data);
        snd_iprintf(buffer, "indirect.hw_io:\t\t%u\n",
                berlin_pcm->pcm_indirect.hw_io);
        snd_iprintf(buffer, "indirect.hw_ready:\t%d\n",
                berlin_pcm->pcm_indirect.hw_ready);
        snd_iprintf(buffer, "indirect.sw_data:\t%u\n",
                berlin_pcm->pcm_indirect.hw_data);
        snd_iprintf(buffer, "indirect.sw_io:\t\t%u\n",
                berlin_pcm->pcm_indirect.hw_io);
        snd_iprintf(buffer, "indirect.sw_ready:\t%d\n",
                berlin_pcm->pcm_indirect.sw_ready);
        snd_iprintf(buffer, "indirect.appl_ptr:\t%lu\n",
                berlin_pcm->pcm_indirect.appl_ptr);
        snd_iprintf(buffer, "\n");
        snd_iprintf(buffer, "substream->runtime->control->appl_ptr:\t%lu\n",
                berlin_pcm->substream->runtime->control->appl_ptr);
        snd_iprintf(buffer, "substream->runtime->status->hw_ptr:\t%lu\n",
                berlin_pcm->substream->runtime->status->hw_ptr);
        spin_unlock_irqrestore(&berlin_pcm->lock, flags);
}

int snd_berlin_playback_open(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_pcm *berlin_pcm;

        int err = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
                        &berlin_constraints_rates);
        if (err < 0) {
                snd_printk("%s: Invalid sample rate.\n", __func__);
                return err;
        }

        err = snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
                        MIN_PERIOD_SIZE);
        if (err < 0) {
                snd_printk("%s: Invalid period size.\n", __func__);
                return err;
        }

        berlin_pcm = kzalloc(sizeof(*berlin_pcm), GFP_KERNEL);
        if (berlin_pcm == NULL)
                return -ENOMEM;

        berlin_pcm->entry = snd_info_create_card_entry(substream->pcm->card,
                "debug", substream->proc_root);
        if (!berlin_pcm->entry) {
                snd_printk("%s: couldn't create debug entry\n", __func__);
        } else {
                snd_info_set_text_ops(berlin_pcm->entry, berlin_pcm, debug_entry);
                snd_info_register(berlin_pcm->entry);
        }

        berlin_pcm->dma_pending = false;
        berlin_pcm->playing = false;
        berlin_pcm->pages_allocated = false;
        berlin_pcm->output_mode = atomic_read(&g_output_mode);
        berlin_pcm->substream = substream;
        spin_lock_init(&berlin_pcm->lock);

        runtime->private_data = berlin_pcm;
        runtime->private_free = snd_berlin_runtime_free;
        runtime->hw = snd_berlin_playback_hw;

	/* enable audio interrupt */
        DhubEnableIntr(0, &AG_dhubHandle, avioDhubChMap_ag_SA0_R_A0, 1);

        // Enable i2s channel without corresponding disable in close.
        // This is intentional: Avoid SPDIF sink 'activation delay' problems.
        AIO_SetAudChEn(AIO_SEC, AIO_TSD0, AUDCH_CTRL_ENABLE_ENABLE);
        AIO_SetAudChMute(AIO_SEC, AIO_TSD0, AUDCH_CTRL_MUTE_MUTE_ON);

        snd_printd("%s: finished.\n", __func__);
        return 0;
}

int snd_berlin_playback_close(struct snd_pcm_substream *substream)
{
	DhubEnableIntr(0, &AG_dhubHandle, avioDhubChMap_ag_SA0_R_A0, 0);
        snd_printd("%s: finished.\n", __func__);
        return 0;
}

int snd_berlin_playback_hw_free(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_pcm *berlin_pcm = runtime->private_data;

        if (berlin_pcm->spdif_dma_area) {
                dma_free_coherent(NULL, berlin_pcm->spdif_dma_bytes,
                                berlin_pcm->spdif_dma_area,
                                berlin_pcm->spdif_dma_addr);
                berlin_pcm->spdif_dma_area = NULL;
                berlin_pcm->spdif_dma_addr = 0;
        }

        if (berlin_pcm->pcm_dma_area) {
                dma_free_coherent(NULL, berlin_pcm->pcm_dma_bytes,
                                berlin_pcm->pcm_dma_area,
                                berlin_pcm->pcm_dma_addr);
                berlin_pcm->pcm_dma_area = NULL;
                berlin_pcm->pcm_dma_addr = 0;
        }

        if (berlin_pcm->pages_allocated == true) {
                snd_pcm_lib_free_pages(substream);
                berlin_pcm->pages_allocated = false;
        }

        return 0;
}

int snd_berlin_playback_hw_params(struct snd_pcm_substream *substream,
                                        struct snd_pcm_hw_params *params)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_pcm *berlin_pcm = runtime->private_data;
        struct spdif_channel_status *chnsts;
        int err;
        unsigned long flags;
        const size_t buffer_size_bytes = params_buffer_bytes(params);
        const size_t period_size_bytes = buffer_size_bytes / params_periods(params);

        snd_berlin_playback_hw_free(substream);

        err = snd_pcm_lib_malloc_pages(substream, buffer_size_bytes);
        if (err < 0) {
                snd_printk("pcm_lib_malloc failed to allocated pages for buffers\n");
                return err;
        }
        berlin_pcm->pages_allocated = true;

        snd_printk("%s: sample_rate:%d channels:%d format:%s\n", __func__,
                   params_rate(params), params_channels(params),
                   snd_pcm_format_name(params_format(params)));

        berlin_pcm->sample_rate = params_rate(params);
        berlin_pcm->sample_format = params_format(params);
        berlin_pcm->channel_num = params_channels(params);
        berlin_pcm->pcm_ratio = 1;

        if (berlin_pcm->sample_format == SNDRV_PCM_FORMAT_S16_LE)
                berlin_pcm->pcm_ratio *= 2;
        if (berlin_pcm->channel_num == 1)
                berlin_pcm->pcm_ratio *= 2;

        berlin_pcm->spdif_ratio = berlin_pcm->pcm_ratio * 2;

        berlin_pcm->pcm_virt_period_bytes = period_size_bytes;
        berlin_pcm->pcm_virt_bytes = buffer_size_bytes;
        berlin_pcm->pcm_dma_bytes = berlin_pcm->pcm_virt_bytes *
                berlin_pcm->pcm_ratio;
        berlin_pcm->pcm_dma_area = dma_zalloc_coherent(
                NULL, berlin_pcm->pcm_dma_bytes,
                &berlin_pcm->pcm_dma_addr, GFP_KERNEL);
        if (!berlin_pcm->pcm_dma_area) {
                snd_printk("%s: failed to allocate PCM DMA area\n", __func__);
                goto err_pcm_dma;
        }

        berlin_pcm->spdif_dma_bytes = berlin_pcm->pcm_virt_bytes *
                berlin_pcm->spdif_ratio;
        berlin_pcm->spdif_dma_area = dma_zalloc_coherent(
                NULL, berlin_pcm->spdif_dma_bytes,
                &berlin_pcm->spdif_dma_addr, GFP_KERNEL);
        if (!berlin_pcm->spdif_dma_area) {
                snd_printk("%s: failed to allocate SPDIF DMA area\n", __func__);
                goto err_spdif_dma;
        }

        /* initialize spdif channel status */
        chnsts = (struct spdif_channel_status *)&(berlin_pcm->channel_status[0]);
        spdif_init_channel_status(chnsts, berlin_pcm->sample_rate);

        /* AVPLL configuration */
        berlin_set_pll(berlin_pcm->sample_rate);

        spin_lock_irqsave(&berlin_pcm->lock, flags);
        /* AIO configuration */
        berlin_set_aio(berlin_pcm);
        spin_unlock_irqrestore(&berlin_pcm->lock, flags);

        return 0;

err_spdif_dma:
        if (berlin_pcm->pcm_dma_area)
                dma_free_coherent(NULL, berlin_pcm->pcm_dma_bytes,
                        berlin_pcm->pcm_dma_area, berlin_pcm->pcm_dma_addr);
        berlin_pcm->pcm_dma_area = NULL;
        berlin_pcm->pcm_dma_addr = 0;
err_pcm_dma:
        snd_pcm_lib_free_pages(substream);
        return -ENOMEM;
}

int snd_berlin_playback_prepare(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_pcm *berlin_pcm = runtime->private_data;
        unsigned long flags;

        spin_lock_irqsave(&berlin_pcm->lock, flags);
        berlin_pcm->current_virt_dma_offset = 0;
        memset(&berlin_pcm->pcm_indirect, 0, sizeof(berlin_pcm->pcm_indirect));
        /* Typically pcm_indirect is used to fake period/buffer size flexibility
         * for hardware with a fixed period/buffer size configuration.
         * In this case it is only used as an intermediate buffer for MMAP
         * support. Note that pcm_indirect is not strictly necessary for an
         * intermediate buffer (we could use the existing allocated DMA areas),
         * but because we convert to SPDIF, expand mono output and expand 16 bit
         * samples to 32 by bitbanging, we cannot write directly into those
         * buffers.
         */
        berlin_pcm->pcm_indirect.hw_buffer_size = snd_pcm_lib_buffer_bytes(substream);
        berlin_pcm->pcm_indirect.sw_buffer_size = snd_pcm_lib_buffer_bytes(substream);
        spin_unlock_irqrestore(&berlin_pcm->lock, flags);

        snd_printd("%s finished. buffer_bytes: %d period_bytes: %d\n", __func__,
                snd_pcm_lib_buffer_bytes(substream),
                snd_pcm_lib_period_bytes(substream));
        return 0;
}

int snd_berlin_playback_trigger(struct snd_pcm_substream *substream, int cmd)
{
        int    ret = 0;

        switch (cmd) {
        case SNDRV_PCM_TRIGGER_START:
        case SNDRV_PCM_TRIGGER_RESUME:
        case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
                snd_berlin_playback_trigger_start(substream);
                break;
        case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_SUSPEND:
        case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
                snd_berlin_playback_trigger_stop(substream);
                break;
        default:
                ret = -EINVAL;
        }

        return ret;
}

snd_pcm_uframes_t
snd_berlin_playback_pointer(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_pcm *berlin_pcm = runtime->private_data;
        uint32_t buf_pos;
        unsigned long flags;

        spin_lock_irqsave(&berlin_pcm->lock, flags);
        buf_pos = berlin_pcm->current_virt_dma_offset;
        spin_unlock_irqrestore(&berlin_pcm->lock, flags);

        return snd_pcm_indirect_playback_pointer(substream, &berlin_pcm->pcm_indirect,
                                                buf_pos);
}

// Encodes |frames| number of stereo S32LE frames from |pcm_in|
// to |spdif_out| SPDIF frames (64-bits per frame)
static void spdif_encode(struct snd_berlin_card_pcm *berlin_pcm,
        const int32_t *pcm_in, int32_t *spdif_out, int frames)
{
        int i;
        for (i = 0; i < frames; ++i) {
                unsigned char channel_status =
                        spdif_get_channel_status(berlin_pcm->channel_status,
                                berlin_pcm->spdif_frames);
                unsigned int sync_word =
                        berlin_pcm->spdif_frames ? TYPE_M : TYPE_B;
                spdif_enc_subframe(&spdif_out[i * 4],
                        pcm_in[i * 2], sync_word, 0, 0, channel_status);

                sync_word = TYPE_W;
                spdif_enc_subframe(&spdif_out[(i * 4) + 2],
                        pcm_in[(i * 2) + 1], sync_word, 0, 0, channel_status);

                ++berlin_pcm->spdif_frames;
                berlin_pcm->spdif_frames %= SPDIF_BLOCK_SIZE;
        }
}

/* Copies (and converts as necessary) data from the pcm_indirect buffer into
 * the PCM and SPDIF DMA buffers.
 */
static int snd_berlin_playback_copy(struct snd_pcm_substream *substream,
                                int channel, snd_pcm_uframes_t pos,
                                void *buf, size_t bytes)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_pcm *berlin_pcm = runtime->private_data;
        int32_t *pcm_buf = (int32_t *)(berlin_pcm->pcm_dma_area +
                pos * berlin_pcm->pcm_ratio);
        int32_t *spdif_buf = (int32_t *)(berlin_pcm->spdif_dma_area +
                pos * berlin_pcm->spdif_ratio);
        const int frames = bytes /
                (berlin_pcm->channel_num *
                snd_pcm_format_width(berlin_pcm->sample_format) / 8);
        int i;

        if (pos >= berlin_pcm->pcm_virt_bytes)
                return -EINVAL;

        if (berlin_pcm->pcm_indirect.hw_ready >= berlin_pcm->pcm_virt_period_bytes) {
                unsigned long flags;
                unsigned long dma_begin, dma_end, write_begin, write_end;

                spin_lock_irqsave(&berlin_pcm->lock, flags);
                dma_begin = berlin_pcm->current_virt_dma_offset;
                spin_unlock_irqrestore(&berlin_pcm->lock, flags);
                dma_end = dma_begin + berlin_pcm->pcm_virt_period_bytes - 1;
                write_begin = pos;
                write_end = write_begin + bytes - 1;

                // Write begin position shouldn't be in DMA area.
                if ((dma_begin <= write_begin) && (write_begin <= dma_end)) {
                        snd_printk("%s: dma_begin:%lu <= write_begin:%lu "
                                   "<= dma_end:%lu\n",
                                   __func__, dma_begin, write_begin, dma_end);
                }

                // Write end position shouldn't be in DMA area.
                if ((dma_begin <= write_end) && (write_end <= dma_end)) {
                        snd_printk("%s: dma_begin:%lu <= write_end:%lu <= "
                                   "dma_end:%lu\n",
                                   __func__, dma_begin, write_end, dma_end);
                }

                // Write shouldn't overlap DMA area.
                if ((write_begin <= dma_begin) && (write_end >= dma_end)) {
                        snd_printk("%s: write_begin:%lu <= dma_begin:%lu && "
                                   "write_end:%lu >= dma_end:%lu\n",
                                   __func__,  write_begin, dma_begin,
                                   write_end, dma_end);
                }
        }

        if (berlin_pcm->sample_format == SNDRV_PCM_FORMAT_S16_LE) {
                const int16_t *s16_pcm_source = (int16_t *)buf;

                if (berlin_pcm->channel_num == 1) {
                        // Shift sample to 32-bits, and upmix to stereo.
                        for (i = 0; i < frames; ++i) {
                                const int32_t s32_sample = s16_pcm_source[i] << 16;
                                pcm_buf[i * 2] = s32_sample;
                                pcm_buf[(i * 2) + 1] = s32_sample;
                        }
                } else {
                        // Copy left and right samples while shifting to 32-bits.
                        for (i = 0; i < frames; ++i) {
                                pcm_buf[i * 2] =
                                        s16_pcm_source[i * 2] << 16;
                                pcm_buf[(i * 2) + 1] =
                                        s16_pcm_source[(i * 2) + 1] << 16;
                        }
                }
        } else if (berlin_pcm->sample_format == SNDRV_PCM_FORMAT_S32_LE) {
                const int32_t *s32_pcm_source = (int32_t *)buf;

                if (berlin_pcm->channel_num == 1) {
                        // Upmix each sample to stereo.
                        for (i = 0; i < frames; ++i) {
                                pcm_buf[i * 2] = s32_pcm_source[i];
                                pcm_buf[(i * 2) + 1] = s32_pcm_source[i];
                        }
                } else {
                        // Copy the left and right samples straight over.
                        for (i = 0; i < frames; ++i) {
                                pcm_buf[i * 2] = s32_pcm_source[i * 2];
                                pcm_buf[(i * 2) + 1] = s32_pcm_source[(i * 2) + 1];
                        }
                }
        } else {
                return -EINVAL;
        }

        spdif_encode(berlin_pcm, pcm_buf, spdif_buf, frames);
        return 0;
}

static void berlin_playback_transfer(struct snd_pcm_substream *substream,
                                        struct snd_pcm_indirect *rec, size_t bytes)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_pcm *berlin_pcm = runtime->private_data;
        void *src = (void *)(runtime->dma_area + rec->sw_data);
        if (!src)
                return;

        snd_berlin_playback_copy(substream, berlin_pcm->channel_num,
                rec->hw_data, src, bytes);
}

int snd_berlin_playback_ack(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_pcm *berlin_pcm = runtime->private_data;
        struct snd_pcm_indirect *pcm_indirect = &berlin_pcm->pcm_indirect;
        unsigned long flags;
        pcm_indirect->hw_queue_size = berlin_pcm->pcm_virt_bytes;
        snd_pcm_indirect_playback_transfer(substream, pcm_indirect,
                                                berlin_playback_transfer);
        spin_lock_irqsave(&berlin_pcm->lock, flags);
        start_dma_if_needed(berlin_pcm);
        spin_unlock_irqrestore(&berlin_pcm->lock, flags);
        return 0;
}

int snd_berlin_playback_isr(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_pcm *berlin_pcm = runtime->private_data;
        unsigned long flags;
        // FIFO fullness
        unsigned int dhub_cnt;

        if (berlin_pcm->need_unmute_audio) {
		AIO_SetAudChMute(AIO_SEC, AIO_TSD0, AUDCH_CTRL_MUTE_MUTE_OFF);
		berlin_pcm->need_unmute_audio = false;
	}
        spin_lock_irqsave(&berlin_pcm->lock, flags);
        /* If we are not running, do not chain, and clear pending */
        if (!berlin_pcm->playing) {
                berlin_pcm->dma_pending = false;
                spin_unlock_irqrestore(&berlin_pcm->lock, flags);
                return IRQ_HANDLED;
        }

        /* If we were not pending, avoid pointer manipulation */
        if (!berlin_pcm->dma_pending) {
                spin_unlock_irqrestore(&berlin_pcm->lock, flags);
                return IRQ_HANDLED;
        }

        /* Roll the DMA pointer, and chain if needed */
        berlin_pcm->current_virt_dma_offset += berlin_pcm->pcm_virt_period_bytes;
        berlin_pcm->current_virt_dma_offset %= berlin_pcm->pcm_virt_bytes;
        berlin_pcm->dma_pending = false;

        /* to query the consumer data count register to check fifo fullness */
        static const uintptr_t addr = AG_DHUB_BASE + RA_dHubReg_HBO + 0x80 +
                                      avioDhubChMap_ag_SA0_R_A0 * 4 * 2 + 4;
        dhub_cnt = (*((volatile unsigned int*)(addr)));
        // According to Marvell, we only use the lower 16 bits
        dhub_cnt &= 0xffff;
        // The size of output FIFO is 1024 bytes
        // dhub count is in units of 64 bit frames (2 channel, 4 byte sample).
        // The 64 bit frames are each 8 bytes.
        // When the size of the output FIFO is 1024 bytes, the typical dhub_cnt
        // value is 128, which is 128*8 = 1024 bytes.
        if (dhub_cnt <= 8) {
          berlin_report_xrun(FIFO_UNDERRUN);
          snd_printk("Dhub cnt: %d bytes, FIFO nearly empty\n", dhub_cnt*8);
        }

        start_dma_if_needed(berlin_pcm);
        spin_unlock_irqrestore(&berlin_pcm->lock, flags);
        snd_pcm_period_elapsed(substream);
	return 0;
}
