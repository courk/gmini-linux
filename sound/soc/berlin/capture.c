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
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/log2.h>

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
#include "api_avpll.h"
#include "api_capture.h"
#include "pcm.h"

#define CIC_MIN_DECIMATION_FACTOR 128
#define CIC_MAX_ORDER 5

/* For every 32 bit PCM sample there are |decimation_factor| PDM bits. Since
 * decimation factor is always a multiple of 32, the PDM buffer sizes are
 * integer multiples of the PCM buffer sizes.
   When adjusting buffer size, check snd_pcm_lib_preallocate_pages_for_all() in
   pcm.c.

   All sizes are in bytes unless specified otherwise.
*/
#define PCM_SAMPLE_SIZE_BITS       32
#define PDM_MIN_DMA_BUFFER_SIZE        1024
#define PDM_MAX_DMA_BUFFER_SIZE        8192
#define PDM_MAX_BUFFER_SIZE        (8 * 8192)
#define PCM_MIN_DMA_BUFFER_SIZE        (PDM_MIN_DMA_BUFFER_SIZE / (CIC_MIN_DECIMATION_FACTOR / PCM_SAMPLE_SIZE_BITS))
#define PCM_MAX_DMA_BUFFER_SIZE        (PDM_MAX_DMA_BUFFER_SIZE / (CIC_MIN_DECIMATION_FACTOR / PCM_SAMPLE_SIZE_BITS))
#define PCM_MAX_BUFFER_SIZE        (PDM_MAX_BUFFER_SIZE / (CIC_MIN_DECIMATION_FACTOR / PCM_SAMPLE_SIZE_BITS))

#define MAX_CHANNELS 2

#define MIC_MUTE_THRESHOLD_IN_MS   300

static int cic_order_from_rate(int sample_rate) {
        switch (sample_rate) {
        case 8000  :
        case 11025 :
        case 12000 :
        case 16000 :
        case 22050 :
        case 24000 :
                return 4;
        case 32000 :
#if defined(CONFIG_SND_SOC_BERLIN_SLOWER_16KHZ_PDM_CLOCK)
                return 4;
#else
                return 5;
#endif
        case 44100 :
        case 48000 :
        case 64000 :
        case 88200 :
        case 96000 :
                return 5;
        default:
                break;
        }
        return 0;
}

static int cic_decimation_factor_from_rate(int sample_rate) {
        switch (sample_rate) {
        case 8000  :
        case 11025 :
        case 12000 :
        case 22050 :
        case 24000 :
                return 128;
        case 16000 :
#if defined(CONFIG_SND_SOC_BERLIN_SLOWER_16KHZ_PDM_CLOCK)
                return 96;
#else
                return 128;
#endif
        case 32000 :
#if defined(CONFIG_SND_SOC_BERLIN_SLOWER_16KHZ_PDM_CLOCK)
                return 96;
#else
                return 64;
#endif
        case 44100 :
        case 48000 :
        case 64000 :
        case 88200 :
        case 96000 :
                return 64;
        default:
                break;
        }
        return 0;
}

struct cic_decimator {
        uint32_t integrator_stage[CIC_MAX_ORDER];
        uint32_t comb_stage[CIC_MAX_ORDER];
        int order;
        int decimation_factor;
        int bmax;
        int32_t (*process) (struct cic_decimator *cic, uint32_t *data);
};

#define DC_HPF_CUTOFF_HZ 60
struct dc_canceller {
        float alpha;
        float last_input;
        float last_output;
};

struct snd_berlin_card_capture {
        /*
         * Tracks the base address of the last submitted DMA block.
         * Moved to next period in ISR.
         * read in snd_berlin_playback_pointer.
         */
        unsigned int current_dma_offset;
        /*
         * Offset of the next DMA buffer that needs to be decimated in bytes.
         * Since it is only read/written on a work queue thread,
         * it does not need locking.
         */
        unsigned int read_offset;
        /*
         * Offset in bytes at which decoded PCM data is being written.
         * Writing should only be done on a work queue thread; must be locked.
         * Reading on a work queue thread does not require locking.
         * Reading outside of a work queue thread requires locking.
         */
        unsigned int runtime_offset;
        /*
         * Number of bytes read but not decoded yet.
         * Read and write under lock.
         */
        unsigned int cnt;
        bool need_audio_unmute;
        /*
         * Is there a submitted DMA request?
         * Set when a DMA request is submitted to DHUB.
         * Cleared on 'stop' or ISR.
         */
        bool dma_pending;

        /*
         * Indicates if page memory is allocated
         */
        bool pages_allocated;

        /*
         * Instance lock between ISR and higher contexts.
         */
        spinlock_t   lock;

        /* PDM DMA buffer */
        unsigned char *pdm_dma_area;
        dma_addr_t pdm_dma_addr;
        unsigned int pdm_dma_bytes;
        unsigned int pdm_dma_period_size;

        /* hw parameter */
        unsigned int sample_rate;
        unsigned int sample_format;
        unsigned int channel_num;
        unsigned int chan_id;

        /* capture status */
        bool capturing;

        struct snd_pcm_substream *substream;
        struct cic_decimator cic[MAX_CHANNELS];
        struct dc_canceller dcc[MAX_CHANNELS];
        struct workqueue_struct *wq;
        struct delayed_work delayed_work;
};

static unsigned int berlin_capture_rates[] = {
        8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000, 96000
};

static struct snd_pcm_hw_constraint_list berlin_constraints_rates = {
        .count = ARRAY_SIZE(berlin_capture_rates),
        .list = berlin_capture_rates,
        .mask = 0,
};

static int hw_rule_period_restricts_rate(struct snd_pcm_hw_params *params,
                                         struct snd_pcm_hw_rule* rule)
{
    struct snd_interval *period_bytes =
            hw_param_interval(params, SNDRV_PCM_HW_PARAM_PERIOD_BYTES);
    // Decimation factor 96 or 128 not possible.
    if (period_bytes->min * 3 > PDM_MAX_DMA_BUFFER_SIZE) {
        const struct snd_interval rate_interval = {
                .min = 44100,
                .max = 96000
        };
        struct snd_interval *rate = hw_param_interval(params,
                                                      SNDRV_PCM_HW_PARAM_RATE);
        snd_interval_refine(rate, &rate_interval);
    }
    return 0;
}

static int hw_rule_buffer_restricts_rate(struct snd_pcm_hw_params *params,
                                         struct snd_pcm_hw_rule* rule)
{
    struct snd_interval *buffer_bytes =
            hw_param_interval(params, SNDRV_PCM_HW_PARAM_BUFFER_BYTES);
    // Decimation factor 96 or 128 not possible.
    if (buffer_bytes->min * 3 > PDM_MAX_BUFFER_SIZE) {
        const struct snd_interval rate_interval = {
                .min = 44100,
                .max = 96000
        };
        struct snd_interval *rate = hw_param_interval(params,
                                                      SNDRV_PCM_HW_PARAM_RATE);
        snd_interval_refine(rate, &rate_interval);
    }
    return 0;
}
static int hw_rule_sample_rate_restricts_period_and_buffer(
        struct snd_pcm_hw_params *params, struct snd_pcm_hw_rule* rule)
{
    struct snd_interval *rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
    const int decimation_factor = cic_decimation_factor_from_rate(rate->max);
    // PDM DMA buffer size and PDM buffer size are hardware limitations.
    const struct snd_interval period_byte_interval = {
            .min = 0,
            .max = PDM_MAX_DMA_BUFFER_SIZE / (decimation_factor / PCM_SAMPLE_SIZE_BITS)
    };
    const struct snd_interval buffer_byte_interval = {
            .min = 0,
            .max = PDM_MAX_BUFFER_SIZE / (decimation_factor / PCM_SAMPLE_SIZE_BITS)
    };
    struct snd_interval *period_bytes =
            hw_param_interval(params, SNDRV_PCM_HW_PARAM_PERIOD_BYTES);
    struct snd_interval *buffer_bytes =
            hw_param_interval(params, SNDRV_PCM_HW_PARAM_BUFFER_BYTES);

    snd_interval_refine(period_bytes, &period_byte_interval);
    snd_interval_refine(buffer_bytes, &buffer_byte_interval);
    return 0;
}

/* This sets the clock divider. Target division rate for the rate
   |sample_rate| should be:
   avpll_frequency / (sample_rate * n_channels * sample_bits * (decimation_factor / 64)).
   For 48kHz, this is 24576000/(48000*2*32*1) = 8. */
static void berlin_set_aio(unsigned int sample_rate)
{
        unsigned int div;

        // update berlin_capture_rates if any rates are added or removed.
        switch (sample_rate) {
        case 8000  :
        case 11025 :
        case 12000 :
                div = AIO_DIV16;  // normally AIO_DIV32 but we doubled the decimation factor
                break;
        case 16000 :
#if defined(CONFIG_SND_SOC_BERLIN_SLOWER_16KHZ_PDM_CLOCK)
                div = AIO_DIV16;
#else
                div = AIO_DIV8;  // normally AIO_DIV16 but we doubled the decimation factor
#endif
                break;
        case 22050 :
        case 24000 :
                div = AIO_DIV8;  // normally AIO_DIV16 but we doubled the decimation factor
                break;
        case 32000 :
        case 44100 :
        case 48000 :
                div = AIO_DIV8;
                break;
        case 64000 :
        case 88200 :
        case 96000 :
                div = AIO_DIV4;
                break;
        default:
                break;
        }

        PdmSetup(CAP_ID, HD_CLK, div, 0, 4, 0, AUDCH_CTRL_MUTE_MUTE_ON);   // clk divider is div

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
        case 64000 :
                apll = 16384000;
                break;
        case 12000 :
        case 24000 :
        case 48000 :
        case 96000 :
                apll = 24576000;
                break;
        case 16000 :
        case 32000 :
#if defined(CONFIG_SND_SOC_BERLIN_SLOWER_16KHZ_PDM_CLOCK)
                apll = 24576000;
#else
                apll = 16384000;
#endif
                break;
        default :
                apll = 24576000;
                break;
        }
        // WARNING: AVPLL channel 3 is reserved for I2S playback. DO NOT
        // set parameters for it here
        AVPLL_Set(0, 4, apll);
        Avio_SetHdclk();
}

/* must always be called under lock. */
static void start_dma_if_needed(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_capture *berlin_capture = runtime->private_data;
        const unsigned int chanId = berlin_capture->chan_id;
        dma_addr_t dma_source_address;
        int dma_size;

        assert_spin_locked(&berlin_capture->lock);

        if (!berlin_capture->capturing) {
                snd_printd("%s: capturing: %u\n", __func__, berlin_capture->capturing);
                return;
        }

        if (berlin_capture->dma_pending) {
                snd_printd("%s: DMA pending. Skipping DMA start...\n", __func__);
                return;
        }

        if (berlin_capture->cnt > (berlin_capture->pdm_dma_bytes - berlin_capture->pdm_dma_period_size)) {
                berlin_report_xrun(PCM_OVERRUN);
                snd_printk("%s: overrun: PCM conversion too slow.\n", __func__);
                return;
        }
#if 0
        dma_source_address = runtime->dma_addr +
                             berlin_capture->current_dma_offset;
#else
        dma_source_address = berlin_capture->pdm_dma_addr +
                                berlin_capture->current_dma_offset;
#endif

        dma_size = berlin_capture->pdm_dma_period_size;

        berlin_capture->dma_pending = true;
        dhub_channel_write_cmd(&AG_dhubHandle.dhub, chanId,
                dma_source_address, dma_size, 0, 0, 0, 1, 0, 0);
}

static void snd_berlin_capture_trigger_start(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_capture *berlin_capture = runtime->private_data;
        const unsigned int chanId = berlin_capture->chan_id;
        unsigned long flags;
        dma_addr_t dma_source_address;
        int dma_size, i;

        spin_lock_irqsave(&berlin_capture->lock, flags);
        if (berlin_capture->capturing) {
                spin_unlock_irqrestore(&berlin_capture->lock, flags);
                return;
        }

        berlin_capture->capturing = true;
        berlin_capture->need_audio_unmute = true;

        dma_size = berlin_capture->pdm_dma_period_size;
        // Queue two DMA commands when trigger start to avoid FIFO overrun
        /* In each interrupt, write one DMA command, so if interrupts are
         * handled in time, there will always be 2 DMA commands in queue.
         * The first interrupt will be raised and stay asserted until cleared.
         * If the first interrupt isn’t cleared before the second interrupt is
         * raised, the interrupt line stays asserted.
         * The overrun happens when IRQ is disabled for longer than 8 ms, such
         * that IRQ is not served and the next DMA is not set up in time before
         * FIFO being filled up.
         * The calculation is based on FIFO size = 2048 bytes, 16khz sampling
         * rate, stereo and 64 bits per sample for PDM data.
         * PDM_MAX_DMA_BUFFER_SIZE / (DECIMATION_FACTOR / 32) /
         * (2 channels * 4 bytes / sample) / (16000 frames / second)
         * 4096 / (128 / 32) / 8 / 16000 = 0.008 sec
         * Queueing 2 DMA commands will relax the limit to ~16ms
         */
        // We intentionally let the dma_source_address in dhub_channel_write_cmd
        // be the same in the for-loop, because the data in the first few DMA
        // blocks may be corrupted and we just ignore them.
        for (i = 0; i < 2; i++) {
                dma_source_address = berlin_capture->pdm_dma_addr +
                                     berlin_capture->current_dma_offset;
                dhub_channel_write_cmd(&AG_dhubHandle.dhub, chanId,
                                       dma_source_address, dma_size, 0, 0, 0, 1,
                                       0, 0);
        }
        berlin_capture->dma_pending = true;

        spin_unlock_irqrestore(&berlin_capture->lock, flags);
        snd_printd("%s: finished.\n", __func__);
}

static void snd_berlin_capture_trigger_stop(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_capture *berlin_capture = runtime->private_data;
        unsigned long flags;
        spin_lock_irqsave(&berlin_capture->lock, flags);
        PdmMuteEn(berlin_capture->chan_id, AUDCH_CTRL_MUTE_MUTE_ON);
        berlin_capture->capturing = false;
        berlin_capture->dma_pending = false;
        spin_unlock_irqrestore(&berlin_capture->lock, flags);
        snd_printd("%s: finished.\n", __func__);
}

static const struct snd_pcm_hardware snd_berlin_capture_hw = {
        .info = (SNDRV_PCM_INFO_MMAP
                | SNDRV_PCM_INFO_INTERLEAVED
                | SNDRV_PCM_INFO_MMAP_VALID
                | SNDRV_PCM_INFO_PAUSE
                | SNDRV_PCM_INFO_RESUME),
        .formats = SNDRV_PCM_FMTBIT_S32_LE,
        .rates = (SNDRV_PCM_RATE_8000_96000
                | SNDRV_PCM_RATE_KNOT),
        .rate_min = 8000,
        .rate_max = 96000,
        .channels_min = MAX_CHANNELS,
        .channels_max = MAX_CHANNELS,
        .buffer_bytes_max = PCM_MAX_BUFFER_SIZE,
        .period_bytes_min = PCM_MIN_DMA_BUFFER_SIZE,
        .period_bytes_max = PCM_MAX_DMA_BUFFER_SIZE,
        .periods_min = 2,
        .periods_max = PCM_MAX_BUFFER_SIZE / PCM_MIN_DMA_BUFFER_SIZE,
        .fifo_size = 0
};

static void snd_berlin_capture_runtime_free(struct snd_pcm_runtime *runtime)
{
        struct snd_berlin_card_capture *berlin_capture = runtime->private_data;

        if (berlin_capture) {
                if (berlin_capture->wq) {
                        destroy_workqueue(berlin_capture->wq);
                        berlin_capture->wq = NULL;
                }
                kfree(berlin_capture);
                runtime->private_data = 0;
        }
}

static int32_t cic_decimator_process(struct cic_decimator *cic, uint32_t *data) {
  return cic->process(cic, data);
}

static int32_t cic_decimator_process_o4_128x(struct cic_decimator *cic, uint32_t *data) {
        const int decimation_factor = 128;
        const int order = 4;
        int block, i, stage;
        int32_t acc, old_acc;

        for (block = 0; block < decimation_factor / PCM_SAMPLE_SIZE_BITS; ++block) {
                uint32_t bits = data[MAX_CHANNELS * block];
                for (i = 0; i < PCM_SAMPLE_SIZE_BITS; ++i) {
                      acc = (bits & 1) ? 1 : -1;
                      for (stage = 0; stage < order; ++stage) {
                              acc = (cic->integrator_stage[stage] += acc);
                      }
                      bits >>= 1;
                }
        }

        acc = cic->integrator_stage[order - 1];
        for (stage = 0; stage < cic->order; ++stage) {
                old_acc = acc;
                acc -= cic->comb_stage[stage];
                cic->comb_stage[stage] = old_acc;
        }

        /*
         * Normalize output so that a PDM bitstream of all 1s gives
         * PCM data that eventually settles at INT32_MAX, and a PDM bitstream of
         * all 0s gives PCM data that eventually settles at INT32_MIN.
         */
        return (acc >= (1 << 28)) ? 0x7fffffff : (acc * 8);
}

static int32_t cic_decimator_process_o5_64x(struct cic_decimator *cic, uint32_t *data) {
        const int decimation_factor = 64;
        const int order = 5;
        int block, i, stage;
        int32_t acc, old_acc;

        for (block = 0; block < decimation_factor / PCM_SAMPLE_SIZE_BITS; ++block) {
                uint32_t bits = data[MAX_CHANNELS * block];
                for (i = 0; i < PCM_SAMPLE_SIZE_BITS; ++i) {
                      acc = (bits & 1) ? 1 : -1;
                      for (stage = 0; stage < order; ++stage) {
                              acc = (cic->integrator_stage[stage] += acc);
                      }
                      bits >>= 1;
                }
        }

        acc = cic->integrator_stage[order - 1];
        for (stage = 0; stage < cic->order; ++stage) {
                old_acc = acc;
                acc -= cic->comb_stage[stage];
                cic->comb_stage[stage] = old_acc;
        }

        /*
         * Normalize output so that a PDM bitstream of all 1s gives
         * PCM data that eventually settles at INT32_MAX, and a PDM bitstream of
         * all 0s gives PCM data that eventually settles at INT32_MIN.
         */
        return (acc >= (1 << 30)) ? 0x7fffffff : (acc * 2);
}

static int32_t cic_decimator_process_o4_96x(struct cic_decimator *cic, uint32_t *data) {
        const int decimation_factor = 96;
        const int order = 4;
        int block, i, stage;
        int32_t acc, old_acc;

        for (block = 0; block < decimation_factor / PCM_SAMPLE_SIZE_BITS; ++block) {
                uint32_t bits = data[MAX_CHANNELS * block];
                for (i = 0; i < PCM_SAMPLE_SIZE_BITS; ++i) {
                      acc = (bits & 1) ? 1 : -1;
                      for (stage = 0; stage < order; ++stage) {
                              acc = (cic->integrator_stage[stage] += acc);
                      }
                      bits >>= 1;
                }
        }

        acc = cic->integrator_stage[order - 1];
        for (stage = 0; stage < cic->order; ++stage) {
                old_acc = acc;
                acc -= cic->comb_stage[stage];
                cic->comb_stage[stage] = old_acc;
        }

        /*
         * Normalize output so that a PDM bitstream of all 1s gives
         * PCM data that eventually settles at INT32_MAX, and a PDM bitstream of
         * all 0s gives PCM data that eventually settles at INT32_MIN.
         */
        return (acc >= (1 << 27)) ? 0x7fffffff : (acc * 16);
}

static int32_t cic_decimator_process_general(struct cic_decimator *cic, uint32_t *data) {
        int block, i, stage;
        int32_t acc, old_acc;

        // decimation_factor must be a multiple of 32.
        for (block = 0; block < cic->decimation_factor / PCM_SAMPLE_SIZE_BITS; ++block) {
                uint32_t bits = data[MAX_CHANNELS * block];
                for (i = 0; i < PCM_SAMPLE_SIZE_BITS; ++i) {
                      acc = (bits & 1) ? 1 : -1;
                      for (stage = 0; stage < cic->order; ++stage) {
                              acc = (cic->integrator_stage[stage] += acc);
                      }
                      bits >>= 1;
                }
        }

        acc = cic->integrator_stage[cic->order - 1];
        for (stage = 0; stage < cic->order; ++stage) {
                old_acc = acc;
                acc -= cic->comb_stage[stage];
                cic->comb_stage[stage] = old_acc;
        }

        /*
         * Normalize output so that a PDM bitstream of all 1s gives
         * PCM data that eventually settles at INT32_MAX, and a PDM bitstream of
         * all 0s gives PCM data that eventually settles at INT32_MIN.
         */
        return (acc >= (1 << (cic->bmax - 1))) ? 0x7fffffff : (acc * (2 << (31 - cic->bmax)));
}

static void cic_decimator_init(struct cic_decimator *cic, int decimation_factor, int order) {
        memset(cic, 0, sizeof(*cic));
        cic->decimation_factor = decimation_factor;
        cic->order = order;
        /*
         * CIC_BMAX = ceil(CIC_ORDER * log2(CIC_DECIMATION_FACTOR * M) + BITWIDTH_IN)
         * with M = 1 and BITWIDTH_IN = 1.
         */
        if (cic->order == 4 && cic->decimation_factor == 96) {
                cic->bmax = 28;
        } else {
                cic->bmax = order * ilog2(decimation_factor) + 1;
                if (!is_power_of_2(decimation_factor)) {
                        snd_printk("%s: Warning! CIC_BMAX calculation may be off.\n", __func__);
                }
        }

        if (cic->order == 4 && cic->decimation_factor == 128) {
                cic->process = &cic_decimator_process_o4_128x;
        } else if (cic->order == 5 && cic->decimation_factor == 64) {
                cic->process = &cic_decimator_process_o5_64x;
        } else if (cic->order == 4 && cic->decimation_factor == 96) {
                cic->process = &cic_decimator_process_o4_96x;
        } else {
                cic->process = &cic_decimator_process_general;
        }
        snd_printk("%s: order %d, %dx, bmax: %d\n", __func__, cic->order, cic->decimation_factor, cic->bmax);
}

static int32_t map_float_to_int32(float value) {
        return value * (1 << 30);
}

static float map_int32_to_float(int32_t value) {
        return value * (1.0f / (1 << 30));
}

static void dc_canceller_init(struct dc_canceller *dcc) {
         memset(dcc, 0, sizeof(*dcc));
}

/*
 * This is a passive first-order high pass filter as seen at
 * https://en.wikipedia.org/wiki/High-pass_filter#Algorithmic_implementation
 * If f_c = 10 Hz is the frequency cutoff and dt is 1/sample_rate,
 * then alpha = RC / (RC + dt)
 *            = 1 / (2*pi*f_c * (1/(2*pi*f_c) + dt))
 *            = 1 / (1 + dt * 2*pi*f_c)
 *            = 0.998 (48 kHz capture) or 0.999 (44.1 kHz capture)
 */
static float dc_canceller_process(struct dc_canceller *dcc, float input) {
  float output = dcc->alpha * (input - dcc->last_input + dcc->last_output);
  dcc->last_input = input;
  dcc->last_output = output;
  return output;
}

/*
 * This is used for checking if PDM stream data is zeros
 */
static const char zero_block [PDM_MAX_DMA_BUFFER_SIZE] = {0};

void check_mic_mute_state(uint32_t* base, size_t length_per_chunk,
                           unsigned int sample_rate, struct snd_pcm_substream *substream) {
         struct snd_berlin_chip *chip = substream->pcm->card->private_data;
         struct input_dev *mic_mute_state_input = chip->mic_mute_state_input;
         const size_t byte_per_chunk = 4*length_per_chunk;
         /* # of chunk = # of millisecond * (1second / 1000 millisecond) * (sample/second) * (byte/sample) * (chunks/byte)
                       = # of millisecond / 1000 * sample_rate(Hz) * (2 channels * 4 bytes/channel-sample) / (byte/chunk)
                       = (# of millisecond * sample_rate(Hz) * 8) / (1000 * byte_per_chunk) */
         const size_t mute_threshold_in_chunk = DIV_ROUND_UP(MIC_MUTE_THRESHOLD_IN_MS * sample_rate * 8,
                                                             1000 * byte_per_chunk);
         bool is_all_zeros = !memcmp(base, zero_block, byte_per_chunk);
         if (is_all_zeros && !chip->is_mic_mute) {
                  if (++chip->zero_chunk_count > mute_threshold_in_chunk) {
                           chip->is_mic_mute = 1;
                           input_event(mic_mute_state_input, EV_KEY, KEY_MICMUTE, 1);
                           input_sync(mic_mute_state_input);
                  }
                  return;
         }
         if (!is_all_zeros) {
                  chip->zero_chunk_count = 0;
                  if (chip->is_mic_mute) {
                           chip->is_mic_mute = 0;
                           input_event(mic_mute_state_input, EV_KEY, KEY_MICMUTE, 0);
                           input_sync(mic_mute_state_input);
                  }
         }
}

static void decode_pdm_to_pcm(struct work_struct *work)
{
        struct snd_berlin_card_capture *berlin_capture = container_of(work,
                    struct snd_berlin_card_capture, delayed_work.work);
        struct snd_pcm_substream *substream = berlin_capture->substream;
        struct snd_berlin_chip *chip = substream->pcm->card->private_data;
        struct snd_pcm_runtime *runtime = substream->runtime;
        uint32_t *src = (uint32_t*) (berlin_capture->pdm_dma_area + berlin_capture->read_offset);
        int32_t *dst = (int32_t*) (runtime->dma_area + berlin_capture->runtime_offset);
        unsigned long flags;
        const size_t period_size_bytes = frames_to_bytes(runtime, runtime->period_size);
        const size_t pcm_buffer_size_bytes = frames_to_bytes(runtime, runtime->buffer_size);
        const size_t pdm_samples_per_pcm_sample =
                        berlin_capture->cic[0].decimation_factor / PCM_SAMPLE_SIZE_BITS;
        const size_t pdm_samples_per_pcm_frame =
                        pdm_samples_per_pcm_sample * berlin_capture->channel_num;
        const size_t pdm_samples_per_pdm_dma_buffer =
                        bytes_to_samples(runtime, berlin_capture->pdm_dma_period_size);

        while (berlin_capture->capturing) {
                size_t i = 0, ch = 0;
                spin_lock_irqsave(&berlin_capture->lock, flags);
                if (berlin_capture->cnt < berlin_capture->pdm_dma_period_size) {
                        start_dma_if_needed(substream);
                        spin_unlock_irqrestore(&berlin_capture->lock, flags);
                        return ;
                }
                spin_unlock_irqrestore(&berlin_capture->lock, flags);
                /* check pdm_data to infer mic mute */
                check_mic_mute_state(src, pdm_samples_per_pdm_dma_buffer,
                                        berlin_capture->sample_rate, substream);
                if (chip->is_mic_mute) {
                        memset(dst, 0, period_size_bytes);
                } else {
                        for (i = 0; i < pdm_samples_per_pdm_dma_buffer;
                                        i += pdm_samples_per_pcm_frame) {
                                for (ch=0; ch < berlin_capture->channel_num; ch++) {
                                        uint32_t *pdm_data = src + i + ch;
                                        int32_t pcm_data;
                                        float temp;
                                        pcm_data = cic_decimator_process(
                                                &berlin_capture->cic[ch], pdm_data);
                                        temp = map_int32_to_float(pcm_data);
                                        temp = dc_canceller_process(&berlin_capture->dcc[ch], temp);
                                        pcm_data = map_float_to_int32(temp);
                                        *dst++ = pcm_data;
                                }
                        }
                }

                berlin_capture->read_offset += berlin_capture->pdm_dma_period_size;
                berlin_capture->read_offset %= berlin_capture->pdm_dma_bytes;
                spin_lock_irqsave(&berlin_capture->lock, flags);
                /* compress two $DECIMATION_FACTOR 1-bit PDM samples to two
                 * 32-bit PCM samples. */
                berlin_capture->runtime_offset += period_size_bytes;
                berlin_capture->runtime_offset %= pcm_buffer_size_bytes;
                berlin_capture->cnt -= berlin_capture->pdm_dma_period_size;
                spin_unlock_irqrestore(&berlin_capture->lock, flags);

                snd_pcm_period_elapsed(substream);
                src = (uint32_t*) (berlin_capture->pdm_dma_area + berlin_capture->read_offset);
                dst = (int32_t*) (runtime->dma_area + berlin_capture->runtime_offset);
        }
}

int snd_berlin_capture_open(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_capture *berlin_capture;
        size_t ch;

        int err = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
                        &berlin_constraints_rates);
        if (err < 0)
                return err;

        // Ensure PDM buffer sizes do not exceed hardware limits.
        err = snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
                                  &hw_rule_period_restricts_rate, NULL,
                                  SNDRV_PCM_HW_PARAM_RATE);
        err = snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
                                  &hw_rule_buffer_restricts_rate, NULL,
                                  SNDRV_PCM_HW_PARAM_RATE);
        err = snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
                                  &hw_rule_sample_rate_restricts_period_and_buffer,
                                  NULL, SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
                                  SNDRV_PCM_HW_PARAM_BUFFER_BYTES, -1);
        berlin_capture = kzalloc(sizeof(struct snd_berlin_card_capture), GFP_KERNEL);
        if (berlin_capture == NULL)
                return -ENOMEM;

        berlin_capture->dma_pending = false;
        berlin_capture->capturing = false;
        berlin_capture->pages_allocated = false;
        berlin_capture->substream = substream;
        berlin_capture->chan_id = CAP_ID;
        for (ch = 0; ch < MAX_CHANNELS; ch++) {
                dc_canceller_init(&berlin_capture->dcc[ch]);
        }
        berlin_capture->wq = alloc_workqueue("berlin_capture", WQ_HIGHPRI | WQ_UNBOUND, 1);
        if (berlin_capture->wq == NULL)
                return -ENOMEM;
        INIT_DELAYED_WORK(&berlin_capture->delayed_work, decode_pdm_to_pcm);

        spin_lock_init(&berlin_capture->lock);

        runtime->private_data = berlin_capture;
        runtime->private_free = snd_berlin_capture_runtime_free;
        runtime->hw = snd_berlin_capture_hw;

        /* enable audio interrupt */
        DhubEnableIntr(0, &AG_dhubHandle, berlin_capture->chan_id, 1);

        snd_printk("%s: finished.\n", __func__);
        return 0;
}

int snd_berlin_capture_close(struct snd_pcm_substream *substream)
{
        /* disable audio interrupt */
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_capture *berlin_capture = runtime->private_data;
        DhubEnableIntr(0, &AG_dhubHandle, berlin_capture->chan_id, 0);
        snd_printk("%s: finished.\n", __func__);
        return 0;
}

int snd_berlin_capture_hw_free(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_capture *berlin_capture = runtime->private_data;
        snd_printk("%s\n", __func__);

        flush_delayed_work(&berlin_capture->delayed_work);
        if (berlin_capture->pdm_dma_area) {
               dma_free_coherent(NULL, berlin_capture->pdm_dma_bytes,
                                berlin_capture->pdm_dma_area,
                                berlin_capture->pdm_dma_addr);
                berlin_capture->pdm_dma_area = NULL;
                berlin_capture->pdm_dma_addr = 0;
         }

         if (berlin_capture->pages_allocated == true) {
                snd_pcm_lib_free_pages(substream);
                berlin_capture->pages_allocated = false;
        }
        return 0;
}

int snd_berlin_capture_hw_params(struct snd_pcm_substream *substream,
                                 struct snd_pcm_hw_params *params)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_capture *berlin_capture = runtime->private_data;
        int err;
        unsigned long flags;
        size_t ch;
        unsigned long long microphone_sleep_time_us;
        const size_t pcm_buffer_size_bytes = params_buffer_bytes(params);
        const size_t pcm_period_size_bytes = pcm_buffer_size_bytes / params_periods(params);

        snd_berlin_capture_hw_free(substream);
        err = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
        if (err < 0) {
                snd_printk("pcm_lib_malloc failed to allocated pages for buffers\n");
                return err;
        }
        berlin_capture->pages_allocated = true;

        snd_printk("%s: sample_rate:%d channels:%d format:%s "
                   "period_size: %d bytes buffer_size: %d bytes\n", __func__,
                   params_rate(params), params_channels(params),
                   snd_pcm_format_name(params_format(params)),
                   params_period_bytes(params),
                   params_buffer_bytes(params));

        berlin_capture->sample_rate = params_rate(params);
        berlin_capture->sample_format = params_format(params);
        berlin_capture->channel_num = params_channels(params);

        for (ch = 0; ch < berlin_capture->channel_num; ch++) {
                int cic_order = cic_order_from_rate(berlin_capture->sample_rate);
                int decimation_factor = cic_decimation_factor_from_rate(berlin_capture->sample_rate);
                if (cic_order == 0 || decimation_factor == 0) {
                        snd_printk("%s: Cannot determine CIC order or decimation factor from sample rate %d\n",
                                   __func__, berlin_capture->sample_rate);
                        snd_pcm_lib_free_pages(substream);
                        return -EINVAL;
                }
                cic_decimator_init(&berlin_capture->cic[ch],
                                   decimation_factor, cic_order);
                berlin_capture->dcc[ch].alpha =
                        1.0f / (1 + 2 * 3.14159f * DC_HPF_CUTOFF_HZ /
                                    berlin_capture->sample_rate);
        }

        berlin_capture->pdm_dma_bytes =
                        pcm_buffer_size_bytes *
                        (berlin_capture->cic[0].decimation_factor / PCM_SAMPLE_SIZE_BITS);
        berlin_capture->pdm_dma_period_size =
                        pcm_period_size_bytes *
                        (berlin_capture->cic[0].decimation_factor / PCM_SAMPLE_SIZE_BITS);
        if (berlin_capture->pdm_dma_bytes > PDM_MAX_BUFFER_SIZE) {
          snd_printk("%s: PDM buffer size too large: %d\n", __func__, berlin_capture->pdm_dma_bytes);
          snd_pcm_lib_free_pages(substream);
          return -EINVAL;
        }
        berlin_capture->pdm_dma_area = dma_zalloc_coherent(
                        NULL, berlin_capture->pdm_dma_bytes,
                        &berlin_capture->pdm_dma_addr, GFP_KERNEL);
        if (!berlin_capture->pdm_dma_area) {
                snd_printk("%s: failed to allocate PDM DMA area\n", __func__);
                goto err_pdm_dma;
        }

        /* AVPLL configuration */
        berlin_set_pll(berlin_capture->sample_rate);
        spin_lock_irqsave(&berlin_capture->lock, flags);
        /* AIO configuration */
        berlin_set_aio(berlin_capture->sample_rate);

        PdmMuteEn(berlin_capture->chan_id, AUDCH_CTRL_MUTE_MUTE_ON);
        PdmRxStart(berlin_capture->chan_id, 0);
        spin_unlock_irqrestore(&berlin_capture->lock, flags);

        /* Microphone takes 32768 cycles to wake up. Sleep for
         * ceil(32768 cycles * (1/sample_rate seconds per cycle) * 10^6 us/sec) us
         * This is not in trigger_start() because sleeps should not be happening
         * in the atomic trigger callback.
         * http://www.alsa-project.org/~tiwai/writing-an-alsa-driver/ch05s06.html#pcm-interface-operators-trigger-callback
         * */
        microphone_sleep_time_us = DIV_ROUND_UP_ULL(32768ull * 1000 * 1000,
                                                    berlin_capture->sample_rate * berlin_capture->cic[0].decimation_factor);
        usleep_range(microphone_sleep_time_us,
                     microphone_sleep_time_us + 100);
        return 0;

err_pdm_dma:
        snd_pcm_lib_free_pages(substream);
        return -ENOMEM;
}

int snd_berlin_capture_prepare(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_capture *berlin_capture = runtime->private_data;
        unsigned long flags;

        spin_lock_irqsave(&berlin_capture->lock, flags);
        berlin_capture->current_dma_offset = 0;
        berlin_capture->read_offset = 0;
        berlin_capture->cnt = 0;
        berlin_capture->runtime_offset = 0;
        spin_unlock_irqrestore(&berlin_capture->lock, flags);
        return 0;
}

int snd_berlin_capture_trigger(struct snd_pcm_substream *substream, int cmd)
{
        int  ret = 0;
        switch (cmd) {
        case SNDRV_PCM_TRIGGER_START:
        case SNDRV_PCM_TRIGGER_RESUME:
        case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
                snd_berlin_capture_trigger_start(substream);
                break;
        case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_SUSPEND:
        case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
                snd_berlin_capture_trigger_stop(substream);
                break;
        default:
                ret = -EINVAL;
        }
        return ret;
}

snd_pcm_uframes_t
snd_berlin_capture_pointer(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_capture *berlin_capture = runtime->private_data;
        uint32_t buf_pos;
        unsigned long flags;

        spin_lock_irqsave(&berlin_capture->lock, flags);
        buf_pos = berlin_capture->runtime_offset;
        spin_unlock_irqrestore(&berlin_capture->lock, flags);

        return bytes_to_frames(runtime, buf_pos);
}

int snd_berlin_capture_isr(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_berlin_card_capture *berlin_capture = runtime->private_data;
        unsigned long flags;
        // FIFO fullness
        unsigned int dhub_cnt;
        UNSG32 addr;

        spin_lock_irqsave(&berlin_capture->lock, flags);
        /* If we are not running, do not chain, and clear pending */
        if (!berlin_capture->capturing) {
                berlin_capture->dma_pending = false;
                spin_unlock_irqrestore(&berlin_capture->lock, flags);
                return IRQ_HANDLED;
        }

        /* If we were not pending, avoid pointer manipulation */
        if (!berlin_capture->dma_pending) {
                spin_unlock_irqrestore(&berlin_capture->lock, flags);
                return IRQ_HANDLED;
        }

        if (berlin_capture->need_audio_unmute) {
             berlin_capture->need_audio_unmute = false;
             PdmMuteEn(berlin_capture->chan_id, AUDCH_CTRL_MUTE_MUTE_OFF);
             berlin_capture->current_dma_offset += berlin_capture->pdm_dma_period_size;
             berlin_capture->current_dma_offset %= berlin_capture->pdm_dma_bytes;
             berlin_capture->dma_pending = false;
             start_dma_if_needed(substream);
             spin_unlock_irqrestore(&berlin_capture->lock, flags);
             return 0;
        }

        /* Roll the DMA pointer, and chain if needed */
        berlin_capture->current_dma_offset += berlin_capture->pdm_dma_period_size;
        berlin_capture->current_dma_offset %= berlin_capture->pdm_dma_bytes;
        berlin_capture->dma_pending = false;
        berlin_capture->cnt += berlin_capture->pdm_dma_period_size;

        /* to query the consumer data count register to check fifo fullness */
        addr = AG_DHUB_BASE + RA_dHubReg_HBO + 0x80 +
               berlin_capture->chan_id * 4 * 2 + 4;
        // This number is determined by the HW. We don't reset it
        dhub_cnt = (*((volatile unsigned int*)(addr)));
        dhub_cnt &= 0xffff;
        /* dhub count is in 64 bit unit, this value is 240*8 = 1920 bytes */
        /* the typical value is: 0  */
        /* when dhub fifo is full, the count is 256 unit, which is: 256*8 = 2048 bytes*/
        if (dhub_cnt >= 240) {
                berlin_report_xrun(FIFO_OVERRUN);
                snd_printk("[FIFO OVERRUN] Dhub cnt: %d bytes, FIFO almost full \n",
                           dhub_cnt * 8);
        }

        start_dma_if_needed(substream);
        spin_unlock_irqrestore(&berlin_capture->lock, flags);
        /* convert PDM signal to PCM data */
        queue_delayed_work(berlin_capture->wq, &berlin_capture->delayed_work, 0);

        return 0;
}
